#!/usr/bin/env python3
"""Verify junction connector geometry in an osm2xodr-generated .xodr file.

For every <connection><laneLink> in every <junction>, this recomputes the actual
drivable-lane centerline (position and direction-of-travel heading) on both sides
of the link -- the connector road's own lane, and the real incoming/outgoing road's
referenced lane -- directly from the XML, independent of how main.cpp computed them.
It then checks that position and heading match exactly at both the connector's
predecessor (s=0) and successor (s=length) ends.

This is a standalone sanity check, not a unit test harness: it parses OpenDRIVE with
regular expressions rather than a real XML/OpenDRIVE parser, and only understands the
subset of OpenDRIVE that osm2xodr itself emits (line/arc planView geometry, at most one
laneOffset polynomial, per-lane width polynomials). It does not validate anything about
non-junction roads, signals, or XML schema conformance.

Usage:
    python3 check_junction_continuity.py <file.xodr> [--left-hand-traffic]

Pass --left-hand-traffic if the .xodr was generated with that osm2xodr flag; it flips
which lane-id sign corresponds to "backward" travel for *real* roads (see infer_lanes in
src/main.cpp). Synthetic junction connector roads are unaffected by that flag -- they
always use lane id -1 to mean their own forward (+s) direction.

Exit code is 0 if every checked lane connection matches within 1mm / 0.001rad, 1 otherwise.
"""
import math
import re
import sys


def parse_xodr(data):
    roads = {}
    for m in re.finditer(r'<road\b([^>]*)>(.*?)</road>', data, re.S):
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
        rid = attrs['id']
        body = m.group(2)

        geoms = []
        for g in re.finditer(r'<geometry\b([^>]*)>(.*?)</geometry>', body, re.S):
            gattrs = dict(re.findall(r'(\w+)="([^"]*)"', g.group(1)))
            curv = 0.0
            arc_m = re.search(r'<arc\s+curvature="([^"]*)"', g.group(2))
            if arc_m:
                curv = float(arc_m.group(1))
            # A fitted curve (build::fit_curve) is written as paramPoly3 with pRange="normalized":
            # the polynomial parameter p runs linearly over [0,1] across the geometry's own
            # (chord) length, not true curve arc length -- see model_builder.cpp's fit_curve for
            # why that's a safe, spec-compliant simplification. Store the raw coefficients rather
            # than pre-evaluating anything, so eval_geometry_point can query any s within [0,length].
            poly = None
            poly_m = re.search(r'<paramPoly3\s+([^/]*)/>', g.group(2))
            if poly_m:
                pa = dict(re.findall(r'(\w+)="([^"]*)"', poly_m.group(1)))
                poly = tuple(float(pa[k]) for k in ('aU', 'bU', 'cU', 'dU', 'aV', 'bV', 'cV', 'dV'))
            geoms.append({
                'x': float(gattrs['x']), 'y': float(gattrs['y']),
                'hdg': float(gattrs['hdg']), 'length': float(gattrs['length']), 'curv': curv, 'poly': poly,
            })

        def parse_link(mm):
            if not mm:
                return None
            return dict(re.findall(r'(\w+)="([^"]*)"', mm.group(1)))

        pred = parse_link(re.search(r'<predecessor\s+([^/]*)/>', body))
        succ = parse_link(re.search(r'<successor\s+([^/]*)/>', body))

        # A merged road can have multiple <laneOffset> and <laneSection> elements at different s
        # (one per internal cross-section boundary); keep them as ordered lists rather than
        # collapsing to a single value, and pick the applicable one at query time by s.
        lane_offsets = []
        for lo_m in re.finditer(r'<laneOffset\s+([^/]*)/>', body):
            lo = dict(re.findall(r'(\w+)="([^"]*)"', lo_m.group(1)))
            lane_offsets.append((float(lo['s']), float(lo['a']), float(lo['b'])))
        if not lane_offsets:
            lane_offsets = [(0.0, 0.0, 0.0)]
        lane_offsets.sort()

        lane_sections = []
        for ls_m in re.finditer(r'<laneSection\s+([^>]*)>(.*?)</laneSection>', body, re.S):
            ls_attrs = dict(re.findall(r'(\w+)="([^"]*)"', ls_m.group(1)))
            lanes = {}
            for side_m in re.finditer(r'<(left|right)>(.*?)</\1>', ls_m.group(2), re.S):
                side = side_m.group(1)
                # Widths of lanes already placed on this side (inner -> outer), kept as (a, b) pairs
                # rather than pre-summed: a lane's cumulative inner offset must be evaluated at the
                # *query* s (each earlier lane's own width(s) = a + b*ds), not frozen at s=0 -- a
                # ramping lane (lane split/merge taper, or this script's own reconciliation bridge)
                # is not always the outermost one, so an s=0-only sum silently drifts away from the
                # true cumulative width everywhere except exactly at the section's own s=0.
                inner = []
                for lane_m in re.finditer(r'<lane\s+id="(-?\d+)"[^>]*>(.*?)</lane>', side_m.group(2), re.S):
                    lid = int(lane_m.group(1))
                    width, width_b = 0.0, 0.0
                    width_m = re.search(r'<width\s+([^/]*)/>', lane_m.group(2))
                    if width_m:
                        wa = dict(re.findall(r'(\w+)="([^"]*)"', width_m.group(1)))
                        width, width_b = float(wa['a']), float(wa.get('b', 0.0))
                    lanes[lid] = {'width': width, 'width_b': width_b, 'side': side, 'inner_lanes': list(inner)}
                    inner.append((width, width_b))
            lane_sections.append((float(ls_attrs['s']), lanes))
        lane_sections.sort(key=lambda t: t[0])

        roads[rid] = {
            'attrs': attrs, 'geoms': geoms, 'pred': pred, 'succ': succ, 'body': body,
            'lane_offsets': lane_offsets, 'lane_sections': lane_sections, 'length': float(attrs['length']),
        }
    return roads


def applicable_at_s(entries, s):
    """Picks the last entry (by s_offset) whose s_offset <= s; entries is sorted ascending."""
    chosen = entries[0]
    for entry in entries:
        if entry[0] <= s + 1e-6:
            chosen = entry
        else:
            break
    return chosen


def eval_geometry_point(g, s):
    """Position/heading at arc-length `s` into a single line/arc/paramPoly3 planView primitive."""
    if g.get('poly') is not None:
        aU, bU, cU, dU, aV, bV, cV, dV = g['poly']
        p = s / g['length'] if g['length'] > 1e-9 else 0.0
        u = aU + bU * p + cU * p * p + dU * p * p * p
        v = aV + bV * p + cV * p * p + dV * p * p * p
        du_dp = bU + 2.0 * cU * p + 3.0 * dU * p * p
        dv_dp = bV + 2.0 * cV * p + 3.0 * dV * p * p
        cos_h, sin_h = math.cos(g['hdg']), math.sin(g['hdg'])
        x = g['x'] + u * cos_h - v * sin_h
        y = g['y'] + u * sin_h + v * cos_h
        hdg = g['hdg'] + math.atan2(dv_dp, du_dp)
        return (x, y, hdg)
    if abs(g['curv']) < 1e-9:
        return (g['x'] + math.cos(g['hdg']) * s, g['y'] + math.sin(g['hdg']) * s, g['hdg'])
    r = 1.0 / g['curv']
    theta = g['curv'] * s
    cx = g['x'] - r * math.sin(g['hdg'])
    cy = g['y'] + r * math.cos(g['hdg'])
    return (cx + r * math.sin(g['hdg'] + theta), cy - r * math.cos(g['hdg'] + theta), g['hdg'] + theta)


def road_point_at_s(road, s):
    acc = 0.0
    for g in road['geoms']:
        if s <= acc + g['length'] + 1e-6:
            return eval_geometry_point(g, s - acc)
        acc += g['length']
    g = road['geoms'][-1]
    return eval_geometry_point(g, g['length'])


def left_normal(hdg):
    return (-math.sin(hdg), math.cos(hdg))


def lane_centerline_at_s(roads, rid, lane_id, s, left_hand):
    """World position and *direction-of-travel* heading of a lane's centerline at arc-length s."""
    road = roads[rid]
    x, y, hdg = road_point_at_s(road, s)
    section_s, section_lanes = applicable_at_s(road['lane_sections'], s)
    lane = section_lanes[lane_id]
    lo_s, lo_a, lo_b = applicable_at_s(road['lane_offsets'], s)
    lane_offset = lo_a + lo_b * (s - lo_s)
    inner_acc = sum(w + wb * (s - section_s) for w, wb in lane['inner_lanes'])
    cum = inner_acc + (lane['width'] + lane['width_b'] * (s - section_s)) / 2.0
    sign = 1.0 if lane['side'] == 'left' else -1.0
    total_offset = lane_offset + sign * cum
    nx, ny = left_normal(hdg)

    is_connector = road['attrs'].get('junction', '-1') != '-1'
    if is_connector:
        # Synthetic connector roads always use lane id -1 to mean "the connector's own forward
        # (+s) direction", independent of the project's left/right-hand-traffic lane-id convention.
        backward = False
    else:
        # Project convention (infer_lanes in src/main.cpp): in right-hand-traffic mode, positive
        # lane ids (left side) carry -s (backward) traffic; left-hand-traffic mode flips this.
        backward = (lane_id > 0) if not left_hand else (lane_id < 0)
    travel_hdg = hdg + math.pi if backward else hdg
    return (x + nx * total_offset, y + ny * total_offset, travel_hdg)


def angle_diff(a, b):
    return abs(((a - b + math.pi) % (2 * math.pi)) - math.pi)


def check(path, left_hand):
    data = open(path, encoding='utf-8').read()
    roads = parse_xodr(data)

    total = 0
    bad = 0
    for junction_m in re.finditer(r'<junction\b[^>]*>(.*?)</junction>', data, re.S):
        for conn_m in re.finditer(r'<connection\b([^>]*)>(.*?)</connection>', junction_m.group(1), re.S):
            cattrs = dict(re.findall(r'(\w+)="([^"]*)"', conn_m.group(1)))
            connecting_road = cattrs['connectingRoad']
            cr = roads[connecting_road]
            if cr['attrs'].get('junction', '-1') == '-1':
                continue  # fallback direct connection (no synthetic connector); nothing to check

            for from_id, to_id in re.findall(r'<laneLink\s+from="(-?\d+)"\s+to="(-?\d+)"', conn_m.group(2)):
                from_id, to_id = int(from_id), int(to_id)
                total += 1

                pred = cr['pred']
                pred_s = 0.0 if pred.get('contactPoint', 'start') == 'start' else roads[pred['elementId']]['length']
                px, py, phdg = lane_centerline_at_s(roads, pred['elementId'], from_id, pred_s, left_hand)
                cx, cy, chdg = lane_centerline_at_s(roads, connecting_road, to_id, 0.0, left_hand)
                dpos1 = math.hypot(px - cx, py - cy)
                dhdg1 = angle_diff(phdg, chdg)

                lane_body = re.search(r'<lane\s+id="' + str(to_id) + r'"[^>]*>(.*?)</lane>', cr['body'], re.S)
                out_lane_id = int(re.search(r'<successor\s+id="(-?\d+)"', lane_body.group(1)).group(1))

                succ = cr['succ']
                succ_s = 0.0 if succ.get('contactPoint', 'start') == 'start' else roads[succ['elementId']]['length']
                ex, ey, ehdg = lane_centerline_at_s(roads, connecting_road, to_id, cr['length'], left_hand)
                tx, ty, thdg = lane_centerline_at_s(roads, succ['elementId'], out_lane_id, succ_s, left_hand)
                dpos2 = math.hypot(ex - tx, ey - ty)
                dhdg2 = angle_diff(ehdg, thdg)

                ok = dpos1 < 1e-3 and dhdg1 < 1e-3 and dpos2 < 1e-3 and dhdg2 < 1e-3
                if not ok:
                    bad += 1
                    print(f"  {connecting_road} (lane {from_id} -> lane {to_id}): "
                          f"start dpos={dpos1:.5f}m dhdg={math.degrees(dhdg1):.3f}deg  "
                          f"end dpos={dpos2:.5f}m dhdg={math.degrees(dhdg2):.3f}deg  [MISMATCH]")

    print(f"Checked {total} junction connector lane links, {bad} mismatched (> 1mm / 0.001rad).")
    return bad == 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    ok = check(sys.argv[1], left_hand='--left-hand-traffic' in sys.argv[2:])
    sys.exit(0 if ok else 1)
