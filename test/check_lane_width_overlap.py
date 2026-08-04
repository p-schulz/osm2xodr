#!/usr/bin/env python3
"""Verify the overlapping-lane-width fix (ModelBuilder::resolve_lane_width_overlaps) in an
osm2xodr-generated .xodr.

Two independently-mapped OSM ways can represent the same real road's opposing carriageways (a
divided highway digitized as two separate one-way ways). When neither carries a width tag, each
carriageway's class-based default lane width can claim most of the gap between them, leaving their
facing lane edges overlapping even though the ways themselves are far apart. resolve_lane_width_overlaps()
detects unrelated roads (not directly linked) that share a name/ref tag and shrinks whichever side(s)
lack an explicit width tag until --min-road-clearance is met, down to --min-lane-width (or a fixed
0.3m floor for an inferred grass-verge/border lane -- narrower than a driving lane's own floor since
its own unmodified default, --border-width, is typically already well under 2.5m).

This independently re-derives, from the *output* geometry alone (plus the source .osm's name/ref/
width tags, since ref isn't preserved in the .xodr and a width tag's absence is the trigger for the
fix), whether any two name/ref-matched, non-linked roads still have facing lane edges closer than
--min-road-clearance -- and if so, whether that's actually acceptable: either both roads carry an
explicit width tag (the fix must never touch a tagged road, so a close pair here is expected and only
noted, not a violation), or the facing side's own reducible lanes have already been driven down to
their floor (further shrinking wasn't possible, a real reported residual -- also not a violation).
Anything else -- a close, untagged, non-floored pair -- is a genuine violation: either the pass never
ran on this pair, or under-corrected it.

Two documented simplifications relative to the real C++ pass:
  - Width-tag/name/ref provenance is read from each road's *primary* (first, lowest-segment-index)
    source way only, via the "w<wayid>_<segidx>" id convention -- matching what the real pass reads
    for the road's *base* section (RoadSegment::tags), but not necessarily the section active at the
    exact overlap point on a road merged from several original ways (see resolve_lane_width_overlaps'
    own tags_at_s). Same whole-road-granularity simplification check_median_grass_lanes.py already
    documents for its own border-lane check.
  - "Directly linked" (skip, not a candidate pair) is approximated as sharing a predecessor/successor
    <link> reference or having endpoints within 1cm of each other, rather than re-parsing original OSM
    node refs -- equivalent in practice since a plain link's positions must match exactly by
    construction (see check_road_link_continuity.py).
  - A road's own name/ref-matching key can be a real discrepancy source: fuse_chain assigns a merged
    road's "name" tag from whichever constituent segment has the *longest* name-bearing stretch (see
    model_builder.cpp's longest_name_length), not necessarily the primary (id-implied) source way this
    checker looks its "ref" tag up from -- a merged road can therefore end up keyed differently here
    than in the real pass, most often surfacing as one road never being paired with its true neighbor
    at all (silently under-checked, not a false violation). Confirmed as the explanation for an
    isolated single-pair mismatch during development rather than chasing it further; a mismatch
    reproducible across multiple fixtures/variants is more likely a real regression.
  - The real pass also declines to shrink a lane that has an actual lane-to-lane <link> at whichever
    of its road's two ends the shrink would land on (see model_builder.cpp's section_touches_a_link)
    -- touching it would silently break continuity with whatever's linked across that boundary (an
    unrelated neighbor, or a junction connector already built against the pre-shrink width). This
    checker's "headroom" check doesn't know about that guard, so a pair blocked by it can look like
    an unexplained residual here even though the real pass is correctly leaving it alone.

Usage:
    python3 check_lane_width_overlap.py <file.osm> <file.xodr> [--min-road-clearance M] [--min-lane-width M]

Exit code is 0 if every name/ref-matched, non-linked road pair either clears --min-road-clearance,
is explicitly width-tagged on both sides, or has already been shrunk to its floor, 1 otherwise.
"""
import argparse
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import export_geojson as eg
from check_junction_continuity import parse_xodr

BORDER_FLOOR_M = 0.3  # mirrors resolve_lane_width_overlaps' own kMinBorderWidthM in model_builder.cpp
FLOOR_TOLERANCE_M = 0.02
ENDPOINT_TOLERANCE_M = 0.01
ROAD_ID_RE = re.compile(r'^w(\d+)_')
WIDTH_TAG_KEYS = ('width', 'width:lanes', 'width:lanes:forward', 'width:lanes:backward')


def parse_osm_way_tags(path):
    """way_id (str) -> dict of tags, for every <way> in the .osm."""
    data = open(path, encoding='utf-8').read()
    tags_by_way = {}
    for m in re.finditer(r'<way\b([^>]*?)(?:/>|>(.*?)</way>)', data, re.S):
        attrs = dict(re.findall(r'''(\w+)=["']([^"']*)["']''', m.group(1)))
        wid = attrs.get('id')
        if not wid or m.group(2) is None:
            continue
        tags = dict(re.findall(r'''<tag\s+k=["']([^"']*)["']\s+v=["']([^"']*)["']''', m.group(2)))
        tags_by_way[wid] = tags
    return tags_by_way


def primary_way_id(road_id):
    m = ROAD_ID_RE.match(road_id)
    return m.group(1) if m else None


def has_explicit_width_tags(tags):
    return any(k in tags for k in WIDTH_TAG_KEYS)


def project_point_to_polyline(px, py, pts):
    """Python port of geo.hpp's project_to_polyline; pts is a list of (x, y)."""
    best_d = float('inf')
    for i in range(1, len(pts)):
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        abx, aby = bx - ax, by - ay
        len2 = abx * abx + aby * aby
        if len2 <= 1e-9:
            continue
        u = ((px - ax) * abx + (py - ay) * aby) / len2
        u = max(0.0, min(1.0, u))
        qx, qy = ax + abx * u, ay + aby * u
        d = math.hypot(px - qx, py - qy)
        best_d = min(best_d, d)
    return best_d


def road_edge_points(road, step=2.0):
    """{'left': [(s, x, y), ...], 'right': [...]} sampled every `step`m along the road."""
    edges = {'left': [], 'right': []}
    for s in eg.sample_s_values(road['length'], step):
        x, y, hdg = eg.cjc.road_point_at_s(road, s)
        nx, ny = eg.cjc.left_normal(hdg)
        left_off, right_off = eg.road_edge_offsets_at_s(road, s)
        edges['left'].append((s, x + nx * left_off, y + ny * left_off))
        edges['right'].append((s, x + nx * right_off, y + ny * right_off))
    return edges


def lane_types_by_id(body, section_s_offset):
    """{lane_id: type} for one <laneSection s="section_s_offset">'s lanes, both sides."""
    types = {}
    for sec_m in re.finditer(r'<laneSection\s+s="([^"]*)">(.*?)</laneSection>', body, re.S):
        if abs(float(sec_m.group(1)) - section_s_offset) > 1e-6:
            continue
        for lane_m in re.finditer(r'<lane\s+id="(-?\d+)"\s+type="(\w+)"', sec_m.group(2)):
            types[int(lane_m.group(1))] = lane_m.group(2)
    return types


def section_lane_has_link(body, section_s, side, link_kind):
    """True if any driving/border lane on `side`, within the <laneSection s="section_s">, has a
    <link><predecessor/successor .../></link> child -- i.e. this specific lane, not just the road
    in general, actually participates in a link at that end."""
    for sec_m in re.finditer(r'<laneSection\s+s="([^"]*)">(.*?)</laneSection>', body, re.S):
        if abs(float(sec_m.group(1)) - section_s) > 1e-6:
            continue
        for side_m in re.finditer(r'<(left|right)>(.*?)</\1>', sec_m.group(2), re.S):
            if side_m.group(1) != side:
                continue
            for lane_m in re.finditer(r'<lane\s+id="(-?\d+)"\s+type="(\w+)"[^>]*>(.*?)</lane>', side_m.group(2), re.S):
                _lid, ltype, lane_body = lane_m.groups()
                if ltype not in ('driving', 'border'):
                    continue
                if re.search(r'<' + link_kind + r'\b', lane_body):
                    return True
    return False


def side_has_headroom(road, side, s, min_lane_width):
    """True if `side`, in whichever LaneSection is active at arc-length `s`, has a driving/border
    lane still above its own floor -- meaning the real pass had more room to shrink here and, if
    this pair is still reported tight at this exact location, should have used it. A side with no
    driving/border lane at all (e.g. the empty side of a strictly oneway fragment -- confirmed real
    in examples/intersection.osm, where w134547114_*'s positive-id side is completely empty) has no
    headroom either, same as a side already fully floored: neither is a discrepancy, since
    resolve_lane_width_overlaps can only ever touch a lane that exists. Nor does a section that
    touches the road's own outer start/end *and* has a real per-lane <link> there -- the real pass
    declines to shrink that (see model_builder.cpp's section_touches_a_link: touching it would
    silently break continuity with whatever's linked across that boundary), confirmed as the
    explanation for several real "declined, both explicit tags or a link" cases under --no-road-merge
    (where nearly every fragment's own outer ends are linked to a neighbor)."""
    _section_s, section_lanes = eg.cjc.applicable_at_s(road['lane_sections'], s)
    is_first_section = road['lane_sections'][0][0] == _section_s
    is_last_section = road['lane_sections'][-1][0] == _section_s
    if is_first_section and road.get('pred') and section_lane_has_link(road['body'], _section_s, side, 'predecessor'):
        return False
    if is_last_section and road.get('succ') and section_lane_has_link(road['body'], _section_s, side, 'successor'):
        return False
    types = lane_types_by_id(road['body'], _section_s)
    for lid, lane in section_lanes.items():
        if lane['side'] != side or types.get(lid) not in ('driving', 'border'):
            continue
        floor = BORDER_FLOOR_M if types.get(lid) == 'border' else min_lane_width
        if lane['width'] > floor + FLOOR_TOLERANCE_M:
            return True
    return False


def endpoints(road):
    x0, y0, _ = eg.cjc.road_point_at_s(road, 0.0)
    x1, y1, _ = eg.cjc.road_point_at_s(road, road['length'])
    return (x0, y0), (x1, y1)


def directly_linked(rid_a, road_a, rid_b, road_b):
    if primary_way_id(rid_a) is not None and primary_way_id(rid_a) == primary_way_id(rid_b):
        return True
    for link in (road_a.get('pred'), road_a.get('succ')):
        if link and link.get('elementId') == rid_b:
            return True
    for link in (road_b.get('pred'), road_b.get('succ')):
        if link and link.get('elementId') == rid_a:
            return True
    a0, a1 = endpoints(road_a)
    b0, b1 = endpoints(road_b)
    for pa in (a0, a1):
        for pb in (b0, b1):
            if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) <= ENDPOINT_TOLERANCE_M:
                return True
    return False


def check(osm_path, xodr_path, min_road_clearance, min_lane_width):
    data = open(xodr_path, encoding='utf-8').read()
    roads = parse_xodr(data)
    way_tags = parse_osm_way_tags(osm_path)

    buckets = {}
    for rid, road in roads.items():
        if road['attrs'].get('junction', '-1') != '-1':
            continue
        wid = primary_way_id(rid)
        tags = way_tags.get(wid, {}) if wid else {}
        name = road['attrs'].get('name', '') or ''
        ref = tags.get('ref', '')
        if not name and not ref:
            continue
        buckets.setdefault((name, ref), []).append((rid, road, tags))

    checked = 0
    violations = []
    for (name, ref), members in buckets.items():
        if len(members) < 2:
            continue
        for i in range(len(members)):
            for j in range(i + 1, len(members)):
                rid_a, road_a, tags_a = members[i]
                rid_b, road_b, tags_b = members[j]
                if directly_linked(rid_a, road_a, rid_b, road_b):
                    continue
                checked += 1

                edges_a = road_edge_points(road_a)
                edges_b = road_edge_points(road_b)
                # Discrete nearest-sample-pair search (not segment projection): at this 2m sampling
                # density the distance is a close enough approximation, and unlike a projected point
                # (which can land between two samples of the *other* road) it always yields a concrete
                # s on *both* roads, needed to look up which LaneSection each side's fix would have
                # landed in.
                best = (float('inf'), None, None, None, None)
                for side_a in ('left', 'right'):
                    for side_b in ('left', 'right'):
                        for s_a, xa, ya in edges_a[side_a]:
                            for s_b, xb, yb in edges_b[side_b]:
                                d = math.hypot(xa - xb, ya - yb)
                                if d < best[0]:
                                    best = (d, side_a, side_b, s_a, s_b)

                dist, side_a, side_b, s_a, s_b = best
                if dist >= min_road_clearance:
                    continue

                a_tagged = has_explicit_width_tags(tags_a)
                b_tagged = has_explicit_width_tags(tags_b)
                if a_tagged and b_tagged:
                    print(f"  NOTE: {rid_a}/{rid_b}: facing edges {dist:.3f}m apart (both width-tagged, expected untouched)")
                    continue

                a_can_give_more = not a_tagged and side_has_headroom(road_a, side_a, s_a, min_lane_width)
                b_can_give_more = not b_tagged and side_has_headroom(road_b, side_b, s_b, min_lane_width)

                acceptable = not a_can_give_more and not b_can_give_more
                if acceptable:
                    print(f"  NOTE: {rid_a}/{rid_b}: facing edges {dist:.3f}m apart, "
                          f"under {min_road_clearance:.2f}m but already at floor width (residual)")
                    continue

                violations.append(
                    f"{rid_a}/{rid_b} (name={name!r} ref={ref!r}): facing edges {dist:.3f}m apart, "
                    f"under {min_road_clearance:.2f}m, and not fully explained by width tags or floor widths [MISMATCH]")

    print(f"Checked {checked} name/ref-matched road pair(s) for lane-width overlap, {len(violations)} violation(s).")
    for v in violations:
        print("  MISMATCH: " + v)
    return not violations


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('osm')
    ap.add_argument('xodr')
    ap.add_argument('--min-road-clearance', type=float, default=0.5)
    ap.add_argument('--min-lane-width', type=float, default=2.5)
    args = ap.parse_args()
    ok = check(args.osm, args.xodr, args.min_road_clearance, args.min_lane_width)
    sys.exit(0 if ok else 1)
