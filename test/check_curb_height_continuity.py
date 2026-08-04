#!/usr/bin/env python3
"""Verify curb-lane height continuity and dropped-curb tapers in an osm2xodr-generated .xodr.

This is a companion to check_road_link_continuity.py (which verifies position/heading continuity
of plain road-to-road lane links) and check_junction_continuity.py (junction connector geometry):
neither looks at a curb lane's <height> at all. This script checks two, independent things:

  1. Continuity: for every plain (non-junction) road-to-road lane link where the linked lane is
     type="curb" on both sides, its <height> value at the shared boundary must match on both sides
     (within a small tolerance) -- a curb shouldn't silently step up or down where two roads meet
     without an intervening crossing.
  2. Curb-cut correctness: cross-referencing the source .osm's highway=crossing/kerb=* nodes
     (projected into the .xodr's own local frame via its <geoReference> origin) against the curb
     height actually present at the nearest matching road end -- kerb=raised must stay at (or very
     close to) the road's own nominal curb height there; every other case (kerb=lowered/flush/
     rounded, or no kerb tag at all) must taper to ~0.

Only checks type="curb" lanes; a road/side with no sidewalk (hence no curb lane at all, since
osm2xodr only inserts a curb where it also inserts a sidewalk) is silently skipped -- there is
nothing to verify there.

Usage:
    python3 check_curb_height_continuity.py <file.osm> <file.xodr>

Exit code is 0 if every checked curb-height link matches within 1cm and every checked crossing's
curb height matches the expected raised/tapered outcome within 1cm, 1 otherwise.
"""
import math
import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_junction_continuity import parse_xodr, road_point_at_s

HEIGHT_TOLERANCE_M = 0.01
GEOREF_RE = re.compile(r'\+lat_0=([\-0-9.]+)\s+\+lon_0=([\-0-9.]+)')
EARTH_RADIUS_M = 6378137.0

# Real crossing-node position tolerance: a highway=crossing node that osm2xodr actually treats as a
# curb-cut is always a forced fragment-split boundary (see build_fragments), so a genuine match's
# road-end coordinate is the *exact same* projected point as the crossing node itself (same OSM
# node, same projector) -- essentially zero error, floating-point noise only. Deliberately tight, not
# generous: a looser tolerance (e.g. a few meters) causes false-positive matches to an unrelated
# nearby-but-distinct crossing node in a dense junction area, where several crossings can legitimately
# sit within a few meters of each other without being the same node.
CROSSING_MATCH_TOLERANCE_M = 0.1


def project(lat, lon, origin_lat, origin_lon):
    """Forward equirectangular projection -- mirrors geo.hpp's LocalProjector::project exactly, not
    export_geojson.py's inverse_project (which goes the other way)."""
    cos_lat0 = math.cos(math.radians(origin_lat))
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * cos_lat0
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def parse_osm_crossings(path):
    """Returns a list of (lat, lon, kerb_or_None) for every highway=crossing node. Tolerates both
    single- and double-quoted attributes: real OSM-API dumps use double quotes, but hand-written
    synthetic fixtures in this repo (e.g. tiny_intersection.osm) use single quotes."""
    data = open(path, encoding='utf-8').read()
    crossings = []
    for m in re.finditer(r'<node\b([^>]*?)(?:/>|>(.*?)</node>)', data, re.S):
        attrs = dict(re.findall(r'''(\w+)=["']([^"']*)["']''', m.group(1)))
        body = m.group(2) or ''
        tags = dict(re.findall(r'''<tag\s+k=["']([^"']*)["']\s+v=["']([^"']*)["']''', body))
        if tags.get('highway') != 'crossing':
            continue
        if 'lat' not in attrs or 'lon' not in attrs:
            continue
        crossings.append((float(attrs['lat']), float(attrs['lon']), tags.get('kerb')))
    return crossings


def parse_curb_lane_sections(body):
    """Returns a list of laneSections, each {lane_id: [(sOffset, inner_height), ...]} for every
    type="curb" lane, plus that section's own <link> predecessor/successor id per lane -- in
    document order (so index 0 is s=0, index -1 is the section touching the road's own end)."""
    sections = []
    for sec_m in re.finditer(r'<laneSection\b[^>]*>(.*?)</laneSection>', body, re.S):
        heights = {}
        links = {}
        for side_m in re.finditer(r'<(left|right)>(.*?)</\1>', sec_m.group(1), re.S):
            for lane_m in re.finditer(r'<lane\s+id="(-?\d+)"\s+type="curb"[^>]*>(.*?)</lane>', side_m.group(2), re.S):
                lid = int(lane_m.group(1))
                lane_body = lane_m.group(2)
                records = [(float(s), float(inner)) for s, inner in
                           re.findall(r'<height\s+sOffset="([^"]*)"\s+inner="([^"]*)"\s+outer="[^"]*"/>', lane_body)]
                heights[lid] = sorted(records)
                pred_m = re.search(r'<predecessor\s+id="(-?\d+)"', lane_body)
                succ_m = re.search(r'<successor\s+id="(-?\d+)"', lane_body)
                links[lid] = {
                    'predecessor': int(pred_m.group(1)) if pred_m else None,
                    'successor': int(succ_m.group(1)) if succ_m else None,
                }
        sections.append({'heights': heights, 'links': links})
    return sections


def height_at_start(records, nominal):
    """Curb height at this laneSection's own s=0 -- the first record if any exist, else `nominal`
    (a section the taper pass never touched carries no explicit <height> at all only when the whole
    road has no curb; a curb lane always has *some* <height>, so this fallback is just defensive)."""
    return records[0][1] if records else nominal


def height_at_end(records, nominal):
    """Curb height at this laneSection's own far end -- the last record if any, else `nominal`."""
    return records[-1][1] if records else nominal


def nominal_curb_height(all_sections):
    """The most common non-tapered height across every section of a road -- used as the 'this is
    what a raised (non-tapered) curb should read as' reference when matching a raised crossing."""
    from collections import Counter
    counts = Counter()
    for section in all_sections:
        for records in section['heights'].values():
            for _, h in records:
                counts[round(h, 3)] += 1
    return counts.most_common(1)[0][0] if counts else 0.0


def check(osm_path, xodr_path):
    data = open(xodr_path, encoding='utf-8').read()
    roads = parse_xodr(data)
    georef_m = GEOREF_RE.search(data)

    curb_sections = {rid: parse_curb_lane_sections(r['body']) for rid, r in roads.items()}

    # --- 1. Plain road-to-road curb-height link continuity ---------------------------------------
    total_links = 0
    bad_links = 0
    checked_pairs = set()

    def contact_s(road, contact_point):
        return 0.0 if contact_point == 'start' else road['length']

    def check_direction(rid, link_elem, section, direction):
        nonlocal total_links, bad_links
        if not link_elem or link_elem.get('elementType') != 'road':
            return
        neighbor_id = link_elem['elementId']
        if neighbor_id not in roads or not curb_sections.get(neighbor_id):
            return
        neighbor_sections = curb_sections[neighbor_id]
        contact_point = link_elem.get('contactPoint', 'start')
        neighbor_section = neighbor_sections[0] if contact_point == 'start' else neighbor_sections[-1]
        neighbor_direction = 'successor' if direction == 'predecessor' else 'predecessor'

        pair_key = tuple(sorted([(rid, direction), (neighbor_id, neighbor_direction)]))
        if pair_key in checked_pairs:
            return
        checked_pairs.add(pair_key)

        for lane_id, link in section['links'].items():
            target_lane = link.get(direction)
            if target_lane is None or target_lane not in neighbor_section['links']:
                continue
            total_links += 1
            records = section['heights'][lane_id]
            h1 = height_at_end(records, 0.0) if direction == 'successor' else height_at_start(records, 0.0)
            neighbor_records = neighbor_section['heights'][target_lane]
            h2 = height_at_start(neighbor_records, 0.0) if neighbor_direction == 'predecessor' else height_at_end(neighbor_records, 0.0)
            # Symmetry: the neighbor's own link back must name this exact lane, not just any lane
            # that happens to read a similar height -- mirrors check_road_link_continuity.py's own
            # `symmetric` check, since a one-directional link declaration is a real topology defect
            # even when the two (possibly wrong) lanes it's comparing happen to coincide in height.
            back = neighbor_section['links'][target_lane].get(neighbor_direction)
            symmetric = (back == lane_id)
            if abs(h1 - h2) > HEIGHT_TOLERANCE_M or not symmetric:
                bad_links += 1
                print(f"  {rid} lane {lane_id} {direction} -> {neighbor_id} lane {target_lane}: "
                      f"curb height {h1:.3f}m vs {h2:.3f}m symmetric={symmetric} [MISMATCH]")

    for rid, road in roads.items():
        if road['attrs'].get('junction', '-1') != '-1':
            continue
        sections = curb_sections[rid]
        if not sections:
            continue
        check_direction(rid, road['pred'], sections[0], 'predecessor')
        check_direction(rid, road['succ'], sections[-1], 'successor')

    # --- 2. Curb-cut correctness at highway=crossing/kerb=* nodes ---------------------------------
    crossing_checks = 0
    crossing_violations = []
    crossings = parse_osm_crossings(osm_path)

    if crossings and georef_m:
        origin_lat, origin_lon = float(georef_m.group(1)), float(georef_m.group(2))
        projected = [(project(lat, lon, origin_lat, origin_lon), kerb) for lat, lon, kerb in crossings]

        for rid, road in roads.items():
            if road['attrs'].get('junction', '-1') != '-1':
                continue
            sections = curb_sections[rid]
            if not sections:
                continue
            nominal = nominal_curb_height(sections)
            if nominal <= 1e-6:
                continue  # no real curb height configured (--curb-height 0 or similar) -- nothing to check

            sx, sy, _ = road_point_at_s(road, 0.0)
            ex, ey, _ = road_point_at_s(road, road['length'])
            for (px, py), kerb in projected:
                for (x, y), section, label in ((sx, sy), sections[0], 'start'), ((ex, ey), sections[-1], 'end'):
                    if math.hypot(px - x, py - y) > CROSSING_MATCH_TOLERANCE_M:
                        continue
                    for lane_id, records in section['heights'].items():
                        h = height_at_start(records, nominal) if label == 'start' else height_at_end(records, nominal)
                        crossing_checks += 1
                        expect_raised = (kerb or '').strip().lower() == 'raised'
                        if expect_raised and h < 0.5 * nominal:
                            crossing_violations.append(
                                f"{rid} lane {lane_id} at road {label} (kerb=raised): "
                                f"curb height {h:.3f}m, expected close to nominal {nominal:.3f}m [MISMATCH]")
                        elif not expect_raised and h > HEIGHT_TOLERANCE_M:
                            crossing_violations.append(
                                f"{rid} lane {lane_id} at road {label} (kerb={kerb or '(absent, default lowered)'}): "
                                f"curb height {h:.3f}m, expected ~0 (tapered) [MISMATCH]")
    elif crossings and not georef_m:
        print("  Warning: .xodr has no <geoReference> origin; skipping crossing/kerb cross-reference.")

    print(f"Checked {total_links} plain road-to-road curb-height link(s), {bad_links} mismatched (> {HEIGHT_TOLERANCE_M*100:.0f}cm).")
    print(f"Checked {crossing_checks} curb height(s) at highway=crossing nodes, {len(crossing_violations)} violation(s).")
    for v in crossing_violations:
        print("  MISMATCH: " + v)
    return bad_links == 0 and len(crossing_violations) == 0


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    ok = check(sys.argv[1], sys.argv[2])
    sys.exit(0 if ok else 1)
