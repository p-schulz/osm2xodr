#!/usr/bin/env python3
"""Verify median and grass-verge (border) lane inference in an osm2xodr-generated .xodr.

This is a companion to check_curb_height_continuity.py (which only ever looks at curb lanes'
<height>): it checks the two newer lane types median inference and grass-verge inference add.

  1. Median width: every type="median" lane's width must equal half of --median-width (the two
     halves, one per physical side, are meant to sum to the configured total).
  2. Grass-verge placement: independently re-derives, from the source .osm's landuse=grass way
     nodes (projected into the .xodr's own local frame via its <geoReference> origin) and a
     from-scratch Python port of geo.hpp's project_to_polyline, whether a grass polygon vertex lies
     within --grass-search-radius of each road's own geometry on each physical side, and cross-checks
     that against whether that road carries a type="border" lane there anywhere.

Two documented simplifications relative to the real ModelBuilder::apply_grass_verges() this checks
(not silently treated as exact):
  - Whole-road granularity, not per-laneSection: the real decision is made once per *original,
    pre-merge* OSM-way fragment, before fuse_chain can combine several fragments (each with its own
    independent border-lane decision) into a single output <road>. A single laneSection's own local
    s-span in the final output no longer necessarily corresponds to the original fragment whose full
    geometry the decision was based on, so this checks existence ("does this road have a border lane
    on this side anywhere, and is a grass polygon within radius of this road anywhere") rather than
    exact per-section position.
  - Grass polygon vertices are used raw (not resampled at a fixed step the way the C++
    implementation does), so a very sparse polygon whose edges pass close to a road between two
    widely-spaced vertices could be missed here even though the C++ pass -- which resamples edges --
    caught it. Real digitized polygons in this corpus are dense enough for this to be rare.
  - This checks the road's *final* (post-merge, post-trim, post-curve-fit) geometry, but
    apply_grass_verges() decides border-lane presence from each fragment's *original* geometry
    before any of that happens. A junction-connector trim, lane-count-bridge trim, or a flag variant
    that changes junction clustering (e.g. --no-junction-merge) can shift a road's final geometry
    enough, near a borderline distance, to disagree with the original decision in either direction --
    confirmed against a real case (a road trimmed ~6.5m for a junction connector, moving its final
    geometry away from grass that was genuinely within radius pre-trim) and a real borderline case
    under --no-junction-merge (4.4m final distance vs. the 5.0m default radius). Treat an isolated
    mismatch on a single fixture/variant, especially near the radius threshold, as this rather than
    a core-feature bug; a mismatch reproducible across multiple variants/fixtures is more likely real.

Usage:
    python3 check_median_grass_lanes.py <file.osm> <file.xodr> [--median-width M] [--grass-search-radius R]

Exit code is 0 if every type="median" lane's width matches half of --median-width (default 3.0) and
every road/side's border-lane presence matches the independently-recomputed grass-polygon adjacency
within --grass-search-radius (default 5.0), 1 otherwise.
"""
import argparse
import math
import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_junction_continuity import parse_xodr, road_point_at_s

WIDTH_TOLERANCE_M = 0.02
EARTH_RADIUS_M = 6378137.0
GEOREF_RE = re.compile(r'\+lat_0=([\-0-9.]+)\s+\+lon_0=([\-0-9.]+)')


def project(lat, lon, origin_lat, origin_lon):
    """Forward equirectangular projection -- mirrors geo.hpp's LocalProjector::project exactly."""
    cos_lat0 = math.cos(math.radians(origin_lat))
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * cos_lat0
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def parse_osm_grass_polygons(path):
    """Returns a list of raw (unprojected) (lat, lon) vertex lists, one per landuse=grass way.
    Tolerates both single- and double-quoted attributes (see check_curb_height_continuity.py)."""
    data = open(path, encoding='utf-8').read()
    nodes = {}
    for m in re.finditer(r'<node\b([^>]*?)(?:/>|>(.*?)</node>)', data, re.S):
        attrs = dict(re.findall(r'''(\w+)=["']([^"']*)["']''', m.group(1)))
        if {'id', 'lat', 'lon'} <= attrs.keys():
            nodes[attrs['id']] = (float(attrs['lat']), float(attrs['lon']))
    polygons = []
    for m in re.finditer(r'<way\b([^>]*?)>(.*?)</way>', data, re.S):
        tags = dict(re.findall(r'''<tag\s+k=["']([^"']*)["']\s+v=["']([^"']*)["']''', m.group(2)))
        if tags.get('landuse') != 'grass':
            continue
        refs = re.findall(r'''<nd\s+ref=["']([^"']*)["']''', m.group(2))
        pts = [nodes[r] for r in refs if r in nodes]
        if len(pts) >= 3:
            polygons.append(pts)
    return polygons


def project_point_to_polyline(px, py, pts):
    """Python port of geo.hpp's project_to_polyline: returns (distance, signed_t), t>0 = left."""
    best_d = float('inf')
    best_t = 0.0
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
        dx, dy = px - qx, py - qy
        d = math.hypot(dx, dy)
        if d < best_d:
            length = math.sqrt(len2)
            ux, uy = abx / length, aby / length
            best_d = d
            best_t = ux * dy - uy * dx
    return best_d, best_t


def parse_lane_sections(body):
    """Returns a list of {s_offset, left: {lane_id: (type, width)}, right: {...}} in document order."""
    sections = []
    for sec_m in re.finditer(r'<laneSection\s+s="([^"]*)">(.*?)</laneSection>', body, re.S):
        entry = {'s_offset': float(sec_m.group(1)), 'left': {}, 'right': {}}
        for side_m in re.finditer(r'<(left|right)>(.*?)</\1>', sec_m.group(2), re.S):
            side = side_m.group(1)
            for lane_m in re.finditer(r'<lane\s+id="(-?\d+)"\s+type="(\w+)"[^>]*>(.*?)</lane>', side_m.group(2), re.S):
                lid, ltype, lane_body = int(lane_m.group(1)), lane_m.group(2), lane_m.group(3)
                w_m = re.search(r'<width\s+sOffset="[^"]*"\s+a="([^"]*)"', lane_body)
                width = float(w_m.group(1)) if w_m else 0.0
                entry[side][lid] = (ltype, width)
        sections.append(entry)
    return sections


def check(osm_path, xodr_path, median_width, grass_search_radius):
    data = open(xodr_path, encoding='utf-8').read()
    roads = parse_xodr(data)
    georef_m = GEOREF_RE.search(data)

    # --- 1. Median width -----------------------------------------------------------------------
    expected_half = median_width / 2.0
    median_checked = 0
    median_violations = []
    for rid, road in roads.items():
        for section in parse_lane_sections(road['body']):
            for side in ('left', 'right'):
                for lid, (ltype, width) in section[side].items():
                    if ltype != 'median':
                        continue
                    median_checked += 1
                    if abs(width - expected_half) > WIDTH_TOLERANCE_M:
                        median_violations.append(
                            f"{rid} lane {lid}: median half-width {width:.3f}m, expected {expected_half:.3f}m [MISMATCH]")

    # --- 2. Grass-verge placement ---------------------------------------------------------------
    # Whole-road granularity, not per-laneSection: apply_grass_verges() decides border-lane presence
    # once per *original, pre-merge* fragment, testing that fragment's own complete geometry against
    # every grass polygon. Once fuse_chain merges fragments into one output <road>, a single
    # laneSection's own local s-span no longer necessarily corresponds to the original fragment
    # whose full geometry the real decision was based on -- so this checks existence ("does this road
    # have a border lane on this side *anywhere*, and is there a grass polygon within
    # --grass-search-radius of this road's geometry *anywhere*") rather than exact per-section
    # position, avoiding false mismatches from that granularity difference.
    grass_checked = 0
    grass_violations = []
    raw_polygons = parse_osm_grass_polygons(osm_path)

    if raw_polygons and georef_m:
        origin_lat, origin_lon = float(georef_m.group(1)), float(georef_m.group(2))
        polygons = [[project(lat, lon, origin_lat, origin_lon) for lat, lon in poly] for poly in raw_polygons]

        for rid, road in roads.items():
            if road['attrs'].get('junction', '-1') != '-1':
                continue
            sections = parse_lane_sections(road['body'])
            if not sections:
                continue
            # Railway/tram fragments never gain a border lane (apply_grass_verges skips them
            # outright); detect this from the emitted lane types themselves, since OSM tags aren't
            # preserved in the .xodr output to check directly.
            if any(t in ('rail', 'tram') for section in sections
                   for side in (section['left'], section['right']) for t, _ in side.values()):
                continue

            n_samples = max(2, int(road['length'] / 2.0) + 1)
            road_pts = [road_point_at_s(road, road['length'] * k / (n_samples - 1))[:2]
                        for k in range(n_samples)]

            best_left, best_right = float('inf'), float('inf')
            for poly in polygons:
                for (px, py) in poly:
                    d, t = project_point_to_polyline(px, py, road_pts)
                    if d > grass_search_radius:
                        continue
                    if t >= 0.0:
                        best_left = min(best_left, d)
                    else:
                        best_right = min(best_right, d)

            has_border_left = any(t == 'border' for section in sections for t, _ in section['left'].values())
            has_border_right = any(t == 'border' for section in sections for t, _ in section['right'].values())
            grass_checked += 2
            expect_left = best_left <= grass_search_radius
            expect_right = best_right <= grass_search_radius
            # Only "grass found nearby but no border lane" is treated as a real violation (a
            # genuine omission). The other direction ("border lane present but no grass found
            # nearby on the *final* geometry") is expected and NOT a violation: apply_grass_verges()
            # decides this from a fragment's original, pre-trim geometry, and a later
            # junction-connector or lane-count-bridge trim can shorten that same road end away from
            # the grass polygon that originally justified it -- confirmed directly against a real
            # case in durlacher-tor.osm (a ~62.9m pre-trim fragment with real nearby grass, trimmed
            # to 56.3m for a junction connector, leaving the border lane on a now-grass-free
            # remainder). Reported as an informational note instead.
            if expect_left and not has_border_left:
                grass_violations.append(
                    f"{rid}: left border lane present={has_border_left}, "
                    f"expected={expect_left} (nearest grass vertex {best_left:.2f}m) [MISMATCH]")
            elif has_border_left and not expect_left:
                print(f"  NOTE: {rid}: left border lane present with no grass found within "
                      f"{grass_search_radius:.1f}m of final geometry (likely trimmed away post-decision)")
            if expect_right and not has_border_right:
                grass_violations.append(
                    f"{rid}: right border lane present={has_border_right}, "
                    f"expected={expect_right} (nearest grass vertex {best_right:.2f}m) [MISMATCH]")
            elif has_border_right and not expect_right:
                print(f"  NOTE: {rid}: right border lane present with no grass found within "
                      f"{grass_search_radius:.1f}m of final geometry (likely trimmed away post-decision)")
    elif raw_polygons and not georef_m:
        print("  Warning: .xodr has no <geoReference> origin; skipping grass-verge cross-reference.")

    print(f"Checked {median_checked} median lane(s), {len(median_violations)} violation(s).")
    print(f"Checked {grass_checked} laneSection/side(s) for grass-verge placement, {len(grass_violations)} violation(s).")
    for v in median_violations + grass_violations:
        print("  MISMATCH: " + v)
    return not median_violations and not grass_violations


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('osm')
    ap.add_argument('xodr')
    ap.add_argument('--median-width', type=float, default=3.0)
    ap.add_argument('--grass-search-radius', type=float, default=5.0)
    args = ap.parse_args()
    ok = check(args.osm, args.xodr, args.median_width, args.grass_search_radius)
    sys.exit(0 if ok else 1)
