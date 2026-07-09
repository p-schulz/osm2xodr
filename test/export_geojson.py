#!/usr/bin/env python3
"""Reproject an osm2xodr .xodr file's road geometry back to WGS84 GeoJSON.

Purpose: let you load the *generated* network into QGIS (or any GIS) on top of
an aerial orthophoto -- e.g. LGL Baden-Wuerttemberg's open DOP (Digitale
Orthophotos) WMS/WMTS layer, https://opengeodata.lgl-bw.de (Datenlizenz
Deutschland - Zero) -- and visually or quantitatively compare the converter's
modeled cross-section against real curb lines/lane markings, not just against
OSM's own (possibly noisy) way geometry.

This reuses osm2xodr's exact local-projection formula (see geo.hpp
LocalProjector::project) run in reverse, so you MUST pass the same
--origin-lat/--origin-lon that was given to osm2xodr for this file (or, if
osm2xodr was run without explicit --origin-lat/--origin-lon, the lat/lon of
the *first node* osm2xodr encountered in the input .osm -- run_benchmark.py
always passes explicit origin flags for exactly this reason).

Usage:
    python3 test/export_geojson.py output.xodr --origin-lat 49.0093 --origin-lon 8.4179 -o output.geojson
    python3 test/export_geojson.py output.xodr --origin-lat 49.0093 --origin-lon 8.4179 --lanes -o output.geojson

Without --lanes: one LineString feature per road (its reference line).
With --lanes: also one LineString per road for the modeled left/right outer
lane edges (approximate curb lines), sampled every --step meters (default 2m).
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_junction_continuity as cjc  # reuse its .xodr geometry parsing/evaluation, not reimplement it

EARTH_RADIUS_M = 6378137.0


def inverse_project(x, y, origin_lat, origin_lon):
    """Inverse of osm2xodr's LocalProjector::project (equirectangular tangent)."""
    cos_lat0 = math.cos(math.radians(origin_lat))
    lon = origin_lon + math.degrees(x / (EARTH_RADIUS_M * cos_lat0))
    lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    return lon, lat


def sample_s_values(length, step):
    n = max(1, int(math.ceil(length / step)))
    return [min(length, i * step) for i in range(n + 1)]


def total_side_extent_at_s(section_lanes, section_s, s, side):
    total = 0.0
    for lane in section_lanes.values():
        if lane['side'] != side:
            continue
        total += lane['width'] + lane['width_b'] * (s - section_s)
    return total


def road_edge_offsets_at_s(road, s):
    """(left_edge_offset, right_edge_offset): signed lateral offset from the
    reference line to the outer edge of the modeled cross-section on each
    side -- an approximation of the real curb line."""
    section_s, section_lanes = cjc.applicable_at_s(road['lane_sections'], s)
    lo_s, lo_a, lo_b = cjc.applicable_at_s(road['lane_offsets'], s)
    lane_offset = lo_a + lo_b * (s - lo_s)
    left_total = total_side_extent_at_s(section_lanes, section_s, s, 'left')
    right_total = total_side_extent_at_s(section_lanes, section_s, s, 'right')
    return lane_offset + left_total, lane_offset - right_total


def road_to_features(rid, road, origin_lat, origin_lon, step, include_lanes):
    features = []
    s_values = sample_s_values(road['length'], step)

    def centerline_coords():
        coords = []
        for s in s_values:
            x, y, _hdg = cjc.road_point_at_s(road, s)
            coords.append(list(inverse_project(x, y, origin_lat, origin_lon)))
        return coords

    features.append({
        'type': 'Feature',
        'properties': {'road_id': rid, 'junction': road['attrs'].get('junction', '-1'),
                        'length_m': road['length'], 'kind': 'reference_line'},
        'geometry': {'type': 'LineString', 'coordinates': centerline_coords()},
    })

    if include_lanes:
        for side_name, sign in (('left_edge', 1.0), ('right_edge', -1.0)):
            coords = []
            for s in s_values:
                x, y, hdg = cjc.road_point_at_s(road, s)
                left_off, right_off = road_edge_offsets_at_s(road, s)
                offset = left_off if sign > 0 else right_off
                nx, ny = cjc.left_normal(hdg)
                coords.append(list(inverse_project(x + nx * offset, y + ny * offset, origin_lat, origin_lon)))
            features.append({
                'type': 'Feature',
                'properties': {'road_id': rid, 'junction': road['attrs'].get('junction', '-1'), 'kind': side_name},
                'geometry': {'type': 'LineString', 'coordinates': coords},
            })

    return features


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xodr')
    ap.add_argument('--origin-lat', type=float, required=True)
    ap.add_argument('--origin-lon', type=float, required=True)
    ap.add_argument('--step', type=float, default=2.0, help='Sampling interval along each road, meters')
    ap.add_argument('--lanes', action='store_true', help='Also export approximate left/right outer lane edges')
    ap.add_argument('-o', '--output', required=True)
    args = ap.parse_args()

    data = Path(args.xodr).read_text(encoding='utf-8')
    roads = cjc.parse_xodr(data)

    features = []
    for rid, road in roads.items():
        features.extend(road_to_features(rid, road, args.origin_lat, args.origin_lon, args.step, args.lanes))

    Path(args.output).write_text(json.dumps({'type': 'FeatureCollection', 'features': features}), encoding='utf-8')
    print(f"Wrote {len(features)} feature(s) from {len(roads)} road(s) to {args.output}", file=sys.stderr)


if __name__ == '__main__':
    main()
