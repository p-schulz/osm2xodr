#!/usr/bin/env python3
"""Quantify geometric deviation between generated network geometry and
hand-digitized ground truth, both as WGS84 GeoJSON LineStrings.

Intended workflow (see EVALUATION.md):
  1. Pick a curated ("junction"-tier) fixture.
  2. In QGIS, load an aerial orthophoto (e.g. LGL Baden-Wuerttemberg's open
     DOP WMS/WMTS layer) and hand-digitize the real curb lines / lane
     markings around that junction as a LineString layer; export as GeoJSON
     in WGS84 (EPSG:4326).
  3. Run export_geojson.py --lanes on the matching .xodr to get the
     converter's own modeled left/right lane edges.
  4. Run this script on the two GeoJSON files to get the distribution of
     perpendicular distances from every ground-truth vertex to the nearest
     generated line -- a real, imagery-grounded accuracy number (as opposed
     to the self-consistency checkers, which only verify the converter is
     internally coherent, not that it matches reality).

Both files are reprojected into the same local ENU meters frame (using
osm2xodr's own projection formula) before distances are computed, so results
are in meters regardless of latitude.

Usage:
    python3 test/compare_geojson.py generated.geojson ground_truth.geojson \\
        --origin-lat 49.0093 --origin-lon 8.4179
"""
import argparse
import json
import math
import sys


EARTH_RADIUS_M = 6378137.0


def project(lat, lon, origin_lat, origin_lon, cos_lat0):
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * cos_lat0
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def load_lines(path, origin_lat, origin_lon):
    cos_lat0 = math.cos(math.radians(origin_lat))
    data = json.loads(open(path, encoding='utf-8').read())
    lines = []
    for feat in data['features']:
        geom = feat['geometry']
        if geom['type'] != 'LineString':
            continue
        pts = [project(lat, lon, origin_lat, origin_lon, cos_lat0) for lon, lat in geom['coordinates']]
        lines.append(pts)
    return lines


def point_segment_distance(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    if len2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def nearest_distance(point, lines):
    best = math.inf
    for line in lines:
        for a, b in zip(line, line[1:]):
            d = point_segment_distance(point, a, b)
            if d < best:
                best = d
    return best


def percentile(sorted_vals, p):
    if not sorted_vals:
        return None
    idx = min(len(sorted_vals) - 1, int(round(p * (len(sorted_vals) - 1))))
    return sorted_vals[idx]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('generated_geojson')
    ap.add_argument('ground_truth_geojson')
    ap.add_argument('--origin-lat', type=float, required=True)
    ap.add_argument('--origin-lon', type=float, required=True)
    ap.add_argument('--sample-step', type=float, default=1.0,
                     help='Resample ground-truth lines at this spacing (m) before measuring, '
                          'so distance sampling density does not depend on how densely the '
                          'ground truth happened to be digitized')
    args = ap.parse_args()

    generated = load_lines(args.generated_geojson, args.origin_lat, args.origin_lon)
    ground_truth = load_lines(args.ground_truth_geojson, args.origin_lat, args.origin_lon)
    if not generated or not ground_truth:
        print("ERROR: one of the two GeoJSON files has no LineString features", file=sys.stderr)
        sys.exit(2)

    samples = []
    for line in ground_truth:
        for a, b in zip(line, line[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(seg_len // args.sample_step))
            for i in range(n + 1):
                t = i / n
                samples.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))

    distances = sorted(nearest_distance(p, generated) for p in samples)

    print(f"{len(samples)} ground-truth sample point(s) compared against {len(generated)} generated line(s).")
    print(f"  mean   = {sum(distances)/len(distances):.3f} m")
    print(f"  median = {percentile(distances, 0.5):.3f} m")
    print(f"  p95    = {percentile(distances, 0.95):.3f} m")
    print(f"  max    = {distances[-1]:.3f} m")


if __name__ == '__main__':
    main()
