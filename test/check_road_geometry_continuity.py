#!/usr/bin/env python3
"""Verifies that a road's own fitted-curve segments (build::fit_curve) are position- and
heading-continuous at every original OSM node -- the property curve fitting actually promises.

Unlike check_junction_continuity.py/check_road_link_continuity.py (which check continuity
*between* different <road> elements), this checks continuity *within* a single <road>'s own
consecutive <geometry> primitives. Today's piecewise-<line> output (with --no-curve-fit, or for
any road/segment fit_curve left as a plain <line>) deliberately does *not* have this property --
each segment is its own straight chord, independent of its neighbors' heading -- so this check
only meaningfully exercises curve-fitted output; it will simply report 0 boundaries checked (not
an error) on a road with a single geometry primitive.

Usage:
    python3 check_road_geometry_continuity.py <file.xodr>

Exit code is 0 if every checked internal boundary matches within 1mm/0.001rad, 1 otherwise.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_junction_continuity import parse_xodr, eval_geometry_point, angle_diff


def check(path):
    data = open(path, encoding='utf-8').read()
    roads = parse_xodr(data)

    total = 0
    bad = 0
    for rid, road in roads.items():
        geoms = road['geoms']
        for i in range(len(geoms) - 1):
            total += 1
            ex, ey, ehdg = eval_geometry_point(geoms[i], geoms[i]['length'])
            sx, sy, shdg = eval_geometry_point(geoms[i + 1], 0.0)
            dpos = math.hypot(ex - sx, ey - sy)
            dhdg = angle_diff(ehdg, shdg)
            if dpos >= 1e-3 or dhdg >= 1e-3:
                bad += 1
                print(f"  {rid} geometry {i}->{i + 1}: dpos={dpos:.5f}m dhdg={math.degrees(dhdg):.3f}deg [MISMATCH]")

    print(f"Checked {total} internal geometry-to-geometry boundary(ies), {bad} mismatched (> 1mm / 0.001rad).")
    return bad == 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    ok = check(sys.argv[1])
    sys.exit(0 if ok else 1)
