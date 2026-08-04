#!/usr/bin/env python3
"""Headless static rendering of an osm2xodr .xodr file's road geometry to a PNG.

Unlike xodr_viewer.py (interactive, pygame, requires an aerial-imagery WMS connection),
this produces a plain top-down plot of centerlines and modeled lane edges directly from
the local (already-projected) coordinates in the .xodr -- no network access, no display,
no --origin-lat/--origin-lon needed. Intended for batch use (e.g. generating one thumbnail
per corpus fixture for a paper's appendix), not for interactive inspection.

Reuses check_junction_continuity.py's .xodr parsing/geometry evaluation and
export_geojson.py's lane-edge-offset computation rather than reimplementing either.

Usage:
    python3 test/render_xodr.py output.xodr -o output.png [--step 1.0] [--dpi 200]
"""
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_junction_continuity as cjc
import export_geojson as eg

COLOR_CENTER = '#c0392b'
COLOR_EDGE = '#1a1a1a'
COLOR_CONNECTOR_EDGE = '#2f6690'


def render(xodr_path, out_path, step=1.0, dpi=200, zoom_margin=None):
    data = open(xodr_path, encoding='utf-8').read()
    roads = cjc.parse_xodr(data)

    fig, ax = plt.subplots(figsize=(6, 6))

    connector_points = []
    for rid, road in roads.items():
        is_connector = road['attrs'].get('junction', '-1') != '-1'
        s_values = eg.sample_s_values(road['length'], step)

        center_xy = [cjc.road_point_at_s(road, s)[:2] for s in s_values]
        left_xy, right_xy = [], []
        for s in s_values:
            x, y, hdg = cjc.road_point_at_s(road, s)
            left_off, right_off = eg.road_edge_offsets_at_s(road, s)
            nx, ny = cjc.left_normal(hdg)
            left_xy.append((x + nx * left_off, y + ny * left_off))
            right_xy.append((x + nx * right_off, y + ny * right_off))

        edge_color = COLOR_CONNECTOR_EDGE if is_connector else COLOR_EDGE
        edge_width = 0.6 if is_connector else 0.9
        ax.plot(*zip(*left_xy), color=edge_color, linewidth=edge_width, zorder=2)
        ax.plot(*zip(*right_xy), color=edge_color, linewidth=edge_width, zorder=2)
        if not is_connector:
            ax.plot(*zip(*center_xy), color=COLOR_CENTER, linewidth=0.4,
                     linestyle=(0, (4, 3)), alpha=0.6, zorder=1)
        else:
            connector_points.extend(center_xy)

    ax.set_aspect('equal')
    ax.axis('off')

    # Frame the view on the junction cluster (where the synthetic connectors are) rather than
    # the full extent of every approach road, so the plot is comparable in scale to a
    # junction-centered OSM screenshot instead of being dominated by long straight approaches.
    if zoom_margin is not None and connector_points:
        xs = [p[0] for p in connector_points]
        ys = [p[1] for p in connector_points]
        cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
        half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0 + zoom_margin
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
    else:
        fig.tight_layout(pad=0.1)

    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xodr')
    ap.add_argument('-o', '--output', required=True)
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--dpi', type=int, default=200)
    ap.add_argument('--zoom-margin', type=float, default=None,
                     help='If set, crop the view to the junction-connector cluster '
                          'plus this many meters of margin, instead of the full road extent.')
    args = ap.parse_args()
    render(args.xodr, args.output, step=args.step, dpi=args.dpi, zoom_margin=args.zoom_margin)


if __name__ == '__main__':
    main()
