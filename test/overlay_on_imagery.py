#!/usr/bin/env python3
"""Standalone tool: overlay an osm2xodr .xodr's road centerlines and lane edges on top of
real LGL Baden-Wuerttemberg aerial imagery, as one self-contained, interactive HTML file.

This is the tool to reach for when you want to eyeball whether the converter's modeled
geometry actually matches the real road surface -- the check_*_continuity.py scripts only
verify internal consistency, not agreement with reality.

By default it fetches a fresh tile from LGL's WMS (see fetch_lgl_imagery.py) centered on
--origin-lat/--origin-lon, which MUST be the same values passed to osm2xodr's own
--origin-lat/--origin-lon for this .xodr (run_benchmark.py's origin_for_osm() derives this
per fixture if you don't already know it). Pass --tile/--sidecar to reuse a previously
fetched tile instead (e.g. to re-render after tweaking --step without hitting the WMS
again).

Output is a single .html file with no external dependencies (the tile is embedded as a
base64 data: URI): open it in any browser. It supports mouse-wheel zoom, click-drag pan, a
reset-view button, and checkboxes to toggle the imagery/centerline/lane-edge layers.

Usage:
    python3 overlay_on_imagery.py output.xodr --origin-lat 49.0093 --origin-lon 8.4179 \\
        --radius-m 150 -o overlay.html

    # Re-render without re-fetching:
    python3 overlay_on_imagery.py output.xodr --tile tile.png --sidecar tile.json -o overlay.html
"""
import argparse
import base64
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_junction_continuity as cjc
import export_geojson as eg
import fetch_lgl_imagery as fli


def road_traces(road, step):
    s_values = eg.sample_s_values(road['length'], step)
    center, left_edge, right_edge = [], [], []
    for s in s_values:
        x, y, hdg = cjc.road_point_at_s(road, s)
        center.append((x, y))
        left_off, right_off = eg.road_edge_offsets_at_s(road, s)
        nx, ny = cjc.left_normal(hdg)
        left_edge.append((x + nx * left_off, y + ny * left_off))
        right_edge.append((x + nx * right_off, y + ny * right_off))
    return center, left_edge, right_edge


def to_pixels(points, bbox, width_px, height_px):
    """Local meters -> SVG pixel space. Image row 0 (top) is north (max_y), matching how the
    WMS server rasterizes GetMap responses."""
    min_x, max_x = bbox['min_x'], bbox['max_x']
    min_y, max_y = bbox['min_y'], bbox['max_y']
    out = []
    for x, y in points:
        px = (x - min_x) / (max_x - min_x) * width_px
        py = (max_y - y) / (max_y - min_y) * height_px
        out.append(f"{px:.1f},{py:.1f}")
    return " ".join(out)


PAGE_TEMPLATE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
  body {{ margin:0; background:#1a1a1a; color:#eee; font-family:system-ui,sans-serif; }}
  .toolbar {{ display:flex; gap:1rem; align-items:center; padding:0.6rem 1rem;
              background:#222; flex-wrap:wrap; }}
  .toolbar label {{ display:flex; gap:0.35rem; align-items:center; cursor:pointer; }}
  .toolbar button {{ background:#333; color:#eee; border:1px solid #555; border-radius:4px;
                      padding:0.3rem 0.7rem; cursor:pointer; }}
  .toolbar button:hover {{ background:#444; }}
  .legend-swatch {{ display:inline-block; width:0.9em; height:0.25em; vertical-align:middle; }}
  .viewport {{ position:relative; width:100%; height:calc(100vh - 3.2rem); overflow:hidden;
               cursor:grab; background:#000; }}
  .viewport.dragging {{ cursor:grabbing; }}
  .stage {{ position:absolute; top:0; left:0; transform-origin:0 0; }}
  .stage img {{ display:block; }}
  .stage svg {{ position:absolute; top:0; left:0; }}
  .centerline {{ fill:none; stroke:#ff5050; stroke-opacity:0.9; }}
  .laneedge {{ fill:none; stroke:#ffd400; stroke-opacity:0.85; }}
  .footer {{ padding:0.3rem 1rem; font-size:0.75rem; color:#999; background:#222; }}
</style>
</head>
<body>
  <div class="toolbar">
    <strong>{title}</strong>
    <label><input type="checkbox" checked onchange="toggleLayer('tile-img', this.checked)">
      Imagery</label>
    <label><span class="legend-swatch" style="background:#ff5050;"></span>
      <input type="checkbox" checked onchange="toggleLayer('center-group', this.checked)">
      Centerlines</label>
    <label><span class="legend-swatch" style="background:#ffd400;"></span>
      <input type="checkbox" checked onchange="toggleLayer('lanes-group', this.checked)">
      Lane edges</label>
    <button onclick="resetView()">Reset view</button>
    <span style="color:#999;">scroll to zoom, drag to pan</span>
  </div>
  <div class="viewport" id="viewport">
    <div class="stage" id="stage" style="width:{width_px}px;height:{height_px}px;">
      <img id="tile-img" src="data:image/png;base64,{image_b64}" alt="aerial tile"
           width="{width_px}" height="{height_px}" />
      <svg id="overlay-svg" width="{width_px}" height="{height_px}"
           viewBox="0 0 {width_px} {height_px}">
        <g id="center-group">{center_polylines}</g>
        <g id="lanes-group">{lane_polylines}</g>
      </svg>
    </div>
  </div>
  <div class="footer">{attribution} &mdash; {road_count} road(s) drawn from {xodr_name}</div>
<script>
(function() {{
  var viewport = document.getElementById('viewport');
  var stage = document.getElementById('stage');
  var nativeW = {width_px}, nativeH = {height_px};
  var scale = 1, tx = 0, ty = 0, fitScale = 1;

  function apply() {{
    stage.style.transform = 'translate(' + tx + 'px,' + ty + 'px) scale(' + scale + ')';
  }}

  function fitToViewport() {{
    fitScale = Math.min(viewport.clientWidth / nativeW, viewport.clientHeight / nativeH);
    scale = fitScale;
    tx = (viewport.clientWidth - nativeW * scale) / 2;
    ty = (viewport.clientHeight - nativeH * scale) / 2;
    apply();
  }}

  window.resetView = fitToViewport;

  window.toggleLayer = function(id, visible) {{
    document.getElementById(id).style.display = visible ? '' : 'none';
  }};

  viewport.addEventListener('wheel', function(e) {{
    e.preventDefault();
    var rect = viewport.getBoundingClientRect();
    var cx = e.clientX - rect.left, cy = e.clientY - rect.top;
    var factor = Math.pow(1.1, -e.deltaY / 100);
    var newScale = Math.min(fitScale * 40, Math.max(fitScale * 0.2, scale * factor));
    tx = cx - (cx - tx) * (newScale / scale);
    ty = cy - (cy - ty) * (newScale / scale);
    scale = newScale;
    apply();
  }}, {{ passive: false }});

  var dragging = false, startX, startY, startTx, startTy;
  viewport.addEventListener('mousedown', function(e) {{
    dragging = true;
    viewport.classList.add('dragging');
    startX = e.clientX; startY = e.clientY; startTx = tx; startTy = ty;
  }});
  window.addEventListener('mousemove', function(e) {{
    if (!dragging) return;
    tx = startTx + (e.clientX - startX);
    ty = startTy + (e.clientY - startY);
    apply();
  }});
  window.addEventListener('mouseup', function() {{
    dragging = false;
    viewport.classList.remove('dragging');
  }});

  window.addEventListener('resize', fitToViewport);
  fitToViewport();
}})();
</script>
</body>
</html>
"""


def build_html(xodr_path, roads, image_bytes, sidecar, step):
    bbox = sidecar['bbox_local_m']
    width_px, height_px = sidecar['width_px'], sidecar['height_px']
    stroke = max(0.4, width_px / 900.0)

    center_lines, lane_lines = [], []
    for rid, road in roads.items():
        center, left_edge, right_edge = road_traces(road, step)
        if len(center) < 2:
            continue
        center_lines.append(
            f'<polyline points="{to_pixels(center, bbox, width_px, height_px)}" '
            f'class="centerline" style="stroke-width:{stroke * 1.4:.2f}"><title>{rid}</title></polyline>')
        for edge in (left_edge, right_edge):
            lane_lines.append(
                f'<polyline points="{to_pixels(edge, bbox, width_px, height_px)}" '
                f'class="laneedge" style="stroke-width:{stroke * 0.7:.2f}"><title>{rid}</title></polyline>')

    return PAGE_TEMPLATE.format(
        title=f"{Path(xodr_path).name} on LGL aerial imagery",
        width_px=width_px, height_px=height_px,
        image_b64=base64.b64encode(image_bytes).decode('ascii'),
        center_polylines=''.join(center_lines),
        lane_polylines=''.join(lane_lines),
        attribution=sidecar.get('attribution', ''),
        road_count=len(roads),
        xodr_name=Path(xodr_path).name,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xodr')
    ap.add_argument('--origin-lat', type=float, help='Required unless --tile/--sidecar are given')
    ap.add_argument('--origin-lon', type=float, help='Required unless --tile/--sidecar are given')
    ap.add_argument('--radius-m', type=float, default=150.0, help='Half side-length of the fetched square, meters')
    ap.add_argument('--resolution-m', type=float, default=0.2, help='Ground resolution, meters/pixel')
    ap.add_argument('--step', type=float, default=1.0, help='Sampling interval along each road, meters')
    ap.add_argument('--tile', help='Reuse an existing tile PNG instead of fetching a new one')
    ap.add_argument('--sidecar', help='Matching tile .json (required if --tile is given)')
    ap.add_argument('-o', '--output', required=True, help='Output .html path')
    args = ap.parse_args()

    if args.tile:
        if not args.sidecar:
            sys.exit("ERROR: --sidecar is required when --tile is given")
        image_bytes = Path(args.tile).read_bytes()
        sidecar = json.loads(Path(args.sidecar).read_text(encoding='utf-8'))
    else:
        if args.origin_lat is None or args.origin_lon is None:
            sys.exit("ERROR: --origin-lat/--origin-lon are required unless --tile/--sidecar are given")
        print("Fetching aerial tile from LGL WMS...", file=sys.stderr)
        try:
            image_bytes, sidecar = fli.fetch_tile(args.origin_lat, args.origin_lon, args.radius_m, args.resolution_m)
        except RuntimeError as exc:
            sys.exit(f"ERROR: {exc}")
        tile_base = Path(args.output).with_suffix('')
        Path(f"{tile_base}_tile.png").write_bytes(image_bytes)
        Path(f"{tile_base}_tile.json").write_text(json.dumps(sidecar, indent=2), encoding='utf-8')
        print(f"Cached tile as {tile_base}_tile.png / {tile_base}_tile.json "
              f"(reuse with --tile/--sidecar to skip re-fetching)", file=sys.stderr)

    data = Path(args.xodr).read_text(encoding='utf-8')
    roads = cjc.parse_xodr(data)

    html = build_html(args.xodr, roads, image_bytes, sidecar, args.step)
    Path(args.output).write_text(html, encoding='utf-8')
    print(f"Wrote {args.output} ({len(roads)} road(s) drawn)", file=sys.stderr)


if __name__ == '__main__':
    main()
