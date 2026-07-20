#!/usr/bin/env python3
"""Dynamic pygame viewer: drag and drop an osm2xodr .xodr file onto the window to see its
road centerlines and lane edges drawn aligned with real LGL Baden-Wuerttemberg aerial
imagery and official land-use ground truth.

The .xodr's own <geoReference> header (osm2xodr always writes one, a proj4 string with
+lat_0/+lon_0) supplies the projection origin, so no --origin-lat/--origin-lon flags are
needed here -- just drop the file. Both WMS tiles are centered on the network's own bounding
box (not the geoReference origin itself, which may sit off to one side of the network) and
sized to cover it, capped at a 2000m radius for sanity on city-scale extracts; roads
extending beyond that radius still draw (as vector lines), just without a background behind
them.

The land-use layer (N to toggle) is LGL's official ATKIS Basis-DLM Landnutzung polygons --
independent ground truth for e.g. confirming a generated road's footprint actually falls
inside the real "Strassen- und Wegeverkehr" (road traffic) class, not just eyeballing it
against the aerial photo.

Requires pygame + numpy. This repo's tooling doesn't add either to the system Python (see
README's "Validating against real ground imagery" section) -- create a project-local
virtualenv once and run through it:
    python3 -m venv .venv && .venv/bin/pip install pygame numpy
    .venv/bin/python3 test/xodr_viewer.py [initial.xodr]

Controls:
    drag & drop a .xodr file onto the window -- loads/replaces the current network
    left-drag: pan            mouse wheel: zoom
    I: toggle imagery         C: toggle centerlines        L: toggle lane edges
    M: toggle lane markings (dashed lines between individual lanes, off by default)
    N: toggle land-use ground truth overlay (semi-transparent, off by default)
    [ / ]: decrease/increase imagery brightness       - / =: decrease/increase contrast
    0: reset brightness/contrast to default (persists across drag & drop reloads otherwise)
    R: reset view             Esc / Q: quit
"""
import argparse
import io
import math
import re
import sys
from pathlib import Path

import numpy as np
import pygame

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_junction_continuity as cjc  # noqa: E402
import export_geojson as eg  # noqa: E402
import fetch_lgl_imagery as fli  # noqa: E402

GEOREF_RE = re.compile(r'\+lat_0=([\-0-9.]+)\s+\+lon_0=([\-0-9.]+)')
EARTH_RADIUS_M = 6378137.0
MAX_IMAGERY_RADIUS_M = 2000.0
MARKING_DASH_M = 3.0  # dash/gap length in world meters, roughly matching real broken lane lines
MARKING_GAP_M = 3.0

COLOR_BG = (10, 10, 12)
COLOR_CENTER = (255, 80, 80)
COLOR_LANE = (255, 212, 0)
COLOR_MARKING = (235, 235, 235)
COLOR_TEXT = (230, 230, 230)
COLOR_TEXT_DIM = (150, 150, 150)


def lane_marking_traces(road, step):
    """Traces every boundary *between* individual lanes (not just each side's outer edge --
    that's lane_edges/road_edge_offsets_at_s) along a road: one polyline per contiguous run of
    a given (side, ordinal-from-centerline) lane's outer boundary. Lane counts can change
    mid-road (splits/merges insert a new <laneSection>), so a boundary is only continuous
    while the same ordinal keeps existing -- gaps end one run and start a new one."""
    s_values = eg.sample_s_values(road['length'], step)
    open_runs = {}
    traces = []

    def flush(key):
        pts = open_runs.pop(key, None)
        if pts and len(pts) >= 2:
            traces.append(np.array(pts))

    for s in s_values:
        x, y, hdg = cjc.road_point_at_s(road, s)
        nx, ny = cjc.left_normal(hdg)
        section_s, section_lanes = cjc.applicable_at_s(road['lane_sections'], s)
        lo_s, lo_a, lo_b = cjc.applicable_at_s(road['lane_offsets'], s)
        lane_offset = lo_a + lo_b * (s - lo_s)

        present = set()
        for lane in section_lanes.values():
            ordinal = len(lane['inner_lanes'])
            key = (lane['side'], ordinal)
            present.add(key)
            inner_acc = sum(w + wb * (s - section_s) for w, wb in lane['inner_lanes'])
            width = lane['width'] + lane['width_b'] * (s - section_s)
            sign = 1.0 if lane['side'] == 'left' else -1.0
            offset = lane_offset + sign * (inner_acc + width)
            open_runs.setdefault(key, []).append((x + nx * offset, y + ny * offset))

        for key in list(open_runs.keys()):
            if key not in present:
                flush(key)

    for key in list(open_runs.keys()):
        flush(key)
    return traces


def adjust_brightness_contrast(surface, brightness, contrast):
    """brightness: additive, roughly -150..150. contrast: multiplicative around mid-gray
    (128), roughly 0.2..3.0 (1.0 = unchanged). Uses numpy directly on the pixel buffer rather
    than per-pixel Python, since tiles are up to 2500x2500."""
    arr = pygame.surfarray.array3d(surface).astype(np.float32)
    arr = (arr - 128.0) * contrast + 128.0 + brightness
    np.clip(arr, 0, 255, out=arr)
    return pygame.surfarray.make_surface(arr.astype(np.uint8))


def apply_alpha_factor(surface, factor):
    """Copy of `surface` (which must have per-pixel alpha, i.e. loaded with convert_alpha())
    with every pixel's alpha multiplied by `factor` (0..1). Used for the land-use overlay's
    blend opacity while preserving the WMS's own per-pixel transparency (no-data areas stay
    fully transparent instead of tinting the photo underneath)."""
    out = surface.copy()
    alpha = pygame.surfarray.pixels_alpha(out)
    alpha[:] = (alpha.astype(np.float32) * factor).astype(np.uint8)
    del alpha  # release the surface lock pixels_alpha() holds
    return out


def dash_runs(pts, dash=MARKING_DASH_M, gap=MARKING_GAP_M):
    """Index ranges of pts (a roughly-uniformly-sampled (N,2) world-space polyline) that fall
    in the 'on' part of a dash/gap pattern measured by cumulative arc length, so the dash
    length stays a constant number of meters regardless of sampling density or zoom."""
    if len(pts) < 2:
        return []
    seg_len = np.hypot(*(pts[1:] - pts[:-1]).T)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    on = (cum % (dash + gap)) < dash
    runs = []
    start = None
    for i, is_on in enumerate(on):
        if is_on and start is None:
            start = i
        elif not is_on and start is not None:
            if i - start >= 2:
                runs.append((start, i))
            start = None
    if start is not None and len(on) - start >= 2:
        runs.append((start, len(on)))
    return runs


def inverse_project(x, y, origin_lat, origin_lon):
    """Inverse of osm2xodr's LocalProjector::project (equirectangular tangent)."""
    cos_lat0 = math.cos(math.radians(origin_lat))
    lon = origin_lon + math.degrees(x / (EARTH_RADIUS_M * cos_lat0))
    lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    return lon, lat


class RoadNetwork:
    """Parses a .xodr into drawable numpy point arrays, entirely in osm2xodr's local
    equirectangular-meters frame -- no WGS84 conversion needed until fetch_layer()."""

    def __init__(self, path, step=2.0):
        self.path = Path(path)
        text = self.path.read_text(encoding='utf-8')
        m = GEOREF_RE.search(text)
        self.origin_lat, self.origin_lon = (float(m.group(1)), float(m.group(2))) if m else (None, None)

        roads = cjc.parse_xodr(text)
        self.road_count = len(roads)
        self.centerlines = []
        self.lane_edges = []
        self.lane_markings = []
        min_x = min_y = math.inf
        max_x = max_y = -math.inf

        for road in roads.values():
            s_values = eg.sample_s_values(road['length'], step)
            if len(s_values) < 2:
                continue
            center = np.empty((len(s_values), 2))
            left = np.empty((len(s_values), 2))
            right = np.empty((len(s_values), 2))
            for i, s in enumerate(s_values):
                x, y, hdg = cjc.road_point_at_s(road, s)
                center[i] = (x, y)
                left_off, right_off = eg.road_edge_offsets_at_s(road, s)
                nx, ny = cjc.left_normal(hdg)
                left[i] = (x + nx * left_off, y + ny * left_off)
                right[i] = (x + nx * right_off, y + ny * right_off)
            self.centerlines.append(center)
            self.lane_edges.append(left)
            self.lane_edges.append(right)
            self.lane_markings.extend(lane_marking_traces(road, step))
            pts = np.vstack([center, left, right])
            min_x = min(min_x, pts[:, 0].min())
            max_x = max(max_x, pts[:, 0].max())
            min_y = min(min_y, pts[:, 1].min())
            max_y = max(max_y, pts[:, 1].max())

        self.bbox = (min_x, min_y, max_x, max_y) if self.centerlines else (-50.0, -50.0, 50.0, 50.0)

    def fetch_layer(self, layer='dop20', resolution_m=0.2):
        """Returns (png_bytes_or_None, sidecar_or_None, note_or_None) for the given WMS layer
        preset (see fetch_lgl_imagery.LAYER_PRESETS), centered/sized to cover this network's
        own bounding box. Reused for both the aerial photo and the land-use overlay -- same
        center/radius math for both, so their bbox_local_m end up numerically identical and
        pixel-for-pixel aligned without extra bookkeeping."""
        if self.origin_lat is None:
            return None, None, "no <geoReference> lat_0/lon_0 in this .xodr -- imagery skipped"
        min_x, min_y, max_x, max_y = self.bbox
        half_w, half_h = (max_x - min_x) / 2, (max_y - min_y) / 2
        wanted_radius = max(half_w, half_h) * 1.15
        radius_m = max(30.0, min(wanted_radius, MAX_IMAGERY_RADIUS_M))
        center_lon, center_lat = inverse_project((min_x + max_x) / 2, (min_y + max_y) / 2,
                                                  self.origin_lat, self.origin_lon)
        try:
            png_bytes, sidecar = fli.fetch_tile(center_lat, center_lon, radius_m, resolution_m,
                                                 local_origin_lat=self.origin_lat,
                                                 local_origin_lon=self.origin_lon, layer=layer)
        except Exception as exc:  # noqa: BLE001 -- surfaced to the HUD, not fatal
            return None, None, f"{layer} fetch failed: {exc}"
        note = None
        if wanted_radius > MAX_IMAGERY_RADIUS_M:
            note = f"network extends beyond the fetched {layer} tile (capped at {MAX_IMAGERY_RADIUS_M:.0f}m radius)"
        return png_bytes, sidecar, note


class Camera:
    """World (osm2xodr local meters, +y = north) <-> screen pixel transform."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.cx = 0.0
        self.cy = 0.0
        self.scale = 1.0  # screen pixels per meter

    def world_to_screen(self, pts):
        sx = self.width / 2 + (pts[:, 0] - self.cx) * self.scale
        sy = self.height / 2 - (pts[:, 1] - self.cy) * self.scale
        return np.stack([sx, sy], axis=1)

    def screen_to_world(self, sx, sy):
        wx = (sx - self.width / 2) / self.scale + self.cx
        wy = -(sy - self.height / 2) / self.scale + self.cy
        return wx, wy

    def re_anchor(self, world_pt, screen_pt):
        """Set cx/cy so that world_pt maps to screen_pt at the current scale."""
        wx, wy = world_pt
        sx, sy = screen_pt
        self.cx = wx - (sx - self.width / 2) / self.scale
        self.cy = wy + (sy - self.height / 2) / self.scale

    def fit(self, bbox, margin=1.15):
        min_x, min_y, max_x, max_y = bbox
        w, h = max(max_x - min_x, 1e-3), max(max_y - min_y, 1e-3)
        self.cx, self.cy = (min_x + max_x) / 2, (min_y + max_y) / 2
        self.scale = min(self.width / (w * margin), self.height / (h * margin))


class Viewer:
    def __init__(self, screen, step=2.0):
        self.screen = screen
        self.step = step
        self.font = pygame.font.SysFont(None, 20)
        self.camera = Camera(*screen.get_size())
        self.network = None
        self.image_surface = None
        self.image_bbox = None
        self._adjusted_surface = None  # image_surface after brightness/contrast, cached
        self._cached_scaled = None  # ((w, h), Surface), scaled from _adjusted_surface
        self.brightness = 0
        self.contrast = 1.0
        self.landuse_surface = None
        self.landuse_bbox = None
        self.landuse_alpha = 160  # 0-255 blend opacity over the aerial photo/background
        self._cached_landuse_blend = None  # Surface, landuse_surface with landuse_alpha applied
        self._cached_landuse_scaled = None  # ((w, h), Surface), scaled from _cached_landuse_blend
        self.show_imagery = True
        self.show_center = True
        self.show_lanes = True
        self.show_markings = False
        self.show_landuse = False
        self.status_lines = ["Drag and drop a .xodr file onto this window to begin."]
        self.dragging = False
        self._drag_anchor_world = None

    def resize(self, width, height):
        self.camera.width, self.camera.height = width, height
        self._cached_scaled = None
        self._cached_landuse_scaled = None

    def _invalidate_image_cache(self):
        self._adjusted_surface = None
        self._cached_scaled = None

    def _invalidate_landuse_cache(self):
        self._cached_landuse_blend = None
        self._cached_landuse_scaled = None

    def load(self, path):
        self.network = None
        self.image_surface = None
        self.image_bbox = None
        self._invalidate_image_cache()
        self.landuse_surface = None
        self.landuse_bbox = None
        self._invalidate_landuse_cache()
        self.status_lines = [f"Loading {Path(path).name}..."]
        self.render()
        pygame.display.flip()

        try:
            network = RoadNetwork(path, step=self.step)
        except Exception as exc:  # noqa: BLE001 -- surfaced to the HUD, not fatal
            self.status_lines = [f"Failed to parse {Path(path).name}: {exc}"]
            return

        self.network = network
        self.camera.fit(network.bbox)
        self.status_lines = [f"{Path(path).name} -- {network.road_count} road(s)"]
        self.render()
        pygame.display.flip()

        png_bytes, sidecar, note = network.fetch_layer('dop20')
        lines = list(self.status_lines)
        if png_bytes:
            self.image_surface = pygame.image.load(io.BytesIO(png_bytes), 'tile.png').convert()
            self.image_bbox = sidecar['bbox_local_m']
            lines.append(f"imagery: {sidecar['width_px']}x{sidecar['height_px']}px "
                         f"({sidecar.get('attribution', '')})")
        if note:
            lines.append(note)
        self.status_lines = lines
        self.render()
        pygame.display.flip()

        lu_bytes, lu_sidecar, lu_note = network.fetch_layer('landnutzung', resolution_m=0.5)
        lines = list(self.status_lines)
        if lu_bytes:
            self.landuse_surface = pygame.image.load(io.BytesIO(lu_bytes), 'tile.png').convert_alpha()
            self.landuse_bbox = lu_sidecar['bbox_local_m']
            lines.append(f"landuse: {lu_sidecar['width_px']}x{lu_sidecar['height_px']}px")
        if lu_note:
            lines.append(lu_note)
        self.status_lines = lines

    def _draw_image(self):
        b = self.image_bbox
        top_left = self.camera.world_to_screen(np.array([[b['min_x'], b['max_y']]]))[0]
        bottom_right = self.camera.world_to_screen(np.array([[b['max_x'], b['min_y']]]))[0]
        w = max(1, round(bottom_right[0] - top_left[0]))
        h = max(1, round(bottom_right[1] - top_left[1]))
        if self._adjusted_surface is None:
            if self.brightness == 0 and self.contrast == 1.0:
                self._adjusted_surface = self.image_surface
            else:
                self._adjusted_surface = adjust_brightness_contrast(
                    self.image_surface, self.brightness, self.contrast)
            self._cached_scaled = None
        if self._cached_scaled is None or self._cached_scaled[0] != (w, h):
            self._cached_scaled = ((w, h), pygame.transform.smoothscale(self._adjusted_surface, (w, h)))
        self.screen.blit(self._cached_scaled[1], (top_left[0], top_left[1]))

    def _draw_landuse(self):
        b = self.landuse_bbox
        top_left = self.camera.world_to_screen(np.array([[b['min_x'], b['max_y']]]))[0]
        bottom_right = self.camera.world_to_screen(np.array([[b['max_x'], b['min_y']]]))[0]
        w = max(1, round(bottom_right[0] - top_left[0]))
        h = max(1, round(bottom_right[1] - top_left[1]))
        if self._cached_landuse_blend is None:
            self._cached_landuse_blend = apply_alpha_factor(self.landuse_surface, self.landuse_alpha / 255.0)
            self._cached_landuse_scaled = None
        if self._cached_landuse_scaled is None or self._cached_landuse_scaled[0] != (w, h):
            self._cached_landuse_scaled = ((w, h), pygame.transform.smoothscale(self._cached_landuse_blend, (w, h)))
        self.screen.blit(self._cached_landuse_scaled[1], (top_left[0], top_left[1]))

    def _draw_hud(self):
        y = 8
        for line in self.status_lines:
            surf = self.font.render(line, True, COLOR_TEXT)
            self.screen.blit(surf, (10, y))
            y += surf.get_height() + 2
        help_text = ("drag&drop .xodr | drag: pan | wheel: zoom | I/C/L/M/N: toggle | "
                     "[ ] brightness | - = contrast | 0 reset image | R: reset view | Esc: quit")
        surf = self.font.render(help_text, True, COLOR_TEXT_DIM)
        self.screen.blit(surf, (10, self.camera.height - surf.get_height() - 6))
        if self.image_surface is not None:
            bc_text = f"brightness {self.brightness:+d}   contrast {self.contrast:.1f}x"
            surf = self.font.render(bc_text, True, COLOR_TEXT_DIM)
            self.screen.blit(surf, (10, self.camera.height - surf.get_height() * 2 - 10))

    def render(self):
        self.screen.fill(COLOR_BG)
        if self.image_surface is not None and self.image_bbox is not None and self.show_imagery:
            self._draw_image()
        if self.landuse_surface is not None and self.landuse_bbox is not None and self.show_landuse:
            self._draw_landuse()
        if self.network is not None:
            if self.show_center:
                for pts in self.network.centerlines:
                    screen_pts = self.camera.world_to_screen(pts)
                    pygame.draw.lines(self.screen, COLOR_CENTER, False, screen_pts.tolist(), 2)
            if self.show_lanes:
                for pts in self.network.lane_edges:
                    screen_pts = self.camera.world_to_screen(pts)
                    pygame.draw.lines(self.screen, COLOR_LANE, False, screen_pts.tolist(), 1)
            if self.show_markings:
                for pts in self.network.lane_markings:
                    screen_pts = self.camera.world_to_screen(pts).tolist()
                    for start, end in dash_runs(pts):
                        pygame.draw.lines(self.screen, COLOR_MARKING, False, screen_pts[start:end], 1)
        self._draw_hud()

    def handle_event(self, event):
        if event.type == pygame.DROPFILE:
            self.load(event.file)
        elif event.type == pygame.VIDEORESIZE:
            self.resize(event.w, event.h)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.dragging = True
            self._drag_anchor_world = self.camera.screen_to_world(*event.pos)
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self.camera.re_anchor(self._drag_anchor_world, event.pos)
            self._cached_scaled = None
            self._cached_landuse_scaled = None
        elif event.type == pygame.MOUSEWHEEL:
            mouse_pos = pygame.mouse.get_pos()
            anchor = self.camera.screen_to_world(*mouse_pos)
            self.camera.scale = max(0.02, min(2000.0, self.camera.scale * (1.1 ** event.y)))
            self.camera.re_anchor(anchor, mouse_pos)
            self._cached_scaled = None
            self._cached_landuse_scaled = None
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r and self.network is not None:
                self.camera.fit(self.network.bbox)
                self._cached_scaled = None
                self._cached_landuse_scaled = None
            elif event.key == pygame.K_i:
                self.show_imagery = not self.show_imagery
            elif event.key == pygame.K_c:
                self.show_center = not self.show_center
            elif event.key == pygame.K_l:
                self.show_lanes = not self.show_lanes
            elif event.key == pygame.K_m:
                self.show_markings = not self.show_markings
            elif event.key == pygame.K_n:
                self.show_landuse = not self.show_landuse
            elif event.key == pygame.K_LEFT:
                self.brightness = max(-150, self.brightness - 10)
                self._invalidate_image_cache()
            elif event.key == pygame.K_RIGHT:
                self.brightness = min(150, self.brightness + 10)
                self._invalidate_image_cache()
            elif event.key == pygame.K_DOWN:
                self.contrast = max(0.2, round(self.contrast - 0.1, 2))
                self._invalidate_image_cache()
            elif event.key == pygame.K_UP:
                self.contrast = min(3.0, round(self.contrast + 0.1, 2))
                self._invalidate_image_cache()
            elif event.key == pygame.K_0:
                self.brightness = 0
                self.contrast = 1.0
                self._invalidate_image_cache()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xodr', nargs='?', help='Optional .xodr to load at startup (or just drag one in later)')
    ap.add_argument('--step', type=float, default=2.0, help='Sampling interval along each road, meters')
    args = ap.parse_args()

    pygame.init()
    pygame.display.set_caption("osm2xodr viewer -- drag & drop a .xodr file")
    screen = pygame.display.set_mode((1200, 900), pygame.RESIZABLE)

    viewer = Viewer(screen, step=args.step)
    if args.xodr:
        viewer.load(args.xodr)

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False
            elif event.type == pygame.VIDEORESIZE:
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                viewer.screen = screen
                viewer.resize(event.w, event.h)
            else:
                viewer.handle_event(event)

        viewer.render()
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()
