#!/usr/bin/env python3
"""Fetch an LGL Baden-Wuerttemberg WMS tile (aerial photo or official land-use polygons) as a
georeferenced PNG.

Two layer presets, both served via https://owsproxy.lgl-bw.de, both Datenlizenz Deutschland -
Namensnennung 2.0, both covering Baden-Wuerttemberg only (roughly lon 7.2-10.7, lat 47.4-50.0):
  - 'dop20' (default): WMS_LGL-BW_ATKIS_DOP_20_C, 20cm color orthophotos.
  - 'landnutzung': WMS_LGL-BW_Landnutzung, nora:Landnutzung -- official ATKIS Basis-DLM land
    use polygons (residential/industrial/agriculture/forestry/road-traffic/etc., half-yearly
    updated), rendered with its own fill-color style (ln_landnutzng_f) and alpha transparency
    where there's no data. Useful as independent ground truth: e.g. compare a generated
    .xodr's road footprint against the real "Strassen- und Wegeverkehr" (road traffic)
    polygons, not just against the aerial photo.

Requests in EPSG:3857 (Web Mercator): this server's EPSG:4326 GetMap responses come back
blank regardless of axis order (verified directly against the live service), so 3857 is the
one CRS confirmed to actually return imagery.

Can be used as a library (fetch_tile()) or run directly, writing <output>.png (the image)
and <output>.json (a sidecar with the image's bounding box in both WGS84 and osm2xodr's own
local equirectangular-meters frame, using the same --origin-lat/--origin-lon convention as
export_geojson.py/compare_geojson.py, so overlay_on_imagery.py can place it without another
reprojection step).

Usage:
    python3 fetch_lgl_imagery.py --origin-lat 49.0093 --origin-lon 8.4179 --radius-m 200 -o karlsruhe
    python3 fetch_lgl_imagery.py --origin-lat 49.0093 --origin-lon 8.4179 --layer landnutzung -o karlsruhe_landuse
"""
import argparse
import json
import math
import sys
import urllib.request
import urllib.parse

EARTH_RADIUS_M = 6378137.0
MAX_PIXELS_PER_SIDE = 2500  # keep requests well under typical WMS server request-size limits

WMS_HOST = "https://owsproxy.lgl-bw.de/owsproxy/ows"
_ATTRIBUTION = '(c) LGL Baden-Wuerttemberg (www.lgl-bw.de), Datenlizenz Deutschland - Namensnennung 2.0'
LAYER_PRESETS = {
    'dop20': {
        'service': f'{WMS_HOST}/WMS_LGL-BW_ATKIS_DOP_20_C',
        'layer': 'IMAGES_DOP_20_RGB',
        'style': '',
        'transparent': False,
        'source': 'WMS_LGL-BW_ATKIS_DOP_20_C / IMAGES_DOP_20_RGB',
        'attribution': _ATTRIBUTION,
    },
    'landnutzung': {
        'service': f'{WMS_HOST}/WMS_LGL-BW_Landnutzung',
        'layer': 'nora:Landnutzung',
        'style': 'ln_landnutzng_f',
        'transparent': True,
        'source': 'WMS_LGL-BW_Landnutzung / nora:Landnutzung (ATKIS Basis-DLM land use)',
        'attribution': _ATTRIBUTION,
    },
}


def lonlat_to_merc(lon, lat):
    x = math.radians(lon) * EARTH_RADIUS_M
    y = math.log(math.tan(math.pi / 4 + math.radians(lat) / 2)) * EARTH_RADIUS_M
    return x, y


def merc_to_lonlat(x, y):
    lon = math.degrees(x / EARTH_RADIUS_M)
    lat = math.degrees(2 * math.atan(math.exp(y / EARTH_RADIUS_M)) - math.pi / 2)
    return lon, lat


def local_project(lat, lon, origin_lat, origin_lon):
    """Same formula as osm2xodr's LocalProjector::project / compare_geojson.py's project()."""
    cos_lat0 = math.cos(math.radians(origin_lat))
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * cos_lat0
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def fetch_tile(origin_lat, origin_lon, radius_m=150.0, resolution_m=0.2,
                local_origin_lat=None, local_origin_lon=None, layer='dop20'):
    """Fetch a georeferenced square WMS tile centered on (origin_lat, origin_lon).

    layer: one of LAYER_PRESETS ('dop20' aerial photo, or 'landnutzung' official land-use
    polygons) -- see the module docstring.

    local_origin_lat/local_origin_lon: the point bbox_local_m is measured relative to.
    Defaults to the fetch center itself (origin_lat/origin_lon), which is correct whenever
    the caller's own local coordinate frame is anchored there too (e.g. export_geojson.py/
    compare_geojson.py/overlay_on_imagery.py, which all take a single explicit
    --origin-lat/--origin-lon used for everything). Pass this explicitly whenever the fetch
    is centered on something else -- e.g. xodr_viewer.py centers the fetch on the network's
    own bounding-box centroid (to frame the imagery on the roads rather than on the .xodr's
    raw <geoReference> origin, which can sit off in a corner), while the road geometry itself
    is still expressed relative to that raw origin -- passing it here keeps bbox_local_m in
    the same frame as that geometry instead of silently centering the image on the wrong
    point (a bug caught exactly this way: roads and imagery each individually correct, but
    offset from each other by the vector between the two origins).

    Returns (png_bytes, sidecar_dict). Raises RuntimeError if the WMS doesn't return an image
    (e.g. the point is outside Baden-Wuerttemberg coverage)."""
    if local_origin_lat is None:
        local_origin_lat, local_origin_lon = origin_lat, origin_lon
    preset = LAYER_PRESETS[layer]
    cx, cy = lonlat_to_merc(origin_lon, origin_lat)
    # Web Mercator's "meters" are ground meters scaled by 1/cos(lat); inflate the requested
    # half-extent so the fetched tile actually covers a radius_m ground-meter square.
    merc_radius = radius_m / math.cos(math.radians(origin_lat))
    min_x, min_y, max_x, max_y = cx - merc_radius, cy - merc_radius, cx + merc_radius, cy + merc_radius

    side_px = min(MAX_PIXELS_PER_SIDE, max(64, round(2 * radius_m / resolution_m)))

    params = {
        'SERVICE': 'WMS', 'VERSION': '1.3.0', 'REQUEST': 'GetMap',
        'LAYERS': preset['layer'], 'STYLES': preset['style'], 'CRS': 'EPSG:3857',
        'BBOX': f"{min_x:.3f},{min_y:.3f},{max_x:.3f},{max_y:.3f}",
        'WIDTH': str(side_px), 'HEIGHT': str(side_px), 'FORMAT': 'image/png',
    }
    if preset['transparent']:
        params['TRANSPARENT'] = 'true'
    url = preset['service'] + '?' + urllib.parse.urlencode(params)
    with urllib.request.urlopen(url, timeout=30) as resp:
        content_type = resp.headers.get('Content-Type', '')
        data = resp.read()
    if 'image' not in content_type:
        raise RuntimeError(f"WMS did not return an image (Content-Type: {content_type}): "
                            f"{data[:500].decode('utf-8', 'replace')}")

    sw_lon, sw_lat = merc_to_lonlat(min_x, min_y)
    ne_lon, ne_lat = merc_to_lonlat(max_x, max_y)
    lx0, ly0 = local_project(sw_lat, sw_lon, local_origin_lat, local_origin_lon)
    lx1, ly1 = local_project(ne_lat, ne_lon, local_origin_lat, local_origin_lon)
    sidecar = {
        'source': preset['source'],
        'attribution': preset['attribution'],
        'origin_lat': origin_lat, 'origin_lon': origin_lon,
        'bbox_wgs84': {'min_lon': sw_lon, 'min_lat': sw_lat, 'max_lon': ne_lon, 'max_lat': ne_lat},
        'bbox_local_m': {'min_x': lx0, 'min_y': ly0, 'max_x': lx1, 'max_y': ly1},
        'width_px': side_px, 'height_px': side_px,
    }
    return data, sidecar


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--origin-lat', type=float, required=True,
                     help='Center latitude -- pass the same value given to osm2xodr --origin-lat')
    ap.add_argument('--origin-lon', type=float, required=True,
                     help='Center longitude -- pass the same value given to osm2xodr --origin-lon')
    ap.add_argument('--radius-m', type=float, default=150.0, help='Half side-length of the fetched square, in meters')
    ap.add_argument('--resolution-m', type=float, default=0.2,
                     help='Target ground resolution, meters/pixel (source imagery is native 20cm)')
    ap.add_argument('--layer', choices=sorted(LAYER_PRESETS), default='dop20',
                     help="'dop20' aerial photo (default) or 'landnutzung' official land-use polygons")
    ap.add_argument('-o', '--output', required=True, help='Output basename (writes <output>.png and <output>.json)')
    args = ap.parse_args()

    try:
        data, sidecar = fetch_tile(args.origin_lat, args.origin_lon, args.radius_m, args.resolution_m,
                                    layer=args.layer)
    except RuntimeError as exc:
        sys.exit(f"ERROR: {exc}")

    if sidecar['width_px'] == MAX_PIXELS_PER_SIDE:
        print(f"warning: requested resolution would exceed {MAX_PIXELS_PER_SIDE}px/side, capping "
              f"(effective resolution ~{2 * args.radius_m / sidecar['width_px']:.3f} m/px)", file=sys.stderr)

    png_path = args.output if args.output.endswith('.png') else args.output + '.png'
    json_path = png_path[:-4] + '.json'
    with open(png_path, 'wb') as f:
        f.write(data)
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(sidecar, f, indent=2)
    print(f"Wrote {png_path} ({sidecar['width_px']}x{sidecar['height_px']}px) and {json_path}", file=sys.stderr)


if __name__ == '__main__':
    main()
