#pragma once

#include <string>
#include <vector>

namespace osm2xodr::gui {

struct ImageryTile {
    unsigned int texture_id = 0; // GLuint; 0 = none/failed
    int width_px = 0;
    int height_px = 0;
    std::vector<unsigned char> pixels; // original decoded RGBA8, width_px*height_px*4 bytes --
                                        // kept around (not just the uploaded texture) so
                                        // apply_brightness_contrast() can be reapplied from the
                                        // untouched source without re-fetching from the network.
    double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0; // world bbox, local meters
    std::string attribution;
    std::string error; // non-empty on failure; texture_id is 0 whenever this is set
};

// Fetches a square WMS tile from LGL Baden-Wuerttemberg -- 'dop20' (aerial photo, default) or
// 'landnutzung' (official ATKIS Basis-DLM land-use polygons) -- centered on (center_lat,
// center_lon) with half-extent radius_m, and uploads it as an OpenGL texture (must be called with
// a current GL context, i.e. from the render thread).
//
// local_origin_lat/local_origin_lon is the point the returned bbox is measured relative to. This
// is deliberately a separate parameter from the fetch center: the road network's own local-meters
// frame is anchored at the .xodr's <geoReference> origin, which is generally NOT the same point as
// wherever the preview happens to center the fetch (e.g. the network's own bounding-box centroid,
// to frame the imagery on the roads rather than on an origin that might sit off in a corner) --
// passing the *road* origin here keeps the tile in that same frame instead of silently drawing it
// offset from the roads (see test/fetch_lgl_imagery.py's fetch_tile() docstring; this is a bug
// that was actually hit once in the Python tool and is deliberately guarded against here too).
//
// Blocking network call -- only call this synchronously the way it's used here (right after a
// user-initiated Convert), not from a tight per-frame path.
ImageryTile fetch_tile(double center_lat, double center_lon, double radius_m,
                        double local_origin_lat, double local_origin_lon,
                        const std::string& layer = "dop20", double resolution_m = 0.2);

void release_tile(ImageryTile& tile);

// Re-uploads tile.texture_id from tile.pixels (the original, untouched decode) with brightness
// (additive, roughly -150..150) and contrast (multiplicative around mid-gray, roughly 0.2..3.0,
// 1.0 = unchanged) applied -- same formula as test/xodr_viewer.py's adjust_brightness_contrast(),
// just re-uploading a texture instead of recomputing a pygame Surface. Alpha is left untouched.
// Cheap enough to call whenever these values actually change (not every frame).
void apply_brightness_contrast(ImageryTile& tile, int brightness, float contrast);

} // namespace osm2xodr::gui
