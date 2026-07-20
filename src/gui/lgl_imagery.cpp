#include "lgl_imagery.hpp"

#include <curl/curl.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#elif defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace osm2xodr::gui {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr int kMaxPixelsPerSide = 2500;

double deg2rad(double d) { return d * kPi / 180.0; }
double rad2deg(double r) { return r * 180.0 / kPi; }

struct XY {
    double x = 0.0;
    double y = 0.0;
};

XY lonlat_to_merc(double lon, double lat) {
    return {deg2rad(lon) * kEarthRadiusM, std::log(std::tan(kPi / 4.0 + deg2rad(lat) / 2.0)) * kEarthRadiusM};
}

void merc_to_lonlat(double x, double y, double& lon, double& lat) {
    lon = rad2deg(x / kEarthRadiusM);
    lat = rad2deg(2.0 * std::atan(std::exp(y / kEarthRadiusM)) - kPi / 2.0);
}

// Same formula as osm2xodr::geo::LocalProjector::project / test/fetch_lgl_imagery.py's
// local_project().
XY local_project(double lat, double lon, double origin_lat, double origin_lon) {
    const double cos_lat0 = std::cos(deg2rad(origin_lat));
    return {deg2rad(lon - origin_lon) * kEarthRadiusM * cos_lat0, deg2rad(lat - origin_lat) * kEarthRadiusM};
}

struct LayerPreset {
    std::string service;
    std::string layer; // already percent-encoded where needed (e.g. ':' -> %3A)
    std::string style;
    bool transparent = false;
    std::string source;
    std::string attribution;
};

const LayerPreset& preset_for(const std::string& name) {
    static const std::string kAttribution =
        "(c) LGL Baden-Wuerttemberg (www.lgl-bw.de), Datenlizenz Deutschland - Namensnennung 2.0";
    static const LayerPreset kDop20{
        "https://owsproxy.lgl-bw.de/owsproxy/ows/WMS_LGL-BW_ATKIS_DOP_20_C",
        "IMAGES_DOP_20_RGB", "", false,
        "WMS_LGL-BW_ATKIS_DOP_20_C / IMAGES_DOP_20_RGB", kAttribution,
    };
    static const LayerPreset kLandnutzung{
        "https://owsproxy.lgl-bw.de/owsproxy/ows/WMS_LGL-BW_Landnutzung",
        "nora%3ALandnutzung", "ln_landnutzng_f", true,
        "WMS_LGL-BW_Landnutzung / nora:Landnutzung (ATKIS Basis-DLM land use)", kAttribution,
    };
    return name == "landnutzung" ? kLandnutzung : kDop20;
}

std::string build_url(const LayerPreset& preset, double min_x, double min_y, double max_x, double max_y, int side_px) {
    std::ostringstream oss;
    oss << preset.service << "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap"
        << "&LAYERS=" << preset.layer << "&STYLES=" << preset.style << "&CRS=EPSG:3857"
        << "&BBOX=" << std::fixed << std::setprecision(3) << min_x << ',' << min_y << ',' << max_x << ',' << max_y
        << "&WIDTH=" << side_px << "&HEIGHT=" << side_px << "&FORMAT=image/png";
    if (preset.transparent) oss << "&TRANSPARENT=true";
    return oss.str();
}

size_t curl_write_cb(char* ptr, size_t size, size_t nmemb, void* userdata) {
    auto* buf = static_cast<std::vector<unsigned char>*>(userdata);
    const size_t n = size * nmemb;
    buf->insert(buf->end(), ptr, ptr + n);
    return n;
}

bool http_get(const std::string& url, std::vector<unsigned char>& out, std::string& content_type, std::string& error) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        error = "curl_easy_init failed";
        return false;
    }
    out.clear();
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "osm2xodr-gui/0.1");

    const CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        error = curl_easy_strerror(res);
        curl_easy_cleanup(curl);
        return false;
    }
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    char* ct = nullptr;
    curl_easy_getinfo(curl, CURLINFO_CONTENT_TYPE, &ct);
    if (ct) content_type = ct;
    curl_easy_cleanup(curl);

    if (http_code != 200) {
        error = "HTTP " + std::to_string(http_code);
        return false;
    }
    return true;
}

} // namespace

ImageryTile fetch_tile(double center_lat, double center_lon, double radius_m,
                        double local_origin_lat, double local_origin_lon,
                        const std::string& layer, double resolution_m) {
    ImageryTile tile;
    const LayerPreset& preset = preset_for(layer);

    const XY center_merc = lonlat_to_merc(center_lon, center_lat);
    // Web Mercator's "meters" are ground meters scaled by 1/cos(lat); inflate the requested
    // half-extent so the fetched tile actually covers a radius_m ground-meter square.
    const double merc_radius = radius_m / std::cos(deg2rad(center_lat));
    const double min_mx = center_merc.x - merc_radius, max_mx = center_merc.x + merc_radius;
    const double min_my = center_merc.y - merc_radius, max_my = center_merc.y + merc_radius;
    const int side_px = std::clamp(static_cast<int>(std::lround(2.0 * radius_m / resolution_m)), 64, kMaxPixelsPerSide);

    const std::string url = build_url(preset, min_mx, min_my, max_mx, max_my, side_px);

    std::vector<unsigned char> bytes;
    std::string content_type, http_error;
    if (!http_get(url, bytes, content_type, http_error)) {
        tile.error = "WMS request failed: " + http_error;
        return tile;
    }
    if (content_type.find("image") == std::string::npos) {
        tile.error = "WMS did not return an image (Content-Type: " + content_type + ")";
        return tile;
    }

    int w = 0, h = 0, channels = 0;
    unsigned char* pixels = stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &w, &h, &channels, 4);
    if (!pixels) {
        tile.error = std::string("PNG decode failed: ") + stbi_failure_reason();
        return tile;
    }

    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    tile.pixels.assign(pixels, pixels + static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
    stbi_image_free(pixels);

    double sw_lon = 0.0, sw_lat = 0.0, ne_lon = 0.0, ne_lat = 0.0;
    merc_to_lonlat(min_mx, min_my, sw_lon, sw_lat);
    merc_to_lonlat(max_mx, max_my, ne_lon, ne_lat);
    const XY local_sw = local_project(sw_lat, sw_lon, local_origin_lat, local_origin_lon);
    const XY local_ne = local_project(ne_lat, ne_lon, local_origin_lat, local_origin_lon);

    tile.texture_id = tex;
    tile.width_px = w;
    tile.height_px = h;
    tile.min_x = local_sw.x;
    tile.min_y = local_sw.y;
    tile.max_x = local_ne.x;
    tile.max_y = local_ne.y;
    tile.attribution = preset.attribution;
    return tile;
}

void release_tile(ImageryTile& tile) {
    if (tile.texture_id != 0) {
        const GLuint tex = tile.texture_id;
        glDeleteTextures(1, &tex);
        tile.texture_id = 0;
    }
    tile.pixels.clear();
    tile.pixels.shrink_to_fit();
}

void apply_brightness_contrast(ImageryTile& tile, int brightness, float contrast) {
    if (tile.texture_id == 0 || tile.pixels.empty()) return;

    std::vector<unsigned char> adjusted(tile.pixels.size());
    for (std::size_t i = 0; i + 3 < tile.pixels.size(); i += 4) {
        for (int c = 0; c < 3; ++c) {
            const float v = (static_cast<float>(tile.pixels[i + c]) - 128.0f) * contrast + 128.0f + static_cast<float>(brightness);
            adjusted[i + c] = static_cast<unsigned char>(std::clamp(v, 0.0f, 255.0f));
        }
        adjusted[i + 3] = tile.pixels[i + 3]; // alpha untouched
    }

    glBindTexture(GL_TEXTURE_2D, tile.texture_id);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tile.width_px, tile.height_px, GL_RGBA, GL_UNSIGNED_BYTE, adjusted.data());
}

} // namespace osm2xodr::gui
