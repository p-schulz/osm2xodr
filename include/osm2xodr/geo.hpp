#pragma once

#include "osm2xodr/util.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace osm2xodr::geo {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEarthRadiusM = 6378137.0;

inline double deg_to_rad(const double deg) { return deg * kPi / 180.0; }

inline double norm_angle(double a) {
    while (a <= -kPi) a += 2.0 * kPi;
    while (a > kPi) a -= 2.0 * kPi;
    return a;
}

struct LonLat {
    double lon = 0.0;
    double lat = 0.0;
};

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(const Vec2& a, const double s) { return {a.x * s, a.y * s}; }

inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline double length(const Vec2& v) { return std::sqrt(dot(v, v)); }
inline double heading(const Vec2& a, const Vec2& b) { return std::atan2(b.y - a.y, b.x - a.x); }

inline Vec2 normalize(const Vec2& v) {
    const double len = length(v);
    if (len <= 1e-12) return {0.0, 0.0};
    return {v.x / len, v.y / len};
}

inline Vec2 left_normal(const Vec2& dir) { return {-dir.y, dir.x}; }

// Solves A + t*dA == B + u*dB for (t, u). Returns nullopt if dA and dB are (near-)parallel.
inline std::optional<std::pair<double, double>> line_intersect_params(const Vec2& A, const Vec2& dA, const Vec2& B, const Vec2& dB) {
    const double denom = cross(dA, dB);
    if (std::abs(denom) <= 1e-9) return std::nullopt;
    const Vec2 ab = B - A;
    const double t = cross(ab, dB) / denom;
    const double u = cross(ab, dA) / denom;
    return std::make_pair(t, u);
}

struct LocalProjector {
    bool has_origin = false;
    LonLat origin{};
    double cos_lat0 = 1.0;

    void set_origin(const double lat, const double lon) {
        origin = {lon, lat};
        cos_lat0 = std::cos(deg_to_rad(lat));
        has_origin = true;
    }

    void ensure_origin(const double lat, const double lon) {
        if (!has_origin) set_origin(lat, lon);
    }

    Vec2 project(const double lat, const double lon) const {
        if (!has_origin) util::fail("projection origin not set");
        return {
            deg_to_rad(lon - origin.lon) * kEarthRadiusM * cos_lat0,
            deg_to_rad(lat - origin.lat) * kEarthRadiusM
        };
    }
};

inline double polyline_length(const std::vector<Vec2>& pts) {
    double s = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) s += length(pts[i] - pts[i - 1]);
    return s;
}

struct ProjectionOnPolyline {
    double distance = std::numeric_limits<double>::max();
    double s = 0.0;
    double t = 0.0;
    Vec2 nearest{};
};

inline ProjectionOnPolyline project_to_polyline(const std::vector<Vec2>& pts, const Vec2& p) {
    ProjectionOnPolyline best;
    double s_base = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Vec2 a = pts[i - 1];
        const Vec2 b = pts[i];
        const Vec2 ab = b - a;
        const double len2 = dot(ab, ab);
        if (len2 <= 1e-9) continue;
        double u = dot(p - a, ab) / len2;
        u = std::max(0.0, std::min(1.0, u));
        const Vec2 q = a + ab * u;
        const double d = length(p - q);
        if (d < best.distance) {
            const double len = std::sqrt(len2);
            best.distance = d;
            best.s = s_base + u * len;
            best.nearest = q;
            const Vec2 unit = ab * (1.0 / len);
            best.t = cross(unit, p - q); // positive means left of reference direction
        }
        s_base += std::sqrt(len2);
    }
    return best;
}

} // namespace osm2xodr::geo
