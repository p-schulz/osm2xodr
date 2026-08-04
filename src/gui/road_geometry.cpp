#include "road_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace osm2xodr::gui {
namespace {

struct PointHeading {
    geo::Vec2 pos;
    double hdg = 0.0;
};

// Evaluates one planView primitive at arc-length s into it. Line/Arc mirror model_builder.cpp's
// own evaluate_geometry_point(); ParamPoly3 evaluates the cubic Bezier directly (P0 implicit at
// the local origin) rather than converting to power-basis coefficients first -- same curve,
// simpler than re-deriving the bU/cU/dU/bV/cV/dV xodr_writer.cpp computes for the XML output.
PointHeading evaluate_primitive(const model::GeomPrimitive& g, double s) {
    if (g.kind == model::GeomKind::Arc) {
        const double r = 1.0 / g.curvature;
        const double theta = g.curvature * s;
        const double cx = g.x - r * std::sin(g.hdg);
        const double cy = g.y + r * std::cos(g.hdg);
        return {{cx + r * std::sin(g.hdg + theta), cy - r * std::cos(g.hdg + theta)}, g.hdg + theta};
    }
    if (g.kind == model::GeomKind::ParamPoly3) {
        const double p = (g.length > 1e-9) ? s / g.length : 0.0;
        const double omp = 1.0 - p;
        const double u = 3 * omp * omp * p * g.local_p1.x + 3 * omp * p * p * g.local_p2.x + p * p * p * g.local_p3.x;
        const double v = 3 * omp * omp * p * g.local_p1.y + 3 * omp * p * p * g.local_p2.y + p * p * p * g.local_p3.y;
        const double du = 3 * omp * omp * g.local_p1.x + 6 * omp * p * (g.local_p2.x - g.local_p1.x) +
                           3 * p * p * (g.local_p3.x - g.local_p2.x);
        const double dv = 3 * omp * omp * g.local_p1.y + 6 * omp * p * (g.local_p2.y - g.local_p1.y) +
                           3 * p * p * (g.local_p3.y - g.local_p2.y);
        const double cos_h = std::cos(g.hdg), sin_h = std::sin(g.hdg);
        return {{g.x + u * cos_h - v * sin_h, g.y + u * sin_h + v * cos_h}, g.hdg + std::atan2(dv, du)};
    }
    return {{g.x + std::cos(g.hdg) * s, g.y + std::sin(g.hdg) * s}, g.hdg};
}

// Position/heading at arc-length s along a whole road: walks road.explicit_geometry if present
// (exactly what xodr_writer.cpp emits as <geometry> elements), else falls back to piecewise-linear
// segments through road.points (xodr_writer.cpp's own fallback for roads with no fitted curve).
PointHeading road_point_at_s(const model::RoadSegment& road, double s) {
    if (!road.explicit_geometry.empty()) {
        double acc = 0.0;
        for (const auto& g : road.explicit_geometry) {
            if (s <= acc + g.length + 1e-6) return evaluate_primitive(g, s - acc);
            acc += g.length;
        }
        const auto& g = road.explicit_geometry.back();
        return evaluate_primitive(g, g.length);
    }
    double acc = 0.0;
    for (std::size_t i = 1; i < road.points.size(); ++i) {
        const auto& a = road.points[i - 1];
        const auto& b = road.points[i];
        const double len = geo::length(b - a);
        if (len <= 1e-6) continue;
        if (s <= acc + len + 1e-6) {
            const double t = (s - acc) / len;
            return {{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}, geo::heading(a, b)};
        }
        acc += len;
    }
    if (road.points.size() >= 2) {
        const auto& a = road.points[road.points.size() - 2];
        const auto& b = road.points.back();
        return {b, geo::heading(a, b)};
    }
    return {road.points.empty() ? geo::Vec2{} : road.points.back(), 0.0};
}

// One lane cross-section boundary (road.lanes at s_offset=0, or one of road.extra_lane_sections),
// with the span its own width/lane-offset tapers are normalized over -- mirrors xodr_writer.cpp's
// write_lane_section span_length computation exactly.
struct SectionRef {
    double s_offset = 0.0;
    const model::LanePlan* lanes = nullptr;
    double span = 1.0;
};

std::vector<SectionRef> collect_sections(const model::RoadSegment& road) {
    std::vector<SectionRef> sections;
    const double first_span = road.extra_lane_sections.empty() ? road.length : road.extra_lane_sections.front().s_offset;
    sections.push_back({0.0, &road.lanes, std::max(first_span, 1e-6)});
    for (std::size_t i = 0; i < road.extra_lane_sections.size(); ++i) {
        const auto& section = road.extra_lane_sections[i];
        const double span = (i + 1 < road.extra_lane_sections.size())
            ? road.extra_lane_sections[i + 1].s_offset - section.s_offset
            : road.length - section.s_offset;
        sections.push_back({section.s_offset, &section.lanes, std::max(span, 1e-6)});
    }
    return sections;
}

const SectionRef& applicable_section(const std::vector<SectionRef>& sections, double s) {
    const SectionRef* chosen = &sections.front();
    for (const auto& section : sections) {
        if (section.s_offset <= s + 1e-6) chosen = &section;
        else break;
    }
    return *chosen;
}

double side_extent(const std::vector<model::LaneSpec>& side, double local_s, double span) {
    double total = 0.0;
    for (const auto& lane : side) {
        double w = lane.width;
        if (lane.width_end >= 0.0) w = lane.width + (lane.width_end - lane.width) * (local_s / span);
        total += w;
    }
    return total;
}

bool has_track_lane(const std::vector<model::LaneSpec>& side) {
    for (const auto& lane : side) {
        if (lane.type == "rail" || lane.type == "tram") return true;
    }
    return false;
}

void expand_bounds(const WorldPoint& p, double& min_x, double& min_y, double& max_x, double& max_y) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
}

} // namespace

RoadTraces sample_road(const model::RoadSegment& road, double step) {
    RoadTraces traces;
    if (road.length <= 1e-6) return traces;

    traces.is_railway = has_track_lane(road.lanes.left) || has_track_lane(road.lanes.right);

    const int n = std::max(1, static_cast<int>(std::ceil(road.length / step)));
    const auto sections = collect_sections(road);
    traces.centerline.reserve(n + 1);
    traces.left_edge.reserve(n + 1);
    traces.right_edge.reserve(n + 1);

    for (int i = 0; i <= n; ++i) {
        const double s = std::min(road.length, road.length * i / n);
        const auto [pos, hdg] = road_point_at_s(road, s);
        const auto& section = applicable_section(sections, s);
        const double local_s = s - section.s_offset;
        const double lane_offset = section.lanes->lane_offset + section.lanes->lane_offset_slope * local_s;
        const double left_total = side_extent(section.lanes->left, local_s, section.span);
        const double right_total = side_extent(section.lanes->right, local_s, section.span);
        const geo::Vec2 n_left = geo::left_normal({std::cos(hdg), std::sin(hdg)});

        traces.centerline.push_back({pos.x, pos.y});
        traces.left_edge.push_back({pos.x + n_left.x * (lane_offset + left_total), pos.y + n_left.y * (lane_offset + left_total)});
        traces.right_edge.push_back({pos.x + n_left.x * (lane_offset - right_total), pos.y + n_left.y * (lane_offset - right_total)});
    }
    return traces;
}

// Ordinal here is simply the lane's index within LanePlan::left/right -- unlike the Python parser
// (which re-derives an "inner_lanes count" from a dict keyed by arbitrary OSM lane id, since it's
// working from parsed XML text), our LanePlan already stores each side's lanes centre-outward in
// vector order, so the index *is* the ordinal directly.
std::vector<std::vector<WorldPoint>> sample_lane_markings(const model::RoadSegment& road, double step) {
    std::vector<std::vector<WorldPoint>> traces;
    if (road.length <= 1e-6) return traces;

    const int n = std::max(1, static_cast<int>(std::ceil(road.length / step)));
    const auto sections = collect_sections(road);

    // Evenly-spaced samples alone can skip a laneSection shorter than `step` entirely (e.g. a brief
    // extra lane spanning under a couple meters), silently dropping any lane that only exists there
    // (its run never reaches the >= 2 points needed below). Every section boundary is sampled twice
    // -- once just before it (still the outgoing section's own lane plan) and once at it (the
    // incoming section's) -- so even a very short section gets at least its own start and end.
    std::vector<double> s_values;
    s_values.reserve(n + 1 + 2 * sections.size());
    for (int i = 0; i <= n; ++i) s_values.push_back(std::min(road.length, road.length * i / n));
    for (std::size_t i = 1; i < sections.size(); ++i) {
        const double boundary = std::clamp(sections[i].s_offset, 0.0, road.length);
        s_values.push_back(std::max(0.0, boundary - 1e-4));
        s_values.push_back(boundary);
    }
    std::sort(s_values.begin(), s_values.end());
    s_values.erase(std::unique(s_values.begin(), s_values.end(),
                                [](const double a, const double b) { return std::abs(a - b) < 1e-9; }),
                   s_values.end());

    // Key: (0=left/1=right, ordinal).
    std::map<std::pair<int, int>, std::vector<WorldPoint>> open_runs;

    for (const double s : s_values) {
        const auto [pos, hdg] = road_point_at_s(road, s);
        const auto& section = applicable_section(sections, s);
        const double local_s = s - section.s_offset;
        const double lane_offset = section.lanes->lane_offset + section.lanes->lane_offset_slope * local_s;
        const geo::Vec2 n_left = geo::left_normal({std::cos(hdg), std::sin(hdg)});

        std::set<std::pair<int, int>> present;
        for (int side = 0; side < 2; ++side) {
            const auto& lanes = side == 0 ? section.lanes->left : section.lanes->right;
            const double sign = side == 0 ? 1.0 : -1.0;
            double inner_acc = 0.0;
            for (int ordinal = 0; ordinal < static_cast<int>(lanes.size()); ++ordinal) {
                const auto& lane = lanes[ordinal];
                double w = lane.width;
                if (lane.width_end >= 0.0) w = lane.width + (lane.width_end - lane.width) * (local_s / section.span);
                // Rail/tram track lanes carry no roadmark (see infer_lanes) -- don't trace a
                // marking line along their boundary either.
                if (lane.type != "rail" && lane.type != "tram") {
                    const double offset = lane_offset + sign * (inner_acc + w);
                    const std::pair<int, int> key{side, ordinal};
                    present.insert(key);
                    open_runs[key].push_back({pos.x + n_left.x * offset, pos.y + n_left.y * offset});
                }
                inner_acc += w;
            }
        }

        for (auto it = open_runs.begin(); it != open_runs.end();) {
            if (present.count(it->first) == 0) {
                if (it->second.size() >= 2) traces.push_back(std::move(it->second));
                it = open_runs.erase(it);
            } else {
                ++it;
            }
        }
    }

    for (auto& [key, pts] : open_runs) {
        if (pts.size() >= 2) traces.push_back(std::move(pts));
    }
    return traces;
}

std::vector<std::vector<WorldPoint>> sample_stop_lines(const model::RoadSegment& road) {
    std::vector<std::vector<WorldPoint>> traces;
    if (road.length <= 1e-6) return traces;
    const auto sections = collect_sections(road);

    for (const auto& sig : road.signals) {
        if (sig.type != "stop_line") continue;
        const double s = std::clamp(sig.s, 0.0, road.length);
        const auto [pos, hdg] = road_point_at_s(road, s);
        const auto& section = applicable_section(sections, s);
        const double local_s = s - section.s_offset;
        const double lane_offset = section.lanes->lane_offset + section.lanes->lane_offset_slope * local_s;
        const geo::Vec2 n_left = geo::left_normal({std::cos(hdg), std::sin(hdg)});

        // Prefer whichever side sig.t actually sits on, but a signal near the reference line
        // (t ~ 0, common on a oneway road where only one side has lanes at all) can round to the
        // empty side -- fall back to whichever side actually has lanes instead of drawing nothing.
        const double left_extent = side_extent(section.lanes->left, local_s, section.span);
        const double right_extent = side_extent(section.lanes->right, local_s, section.span);
        const bool prefer_left = sig.t >= 0.0;
        const bool left_side = prefer_left ? left_extent > 1e-6 : !(right_extent > 1e-6);
        const double extent = left_side ? left_extent : right_extent;
        if (extent <= 1e-6) continue; // no lanes on either side to draw a stop line across
        const double inner_offset = lane_offset;
        const double outer_offset = lane_offset + (left_side ? extent : -extent);
        traces.push_back({{pos.x + n_left.x * inner_offset, pos.y + n_left.y * inner_offset},
                           {pos.x + n_left.x * outer_offset, pos.y + n_left.y * outer_offset}});
    }
    return traces;
}

PreviewGeometry build_preview_geometry(const model::MapModel& model, double step) {
    PreviewGeometry out;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& road : model.roads) {
        RoadTraces traces = sample_road(road, step);
        if (traces.centerline.size() < 2) continue;
        for (const auto& p : traces.centerline) expand_bounds(p, min_x, min_y, max_x, max_y);
        for (const auto& p : traces.left_edge) expand_bounds(p, min_x, min_y, max_x, max_y);
        for (const auto& p : traces.right_edge) expand_bounds(p, min_x, min_y, max_x, max_y);
        out.roads.push_back(std::move(traces));

        const bool is_junction = !road.junction_id.empty();
        for (auto& marking : sample_lane_markings(road, step)) {
            for (const auto& p : marking) expand_bounds(p, min_x, min_y, max_x, max_y);
            out.lane_markings.push_back({std::move(marking), is_junction});
        }

        for (auto& stop_line : sample_stop_lines(road)) {
            for (const auto& p : stop_line) expand_bounds(p, min_x, min_y, max_x, max_y);
            out.stop_lines.push_back(std::move(stop_line));
        }
    }

    if (!out.roads.empty()) {
        out.min_x = min_x;
        out.min_y = min_y;
        out.max_x = max_x;
        out.max_y = max_y;
    }
    return out;
}

} // namespace osm2xodr::gui
