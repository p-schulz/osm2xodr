#include "xodr_preview.hpp"

#include <OpenDriveMap.h>

#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

namespace osm2xodr::gui {
namespace {

WorldPoint to_world(const odr::Vec3D& v) { return {v[0], v[1]}; }

bool is_track_type(const std::string& type) { return type == "rail" || type == "tram"; }

void expand_bounds(const WorldPoint& p, double& min_x, double& min_y, double& max_x, double& max_y) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
}

// Outermost lane on each side of one laneSection -- "outermost" meaning the full lane stack's
// outer edge (sidewalk/curb/median/border lanes included, exactly as xodr_writer.cpp emits them
// as real <lane> elements, not just the outermost driving lane), or null if that side has none.
struct SideLanes {
    const odr::Lane* left = nullptr;
    const odr::Lane* right = nullptr;
};

SideLanes outer_lanes(const std::vector<odr::Lane>& lanes) {
    SideLanes out;
    for (const auto& lane : lanes) {
        if (lane.id > 0 && (!out.left || lane.id > out.left->id)) out.left = &lane;
        if (lane.id < 0 && (!out.right || lane.id < out.right->id)) out.right = &lane;
    }
    return out;
}

// One side's outer edge for a single laneSection, world-space, `step` meters apart: the real
// lane-stack border when that side has lanes, else the reference line itself (already offset by
// the road's own lane_offset) -- so a one-sided road's empty side still traces a sensible line
// instead of being left out of the polyline entirely.
std::vector<WorldPoint> side_edge_points(const odr::Road& road, const odr::LaneSection& section, const odr::Lane* outer, double step) {
    std::vector<WorldPoint> pts;
    if (outer) {
        for (const auto& p : road.get_lane_border_line(*outer, step, /*outer=*/true)) pts.push_back(to_world(p));
        return pts;
    }
    const double s0 = section.s0;
    const double s1 = road.get_lanesection_end(section);
    for (const auto& p : road.ref_line.get_line(s0, s1, step)) pts.push_back(to_world(p));
    return pts;
}

// xodr_writer.cpp's <geoReference> is "+proj=eqc +lat_ts=<lat> +lat_0=<lat> +lon_0=<lon> ...";
// pull lat_0/lon_0 back out so a file opened this way can still fetch aerial imagery.
bool parse_origin_from_proj4(const std::string& proj4, double& lat, double& lon) {
    double lat0 = 0.0, lon0 = 0.0;
    const std::size_t lat_pos = proj4.find("+lat_0=");
    const std::size_t lon_pos = proj4.find("+lon_0=");
    if (lat_pos == std::string::npos || lon_pos == std::string::npos) return false;
    if (std::sscanf(proj4.c_str() + lat_pos, "+lat_0=%lf", &lat0) != 1) return false;
    if (std::sscanf(proj4.c_str() + lon_pos, "+lon_0=%lf", &lon0) != 1) return false;
    lat = lat0;
    lon = lon0;
    return true;
}

} // namespace

XodrPreviewResult build_preview_geometry_from_xodr(const std::string& path, double step) {
    odr::OpenDriveMap map(path, /*center_map=*/false, /*with_road_objects=*/false,
                           /*with_lateral_profile=*/true, /*with_lane_height=*/false,
                           /*abs_z_for_for_local_road_obj_outline=*/false,
                           /*fix_spiral_edge_cases=*/true, /*with_road_signals=*/true);

    XodrPreviewResult result;
    result.has_origin = parse_origin_from_proj4(map.proj4, result.origin_lat, result.origin_lon);

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    const auto roads = map.get_roads();
    result.road_count = static_cast<int>(roads.size());
    result.junction_count = static_cast<int>(map.get_junctions().size());

    for (const auto& road : roads) {
        if (road.length <= 1e-6) continue;

        RoadTraces traces;
        for (const auto& p : road.ref_line.get_line(0.0, road.length, step)) traces.centerline.push_back(to_world(p));

        const auto sections = road.get_lanesections();
        for (const auto& section : sections) {
            const auto lanes = section.get_lanes();
            for (const auto& lane : lanes) {
                if (lane.id != 0 && is_track_type(lane.type)) traces.is_railway = true;
            }
            const SideLanes sides = outer_lanes(lanes);
            for (const auto& p : side_edge_points(road, section, sides.left, step)) traces.left_edge.push_back(p);
            for (const auto& p : side_edge_points(road, section, sides.right, step)) traces.right_edge.push_back(p);
        }

        if (traces.centerline.size() < 2) continue;
        for (const auto& p : traces.centerline) expand_bounds(p, min_x, min_y, max_x, max_y);
        for (const auto& p : traces.left_edge) expand_bounds(p, min_x, min_y, max_x, max_y);
        for (const auto& p : traces.right_edge) expand_bounds(p, min_x, min_y, max_x, max_y);
        result.geometry.roads.push_back(std::move(traces));

        const bool is_junction = !road.junction.empty() && road.junction != "-1";
        for (const auto& section : sections) {
            for (const auto& lane : section.get_lanes()) {
                if (lane.id == 0 || is_track_type(lane.type)) continue;
                LaneMarkingTrace marking;
                marking.is_junction = is_junction;
                for (const auto& p : road.get_lane_border_line(lane, step, /*outer=*/true)) marking.points.push_back(to_world(p));
                if (marking.points.size() < 2) continue;
                for (const auto& p : marking.points) expand_bounds(p, min_x, min_y, max_x, max_y);
                result.geometry.lane_markings.push_back(std::move(marking));
            }
        }

        for (const auto& sig : road.get_road_signals()) {
            if (sig.type != "stop_line") continue;
            const double s = std::clamp(sig.s0, 0.0, road.length);
            const auto lanes_here = road.get_lanesection(s).get_lanes();
            const SideLanes sides = outer_lanes(lanes_here);
            const bool prefer_left = sig.t0 >= 0.0;
            const bool left_side = prefer_left ? (sides.left != nullptr) : !(sides.right != nullptr);
            const odr::Lane* outer = left_side ? sides.left : sides.right;
            if (!outer) continue; // no lanes on either side to draw a stop line across

            const double t_inner = road.lane_offset.evaluate(s);
            const double t_outer = outer->outer_border.evaluate(s);
            std::vector<WorldPoint> line{to_world(road.get_surface_pt(s, t_inner)), to_world(road.get_surface_pt(s, t_outer))};
            expand_bounds(line[0], min_x, min_y, max_x, max_y);
            expand_bounds(line[1], min_x, min_y, max_x, max_y);
            result.geometry.stop_lines.push_back(std::move(line));
        }
    }

    if (!result.geometry.roads.empty()) {
        result.geometry.min_x = min_x;
        result.geometry.min_y = min_y;
        result.geometry.max_x = max_x;
        result.geometry.max_y = max_y;
    }
    return result;
}

} // namespace osm2xodr::gui
