#pragma once

#include "osm2xodr/geo.hpp"
#include "osm2xodr/tags.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace osm2xodr::model {

struct LaneSpec {
    int id = 0;
    std::string type = "driving";
    double width = 3.5;
    double width_end = -1.0; // if >= 0, width tapers linearly from `width` (s=0) to `width_end` (s=length)
    std::string roadmark_type = "broken";
    std::string roadmark_weight = "standard";
    std::string roadmark_color = "standard";
    std::string lane_change = "both";
    std::optional<int> link_predecessor_id;
    std::optional<int> link_successor_id;
    std::vector<std::string> turn_directions; // from OSM turn:lanes; empty = unrestricted
};

struct LanePlan {
    int forward_lanes = 1;
    int backward_lanes = 1;
    bool oneway = false;
    bool left_sidewalk = false;
    bool right_sidewalk = false;
    double lane_offset = 0.0;
    double lane_offset_slope = 0.0; // "b" coefficient for laneOffset: nonzero for junction connectors
                                    // (taper between differing in/out lane widths) and for a merged
                                    // road's lane-split/merge taper sections (see build::fuse_chain).
    std::vector<LaneSpec> left;   // positive lane ids, center outward
    std::vector<LaneSpec> right;  // negative lane ids, center outward
    std::string center_mark = "broken";
};

// Sums one physical side's lane widths, at the start (`at_end=false`) or end (`at_end=true`) of
// whatever span those widths apply over -- for a lane with no taper (width_end < 0), start and end
// are the same (`width`).
inline double sum_side_width(const std::vector<LaneSpec>& side, const bool at_end) {
    double total = 0.0;
    for (const auto& lane : side) total += (at_end && lane.width_end >= 0.0) ? lane.width_end : lane.width;
    return total;
}

// The reference line is centered on the road's total cross-section: an OSM way's polyline (the
// planView geometry) is the *centerline* of everything the way represents, not the edge of one
// lane, so the laneOffset must place the reference line at (right_total - left_total)/2 for the
// left/right stacks to come out symmetric around it. When any lane's width tapers (a lane
// split/merge -- see build::fuse_chain) the total, and therefore this offset, is only constant if
// evaluated consistently at the same point of the taper; `at_end` selects which.
inline double compute_lane_offset(const LanePlan& plan, const bool at_end = false) {
    return (sum_side_width(plan.right, at_end) - sum_side_width(plan.left, at_end)) / 2.0;
}

enum class GeomKind { Line, Arc, ParamPoly3 };

// A single OpenDRIVE <geometry> primitive: a line, a constant-curvature arc, or a cubic paramPoly3.
// `curvature` is meaningful only for Arc. `local_p1/p2/p3` are meaningful only for ParamPoly3: the
// cubic's 4 Bezier control points in the primitive's own local frame (rotated by hdg, origin
// (x,y)); the first control point is implicitly (0,0) and not stored. Storing Bezier control
// points rather than raw paramPoly3 polynomial coefficients means trimming a curve to a
// sub-interval is exact de Casteljau subdivision instead of re-deriving shifted-domain
// coefficients; power-basis conversion happens only once, at XML-write time.
struct GeomPrimitive {
    double x = 0.0;
    double y = 0.0;
    double hdg = 0.0;
    double length = 0.0;
    double curvature = 0.0;
    GeomKind kind = GeomKind::Line;
    geo::Vec2 local_p1{};
    geo::Vec2 local_p2{};
    geo::Vec2 local_p3{};
};

struct RoadSignal {
    std::string id;
    double s = 0.0;
    double t = 0.0;
    bool dynamic = false;
    std::string name;
    std::string country;
    std::string type;
    std::string subtype;
    std::string orientation = "+";
    std::string unit;
    std::string text;
    double value = 0.0;
    bool has_value = false;
};

// An additional lane cross-section (and OSM tag set) starting partway along a merged road. Roads
// that were not produced by merging never populate this; `RoadSegment::lanes`/`tags` always
// describe the section starting at s=0.
struct LaneSection {
    double s_offset = 0.0;
    LanePlan lanes;
    Tags tags;
    std::int64_t source_way_id = 0;
    int segment_index = 0;
};

struct RoadSegment {
    std::string id;
    std::int64_t source_way_id = 0;
    int segment_index = 0;
    Tags tags;
    std::vector<std::int64_t> refs;
    std::vector<geo::Vec2> points;
    double length = 0.0;
    LanePlan lanes;
    std::int64_t start_ref = 0;
    std::int64_t end_ref = 0;
    std::string predecessor_xml;
    std::string successor_xml;
    std::vector<RoadSignal> signals;
    std::string junction_id;                    // non-empty for synthetic junction-connector roads
    std::vector<GeomPrimitive> explicit_geometry; // if non-empty, overrides `points` for planView output
    std::vector<LaneSection> extra_lane_sections; // additional s>0 laneSection/type boundaries from merging
};

struct EndpointRef {
    std::size_t road_index = 0;
    bool at_start = false;
};

struct JunctionConnection {
    std::string id;
    std::string incoming_road;
    std::string connecting_road;
    std::string contact_point;
    std::vector<std::pair<int, int>> lane_links;
};

struct Junction {
    std::string id;
    std::int64_t node_ref = 0;
    geo::Vec2 point{};
    std::vector<JunctionConnection> connections;
};

struct MapModel {
    std::vector<RoadSegment> roads;
    std::vector<Junction> junctions;
    std::vector<std::string> warnings;
    geo::LocalProjector projector;
    double north = -std::numeric_limits<double>::max();
    double south = std::numeric_limits<double>::max();
    double east = -std::numeric_limits<double>::max();
    double west = std::numeric_limits<double>::max();
    int compound_junction_count = 0;      // junctions formed by merging >1 OSM junction node
    int compound_junction_node_total = 0; // total OSM junction nodes folded into those junctions
};

} // namespace osm2xodr::model
