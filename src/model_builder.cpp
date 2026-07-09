#include "osm2xodr/model_builder.hpp"

#include "osm2xodr/infer.hpp"
#include "osm2xodr/tags.hpp"
#include "osm2xodr/util.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace osm2xodr::build {

std::vector<std::size_t> split_indices_for_way(const osm::RawWay& way, const std::unordered_set<std::int64_t>& split_nodes) {
    std::vector<std::size_t> indices;
    indices.push_back(0);
    for (std::size_t i = 1; i + 1 < way.nodes.size(); ++i) {
        if (split_nodes.count(way.nodes[i].ref)) indices.push_back(i);
    }
    indices.push_back(way.nodes.size() - 1);
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    return indices;
}

std::string road_id_for(const std::int64_t way_id, const int segment_index) {
    return "w" + std::to_string(way_id) + "_" + std::to_string(segment_index);
}

struct EndpointKeyHash {
    std::size_t operator()(const std::int64_t v) const noexcept { return std::hash<std::int64_t>{}(v); }
};

bool road_has_drive_lane(const model::RoadSegment& road, const bool at_start, const int lane_id) {
    const auto& lanes = at_start || road.extra_lane_sections.empty() ? road.lanes : road.extra_lane_sections.back().lanes;
    const auto& side = lane_id > 0 ? lanes.left : lanes.right;
    return std::any_of(side.begin(), side.end(), [&](const model::LaneSpec& l) { return l.id == lane_id && l.type == "driving"; });
}

std::vector<int> incoming_lane_ids(const model::RoadSegment& road, const bool at_start, const Options& options) {
    const auto& lanes = at_start || road.extra_lane_sections.empty() ? road.lanes : road.extra_lane_sections.back().lanes;
    std::vector<int> ids;
    if (!options.left_hand_traffic) {
        const auto& side = at_start ? lanes.left : lanes.right;
        for (const auto& lane : side) if (lane.type == "driving") ids.push_back(lane.id);
    } else {
        const auto& side = at_start ? lanes.right : lanes.left;
        for (const auto& lane : side) if (lane.type == "driving") ids.push_back(lane.id);
    }
    return ids;
}

std::vector<int> outgoing_lane_ids(const model::RoadSegment& road, const bool at_start, const Options& options) {
    const auto& lanes = at_start || road.extra_lane_sections.empty() ? road.lanes : road.extra_lane_sections.back().lanes;
    std::vector<int> ids;
    if (!options.left_hand_traffic) {
        const auto& side = at_start ? lanes.right : lanes.left;
        for (const auto& lane : side) if (lane.type == "driving") ids.push_back(lane.id);
    } else {
        const auto& side = at_start ? lanes.left : lanes.right;
        for (const auto& lane : side) if (lane.type == "driving") ids.push_back(lane.id);
    }
    return ids;
}

std::string make_road_link_xml(const std::string& element_type, const std::string& element_id, const std::string& contact_point) {
    std::ostringstream ss;
    ss << util::attr("elementType", element_type)
       << util::attr("elementId", element_id);
    if (!contact_point.empty()) ss << util::attr("contactPoint", contact_point);
    return ss.str();
}

std::string contact_point_of(const bool at_start) { return at_start ? std::string("start") : std::string("end"); }

// ---- Junction connector geometry helpers -----------------------------------------------------
//
// At a junction node, each road end has a "role" (incoming: traffic enters the junction along it;
// outgoing: traffic leaves the junction along it) independent of which physical lane is used.
// direction_into_junction / direction_away_from_junction give that role's direction of travel;
// forward_s_direction_at_end gives the road's own +s direction at that end (used only to place
// individual lanes, which are constant lateral offsets from the reference line).

geo::Vec2 endpoint_point(const model::RoadSegment& road, const bool at_start) {
    return at_start ? road.points.front() : road.points.back();
}

geo::Vec2 forward_s_direction_at_end(const model::RoadSegment& road, const bool at_start) {
    const auto& pts = road.points;
    return at_start ? geo::normalize(pts[1] - pts[0]) : geo::normalize(pts.back() - pts[pts.size() - 2]);
}

geo::Vec2 direction_away_from_junction(const model::RoadSegment& road, const bool at_start) {
    const auto& pts = road.points;
    const geo::Vec2 d = at_start ? (pts[1] - pts[0]) : (pts[pts.size() - 2] - pts.back());
    return geo::normalize(d);
}

geo::Vec2 direction_into_junction(const model::RoadSegment& road, const bool at_start) {
    const auto d = direction_away_from_junction(road, at_start);
    return {-d.x, -d.y};
}

// Point reached after walking `distance` along a polyline from one end (clamped to the polyline's
// own length).
geo::Vec2 point_along_polyline(const std::vector<geo::Vec2>& pts, const bool from_start, const double distance) {
    std::vector<geo::Vec2> ordered = pts;
    if (!from_start) std::reverse(ordered.begin(), ordered.end());
    double remaining = distance;
    for (std::size_t i = 0; i + 1 < ordered.size(); ++i) {
        const double seg = geo::length(ordered[i + 1] - ordered[i]);
        if (seg >= remaining) return ordered[i] + geo::normalize(ordered[i + 1] - ordered[i]) * remaining;
        remaining -= seg;
    }
    return ordered.back();
}

// direction_away_from_junction/direction_into_junction (the immediate first-micro-segment tangent)
// are exactly what a connector's own geometry must match for position/heading continuity at the
// seam, so they stay as-is for that purpose. But that same immediate tangent badly misrepresents a
// road that keeps curving well beyond its very first sub-segment (a mapped ramp/slip-lane with
// several short internal segments approximating a real curve) -- classifying the movement's
// through/left/right bucket from only the first ~1m of a 30m curve can call a road that ends up
// turning 90+ degrees "through" simply because it started out nearly straight. Using the chord to
// a point further down the road (capped at `lookahead`, and at the road's own length so this never
// reaches into a *different* <road> across a further boundary) gives classification a much more
// representative sense of where the movement actually goes, without touching the connector geometry
// itself.
geo::Vec2 classification_direction_away_from_junction(const model::RoadSegment& road, const bool at_start, const double lookahead) {
    const auto origin = endpoint_point(road, at_start);
    const auto target = point_along_polyline(road.points, at_start, std::min(lookahead, road.length));
    const auto d = target - origin;
    if (geo::length(d) < 1e-6) return direction_away_from_junction(road, at_start);
    return geo::normalize(d);
}

geo::Vec2 classification_direction_into_junction(const model::RoadSegment& road, const bool at_start, const double lookahead) {
    const auto d = classification_direction_away_from_junction(road, at_start, lookahead);
    return {-d.x, -d.y};
}

double segment_length_at_end(const model::RoadSegment& road, const bool at_start) {
    const auto& pts = road.points;
    return at_start ? geo::length(pts[1] - pts[0]) : geo::length(pts.back() - pts[pts.size() - 2]);
}

// ---- Curve fitting for non-junction road planView geometry -----------------------------------
//
// Ordinary roads are otherwise emitted as one <line> per consecutive pair of points (see
// xodr_writer's write_plan_view), heading-discontinuous at every original OSM node. This fits a
// cubic Bezier (written as OpenDRIVE <paramPoly3>) through each consecutive pair instead, using a
// Catmull-Rom-style tangent at each point so consecutive pieces are heading-continuous -- while
// still passing through every original point exactly (an interpolating, not approximating, fit),
// so no existing s-offset-based bookkeeping (lane sections, signals, junction-connector trim
// budgets) needs to change: every <geometry> here keeps the same x/y/hdg/length already computed
// for a plain line at that position, and the curve's own endpoint at parameter p=1 is always
// exactly control point P3 by construction, regardless of any tiny difference between a curve's
// true arc length and the declared (chord) length.
//
// Endpoint tangents are deliberately pinned to the exact same directions
// direction_away_from_junction/direction_into_junction already use (the immediate first/last
// micro-segment), not a smoothed/averaged tangent -- junction connectors and lane-count bridges
// size themselves against those directions and are not touched by this feature, so a road's fitted
// curve must end with the same tangent they already assume, or the seam would gain a new
// discontinuity in exchange for removing the old ones.
std::vector<geo::Vec2> catmull_rom_tangents(const std::vector<geo::Vec2>& points) {
    const std::size_t n = points.size();
    std::vector<geo::Vec2> tangents(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (i == 0) tangents[i] = geo::normalize(points[1] - points[0]);
        else if (i + 1 == n) tangents[i] = geo::normalize(points[n - 1] - points[n - 2]);
        else tangents[i] = geo::normalize(points[i + 1] - points[i - 1]);
    }
    return tangents;
}

// Builds a single GeomPrimitive from p0 (with travel heading tangent_in there) to p3 (with travel
// heading tangent_out there): a plain Line when both tangents already point exactly along the
// chord (a Bezier would degenerate to that same line anyway), else a ParamPoly3 from a standard
// 1/3-rule Hermite-to-Bezier conversion. Shared by fit_curve (consecutive points of one road,
// tangents from catmull_rom_tangents) and build_junction_connectors' own direct-bridge fallback
// (two arbitrary lane endpoints with two independently-required headings, where a single <line>
// -- one fixed heading -- cannot match both ends the way this can).
model::GeomPrimitive hermite_bezier_segment(const geo::Vec2& p0, const geo::Vec2& tangent_in,
                                             const geo::Vec2& p3, const geo::Vec2& tangent_out) {
    const geo::Vec2 chord = p3 - p0;
    const double length = geo::length(chord);
    const double hdg = length > 1e-9 ? std::atan2(chord.y, chord.x) : std::atan2(tangent_in.y, tangent_in.x);

    model::GeomPrimitive g;
    g.x = p0.x;
    g.y = p0.y;
    g.hdg = hdg;
    g.length = length;
    g.curvature = 0.0;
    g.kind = model::GeomKind::Line;
    if (length <= 1e-9) return g;

    // Both tangents already exactly along the chord (always true for a 2-point road, whose only
    // two tangents are both forced to equal the chord direction by construction): a Bezier here
    // would degenerate to this same line anyway, so just keep it. Deliberately an exact-equality
    // threshold, not a "close enough" one -- a <line>'s heading is a single fixed value, but a
    // neighboring segment (in fit_curve's multi-point case) uses this exact same shared tangent
    // value at the boundary, so any looser per-segment straightness threshold breaks exact
    // continuity right at the boundary where one side simplifies and the other doesn't (confirmed
    // by test/check_road_geometry_continuity.py during development: a 0.5-degree threshold left
    // ~0.2-0.7 degree residuals at exactly those boundaries). A near-straight segment still gets a
    // real ParamPoly3; its control points just end up very close to the chord, which is harmless.
    const geo::Vec2 chord_dir = chord * (1.0 / length);
    if (geo::dot(tangent_in, chord_dir) > 1.0 - 1e-12 && geo::dot(tangent_out, chord_dir) > 1.0 - 1e-12) {
        return g;
    }

    const geo::Vec2 p1 = p0 + tangent_in * (length / 3.0);
    const geo::Vec2 p2 = p3 - tangent_out * (length / 3.0);
    const double cos_h = std::cos(hdg), sin_h = std::sin(hdg);
    auto to_local = [&](const geo::Vec2& p) {
        const geo::Vec2 d = p - p0;
        return geo::Vec2{d.x * cos_h + d.y * sin_h, -d.x * sin_h + d.y * cos_h};
    };
    g.kind = model::GeomKind::ParamPoly3;
    g.local_p1 = to_local(p1);
    g.local_p2 = to_local(p2);
    g.local_p3 = to_local(p3);
    return g;
}

// One GeomPrimitive per consecutive pair of `points`, via hermite_bezier_segment above using the
// Catmull-Rom tangents at each point.
std::vector<model::GeomPrimitive> fit_curve(const std::vector<geo::Vec2>& points) {
    std::vector<model::GeomPrimitive> geoms;
    if (points.size() < 2) return geoms;
    const auto tangents = catmull_rom_tangents(points);
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        if (geo::length(points[i + 1] - points[i]) <= 1e-6) continue;
        geoms.push_back(hermite_bezier_segment(points[i], tangents[i], points[i + 1], tangents[i + 1]));
    }
    return geoms;
}

// A merged road's own `.tags`/`.lanes` only describe its first (s=0) cross-section; a junction at
// the road's *end* must be tiered/typed/measured using whichever section actually reaches that
// end, not always section 0.
const Tags& tags_at_end(const model::RoadSegment& road, const bool at_start) {
    if (at_start || road.extra_lane_sections.empty()) return road.tags;
    return road.extra_lane_sections.back().tags;
}

const model::LanePlan& lanes_at_end(const model::RoadSegment& road, const bool at_start) {
    if (at_start || road.extra_lane_sections.empty()) return road.lanes;
    return road.extra_lane_sections.back().lanes;
}

model::LanePlan& lanes_at_end_mut(model::RoadSegment& road, const bool at_start) {
    if (at_start || road.extra_lane_sections.empty()) return road.lanes;
    return road.extra_lane_sections.back().lanes;
}

// The actual laneOffset value at a specific end of a road: at s=0 (at_start) this is simply
// LanePlan::lane_offset, but at the road's far end it's whatever the *last* section's offset
// polynomial evaluates to at that section's own local span (lane_offset + slope * local_length),
// not the raw `lane_offset` field, which is that section's value at its own s=0.
double lane_offset_at_road_end(const model::RoadSegment& road, const bool at_start) {
    if (at_start) return road.lanes.lane_offset;
    if (road.extra_lane_sections.empty()) return road.lanes.lane_offset + road.lanes.lane_offset_slope * road.length;
    const auto& last = road.extra_lane_sections.back();
    return last.lanes.lane_offset + last.lanes.lane_offset_slope * (road.length - last.s_offset);
}

const model::LaneSpec* lane_spec_for(const model::RoadSegment& road, const bool at_start, const int lane_id) {
    const auto& lanes = lanes_at_end(road, at_start);
    for (const auto& lane : lanes.left) if (lane.id == lane_id) return &lane;
    for (const auto& lane : lanes.right) if (lane.id == lane_id) return &lane;
    return nullptr;
}

// Classifies a junction movement's geometric direction from the signed angle already computed for
// the fillet math (positive = left, matching geo's CCW-positive convention), so it can be matched
// against a lane's OSM turn:lanes permission set. Thresholds are a reasonable engineering default,
// not derived from any spec.
std::string turn_bucket_for_delta(const double signed_delta_rad) {
    const double deg = signed_delta_rad * 180.0 / geo::kPi;
    if (deg > 135.0) return "sharp_left";
    if (deg > 45.0) return "left";
    if (deg > 20.0) return "slight_left";
    if (deg >= -20.0) return "through";
    if (deg >= -45.0) return "slight_right";
    if (deg >= -135.0) return "right";
    return "sharp_right";
}

// True if `lane_id` (an incoming lane) is permitted to make a movement classified as `bucket`.
// A lane with no parsed turn:lanes data is unrestricted (matches every movement, today's
// behavior); merge_to_left/merge_to_right alias to the slight_* buckets, and reverse aliases to
// whichever sharp_* bucket matches the movement's own sign (rare token, not expected in practice).
bool lane_allows_movement(const model::RoadSegment& road, const bool at_start, const int lane_id,
                           const std::string& bucket, const double signed_delta_rad) {
    const auto* lane = lane_spec_for(road, at_start, lane_id);
    if (!lane || lane->turn_directions.empty()) return true;
    for (const auto& token : lane->turn_directions) {
        if (token == bucket) return true;
        if (token == "merge_to_left" && bucket == "slight_left") return true;
        if (token == "merge_to_right" && bucket == "slight_right") return true;
        // OSM's turn:lanes vocabulary distinguishes left/slight_left/sharp_left (and the right
        // equivalents) as separate values, but most mappers only bother with the plain "left"/
        // "right" even for a moderate bend, reserving "slight_*"/"sharp_*" for cases they consider
        // notably gentle or sharp. Treating plain left/right as also covering the immediately
        // adjacent slight_* bucket (not sharp_*, which is a large enough deviation that a mapper
        // choosing not to say "sharp" is meaningful) avoids losing a real, tagged turn lane's only
        // destination just because its geometry resolves a little short of a "full" turn.
        if (token == "left" && bucket == "slight_left") return true;
        if (token == "right" && bucket == "slight_right") return true;
        if (token == "reverse" && (signed_delta_rad >= 0.0 ? bucket == "sharp_left" : bucket == "sharp_right")) return true;
    }
    return false;
}

// Signed lateral distance of a lane's centerline from the road's reference line (road.points),
// positive = left of the road's own +s direction, matching OpenDRIVE's t-axis convention.
double lane_lateral_offset(const model::RoadSegment& road, const bool at_start, const int lane_id) {
    const auto& lanes = lanes_at_end(road, at_start);
    if (lane_id > 0) {
        double acc = 0.0;
        for (const auto& lane : lanes.left) {
            if (lane.id == lane_id) return lanes.lane_offset + acc + lane.width / 2.0;
            acc += lane.width;
        }
    } else if (lane_id < 0) {
        double acc = 0.0;
        for (const auto& lane : lanes.right) {
            if (lane.id == lane_id) return lanes.lane_offset - (acc + lane.width / 2.0);
            acc += lane.width;
        }
    }
    return lanes.lane_offset;
}

double lane_width_of(const model::RoadSegment& road, const bool at_start, const int lane_id) {
    const auto& lanes = lanes_at_end(road, at_start);
    for (const auto& lane : lanes.left) if (lane.id == lane_id) return lane.width;
    for (const auto& lane : lanes.right) if (lane.id == lane_id) return lane.width;
    return 3.5;
}

geo::Vec2 lane_world_point(const model::RoadSegment& road, const bool at_start, const int lane_id) {
    const auto p = endpoint_point(road, at_start);
    const auto n = geo::left_normal(forward_s_direction_at_end(road, at_start));
    const double off = lane_lateral_offset(road, at_start, lane_id);
    return {p.x + n.x * off, p.y + n.y * off};
}

std::size_t endpoint_key(const std::size_t road_index, const bool at_start) {
    return road_index * 2 + (at_start ? 1 : 0);
}

// Shortens a polyline by `distance` measured from one end, replacing the cut segment with an
// interpolated point. Used to pull road ends back from a junction to make room for a connector.
std::vector<geo::Vec2> trim_polyline(const std::vector<geo::Vec2>& pts, const bool from_start, const double distance) {
    if (distance <= 1e-6 || pts.size() < 2) return pts;
    std::vector<geo::Vec2> ordered = pts;
    if (!from_start) std::reverse(ordered.begin(), ordered.end());

    double remaining = distance;
    std::size_t i = 0;
    while (i + 1 < ordered.size()) {
        const double seg = geo::length(ordered[i + 1] - ordered[i]);
        if (seg >= remaining) {
            const auto dir = geo::normalize(ordered[i + 1] - ordered[i]);
            const geo::Vec2 new_point = ordered[i] + dir * remaining;
            std::vector<geo::Vec2> result;
            result.push_back(new_point);
            for (std::size_t k = i + 1; k < ordered.size(); ++k) result.push_back(ordered[k]);
            if (!from_start) std::reverse(result.begin(), result.end());
            return result;
        }
        remaining -= seg;
        ++i;
    }
    std::vector<geo::Vec2> result{ordered.back()};
    if (!from_start) std::reverse(result.begin(), result.end());
    return result;
}

// Shortens `road` by `applied` meters from one end (see trim_polyline), rebasing whatever carries
// an s-coordinate relative to the road's own s=0 so it stays correct after the cut: trimming the
// start shifts every signal/extra_lane_section's s left by `applied` (and promotes whichever
// section that shift now puts at s<=0 to become the road's own s=0 section/tags, dropping it from
// extra_lane_sections); trimming the end just drops any extra_lane_sections boundary that's now
// beyond the new (shorter) length. Shared by build_junction_connectors (trimming an approach road
// back to make room for a connector) and plan_lane_count_bridge (trimming back to make room for a
// plain-boundary lane-count bridge) -- the same operation either way.
void apply_end_trim(model::RoadSegment& road, const bool at_start, const double applied) {
    if (applied <= 1e-6) return;
    road.points = trim_polyline(road.points, at_start, applied);
    road.length = geo::polyline_length(road.points);

    if (at_start) {
        for (auto& sig : road.signals) sig.s -= applied;
        for (auto& section : road.extra_lane_sections) section.s_offset -= applied;
        std::size_t consumed = 0;
        while (consumed < road.extra_lane_sections.size() && road.extra_lane_sections[consumed].s_offset <= 1e-6) ++consumed;
        if (consumed > 0) {
            road.lanes = road.extra_lane_sections[consumed - 1].lanes;
            road.tags = road.extra_lane_sections[consumed - 1].tags;
            road.extra_lane_sections.erase(road.extra_lane_sections.begin(), road.extra_lane_sections.begin() + consumed);
        }
    } else {
        while (!road.extra_lane_sections.empty() && road.extra_lane_sections.back().s_offset >= road.length - 1e-6) {
            road.extra_lane_sections.pop_back();
        }
    }
}

// Evaluates a point at arc-length `s` along a single planView geometry primitive (line or arc).
geo::Vec2 evaluate_geometry_point(const model::GeomPrimitive& g, const double s) {
    if (std::abs(g.curvature) < 1e-9) {
        return {g.x + std::cos(g.hdg) * s, g.y + std::sin(g.hdg) * s};
    }
    const double r = 1.0 / g.curvature;
    const double theta = g.curvature * s;
    const double cx = g.x - r * std::sin(g.hdg);
    const double cy = g.y + r * std::cos(g.hdg);
    return {cx + r * std::sin(g.hdg + theta), cy - r * std::cos(g.hdg + theta)};
}

// A candidate connector between one incoming lane and one outgoing lane at a junction. Built in
// two passes: first every candidate across every junction is evaluated to find out how much each
// touched road end must be trimmed back (`b_in`/`b_out`, the tangent-fillet setback distances);
// then, once trims are finalized, the actual connector road geometry is emitted.
struct PendingConnector {
    std::size_t in_road_index = 0;
    bool in_at_start = false;
    std::size_t out_road_index = 0;
    bool out_at_start = false;
    int incoming_lane_id = 0;
    int outgoing_lane_id = 0;
    geo::Vec2 dir_in{};
    geo::Vec2 dir_out{};
    geo::Vec2 a_in{};
    geo::Vec2 a_out{};
    double radius = 0.0;
    double signed_delta = 0.0;
    double b_in = 0.0;
    double b_out = 0.0;
    bool feasible = false;
    std::string junction_id;
    std::int64_t node_ref = 0;
    geo::Vec2 junction_point{};
};

// ---- Road merging -----------------------------------------------------------------------------
//
// Fuses chains of RoadSegments connected at plain pass-through nodes (exactly two road ends touch
// the node, it's not a junction, and it isn't a forced traffic-light/stop/give-way split) into a
// single OpenDRIVE <road>, even when OSM tags change along the chain. Runs once, right after the
// initial per-way fragment build and before junction detection/connectors, over the *raw* OSM
// point chain; endpoint_map/junction_nodes are rebuilt from scratch afterward since road indices
// change.

bool lane_spec_differs(const model::LaneSpec& a, const model::LaneSpec& b) {
    // turn_directions matters here for the same reason type/width/roadmark do: a merge boundary
    // where a lane's OSM turn:lanes-derived restriction changes (including appearing or
    // disappearing) must get its own LaneSection, or the merged road's single LanePlan silently
    // carries whichever side happened to be its first section's restriction across the whole
    // chain -- either extending it past the way it was actually tagged on, or discarding a later
    // way's own restriction entirely. Since this triggers the plain "no lane-count change" branch
    // below (no width difference implied), it's a zero-geometry-impact section split: it only
    // rescopes which lanes carry which turn_directions and where per-lane links point.
    return a.type != b.type || std::abs(a.width - b.width) > 1e-6 || a.roadmark_type != b.roadmark_type ||
           a.turn_directions != b.turn_directions;
}

bool lane_side_differs(const std::vector<model::LaneSpec>& a, const std::vector<model::LaneSpec>& b) {
    if (a.size() != b.size()) return true;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (lane_spec_differs(a[i], b[i])) return true;
    }
    return false;
}

bool lane_plan_differs(const model::LanePlan& a, const model::LanePlan& b) {
    return a.forward_lanes != b.forward_lanes || a.backward_lanes != b.backward_lanes ||
           lane_side_differs(a.left, b.left) || lane_side_differs(a.right, b.right);
}

// How one type-homogeneous run of lanes on one physical side (e.g. all "driving" lanes of .right)
// maps between two laneSections. When the run's length is unchanged, every lane pairs positionally
// (`paired[k] = (prev_idxs[k], next_idxs[k])`); when it differs, `added`/`removed` list the surplus
// lane(s) on whichever side is longer.
struct LaneRunAlignment {
    std::vector<std::pair<std::size_t, std::size_t>> paired; // (index into prev_side, index into next_side)
    std::vector<std::size_t> added;   // next_side indices with no predecessor (a lane split)
    std::vector<std::size_t> removed; // prev_side indices with no successor (a lane merge)
    bool extra_at_start = false; // added/removed lane(s) are innermost (index 0), not outermost
};

// Aligns one type-homogeneous run when its lane count differs between two laneSections (a real
// lane split or merge, not just a junction/road boundary where counts should normally match). The
// surplus lane(s) can sit at either edge of the longer run -- a new dedicated left-turn lane is
// innermost (index 0 of .right in RHT, closest to the road's own center), a new dedicated
// right-turn lane is outermost -- so which edge is decided by comparing turn_directions on the
// lanes present in *both* runs under each hypothesis and keeping whichever alignment preserves
// more of them, rather than assuming new lanes always append at the end.
LaneRunAlignment align_lane_run(const std::vector<model::LaneSpec>& prev_side, const std::vector<std::size_t>& prev_idxs,
                                 const std::vector<model::LaneSpec>& next_side, const std::vector<std::size_t>& next_idxs) {
    LaneRunAlignment result;
    const std::size_t prev_n = prev_idxs.size();
    const std::size_t next_n = next_idxs.size();
    const std::size_t n = std::min(prev_n, next_n);
    const std::size_t diff = prev_n > next_n ? prev_n - next_n : next_n - prev_n;

    bool extra_at_start = false;
    if (diff > 0 && n > 0) {
        const std::size_t prev_extra = prev_n > next_n ? diff : 0;
        const std::size_t next_extra = next_n > prev_n ? diff : 0;
        auto score = [&](const std::size_t prev_off, const std::size_t next_off) {
            int matches = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (prev_side[prev_idxs[prev_off + i]].turn_directions == next_side[next_idxs[next_off + i]].turn_directions) ++matches;
            }
            return matches;
        };
        extra_at_start = score(prev_extra, next_extra) > score(0, 0);
    }
    result.extra_at_start = extra_at_start;

    const std::size_t prev_offset = (extra_at_start && prev_n > next_n) ? diff : 0;
    const std::size_t next_offset = (extra_at_start && next_n > prev_n) ? diff : 0;
    for (std::size_t i = 0; i < n; ++i) result.paired.emplace_back(prev_idxs[prev_offset + i], next_idxs[next_offset + i]);
    if (prev_n > next_n) {
        const std::size_t start = extra_at_start ? 0 : n;
        for (std::size_t i = 0; i < diff; ++i) result.removed.push_back(prev_idxs[start + i]);
    } else if (next_n > prev_n) {
        const std::size_t start = extra_at_start ? 0 : n;
        for (std::size_t i = 0; i < diff; ++i) result.added.push_back(next_idxs[start + i]);
    }
    return result;
}

void apply_lane_run_alignment(std::vector<model::LaneSpec>& prev_side, std::vector<model::LaneSpec>& next_side,
                               const LaneRunAlignment& align) {
    for (const auto& [pi, ni] : align.paired) {
        prev_side[pi].link_successor_id = next_side[ni].id;
        next_side[ni].link_predecessor_id = prev_side[pi].id;
    }
}

// Whether one physical side's lane count genuinely changes between two laneSections (a real split
// or merge, as opposed to a same-count width/type/roadmark change) -- shared by fuse_chain (a
// lane-count change within one merged road's own chain) and plan_lane_count_bridge (the same
// question at a plain road-to-road boundary between two separate roads).
struct SidePreview {
    std::vector<std::size_t> added;
    std::vector<std::size_t> removed;
    bool extra_at_start = false; // the added/removed lane(s) are innermost, not outermost
};

SidePreview lane_side_preview(const std::vector<model::LaneSpec>& prev_side, const std::vector<model::LaneSpec>& next_side) {
    SidePreview result;
    std::map<std::string, std::vector<std::size_t>> prev_by_type, next_by_type;
    for (std::size_t i = 0; i < prev_side.size(); ++i) prev_by_type[prev_side[i].type].push_back(i);
    for (std::size_t i = 0; i < next_side.size(); ++i) next_by_type[next_side[i].type].push_back(i);
    for (auto& [type, prev_idxs] : prev_by_type) {
        const auto it = next_by_type.find(type);
        if (it == next_by_type.end()) continue;
        const auto align = align_lane_run(prev_side, prev_idxs, next_side, it->second);
        result.added.insert(result.added.end(), align.added.begin(), align.added.end());
        result.removed.insert(result.removed.end(), align.removed.begin(), align.removed.end());
        if (!align.added.empty() || !align.removed.empty()) result.extra_at_start = align.extra_at_start;
    }
    return result;
}

// Links matching lanes across a laneSection boundary, within each type-homogeneous run (e.g.
// driving-to-driving, sidewalk-to-sidewalk), using align_lane_run/apply_lane_run_alignment so a
// lane split/merge (differing run length) links by turn-direction similarity instead of naively
// truncating from index 0. `prev`'s own +s direction is assumed to flow into `next`'s own +s
// direction (i.e. prev is "upstream"); callers at a plain road-to-road boundary, where either
// road's s-direction may point either way relative to the shared node, are responsible for
// picking which physical side (`.left`/`.right`) plays "prev" and which plays "next" for each
// direction of travel -- see link_plain_road_lanes.
struct SideTransition {
    std::vector<std::size_t> added;   // next_side indices with no predecessor (a lane split)
    std::vector<std::size_t> removed; // prev_side indices with no successor (a lane merge)
};

SideTransition link_lane_side_with_transition(std::vector<model::LaneSpec>& prev_side, std::vector<model::LaneSpec>& next_side) {
    SideTransition result;
    std::map<std::string, std::vector<std::size_t>> prev_by_type, next_by_type;
    for (std::size_t i = 0; i < prev_side.size(); ++i) prev_by_type[prev_side[i].type].push_back(i);
    for (std::size_t i = 0; i < next_side.size(); ++i) next_by_type[next_side[i].type].push_back(i);
    for (auto& [type, prev_idxs] : prev_by_type) {
        const auto it = next_by_type.find(type);
        if (it == next_by_type.end()) continue;
        const auto align = align_lane_run(prev_side, prev_idxs, next_side, it->second);
        apply_lane_run_alignment(prev_side, next_side, align);
        result.added.insert(result.added.end(), align.added.begin(), align.added.end());
        result.removed.insert(result.removed.end(), align.removed.begin(), align.removed.end());
    }
    return result;
}

void link_lane_side(std::vector<model::LaneSpec>& prev_side, std::vector<model::LaneSpec>& next_side) {
    link_lane_side_with_transition(prev_side, next_side);
}

// Links matching lanes across an internal laneSection boundary within a merged road (prev's end
// is next's start, by construction of the merge chain).
void link_lane_sections(model::LanePlan& prev, model::LanePlan& next) {
    link_lane_side(prev.left, next.left);
    link_lane_side(prev.right, next.right);
}

// Links matching lanes across a plain (non-junction) road-to-road boundary -- two separate
// roads meeting directly at a node (e.g. a feature-split traffic light/stop sign, or any road end
// that isn't part of a merge chain and isn't a junction). Unlike link_lane_sections, either road's
// own +s direction can point toward or away from the shared node in any combination (OSM way
// directions are arbitrary), so which physical side (.left/.right) carries arriving vs. departing
// traffic must be worked out per pairing rather than assumed.
//
// left_hand_traffic decides which physical side is the "forward" (+s) one (see infer_lanes):
// RHT keeps forward lanes on .right, LHT on .left. Given that, four topological cases:
//   - one road's end here, the other's start here (+s flows straight through): same physical side
//     continues into the same physical side on the other road.
//   - both roads' ends here, or both roads' starts here (both +s directions point the same way
//     relative to the node -- both toward it, or both away): a continuing movement must switch
//     sides, since one road's arriving traffic is the other's departing traffic on the opposite
//     physical side.
void link_plain_road_lanes(model::RoadSegment& road_a, const bool a_at_start,
                            model::RoadSegment& road_b, const bool b_at_start,
                            const bool left_hand_traffic) {
    auto& lanes_a = lanes_at_end_mut(road_a, a_at_start);
    auto& lanes_b = lanes_at_end_mut(road_b, b_at_start);
    auto& fwd_a = left_hand_traffic ? lanes_a.left : lanes_a.right;
    auto& bwd_a = left_hand_traffic ? lanes_a.right : lanes_a.left;
    auto& fwd_b = left_hand_traffic ? lanes_b.left : lanes_b.right;
    auto& bwd_b = left_hand_traffic ? lanes_b.right : lanes_b.left;

    if (a_at_start != b_at_start) {
        if (!a_at_start) { link_lane_side(fwd_a, fwd_b); link_lane_side(bwd_b, bwd_a); }
        else { link_lane_side(fwd_b, fwd_a); link_lane_side(bwd_a, bwd_b); }
    } else {
        if (!a_at_start) { link_lane_side(fwd_a, bwd_b); link_lane_side(fwd_b, bwd_a); }
        else { link_lane_side(bwd_a, fwd_b); link_lane_side(bwd_b, fwd_a); }
    }
}

// ---- Plain-boundary lane-count bridge --------------------------------------------------------
//
// At a plain (non-junction) road-to-road boundary where the OSM lane count genuinely changes --
// e.g. a lane genuinely ends right at a signalized crossing, a real and not-rare shape found by
// running the project's own evaluation harness (test/run_benchmark.py) across real city extracts
// -- link_plain_road_lanes above links the lanes topologically correctly (see align_lane_run) but
// leaves a real *positional* discontinuity: each side's own laneOffset independently centers its
// own cross-section, so two roads with a differing lane count land their surviving lanes'
// centerlines a full lane-width apart even though their reference lines coincide exactly at the
// shared node. Unlike a merge chain (fuse_chain below), the two roads here are never fused into
// one <road>, so there is no single LanePlan to insert a taper LaneSection into.
//
// This inserts a short synthetic bridging <road> between the two, trimming both back from the
// shared node -- mirroring exactly how build_junction_connectors makes room for a connector, down
// to reusing apply_end_trim -- and connecting the trim points with the same tangent-fillet
// construction (a single line or arc). The two roads' own directions already meet at the
// identical shared node here (not merely close, as at a real junction with laterally offset
// lanes), so the fillet's two tangent lengths are necessarily equal: trimming both roads by the
// same amount and joining the trim points gives exact position *and* heading continuity at both
// ends by construction. The bridge's own laneOffset is pinned to each neighbor's actual value at
// its end, and every lane's width ramps linearly from the road_a-side cross-section to the
// road_b-side cross-section over the bridge's length -- the same kind of split/merge taper
// fuse_chain already does for a merge chain, just across an actual <road> boundary instead of
// within one road's own laneSections.
//
// Deliberately narrow in scope: only the "one road ends here, the other starts here" topology
// (a_at_start != b_at_start, a straight continuation) is handled; the rarer "both roads end/start
// here" cross-connect case falls back to today's direct link, unchanged. Any guard failure (near-
// reversal angle, not enough room to trim) likewise falls back to today's direct link -- this only
// ever adds a bridge where one is both needed and safe to build; every case it doesn't handle
// degrades to prior behavior exactly.
struct LaneCountBridgePlan {
    double trim_a = 0.0;
    double trim_b = 0.0;
    model::RoadSegment bridge;
    // a_is_upstream is true when road_a's own +s direction flows into the shared node (a_at_start
    // == false) and the bridge therefore continues from road_a into road_b; false when it's the
    // other way around (b_at_start == false). Exactly one of the two is upstream, since this whole
    // plan only ever exists for the a_at_start != b_at_start topology.
    bool a_is_upstream = true;
    std::vector<std::pair<int, int>> upstream_links;   // (upstream road's real lane id, bridge lane id)
    std::vector<std::pair<int, int>> downstream_links; // (bridge lane id, downstream road's real lane id)
};

// One physically-ordered lane of the bridge's own cross-section.
struct BridgeLane {
    model::LaneSpec spec;
    std::optional<int> pred_real_id; // upstream road's real lane id this continues from, if any
    std::optional<int> succ_real_id; // downstream road's real lane id this continues into, if any
};

// Builds one physical side (.left or .right) of the bridge's cross-section from the two roads'
// own lane lists at the boundary: paired lanes (present on both sides, via align_lane_run) ramp
// from their road_a-side width to their road_b-side width; a removed lane (only in prev/road_a)
// ramps to zero; an added lane (only in next/road_b) ramps from zero. Type-runs are processed in
// first-seen order (prev's own order, then any type appearing only in next) rather than sorted,
// so physical inner-to-outer ordering is preserved regardless of type name.
std::vector<BridgeLane> build_bridge_side(const std::vector<model::LaneSpec>& prev_side,
                                            const std::vector<model::LaneSpec>& next_side) {
    std::vector<std::string> type_order;
    auto note_type = [&](const std::string& t) {
        if (std::find(type_order.begin(), type_order.end(), t) == type_order.end()) type_order.push_back(t);
    };
    for (const auto& l : prev_side) note_type(l.type);
    for (const auto& l : next_side) note_type(l.type);

    std::map<std::string, std::vector<std::size_t>> prev_by_type, next_by_type;
    for (std::size_t i = 0; i < prev_side.size(); ++i) prev_by_type[prev_side[i].type].push_back(i);
    for (std::size_t i = 0; i < next_side.size(); ++i) next_by_type[next_side[i].type].push_back(i);

    std::vector<BridgeLane> out;
    static const std::vector<std::size_t> kEmpty;
    for (const auto& type : type_order) {
        const auto& prev_idxs = prev_by_type.count(type) ? prev_by_type[type] : kEmpty;
        const auto& next_idxs = next_by_type.count(type) ? next_by_type[type] : kEmpty;
        const auto align = align_lane_run(prev_side, prev_idxs, next_side, next_idxs);

        std::vector<BridgeLane> paired_run, extra_run;
        for (const auto& [pi, ni] : align.paired) {
            BridgeLane bl;
            bl.spec = next_side[ni];
            bl.spec.width = prev_side[pi].width;
            bl.spec.width_end = next_side[ni].width;
            bl.pred_real_id = prev_side[pi].id;
            bl.succ_real_id = next_side[ni].id;
            paired_run.push_back(std::move(bl));
        }
        for (const auto i : align.removed) {
            BridgeLane bl;
            bl.spec = prev_side[i];
            bl.spec.width_end = 0.0;
            bl.pred_real_id = prev_side[i].id;
            extra_run.push_back(std::move(bl));
        }
        for (const auto i : align.added) {
            BridgeLane bl;
            bl.spec = next_side[i];
            bl.spec.width_end = next_side[i].width;
            bl.spec.width = 0.0;
            bl.succ_real_id = next_side[i].id;
            extra_run.push_back(std::move(bl));
        }
        if (align.extra_at_start) {
            out.insert(out.end(), extra_run.begin(), extra_run.end());
            out.insert(out.end(), paired_run.begin(), paired_run.end());
        } else {
            out.insert(out.end(), paired_run.begin(), paired_run.end());
            out.insert(out.end(), extra_run.begin(), extra_run.end());
        }
    }
    return out;
}

std::optional<LaneCountBridgePlan> plan_lane_count_bridge(const model::RoadSegment& road_a, const bool a_at_start,
                                                            const model::RoadSegment& road_b, const bool b_at_start,
                                                            const Options& options) {
    if (a_at_start == b_at_start) return std::nullopt; // cross-connect: out of scope, see comment above

    const auto& lanes_a = lanes_at_end(road_a, a_at_start);
    const auto& lanes_b = lanes_at_end(road_b, b_at_start);
    const bool lht = options.left_hand_traffic;
    const auto& fwd_a = lht ? lanes_a.left : lanes_a.right;
    const auto& bwd_a = lht ? lanes_a.right : lanes_a.left;
    const auto& fwd_b = lht ? lanes_b.left : lanes_b.right;
    const auto& bwd_b = lht ? lanes_b.right : lanes_b.left;

    // Mirrors link_plain_road_lanes's own a_at_start != b_at_start branch: dirs[0] is always the
    // forward-direction-of-travel (prev, next) pair, dirs[1] the backward one, regardless of which
    // road's physical .left/.right ends up playing which role.
    struct DirPair { const std::vector<model::LaneSpec>* prev; const std::vector<model::LaneSpec>* next; };
    std::vector<DirPair> dirs;
    if (!a_at_start) {
        dirs.push_back({&fwd_a, &fwd_b});
        dirs.push_back({&bwd_b, &bwd_a});
    } else {
        dirs.push_back({&fwd_b, &fwd_a});
        dirs.push_back({&bwd_a, &bwd_b});
    }

    bool any_change = false;
    for (const auto& d : dirs) {
        const auto preview = lane_side_preview(*d.prev, *d.next);
        if (!preview.added.empty() || !preview.removed.empty()) any_change = true;
    }
    if (!any_change) return std::nullopt; // plain width/type change only: today's direct link is already correct

    // Whichever of the two roads has at_start == false is "upstream" here: its own +s direction
    // flows into the shared node, so the bridge must continue in that same direction (bridge s=0
    // touches it, s=length touches the other, "downstream" road) for the combined path to have one
    // consistent forward sense -- getting this backwards (e.g. always assuming road_a is upstream)
    // reverses the bridge's own heading by exactly pi at whichever end that assumption is wrong,
    // since dir_into/dir_away are each other's negation.
    const bool a_is_upstream = !a_at_start;
    const model::RoadSegment& upstream_road = a_is_upstream ? road_a : road_b;
    const model::RoadSegment& downstream_road = a_is_upstream ? road_b : road_a;

    const auto node = endpoint_point(upstream_road, false);
    const auto dir_into = direction_into_junction(upstream_road, false);
    const auto dir_away = direction_away_from_junction(downstream_road, true);
    const double signed_delta = std::atan2(geo::cross(dir_into, dir_away), geo::dot(dir_into, dir_away));
    const double abs_delta = std::min(std::abs(signed_delta), geo::kPi - 0.001);
    if (abs_delta > geo::deg_to_rad(160.0)) return std::nullopt; // near-reversal: ill-conditioned, same guard as junction connectors

    // Capped the same way fuse_chain caps its own taper length: at most the taper-length option,
    // and at most 40% of the immediately adjacent OSM sub-segment so a trim never eats into an
    // earlier real bend. Both roads get the *same* trim -- a fillet inscribed at a single shared
    // vertex necessarily has equal tangent lengths on both rays, so this is a geometric requirement
    // here, not just a simplification.
    const double budget_a = 0.4 * segment_length_at_end(road_a, a_at_start);
    const double budget_b = 0.4 * segment_length_at_end(road_b, b_at_start);
    const double trim = std::min({options.lane_taper_length, budget_a, budget_b});
    if (trim < 0.5) return std::nullopt; // not enough room to bother

    const double tan_half = std::tan(abs_delta / 2.0);
    double bridge_length = 0.0;
    double curvature = 0.0;
    model::GeomKind kind = model::GeomKind::Line;
    if (tan_half > 1e-6) {
        const double radius = trim / tan_half;
        bridge_length = radius * abs_delta;
        curvature = (signed_delta >= 0.0 ? 1.0 : -1.0) / radius;
        kind = model::GeomKind::Arc;
    } else {
        bridge_length = 2.0 * trim; // already near-collinear: a straight bridge needs no arc
    }
    if (bridge_length < 1e-3) return std::nullopt;

    LaneCountBridgePlan plan;
    plan.trim_a = trim;
    plan.trim_b = trim;
    plan.a_is_upstream = a_is_upstream;
    model::RoadSegment& bridge = plan.bridge;
    bridge.id = "br_" + upstream_road.id + "_" + downstream_road.id;
    bridge.length = bridge_length;
    bridge.junction_id.clear();
    bridge.predecessor_xml = make_road_link_xml("road", upstream_road.id, "end");
    bridge.successor_xml = make_road_link_xml("road", downstream_road.id, "start");
    bridge.lanes.center_mark = a_is_upstream ? lanes_a.center_mark : lanes_b.center_mark;
    bridge.lanes.lane_offset = lane_offset_at_road_end(upstream_road, false);
    const double far_offset = lane_offset_at_road_end(downstream_road, true);
    bridge.lanes.lane_offset_slope = (far_offset - bridge.lanes.lane_offset) / bridge_length;

    const auto bridge_start = geo::Vec2{node.x - dir_into.x * trim, node.y - dir_into.y * trim};
    model::GeomPrimitive g;
    g.x = bridge_start.x;
    g.y = bridge_start.y;
    g.hdg = std::atan2(dir_into.y, dir_into.x);
    g.length = bridge_length;
    g.curvature = curvature;
    g.kind = kind;
    bridge.explicit_geometry.push_back(g);

    const auto fwd_bridge_lanes = build_bridge_side(*dirs[0].prev, *dirs[0].next);
    const auto bwd_bridge_lanes = build_bridge_side(*dirs[1].prev, *dirs[1].next);

    auto assign_side = [&](std::vector<model::LaneSpec>& out_side, const std::vector<BridgeLane>& bridge_lanes, const bool left_side) {
        int counter = 1;
        for (const auto& bl : bridge_lanes) {
            model::LaneSpec spec = bl.spec;
            const int bid = left_side ? counter : -counter;
            ++counter;
            spec.id = bid;
            spec.link_predecessor_id = bl.pred_real_id;
            spec.link_successor_id = bl.succ_real_id;
            if (bl.pred_real_id) plan.upstream_links.emplace_back(*bl.pred_real_id, bid);
            if (bl.succ_real_id) plan.downstream_links.emplace_back(bid, *bl.succ_real_id);
            out_side.push_back(spec);
        }
    };
    if (lht) {
        assign_side(bridge.lanes.left, fwd_bridge_lanes, true);
        assign_side(bridge.lanes.right, bwd_bridge_lanes, false);
    } else {
        assign_side(bridge.lanes.right, fwd_bridge_lanes, false);
        assign_side(bridge.lanes.left, bwd_bridge_lanes, true);
    }

    return plan;
}

// Builds the "glue" graph: endpoint_key -> endpoint_key for every pair of road ends that should be
// fused together. Symmetric (glue[a]=b implies glue[b]=a).
std::unordered_map<std::size_t, std::size_t> build_glue_map(
    const std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash>& endpoint_map,
    const std::unordered_set<std::int64_t>& junction_nodes,
    const std::unordered_set<std::int64_t>& feature_split_nodes) {
    std::unordered_map<std::size_t, std::size_t> glue;
    for (const auto& [node, endpoints] : endpoint_map) {
        if (endpoints.size() != 2) continue;
        if (junction_nodes.count(node)) continue;
        if (feature_split_nodes.count(node)) continue;
        const auto& a = endpoints[0];
        const auto& b = endpoints[1];
        if (a.road_index == b.road_index) continue; // closed-loop/self-referencing way: not a real join
        const std::size_t ka = endpoint_key(a.road_index, a.at_start);
        const std::size_t kb = endpoint_key(b.road_index, b.at_start);
        glue[ka] = kb;
        glue[kb] = ka;
    }
    return glue;
}

struct ChainSlot {
    std::size_t road_index = 0;
    bool reversed = false;
};

std::size_t chain_exit_port(std::size_t road_index, bool reversed) { return endpoint_key(road_index, reversed); }
std::size_t chain_entry_port(std::size_t road_index, bool reversed) { return endpoint_key(road_index, !reversed); }

// Partitions every road into maximal chains along the glue graph. Each chain is ordered so
// consecutive slots connect end-to-end (accounting for `reversed`); a chain of length 1 is just an
// unmerged road. Defensively bounded against pure closed loops (no junction anywhere) by refusing
// to revisit a road already placed in the chain being built.
std::vector<std::vector<ChainSlot>> build_merge_chains(
    const std::size_t road_count, const std::unordered_map<std::size_t, std::size_t>& glue) {
    std::vector<std::vector<ChainSlot>> chains;
    std::vector<bool> visited(road_count, false);

    for (std::size_t i = 0; i < road_count; ++i) {
        if (visited[i]) continue;

        // Walk backward to find the true front of this chain.
        std::size_t front_road = i;
        bool front_reversed = false;
        std::unordered_set<std::size_t> backward_guard;
        while (true) {
            backward_guard.insert(front_road);
            const auto it = glue.find(chain_entry_port(front_road, front_reversed));
            if (it == glue.end()) break;
            const std::size_t prev_road = it->second / 2;
            const bool prev_at_start = (it->second % 2) == 1;
            if (visited[prev_road] || backward_guard.count(prev_road)) break;
            front_road = prev_road;
            front_reversed = prev_at_start;
        }

        // Walk forward from the front, building the ordered chain.
        std::vector<ChainSlot> chain;
        std::unordered_set<std::size_t> in_chain;
        std::size_t cur_road = front_road;
        bool cur_reversed = front_reversed;
        while (!in_chain.count(cur_road)) {
            chain.push_back({cur_road, cur_reversed});
            in_chain.insert(cur_road);
            visited[cur_road] = true;

            const auto it = glue.find(chain_exit_port(cur_road, cur_reversed));
            if (it == glue.end()) break;
            const std::size_t next_road = it->second / 2;
            const bool next_at_start = (it->second % 2) == 1;
            if (visited[next_road] || in_chain.count(next_road)) break;
            cur_road = next_road;
            cur_reversed = !next_at_start;
        }
        chains.push_back(std::move(chain));
    }
    return chains;
}

// Fuses one chain into a single RoadSegment. Reversed constituents are re-inferred from
// direction-swapped tags (reusing swap_directional_tags, already exercised for OSM oneway=-1 ways)
// rather than hand-mirroring LanePlan's left/right vectors and offsets.
model::RoadSegment fuse_chain(const std::vector<ChainSlot>& chain, const std::vector<model::RoadSegment>& old_roads,
                               const Options& options, std::vector<std::string>& warnings) {
    model::RoadSegment merged;
    model::LanePlan prev_lanes;
    double running_length = 0.0; // arc length of `merged.points` as built so far
    double prev_part_length = 0.0; // length of the immediately preceding chain part
    std::size_t longest_name_source = 0;
    double longest_name_length = -1.0;

    for (std::size_t idx = 0; idx < chain.size(); ++idx) {
        const auto& slot = chain[idx];
        const model::RoadSegment& part = old_roads[slot.road_index];
        const double part_start_s = running_length; // s at which this part begins on the merged road

        Tags part_tags = part.tags;
        std::vector<geo::Vec2> part_points = part.points;
        std::vector<std::int64_t> part_refs = part.refs;
        model::LanePlan part_lanes;

        if (slot.reversed) {
            std::reverse(part_points.begin(), part_points.end());
            std::reverse(part_refs.begin(), part_refs.end());
            swap_directional_tags(part_tags);
            std::vector<std::string> throwaway_warnings;
            part_lanes = infer::infer_lanes(part_tags, options, throwaway_warnings, part.id);
        } else {
            part_lanes = part.lanes;
        }

        if (!tag_value_or(part_tags, "name", "").empty() && part.length > longest_name_length) {
            longest_name_length = part.length;
            longest_name_source = idx;
        }

        if (idx == 0) {
            merged.id = part.id;
            merged.source_way_id = part.source_way_id;
            merged.segment_index = part.segment_index;
            merged.tags = part_tags;
            merged.lanes = part_lanes;
            merged.points = part_points;
            merged.refs = part_refs;
        } else {
            merged.points.insert(merged.points.end(), part_points.begin() + 1, part_points.end());
            merged.refs.insert(merged.refs.end(), part_refs.begin() + 1, part_refs.end());

            if (lane_plan_differs(prev_lanes, part_lanes)) {
                if (prev_lanes.forward_lanes != part_lanes.forward_lanes || prev_lanes.backward_lanes != part_lanes.backward_lanes) {
                    warnings.push_back("Merged road " + merged.id + " has a lane-count change at s=" +
                                        std::to_string(part_start_s) + " (from source way " + std::to_string(part.source_way_id) + ").");
                }

                model::LanePlan& prior_lanes = merged.extra_lane_sections.empty() ? merged.lanes : merged.extra_lane_sections.back().lanes;

                // Preview (without mutating link ids yet) whether this transition actually adds or
                // drops a lane on either side, as opposed to merely a width/type/roadmark change at
                // the same lane count -- only the former needs a taper. Doesn't touch prior_lanes or
                // part_lanes: align_lane_run is read-only, and the real linking happens below via
                // whichever section ends up adjacent to `part_lanes` (which may be a freshly
                // inserted taper section rather than `prior_lanes` itself).
                const auto left_preview = lane_side_preview(prior_lanes.left, part_lanes.left);
                const auto right_preview = lane_side_preview(prior_lanes.right, part_lanes.right);
                const auto& added_left = left_preview.added;
                const auto& removed_left = left_preview.removed;
                const auto& added_right = right_preview.added;
                const auto& removed_right = right_preview.removed;
                const bool has_added = !added_left.empty() || !added_right.empty();
                const bool has_removed = !removed_left.empty() || !removed_right.empty();

                if (!has_added && !has_removed) {
                    // Plain width/type/roadmark change at the same lane count: single boundary,
                    // no taper, same behavior as before this feature existed.
                    model::LaneSection section;
                    section.s_offset = part_start_s;
                    section.lanes = part_lanes;
                    section.tags = part_tags;
                    section.source_way_id = part.source_way_id;
                    section.segment_index = part.segment_index;
                    link_lane_sections(prior_lanes, section.lanes);
                    merged.extra_lane_sections.push_back(std::move(section));
                } else {
                    // A real lane split (has_added) or merge (has_removed): the appearing/
                    // disappearing lane ramps its width to/from zero over a short distance instead
                    // of popping in/out abruptly at the boundary. The reference line's laneOffset
                    // only needs to move at all when the changing lane is *innermost* (index 0,
                    // adjacent to the reference line/center) -- an *outermost* addition/removal
                    // simply extends or retracts the pavement edge beyond the existing lanes, which
                    // keep their exact prior absolute position with no laneOffset change at all. An
                    // innermost change shifts the existing lanes by exactly its own width (that's
                    // the only way their absolute position stays fixed while a lane is inserted or
                    // removed between them and the reference line): +width for a .right lane (whose
                    // positions are `lane_offset - accumulated_width`, so growing the innermost lane
                    // must grow lane_offset to compensate), -width for a .left lane (mirrored sign,
                    // since .left positions are `lane_offset + accumulated_width`).
                    const double taper_out_len = has_removed ? std::min(options.lane_taper_length, prev_part_length * 0.4) : 0.0;
                    const double taper_in_len = has_added ? std::min(options.lane_taper_length, geo::polyline_length(part_points) * 0.4) : 0.0;

                    // Tracks the lane_offset value at the point the *next* section should pick up
                    // from -- not simply "whatever LanePlan::lane_offset says", since that field is
                    // each section's value at its own s=0, and a just-inserted taper section's own
                    // end (where the next section attaches) is offset from that by its own slope.
                    double handoff_offset = prior_lanes.lane_offset;

                    if (has_removed) {
                        model::LanePlan taper_out = prior_lanes; // same lane list/ids; only widths change
                        double full_width_left = 0.0, full_width_right = 0.0;
                        for (auto i : removed_left) full_width_left += taper_out.left[i].width;
                        for (auto i : removed_right) full_width_right += taper_out.right[i].width;
                        for (auto i : removed_left) taper_out.left[i].width_end = 0.0;
                        for (auto i : removed_right) taper_out.right[i].width_end = 0.0;

                        taper_out.lane_offset = handoff_offset; // matches prior exactly at full width
                        const double offset_delta = (right_preview.extra_at_start ? -full_width_right : 0.0) +
                                                     (left_preview.extra_at_start ? full_width_left : 0.0);
                        taper_out.lane_offset_slope = taper_out_len > 1e-6 ? offset_delta / taper_out_len : 0.0;
                        handoff_offset += offset_delta;

                        model::LaneSection section;
                        section.s_offset = part_start_s - taper_out_len;
                        section.tags = merged.extra_lane_sections.empty() ? merged.tags : merged.extra_lane_sections.back().tags;
                        section.source_way_id = merged.extra_lane_sections.empty() ? merged.source_way_id : merged.extra_lane_sections.back().source_way_id;
                        section.segment_index = merged.extra_lane_sections.empty() ? merged.segment_index : merged.extra_lane_sections.back().segment_index;
                        link_lane_sections(prior_lanes, taper_out); // trivial: same lane ids on both sides
                        section.lanes = std::move(taper_out);
                        merged.extra_lane_sections.push_back(std::move(section));
                    }

                    model::LanePlan& immediate_prior = merged.extra_lane_sections.empty() ? merged.lanes : merged.extra_lane_sections.back().lanes;

                    model::LanePlan real_next = part_lanes;
                    double full_width_left_added = 0.0, full_width_right_added = 0.0;
                    for (auto i : added_left) full_width_left_added += part_lanes.left[i].width;
                    for (auto i : added_right) full_width_right_added += part_lanes.right[i].width;
                    for (auto i : added_left) {
                        real_next.left[i].width_end = part_lanes.left[i].width; // taper 0 -> full width
                        real_next.left[i].width = 0.0;
                    }
                    for (auto i : added_right) {
                        real_next.right[i].width_end = part_lanes.right[i].width;
                        real_next.right[i].width = 0.0;
                    }
                    real_next.lane_offset = handoff_offset; // matches the prior section's own end, at zero added-width
                    if (has_added) {
                        const double offset_delta = (right_preview.extra_at_start ? full_width_right_added : 0.0) +
                                                     (left_preview.extra_at_start ? -full_width_left_added : 0.0);
                        real_next.lane_offset_slope = taper_in_len > 1e-6 ? offset_delta / taper_in_len : 0.0;
                        handoff_offset += offset_delta;
                    }
                    link_lane_sections(immediate_prior, real_next);

                    model::LaneSection real_section;
                    real_section.s_offset = part_start_s;
                    real_section.lanes = real_next;
                    real_section.tags = part_tags;
                    real_section.source_way_id = part.source_way_id;
                    real_section.segment_index = part.segment_index;
                    merged.extra_lane_sections.push_back(std::move(real_section));

                    if (has_added) {
                        // Constant-width continuation once the added lane has finished widening:
                        // same structure as `part_lanes`, just picking up where the taper-in left
                        // off (widths reach part_lanes' own full-width values; lane_offset picks up
                        // from handoff_offset rather than part_lanes' own independently-computed
                        // value, since this road's reference line is anchored to its unaffected
                        // lanes' actual position, not recentered on this way's own total width).
                        model::LanePlan constant_next = part_lanes;
                        constant_next.lane_offset = handoff_offset;
                        model::LaneSection section;
                        section.s_offset = part_start_s + taper_in_len;
                        section.lanes = constant_next;
                        section.tags = part_tags;
                        section.source_way_id = part.source_way_id;
                        section.segment_index = part.segment_index;
                        link_lane_sections(merged.extra_lane_sections.back().lanes, section.lanes); // trivial: same lane ids
                        merged.extra_lane_sections.push_back(std::move(section));
                    }
                }
            }
        }
        prev_part_length = geo::polyline_length(part_points);
        running_length = part_start_s + geo::polyline_length(part_points);

        // Signals carry s/t relative to their own constituent's original (possibly now-reversed)
        // orientation; rebase s onto the merged road and flip t/orientation when reversed.
        for (auto sig : part.signals) {
            if (slot.reversed) {
                sig.s = part.length - sig.s;
                sig.t = -sig.t;
                sig.orientation = sig.t >= 0.0 ? "+" : "-";
            }
            sig.s += part_start_s;
            merged.signals.push_back(sig);
        }

        prev_lanes = part_lanes;
    }

    merged.length = geo::polyline_length(merged.points);
    merged.start_ref = merged.refs.front();
    merged.end_ref = merged.refs.back();
    if (longest_name_length >= 0.0) merged.tags["name"] = tag_value_or(old_roads[chain[longest_name_source].road_index].tags, "name", "");
    return merged;
}

// ---- ModelBuilder -------------------------------------------------------------------------------
//
// Orchestrates build_model()'s pipeline as a sequence of phases, each a private method. Fields hold
// exactly the state that must survive across phase boundaries (endpoint/junction maps, compound
// junction cluster info); everything scoped to a single phase stays a local variable inside that
// phase's method, same as it was a local variable inside the original monolithic function.
class ModelBuilder {
public:
    ModelBuilder(const osm::ParseResult& parsed, const Options& options) : parsed_(parsed), options_(options) {}

    model::MapModel build();

private:
    void build_fragments();
    void merge_roads();
    void cluster_compound_junctions();
    void link_plain_roads();
    void build_junction_connectors();
    void place_signals();
    void fit_curves();
    void apply_tracked_trim(std::size_t road_index, bool at_start, double applied);

    std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash> build_endpoint_map(
        const std::vector<model::RoadSegment>& roads) const;
    std::unordered_set<std::int64_t> find_junction_nodes(
        const std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash>& map) const;

    const osm::ParseResult& parsed_;
    const Options& options_;
    model::MapModel model_;

    std::unordered_set<std::int64_t> feature_split_nodes_;
    std::unordered_set<std::int64_t> traffic_light_nodes_; // subset of feature_split_nodes_, traffic_light only
    std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash> endpoint_map_;
    std::unordered_set<std::int64_t> junction_nodes_;
    std::unordered_map<std::int64_t, std::vector<std::int64_t>, EndpointKeyHash> cluster_members_;
    std::unordered_map<std::int64_t, std::string, EndpointKeyHash> node_to_junction_id_;
    int direct_fallback_count_ = 0;

    // A road pulled back to make room for a junction connector or lane-count bridge (apply_end_trim,
    // via apply_tracked_trim) can strand a signal that used to sit near the removed stretch --
    // place_signals() runs after all trimming, so a naive nearest-road search would snap that signal
    // onto whatever else happens to be close, sometimes with a large, nonsensical lateral offset.
    // Keyed by road index (stable across trimming: it only mutates roads in place / appends new
    // ones, never removes or reorders existing ones), these let place_signals() match against each
    // road's original (pre-trim) shape instead, then clamp the result back into the final geometry.
    std::unordered_map<std::size_t, std::vector<geo::Vec2>> pre_trim_points_;
    std::unordered_map<std::size_t, double> trimmed_from_start_;
};

model::MapModel ModelBuilder::build() {
    model_.projector = parsed_.projector;
    model_.warnings = parsed_.warnings;

    build_fragments();
    merge_roads();
    cluster_compound_junctions();
    link_plain_roads();
    build_junction_connectors();
    place_signals();
    if (options_.curve_fit) fit_curves();

    if (model_.roads.empty()) {
        model_.north = model_.south = model_.east = model_.west = 0.0;
    }

    if (direct_fallback_count_ > 0) {
        model_.warnings.push_back(std::to_string(direct_fallback_count_) +
            " junction lane connection(s) could not fit a curved connector road within the available "
            "road geometry (roads too short, or an original bend too close to the junction) and were "
            "linked directly between the incoming and outgoing roads instead.");
    }

    return model_;
}

std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash> ModelBuilder::build_endpoint_map(
    const std::vector<model::RoadSegment>& roads) const {
    std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash> map;
    for (std::size_t i = 0; i < roads.size(); ++i) {
        map[roads[i].start_ref].push_back({i, true});
        map[roads[i].end_ref].push_back({i, false});
    }
    return map;
}

std::unordered_set<std::int64_t> ModelBuilder::find_junction_nodes(
    const std::unordered_map<std::int64_t, std::vector<model::EndpointRef>, EndpointKeyHash>& map) const {
    std::unordered_set<std::int64_t> nodes;
    for (const auto& [node, endpoints] : map) {
        if (static_cast<int>(endpoints.size()) >= options_.junction_degree) nodes.insert(node);
    }
    return nodes;
}

void ModelBuilder::build_fragments() {
    std::unordered_map<std::int64_t, int, EndpointKeyHash> road_node_occurrences;
    for (const auto& way : parsed_.roads) {
        std::unordered_set<std::int64_t> seen_in_way;
        for (const auto& node : way.nodes) {
            if (seen_in_way.insert(node.ref).second) road_node_occurrences[node.ref] += 1;
        }
    }

    std::unordered_set<std::int64_t> split_nodes;
    for (const auto& [ref, count] : road_node_occurrences) {
        if (count >= 2) split_nodes.insert(ref);
    }
    // Nodes split only because a traffic light/stop/give-way sits there (not because of real OSM
    // topology). Tracked separately from `split_nodes` so road merging can fuse ordinary
    // topology-only splits back together while still treating these as permanent road boundaries.
    for (const auto& pf : parsed_.point_features) {
        if (pf.kind == "traffic_light" || pf.kind == "stop" || pf.kind == "give_way") {
            split_nodes.insert(pf.node_ref);
            feature_split_nodes_.insert(pf.node_ref);
            if (pf.kind == "traffic_light") traffic_light_nodes_.insert(pf.node_ref);
        }
    }

    for (const auto& way : parsed_.roads) {
        const auto split = split_indices_for_way(way, split_nodes);
        for (std::size_t part = 1; part < split.size(); ++part) {
            const auto from = split[part - 1];
            const auto to = split[part];
            if (to <= from) continue;
            model::RoadSegment road;
            road.source_way_id = way.id;
            road.segment_index = static_cast<int>(part - 1);
            road.id = road_id_for(way.id, road.segment_index);
            road.tags = way.tags;
            for (std::size_t i = from; i <= to; ++i) {
                road.refs.push_back(way.nodes[i].ref);
                const auto p = parsed_.projector.project(way.nodes[i].ll.lat, way.nodes[i].ll.lon);
                road.points.push_back(p);
                model_.north = std::max(model_.north, p.y);
                model_.south = std::min(model_.south, p.y);
                model_.east = std::max(model_.east, p.x);
                model_.west = std::min(model_.west, p.x);
            }
            road.start_ref = road.refs.front();
            road.end_ref = road.refs.back();
            road.length = geo::polyline_length(road.points);
            if (road.length < 0.05) {
                model_.warnings.push_back("Road " + road.id + " has near-zero length and was skipped.");
                continue;
            }
            road.lanes = infer::infer_lanes(road.tags, options_, model_.warnings, road.id);
            model_.roads.push_back(std::move(road));
        }
    }
}

void ModelBuilder::merge_roads() {
    endpoint_map_ = build_endpoint_map(model_.roads);
    junction_nodes_ = find_junction_nodes(endpoint_map_);

    if (options_.merge_roads) {
        const auto glue = build_glue_map(endpoint_map_, junction_nodes_, feature_split_nodes_);
        const auto chains = build_merge_chains(model_.roads.size(), glue);
        std::vector<model::RoadSegment> merged_roads;
        merged_roads.reserve(chains.size());
        for (const auto& chain : chains) {
            merged_roads.push_back(chain.size() == 1
                ? model_.roads[chain.front().road_index]
                : fuse_chain(chain, model_.roads, options_, model_.warnings));
        }
        model_.roads = std::move(merged_roads);

        // Road indices changed; endpoint_map/junction_nodes must be rebuilt from scratch (never
        // incrementally patched) over the merged road list.
        endpoint_map_ = build_endpoint_map(model_.roads);
        junction_nodes_ = find_junction_nodes(endpoint_map_);
    }
}

// ---- Compound junction clustering --------------------------------------------------------
//
// Real intersections are sometimes mapped as several close-together OSM junction nodes (e.g.
// tram tracks crossing at a slightly offset point, or a wide junction's corner geometry)
// rather than one point, with the real traffic-light-controlled boundary further out on each
// approach. A road whose *both* ends are junction nodes cannot have a traffic light between
// them (feature_split_nodes are excluded from merging, so merging would have stopped there
// otherwise) -- so such roads are exactly the "interior" links of one bigger physical
// intersection, and get folded into a single compound <junction> instead of being kept as
// separate tiny roads between separate tiny junctions.
void ModelBuilder::cluster_compound_junctions() {
    std::unordered_map<std::int64_t, std::int64_t, EndpointKeyHash> parent;
    for (const auto& node : junction_nodes_) parent[node] = node;
    std::function<std::int64_t(std::int64_t)> find_root = [&](std::int64_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };
    auto unite_nodes = [&](const std::int64_t a, const std::int64_t b) {
        const auto ra = find_root(a);
        const auto rb = find_root(b);
        if (ra != rb) parent[ra] = rb;
    };

    if (options_.merge_junctions) {
        for (std::size_t i = 0; i < model_.roads.size(); ++i) {
            const auto& road = model_.roads[i];
            if (road.start_ref != road.end_ref && junction_nodes_.count(road.start_ref) &&
                junction_nodes_.count(road.end_ref) && road.length <= options_.junction_cluster_max_gap) {
                unite_nodes(road.start_ref, road.end_ref);
            }
        }

        // A road whose two ends land in the same cluster only *transitively* (joined via other,
        // shorter interior roads, not directly under the gap cap) must also be treated as
        // interior -- otherwise it would survive as a real road with both ends pointing at the
        // same compound junction, redundant with (and inconsistent with) the connectors already
        // spanning its endpoints as two of the junction's own legs.
        std::vector<std::size_t> interior_road_indices;
        for (std::size_t i = 0; i < model_.roads.size(); ++i) {
            const auto& road = model_.roads[i];
            if (road.start_ref != road.end_ref && junction_nodes_.count(road.start_ref) &&
                junction_nodes_.count(road.end_ref) && find_root(road.start_ref) == find_root(road.end_ref)) {
                interior_road_indices.push_back(i);
            }
        }

        if (!interior_road_indices.empty()) {
            std::size_t dropped_signals = 0;
            for (const auto idx : interior_road_indices) dropped_signals += model_.roads[idx].signals.size();
            if (dropped_signals > 0) {
                model_.warnings.push_back(std::to_string(dropped_signals) +
                    " signal(s) on interior compound-junction link road(s) were dropped when those roads were absorbed into the junction.");
            }

            std::unordered_set<std::size_t> to_remove(interior_road_indices.begin(), interior_road_indices.end());
            std::vector<model::RoadSegment> kept_roads;
            kept_roads.reserve(model_.roads.size() - interior_road_indices.size());
            for (std::size_t i = 0; i < model_.roads.size(); ++i) {
                if (!to_remove.count(i)) kept_roads.push_back(std::move(model_.roads[i]));
            }
            model_.roads = std::move(kept_roads);

            // Road indices changed again; rebuild endpoint_map/junction_nodes from the
            // post-deletion road list. Note this can make an individual cluster member's own
            // degree drop below junction_degree (its interior legs are gone) even though the
            // cluster's combined degree still clearly qualifies -- handled below by summing
            // per-cluster, not re-testing each node's own degree in isolation.
            endpoint_map_ = build_endpoint_map(model_.roads);
            junction_nodes_ = find_junction_nodes(endpoint_map_);
        }
    }

    // A traffic light is typically mapped a short distance before the real junction node it
    // controls, splitting the approach way into a short "stub" road between the light and the
    // junction (feature_split_nodes_ always treats the light as a hard boundary, so road merging
    // never fuses it away). That stub's junction-facing end already reaches a real,
    // already-classified junction node by this point; absorbing the stub the same way an
    // "interior" road above gets absorbed lets the light's own node join the junction's member
    // set, so connectors start right at the light instead of one node further downstream.
    // Independent of merge_junctions (this doesn't require linking multiple *real* junction nodes
    // together), but must run after that block's own removal/rebuild so junction_nodes_ here is
    // final and accurate. Iterated to a fixed point: a pedestrian/level-crossing signal folded
    // into a junction's signal control (crossing=traffic_signals) is also classified as a traffic
    // light, so a real approach can have more than one such node in a row (light -> light ->
    // junction) -- absorbing the inner hop can newly qualify the outer one on the next pass.
    if (options_.absorb_signal_setbacks) {
        std::vector<std::size_t> setback_road_indices;
        std::unordered_set<std::size_t> absorbed;
        for (bool changed = true; changed;) {
            changed = false;
            for (std::size_t i = 0; i < model_.roads.size(); ++i) {
                if (absorbed.count(i)) continue;
                const auto& road = model_.roads[i];
                if (road.start_ref == road.end_ref || road.length > options_.junction_signal_setback_max_gap) continue;
                // `parent` (not junction_nodes_, which was just rebuilt above) is the right membership
                // test here: a compound cluster's individual member node can legitimately drop below
                // junction_degree on its own once the interior-road removal above takes out its
                // now-redundant links to other members (see that block's comment), even though the
                // cluster it belongs to still clearly qualifies. `parent` was seeded from every
                // originally-qualifying junction node and never reset, so it still reflects "is this
                // node part of a cluster" correctly regardless of how the node's own degree moved.
                const bool start_is_junction = parent.count(road.start_ref) != 0;
                const bool end_is_junction = parent.count(road.end_ref) != 0;
                if (start_is_junction == end_is_junction) continue; // need exactly one junction end
                const std::int64_t junction_node = start_is_junction ? road.start_ref : road.end_ref;
                const std::int64_t other_node = start_is_junction ? road.end_ref : road.start_ref;
                if (!traffic_light_nodes_.count(other_node)) continue;
                if (parent.find(other_node) == parent.end()) parent[other_node] = other_node;
                unite_nodes(junction_node, other_node);
                absorbed.insert(i);
                setback_road_indices.push_back(i);
                changed = true;
            }
        }

        if (!setback_road_indices.empty()) {
            std::unordered_set<std::size_t> to_remove(setback_road_indices.begin(), setback_road_indices.end());
            std::vector<model::RoadSegment> kept_roads;
            kept_roads.reserve(model_.roads.size() - setback_road_indices.size());
            for (std::size_t i = 0; i < model_.roads.size(); ++i) {
                if (!to_remove.count(i)) kept_roads.push_back(std::move(model_.roads[i]));
            }
            model_.roads = std::move(kept_roads);
            endpoint_map_ = build_endpoint_map(model_.roads);
            junction_nodes_ = find_junction_nodes(endpoint_map_);
        }
    }

    std::unordered_map<std::int64_t, std::vector<std::int64_t>, EndpointKeyHash> cluster_groups;
    for (const auto& [node, root] : parent) cluster_groups[find_root(node)].push_back(node);

    std::unordered_map<std::int64_t, std::int64_t, EndpointKeyHash> node_to_canonical;
    int compound_cluster_count = 0;
    int compound_cluster_node_total = 0;
    for (auto& [root, members] : cluster_groups) {
        std::size_t total_degree = 0;
        for (const auto& m : members) {
            const auto it = endpoint_map_.find(m);
            if (it != endpoint_map_.end()) total_degree += it->second.size();
        }
        if (total_degree < static_cast<std::size_t>(options_.junction_degree)) continue;
        const std::int64_t canonical = *std::min_element(members.begin(), members.end());
        cluster_members_[canonical] = members;
        for (const auto& m : members) node_to_canonical[m] = canonical;
        if (members.size() > 1) {
            ++compound_cluster_count;
            compound_cluster_node_total += static_cast<int>(members.size());
        }
    }
    for (const auto& [node, canonical] : node_to_canonical) node_to_junction_id_[node] = "j_" + std::to_string(canonical);
    model_.compound_junction_count = compound_cluster_count;
    model_.compound_junction_node_total = compound_cluster_node_total;
}

// Wraps apply_end_trim to also remember, per road index, the shape the road had before its first
// trim and how much has been cut from its start -- see pre_trim_points_/trimmed_from_start_.
void ModelBuilder::apply_tracked_trim(const std::size_t road_index, const bool at_start, const double applied) {
    if (applied <= 1e-6) return;
    if (!pre_trim_points_.count(road_index)) pre_trim_points_[road_index] = model_.roads[road_index].points;
    if (at_start) trimmed_from_start_[road_index] += applied;
    apply_end_trim(model_.roads[road_index], at_start, applied);
}

void ModelBuilder::link_plain_roads() {
    auto expand_bounds = [&](const geo::Vec2& p) {
        model_.north = std::max(model_.north, p.y);
        model_.south = std::min(model_.south, p.y);
        model_.east = std::max(model_.east, p.x);
        model_.west = std::min(model_.west, p.x);
    };
    auto set_successor_links = [](model::LanePlan& lanes, const std::vector<std::pair<int, int>>& pairs) {
        for (const auto& [real_id, bridge_id] : pairs) {
            for (auto& l : lanes.left) if (l.id == real_id) l.link_successor_id = bridge_id;
            for (auto& l : lanes.right) if (l.id == real_id) l.link_successor_id = bridge_id;
        }
    };
    auto set_predecessor_links = [](model::LanePlan& lanes, const std::vector<std::pair<int, int>>& pairs) {
        for (const auto& [bridge_id, real_id] : pairs) {
            for (auto& l : lanes.left) if (l.id == real_id) l.link_predecessor_id = bridge_id;
            for (auto& l : lanes.right) if (l.id == real_id) l.link_predecessor_id = bridge_id;
        }
    };
    auto direct_link = [&](const model::EndpointRef& a, const model::EndpointRef& b) {
        auto& road_a = model_.roads[a.road_index];
        auto& road_b = model_.roads[b.road_index];
        const auto cp_b = contact_point_of(b.at_start);
        const auto cp_a = contact_point_of(a.at_start);
        if (a.at_start) road_a.predecessor_xml = make_road_link_xml("road", road_b.id, cp_b);
        else road_a.successor_xml = make_road_link_xml("road", road_b.id, cp_b);
        if (b.at_start) road_b.predecessor_xml = make_road_link_xml("road", road_a.id, cp_a);
        else road_b.successor_xml = make_road_link_xml("road", road_a.id, cp_a);
        link_plain_road_lanes(road_a, a.at_start, road_b, b.at_start, options_.left_hand_traffic);
    };
    auto build_bridge = [&](const model::EndpointRef& a, const model::EndpointRef& b, LaneCountBridgePlan plan) {
        apply_tracked_trim(a.road_index, a.at_start, plan.trim_a);
        apply_tracked_trim(b.road_index, b.at_start, plan.trim_b);

        auto& road_a = model_.roads[a.road_index];
        auto& road_b = model_.roads[b.road_index];
        // Whichever road is upstream (at_start == false, the one that used to get .successor_xml
        // in a plain direct link) continues into the bridge's own start; the downstream one
        // (at_start == true) continues from the bridge's own end -- see plan_lane_count_bridge.
        if (a.at_start) road_a.predecessor_xml = make_road_link_xml("road", plan.bridge.id, "end");
        else road_a.successor_xml = make_road_link_xml("road", plan.bridge.id, "start");
        if (b.at_start) road_b.predecessor_xml = make_road_link_xml("road", plan.bridge.id, "end");
        else road_b.successor_xml = make_road_link_xml("road", plan.bridge.id, "start");
        if (plan.a_is_upstream) {
            set_successor_links(lanes_at_end_mut(road_a, a.at_start), plan.upstream_links);
            set_predecessor_links(lanes_at_end_mut(road_b, b.at_start), plan.downstream_links);
        } else {
            set_successor_links(lanes_at_end_mut(road_b, b.at_start), plan.upstream_links);
            set_predecessor_links(lanes_at_end_mut(road_a, a.at_start), plan.downstream_links);
        }

        for (const auto& g : plan.bridge.explicit_geometry) {
            expand_bounds({g.x, g.y});
            if (g.kind == model::GeomKind::Arc) expand_bounds(evaluate_geometry_point(g, g.length * 0.5));
            expand_bounds(evaluate_geometry_point(g, g.length));
        }
        model_.warnings.push_back("Inserted lane-count reconciliation road " + plan.bridge.id +
                                   " between " + road_a.id + " and " + road_b.id + ".");
        model_.roads.push_back(std::move(plan.bridge));
    };

    // Phase 1: decide, once, which plain boundaries will get a lane-count bridge -- purely from
    // lane *topology*, on the original unmutated roads. This has to happen before any lane_offset
    // reconciliation runs (phase 2 below): a bridge boundary is exactly the case where the two
    // sides' cross-sections are genuinely different and *shouldn't* be forced to match, so
    // reconciliation must know in advance to leave these nodes alone rather than undo them.
    std::unordered_set<std::int64_t> bridge_nodes;
    if (options_.bridge_lane_count_changes) {
        for (const auto& [node, endpoints] : endpoint_map_) {
            if (endpoints.size() != 2 || node_to_junction_id_.count(node)) continue;
            const auto& a = endpoints[0];
            const auto& b = endpoints[1];
            if (plan_lane_count_bridge(model_.roads[a.road_index], a.at_start,
                                        model_.roads[b.road_index], b.at_start, options_)) {
                bridge_nodes.insert(node);
            }
        }
    }

    // Phase 2: direct links + lane linking for every boundary not becoming a bridge (bridge_nodes
    // are handled in phase 3 instead, once lane_offset below has stabilized), then the existing
    // cross-chain lane_offset reconciliation.
    for (const auto& [node, endpoints] : endpoint_map_) {
        if (node_to_junction_id_.count(node)) {
            const std::string& junction_id = node_to_junction_id_.at(node);
            for (const auto& ep : endpoints) {
                auto& road = model_.roads[ep.road_index];
                if (ep.at_start) road.predecessor_xml = make_road_link_xml("junction", junction_id, "");
                else road.successor_xml = make_road_link_xml("junction", junction_id, "");
            }
            continue;
        }
        if (bridge_nodes.count(node)) continue;
        if (endpoints.size() == 2) direct_link(endpoints[0], endpoints[1]);
    }

    // A merged road whose lane_offset drifted from a plain (same-count) width/type-driven recompute
    // -- because a lane split/merge taper elsewhere in its chain anchored it to keep unaffected
    // lanes' absolute position fixed instead -- must hand that actual value off to whichever
    // separate road continues from it, rather than let that neighbor keep its own
    // independently-computed value and silently create a lateral jump at the boundary. Only the
    // "one road ends here, the other starts here" case (a straight continuation, s-direction
    // flowing through) is reconciled; the rarer "both roads end/start here" cross-connect case is
    // left as each side's own independently-computed value, same as before this reconciliation
    // existed. A single pass over `endpoint_map` (an unordered_map, so its iteration order need not
    // follow the roads' physical chain order) can leave an a->b->c chain of plain boundaries only
    // half-corrected if b->c happens to be visited before a->b; repeating until a full pass makes
    // no further change propagates a correction through a chain of any length. Bounded by the
    // number of roads (the longest possible acyclic chain of plain boundaries) so a closed loop of
    // plain-linked roads -- a ring with no junction or feature-split node anywhere on it, unusual
    // but not impossible -- can't spin forever chasing floating-point noise below the threshold.
    for (std::size_t pass = 0, limit = model_.roads.size() + 1; pass < limit; ++pass) {
        bool changed = false;
        for (const auto& [node, endpoints] : endpoint_map_) {
            if (endpoints.size() != 2 || node_to_junction_id_.count(node) || bridge_nodes.count(node)) continue;
            const auto& a = endpoints[0];
            const auto& b = endpoints[1];
            if (a.at_start == b.at_start) continue; // cross-connect case, not reconciled
            auto& road_a = model_.roads[a.road_index];
            auto& road_b = model_.roads[b.road_index];
            auto& target = !a.at_start ? lanes_at_end_mut(road_b, b.at_start) : lanes_at_end_mut(road_a, a.at_start);
            const double source_end = !a.at_start ? lane_offset_at_road_end(road_a, a.at_start) : lane_offset_at_road_end(road_b, b.at_start);
            if (std::abs(target.lane_offset - source_end) > 1e-9) {
                target.lane_offset = source_end;
                changed = true;
            }
        }
        if (!changed) break;
    }

    // Phase 3: every non-bridged boundary's lane_offset is now stable, so build the actual bridges
    // (recomputing each plan from scratch rather than reusing phase 1's, since phase 1 only ever
    // needed to know *whether* one applies -- the geometry/topology decision can't change between
    // phase 1 and here, only the lane_offset values it reads, which is exactly what needed to
    // settle first). A plan failing this time despite succeeding in phase 1 shouldn't happen (only
    // this boundary's own two roads' lane_offset can have moved, not the geometry/budget guards),
    // but falls back to a direct link rather than silently dropping the boundary if it ever did.
    for (const auto node : bridge_nodes) {
        const auto& endpoints = endpoint_map_.at(node);
        const auto& a = endpoints[0];
        const auto& b = endpoints[1];
        auto plan = plan_lane_count_bridge(model_.roads[a.road_index], a.at_start,
                                            model_.roads[b.road_index], b.at_start, options_);
        if (plan) build_bridge(a, b, std::move(*plan));
        else direct_link(a, b);
    }
}

void ModelBuilder::build_junction_connectors() {
    // Pass 1: enumerate every incoming-lane -> outgoing-lane movement at every junction and work
    // out, for each, the tangent-fillet setback (b_in/b_out) needed to insert a curved connector
    // road of an appropriate turn radius without disturbing the lane's heading. A road end may
    // serve several movements (a 4-way crossing has 3 per approach); the amount actually trimmed
    // off that road is the *maximum* setback demanded by any of them.
    //
    // The setback is capped by a "budget" per road end: at most 90% of the OSM polyline segment
    // immediately adjacent to the junction (so a trim never crosses an original bend, which would
    // change the road's true heading at the cut) and at most 45% of the road's total length (so
    // two junctions close together on one road each get a fair share). When even a reduced turn
    // radius cannot fit inside that budget, the movement falls back to a direct road-to-road
    // junction connection (the previous behavior) instead of a curved connector.
    std::unordered_map<std::size_t, double> budget_by_endpoint;
    for (const auto& [node, endpoints] : endpoint_map_) {
        if (!node_to_junction_id_.count(node)) continue;
        for (const auto& ep : endpoints) {
            const auto& road = model_.roads[ep.road_index];
            const double budget = std::min(0.9 * segment_length_at_end(road, ep.at_start), 0.45 * road.length);
            budget_by_endpoint[endpoint_key(ep.road_index, ep.at_start)] = std::max(0.0, budget);
        }
    }

    std::vector<PendingConnector> pending;
    std::unordered_map<std::size_t, double> trim_by_endpoint;

    for (const auto& [canonical, members] : cluster_members_) {
        std::vector<model::EndpointRef> endpoints;
        for (const auto& member : members) {
            const auto it = endpoint_map_.find(member);
            if (it == endpoint_map_.end()) continue;
            endpoints.insert(endpoints.end(), it->second.begin(), it->second.end());
        }
        const std::string junction_id = "j_" + std::to_string(canonical);
        for (const auto& incoming : endpoints) {
            const auto& in_road = model_.roads[incoming.road_index];
            const auto incoming_lanes = incoming_lane_ids(in_road, incoming.at_start, options_);
            if (incoming_lanes.empty()) continue;
            const auto dir_in = direction_into_junction(in_road, incoming.at_start);

            for (const auto& outgoing : endpoints) {
                if (incoming.road_index == outgoing.road_index) continue;
                const auto& out_road = model_.roads[outgoing.road_index];
                const auto outgoing_lanes = outgoing_lane_ids(out_road, outgoing.at_start, options_);
                if (outgoing_lanes.empty()) continue;
                const auto dir_out = direction_away_from_junction(out_road, outgoing.at_start);

                const double signed_delta = std::atan2(geo::cross(dir_in, dir_out), geo::dot(dir_in, dir_out));
                const double abs_delta = std::min(std::abs(signed_delta), geo::kPi - 0.001);

                // Classification (which turn bucket this movement is, for matching against a
                // lane's turn:lanes-derived permitted set) uses a longer-range look-ahead direction,
                // not the immediate dir_in/dir_out above: a mapped ramp/slip-lane that keeps curving
                // well past its first short sub-segment would otherwise read as "through" simply
                // because it starts out nearly straight. The connector's own geometry (pc.signed_delta
                // below) still uses the immediate dir_in/dir_out -- it must, for exact position/
                // heading continuity at the seam -- only the bucket used for turn:lanes filtering
                // benefits from looking further ahead.
                constexpr double kClassificationLookahead = 15.0;
                const auto classification_dir_in = classification_direction_into_junction(in_road, incoming.at_start, kClassificationLookahead);
                const auto classification_dir_out = classification_direction_away_from_junction(out_road, outgoing.at_start, kClassificationLookahead);
                const double classification_delta = std::atan2(geo::cross(classification_dir_in, classification_dir_out),
                                                                 geo::dot(classification_dir_in, classification_dir_out));
                const double tan_half = std::tan(abs_delta / 2.0);
                const double radius_nominal = std::max(
                    infer::turn_radius_for_highway(tag_value_or(tags_at_end(in_road, incoming.at_start), "highway", "road"), options_),
                    infer::turn_radius_for_highway(tag_value_or(tags_at_end(out_road, outgoing.at_start), "highway", "road"), options_));

                const std::size_t in_key = endpoint_key(incoming.road_index, incoming.at_start);
                const std::size_t out_key = endpoint_key(outgoing.road_index, outgoing.at_start);
                const double budget_in = budget_by_endpoint.count(in_key) ? budget_by_endpoint[in_key] : 0.0;
                const double budget_out = budget_by_endpoint.count(out_key) ? budget_by_endpoint[out_key] : 0.0;

                // A lane with an OSM turn:lanes restriction may only serve movements whose
                // geometric direction matches its permitted set (e.g. a left-turn-only lane must
                // not connect to a through destination); unrestricted lanes (the common case)
                // match every movement, so this reduces to today's plain positional pairing
                // whenever no lane at this junction has turn:lanes data.
                const std::string movement_bucket = turn_bucket_for_delta(classification_delta);
                // k must be positional among only the lanes that actually survive the movement
                // filter below, not the raw loop index i -- otherwise a filtered-out lane earlier
                // in incoming_lanes (e.g. a left-turn-only lane skipped for a "through" movement)
                // leaves every surviving lane's own i one higher than its position among
                // survivors, so the *last* two (or more) surviving lanes both clamp to
                // outgoing_lanes.size()-1 and collide on the same outgoing lane while an earlier
                // outgoing lane goes unclaimed, even though the surviving incoming lane count
                // matches the outgoing count exactly and a clean 1:1 pairing exists.
                std::size_t allowed_index = 0;
                for (std::size_t i = 0; i < incoming_lanes.size(); ++i) {
                    if (!lane_allows_movement(in_road, incoming.at_start, incoming_lanes[i], movement_bucket, classification_delta)) continue;
                    const std::size_t k = std::min(allowed_index, outgoing_lanes.size() - 1);
                    ++allowed_index;
                    PendingConnector pc;
                    pc.in_road_index = incoming.road_index;
                    pc.in_at_start = incoming.at_start;
                    pc.out_road_index = outgoing.road_index;
                    pc.out_at_start = outgoing.at_start;
                    pc.incoming_lane_id = incoming_lanes[i];
                    pc.outgoing_lane_id = outgoing_lanes[k];
                    pc.dir_in = dir_in;
                    pc.dir_out = dir_out;
                    pc.a_in = lane_world_point(in_road, incoming.at_start, pc.incoming_lane_id);
                    pc.a_out = lane_world_point(out_road, outgoing.at_start, pc.outgoing_lane_id);
                    pc.signed_delta = signed_delta;
                    pc.junction_id = junction_id;
                    pc.node_ref = canonical;
                    pc.junction_point = endpoint_point(in_road, incoming.at_start);

                    // Near-180 degree movements (the rays are close to anti-parallel) make the
                    // line-intersection ill-conditioned: the intersection point can land very far
                    // away, producing enormous |s_in|/|s_out| and, after flooring the radius, a
                    // tangent length that no longer bears any relation to the actual gap between
                    // the roads. Rather than let that slip through as a "feasible" connector with
                    // absurd straight run-in/run-out (b_in/b_out deeply negative but still above
                    // an ad-hoc sanity floor), reject it outright and fall back to a direct link,
                    // the same way an exactly-antiparallel pair already does.
                    const auto inter = geo::line_intersect_params(pc.a_in, dir_in, pc.a_out, dir_out);
                    if (!inter || abs_delta > geo::deg_to_rad(160.0)) {
                        // Directions coincide (near-parallel rays, so line_intersect_params can't
                        // find a usable PI). A "just two straight segments, no fillet" connector is
                        // only valid if the rays are also positionally aligned -- b_in=b_out=0
                        // assumes zero lateral gap between them; a nearly-parallel but laterally
                        // offset pair (e.g. two lanes that don't quite line up straight across a
                        // wide compound junction) would silently kink without actually reaching the
                        // destination lane, so that case must fall back instead, not be accepted.
                        const double lateral_gap = std::abs(geo::cross(dir_in, pc.a_out - pc.a_in));
                        if (geo::dot(dir_in, dir_out) > 0.0 && lateral_gap < 1e-4) {
                            pc.radius = 0.0;
                            pc.b_in = 0.0;
                            pc.b_out = 0.0;
                            pc.feasible = true;
                        } else {
                            pc.feasible = false; // near-180 reversal, or laterally offset near-parallel rays
                        }
                    } else {
                        const auto [s_in, s_out] = *inter;
                        double radius = radius_nominal;
                        if (tan_half > 1e-9) {
                            const double radius_cap_in = (budget_in + s_in) / tan_half;
                            const double radius_cap_out = (budget_out + s_out) / tan_half;
                            radius = std::min({radius, radius_cap_in, radius_cap_out});
                        }
                        // A radius much below this reads as a sharp, unrealistic kink rather than
                        // a curve (no real road turns this tight); better to fall back to a direct
                        // link (checked via the feasibility test just below) than draw one.
                        if (radius < 3.0) radius = 3.0;
                        const double tangent_length = radius * tan_half;
                        pc.radius = radius;
                        pc.b_in = tangent_length - s_in;
                        pc.b_out = tangent_length + s_out;
                        pc.feasible = pc.b_in <= budget_in + 1e-6 && pc.b_out <= budget_out + 1e-6 &&
                                      pc.b_in >= -100.0 && pc.b_out >= -100.0;
                    }

                    // Every movement claims whatever budget it needs at each endpoint, capped to
                    // what's actually available there -- not just feasible ones. A movement can be
                    // "infeasible" purely because its own ideal (or floor-forced) radius exceeded
                    // its own budget, while budget was still genuinely available at that endpoint;
                    // if only feasible movements ever bumped trim_by_endpoint, an infeasible
                    // movement that's the *sole* user of an endpoint would leave it completely
                    // untrimmed (trim=0) even when up to `budget_in`/`budget_out` was available,
                    // starving pass 2's refit of any room to rescue it into a real connector and
                    // forcing a fallback to a direct link with a visible heading kink. Capping to
                    // budget (rather than the movement's raw, possibly wildly excessive b_in/b_out)
                    // means a hopelessly awkward movement just claims its full budget harmlessly,
                    // same as it would if it were feasible with that budget exactly.
                    auto bump = [&](const std::size_t key, const double b, const double budget) {
                        auto it = trim_by_endpoint.find(key);
                        const double v = std::max(0.0, std::min(b, budget));
                        if (it == trim_by_endpoint.end() || it->second < v) trim_by_endpoint[key] = v;
                    };
                    bump(in_key, pc.b_in, budget_in);
                    bump(out_key, pc.b_out, budget_out);
                    pending.push_back(pc);
                }
            }
        }
    }

    // Apply the final (budget-capped) trims to the real road geometries.
    std::unordered_map<std::size_t, double> applied_trim;
    for (const auto& [key, trim] : trim_by_endpoint) {
        const double budget = budget_by_endpoint.count(key) ? budget_by_endpoint[key] : 0.0;
        const double applied = std::min(trim, budget);
        applied_trim[key] = applied;
        if (applied <= 1e-6) continue;
        const std::size_t road_index = key / 2;
        const bool at_start = (key % 2) == 1;
        apply_tracked_trim(road_index, at_start, applied);
    }

    // Pass 2: build the actual connector roads (or, for infeasible movements, a direct fallback
    // junction connection) now that every road end's final trim is known.
    std::unordered_map<std::string, std::size_t> junction_index_of;
    auto get_or_create_junction = [&](const PendingConnector& pc) -> model::Junction& {
        const auto it = junction_index_of.find(pc.junction_id);
        if (it != junction_index_of.end()) return model_.junctions[it->second];
        model::Junction j;
        j.id = pc.junction_id;
        j.node_ref = pc.node_ref;
        j.point = pc.junction_point;
        model_.junctions.push_back(std::move(j));
        junction_index_of[pc.junction_id] = model_.junctions.size() - 1;
        return model_.junctions.back();
    };

    auto expand_bounds = [&](const geo::Vec2& p) {
        model_.north = std::max(model_.north, p.y);
        model_.south = std::min(model_.south, p.y);
        model_.east = std::max(model_.east, p.x);
        model_.west = std::min(model_.west, p.x);
    };

    int connector_counter = 0;

    for (const auto& pc : pending) {
        // Captured by value: model_.roads is appended to below (a new connector road per feasible
        // pending item), which can reallocate the vector and invalidate references/iterators into
        // it, so `in_road`/`out_road` must not be held across that push_back.
        const auto& in_road = model_.roads[pc.in_road_index];
        const auto& out_road = model_.roads[pc.out_road_index];
        const std::string in_road_id = in_road.id;
        const std::string out_road_id = out_road.id;
        const double in_lane_width = lane_width_of(in_road, pc.in_at_start, pc.incoming_lane_id);
        const double out_lane_width = lane_width_of(out_road, pc.out_at_start, pc.outgoing_lane_id);
        const std::string cp_in = contact_point_of(pc.in_at_start);
        const std::string cp_out = contact_point_of(pc.out_at_start);
        auto& junction = get_or_create_junction(pc);

        model::JunctionConnection c;
        c.id = std::to_string(junction.connections.size());

        const double trim_in = applied_trim.count(endpoint_key(pc.in_road_index, pc.in_at_start))
            ? applied_trim[endpoint_key(pc.in_road_index, pc.in_at_start)] : 0.0;
        const double trim_out = applied_trim.count(endpoint_key(pc.out_road_index, pc.out_at_start))
            ? applied_trim[endpoint_key(pc.out_road_index, pc.out_at_start)] : 0.0;

        if (!pc.feasible) {
            // A movement can be "infeasible" (no nice fillet fits its own per-movement budget)
            // while another movement sharing the same road end still forces that end to be
            // trimmed back regardless. A bare road-to-road link implicitly assumes the two roads
            // are still directly adjacent (zero gap); if either end was actually trimmed, that
            // assumption is false and the link would point at a stale, no-longer-adjacent
            // position. Zero trim only guarantees *position* is exact (a_in/a_out are the real,
            // untrimmed endpoints) -- it says nothing about whether the two roads' own headings
            // actually agree there, since a movement can be infeasible purely because its own
            // budget was too small for any real curve, independent of whether anyone else also
            // needed trim at that end. So the shortcut additionally requires the immediate angle
            // itself to be small (roughly "through"): otherwise fall through and build an explicit
            // connector (or, if nothing better fits, the same direct-bridge stub used for the
            // trimmed case) rather than silently link two roads whose headings visibly kink.
            if (trim_in <= 1e-6 && trim_out <= 1e-6 && std::abs(pc.signed_delta) < geo::deg_to_rad(20.0)) {
                ++direct_fallback_count_;
                c.incoming_road = in_road_id;
                c.connecting_road = out_road_id;
                c.contact_point = cp_out;
                c.lane_links.emplace_back(pc.incoming_lane_id, pc.outgoing_lane_id);
                junction.connections.push_back(std::move(c));
                continue;
            }
        }

        // pc.radius/b_in/b_out were sized in pass 1 against this movement's own budget. If the
        // road ends were trimmed back further anyway (another movement sharing the same endpoint
        // needed more room), re-fit the radius against that final trim instead of leaving it as
        // straight run-in/run-out padding -- same tangent math as pass 1, just against the final
        // trim rather than the raw per-movement budget. This always grows the radius for a
        // movement that already fit its own (smaller) budget (trim_in/trim_out are provably >=
        // this movement's own b_in/b_out, since it was one of the contributors to the endpoint's
        // applied trim, so the refit below can only come out >= the original radius there) --
        // turning a shared endpoint's "spare" setback into a longer, gentler curve. For a movement
        // that was marked infeasible for exceeding its own budget, this may instead *shrink* the
        // radius to whatever the endpoint's final trim (possibly widened by a different movement,
        // but not necessarily as much as this one originally wanted) can actually support,
        // rescuing it into a real curve instead of a stale direct link that no longer reaches its
        // target because some other movement at the same end forced a trim it didn't account for.
        double radius = pc.radius;
        double b_in = pc.b_in;
        double b_out = pc.b_out;
        // Whether `radius` above actually corresponds to a fillet that reaches phys_out: false
        // either when pass 1 never solved one at all (radius <= 0, the near-180/laterally-offset
        // case), or when the refit below finds that not even a small curve fits the final trim
        // (radius_cap_in/out come out non-positive, e.g. a movement whose own geometry was so
        // awkward that s_in/s_out are large negative numbers no realistic trim could offset) --
        // in either case run_in/arc_length/run_out further down must not be trusted, since they'd
        // be built from a radius that no longer matches the b_in/b_out used to derive phys_in/out.
        bool fitted = radius > 0.0;
        if (fitted) {
            const double abs_delta = std::min(std::abs(pc.signed_delta), geo::kPi - 0.001);
            const double tan_half = std::tan(abs_delta / 2.0);
            if (tan_half > 1e-9) {
                const double s_in = radius * tan_half - b_in;
                const double s_out = b_out - radius * tan_half;
                const double radius_nominal = std::max(
                    infer::turn_radius_for_highway(tag_value_or(tags_at_end(in_road, pc.in_at_start), "highway", "road"), options_),
                    infer::turn_radius_for_highway(tag_value_or(tags_at_end(out_road, pc.out_at_start), "highway", "road"), options_));
                const double radius_cap_in = (trim_in + s_in) / tan_half;
                const double radius_cap_out = (trim_out - s_out) / tan_half;
                const double refit = std::min({radius_nominal, radius_cap_in, radius_cap_out});
                // Floored well below pass 1's "no unrealistic kinks" 3.0m floor: unlike pass 1
                // (which can fall back to a direct link when nothing reasonable fits), this is the
                // last chance to produce a curve at all, so a tight-but-continuous one beats a
                // discontinuous one -- but only down to a point; below that, forcing the floor
                // would use a radius inconsistent with the b_in/b_out actually available; a direct
                // bridge is more correct.
                if (refit >= 1.0) {
                    radius = refit;
                    b_in = radius * tan_half - s_in;
                    b_out = radius * tan_half + s_out;
                } else {
                    fitted = false;
                }
            }
        }

        const double run_in = std::max(0.0, trim_in - b_in);
        const double run_out = std::max(0.0, trim_out - b_out);
        const double arc_length = radius * std::abs(pc.signed_delta);

        const geo::Vec2 phys_in = pc.a_in - pc.dir_in * trim_in;
        const geo::Vec2 phys_out = pc.a_out + pc.dir_out * trim_out;

        std::vector<model::GeomPrimitive> geoms;
        // Without a fitted radius (see `fitted` above), the run_in-then-run_out construction below
        // has no arc to bridge a direction change, so it only lands on phys_out when a_in and a_out
        // are the same point -- not true in general, regardless of whether any trim was applied:
        // pc.radius/b_in/b_out can already be stale/unfit even at zero trim (a movement can be
        // infeasible purely because its own budget was too small for any real curve, independent of
        // whether trimming happened at all). Go straight to the direct bridge in every unfitted
        // case; when phys_in/phys_out happen to coincide too (the common zero-trim, aligned case),
        // this produces the exact same result as the geoms.empty() stub further below would anyway.
        const bool needs_direct_bridge = !fitted;
        if (needs_direct_bridge) {
            const double gap = geo::length(phys_out - phys_in);
            const double bridge_hdg = gap > 1e-6 ? geo::heading(phys_in, phys_out) : std::atan2(pc.dir_in.y, pc.dir_in.x);

            // The chord direction between phys_in and phys_out can, for a large lateral-vs-
            // longitudinal gap ratio, end up nowhere near either the incoming or outgoing lane's
            // own heading -- not a plausible through/slight-turn shape, just an artifact of an
            // extreme, near-reversed pairing. hermite_bezier_segment below matches both endpoint
            // headings exactly regardless (unlike a single straight line, which can only assert
            // one direction), but a Bezier built from two wildly diverging tangents over a short
            // gap can loop or double back on itself rather than reading as a real road shape, so
            // this sanity check still gates whether to attempt a bridge at all, not (as it used to)
            // whether the single heading it could offer was close enough to both ends.
            auto angle_diff = [](const double a, const double b) {
                double d = std::fmod(a - b + geo::kPi, 2.0 * geo::kPi);
                if (d < 0.0) d += 2.0 * geo::kPi;
                return std::abs(d - geo::kPi);
            };
            const double in_hdg = std::atan2(pc.dir_in.y, pc.dir_in.x);
            const double out_hdg = std::atan2(pc.dir_out.y, pc.dir_out.x);
            if (angle_diff(bridge_hdg, in_hdg) > geo::deg_to_rad(45.0) ||
                angle_diff(bridge_hdg, out_hdg) > geo::deg_to_rad(45.0)) {
                ++direct_fallback_count_;
                c.incoming_road = in_road_id;
                c.connecting_road = out_road_id;
                c.contact_point = cp_out;
                c.lane_links.emplace_back(pc.incoming_lane_id, pc.outgoing_lane_id);
                junction.connections.push_back(std::move(c));
                continue;
            }

            model::GeomPrimitive g = hermite_bezier_segment(phys_in, pc.dir_in, phys_out, pc.dir_out);
            g.length = std::max(1e-4, g.length);
            geoms.push_back(g);
        } else {
            geo::Vec2 cursor = phys_in;
            if (run_in > 1e-4) {
                model::GeomPrimitive g;
                g.x = cursor.x; g.y = cursor.y;
                g.hdg = std::atan2(pc.dir_in.y, pc.dir_in.x);
                g.length = run_in;
                g.curvature = 0.0;
                g.kind = model::GeomKind::Line;
                geoms.push_back(g);
                cursor = cursor + pc.dir_in * run_in;
            }
            if (arc_length > 1e-4) {
                model::GeomPrimitive g;
                g.x = cursor.x; g.y = cursor.y;
                g.hdg = std::atan2(pc.dir_in.y, pc.dir_in.x);
                g.length = arc_length;
                g.curvature = (pc.signed_delta >= 0.0 ? 1.0 : -1.0) / radius;
                g.kind = model::GeomKind::Arc;
                geoms.push_back(g);
                cursor = evaluate_geometry_point(g, arc_length);
            }
            if (run_out > 1e-4) {
                model::GeomPrimitive g;
                g.x = cursor.x; g.y = cursor.y;
                g.hdg = std::atan2(pc.dir_out.y, pc.dir_out.x);
                g.length = run_out;
                g.curvature = 0.0;
                g.kind = model::GeomKind::Line;
                geoms.push_back(g);
                cursor = cursor + pc.dir_out * run_out;
            }
        }
        if (geoms.empty()) {
            // No line/arc segment was long enough to bother emitting (run_in, arc_length, and
            // run_out were all ~0) -- phys_in and phys_out are themselves almost coincident. Point
            // straight at phys_out (not pc.dir_in, which need not be aligned with whatever tiny
            // gap remains) so this stub doesn't overshoot in the wrong direction; only fall back to
            // dir_in when the two points are truly coincident and no direction is derivable.
            const double gap = geo::length(phys_out - phys_in);
            model::GeomPrimitive g;
            g.x = phys_in.x; g.y = phys_in.y;
            g.hdg = gap > 1e-6 ? geo::heading(phys_in, phys_out) : std::atan2(pc.dir_in.y, pc.dir_in.x);
            g.length = std::max(1e-4, gap);
            g.curvature = 0.0;
            g.kind = model::GeomKind::Line;
            geoms.push_back(g);
        }

        double total_length = 0.0;
        for (const auto& g : geoms) total_length += g.length;

        model::RoadSegment connector;
        connector.id = pc.junction_id + "_c" + std::to_string(connector_counter++);
        connector.junction_id = pc.junction_id;
        connector.length = total_length;
        connector.explicit_geometry = geoms;
        connector.predecessor_xml = make_road_link_xml("road", in_road_id, cp_in);
        connector.successor_xml = make_road_link_xml("road", out_road_id, cp_out);
        connector.lanes.center_mark = "none";
        // explicit_geometry was built to trace the driving lane's own path exactly (so it lines up
        // with the incoming/outgoing lane centerlines). But a lane's centerline is normally offset
        // from its road's reference line by half its width (OpenDRIVE lane stacking), so without
        // correction the actual drivable lane would sit half a lane width off to the side of this
        // path. laneOffset(s) = width(s)/2 cancels that out for every s, including through the
        // linear width taper when in/out widths differ.
        connector.lanes.lane_offset = in_lane_width / 2.0;
        connector.lanes.lane_offset_slope = total_length > 1e-6 ? (out_lane_width - in_lane_width) / (2.0 * total_length) : 0.0;

        model::LaneSpec lane;
        lane.id = -1;
        lane.type = "driving";
        lane.width = in_lane_width;
        lane.width_end = out_lane_width;
        lane.roadmark_type = "none";
        lane.roadmark_weight = "standard";
        lane.roadmark_color = "standard";
        lane.lane_change = "none";
        lane.link_predecessor_id = pc.incoming_lane_id;
        lane.link_successor_id = pc.outgoing_lane_id;
        connector.lanes.right.push_back(lane);

        for (const auto& g : connector.explicit_geometry) {
            expand_bounds({g.x, g.y});
            if (g.kind == model::GeomKind::Arc) expand_bounds(evaluate_geometry_point(g, g.length * 0.5));
            expand_bounds(evaluate_geometry_point(g, g.length));
        }

        model_.roads.push_back(std::move(connector));

        c.incoming_road = in_road_id;
        c.connecting_road = model_.roads.back().id;
        c.contact_point = "start";
        c.lane_links.emplace_back(pc.incoming_lane_id, -1);
        junction.connections.push_back(std::move(c));
    }
}

void ModelBuilder::place_signals() {
    // Attach node-level traffic controls/signs to nearest road.
    int signal_id = 0;
    for (const auto& pf : parsed_.point_features) {
        const auto p = parsed_.projector.project(pf.ll.lat, pf.ll.lon);
        double best_distance = std::numeric_limits<double>::max();
        std::size_t best_road = 0;
        geo::ProjectionOnPolyline best_projection;
        for (std::size_t i = 0; i < model_.roads.size(); ++i) {
            // Match against the road's pre-trim shape if a junction connector or lane-count bridge
            // later pulled its end back (see pre_trim_points_) -- otherwise a point that used to
            // sit right at that road's original end would lose its real nearest-road match and
            // fall through to whatever else happens to be close, sometimes with a large, wrong
            // lateral offset (see apply_tracked_trim).
            const auto it = pre_trim_points_.find(i);
            const auto& match_points = it != pre_trim_points_.end() ? it->second : model_.roads[i].points;
            const auto proj = geo::project_to_polyline(match_points, p);
            if (proj.distance < best_distance) {
                best_distance = proj.distance;
                best_road = i;
                best_projection = proj;
            }
        }
        if (model_.roads.empty() || best_distance > options_.signal_search_radius) {
            model_.warnings.push_back("Point feature node " + std::to_string(pf.node_ref) + " (" + pf.kind + ") was not matched to a road within the search radius.");
            continue;
        }
        // best_projection.s is relative to the pre-trim shape; rebase past any start-trim (which
        // shifted s=0 forward) and clamp into the road's final length (an end-trim just drops the
        // tail, so a point that landed there now sits right at the new, closer end instead).
        double s = best_projection.s;
        const auto start_trim = trimmed_from_start_.find(best_road);
        if (start_trim != trimmed_from_start_.end()) s -= start_trim->second;
        s = std::clamp(s, 0.0, model_.roads[best_road].length);
        const std::string id = "sig_" + std::to_string(signal_id++);
        model_.roads[best_road].signals.push_back(infer::signal_from_point_feature(pf, id, s, best_projection.t));
    }

    // Add maxspeed as a static signal at the start of each cross-section that declares one; a
    // merged road with unchanged maxspeed across all its sections still gets just one signal.
    for (auto& road : model_.roads) {
        auto add_maxspeed_signal = [&](const Tags& tags, const double s_base, const double span, const std::string& id_suffix) {
            const auto maxspeed = tag_value(tags, "maxspeed");
            if (!maxspeed) return;
            const auto value = infer::parse_maxspeed(*maxspeed);
            if (!value) return;
            model::RoadSignal sig;
            sig.id = "maxspeed_" + road.id + id_suffix;
            sig.s = s_base + std::min(1.0, span * 0.1);
            sig.t = road.lanes.lane_offset == 0.0 ? -0.5 : road.lanes.lane_offset;
            sig.dynamic = false;
            sig.name = "maxspeed";
            sig.type = "speed";
            sig.subtype = "max";
            sig.unit = maxspeed->find("mph") != std::string::npos ? "mph" : "km/h";
            sig.value = *value;
            sig.has_value = true;
            sig.text = *maxspeed;
            road.signals.push_back(std::move(sig));
        };

        const double first_span = road.extra_lane_sections.empty() ? road.length : road.extra_lane_sections.front().s_offset;
        add_maxspeed_signal(road.tags, 0.0, first_span, "");
        std::optional<std::string> prev_maxspeed = tag_value(road.tags, "maxspeed");
        for (std::size_t i = 0; i < road.extra_lane_sections.size(); ++i) {
            const auto& section = road.extra_lane_sections[i];
            const auto this_maxspeed = tag_value(section.tags, "maxspeed");
            const double span = (i + 1 < road.extra_lane_sections.size())
                ? (road.extra_lane_sections[i + 1].s_offset - section.s_offset)
                : (road.length - section.s_offset);
            if (this_maxspeed != prev_maxspeed) {
                add_maxspeed_signal(section.tags, section.s_offset, span, "_s" + std::to_string(i));
            }
            prev_maxspeed = this_maxspeed;
        }
    }
}

void ModelBuilder::fit_curves() {
    // Junction connectors and lane-count bridges already populate their own explicit_geometry
    // (line/arc, built to align exactly with the roads they connect); only ordinary roads --
    // which so far only ever have `.points`, no explicit_geometry -- get a fitted curve here.
    for (auto& road : model_.roads) {
        if (!road.explicit_geometry.empty()) continue;
        road.explicit_geometry = fit_curve(road.points);

        for (const auto& g : road.explicit_geometry) {
            if (g.kind != model::GeomKind::ParamPoly3) continue;
            const double cos_h = std::cos(g.hdg), sin_h = std::sin(g.hdg);
            auto to_global = [&](const geo::Vec2& local) {
                return geo::Vec2{g.x + local.x * cos_h - local.y * sin_h, g.y + local.x * sin_h + local.y * cos_h};
            };
            for (const auto& local : {g.local_p1, g.local_p2, g.local_p3}) {
                const auto p = to_global(local);
                model_.north = std::max(model_.north, p.y);
                model_.south = std::min(model_.south, p.y);
                model_.east = std::max(model_.east, p.x);
                model_.west = std::min(model_.west, p.x);
            }
        }
    }
}

model::MapModel build_model(const osm::ParseResult& parsed, const Options& options) {
    return ModelBuilder(parsed, options).build();
}

} // namespace osm2xodr::build
