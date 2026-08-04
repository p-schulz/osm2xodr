#pragma once

#include "osm2xodr/model.hpp"
#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/tags.hpp"

#include <optional>
#include <string>
#include <vector>

namespace osm2xodr::infer {

model::LanePlan infer_lanes(const Tags& tags, const Options& options, std::vector<std::string>& warnings, const std::string& road_id);

// Design turning radius (meters) for junction connector fillets, tiered by OSM highway class.
// Faster/wider road classes get a larger radius (vehicles need more room to turn without
// slowing to a crawl); minor/service roads get a tight radius. Unknown classes fall back to
// options.junction_turn_radius so the CLI flag still has an effect for unmapped tags.
double turn_radius_for_highway(const std::string& highway, const Options& options);

// Calibration-anchor "typical" running speed (km/h) for turn_radius_for_highway's own class
// groupings; nullopt for a class turn_radius_for_highway itself has no tier for.
std::optional<double> typical_speed_for_highway(const std::string& highway);

// Adaptive version of turn_radius_for_highway: starts from the same tier (max of both connected
// roads' class), then scales it by each side's actual OSM maxspeed (relative to its class's own
// typical_speed_for_highway anchor) and the connector's own deflection angle (abs_delta, radians),
// so a fast/gentle movement reads a bigger radius and a slow/sharp one reads a tighter one instead
// of every movement on a given road class getting an identical value. Falls back to the plain tier
// (matching turn_radius_for_highway exactly) when adaptation is disabled
// (options.adaptive_turn_radius), the governing class isn't in typical_speed_for_highway's table,
// or neither road has a usable maxspeed tag.
double adaptive_turn_radius(const std::string& in_highway, std::optional<double> in_maxspeed_kmh,
                             const std::string& out_highway, std::optional<double> out_maxspeed_kmh,
                             double abs_delta, const Options& options);

// Lane-count-change taper length (meters), modeled on the general German lane-taper/
// Verziehungslaenge convention (taper length scales with design speed and the width being shifted)
// -- an approximation of that general convention, not a verified reproduction of the current
// official RSA 21 (or RASt 06) table; consult those directly if exact regulatory compliance is
// required. Falls back to `fallback_length` (today's flat --lane-taper-length) when adaptation is
// disabled (options.adaptive_lane_taper) or no usable maxspeed tag is available.
double adaptive_taper_length(double lane_width, std::optional<double> maxspeed_kmh, double fallback_length, const Options& options);

std::string road_type(const Tags& tags);

std::optional<double> parse_maxspeed(const std::string& v);

// Vertical offset in meters from OSM's `layer=*` tag (e.g. layer=-1 -> a tunnel one level below
// ground, layer=1 -> a bridge one level above), at a fixed 3.5m per layer. 0.0 when the tag is
// absent or unparseable (the default/ground layer).
double elevation_offset(const Tags& tags);

model::RoadSignal signal_from_point_feature(const osm::PointFeature& pf, const std::string& id, const double s, const double t);

// A companion "stop_line" signal at the same s/t as a traffic_light signal: OSM has no separate tag
// for a distinct stop-line position, and a highway=traffic_signals node is conventionally placed
// right at the stop line already, so this just marks the same point as the other kind of thing.
model::RoadSignal stop_line_signal(const std::string& id, const double s, const double t);

// True for a traffic_light point feature that's actually pedestrian-facing, not a vehicle stop
// position: a highway=traffic_signals node tagged traffic_signals=pedestrian_crossing (a
// pedestrian-facing signal head at an otherwise vehicle intersection), or a highway=crossing node
// tagged crossing=traffic_signals (a dedicated, often standalone/mid-block pedestrian crossing
// signal). Both are still real traffic control devices -- signal_from_point_feature emits the same
// traffic_light signal for either -- they just don't mark where a vehicle must stop.
bool is_pedestrian_traffic_light(const Tags& tags);

} // namespace osm2xodr::infer
