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

std::string road_type(const Tags& tags);

std::optional<double> parse_maxspeed(const std::string& v);

model::RoadSignal signal_from_point_feature(const osm::PointFeature& pf, const std::string& id, const double s, const double t);

} // namespace osm2xodr::infer
