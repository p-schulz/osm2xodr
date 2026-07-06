#include "osm2xodr/osm_parse.hpp"

#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>

#include <unordered_set>

namespace osm2xodr::osm {

bool is_vehicle_highway_value(const std::string& highway) {
    static const std::unordered_set<std::string> allowed = {
        "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
        "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
        "living_street", "service", "road", "busway", "construction"
    };
    return allowed.count(highway) != 0;
}

bool is_road_way(const Tags& tags) {
    const auto highway = tag_value(tags, "highway");
    if (!highway || !is_vehicle_highway_value(*highway)) return false;
    if (has_tag_value(tags, "area", "yes")) return false;
    if (has_tag_value(tags, "access", "private")) {
        // Keep private roads out by default. A production converter should make this configurable.
        return false;
    }
    return true;
}

bool is_relevant_point_feature(const Tags& tags, std::string* kind) {
    if (has_tag_value(tags, "highway", "traffic_signals")) {
        *kind = "traffic_light";
        return true;
    }
    if (has_tag_value(tags, "highway", "stop")) {
        *kind = "stop";
        return true;
    }
    if (has_tag_value(tags, "highway", "give_way")) {
        *kind = "give_way";
        return true;
    }
    if (tag_value(tags, "traffic_sign")) {
        *kind = "traffic_sign";
        return true;
    }
    if (has_tag_value(tags, "crossing", "traffic_signals")) {
        *kind = "traffic_light";
        return true;
    }
    return false;
}

ParseResult parse_osm(const Options& options) {
    using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
    using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

    osmium::io::Reader reader{options.input, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
    index_type index;
    location_handler_type location_handler{index};
    location_handler.ignore_errors();

    CollectHandler handler{options};
    osmium::apply(reader, location_handler, handler);
    reader.close();

    ParseResult result;
    result.roads = std::move(handler.roads);
    result.point_features = std::move(handler.point_features);
    result.projector = handler.projector;
    result.warnings = std::move(handler.warnings);
    return result;
}

} // namespace osm2xodr::osm
