#pragma once

#include "osm2xodr/geo.hpp"
#include "osm2xodr/options.hpp"
#include "osm2xodr/tags.hpp"

#include <osmium/handler.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace osm2xodr::osm {

struct WayNode {
    std::int64_t ref = 0;
    geo::LonLat ll{};
};

struct RawWay {
    std::int64_t id = 0;
    Tags tags;
    std::vector<WayNode> nodes;
    bool reversed_for_oneway_minus_one = false;
};

struct PointFeature {
    std::int64_t node_ref = 0;
    geo::LonLat ll{};
    Tags tags;
    std::string kind;
};

// The fixed set of highway=* values admitted as ordinary vehicle roads (see is_vehicle_highway_value
// below, which is just a fast-lookup wrapper around this same list) -- exposed so a UI can offer a
// per-value "ignore this class entirely" toggle without duplicating the list independently.
const std::vector<std::string>& vehicle_highway_values();
bool is_vehicle_highway_value(const std::string& highway);
bool is_road_way(const Tags& tags, const std::unordered_set<std::string>& ignore_highways);
bool is_railway_track_way(const Tags& tags);
bool is_grass_area_way(const Tags& tags);
bool is_relevant_point_feature(const Tags& tags, std::string* kind);

class CollectHandler final : public osmium::handler::Handler {
public:
    std::vector<RawWay> roads;
    std::vector<RawWay> grass_areas; // closed ways tagged landuse=grass; reuses RawWay as-is
    std::vector<PointFeature> point_features;
    std::vector<std::string> warnings;
    geo::LocalProjector projector;

    explicit CollectHandler(const Options& options) : ignore_highways_(options.ignore_highways) {
        if (options.origin_lat && options.origin_lon) {
            projector.set_origin(*options.origin_lat, *options.origin_lon);
        } else if (options.origin_lat || options.origin_lon) {
            util::fail("Both --origin-lat and --origin-lon must be supplied together");
        }
    }

    void node(const osmium::Node& node) {
        if (!node.location().valid()) return;
        projector.ensure_origin(node.location().lat(), node.location().lon());
        Tags tags = read_tags(node.tags());
        std::string kind;
        if (is_relevant_point_feature(tags, &kind)) {
            point_features.push_back({
                static_cast<std::int64_t>(node.id()),
                {node.location().lon(), node.location().lat()},
                std::move(tags),
                kind
            });
        }
    }

    void way(const osmium::Way& way) {
        Tags tags = read_tags(way.tags());
        if (is_grass_area_way(tags)) {
            RawWay raw;
            raw.id = static_cast<std::int64_t>(way.id());
            raw.tags = std::move(tags);
            for (const auto& nr : way.nodes()) {
                if (!nr.location().valid()) {
                    warnings.push_back("Grass area way " + std::to_string(raw.id) + " references node " +
                        std::to_string(nr.ref()) + " without a resolved location; skipped that node.");
                    continue;
                }
                projector.ensure_origin(nr.lat(), nr.lon());
                raw.nodes.push_back({static_cast<std::int64_t>(nr.ref()), {nr.lon(), nr.lat()}});
            }
            if (raw.nodes.size() < 3) {
                warnings.push_back("Grass area way " + std::to_string(raw.id) + " has fewer than three located nodes and was ignored.");
                return;
            }
            grass_areas.push_back(std::move(raw));
            return;
        }
        if (!is_road_way(tags, ignore_highways_) && !is_railway_track_way(tags)) return;

        RawWay raw;
        raw.id = static_cast<std::int64_t>(way.id());
        raw.tags = std::move(tags);
        for (const auto& nr : way.nodes()) {
            if (!nr.location().valid()) {
                warnings.push_back("Way " + std::to_string(raw.id) + " references node " + std::to_string(nr.ref()) + " without a resolved location; skipped that node.");
                continue;
            }
            projector.ensure_origin(nr.lat(), nr.lon());
            raw.nodes.push_back({static_cast<std::int64_t>(nr.ref()), {nr.lon(), nr.lat()}});
        }
        if (raw.nodes.size() < 2) {
            warnings.push_back("Way " + std::to_string(raw.id) + " has fewer than two located nodes and was ignored.");
            return;
        }
        const auto oneway = tag_value(raw.tags, "oneway");
        if (oneway && *oneway == "-1") {
            std::reverse(raw.nodes.begin(), raw.nodes.end());
            raw.reversed_for_oneway_minus_one = true;
            swap_directional_tags(raw.tags);
        }
        roads.push_back(std::move(raw));
    }

private:
    static Tags read_tags(const osmium::TagList& list) {
        Tags tags;
        for (const auto& t : list) tags.emplace(t.key(), t.value());
        return tags;
    }

    std::unordered_set<std::string> ignore_highways_;
};

struct ParseResult {
    std::vector<RawWay> roads;
    std::vector<RawWay> grass_areas;
    std::vector<PointFeature> point_features;
    geo::LocalProjector projector;
    std::vector<std::string> warnings;
};

ParseResult parse_osm(const Options& options);

} // namespace osm2xodr::osm
