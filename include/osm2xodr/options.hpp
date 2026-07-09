#pragma once

#include <optional>
#include <string>

namespace osm2xodr {

struct Options {
    std::string input;
    std::string output;
    std::string name = "osm2xodr-map";
    std::optional<double> origin_lat;
    std::optional<double> origin_lon;
    double default_lane_width = 3.50;
    double sidewalk_width = 2.00;
    bool left_hand_traffic = false;
    int junction_degree = 3;
    double signal_search_radius = 20.0;
    double junction_turn_radius = 8.0;
    bool merge_roads = true;
    bool merge_junctions = true;
    double junction_cluster_max_gap = 20.0;
    bool absorb_signal_setbacks = true;
    double junction_signal_setback_max_gap = 15.0;
    double lane_taper_length = 15.0;
    bool bridge_lane_count_changes = true;
    bool curve_fit = true;
    std::string report_path;
    bool validate = false;
};

} // namespace osm2xodr
