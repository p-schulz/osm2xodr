#include "osm2xodr/infer.hpp"

#include <algorithm>
#include <cstddef>

namespace osm2xodr::infer {

bool is_oneway(const Tags& tags) {
    const auto highway = tag_value_or(tags, "highway", "");
    const auto oneway = tag_value(tags, "oneway");
    if (oneway) {
        const auto l = util::lower(*oneway);
        if (l == "-1" || l == "yes" || l == "true" || l == "1") return true;
        if (l == "no" || l == "false" || l == "0") return false;
    }
    if (has_tag_value(tags, "junction", "roundabout") || has_tag_value(tags, "junction", "circular")) return true;
    return highway == "motorway" || highway == "motorway_link";
}

double default_width_for_highway(const std::string& highway, const Options& options) {
    if (highway == "motorway" || highway == "trunk") return 3.75;
    if (highway == "primary" || highway == "secondary" || highway == "tertiary") return 3.50;
    if (highway == "residential" || highway == "unclassified") return 3.20;
    if (highway == "service" || highway == "living_street") return 3.00;
    return options.default_lane_width;
}

std::optional<int> int_tag(const Tags& tags, const std::string& key) {
    const auto value = tag_value(tags, key);
    if (!value) return std::nullopt;
    return util::parse_int(*value);
}

std::optional<double> double_tag(const Tags& tags, const std::string& key) {
    const auto value = tag_value(tags, key);
    if (!value) return std::nullopt;
    return util::parse_double_prefix(*value);
}

std::vector<double> parse_width_list(const std::string& raw) {
    std::vector<double> values;
    for (const auto& p : util::split_any(raw, "|;")) {
        if (const auto v = util::parse_double_prefix(p); v && *v > 0.2) values.push_back(*v);
    }
    return values;
}

// Unlike util::split_any, this never drops empty tokens: OSM's *:lanes tags (turn:lanes,
// access:lanes, vehicle:lanes, width:lanes, ...) are positional -- slot i in one tag describes the
// same physical lane as slot i in another -- so an empty slot (e.g. "left||through") must still
// occupy a position, not be silently skipped and shift every later slot's alignment.
std::vector<std::string> split_pipe_preserve_empty(const std::string& s) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const auto pos = s.find('|', start);
        parts.push_back(s.substr(start, pos == std::string::npos ? std::string::npos : pos - start));
        if (pos == std::string::npos) break;
        start = pos + 1;
    }
    return parts;
}

// Splits a single turn:lanes slot's value (e.g. "through;right") into normalized tokens. An empty
// string or literal "none" is OSM's own way of saying "no indicated restriction" for that lane.
std::vector<std::string> parse_turn_tokens(const std::string& raw) {
    std::vector<std::string> tokens;
    for (const auto& t : util::split_any(raw, ";")) {
        const auto l = util::lower(t);
        if (l.empty() || l == "none") continue;
        tokens.push_back(l);
    }
    return tokens;
}

// Decodes real car-lane count and per-lane turn permission from turn:lanes<suffix>, cross-checked
// against access:lanes<suffix>/vehicle:lanes<suffix> to exclude non-car lane slots (bike/bus/etc,
// generically: anything explicitly marked "no" for general/vehicle access) that are commonly
// interleaved with real lanes in the same pipe-separated list. `suffix` is "" for a oneway road's
// plain turn:lanes, or ":forward"/":backward" for a two-way road. Returns `available=false` (and
// the caller keeps today's lanes=N-based inference untouched) when turn:lanes<suffix> is absent.
struct DecodedTurnLanes {
    bool available = false;
    std::vector<std::size_t> car_slot_positions; // indices into the raw pipe-split lists that are real car lanes, left-to-right
    std::vector<std::vector<std::string>> turn_sets; // per selected car lane, its allowed-direction tokens (empty = unrestricted)
};

DecodedTurnLanes decode_turn_lanes(const Tags& tags, const std::string& suffix) {
    DecodedTurnLanes result;
    const auto turn_raw = tag_value(tags, "turn:lanes" + suffix);
    if (!turn_raw) return result;

    const auto turn_slots = split_pipe_preserve_empty(*turn_raw);
    const auto access_raw = tag_value(tags, "access:lanes" + suffix);
    const auto vehicle_raw = tag_value(tags, "vehicle:lanes" + suffix);
    const auto access_slots = access_raw ? split_pipe_preserve_empty(*access_raw) : std::vector<std::string>{};
    const auto vehicle_slots = vehicle_raw ? split_pipe_preserve_empty(*vehicle_raw) : std::vector<std::string>{};

    result.available = true;
    for (std::size_t i = 0; i < turn_slots.size(); ++i) {
        const bool excluded = (i < access_slots.size() && util::lower(access_slots[i]) == "no") ||
                              (i < vehicle_slots.size() && util::lower(vehicle_slots[i]) == "no");
        if (excluded) continue;
        result.car_slot_positions.push_back(i);
        result.turn_sets.push_back(parse_turn_tokens(turn_slots[i]));
    }
    return result;
}

void detect_sidewalks(const Tags& tags, bool* left, bool* right) {
    *left = false;
    *right = false;

    const auto sw = tag_value(tags, "sidewalk");
    if (sw) {
        const auto l = util::lower(*sw);
        if (l == "both" || l == "yes") {
            *left = true;
            *right = true;
        } else if (l == "left") {
            *left = true;
        } else if (l == "right") {
            *right = true;
        }
    }
    const auto sw_left = tag_value(tags, "sidewalk:left");
    if (sw_left) *left = !util::falsy_osm(*sw_left);
    const auto sw_right = tag_value(tags, "sidewalk:right");
    if (sw_right) *right = !util::falsy_osm(*sw_right);
}

std::string center_marking_type(const Tags& tags, const bool oneway) {
    if (tag_value_or(tags, "lane_markings", "yes") == "no") return "none";
    if (oneway) return "broken";
    if (has_tag_value(tags, "overtaking", "no") || has_tag_value(tags, "divider", "solid_line")) return "solid";
    return "broken";
}

model::LanePlan infer_lanes(const Tags& tags, const Options& options, std::vector<std::string>& warnings, const std::string& road_id) {
    model::LanePlan plan;
    plan.oneway = is_oneway(tags);
    const std::string highway = tag_value_or(tags, "highway", "road");
    const double default_width = default_width_for_highway(highway, options);

    int total_lanes = int_tag(tags, "lanes").value_or(plan.oneway ? 1 : 2);
    if (total_lanes < 1) total_lanes = 1;

    auto f = int_tag(tags, "lanes:forward");
    auto b = int_tag(tags, "lanes:backward");

    if (plan.oneway) {
        plan.forward_lanes = f.value_or(total_lanes);
        plan.backward_lanes = 0;
    } else if (f || b) {
        plan.forward_lanes = f.value_or(std::max(1, total_lanes - b.value_or(1)));
        plan.backward_lanes = b.value_or(std::max(1, total_lanes - plan.forward_lanes));
    } else {
        if (total_lanes == 1) {
            plan.forward_lanes = 1;
            plan.backward_lanes = 1;
            warnings.push_back("Road " + road_id + " has lanes=1 on a two-way road; represented as one lane per direction because OpenDRIVE does not model undivided shared bidirectional single-lane roads well.");
        } else if (total_lanes % 2 == 0) {
            plan.forward_lanes = total_lanes / 2;
            plan.backward_lanes = total_lanes / 2;
        } else {
            plan.forward_lanes = total_lanes / 2;
            plan.backward_lanes = total_lanes / 2;
            warnings.push_back("Road " + road_id + " has odd two-way lanes=" + std::to_string(total_lanes) + "; middle/turn lane was not modeled explicitly.");
        }
    }

    plan.forward_lanes = std::max(0, plan.forward_lanes);
    plan.backward_lanes = std::max(0, plan.backward_lanes);
    if (plan.forward_lanes == 0 && plan.backward_lanes == 0) plan.forward_lanes = 1;

    // turn:lanes (oneway roads) / turn:lanes:forward+:backward (two-way roads) is a more precise,
    // per-lane source of truth than the plain lanes=N tag: it also reveals lanes commonly
    // interleaved in the same tag (bike/bus) via access:lanes/vehicle:lanes, and tells each real
    // car lane's permitted turn direction(s). When present, it overrides the count derived above.
    const DecodedTurnLanes decoded_fwd = decode_turn_lanes(tags, plan.oneway ? "" : ":forward");
    const DecodedTurnLanes decoded_back = plan.oneway ? DecodedTurnLanes{} : decode_turn_lanes(tags, ":backward");
    if (decoded_fwd.available && !decoded_fwd.car_slot_positions.empty()) {
        const int decoded_count = static_cast<int>(decoded_fwd.car_slot_positions.size());
        if (decoded_count != plan.forward_lanes) {
            warnings.push_back("Road " + road_id + ": turn:lanes indicates " + std::to_string(decoded_count) +
                " forward car lane(s) (after excluding non-car slots), overriding the lanes-derived count of " +
                std::to_string(plan.forward_lanes) + ".");
        }
        plan.forward_lanes = decoded_count;
    }
    if (decoded_back.available && !decoded_back.car_slot_positions.empty()) {
        const int decoded_count = static_cast<int>(decoded_back.car_slot_positions.size());
        if (decoded_count != plan.backward_lanes) {
            warnings.push_back("Road " + road_id + ": turn:lanes:backward indicates " + std::to_string(decoded_count) +
                " backward car lane(s) (after excluding non-car slots), overriding the lanes-derived count of " +
                std::to_string(plan.backward_lanes) + ".");
        }
        plan.backward_lanes = decoded_count;
    }

    bool left_sw = false;
    bool right_sw = false;
    detect_sidewalks(tags, &left_sw, &right_sw);
    plan.left_sidewalk = left_sw;
    plan.right_sidewalk = right_sw;
    plan.center_mark = center_marking_type(tags, plan.oneway);

    std::vector<double> widths_all;
    if (const auto raw = tag_value(tags, "width:lanes")) widths_all = parse_width_list(*raw);
    std::vector<double> widths_fwd;
    if (const auto raw = tag_value(tags, "width:lanes:forward")) widths_fwd = parse_width_list(*raw);
    std::vector<double> widths_back;
    if (const auto raw = tag_value(tags, "width:lanes:backward")) widths_back = parse_width_list(*raw);

    const double total_width = double_tag(tags, "width").value_or(0.0);
    double lane_width = default_width;
    if (total_width > 1.0) {
        const int modeled_drive_lanes = std::max(1, plan.forward_lanes + plan.backward_lanes);
        lane_width = std::max(2.0, total_width / modeled_drive_lanes);
    }

    // When turn:lanes decoding is active, a lane's width still comes from width:lanes, but at that
    // lane's own raw slot position (car_slot_positions), not sequential 1..N -- the retained car
    // lanes are not necessarily contiguous once interleaved bike/bus slots are excluded.
    auto width_for_forward = [&](const int idx_from_center) {
        std::size_t widths_idx = static_cast<std::size_t>(idx_from_center - 1);
        if (decoded_fwd.available && widths_idx < decoded_fwd.car_slot_positions.size()) {
            widths_idx = decoded_fwd.car_slot_positions[widths_idx];
        }
        if (!widths_fwd.empty() && widths_idx < widths_fwd.size()) return widths_fwd[widths_idx];
        if (!widths_all.empty() && widths_idx < widths_all.size()) return widths_all[widths_idx];
        return lane_width;
    };
    auto width_for_backward = [&](const int idx_from_center) {
        std::size_t widths_idx = static_cast<std::size_t>(idx_from_center - 1);
        if (decoded_back.available && widths_idx < decoded_back.car_slot_positions.size()) {
            widths_idx = decoded_back.car_slot_positions[widths_idx];
        }
        if (!widths_back.empty() && widths_idx < widths_back.size()) return widths_back[widths_idx];
        return lane_width;
    };

    const bool markings = tag_value_or(tags, "lane_markings", "yes") != "no";

    auto make_drive_lane = [&](const int id, const double width, const bool outer) {
        model::LaneSpec lane;
        lane.id = id;
        lane.type = "driving";
        lane.width = width;
        lane.roadmark_type = markings ? (outer ? "solid" : "broken") : "none";
        lane.roadmark_weight = "standard";
        lane.roadmark_color = "standard";
        lane.lane_change = lane.roadmark_type == "solid" ? "none" : "both";
        return lane;
    };

    // turn_sets[i-1] is this lane's allowed-direction set (empty = unrestricted) when decoding
    // succeeded for that direction; otherwise every lane stays unrestricted (today's behavior).
    auto turn_set_for = [](const DecodedTurnLanes& decoded, const int i) -> std::vector<std::string> {
        const auto idx = static_cast<std::size_t>(i - 1);
        if (decoded.available && idx < decoded.turn_sets.size()) return decoded.turn_sets[idx];
        return {};
    };

    if (!options.left_hand_traffic) {
        for (int i = 1; i <= plan.backward_lanes; ++i) {
            auto lane = make_drive_lane(i, width_for_backward(i), i == plan.backward_lanes && !plan.left_sidewalk);
            lane.turn_directions = turn_set_for(decoded_back, i);
            plan.left.push_back(std::move(lane));
        }
        for (int i = 1; i <= plan.forward_lanes; ++i) {
            auto lane = make_drive_lane(-i, width_for_forward(i), i == plan.forward_lanes && !plan.right_sidewalk);
            lane.turn_directions = turn_set_for(decoded_fwd, i);
            plan.right.push_back(std::move(lane));
        }
    } else {
        // Left-hand traffic reverses the convention of which physical side carries reference-direction traffic.
        for (int i = 1; i <= plan.forward_lanes; ++i) {
            auto lane = make_drive_lane(i, width_for_forward(i), i == plan.forward_lanes && !plan.left_sidewalk);
            lane.turn_directions = turn_set_for(decoded_fwd, i);
            plan.left.push_back(std::move(lane));
        }
        for (int i = 1; i <= plan.backward_lanes; ++i) {
            auto lane = make_drive_lane(-i, width_for_backward(i), i == plan.backward_lanes && !plan.right_sidewalk);
            lane.turn_directions = turn_set_for(decoded_back, i);
            plan.right.push_back(std::move(lane));
        }
    }

    if (plan.left_sidewalk) {
        model::LaneSpec lane;
        lane.id = static_cast<int>(plan.left.size()) + 1;
        lane.type = "sidewalk";
        lane.width = options.sidewalk_width;
        lane.roadmark_type = "solid";
        lane.lane_change = "none";
        plan.left.push_back(lane);
    }
    if (plan.right_sidewalk) {
        model::LaneSpec lane;
        lane.id = -static_cast<int>(plan.right.size()) - 1;
        lane.type = "sidewalk";
        lane.width = options.sidewalk_width;
        lane.roadmark_type = "solid";
        lane.lane_change = "none";
        plan.right.push_back(lane);
    }

    plan.lane_offset = model::compute_lane_offset(plan);

    return plan;
}

double turn_radius_for_highway(const std::string& highway, const Options& options) {
    if (highway == "motorway" || highway == "motorway_link" || highway == "trunk" || highway == "trunk_link") return 15.0;
    if (highway == "primary" || highway == "primary_link") return 12.0;
    if (highway == "secondary" || highway == "secondary_link") return 10.0;
    if (highway == "tertiary" || highway == "tertiary_link" || highway == "unclassified") return 8.0;
    if (highway == "residential" || highway == "living_street") return 6.0;
    if (highway == "service" || highway == "road" || highway == "busway" || highway == "construction") return 5.0;
    return options.junction_turn_radius;
}

std::string road_type(const Tags& tags) {
    const auto highway = tag_value_or(tags, "highway", "road");
    if (highway == "motorway" || highway == "motorway_link") return "motorway";
    if (highway == "trunk" || highway == "primary" || highway == "secondary" || highway == "tertiary") return "rural";
    return "town";
}

std::optional<double> parse_maxspeed(const std::string& v) {
    const auto l = util::lower(v);
    if (l == "none" || l == "signals" || l == "walk") return std::nullopt;
    return util::parse_double_prefix(l);
}

model::RoadSignal signal_from_point_feature(const osm::PointFeature& pf, const std::string& id, const double s, const double t) {
    model::RoadSignal sig;
    sig.id = id;
    sig.s = std::max(0.0, s);
    sig.t = t;
    sig.name = tag_value_or(pf.tags, "name", pf.kind);
    sig.orientation = t >= 0.0 ? "+" : "-";

    if (pf.kind == "traffic_light") {
        sig.dynamic = true;
        sig.type = "traffic_light";
        sig.subtype = tag_value_or(pf.tags, "traffic_signals", "");
        sig.text = tag_value_or(pf.tags, "traffic_signals:direction", "");
    } else if (pf.kind == "stop") {
        sig.dynamic = false;
        sig.type = "stop";
        sig.subtype = "";
    } else if (pf.kind == "give_way") {
        sig.dynamic = false;
        sig.type = "give_way";
        sig.subtype = "";
    } else {
        sig.dynamic = false;
        sig.type = tag_value_or(pf.tags, "traffic_sign", "traffic_sign");
        sig.subtype = tag_value_or(pf.tags, "traffic_sign:subtype", "");
        sig.text = tag_value_or(pf.tags, "traffic_sign", "");
    }
    return sig;
}

} // namespace osm2xodr::infer
