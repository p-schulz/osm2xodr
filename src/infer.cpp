#include "osm2xodr/infer.hpp"

#include <algorithm>
#include <cmath>
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

void detect_sidewalks(const Tags& tags, const Options& options, bool* left, bool* right,
                       double* left_width, double* right_width) {
    *left = false;
    *right = false;
    *left_width = options.sidewalk_width;
    *right_width = options.sidewalk_width;

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

    // sidewalk:width is a both-sides fallback; sidewalk:left:width/sidewalk:right:width override it
    // per side when present -- the same fallback-then-per-side-override chain used for driving-lane
    // widths (width -> width:lanes -> width:lanes:forward|backward) above.
    if (const auto both_w = double_tag(tags, "sidewalk:width")) {
        *left_width = *both_w;
        *right_width = *both_w;
    }
    if (const auto lw = double_tag(tags, "sidewalk:left:width")) *left_width = *lw;
    if (const auto rw = double_tag(tags, "sidewalk:right:width")) *right_width = *rw;
}

std::string center_marking_type(const Tags& tags, const bool oneway) {
    if (tag_value_or(tags, "lane_markings", "yes") == "no") return "none";
    if (oneway) return "broken";
    if (has_tag_value(tags, "overtaking", "no") || has_tag_value(tags, "divider", "solid_line")) return "solid";
    return "broken";
}

model::LanePlan infer_lanes(const Tags& tags, const Options& options, std::vector<std::string>& warnings, const std::string& road_id) {
    model::LanePlan plan;

    // railway=rail/tram tracks aren't road traffic: a single track lane, no driving-lane inference
    // (turn:lanes, sidewalks, lane counts, markings) applies to them.
    if (const auto railway = tag_value(tags, "railway"); railway && (*railway == "rail" || *railway == "tram")) {
        plan.oneway = true;
        plan.forward_lanes = 1;
        plan.backward_lanes = 0;
        plan.center_mark = "none";

        model::LaneSpec lane;
        lane.id = -1;
        lane.type = *railway;
        lane.width = options.default_lane_width;
        lane.roadmark_type = "none";
        lane.roadmark_weight = "standard";
        lane.roadmark_color = "standard";
        lane.lane_change = "none";
        plan.right.push_back(std::move(lane));

        plan.lane_offset = model::compute_lane_offset(plan);
        return plan;
    }

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
    double left_sw_width = options.sidewalk_width;
    double right_sw_width = options.sidewalk_width;
    detect_sidewalks(tags, options, &left_sw, &right_sw, &left_sw_width, &right_sw_width);
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

    // median=yes is a real OSM boolean tag on the way itself; a median is inherently symmetric, so
    // it always gets half of options.median_width on each physical side (innermost, index 0), with
    // its own adjacent curb LaneSpec (reusing options.curb_width/curb_height -- the same fields the
    // sidewalk-curb block below already uses) immediately outboard of it, between the median and
    // the first real driving lane. left_id_base/right_id_base shift every driving lane's own id by
    // however many of these two slots actually got inserted; nothing downstream needs to change,
    // since the sidewalk-curb block already computes its own id from plan.left.size()/
    // plan.right.size() at push time rather than a fixed formula, and compute_lane_offset sums
    // whatever is actually present rather than assuming a particular lane count.
    int left_id_base = 0;
    int right_id_base = 0;
    if (options.infer_medians && has_tag_value(tags, "median", "yes")) {
        const double half_width = options.median_width / 2.0;
        auto make_median_half = [&](const int id) {
            model::LaneSpec median;
            median.id = id;
            median.type = "median";
            median.width = half_width;
            median.roadmark_type = "none";
            median.lane_change = "none";
            return median;
        };
        plan.left.push_back(make_median_half(1));
        plan.right.push_back(make_median_half(-1));
        left_id_base = right_id_base = 1;

        if (options.infer_curbs) {
            auto make_median_curb = [&](const int id) {
                model::LaneSpec curb;
                curb.id = id;
                curb.type = "curb";
                curb.width = options.curb_width;
                curb.height = options.curb_height;
                curb.roadmark_type = "none";
                curb.lane_change = "none";
                return curb;
            };
            plan.left.push_back(make_median_curb(2));
            plan.right.push_back(make_median_curb(-2));
            left_id_base = right_id_base = 2;
        }
    }

    if (!options.left_hand_traffic) {
        for (int i = 1; i <= plan.backward_lanes; ++i) {
            auto lane = make_drive_lane(left_id_base + i, width_for_backward(i), i == plan.backward_lanes && !plan.left_sidewalk);
            lane.turn_directions = turn_set_for(decoded_back, i);
            plan.left.push_back(std::move(lane));
        }
        for (int i = 1; i <= plan.forward_lanes; ++i) {
            auto lane = make_drive_lane(-(right_id_base + i), width_for_forward(i), i == plan.forward_lanes && !plan.right_sidewalk);
            lane.turn_directions = turn_set_for(decoded_fwd, i);
            plan.right.push_back(std::move(lane));
        }
    } else {
        // Left-hand traffic reverses the convention of which physical side carries reference-direction traffic.
        for (int i = 1; i <= plan.forward_lanes; ++i) {
            auto lane = make_drive_lane(left_id_base + i, width_for_forward(i), i == plan.forward_lanes && !plan.left_sidewalk);
            lane.turn_directions = turn_set_for(decoded_fwd, i);
            plan.left.push_back(std::move(lane));
        }
        for (int i = 1; i <= plan.backward_lanes; ++i) {
            auto lane = make_drive_lane(-(right_id_base + i), width_for_backward(i), i == plan.backward_lanes && !plan.right_sidewalk);
            lane.turn_directions = turn_set_for(decoded_back, i);
            plan.right.push_back(std::move(lane));
        }
    }

    // Adjacent lanes that are both exclusively dedicated to the same turn (e.g. a dual left-turn
    // pocket) get dashed markings all around the pair -- between them, and on the boundary to
    // either side -- instead of the default outer-solid/inner-broken rule, since drivers routinely
    // shift between the two lanes while queuing for the same turn. A lane's own roadmark always
    // describes the boundary on its outward (away-from-center) side, so "left of the pair" is
    // owned by the lane just inside it (or plan.center_mark, if the pair starts at the centerline).
    auto same_exclusive_turn = [](const std::vector<std::string>& a, const std::vector<std::string>& b) {
        return a.size() == 1 && b.size() == 1 && a[0] == b[0] && a[0] != "through";
    };
    auto dash_double_turn_lanes = [&](std::vector<model::LaneSpec>& side) {
        for (std::size_t i = 0; markings && i + 1 < side.size(); ++i) {
            if (!same_exclusive_turn(side[i].turn_directions, side[i + 1].turn_directions)) continue;
            side[i].roadmark_type = "broken";
            side[i].lane_change = "both";
            side[i + 1].roadmark_type = "broken";
            side[i + 1].lane_change = "both";
            if (i == 0) {
                plan.center_mark = "broken";
            } else {
                side[i - 1].roadmark_type = "broken";
                side[i - 1].lane_change = "both";
            }
        }
    };
    dash_double_turn_lanes(plan.left);
    dash_double_turn_lanes(plan.right);

    if (plan.left_sidewalk) {
        if (options.infer_curbs) {
            model::LaneSpec curb;
            curb.id = static_cast<int>(plan.left.size()) + 1;
            curb.type = "curb";
            curb.width = options.curb_width;
            curb.height = options.curb_height;
            curb.roadmark_type = "none";
            curb.lane_change = "none";
            plan.left.push_back(curb);
        }
        model::LaneSpec lane;
        lane.id = static_cast<int>(plan.left.size()) + 1;
        lane.type = "sidewalk";
        lane.width = left_sw_width;
        lane.roadmark_type = "solid";
        lane.lane_change = "none";
        plan.left.push_back(lane);
    }
    if (plan.right_sidewalk) {
        if (options.infer_curbs) {
            model::LaneSpec curb;
            curb.id = -static_cast<int>(plan.right.size()) - 1;
            curb.type = "curb";
            curb.width = options.curb_width;
            curb.height = options.curb_height;
            curb.roadmark_type = "none";
            curb.lane_change = "none";
            plan.right.push_back(curb);
        }
        model::LaneSpec lane;
        lane.id = -static_cast<int>(plan.right.size()) - 1;
        lane.type = "sidewalk";
        lane.width = right_sw_width;
        lane.roadmark_type = "solid";
        lane.lane_change = "none";
        plan.right.push_back(lane);
    }

    // Roadside grass verges are spatially detected (real landuse=grass polygon adjacency, not a
    // real OSM tag on the road) by ModelBuilder::apply_grass_verges(), which runs once per fragment
    // before merging and, on a match, sets an internal synthetic tag here
    // ("osm2xodr:grass_verge_left"/"osm2xodr:grass_verge_right" = "yes") and re-invokes this
    // function -- keeping infer_lanes itself pure/way-local (no spatial data ever crosses its
    // signature) while still surviving a fuse_chain reversal correctly: swap_directional_tags
    // already flips this tag pair the same way it flips sidewalk:left/right, so a reversed
    // merge-chain constituent's freshly-recomputed LanePlan still gets the border lane on the
    // correct physical side. Runs after the sidewalk/curb block above and computes its own id from
    // the side vector's current size, so it always lands outboard of whatever's already there
    // (driving lanes, and curb+sidewalk if present) without special-casing sidewalk presence.
    if (options.infer_grass_verges) {
        auto add_grass_verge = [&](std::vector<model::LaneSpec>& side, const bool is_left) {
            // A side can only ever have ONE contiguous run of a given lane type for fuse_chain's/
            // build_bridge_side's own type-keyed alignment to work correctly (they group every
            // lane of a type across the whole side, not by contiguous run) -- so skip this verge's
            // own curb when the side already carries one (typically the roadway-to-sidewalk curb).
            // A second, physically separate curb further outboard isn't realistic streetscape
            // design anyway (the sidewalk-to-verge transition is normally flush, not raised).
            const bool already_has_curb = std::any_of(side.begin(), side.end(),
                [](const model::LaneSpec& l) { return l.type == "curb"; });
            if (options.infer_curbs && !already_has_curb) {
                model::LaneSpec curb;
                curb.id = is_left ? static_cast<int>(side.size()) + 1 : -static_cast<int>(side.size()) - 1;
                curb.type = "curb";
                curb.width = options.curb_width;
                curb.height = options.curb_height;
                curb.roadmark_type = "none";
                curb.lane_change = "none";
                side.push_back(curb);
            }
            model::LaneSpec border;
            border.id = is_left ? static_cast<int>(side.size()) + 1 : -static_cast<int>(side.size()) - 1;
            border.type = "border";
            border.width = options.border_width;
            border.roadmark_type = "none";
            border.lane_change = "none";
            side.push_back(border);
        };
        if (has_tag_value(tags, "osm2xodr:grass_verge_left", "yes")) add_grass_verge(plan.left, true);
        if (has_tag_value(tags, "osm2xodr:grass_verge_right", "yes")) add_grass_verge(plan.right, false);
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

double elevation_offset(const Tags& tags) {
    constexpr double kLayerHeightM = 3.5;
    const auto layer = tag_value(tags, "layer");
    if (!layer) return 0.0;
    const auto n = util::parse_int(*layer);
    if (!n) return 0.0;
    return static_cast<double>(*n) * kLayerHeightM;
}

std::optional<double> parse_maxspeed(const std::string& v) {
    const auto l = util::lower(v);
    if (l == "none" || l == "signals" || l == "walk") return std::nullopt;
    const auto value = util::parse_double_prefix(l);
    if (!value) return std::nullopt;
    // OSM allows an explicit unit suffix (e.g. "30 mph"); a bare number is always km/h. Always
    // return km/h so callers (the maxspeed signal, and the adaptive turn-radius formula below)
    // never have to special-case units themselves.
    if (l.find("mph") != std::string::npos) return *value * 1.609344;
    return value;
}

// Calibration-anchor "typical" running speed (km/h) per highway class, mirroring
// turn_radius_for_highway's own class groupings exactly -- used only to calibrate
// adaptive_turn_radius against that function's tiers, not asserted as a real speed limit.
// Unmapped classes return nullopt, same spirit as turn_radius_for_highway's own fallback: there's
// no sensible class-typical speed to calibrate against either.
std::optional<double> typical_speed_for_highway(const std::string& highway) {
    if (highway == "motorway" || highway == "motorway_link" || highway == "trunk" || highway == "trunk_link") return 100.0;
    if (highway == "primary" || highway == "primary_link") return 50.0;
    if (highway == "secondary" || highway == "secondary_link") return 50.0;
    if (highway == "tertiary" || highway == "tertiary_link" || highway == "unclassified") return 50.0;
    if (highway == "residential" || highway == "living_street") return 30.0;
    if (highway == "service" || highway == "road" || highway == "busway" || highway == "construction") return 20.0;
    return std::nullopt;
}

double adaptive_turn_radius(const std::string& in_highway, const std::optional<double> in_maxspeed_kmh,
                             const std::string& out_highway, const std::optional<double> out_maxspeed_kmh,
                             const double abs_delta, const Options& options) {
    const double tier_in = turn_radius_for_highway(in_highway, options);
    const double tier_out = turn_radius_for_highway(out_highway, options);
    const double tier = std::max(tier_in, tier_out);
    if (!options.adaptive_turn_radius) return tier;

    const bool governing_is_in = tier_in >= tier_out;
    const auto v_typical = typical_speed_for_highway(governing_is_in ? in_highway : out_highway);
    if (!v_typical) return tier; // unrecognized class -- no anchor to calibrate against

    const auto governing_speed = governing_is_in ? in_maxspeed_kmh : out_maxspeed_kmh;
    const auto other_speed = governing_is_in ? out_maxspeed_kmh : in_maxspeed_kmh;
    const auto v_actual = governing_speed ? governing_speed : other_speed;
    if (!v_actual) return tier; // neither road tagged -- fall back to today's tier exactly

    const double speed_factor = (*v_actual / *v_typical) * (*v_actual / *v_typical);
    // Smooth, monotonic in abs_delta, exactly 1.0 at the 90 deg anchor (matching the tier's own
    // calibration point above): a near-straight movement reads a bigger effective radius, a
    // near-reversal shrinks toward 0 (the caller's own existing floor clamps the final result).
    const double angle_factor_base = std::cos(abs_delta / 2.0) / std::cos(geo::kPi / 4.0);
    const double angle_factor = angle_factor_base * angle_factor_base;
    return tier * speed_factor * angle_factor;
}

double adaptive_taper_length(const double lane_width, const std::optional<double> maxspeed_kmh,
                              const double fallback_length, const Options& options) {
    if (!options.adaptive_lane_taper || !maxspeed_kmh) return fallback_length;
    // Approximation of the general German lane-taper/Verziehungslaenge convention (length scales
    // with design speed and the width being shifted) -- see the declaration comment in infer.hpp
    // for the "not a verified RSA 21 reproduction" caveat.
    return *maxspeed_kmh * lane_width / 3.0;
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

model::RoadSignal stop_line_signal(const std::string& id, const double s, const double t) {
    model::RoadSignal sig;
    sig.id = id;
    sig.s = std::max(0.0, s);
    sig.t = t;
    sig.name = "stop_line";
    sig.orientation = t >= 0.0 ? "+" : "-";
    sig.dynamic = false;
    sig.type = "stop_line";
    return sig;
}

bool is_pedestrian_traffic_light(const Tags& tags) {
    return has_tag_value(tags, "traffic_signals", "pedestrian_crossing") || has_tag_value(tags, "crossing", "traffic_signals");
}

} // namespace osm2xodr::infer
