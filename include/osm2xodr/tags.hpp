#pragma once

#include "osm2xodr/util.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace osm2xodr {

using Tags = std::unordered_map<std::string, std::string>;

inline std::optional<std::string> tag_value(const Tags& tags, const std::string& key) {
    const auto it = tags.find(key);
    if (it == tags.end()) return std::nullopt;
    return it->second;
}

inline std::string tag_value_or(const Tags& tags, const std::string& key, const std::string& fallback) {
    const auto v = tag_value(tags, key);
    return v ? *v : fallback;
}

inline bool has_tag_value(const Tags& tags, const std::string& key, const std::string& value) {
    const auto v = tag_value(tags, key);
    return v && util::lower(*v) == util::lower(value);
}

// Swaps directional tag pairs so a mirrored/reversed way's tags still describe "forward"/"left"
// correctly relative to its new direction. Used both for OSM oneway=-1 ways and for reversing a
// constituent when merging a chain of roads whose OSM digitization directions disagree.
inline void swap_if_present(Tags& tags, const std::string& a, const std::string& b) {
    const auto ia = tags.find(a);
    const auto ib = tags.find(b);
    if (ia != tags.end() && ib != tags.end()) {
        std::swap(ia->second, ib->second);
    } else if (ia != tags.end()) {
        tags.emplace(b, ia->second);
        tags.erase(ia);
    } else if (ib != tags.end()) {
        tags.emplace(a, ib->second);
        tags.erase(ib);
    }
}

inline void swap_directional_tags(Tags& tags) {
    swap_if_present(tags, "lanes:forward", "lanes:backward");
    swap_if_present(tags, "width:lanes:forward", "width:lanes:backward");
    swap_if_present(tags, "turn:lanes:forward", "turn:lanes:backward");
    swap_if_present(tags, "change:lanes:forward", "change:lanes:backward");
    swap_if_present(tags, "sidewalk:left", "sidewalk:right");
}

} // namespace osm2xodr
