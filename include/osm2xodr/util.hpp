#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace osm2xodr::util {

[[noreturn]] inline void fail(const std::string& msg) {
    throw std::runtime_error(msg);
}

inline std::string xml_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (const char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default: out += c; break;
        }
    }
    return out;
}

inline std::string attr(const std::string& key, const std::string& value) {
    return " " + key + "=\"" + xml_escape(value) + "\"";
}

inline std::string attr(const std::string& key, const double value) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(8) << value;
    return " " + key + "=\"" + ss.str() + "\"";
}

inline std::string attr(const std::string& key, const int value) {
    return " " + key + "=\"" + std::to_string(value) + "\"";
}

inline std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

inline std::vector<std::string> split_any(const std::string& s, const std::string& delimiters) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (start <= s.size()) {
        const auto pos = s.find_first_of(delimiters, start);
        auto token = s.substr(start, pos == std::string::npos ? std::string::npos : pos - start);
        const auto first = token.find_first_not_of(" \t\n\r");
        const auto last = token.find_last_not_of(" \t\n\r");
        if (first != std::string::npos) {
            parts.emplace_back(token.substr(first, last - first + 1));
        }
        if (pos == std::string::npos) break;
        start = pos + 1;
    }
    return parts;
}

inline std::optional<double> parse_double_prefix(std::string s) {
    s = lower(s);
    for (char& c : s) {
        if (c == ',') c = '.';
    }
    char* end = nullptr;
    const double value = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || !std::isfinite(value)) return std::nullopt;
    return value;
}

inline std::optional<int> parse_int(const std::string& s) {
    char* end = nullptr;
    const long value = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str()) return std::nullopt;
    return static_cast<int>(value);
}

inline bool truthy_osm(const std::string& v) {
    const auto l = lower(v);
    return l == "yes" || l == "true" || l == "1" || l == "designated" || l == "both";
}

inline bool falsy_osm(const std::string& v) {
    const auto l = lower(v);
    return l == "no" || l == "false" || l == "0" || l == "none" || l == "separate";
}

} // namespace osm2xodr::util
