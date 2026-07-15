#include "osm2xodr/config.hpp"

#include "osm2xodr/util.hpp"

#include <fstream>
#include <sstream>

namespace osm2xodr::config {

namespace {

std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::string strip_quotes(std::string s) {
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
        s = s.substr(1, s.size() - 2);
    }
    return s;
}

void add_ignore_highway(Options& options, const std::string& raw_item) {
    const std::string item = util::lower(strip_quotes(trim(raw_item)));
    if (!item.empty()) options.ignore_highways.insert(item);
}

} // namespace

void load_config_file(const std::string& path, Options& options) {
    std::ifstream in(path);
    if (!in) util::fail("Could not open config file: " + path);

    std::string current_key;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        const std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        if (trimmed.front() == '-') {
            // Block-style list item under whichever key we last saw.
            if (current_key == "ignore_highways") {
                add_ignore_highway(options, trimmed.substr(1));
            } else {
                util::fail("Config file " + path + ":" + std::to_string(line_no) +
                           ": list item outside a recognized key ('" + current_key + "')");
            }
            continue;
        }

        const auto colon = trimmed.find(':');
        if (colon == std::string::npos) {
            util::fail("Config file " + path + ":" + std::to_string(line_no) + ": expected 'key: value', got: " + trimmed);
        }
        const std::string key = trim(trimmed.substr(0, colon));
        const std::string value = trim(trimmed.substr(colon + 1));
        current_key = key;

        if (key == "ignore_highways") {
            if (value.empty()) continue; // block-style list follows on subsequent lines
            std::string flow = value;
            if (flow.front() == '[' && flow.back() == ']') flow = flow.substr(1, flow.size() - 2);
            for (const auto& tok : util::split_any(flow, ",")) add_ignore_highway(options, tok);
        } else {
            util::fail("Config file " + path + ":" + std::to_string(line_no) +
                       ": unknown key '" + key + "' (only 'ignore_highways' is supported)");
        }
    }
}

} // namespace osm2xodr::config
