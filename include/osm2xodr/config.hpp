#pragma once

#include "osm2xodr/options.hpp"

#include <string>

namespace osm2xodr::config {

// Loads a small YAML config file and applies it to `options`. Only a flat list-of-strings shape is
// supported (one key so far: `ignore_highways`, either flow style `[a, b]` or block style with `-`
// items on following lines) -- this is a minimal hand-rolled parser for that one shape, not a general
// YAML implementation. Fails (via util::fail) on an unreadable file or an unrecognized top-level key.
void load_config_file(const std::string& path, Options& options);

} // namespace osm2xodr::config
