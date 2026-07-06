#pragma once

#include "osm2xodr/options.hpp"

#include <string>

namespace osm2xodr {

Options parse_args(int argc, char** argv);

void validate_with_libopendrive(const std::string& path);

} // namespace osm2xodr
