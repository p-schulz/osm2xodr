#pragma once

#include "osm2xodr/model.hpp"
#include "osm2xodr/options.hpp"

namespace osm2xodr::xodr {

void write_file(const model::MapModel& model, const Options& options);

} // namespace osm2xodr::xodr
