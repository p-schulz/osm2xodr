#pragma once

#include "osm2xodr/model.hpp"
#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"

namespace osm2xodr {

void write_report(const model::MapModel& model, const Options& options, const osm::ParseResult& parsed);

} // namespace osm2xodr
