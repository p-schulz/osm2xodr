#pragma once

#include "osm2xodr/model.hpp"
#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"

namespace osm2xodr::build {

model::MapModel build_model(const osm::ParseResult& parsed, const Options& options);

} // namespace osm2xodr::build
