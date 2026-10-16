#pragma once

#include "road_geometry.hpp"

#include <string>

namespace osm2xodr::gui {

struct XodrPreviewResult {
    PreviewGeometry geometry;
    int road_count = 0;
    int junction_count = 0;
    // Recovered from the <geoReference> proj4 string xodr_writer.cpp writes (+lat_0/+lon_0), when
    // present -- lets the aerial-imagery fetch keep working for a file opened this way, same as a
    // freshly converted session.
    bool has_origin = false;
    double origin_lat = 0.0;
    double origin_lon = 0.0;
};

// Parses an existing .xodr file directly with libOpenDRIVE, bypassing the OSM pipeline and
// model::MapModel entirely -- for previewing an already-converted file (File > Open XODR...)
// without needing its original OSM input. Throws std::runtime_error (message suitable for
// Session::status) on any parse failure.
XodrPreviewResult build_preview_geometry_from_xodr(const std::string& path, double step = 2.0);

} // namespace osm2xodr::gui
