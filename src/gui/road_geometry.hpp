#pragma once

#include "osm2xodr/model.hpp"

#include <vector>

namespace osm2xodr::gui {

struct WorldPoint {
    double x = 0.0;
    double y = 0.0;
};

struct RoadTraces {
    std::vector<WorldPoint> centerline;
    std::vector<WorldPoint> left_edge;  // outer edge of the left lane stack (lane_offset side)
    std::vector<WorldPoint> right_edge; // outer edge of the right lane stack
};

// Samples one road's centerline and outer left/right lane-cross-section edges as world-space
// (osm2xodr's local equirectangular meters -- the same frame model::RoadSegment geometry is
// already in, no reprojection needed) polylines, `step` meters apart. Deliberately mirrors
// xodr_writer.cpp exactly (same explicit_geometry-vs-points fallback, same per-laneSection
// width/lane-offset taper math) so the preview matches what was actually written to the .xodr,
// not a reinterpretation of it.
RoadTraces sample_road(const model::RoadSegment& road, double step = 2.0);

// Traces every boundary *between* individual lanes (not just each side's outer edge -- that's
// left_edge/right_edge above): one polyline per contiguous run of a given lane's outer boundary.
// Lane counts can change mid-road (splits/merges insert a new laneSection), so a boundary is only
// continuous while that lane ordinal keeps existing -- gaps end one run and start a new one.
// Mirrors test/xodr_viewer.py's lane_marking_traces(), meant to be drawn dashed.
std::vector<std::vector<WorldPoint>> sample_lane_markings(const model::RoadSegment& road, double step = 2.0);

struct PreviewGeometry {
    std::vector<RoadTraces> roads;
    std::vector<std::vector<WorldPoint>> lane_markings;
    double min_x = -50.0, min_y = -50.0, max_x = 50.0, max_y = 50.0; // bounding box, local meters
};

PreviewGeometry build_preview_geometry(const model::MapModel& model, double step = 2.0);

} // namespace osm2xodr::gui
