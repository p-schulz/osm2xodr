#include "osm2xodr/xodr_writer.hpp"

#include "osm2xodr/infer.hpp"
#include "osm2xodr/tags.hpp"
#include "osm2xodr/util.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>

namespace osm2xodr::xodr {

void write_indent(std::ostream& os, const int indent) {
    for (int i = 0; i < indent; ++i) os << ' ';
}

void write_lane(std::ostream& os, const model::LaneSpec& lane, const int indent, const double road_length) {
    write_indent(os, indent);
    os << "<lane" << util::attr("id", lane.id) << util::attr("type", lane.type) << util::attr("level", "false") << ">\n";
    if (lane.link_predecessor_id || lane.link_successor_id) {
        write_indent(os, indent + 2);
        os << "<link>\n";
        if (lane.link_predecessor_id) {
            write_indent(os, indent + 4);
            os << "<predecessor" << util::attr("id", *lane.link_predecessor_id) << "/>\n";
        }
        if (lane.link_successor_id) {
            write_indent(os, indent + 4);
            os << "<successor" << util::attr("id", *lane.link_successor_id) << "/>\n";
        }
        write_indent(os, indent + 2);
        os << "</link>\n";
    } else {
        write_indent(os, indent + 2);
        os << "<link/>\n";
    }
    // A driving lane that's just materializing at a split (or about to vanish at a merge) has
    // width 0 at one end of its taper -- still a real, meaningful <width> entry, not something to
    // omit; only a lane with no width at *either* end (shouldn't normally occur) is skipped.
    if (lane.width > 0.0 || lane.width_end > 0.0) {
        write_indent(os, indent + 2);
        const double b = (lane.width_end >= 0.0 && road_length > 1e-6) ? (lane.width_end - lane.width) / road_length : 0.0;
        os << "<width" << util::attr("sOffset", 0.0) << util::attr("a", lane.width)
           << util::attr("b", b) << util::attr("c", 0.0) << util::attr("d", 0.0) << "/>\n";
    }
    write_indent(os, indent + 2);
    os << "<roadMark" << util::attr("sOffset", 0.0)
       << util::attr("type", lane.roadmark_type)
       << util::attr("weight", lane.roadmark_weight)
       << util::attr("color", lane.roadmark_color)
       << util::attr("material", "standard")
       << util::attr("laneChange", lane.lane_change) << "/>\n";
    write_indent(os, indent);
    os << "</lane>\n";
}

void write_plan_view(std::ostream& os, const model::RoadSegment& road, const int indent) {
    write_indent(os, indent);
    os << "<planView>\n";
    double s = 0.0;
    if (!road.explicit_geometry.empty()) {
        for (const auto& g : road.explicit_geometry) {
            if (g.length <= 1e-6) continue;
            write_indent(os, indent + 2);
            os << "<geometry" << util::attr("s", s)
               << util::attr("x", g.x)
               << util::attr("y", g.y)
               << util::attr("hdg", g.hdg)
               << util::attr("length", g.length) << ">\n";
            write_indent(os, indent + 4);
            if (g.kind == model::GeomKind::Arc) {
                os << "<arc" << util::attr("curvature", g.curvature) << "/>\n";
            } else if (g.kind == model::GeomKind::ParamPoly3) {
                // Bezier control points (local frame, P0 implicit at the origin) to paramPoly3
                // power-basis coefficients -- standard closed-form, specialized for P0=(0,0):
                // a=0, b=3*P1, c=3*P2-6*P1, d=P3-3*P2+3*P1 (applied per axis).
                const double bU = 3.0 * g.local_p1.x;
                const double cU = 3.0 * g.local_p2.x - 6.0 * g.local_p1.x;
                const double dU = g.local_p3.x - 3.0 * g.local_p2.x + 3.0 * g.local_p1.x;
                const double bV = 3.0 * g.local_p1.y;
                const double cV = 3.0 * g.local_p2.y - 6.0 * g.local_p1.y;
                const double dV = g.local_p3.y - 3.0 * g.local_p2.y + 3.0 * g.local_p1.y;
                os << "<paramPoly3" << util::attr("aU", 0.0) << util::attr("bU", bU)
                   << util::attr("cU", cU) << util::attr("dU", dU)
                   << util::attr("aV", 0.0) << util::attr("bV", bV)
                   << util::attr("cV", cV) << util::attr("dV", dV)
                   << util::attr("pRange", "normalized") << "/>\n";
            } else {
                os << "<line/>\n";
            }
            write_indent(os, indent + 2);
            os << "</geometry>\n";
            s += g.length;
        }
    } else {
        for (std::size_t i = 1; i < road.points.size(); ++i) {
            const auto& a = road.points[i - 1];
            const auto& b = road.points[i];
            const double len = geo::length(b - a);
            if (len <= 1e-6) continue;
            write_indent(os, indent + 2);
            os << "<geometry" << util::attr("s", s)
               << util::attr("x", a.x)
               << util::attr("y", a.y)
               << util::attr("hdg", geo::heading(a, b))
               << util::attr("length", len) << ">\n";
            write_indent(os, indent + 4);
            os << "<line/>\n";
            write_indent(os, indent + 2);
            os << "</geometry>\n";
            s += len;
        }
    }
    write_indent(os, indent);
    os << "</planView>\n";
}

// Writes a single <laneSection> at the given indent (matching the <laneSection> element's own
// position; `<left>`/`<center>`/`<right>` and their lanes are written one and two levels deeper
// respectively). `span_length` is this section's own extent, used for its lanes' width taper --
// not the whole road's length, since a merged road's later sections taper over their own span.
void write_lane_section(std::ostream& os, const double s_offset, const model::LanePlan& lanes, const double span_length, const int indent) {
    write_indent(os, indent);
    os << "<laneSection" << util::attr("s", s_offset) << ">\n";

    if (!lanes.left.empty()) {
        write_indent(os, indent + 2);
        os << "<left>\n";
        for (const auto& lane : lanes.left) write_lane(os, lane, indent + 4, span_length);
        write_indent(os, indent + 2);
        os << "</left>\n";
    }

    write_indent(os, indent + 2);
    os << "<center>\n";
    model::LaneSpec center;
    center.id = 0;
    center.type = "none";
    center.width = 0.0;
    center.roadmark_type = lanes.center_mark;
    center.lane_change = lanes.center_mark == "solid" ? "none" : "both";
    write_lane(os, center, indent + 4, span_length);
    write_indent(os, indent + 2);
    os << "</center>\n";

    if (!lanes.right.empty()) {
        write_indent(os, indent + 2);
        os << "<right>\n";
        for (const auto& lane : lanes.right) write_lane(os, lane, indent + 4, span_length);
        write_indent(os, indent + 2);
        os << "</right>\n";
    }

    write_indent(os, indent);
    os << "</laneSection>\n";
}

void write_lanes(std::ostream& os, const model::RoadSegment& road, const int indent) {
    write_indent(os, indent);
    os << "<lanes>\n";

    // Each merged section can have its own lane_offset (infer_lanes computes it independently per
    // constituent's own asymmetric lane layout); if any section actually needs one, emit one
    // <laneOffset> per section boundary so later sections aren't silently positioned using an
    // earlier, unrelated section's offset.
    bool any_offset = std::abs(road.lanes.lane_offset) > 1e-6 || std::abs(road.lanes.lane_offset_slope) > 1e-9;
    for (const auto& section : road.extra_lane_sections) {
        any_offset = any_offset || std::abs(section.lanes.lane_offset) > 1e-6 || std::abs(section.lanes.lane_offset_slope) > 1e-9;
    }
    if (any_offset) {
        write_indent(os, indent + 2);
        os << "<laneOffset" << util::attr("s", 0.0) << util::attr("a", road.lanes.lane_offset)
           << util::attr("b", road.lanes.lane_offset_slope) << util::attr("c", 0.0) << util::attr("d", 0.0) << "/>\n";
        for (const auto& section : road.extra_lane_sections) {
            write_indent(os, indent + 2);
            os << "<laneOffset" << util::attr("s", section.s_offset) << util::attr("a", section.lanes.lane_offset)
               << util::attr("b", section.lanes.lane_offset_slope) << util::attr("c", 0.0) << util::attr("d", 0.0) << "/>\n";
        }
    }

    const double first_span = road.extra_lane_sections.empty() ? road.length : road.extra_lane_sections.front().s_offset;
    write_lane_section(os, 0.0, road.lanes, first_span, indent + 2);
    for (std::size_t i = 0; i < road.extra_lane_sections.size(); ++i) {
        const auto& section = road.extra_lane_sections[i];
        const double span = (i + 1 < road.extra_lane_sections.size())
            ? (road.extra_lane_sections[i + 1].s_offset - section.s_offset)
            : (road.length - section.s_offset);
        write_lane_section(os, section.s_offset, section.lanes, span, indent + 2);
    }

    write_indent(os, indent);
    os << "</lanes>\n";
}

void write_signals(std::ostream& os, const model::RoadSegment& road, const int indent) {
    if (road.signals.empty()) return;
    write_indent(os, indent);
    os << "<signals>\n";
    for (const auto& sig : road.signals) {
        write_indent(os, indent + 2);
        os << "<signal"
           << util::attr("s", std::max(0.0, std::min(sig.s, road.length)))
           << util::attr("t", sig.t)
           << util::attr("id", sig.id)
           << util::attr("name", sig.name)
           << util::attr("dynamic", sig.dynamic ? "yes" : "no")
           << util::attr("orientation", sig.orientation)
           << util::attr("zOffset", 0.0)
           << util::attr("country", sig.country)
           << util::attr("type", sig.type)
           << util::attr("subtype", sig.subtype)
           << util::attr("unit", sig.unit)
           << util::attr("text", sig.text);
        if (sig.has_value) os << util::attr("value", sig.value);
        os << "/>\n";
    }
    write_indent(os, indent);
    os << "</signals>\n";
}

void write_road(std::ostream& os, const model::RoadSegment& road, const int indent) {
    const std::string road_name = tag_value_or(road.tags, "name", road.id);
    write_indent(os, indent);
    os << "<road" << util::attr("name", road_name)
       << util::attr("length", road.length)
       << util::attr("id", road.id)
       << util::attr("junction", road.junction_id.empty() ? std::string("-1") : road.junction_id) << ">\n";

    if (!road.predecessor_xml.empty() || !road.successor_xml.empty()) {
        write_indent(os, indent + 2);
        os << "<link>\n";
        if (!road.predecessor_xml.empty()) {
            write_indent(os, indent + 4);
            os << "<predecessor" << road.predecessor_xml << "/>\n";
        }
        if (!road.successor_xml.empty()) {
            write_indent(os, indent + 4);
            os << "<successor" << road.successor_xml << "/>\n";
        }
        write_indent(os, indent + 2);
        os << "</link>\n";
    }

    write_indent(os, indent + 2);
    os << "<type" << util::attr("s", 0.0) << util::attr("type", infer::road_type(road.tags)) << "/>\n";
    for (const auto& section : road.extra_lane_sections) {
        write_indent(os, indent + 2);
        os << "<type" << util::attr("s", section.s_offset) << util::attr("type", infer::road_type(section.tags)) << "/>\n";
    }

    write_plan_view(os, road, indent + 2);
    write_lanes(os, road, indent + 2);
    write_signals(os, road, indent + 2);

    write_indent(os, indent);
    os << "</road>\n";
}

void write_junction(std::ostream& os, const model::Junction& junction, const int indent) {
    write_indent(os, indent);
    os << "<junction" << util::attr("name", "junction_" + std::to_string(junction.node_ref)) << util::attr("id", junction.id) << ">\n";
    for (const auto& c : junction.connections) {
        write_indent(os, indent + 2);
        os << "<connection" << util::attr("id", c.id)
           << util::attr("incomingRoad", c.incoming_road)
           << util::attr("connectingRoad", c.connecting_road)
           << util::attr("contactPoint", c.contact_point) << ">\n";
        for (const auto& [from, to] : c.lane_links) {
            write_indent(os, indent + 4);
            os << "<laneLink" << util::attr("from", from) << util::attr("to", to) << "/>\n";
        }
        write_indent(os, indent + 2);
        os << "</connection>\n";
    }
    write_indent(os, indent);
    os << "</junction>\n";
}

void write_file(const model::MapModel& model, const Options& options) {
    std::ofstream os(options.output);
    if (!os) util::fail("Could not open output file: " + options.output);

    os << "<?xml version=\"1.0\" standalone=\"yes\"?>\n";
    os << "<OpenDRIVE>\n";
    write_indent(os, 2);
    os << "<header"
       << util::attr("revMajor", 1)
       << util::attr("revMinor", 4)
       << util::attr("name", options.name)
       << util::attr("version", "1.00")
       << util::attr("date", "2026-07-03") // TODO: set current date
       << util::attr("north", model.north)
       << util::attr("south", model.south)
       << util::attr("east", model.east)
       << util::attr("west", model.west)
       << util::attr("vendor", "osm2xodr") << ">\n";
    write_indent(os, 4);
    os << "<geoReference><![CDATA[+proj=eqc +lat_ts=" << std::fixed << std::setprecision(8) << model.projector.origin.lat
       << " +lat_0=" << model.projector.origin.lat << " +lon_0=" << model.projector.origin.lon
       << " +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>\n";
    write_indent(os, 2);
    os << "</header>\n";

    for (const auto& road : model.roads) write_road(os, road, 2);
    for (const auto& junction : model.junctions) write_junction(os, junction, 2);

    os << "</OpenDRIVE>\n";
}

} // namespace osm2xodr::xodr
