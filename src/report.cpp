#include "osm2xodr/report.hpp"

#include "osm2xodr/util.hpp"

#include <fstream>
#include <iomanip>

namespace osm2xodr {

void write_report(const model::MapModel& model, const Options& options, const osm::ParseResult& parsed) {
    if (options.report_path.empty()) return;
    std::ofstream os(options.report_path);
    if (!os) util::fail("Could not open report file: " + options.report_path);
    os << "osm2xodr conversion report\n";
    os << "=========================\n\n";
    os << "Input: " << options.input << "\n";
    os << "Output: " << options.output << "\n";
    std::size_t connector_count = 0;
    for (const auto& r : model.roads) if (!r.junction_id.empty()) ++connector_count;
    os << "OSM road ways parsed: " << parsed.roads.size() << "\n";
    os << "OpenDRIVE roads written: " << model.roads.size() << " (" << connector_count << " junction connector roads)\n";
    os << "Junctions written: " << model.junctions.size() << "\n";
    if (model.compound_junction_count > 0) {
        os << "Compound junctions: " << model.compound_junction_count << " (merging "
           << model.compound_junction_node_total << " OSM junction nodes)\n";
    }
    std::size_t signal_count = 0;
    for (const auto& r : model.roads) signal_count += r.signals.size();
    os << "Signals/signs written: " << signal_count << "\n";
    os << "Projection origin: lat=" << model.projector.origin.lat << " lon=" << model.projector.origin.lon << "\n\n";

    os << "Road summary\n";
    os << "------------\n";
    for (const auto& r : model.roads) {
        if (!r.junction_id.empty()) {
            os << r.id << " junction=" << r.junction_id
               << " length=" << std::fixed << std::setprecision(2) << r.length << " (connector)\n";
            continue;
        }
        os << r.id << " source_way=" << r.source_way_id
           << " length=" << std::fixed << std::setprecision(2) << r.length
           << " forward=" << r.lanes.forward_lanes
           << " backward=" << r.lanes.backward_lanes
           << " left_lanes=" << r.lanes.left.size()
           << " right_lanes=" << r.lanes.right.size()
           << " signals=" << r.signals.size() << "\n";
    }

    os << "\nWarnings\n";
    os << "--------\n";
    if (model.warnings.empty()) os << "None\n";
    for (const auto& w : model.warnings) os << "- " << w << "\n";
}

} // namespace osm2xodr
