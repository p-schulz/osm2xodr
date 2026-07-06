#include "osm2xodr/cli.hpp"
#include "osm2xodr/model_builder.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/report.hpp"
#include "osm2xodr/xodr_writer.hpp"

#include <exception>
#include <iostream>

namespace osm2xodr {

int run(const int argc, char** argv) {
    const Options options = parse_args(argc, argv);
    auto parsed = osm::parse_osm(options);
    auto model = build::build_model(parsed, options);
    xodr::write_file(model, options);
    write_report(model, options, parsed);
    if (options.validate) validate_with_libopendrive(options.output);

    std::cerr << "Wrote " << options.output << " with " << model.roads.size() << " roads and " << model.junctions.size() << " junctions.\n";
    if (!model.warnings.empty()) {
        std::cerr << model.warnings.size() << " warning(s).";
        if (!options.report_path.empty()) std::cerr << " See " << options.report_path << ".";
        std::cerr << "\n";
    }
    return 0;
}

} // namespace osm2xodr

int main(int argc, char** argv) {
    try {
        return osm2xodr::run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
