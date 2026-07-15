#include "osm2xodr/cli.hpp"

#include "osm2xodr/config.hpp"
#include "osm2xodr/util.hpp"

#ifdef OSM2XODR_WITH_LIBOPENDRIVE
#include <OpenDriveMap.h>
#endif

#include <cstdlib>
#include <iostream>

namespace osm2xodr {

#ifdef OSM2XODR_WITH_LIBOPENDRIVE
void validate_with_libopendrive(const std::string& path) {
    odr::OpenDriveMap map(path);
    std::cerr << "libOpenDRIVE read-back: " << map.get_roads().size() << " roads parsed\n";
}
#else
void validate_with_libopendrive(const std::string&) {
    util::fail("--validate requested, but osm2xodr was built without OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON");
}
#endif

Options parse_args(const int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: osm2xodr <input.osm|osm.pbf> <output.xodr> [options]\n";
        std::cerr << "Try --help for details.\n";
        std::exit(2);
    }

    Options o;
    o.input = argv[1];
    o.output = argv[2];

    auto require_value = [&](int& i, const std::string& opt) -> std::string {
        if (i + 1 >= argc) util::fail("Missing value after " + opt);
        return argv[++i];
    };

    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "Usage: osm2xodr <input.osm|osm.pbf> <output.xodr> [options]\n\n"
                      << "Options:\n"
                      << "  --name <name>                  OpenDRIVE header name\n"
                      << "  --origin-lat <deg>             Override projection origin latitude\n"
                      << "  --origin-lon <deg>             Override projection origin longitude\n"
                      << "  --default-lane-width <m>       Default lane width, default 3.50\n"
                      << "  --sidewalk-width <m>           Sidewalk lane width, default 2.00\n"
                      << "  --left-hand-traffic            Use left-hand lane-direction assumptions\n"
                      << "  --junction-degree <n>          Minimum endpoint degree for junction, default 3\n"
                      << "  --signal-search-radius <m>     Max signal/sign matching distance, default 20\n"
                      << "  --junction-turn-radius <m>     Fallback connector turn radius for road classes\n"
                      << "                                 without a specific tier, default 8.0\n"
                      << "  --no-adaptive-turn-radius      Disable scaling connector turn radius by each road's\n"
                      << "                                 maxspeed tag and the connector's own deflection angle\n"
                      << "  --no-road-merge                Keep one <road> per OSM way segment (disable merging)\n"
                      << "  --junction-cluster-max-gap <m> Max length of an inter-junction road to fold into one\n"
                      << "                                 compound junction, default 20.0\n"
                      << "  --no-junction-merge            Disable compound-junction clustering\n"
                      << "  --junction-signal-setback-max-gap <m>\n"
                      << "                                 Max length of a traffic-light-to-junction stub road to\n"
                      << "                                 absorb into the junction, default 15.0\n"
                      << "  --no-signal-setback-absorption Disable absorbing traffic-light setback stubs into junctions\n"
                      << "  --lane-taper-length <m>        Fallback target length over which an added/dropped lane's\n"
                      << "                                 width ramps to/from zero at a merge/split, default 15.0\n"
                      << "  --no-adaptive-lane-taper       Disable scaling the lane taper length by each road's\n"
                      << "                                 maxspeed tag (German taper/Verziehungslaenge convention)\n"
                      << "  --no-lane-count-bridge         Disable inserting a short connecting road to reconcile a\n"
                      << "                                 real lane-count change at a plain (non-junction) road-to-\n"
                      << "                                 road boundary, e.g. right before a traffic signal\n"
                      << "  --no-curve-fit                 Keep piecewise <line> planView geometry for non-junction\n"
                      << "                                 roads instead of fitted <paramPoly3> curves\n"
                      << "  --no-link-continuity-fix       Disable fixing up heading mismatches at plain road-to-\n"
                      << "                                 road link boundaries (only takes effect with curve-fit on)\n"
                      << "  --config <path.yaml>           Load a config file; currently supports one key,\n"
                      << "                                 ignore_highways: a list of OSM highway=* values to\n"
                      << "                                 exclude entirely (e.g. ignore_highways: [service, track])\n"
                      << "  --report <file>                Write conversion report\n"
                      << "  --validate                     Read generated XODR back with libOpenDRIVE if enabled\n";
            std::exit(0);
        } else if (arg == "--name") {
            o.name = require_value(i, arg);
        } else if (arg == "--origin-lat") {
            o.origin_lat = util::parse_double_prefix(require_value(i, arg));
        } else if (arg == "--origin-lon") {
            o.origin_lon = util::parse_double_prefix(require_value(i, arg));
        } else if (arg == "--default-lane-width") {
            o.default_lane_width = util::parse_double_prefix(require_value(i, arg)).value_or(o.default_lane_width);
        } else if (arg == "--sidewalk-width") {
            o.sidewalk_width = util::parse_double_prefix(require_value(i, arg)).value_or(o.sidewalk_width);
        } else if (arg == "--left-hand-traffic") {
            o.left_hand_traffic = true;
        } else if (arg == "--junction-degree") {
            o.junction_degree = util::parse_int(require_value(i, arg)).value_or(o.junction_degree);
        } else if (arg == "--signal-search-radius") {
            o.signal_search_radius = util::parse_double_prefix(require_value(i, arg)).value_or(o.signal_search_radius);
        } else if (arg == "--junction-turn-radius") {
            o.junction_turn_radius = util::parse_double_prefix(require_value(i, arg)).value_or(o.junction_turn_radius);
        } else if (arg == "--no-adaptive-turn-radius") {
            o.adaptive_turn_radius = false;
        } else if (arg == "--no-road-merge") {
            o.merge_roads = false;
        } else if (arg == "--junction-cluster-max-gap") {
            o.junction_cluster_max_gap = util::parse_double_prefix(require_value(i, arg)).value_or(o.junction_cluster_max_gap);
        } else if (arg == "--no-junction-merge") {
            o.merge_junctions = false;
        } else if (arg == "--junction-signal-setback-max-gap") {
            o.junction_signal_setback_max_gap = util::parse_double_prefix(require_value(i, arg)).value_or(o.junction_signal_setback_max_gap);
        } else if (arg == "--no-signal-setback-absorption") {
            o.absorb_signal_setbacks = false;
        } else if (arg == "--lane-taper-length") {
            o.lane_taper_length = util::parse_double_prefix(require_value(i, arg)).value_or(o.lane_taper_length);
        } else if (arg == "--no-adaptive-lane-taper") {
            o.adaptive_lane_taper = false;
        } else if (arg == "--no-lane-count-bridge") {
            o.bridge_lane_count_changes = false;
        } else if (arg == "--no-curve-fit") {
            o.curve_fit = false;
        } else if (arg == "--no-link-continuity-fix") {
            o.fix_link_continuity = false;
        } else if (arg == "--config") {
            config::load_config_file(require_value(i, arg), o);
        } else if (arg == "--report") {
            o.report_path = require_value(i, arg);
        } else if (arg == "--validate") {
            o.validate = true;
        } else {
            util::fail("Unknown option: " + arg);
        }
    }

    if (o.default_lane_width <= 0.1) util::fail("--default-lane-width must be positive");
    if (o.sidewalk_width <= 0.1) util::fail("--sidewalk-width must be positive");
    if (o.junction_degree < 2) util::fail("--junction-degree must be at least 2");
    if (o.signal_search_radius <= 0.0) util::fail("--signal-search-radius must be positive");
    if (o.junction_turn_radius <= 0.0) util::fail("--junction-turn-radius must be positive");
    if (o.junction_cluster_max_gap < 0.0) util::fail("--junction-cluster-max-gap must not be negative");
    if (o.junction_signal_setback_max_gap < 0.0) util::fail("--junction-signal-setback-max-gap must not be negative");
    if (o.lane_taper_length <= 0.0) util::fail("--lane-taper-length must be positive");
    return o;
}

} // namespace osm2xodr
