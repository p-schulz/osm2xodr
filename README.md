# osm2xodr

![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%E2%89%A5%203.20-064F8C?logo=cmake&logoColor=white)
![OpenDRIVE](https://img.shields.io/badge/OpenDRIVE-1.4-orange)
![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20Windows-lightgrey)

**`osm2xodr`** is a C++17 command-line converter that reads OpenStreetMap files and writes an approximate OpenDRIVE 1.4 road-network file.

It uses **[libosmium](https://github.com/osmcode/libosmium)** for OpenStreetMap parsing and node-location handling, and writes OpenDRIVE XML directly. **libOpenDRIVE** is optional and used only for read-back validation with `--validate`; it is not required for normal conversion.

## Table of contents

- [Current capabilities](#current-capabilities)
- [Important limitations](#important-limitations)
- [Quick start](#quick-start)
- [Repository layout](#repository-layout)
- [Dependencies](#dependencies)
- [Building from source](#building-from-source)
- [Optional libOpenDRIVE validation](#optional-libopendrive-validation)
- [Manual/vendor libosmium setup](#manualvendor-libosmium-setup)
- [Usage](#usage)
- [Input notes](#input-notes)
- [Troubleshooting](#troubleshooting)
- [Development notes](#development-notes)
- [Testing](#testing)
- [Upstream references](#upstream-references)

## Current capabilities

The converter is deliberately conservative. It creates usable, inspectable `.xodr` output, but OSM does not contain enough information to reconstruct all OpenDRIVE semantics exactly.

**Features:**

- **OSM input** — `.osm`, `.osm.bz2`, and `.osm.pbf` through libosmium.
- **Road extraction** from OSM `highway=*` ways.
- **Topology splitting** — OSM ways are split at road-intersection nodes so topology can be represented at road boundaries.
- **Road merging** — chains of way segments connected only by a plain pass-through node (not a junction, not a traffic-light/stop/give-way node) are fused back into a single OpenDRIVE `<road>`, even across OSM tag changes (lane count, width, name, highway class) &mdash; cross-section changes partway along a merged road become additional `<laneSection>`/`<type>` elements rather than a new `<road>`. Disable with `--no-road-merge` to keep one `<road>` per OSM way segment.
- **Lane split/merge tapering** — when a merge boundary's lane count actually changes (as opposed to a same-count width/type/roadmark change), the appearing/disappearing lane ramps its width to/from zero over `--lane-taper-length` (default 15m, capped to 40% of the adjacent way segment's own length) instead of popping in or out abruptly at the boundary, and the road's reference-line `<laneOffset>` gets a matching slope so it stays centered on the true (tapering) total cross-section width throughout. Which physical lane is the one appearing/disappearing (leftmost or rightmost) is decided by comparing `turn:lanes`-derived permitted directions on the lanes present on both sides of the boundary, not by assuming it's always the outermost one.
- **Local metric projection** using an equirectangular tangent approximation around the first coordinate or a user-supplied origin.
- **PlanView geometry** — OpenDRIVE `<road>` elements with `<planView>` line geometries for each OSM polyline segment.
- **Road linking** — predecessor/successor links for non-junction endpoints.
- **Junction detection** at nodes with degree >= 3.
- **Compound junction clustering** — real intersections are sometimes mapped as several close-together OSM junction nodes (tram tracks crossing at a slightly offset point, a wide junction's corner geometry, etc.) rather than one point. Any inter-junction road short enough to have no traffic light along it (`--junction-cluster-max-gap`, default 20m) is folded into one compound `<junction>` instead of a chain of tiny separate ones, so connector lanes span directly between the real approaches without leftover stub roads in between. Disable with `--no-junction-merge`.
- **Physically modelled junction connectors** — incoming/outgoing roads are shortened back from the junction node, and each incoming-lane-to-outgoing-lane movement gets its own connector `<road>` (line/arc/line planView) sized from a tangent-fillet turn radius, so lane heading and position match exactly at both ends. The turn radius is tiered by OSM highway class (motorway/trunk down to service), with `--junction-turn-radius` as the fallback for unmapped classes, and re-maximized against each road end's actual available setback so a shared endpoint's spare room becomes a longer, gentler curve rather than a small arc padded with straight lead-in/lead-out. Every movement — not just ones that already fit their own per-movement budget — claims whatever budget is actually available at its road ends (capped to what exists), so a movement that's "infeasible" only because its own ideal/floor-forced radius exceeded its own share still gets a real, trimmed connector rather than an untrimmed, silently-kinked direct link. When a movement's own turn radius doesn't fit its own trim budget but another movement sharing the same road end still forces that end to be trimmed back, the radius is re-fit against the final trim (shrinking if necessary) instead of falling back to a bare road-to-road link that would silently assume a now-untrue zero gap; if even that doesn't produce a sane direction (checked against a 45° tolerance to both the incoming and outgoing lane's own heading), the movement falls back to a direct road-to-road junction link, same as when a connector cannot fit within the available road geometry at all (very short segments, or a bend too close to the junction).
- **Turn-lane-aware junction routing** — `turn:lanes`/`turn:lanes:forward`/`turn:lanes:backward` (cross-checked against `access:lanes`/`vehicle:lanes` to exclude interleaved bike/bus lane slots) gives the real car-lane count and each lane's permitted direction(s), overriding the often-inconsistent plain `lanes=N` tag. A lane with a parsed restriction only connects to junction movements whose actual geometric direction (through/left/right/slight_*/sharp_*) matches it, instead of every incoming lane wiring to every destination. That direction is classified from a look-ahead chord along the destination/approach road's own reference line (default 15m, capped to the road's own length) rather than the immediate first-micro-segment tangent the connector geometry itself must use for continuity -- a mapped ramp/slip-lane approximated with several short internal segments can curve well past its first meter or two, and classifying from only that first sliver would call a road that ends up turning 90+ degrees "through" simply because it starts out nearly straight. A lane tagged plain `left`/`right` also matches the immediately adjacent `slight_left`/`slight_right` classification (not just an exact bucket match), since most OSM mappers don't bother distinguishing a gentle bend from a "full" turn unless they specifically mean to.
- **Per-lane `<link>` continuity** at plain (non-junction) road-to-road boundaries — e.g. a feature-split traffic light/stop sign, or any two separate roads meeting directly at a node. Previously only the road-level `<predecessor>/<successor>` was written for this case; each lane's own `<link>` is now populated too (matched positionally by type, reciprocal on both sides), working out which physical side carries arriving vs. departing traffic for all four possible combinations of which road end (start/end) touches the shared node, and respecting `--left-hand-traffic`.
- **Lane-count inference** from `lanes`, `lanes:forward`, `lanes:backward`, `oneway`, and roundabout tags.
- **Lane widths** from `width`, `width:lanes`, `width:lanes:forward/backward`, or highway defaults.
- **Lane offsets** for asymmetric/one-way lane layouts.
- **Road markings** from `lane_markings`, `overtaking`, and simple default heuristics.
- **Sidewalk lanes** from `sidewalk`, `sidewalk:left`, and `sidewalk:right`.
- **Traffic lights** from `highway=traffic_signals` nodes.
- **Stop/give-way/sign nodes** from `highway=stop`, `highway=give_way`, and `traffic_sign=*`.
- **Speed limits** — way-level `maxspeed=*` exported as OpenDRIVE signal records.
- **Conversion report** with warnings about inferred or lossy data.

## Important limitations

- Non-junction road geometry is generated as piecewise `<line>` segments; no fitted spirals or paramPoly3 curves for the OSM polylines themselves (junction connectors do use `<arc>` geometry). A road's own segments faithfully reproduce whatever bends exist in the source OSM polyline -- a small heading "kink" between two adjacent segments of the same named way is usually the mapper's own digitization, not a converter error (OSM node positions are frequently only accurate to a few meters).
- A rescued junction connector (see above) trades perfect tangent alignment for guaranteed connectivity when the available trim room is tight: its position always matches the incoming/outgoing lanes exactly, but its heading may be off by anywhere from a fraction of a degree up to (rarely) several tens of degrees before the 45° sanity check gives up and falls back to a plain direct link instead. `test/check_junction_continuity.py` reports these as mismatches so they stay visible; they are not silently wrong.
- `restriction=*` turn-restriction relations are not read; only per-lane `turn:lanes` data (when present on the way itself) filters junction routing. A lane with no `turn:lanes` tag still wires to every compatible destination, as before.
- Compound junction clustering is a general, distance-capped mechanism, not a semantic one: it cannot distinguish "one physical intersection mapped with extra nodes" from "two genuinely separate intersections on a short, unsignalled block" by geometry alone. `--junction-cluster-max-gap` is a blunt safety valve, tuned against real tram-adjacent intersections; very short ordinary city blocks could in principle be merged incorrectly.
- The lane split/merge taper length is a fixed heuristic (`--lane-taper-length`), not derived from design speed or any real-world taper-rate standard, and the "which side is the new lane" decision falls back to "assume it's at the end" when turn-direction overlap scoring ties (e.g. neither side's lanes carry any `turn:lanes` data at all).
- OSM sign tagging is country-specific and inconsistent; the converter preserves raw sign text/type where exact OpenDRIVE catalogue mapping is not known.
- Standalone sidewalk ways are not fused into adjacent roads; sidewalk support is lane-based through road tags.
- Elevation, superelevation, detailed traffic-light phases, turn restrictions, lane-specific access, and relation-based complex intersections are scaffolding targets, not fully solved in this prototype.

## Quick start

On a Debian/Ubuntu system with the required packages available (see [Dependencies](#dependencies)):

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential cmake git \
  libosmium2-dev libprotozero-dev libboost-dev \
  libbz2-dev libexpat1-dev zlib1g-dev

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/osm2xodr examples/tiny_intersection.osm examples/tiny_intersection.xodr \
  --report examples/tiny_intersection_report.txt
```

For Windows, Fedora, Arch, vcpkg, or a manually vendored dependency tree, see [Building from source](#building-from-source).

## Repository layout

```text
osm2xodr/
  CMakeLists.txt
  README.md
  vcpkg.json                  Optional vcpkg manifest for libosmium
  cmake/FindOsmium.cmake       Fallback CMake finder for libosmium
  include/osm2xodr/            Public headers, one per module
    util.hpp                    String/number parsing helpers
    geo.hpp                      Local projection, vector math, polyline helpers
    options.hpp                   Options (CLI-configurable parameters)
    tags.hpp                       OSM tag helpers
    osm_parse.hpp                   libosmium handler and raw OSM feature extraction
    model.hpp                       Normalized road/lane/signal/junction structures
    infer.hpp                        OSM tag-to-road/lane/signal inference
    model_builder.hpp                 build::build_model() entry point
    xodr_writer.hpp                    OpenDRIVE XML writer
    report.hpp                          Conversion report writer
    cli.hpp                              Command-line argument parsing
  src/                          Implementation; one .cpp per header, plus main.cpp
    main.cpp                     Thin CLI entry point (run() + main())
    cli.cpp
    osm_parse.cpp
    infer.cpp
    model_builder.cpp             ModelBuilder: road-fragment/merge/junction/signal pipeline
    xodr_writer.cpp
    report.cpp
  examples/tiny_intersection.osm
  external/                    Optional manual/vendor dependencies
    libosmium/                 Optional; only needed if not installed system-wide
    protozero/                 Optional; only needed if not installed system-wide
    libOpenDRIVE/              Optional; only needed if not using FetchContent
```

Keep third-party code under `external/` or install it through your package manager. Do not copy libosmium or libOpenDRIVE into `src/`.

## Dependencies

Required for normal conversion:

- CMake 3.20 or newer.
- A C++17 compiler.
- Git, if CMake needs to fetch optional dependencies.
- libosmium headers.
- protozero headers, required by libosmium for PBF support.
- zlib, bzip2, expat, and threads for libosmium I/O.
- Boost headers used by libosmium on some platforms/packages.

Optional:

- libOpenDRIVE, only if `OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON` and you want `--validate`.

<details>
<summary><strong>Which library files are actually needed?</strong></summary>

For libosmium, you need the header tree:

```text
libosmium/include/osmium/...
```

For PBF input you also need protozero headers:

```text
protozero/include/protozero/...
```

You do not build or link a `libosmium` binary; libosmium is header-only. The linked libraries are the compression/XML dependencies used by libosmium I/O.

For libOpenDRIVE, use the full source tree if you vendor it:

```text
libOpenDRIVE/CMakeLists.txt
libOpenDRIVE/include/...
libOpenDRIVE/src/...
```

Do not copy only `OpenDriveMap.h`; that header depends on the rest of libOpenDRIVE.

</details>

## Building from source

<details>
<summary><strong>Ubuntu / Debian</strong></summary>

Recommended system-package setup:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libosmium2-dev \
  libprotozero-dev \
  libboost-dev \
  libbz2-dev \
  libexpat1-dev \
  zlib1g-dev
```

Build the converter:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run the bundled example:

```bash
./build/osm2xodr examples/tiny_intersection.osm examples/tiny_intersection.xodr \
  --report examples/tiny_intersection_report.txt
```

If your generator places the binary in a configuration subdirectory, use `find build -name osm2xodr -type f` to locate it.

</details>

<details>
<summary><strong>Fedora</strong></summary>

```bash
sudo dnf install -y \
  gcc-c++ \
  cmake \
  git \
  libosmium-devel \
  protozero-devel \
  boost-devel \
  bzip2-devel \
  expat-devel \
  zlib-devel

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Package names differ a little across Fedora releases. If `libosmium-devel` is not available, use the vcpkg setup below or vendor libosmium manually.

</details>

<details>
<summary><strong>Arch Linux</strong></summary>

```bash
sudo pacman -Syu --needed \
  base-devel \
  cmake \
  git \
  libosmium \
  protozero \
  boost \
  bzip2 \
  expat \
  zlib

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

</details>

<details>
<summary><strong>Linux with vcpkg</strong></summary>

This repository includes a minimal `vcpkg.json` manifest. From the project root:

```bash
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=$HOME/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build -j
```

vcpkg will install `libosmium` and its declared dependencies for the active triplet.

</details>

<details>
<summary><strong>Windows (Visual Studio 2022 + CMake + vcpkg)</strong></summary>

The recommended Windows setup is **Visual Studio 2022 + CMake + vcpkg**. Use a 64-bit build. libosmium does not support 32-bit builds.

**1. Install tools**

- Visual Studio 2022 with the **Desktop development with C++** workload.
- Git for Windows.
- CMake, either from Visual Studio or from cmake.org.

Open a **Developer PowerShell for VS 2022** or **x64 Native Tools Command Prompt for VS 2022**.

**2. Install vcpkg**

```powershell
git clone https://github.com/microsoft/vcpkg C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat
```

**3. Configure and build with the Visual Studio generator**

From the `osm2xodr` project root:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake

cmake --build build --config Release
```

Run the example:

```powershell
.\build\Release\osm2xodr.exe .\examples\tiny_intersection.osm .\examples\tiny_intersection.xodr `
  --report .\examples\tiny_intersection_report.txt
```

**4. Alternative: build with Ninja**

Install Ninja or use the one bundled with Visual Studio, then run from a VS developer shell:

```powershell
cmake -S . -B build-ninja -G Ninja `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake

cmake --build build-ninja

.\build-ninja\osm2xodr.exe .\examples\tiny_intersection.osm .\examples\tiny_intersection.xodr
```

</details>

## Optional libOpenDRIVE validation

Normal conversion does not require libOpenDRIVE. Enable it only when you want this command to parse the generated `.xodr` back after writing:

```bash
./build/osm2xodr input.osm.pbf output.xodr --validate
```

<details>
<summary><strong>Fetch libOpenDRIVE automatically</strong></summary>

Linux/macOS shell:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON
cmake --build build -j
```

Windows PowerShell:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON
cmake --build build --config Release
```

CMake will fetch libOpenDRIVE with `FetchContent`. This requires Git and internet access during configure.

</details>

<details>
<summary><strong>Use a local/vendor libOpenDRIVE checkout</strong></summary>

Place it here:

```text
external/libOpenDRIVE/
```

Then configure with validation enabled:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON \
  -DOSM2XODR_LIBOPENDRIVE_SOURCE_DIR=$PWD/external/libOpenDRIVE
cmake --build build -j
```

On Windows PowerShell:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON `
  -DOSM2XODR_LIBOPENDRIVE_SOURCE_DIR=$PWD\external\libOpenDRIVE
cmake --build build --config Release
```

The local directory must contain libOpenDRIVE's own `CMakeLists.txt`, `include/`, and `src/` directories.

</details>

## Manual/vendor libosmium setup

Use this only if your package manager or vcpkg cannot provide libosmium.

Recommended layout:

```text
osm2xodr/external/libosmium/include/osmium/...
osm2xodr/external/protozero/include/protozero/...
```

Configure with explicit include paths:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOSMIUM_INCLUDE_DIR=$PWD/external/libosmium/include \
  -DPROTOZERO_INCLUDE_DIR=$PWD/external/protozero/include
cmake --build build -j
```

You still need zlib, bzip2, expat, threads, and Boost headers installed or discoverable by CMake. On Windows, manual dependency wiring is possible but not pleasant; vcpkg is strongly preferred.

## Usage

```bash
./build/osm2xodr input.osm.pbf output.xodr --name "example-map"
```

On Windows with the Visual Studio generator:

```powershell
.\build\Release\osm2xodr.exe input.osm.pbf output.xodr --name "example-map"
```

Useful options:

| Option | Description |
| --- | --- |
| `--origin-lat <deg>` | Override local projection origin latitude |
| `--origin-lon <deg>` | Override local projection origin longitude |
| `--default-lane-width <m>` | Default width when OSM has no width tags; default `3.50` |
| `--sidewalk-width <m>` | Width of generated sidewalk lanes; default `2.00` |
| `--left-hand-traffic` | Use left-hand lane-direction assumptions |
| `--junction-degree <n>` | Minimum endpoint degree to classify as junction; default `3` |
| `--signal-search-radius <m>` | Max distance for matching sign/signal nodes to roads; default `20` |
| `--junction-turn-radius <m>` | Fallback connector turn radius for highway classes without a specific tier; default `8.0` |
| `--no-road-merge` | Keep one `<road>` per OSM way segment (disable road merging) |
| `--junction-cluster-max-gap <m>` | Max length of an inter-junction road to fold into one compound junction; default `20.0` |
| `--no-junction-merge` | Disable compound-junction clustering |
| `--lane-taper-length <m>` | Target length for a lane split/merge width taper; default `15.0` |
| `--report <file>` | Write a conversion report |
| `--validate` | Read generated `.xodr` back with libOpenDRIVE, when compiled with validation support |

## Input notes

The converter expects normal OSM data containing nodes and ways. It does not download OSM data itself. Use an extract from Geofabrik, Overpass, JOSM, or another OSM source, then pass the local `.osm`, `.osm.bz2`, or `.osm.pbf` file to `osm2xodr`.

For small tests, prefer `.osm` XML. For larger maps, prefer `.osm.pbf`.

## Troubleshooting

<details>
<summary><strong>CMake cannot find libosmium</strong></summary>

Check that the headers exist:

```bash
ls /usr/include/osmium/version.hpp
```

or, for a manual checkout:

```bash
ls external/libosmium/include/osmium/version.hpp
```

Then pass the include path explicitly:

```bash
cmake -S . -B build -DOSMIUM_INCLUDE_DIR=/path/to/libosmium/include
```

</details>

<details>
<summary><strong>CMake or the compiler cannot find protozero</strong></summary>

Install `libprotozero-dev` / `protozero-devel`, use vcpkg, or pass:

```bash
cmake -S . -B build -DPROTOZERO_INCLUDE_DIR=/path/to/protozero/include
```

</details>

<details>
<summary><strong>Link errors for zlib, bzip2, or expat</strong></summary>

Install the development packages, not only the runtime packages. On Debian/Ubuntu these are `zlib1g-dev`, `libbz2-dev`, and `libexpat1-dev`.

</details>

<details>
<summary><strong>Windows build accidentally targets Win32</strong></summary>

Use `-A x64` with the Visual Studio generator. Do not build Win32.

</details>

<details>
<summary><strong><code>--validate</code> says validation support is disabled</strong></summary>

Reconfigure with:

```bash
cmake -S . -B build -DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON
```

Then rebuild.

</details>

<details>
<summary><strong>libOpenDRIVE fetch fails</strong></summary>

Use a local checkout and pass `-DOSM2XODR_LIBOPENDRIVE_SOURCE_DIR=/path/to/libOpenDRIVE`, or disable validation. The converter itself does not need libOpenDRIVE.

</details>

## Development notes

The converter is split into headers under `include/osm2xodr/` and matching sources under `src/` (one pair per module), with `src/main.cpp` reduced to a thin `run()`/`main()` entry point. The internal modules, each its own namespace:

- `util` (`util.hpp`, header-only): string/number parsing helpers.
- `geo` (`geo.hpp`, header-only): local projection, vector math, length/projection helpers.
- `osm` (`osm_parse.hpp`/`osm_parse.cpp`): libosmium handler and raw OSM feature extraction.
- `model` (`model.hpp`, header-only): normalized road, lane, signal, and junction structures.
- `infer` (`infer.hpp`/`infer.cpp`): OSM tag-to-road/lane/signal inference.
- `build` (`model_builder.hpp`/`model_builder.cpp`): builds the normalized `model::MapModel` from parsed OSM data — see below.
- `xodr` (`xodr_writer.hpp`/`xodr_writer.cpp`): OpenDRIVE XML writer.
- `report.hpp`/`report.cpp` and `cli.hpp`/`cli.cpp` (both in the `osm2xodr` namespace): conversion report writer and command-line argument parsing, respectively.

`build::build_model()` is a one-line wrapper around a `ModelBuilder` class (`src/model_builder.cpp`) whose pipeline is a fixed sequence of phase methods, each responsible for one part of the pipeline described below: `build_fragments()` (per-way splitting at junction/feature-split nodes), `merge_roads()`, `cluster_compound_junctions()`, `link_plain_roads()`, `build_junction_connectors()`, and `place_signals()`. Fields on `ModelBuilder` hold only the state that must survive across phases (endpoint/junction node maps, compound-junction cluster membership); state local to one phase is a local variable inside that phase's method, same as it always was.

<details>
<summary><strong>Implementation details for junction connectors, road merging, clustering, turn-lane routing, and lane tapering</strong></summary>

Junction connector-road generation (tangent-fillet arcs sized by tiered turn radius, per incoming/outgoing lane pair) is implemented; see `build::PendingConnector` and the two-pass construction in `ModelBuilder::build_junction_connectors`. A movement whose own radius doesn't fit its own trim budget is re-fit against the endpoint's final applied trim (which can be larger, from another movement sharing that end) via the same tangent math, rather than immediately falling back to a link; when even that can't produce a sane heading (checked against both `dir_in`/`dir_out` within 45°), it falls back to a direct road-to-road link -- see the `fitted`/`needs_direct_bridge` logic in the pass-2 loop.

Road merging (fusing pass-through-connected way segments into fewer, longer `<road>` elements with multiple `<laneSection>`/`<type>` boundaries where needed) is implemented; see `build::fuse_chain`, `build::build_merge_chains` (both invoked from `ModelBuilder::merge_roads`), and `model::LaneSection`/`RoadSegment::extra_lane_sections`.

Compound junction clustering (folding several close OSM junction nodes into one `<junction>`) is implemented via a union-find over "interior" roads (both ends are junction nodes, short enough to have no traffic light in between); see `ModelBuilder::cluster_compound_junctions` and `model::MapModel::compound_junction_count`.

Turn-lane-aware routing (`infer::decode_turn_lanes`, `model::LaneSpec::turn_directions`, `build::lane_allows_movement`/`turn_bucket_for_delta`) restricts junction connections to geometrically-compatible movements when a lane carries `turn:lanes` data, classifying each movement's bucket from `build::classification_direction_away_from_junction`/`classification_direction_into_junction` (a look-ahead chord along the road's own reference line, separate from the immediate-tangent `direction_away_from_junction`/`direction_into_junction` the connector geometry itself uses) so a curving ramp/slip-lane isn't misread as "through" just because it starts out nearly straight; every movement (not only ones already within their own budget) also claims whatever trim budget is available at its road ends via the unconditional `bump()` in pass 1, so an otherwise-infeasible movement still gets a real, trimmed connector instead of an untrimmed direct link.

Per-lane `<link>` continuity at plain road-to-road boundaries is implemented via `build::link_plain_road_lanes`/`link_lane_side`, called from the two-endpoint case in `ModelBuilder::link_plain_roads`.

Lane split/merge tapering (`build::align_lane_run`, `model::compute_lane_offset`) is implemented inside `build::fuse_chain`: a real lane-count change between two chain parts inserts a short taper-out and/or taper-in `LaneSection` (width ramping via `LaneSpec::width`/`width_end`, reference-line `lane_offset`/`lane_offset_slope` recentered to match) around the boundary, instead of the lane count changing abruptly; `align_lane_run` decides which physical lane is the one appearing/disappearing by comparing `turn_directions` overlap under both "extra lane at the start" and "extra lane at the end" hypotheses.

The next engineering steps are curve-fitted `paramPoly3` planView geometry (replacing the current piecewise `<line>` output for non-junction roads), `restriction=*` relation support, and superelevation/elevation profiles.

</details>

## Testing

Three independent, regex-based (not a full OpenDRIVE parser) verification scripts under `test/` re-derive geometry/semantics directly from the emitted XML rather than trusting the generator's own internal state, and are the primary regression gate for any change touching junction/connector/lane code:

```bash
python3 test/check_junction_continuity.py output.xodr [--left-hand-traffic]
python3 test/check_turn_lane_routing.py input.osm output.xodr [--left-hand-traffic]
python3 test/check_road_link_continuity.py output.xodr [--left-hand-traffic]
```

| Script | Checks |
| --- | --- |
| `check_junction_continuity.py` | For every `<junction><connection><laneLink>` whose connecting road is a real synthetic connector (not a direct road-to-road fallback link), checks that position and heading match exactly at both the connector's predecessor and successor ends. |
| `check_turn_lane_routing.py` | For every OSM way carrying `turn:lanes`/`turn:lanes:forward/backward` with an actual per-lane restriction, independently re-derives the same `{lane -> allowed turn bucket}` mapping `infer::decode_turn_lanes` computes, and checks every real `<connection>` that lane routes to lands in a geometrically-compatible bucket (through/left/right/slight_*/sharp_*). |
| `check_road_link_continuity.py` | For every plain (non-junction) road-to-road `<predecessor>`/`<successor>`, checks that each lane's own `<link>` is present, reciprocal on both sides, and positionally/headingwise continuous. |

Exit code is 0 if everything checked matches within 1mm/0.001rad (and, for the road-link check, is reciprocal), 1 otherwise. Pass `--left-hand-traffic` if the `.xodr` was generated with that osm2xodr flag. A reported mismatch is not automatically a converter bug -- check whether the underlying OSM way itself bends at that exact point first (see [Important limitations](#important-limitations) above).

## Upstream references

- libosmium: https://github.com/osmcode/libosmium
- libosmium manual: https://osmcode.org/libosmium/manual.html
- vcpkg libosmium package: https://vcpkg.io/en/package/libosmium.html
- libOpenDRIVE: https://github.com/pageldev/libOpenDRIVE
