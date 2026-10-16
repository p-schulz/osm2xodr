#!/usr/bin/env bash
# Fetches vendored third-party sources into external/ and configures the CMake preset
# matching this platform (macOS or Linux). See setup.bat for the Windows equivalent.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_TYPE="release"
WITH_GUI=0
WITH_VALIDATION=0
DO_CONFIGURE=1
FORCE=0

usage() {
    cat <<'EOF'
Usage: ./setup.sh [options]

Fetches the vendored dependencies under external/ (imgui, libosmium, protozero,
stb, tinyfiledialogs, libOpenDRIVE) and runs `cmake --preset` for this OS.

Options:
  --debug           Configure a Debug preset instead of Release
  --gui             Also configure the GUI (osm2xodr-gui) preset
  --validate        Enable libOpenDRIVE read-back validation (--validate CLI flag)
  --force           Re-fetch dependencies even if external/<name> already exists
  --no-configure    Only fetch dependencies, skip the `cmake --preset` step
  -h, --help        Show this help
EOF
}

for arg in "$@"; do
    case "$arg" in
        --debug) BUILD_TYPE="debug" ;;
        --gui) WITH_GUI=1 ;;
        --validate) WITH_VALIDATION=1 ;;
        --force) FORCE=1 ;;
        --no-configure) DO_CONFIGURE=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $arg" >&2; usage; exit 1 ;;
    esac
done

log() { printf '==> %s\n' "$1"; }
warn() { printf 'warning: %s\n' "$1" >&2; }

for tool in git cmake; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: required tool '$tool' not found in PATH" >&2
        exit 1
    fi
done

case "$(uname -s)" in
    Darwin) OS="macos" ;;
    Linux) OS="linux" ;;
    *)
        echo "error: unsupported platform '$(uname -s)'; use setup.bat on Windows" >&2
        exit 1
        ;;
esac

# name|url|ref|destination (relative to repo root)
DEPS=(
    "imgui|https://github.com/ocornut/imgui.git|v1.92.8|external/imgui"
    "libosmium|https://github.com/osmcode/libosmium.git|v2.23.1|external/libosmium"
    "protozero|https://github.com/mapbox/protozero.git|v1.8.2|external/protozero"
    "stb|https://github.com/nothings/stb.git|master|external/stb"
    "tinyfiledialogs|https://git.code.sf.net/p/tinyfiledialogs/code|master|external/tinyfiledialogs"
    "libOpenDRIVE|https://github.com/pageldev/libOpenDRIVE.git|main|external/libOpenDRIVE"
)

log "Fetching vendored dependencies into external/"
for entry in "${DEPS[@]}"; do
    IFS='|' read -r name url ref dest <<< "$entry"
    if [ -d "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ]; then
        if [ "$FORCE" -eq 1 ]; then
            log "$name: --force given, removing existing $dest"
            rm -rf "$dest"
        else
            log "$name: $dest already present, skipping (use --force to re-fetch)"
            continue
        fi
    fi
    log "$name: cloning $ref from $url"
    mkdir -p "$(dirname "$dest")"
    git clone --quiet --branch "$ref" --depth 1 "$url" "$dest"
done

if [ "$OS" = "macos" ]; then
    if [ "$WITH_GUI" -eq 1 ] && ! command -v sdl2-config >/dev/null 2>&1 && [ ! -d /opt/homebrew/opt/sdl2 ] && [ ! -d /usr/local/opt/sdl2 ]; then
        warn "SDL2 not found; install it with: brew install sdl2 curl"
    fi
else
    if command -v pkg-config >/dev/null 2>&1; then
        for lib in zlib bzip2 expat; do
            pkg-config --exists "$lib" 2>/dev/null || warn "$lib not found via pkg-config; install it (e.g. apt-get install zlib1g-dev libbz2-dev libexpat1-dev)"
        done
        if [ "$WITH_GUI" -eq 1 ]; then
            pkg-config --exists sdl2 2>/dev/null || warn "sdl2 not found via pkg-config; install it (e.g. apt-get install libsdl2-dev)"
            pkg-config --exists libcurl 2>/dev/null || warn "libcurl not found via pkg-config; install it (e.g. apt-get install libcurl4-openssl-dev)"
        fi
    fi
fi

PRESET="${OS}-${BUILD_TYPE}"
[ "$WITH_GUI" -eq 1 ] && PRESET="${PRESET}-gui"

if [ "$DO_CONFIGURE" -eq 0 ]; then
    log "Dependencies fetched. Skipping configure (--no-configure given)."
    log "Configure later with: cmake --preset $PRESET"
    exit 0
fi

CMAKE_ARGS=()
if [ "$WITH_VALIDATION" -eq 1 ]; then
    CMAKE_ARGS+=(-DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON -DOSM2XODR_LIBOPENDRIVE_SOURCE_DIR="$SCRIPT_DIR/external/libOpenDRIVE")
fi

log "Configuring preset '$PRESET'"
cmake --preset "$PRESET" "${CMAKE_ARGS[@]}"

log "Done. Build with: cmake --build build/$PRESET -j"
