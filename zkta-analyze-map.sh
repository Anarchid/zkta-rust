#!/bin/bash
# zkta-analyze-map.sh — Extract map data via the engine and run terrain analysis.
#
# Usage:
#   ./zkta-analyze-map.sh extract <MapName>                    Extract .zktamap from engine
#   ./zkta-analyze-map.sh render  <MapName> [tiers]            Render heightmap/slope/passability
#   ./zkta-analyze-map.sh analyze <MapName> [tiers] [timeout]  Run terrain analysis
#   ./zkta-analyze-map.sh all     <MapName> [tiers] [timeout]  All of the above
#
# Examples:
#   ./zkta-analyze-map.sh extract "Thornford 2"
#   ./zkta-analyze-map.sh render  "Thornford 2" bot,vehicle
#   ./zkta-analyze-map.sh analyze "Thornford 2" bot,vehicle 10
#   ./zkta-analyze-map.sh all     "Quicksilver 1.1"
#
# Results are stored in results/<MapName>/ with per-tier subdirectories.

set -euo pipefail

CMD="${1:?Usage: $0 <extract|render|analyze|all> <MapName> [tiers] [timeout]}"
MAP="${2:?Usage: $0 $CMD <MapName> [tiers] [timeout]}"
TIERS="${3:-spider,bot,vehicle,hover,ship,amphib}"
TIMEOUT="${4:-10}"

RESULTS_DIR="results/$MAP"
MAPFILE="$RESULTS_DIR/map.zktamap"

# --- Configurable paths ---
ENGINE_DIR="${ENGINE_DIR:-$HOME/.spring/engine/linux64/2025.06.18}"
ENGINE="$ENGINE_DIR/spring-headless"
GAMETYPE="${GAMETYPE:-Zero-K \$VERSION}"
ZKTA_RS="${ZKTA_RS:-$(dirname "$0")/zkta-rs/target/release/zkta-rs}"
TMPDIR_EXTRACT="/tmp/zkta"

# --- Functions ---

do_extract() {
    if [ ! -x "$ENGINE" ]; then
        echo "Engine not found: $ENGINE"
        echo "Set ENGINE_DIR to point to your engine installation."
        exit 1
    fi

    # Generate startscript
    SCRIPT=$(mktemp /tmp/zkta_script_XXXXXX.txt)
    trap 'rm -f "$SCRIPT"' EXIT

    cat > "$SCRIPT" << STARTSCRIPT
[game]
{
mapname=$MAP;
gametype=$GAMETYPE;
ishost=1;
startpostype=0;
numplayers=0;
numusers=1;
numteams=1;
numallyteams=1;
myplayername=ZKTAHost;
[player0]
{
name=ZKTAHost;
team=0;
spectator=1;
}
[ai0]
{
name=ZKTAExtractor;
shortname=ZKTAExtractor;
team=0;
host=0;
[options]
{
output_dir=$TMPDIR_EXTRACT;
}
}
[team0]
{
teamleader=0;
allyteam=0;
}
[allyteam0]
{
numallies=0;
}
[modoptions]
{
}
}
STARTSCRIPT

    echo "=== Extracting map data: $MAP ==="
    mkdir -p "$TMPDIR_EXTRACT"

    # Run engine headless — it will init the AI, which writes the .zktamap, then we kill it
    timeout 30 "$ENGINE" "$SCRIPT" || true

    # Find the .zktamap file
    EXTRACTED=$(find "$TMPDIR_EXTRACT" -name "*.zktamap" -newer "$SCRIPT" -print -quit 2>/dev/null)
    if [ -z "$EXTRACTED" ]; then
        echo "ERROR: No .zktamap file produced in $TMPDIR_EXTRACT"
        echo "Check engine AI log for errors."
        exit 1
    fi

    # Copy to per-map results dir
    mkdir -p "$RESULTS_DIR"
    cp "$EXTRACTED" "$MAPFILE"
    echo "=== Map data extracted: $MAPFILE ==="
}

require_mapfile() {
    if [ ! -f "$MAPFILE" ]; then
        echo "ERROR: $MAPFILE not found."
        echo "Run '$0 extract \"$MAP\"' first."
        exit 1
    fi
}

require_zkta_rs() {
    if [ ! -x "$ZKTA_RS" ]; then
        echo "zkta-rs binary not found at $ZKTA_RS"
        echo "Build it with: cd zkta-rs && cargo build --release"
        exit 1
    fi
}

do_render() {
    require_mapfile
    require_zkta_rs
    echo "=== Rendering map visualizations ==="
    "$ZKTA_RS" render-map "$MAPFILE" --output-dir "$RESULTS_DIR" --tiers "$TIERS"
}

do_analyze() {
    require_mapfile
    require_zkta_rs
    echo "=== Running terrain analysis (tiers: $TIERS, timeout: ${TIMEOUT}s) ==="
    "$ZKTA_RS" analyze "$MAPFILE" --output-dir "$RESULTS_DIR" --tiers "$TIERS" --timeout "$TIMEOUT"
}

# --- Dispatch ---

case "$CMD" in
    extract)
        do_extract
        ;;
    render)
        do_render
        ;;
    analyze)
        do_analyze
        ;;
    all)
        do_extract
        do_render
        do_analyze
        ;;
    *)
        echo "Unknown command: $CMD"
        echo "Usage: $0 <extract|render|analyze|all> <MapName> [tiers] [timeout]"
        exit 1
        ;;
esac

echo "=== Done ==="
