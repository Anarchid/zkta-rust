#!/bin/bash
# zkta-analyze-map.sh — Extract map data via the engine and run terrain analysis.
#
# Usage:
#   ./zkta-analyze-map.sh <MapName> [tier1,tier2,...]
#
# Examples:
#   ./zkta-analyze-map.sh "TitanDuel 2.2"
#   ./zkta-analyze-map.sh "Red Comet" bot,vehicle
#   ./zkta-analyze-map.sh "Quicksilver 1.1" spider,bot,vehicle,hover,ship,amphib

set -euo pipefail

MAP="${1:?Usage: $0 <MapName> [tiers]}"
TIERS="${2:-spider,bot,vehicle,hover,ship,amphib}"
OUTDIR="/tmp/zkta"

# --- Configurable paths ---
ENGINE_DIR="${ENGINE_DIR:-$HOME/.spring/engine/linux64/2025.06.18}"
ENGINE="$ENGINE_DIR/spring-headless"
GAMETYPE="${GAMETYPE:-Zero-K \$VERSION}"
ZKTA_RS="${ZKTA_RS:-$(dirname "$0")/zkta-rs/target/release/zkta-rs}"

if [ ! -x "$ENGINE" ]; then
    echo "Engine not found: $ENGINE"
    echo "Set ENGINE_DIR to point to your engine installation."
    exit 1
fi

# --- Generate startscript ---
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
output_dir=$OUTDIR;
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
mkdir -p "$OUTDIR"

# Run engine headless — it will init the AI, which writes the .zktamap, then we kill it
# The engine may hang after init, so timeout after 30s
timeout 30 "$ENGINE" "$SCRIPT" || true

# Find the .zktamap file
MAPFILE=$(find "$OUTDIR" -name "*.zktamap" -newer "$SCRIPT" -print -quit 2>/dev/null)
if [ -z "$MAPFILE" ]; then
    echo "ERROR: No .zktamap file produced in $OUTDIR"
    echo "Check engine AI log for errors."
    exit 1
fi

echo "=== Map data extracted: $MAPFILE ==="
echo "=== Running terrain analysis (tiers: $TIERS) ==="

# Run zkta-rs in Spring mode
if [ -x "$ZKTA_RS" ]; then
    "$ZKTA_RS" --spring "$MAPFILE" --tiers "$TIERS"
else
    echo "zkta-rs binary not found at $ZKTA_RS"
    echo "Build it with: cd zkta-rs && cargo build --release"
    echo "Or run manually: zkta-rs --spring $MAPFILE --tiers $TIERS"
    exit 1
fi

echo "=== Done ==="
