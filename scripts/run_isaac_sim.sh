#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim}"
EXTENSION_TOML="$REPO_ROOT/isaac.sim.mcp_extension/config/extension.toml"
EXTENSION_ID="isaac.sim.mcp_extension"

# shellcheck source=lib/isaac_launcher.sh
source "$REPO_ROOT/scripts/lib/isaac_launcher.sh"

# Resolves ISAACSIM_ENGINE, ISAAC_SIM_SH and ISAAC_PASSTHRU_ARGS.
# Engine: ISAACSIM_ENGINE=newton, or --newton / --engine newton on the CLI.
isaac_resolve_launcher "$ISAACSIM_ROOT" "$@"

if [[ ! -f "$EXTENSION_TOML" ]]; then
  echo "Error: extension manifest not found at: $EXTENSION_TOML" >&2
  echo "Run this script from inside the isaacsim-mcp-server checkout." >&2
  exit 1
fi

echo "Repo root: $REPO_ROOT"
echo "Isaac Sim: $ISAAC_SIM_SH"
echo "Engine:    $ISAACSIM_ENGINE"
echo "Extension: $EXTENSION_ID"

# This bare launcher does not set the socket port: it leaves that to the
# manifest/defaults, and launch_isaac_sim_mcp.sh is what injects
# --/exts/.../server.port. The Windows counterpart (run_isaac_sim.ps1) has no
# launch_isaac_sim_mcp.ps1 to pair with, so it folds a -Port option in itself --
# that divergence is intentional, not a bug.
exec "$ISAAC_SIM_SH" \
  --ext-folder "$REPO_ROOT" \
  --enable "$EXTENSION_ID" \
  "${ISAAC_PASSTHRU_ARGS[@]}"
