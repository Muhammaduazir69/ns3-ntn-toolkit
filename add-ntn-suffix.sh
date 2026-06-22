#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mapfile -t files < <(grep -rIl 'ns3/traffic-module.h' contrib 2>/dev/null || true)
if ((${#files[@]})); then
  sed -i 's|ns3/traffic-module.h|ns3/ntn-traffic-module.h|g' "${files[@]}"
fi
