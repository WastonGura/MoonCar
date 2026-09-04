#!/usr/bin/env bash
set -eo pipefail
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$root/scripts/check_software.sh"
exec "$root/scripts/accept_m5.sh" --terrain "$@"
