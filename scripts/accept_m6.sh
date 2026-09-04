#!/usr/bin/env bash
set -eo pipefail
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -f "${MOONCAR_M6_REPORT:-$HOME/mooncar_m6_acceptance.json}"
"$root/scripts/check_software.sh"
exec "$root/scripts/accept_m5.sh" --terrain "$@"
