#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$root/scripts/check_software.sh"
exec "$root/scripts/accept_m5.sh" --terrain "$@"
