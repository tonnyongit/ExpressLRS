#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

git apply --check srxl2-smart-esc-integration.patch
git apply srxl2-smart-esc-integration.patch

echo "SRXL2 SMART ESC integration patch applied."
