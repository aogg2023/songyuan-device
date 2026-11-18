#!/bin/bash
# Phase 1 probe: apply the WCN7860 (PCI 17cb:110e) hardware definition to the
# fetched Linux v7.0.14 ath12k tree, then verify each hunk landed.
#
# NOTE: deps/linux here is populated via tools/fetch-raw.py (per-file raw
# download) and has NO .git of its own, so git apply operates on the wrong
# repository. Use plain `patch -p1` from the deps/linux root instead.
# Idempotent: re-running is safe.
set -u
cd "$(dirname "$0")/.." || exit 1
PATCH="$(pwd)/patches/wcn7860/0039-wifi-ath12k-add-initial-hardware-definition-for-WCN7.patch"
LINUX="$(pwd)/deps/linux"
ATH="$LINUX/drivers/net/wireless/ath/ath12k"

if [ ! -f "$ATH/wifi7/hw.c" ]; then
  echo "ERROR: ath12k tree missing; run tools/fetch-raw.py first" >&2
  exit 1
fi
if grep -q 'WCN7860_DEVICE_ID' "$ATH/wifi7/pci.c" 2>/dev/null; then
  echo "ALREADY_APPLIED: WCN7860 definition present"
  exit 0
fi

echo "=== dry-run patch -p1 ==="
( cd "$LINUX" && patch -p1 --dry-run --fuzz=3 < "$PATCH" ) || {
  echo "FAILED: dry-run; needs manual 0039 backport" >&2; exit 1; }
echo "=== apply patch -p1 ==="
( cd "$LINUX" && patch -p1 --forward --fuzz=3 < "$PATCH" ) || {
  echo "FAILED: apply" >&2; exit 1; }

echo "=== verify WCN7860 landed ==="
grep -n 'WCN7860_DEVICE_ID\|0x110e' "$ATH/wifi7/pci.c"
grep -n 'wcn7860 hw2.0\|ATH12K_HW_WCN7860_HW20' "$ATH/wifi7/hw.c"
grep -n 'ATH12K_HW_WCN7860_HW20' "$ATH/core.h" "$ATH/wifi7/hal.c"
echo DONE
