#!/bin/bash
# Phase 1 probe helper: fetch Linux v7.0.14 ath12k/MHI/QMI sources from the
# gregkh/linux GitHub mirror (kernel.org is unreachable from this host).
# Retries on transient 502/SSL errors from the flaky network. Idempotent.
set -u
cd "$(dirname "$0")/.." || exit 1
TAG=v7.0.14
REPO=https://github.com/gregkh/linux.git
DEST=deps/linux
OK_MARK="$DEST/drivers/net/wireless/ath/ath12k/wifi7/hw.c"

if [ -f "$OK_MARK" ]; then
  echo "ALREADY_FETCHED: $OK_MARK exists"
  exit 0
fi

echo "=== clean partial clone ==="
rm -rf "$DEST"
mkdir -p deps

# Try up to N times: clone metadata only (blob:none), no checkout yet.
CLONED=0
for i in $(seq 1 8); do
  echo "--- clone attempt $i ---"
  if git clone -q --depth 1 --branch "$TAG" --filter=blob:none --no-checkout \
       --no-tags "$REPO" "$DEST" 2>&1; then
    CLONED=1; echo "CLONE_OK"; break
  fi
  rm -rf "$DEST"
done
[ "$CLONED" = 1 ] || { echo "FAILED: clone did not succeed after retries" >&2; exit 1; }

cd "$DEST" || exit 1
# Write sparse rules WITHOUT leading-slash mangling (Git Bash mangles --no-cone args).
git config core.sparseCheckout true
git config core.sparseCheckoutCone false
cat > .git/info/sparse-checkout <<'EOF'
/drivers/net/wireless/ath/
/drivers/bus/mhi/
/include/
/net/qrtr/
/drivers/soc/qcom/qmi_interface.c
/drivers/soc/qcom/qmi_encdec.c
EOF

# Blob fetch happens lazily via promisor; retry reapply until files land.
OK=0
for i in $(seq 1 12); do
  echo "--- checkout attempt $i ---"
  git sparse-checkout reapply 2>&1 && [ -f "$OK_MARK" ] && { OK=1; echo CHECKOUT_OK; break; }
  # force-fetch the specific tree blobs if reapply alone is short
  git checkout -f HEAD -- drivers/net/wireless/ath/ drivers/bus/mhi/ include/ net/qrtr/ \
      drivers/soc/qcom/qmi_interface.c drivers/soc/qcom/qmi_encdec.c 2>&1 || true
  [ -f "$OK_MARK" ] && { OK=1; echo CHECKOUT_OK2; break; }
done
[ "$OK" = 1 ] || { echo "FAILED: files did not land after retries" >&2; exit 1; }

echo "=== verify ==="
ls -la "$OK_MARK" drivers/net/wireless/ath/ath12k/wifi7/pci.c drivers/net/wireless/ath/ath12k/core.h
du -sh .
echo DONE
