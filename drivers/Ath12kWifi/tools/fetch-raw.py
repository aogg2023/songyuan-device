#!/usr/bin/env python3
"""Phase 1 probe: download the exact Linux v7.0.14 source files the Ath12kWifi
build needs, from raw.githubusercontent.com (kernel.org/GitHub clone is blocked
or too slow from this host). raw is per-file, retryable, and not API-rate-limited.

Downloads into deps/linux/<path> preserving the directory layout that
prepare.sh / pin-headers.sh / fetch-sources.sh expect.
"""
import os, sys, time, urllib.request, urllib.error

TAG = "v7.0.14"
BASE = "https://raw.githubusercontent.com/gregkh/linux/%s/" % TAG
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEST = os.path.join(ROOT, "deps", "linux")

ATH = "drivers/net/wireless/ath/ath12k/"
MHI = "drivers/bus/mhi/host/"

# ath12k top-level .c and .h — complete set for Linux 7.0 (prepare.sh links all *.c *.h)
files = []
for f in """ce.c core.c dbring.c dp.c dp_htt.c dp_mon.c dp_peer.c dp_rx.c dp_tx.c
fw.c hal.c htc.c mac.c mhi.c p2p.c pci.c peer.c qmi.c reg.c wmi.c debug.c
core.h hal.h dp.h peer.h pci.h qmi.h wmi.h htc.h ce.h debug.h dbring.h mhi.h
reg.h fw.h trace.h""".split():
    files.append(ATH + f)

# ath12k/wifi7/*
for f in """ce.c core.c dp.c dp_mon.c dp_rx.c dp_tx.c hal.c hal_qcc2072.c
hal_qcn9274.c hal_rx.c hal_tx.c hal_wcn7850.c hw.c mhi.c pci.c wmi.c
hal.h hal_rx.h hal_tx.h hw.h wmi.h pci.h trace.h""".split():
    files.append(ATH + "wifi7/" + f)

# MHI host
for f in """init.c main.c pm.c boot.c internal.h trace.h""".split():
    files.append(MHI + f)

# QMI / QRTR
files += [
    "drivers/soc/qcom/qmi_interface.c",
    "drivers/soc/qcom/qmi_encdec.c",
    "include/linux/soc/qcom/qmi.h",
    "net/qrtr/qrtr.c",
    "net/qrtr/ns.c",
    "net/qrtr/qrtr.h",
    "include/net/qrtr.h",
]

# public headers used verbatim by pin-headers.sh
for f in """include/linux/mhi.h include/linux/ieee80211.h include/linux/soc/qcom/qmi.h
include/net/cfg80211.h include/net/regulatory.h include/net/codel.h
include/net/ieee80211_radiotap.h include/uapi/linux/nl80211.h
include/uapi/linux/rfkill.h include/net/mac80211.h""".split():
    files.append(f)

# de-dup, keep order
seen = set(); files = [f for f in files if not (f in seen or seen.add(f))]

def fetch(rel, tries=4):
    url = BASE + rel
    dst = os.path.join(DEST, *rel.split("/"))
    if os.path.exists(dst) and os.path.getsize(dst) > 0:
        return "cached", os.path.getsize(dst)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    last = ""
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "phase1-probe/1.0"})
            with urllib.request.urlopen(req, timeout=30) as r:
                data = r.read()
            if len(data) < 20 or data.lstrip()[:1] in (b"<",):
                raise ValueError("not a source file (len=%d)" % len(data))
            with open(dst, "wb") as fh:
                fh.write(data)
            return "ok", len(data)
        except Exception as e:
            last = "%s: %s" % (type(e).__name__, e)
            time.sleep(1 + i)
    return "FAIL", last

ok = fail = cached = 0
failed_list = []
for idx, rel in enumerate(files, 1):
    status, info = fetch(rel)
    if status == "ok": ok += 1
    elif status == "cached": cached += 1
    else:
        fail += 1; failed_list.append((rel, info))
    print("[%02d/%02d] %-9s %-55s %s" % (idx, len(files), status, rel, info), flush=True)

print("\nSUMMARY ok=%d cached=%d fail=%d" % (ok, cached, fail))
for rel, info in failed_list:
    print("  FAILED", rel, info)
sys.exit(1 if fail else 0)
