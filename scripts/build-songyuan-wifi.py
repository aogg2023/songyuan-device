#!/usr/bin/env python3
"""Build libSongyuanWifi.a from this device's Wi-Fi sources.

drivers/SongyuanWifi holds WifiProbe.cpp and WifiNet.cpp. drivers/Ath12kWifi is
the module. Linux v7.0.14 is fetched by that module when deps/linux is absent,
then drivers/SongyuanWifi/wcn7860-0039.patch is applied once. The display
archive is left alone. build-xnu-4k-baseline.sh links this archive through
CONFIG_XNU_EXTRA_LD_ARCHIVES on the non-bootstrap pass.
"""
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/songyuan-wifi'
PROBE = ROOT / 'drivers/SongyuanWifi'
MODULE = ROOT / 'drivers/Ath12kWifi'
RECORD = ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'
CRECORD = ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/osfmk/DEVELOPMENT/startup.o.json'
PATCH = PROBE / 'wcn7860-0039.patch'


def run(cmd, **kw):
    subprocess.check_call([str(x) for x in cmd], **kw)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(ok, why):
    if not ok:
        raise SystemExit(why)


def replay(source, obj, extra):
    record = json.loads(RECORD.read_text().strip().rstrip(','))
    cmd, args, i = [], record['arguments'], 0
    while i < len(args):
        value = args[i]
        if value == record['file']:
            cmd.append(str(source))
        elif value == '-o':
            cmd += ['-o', str(obj)]
            i += 1
        elif value == '-Xclang' and args[i + 1].startswith('-W'):
            i += 1
        elif not value.startswith('-W'):
            cmd.append(value)
        i += 1
    cmd += ['-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
            '-Wno-unnecessary-virtual-specifier', '-Wno-c99-designator',
            '-Wno-reorder-init-list', '-Wno-deprecated-enum-enum-conversion',
            '-I', str(ROOT / '.cache/xnu-work/osfmk'), *extra]
    run(cmd, cwd=record['directory'], timeout=180)


def compile_c(source, obj):
    record = json.loads(CRECORD.read_text().strip().rstrip(','))
    cmd, args, i = [], record['arguments'], 0
    while i < len(args):
        value = args[i]
        if value == record['file']:
            cmd.append(str(source))
        elif value in ('-o', '-MF', '-MT', '-MJ'):
            if value == '-o':
                cmd += ['-o', str(obj)]
            i += 1
        elif value == '-Xclang' and args[i + 1].startswith('-W'):
            i += 1
        elif not value.startswith('-W'):
            cmd.append(value)
        i += 1
    cmd += ['-mgeneral-regs-only', '-Wall', '-Wextra', '-Werror',
            '-I', str(ROOT / '.cache/xnu-work/osfmk'), '-I', str(PROBE)]
    run(cmd, cwd=record['directory'], timeout=180)


def main():
    require(RECORD.is_file() and CRECORD.is_file(), 'XNU compile record missing: ' + str(RECORD))
    require((PROBE / 'WifiProbe.cpp').is_file(), 'drivers/SongyuanWifi/WifiProbe.cpp missing')
    require((MODULE / 'include/nw_abi.h').is_file() and (MODULE / 'tools/fetch-sources.sh').is_file(),
            'drivers/Ath12kWifi missing')
    require(PATCH.is_file(), 'wcn7860-0039.patch missing')
    OUT.mkdir(parents=True, exist_ok=True)
    if not (MODULE / 'deps/linux/.git').is_dir() or not (MODULE / 'deps/freebsd').is_dir():
        run(['bash', MODULE / 'tools/fetch-sources.sh'], cwd=MODULE, timeout=900)
    pci = MODULE / 'deps/linux/drivers/net/wireless/ath/ath12k/wifi7/pci.c'
    require(pci.is_file(), 'ath12k pci.c missing after fetch-sources.sh')
    if 'WCN7860_DEVICE_ID' not in pci.read_text(errors='replace'):
        run(['git', '-C', MODULE / 'deps/linux', 'apply', '--check', PATCH])
        run(['git', '-C', MODULE / 'deps/linux', 'apply', PATCH])
    includes = ['-I', str(PROBE), '-I', str(MODULE / 'include')]
    probe_obj = OUT / 'WifiProbe.o'
    net_obj = OUT / 'WifiNet.o'
    net_late = OUT / 'WifiNet.late.o'
    replay(PROBE / 'WifiProbe.cpp', probe_obj, includes)
    replay(PROBE / 'WifiNet.cpp', net_obj, includes)
    rpmh_obj = OUT / 'QcomRpmh.o'
    mhi_obj = OUT / 'MhiBoot.o'
    compile_c(PROBE / 'QcomRpmh.c', rpmh_obj)
    compile_c(PROBE / 'MhiBoot.c', mhi_obj)
    llvm = Path(subprocess.check_output(['brew', '--prefix', 'llvm'], text=True).strip()) / 'bin'
    run([llvm / 'llvm-objcopy', '--remove-section=__DATA,__mod_init_func',
         '--globalize-symbol=__GLOBAL__sub_I_WifiNet.cpp', net_obj, net_late])
    text = subprocess.check_output(['python3', ROOT / 'scripts/build-ath12k-wifi.py', MODULE, OUT / 'ath12k'],
                                   text=True, timeout=900)
    objects = [Path(line.split(' ', 1)[1]) for line in text.splitlines() if line.startswith('OBJ ')]
    require(objects, 'build-ath12k-wifi.py produced no objects')
    archive = OUT / 'libSongyuanWifi.a'
    archive.unlink(missing_ok=True)
    run(['xcrun', 'ar', 'rcs', archive, probe_obj, net_late, rpmh_obj, mhi_obj, *objects])
    sources = [PROBE / 'WifiProbe.cpp', PROBE / 'WifiNet.cpp', PROBE / 'WifiPcieTables.h',
               PROBE / 'QcomRpmh.h', PROBE / 'QcomRpmh.c', PROBE / 'BootResetCore.h',
               PROBE / 'MhiBoot.h', PROBE / 'MhiBoot.c', PATCH,
               ROOT / 'scripts/build-songyuan-wifi.py', ROOT / 'scripts/build-ath12k-wifi.py']
    (OUT / 'build-report.json').write_text(json.dumps(dict(
        archive_sha256=sha(archive),
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in sources},
        objects=len(objects) + 4), indent=2) + '\n')
    print('songyuan wifi archive', archive, sha(archive))


if __name__ == '__main__':
    main()
