// SPDX-License-Identifier: BSD-2-Clause-Patent
// WiFi bring-up. Stage 1 is a read-only look at what the boot loaders left. Stage 2 powers the link:
// WLAN_EN, the PCIe0 and PHY GDSCs, clocks, block resets, the QMP PHY tables (WifiPcieTables.h),
// PARF, PERST# release, LTSSM, and a configuration read of the endpoint's ID.
// PHY writes are the 199-word songyuan sequence. Rail names are the songyuan seven.
// Songyuan facts that agree between work/WIFI.md §1.2–§1.3 and 0350-pandora-varcache.patch:
// PCIe0 parf 0x1c00000, phy 0x1c06000, DBI 0x400000000, MEM window 0x40300000/0x3d00000,
// WLAN_EN GPIO 16, PERST GPIO 102, CLKREQ GPIO 103. SW_CTRL is GPIO 18 and is only sampled.
// GCC votes, halt registers and RCG encodings below follow gcc-canoe.c and sy_clocks_on.
// DMA window follows sy_mem_setup: IOVA 0x18000000, 32 MiB, PTE 0x0060000000000e47. The 8 MiB MHI window at 0xAF800000 is a different mapping.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include "WifiPcieTables.h"
#include <string.h>
#include "QcomRpmh.h"
#include "BootResetCore.h"
#include "MhiBoot.h"
#include "nw_abi.h"
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <libkern/c++/OSData.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <stdarg.h>
#include <kern/task.h>
#include <stdint.h>
extern "C" {
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <mach/vm_param.h>
#include <arm/pmap.h>
void flush_dcache64(addr64_t addr, unsigned count, int phys);
}

// [0] stage (1 mapped, 2 clocks read, 3 controller read; negative: the step that failed), [1] vote 0x52008, [2] vote 0x52000,
// [3] GDSC, [4] halt bits (bit i set: branch i of kBranches is off), [5] GPIO 16 | 18 << 8 | 102 << 16 (bit 0 input level, bit 1 output),
// [6] PARF LTSSM, [7] root port vendor/device, [8] link control/status, [9] PARF PHY control.
// Stage 2 steps in [0]: 10 WLAN_EN high, 11 GDSCs on, 12 clocks on, 13 PARF set up, 14 PHY ready and root port set up, 15 PERST# released, 16 link up,
// 17 endpoint ID read. [10] GPIO word after WLAN_EN, [11] GDSC bits (bit 0 PCIe0, bit 1 PHY) | halt bits << 8, [12] PCS_STATUS1 |
// PHY poll count << 16, [13] port debug 0 (LTSSM state in bits 0-5), [14] port debug 1, [15] endpoint vendor/device, [16] link ms
extern "C" int native_ufs_part_find(const char *, uint32_t *, uint64_t *, uint64_t *) __attribute__((weak_import));
extern "C" int native_ufs_part_read(uint32_t, uint64_t, uint32_t, void *) __attribute__((weak_import));
extern "C" {volatile uint32_t native_wifi_stats[29]; volatile uint32_t native_wifi_dump[6 * 18];
// PMIC state read through the SPMI arbiter, which does not depend on RPMh answering. [0] arbiter version, [1] APIDs, [2] regulators found before | after << 16,
// [3] PMK8550 gpio3 (the sleep clock pin) APID | owner << 16 (0xffff: none), [4..6] its registers 04 05 40 41 | 42 43 44 45 | 46 4a before, [7..9] after,
// [10] result of the sleep clock write (0 written, 0xff not needed, other: the step that failed), [11..16] command database addresses of the six rails
// (0 when not found), [17] pipe clock branch halt register after the PHY is ready, [18] PERST# pin config | io << 16 after its release, [19] port debug 0 20 ms after
// the LTSSM enable, [20] pipe source mux register at the end, [21] port debug 1 at 20 ms. Regulator tables follow: 112 entries of two words, first word PPID << 16 | type << 8 | subtype, second
// 0x08 << 24 | 0x46 << 16 | 0x40 << 8 | 0x41; before at [24], after at [248].
volatile uint32_t native_wifi_pmic[24 + 2 * 2 * 112];
// The endpoint after the link is up (stage 4): [0] stage reached, [1] BAR0 sizing mask, [2] BAR0 as assigned, [3] command register read back, [4] root port memory
// window register, [5..11] MHI MHIREGLEN MHIVER MHICFG BHIOFF BHIEOFF MHICTRL MHISTATUS, [12] BHI version major | minor << 16, [13] BHI execution environment,
// [14] BHI status, [15] BHI error code.
volatile uint32_t native_wifi_ep[16];
volatile uint32_t native_wifi_pon[12];
volatile uint32_t native_wifi_cfg[32];
volatile uint32_t native_wifi_chip[16];
volatile uint32_t native_wifi_ev[48];
volatile uint32_t native_wifi_ldo[24];
// The firmware download (stage 5): [0] step (1 firmware container found, 2 amss.bin read, 3 DMA buffers, 4 SMMU stream mapped, 5 MHI opened and SBL started,
// 6 contexts programmed, 7 mission mode), negative: the step that failed; [1] SMMU slot | bank << 16 (or the planner's error index); [2] partition LUN | first block << 8;
// [3] amss.bin size; [4] segments; [5] MhiBoot phase | error << 8 | failed phase << 16; [6] execution environment; [7] MHI state; [8] BHI status; [9] BHI error code;
// [10..12] BHI debug words; [13] MHI status register; [14] ms spent; [15] BHIe status.
volatile uint32_t native_wifi_mhi[16];
// Milliseconds since the download started when: [0] the buffers were ready, [1] the container was found, [2] the image was read, [3] the SMMU stream was mapped,
// [4] the boot loader was started, [5] the contexts were programmed, [6] the run ended.
volatile uint32_t native_wifi_mhi_ms[8];
// The WiFi module's messages: a ring of 512 lines, newest at native_wifi_text_n - 1.
char native_wifi_text[512][112]; volatile uint32_t native_wifi_text_n;
// The same messages of level 6 and below (notices, warnings, errors) in a ring of their own, which the driver's debug lines do not push out.
char native_wifi_note[128][112]; volatile uint32_t native_wifi_note_n;}

extern kmod_info_t g_kernel_kmod_info;
extern "C" void native_wifi_net_static_init(void) __asm__("__GLOBAL__sub_I_WifiNet.cpp");
extern "C" void sheng_wifi_net_start(const uint8_t mac[6]);
namespace {
constexpr uint64_t kGcc = 0x100000, kParf = 0x1c00000, kDbi = 0x400000000ull, kPhy = 0x1c06000, kTcsrClkref = 0x1fd5000;
// Two TLMM windows, same split as sy_pin_reg. A single map at 0xf100000 of 0x60000 ends at 0xf160000;
// pin 102 lives at 0xf166000, 0x6000 bytes past that end.
constexpr uint64_t kTlmmLow = 0x0f110000ull, kTlmmHi = 0x0f166000ull;
// Patch SY_IATU_PA / SY_CONF_PA. sy_parf_setup writes this same iATU address to PARF 0x634.
// WIFI.md §1.2 and §10.3 also record 0x410000000. The current patch does not program that value.
constexpr uint64_t kAtu = 0x410100000ull, kConfig = 0x400100000ull;
constexpr uint32_t kVote8 = 0x52008, kVote0 = 0x52000, kVote18 = 0x52018, kGdsc = 0x6b004, kPhyGdsc = 0x6c000, kCollapse = 0x5214c;
// Halt registers polled by sy_clocks_on. Index 4 is the pipe branch, which stays off until the PHY sequence.
constexpr uint32_t kBranches[] = {0x6b020, 0x6b030, 0x6b040, 0x6b044, 0x6b064, 0x6b01c, 0x10058};
constexpr uint32_t kNeeded = 0xf;   // slv_axi, mstr_axi, cfg_ahb, aux
constexpr unsigned kGpios[] = {16, 18, 102};
constexpr uint32_t kParfLtssm = 0x1b0, kParfPhyCtrl = 0x40, kLinkControl = 0x80;
constexpr unsigned kWlanEn = 16, kPerst = 102, kClkReq = 103;
// gcc-canoe.c: pipe mux parent 0 is the PHY pipe and parent 2 is TCXO. Aux RCG is TCXO.
// rchng CFG 0x602 is parent-map value 6 (GPLL0 even) and pre_div 2, which divides by 3.
constexpr uint32_t kPcieBcr = 0x6b000, kPhyBcr = 0x6c01c, kPhyNocsrBcr = 0x6c028, kPipeMux = 0x6b090, kAuxRcg = 0x6b094, kRchngRcg = 0x6b078;
constexpr uint32_t kVote8Bits = 0x6f, kVote0Bits = 1u << 12 | 1u << 19 | 1u << 20 | 1u << 22, kVote18Bits = 1u << 31;
constexpr uint32_t kTcsrClkrefEn = 0x44;
// pcie-qcom.c
constexpr uint32_t kParfSysCtrl = 0x0, kParfPmCtrl = 0x20, kParfDbiBase = 0x350, kParfDbiBaseHi = 0x354, kParfMhiClockReset = 0x174;
constexpr uint32_t kParfAxiHalt = 0x1a8, kParfAtuBase = 0x634, kParfAtuBaseHi = 0x638, kParfDeviceType = 0x1000;
constexpr uint32_t kParfSlvSpace = 0x358, kParfSlvSpaceHi = 0x35c;
// pcie-designware.h
constexpr uint32_t kPortDebug0 = 0x728, kPortDebug1 = 0x72c, kBusNumbers = 0x18, kCommand = 0x4;

struct Window {IOMemoryMap *map; volatile uint32_t *va;};
bool mapWindow(Window &w, uint64_t base, uint64_t size, IOOptionBits cache = kIOMapInhibitCache) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, size);
    w.map = memory ? memory->map(kIOMapAnywhere | cache) : nullptr;
    if (memory) memory->release();
    w.va = w.map ? reinterpret_cast<volatile uint32_t *>(w.map->getVirtualAddress()) : nullptr;
    return w.va != nullptr;
}
void release(Window &w) {if (w.map) w.map->release(); w = {};}

// Pins 16..19 sit in the low window; pins 102..104 sit in the high window (patch sy_pin_reg).
struct Pins {volatile uint32_t *low, *hi;};
volatile uint32_t *pinReg(Pins const &pins, unsigned pin, unsigned add) {
    if (pin >= 16 && pin <= 19) return pins.low + ((pin - 16) * 0x1000u + add) / 4;
    return pins.hi + ((pin - 102) * 0x1000u + add) / 4;
}
// TLMM pin config: pull in bits 0-1 (1 down, 3 up), function in 2-5, drive (mA / 2 - 1) in 6-8, output enable bit 9; io bit 1 drives.
// sy_pin_ctl in 0350-pandora-varcache.patch keeps every bit outside 0x3ff. Bit 12 follows bit 11 of the value already in the register.
void setPin(Pins const &pins, unsigned pin, uint32_t config, int level) {
    volatile uint32_t *ctl = pinReg(pins, pin, 0);
    if (level >= 0) *pinReg(pins, pin, 4) = level ? 2u : 0u;
    uint32_t old = *ctl;
    uint32_t value = (old & ~(0x3ffu | (1u << 12))) | (config & 0x3ffu);
    if (old & (1u << 11)) value |= 1u << 12;
    *ctl = value;
    if (level >= 0) *pinReg(pins, pin, 4) = level ? 2u : 0u;
}

uint32_t pinWord(Pins const &pins) {
    uint32_t gpios = 0;
    for (unsigned i = 0; i < 3; ++i) gpios |= (*pinReg(pins, kGpios[i], 4) & 3u) << (8 * i);
    return gpios;
}
uint32_t pinSnap(Pins const &pins, unsigned pin) {
    return (*pinReg(pins, pin, 0) & 0xffffu) | (*pinReg(pins, pin, 4) << 16);
}

bool waitClear(volatile uint32_t *reg, uint32_t mask, unsigned tries, unsigned *used = nullptr) {
    unsigned i = 0;
    for (; i < tries && (*reg & mask); ++i) IODelay(100);
    if (used) *used = i;
    return !(*reg & mask);
}

void pulseReset(volatile uint32_t *gcc, uint32_t bcr) {
    gcc[bcr / 4] |= 1u;
    IODelay(1000);
    gcc[bcr / 4] &= ~1u;
    IODelay(1000);
}

// sy_rcg_set: root enable, wait until the root is on, write CFG, pulse CMD bit 0, drop root enable.
bool rcgSet(volatile uint32_t *gcc, uint32_t rcg, uint32_t cfg) {
    gcc[rcg / 4] |= 2u;
    if (!waitClear(&gcc[rcg / 4], 1u << 31, 100)) return false;
    gcc[(rcg + 4) / 4] = cfg;
    gcc[rcg / 4] |= 1u;
    if (!waitClear(&gcc[rcg / 4], 1u, 100)) return false;
    gcc[rcg / 4] &= ~2u;
    return true;
}

// ---- PMIC state through the SPMI arbiter (spmi@c400000, EE 0; the same layout the button driver uses) ----
// Regulator peripherals: type 0x04 LDO, 0x03 BUCK, 0x1c FTS, 0x21/0x22 ULT; status 0x08 (bit 7 in regulation), enable 0x46 (bit 7), the buck
// voltage in mV at 0x40/0x41. The sleep clock: mainline's sheng description has the WCN7850 PMU pinctrl select PMK8550 gpio3, function func1,
// output enabled, no bias, power source 0; the gpio block of PMK8550 starts at 0xb800 with 0x100 per pin, so gpio3 is peripheral 0xba of SID 0.
// A low voltage/medium voltage pin takes mode 1 (output) at 0x40, the function index (func1 = 2) at 0x44 and the master enable at 0x46.
constexpr uint64_t kSpmiCore = 0xc400000, kSpmiObs = 0xc440000, kSpmiChn = 0xc500000, kSpmiCnfg = 0xc42d000;
constexpr uint32_t kSleepClkPpid = 0x0ba, kMaxRegs = 112;
// The PMIC regulator peripherals the six rails live in, by the low byte of the PPID: SMPS n at 0x9b00 + 0x300 * (n - 1) pages (smps1 0x9b ... smps6 0xaa, the
// pattern seen in the first scan: smps4 0xa4, smps6 0xaa), LDO15 at 0x4e. Every SID 0 to 15 is looked at for these.
constexpr uint8_t kRegPages[] = {0x9b, 0x9e, 0xa1, 0xa4, 0xa7, 0xaa, 0x4e};

struct Pmic {Window core, cnfg, obs, chn; br_arbiter arb; uint32_t apids;};
Pmic gPmic;

bool pmicOpen(Pmic &p) {
    p.arb.ee = 0;
    if (!mapWindow(p.core, kSpmiCore, 0x3000) || !mapWindow(p.cnfg, kSpmiCnfg, 0x4000)) return false;
    p.arb.core = p.core.va; p.arb.cnfg = p.cnfg.va;
    native_wifi_pmic[0] = p.core.va[0];
    p.apids = p.core.va[BR_FEATURES / 4] & BR_FEATURES_APIDS_MASK;
    native_wifi_pmic[1] = p.apids;
    if (p.core.va[0] < BR_VERSION_V7_MIN || p.core.va[0] >= BR_VERSION_V8_MIN || !p.apids || p.apids > 1024) return false;
    return mapWindow(p.obs, kSpmiObs, ((uint64_t)BR_OBS_APID_STRIDE * p.apids + 0xfff) & ~0xfffull) &&
           mapWindow(p.chn, kSpmiChn, (uint64_t)BR_CHNL_APID_STRIDE * p.apids);
}

int pmicApid(Pmic &p, uint32_t ppid, int *owner) {
    int found = -1;
    for (uint32_t i = 0; i < p.apids; ++i) {
        const uint32_t map = p.core.va[(BR_APID_MAP + 4 * i) / 4];
        if (!map || ((map >> 8) & 0xfff) != ppid) continue;
        const int o = (int)(p.cnfg.va[(BR_CNFG_OWNER + 4 * i) / 4] & 7u);
        if (found < 0 || o == p.arb.ee) {found = (int)i; *owner = o;}
    }
    return found;
}

// The candidate regulator peripherals (kRegPages in every SID) that exist; entries go to out (two words each). Returns how many.
unsigned pmicScan(Pmic &p, volatile uint32_t *out) {
    unsigned n = 0;
    for (uint32_t sid = 0; sid < 16 && n < kMaxRegs; ++sid)
        for (unsigned k = 0; k < sizeof kRegPages && n < kMaxRegs; ++k) {
            const uint32_t ppid = sid << 8 | kRegPages[k];
            int owner = -1;
            const int apid = pmicApid(p, ppid, &owner);
            if (apid < 0) continue;
            uint8_t type = 0, sub = 0, st = 0, en = 0, lo = 0, hi = 0;
            if (br_read8(p.obs.va, &p.arb, apid, 0x04, &type) || br_read8(p.obs.va, &p.arb, apid, 0x05, &sub)) continue;
            if (type != 0x03 && type != 0x04 && type != 0x1c && type != 0x21 && type != 0x22) continue;
            if (br_read8(p.obs.va, &p.arb, apid, 0x08, &st) || br_read8(p.obs.va, &p.arb, apid, 0x46, &en) ||
                br_read8(p.obs.va, &p.arb, apid, 0x40, &lo) || br_read8(p.obs.va, &p.arb, apid, 0x41, &hi)) continue;
            out[2 * n] = ppid << 16 | (uint32_t)type << 8 | sub;
            out[2 * n + 1] = (uint32_t)st << 24 | (uint32_t)en << 16 | (uint32_t)lo << 8 | hi;
            ++n;
        }
    return n;
}

// The sleep clock pin's registers 04 05 40 41 | 42 43 44 45 | 46 4a, packed into three words at out.
void pmicSleepClk(Pmic &p, int apid, volatile uint32_t *out) {
    static const uint8_t regs[] = {0x04, 0x05, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x4a};
    uint32_t w[3] = {0, 0, 0};
    for (unsigned i = 0; i < sizeof regs; ++i) {
        uint8_t v = 0xee;
        br_read8(p.obs.va, &p.arb, apid, regs[i], &v);
        w[i / 4] |= (uint32_t)v << (8 * (i % 4));
    }
    out[0] = w[0]; out[1] = w[1]; out[2] = w[2];
}

void pmicSurvey(bool after) {
    Pmic &p = gPmic;
    const unsigned n = pmicScan(p, native_wifi_pmic + (after ? 24 + 2 * kMaxRegs : 24));
    native_wifi_pmic[2] = after ? (native_wifi_pmic[2] & 0xffff) | n << 16 : n;
}

// The boot's power-on peripheral (type 0x09 in SID 0 or 1): ppid, then bytes 0x08 to 0x27 (the reason registers of the previous power-off and warm reset).
void pmicPon(Pmic &p) {
    for (uint32_t sid = 0; sid < 2; ++sid)
        for (uint32_t page = 0; page < 0x100; ++page) {
            int owner = -1;
            const int apid = pmicApid(p, sid << 8 | page, &owner);
            uint8_t type = 0;
            if (apid < 0 || br_read8(p.obs.va, &p.arb, apid, 0x04, &type) || type != 0x09) continue;
            native_wifi_pon[0] = (sid << 8 | page) << 8 | (uint32_t)owner;
            for (unsigned i = 0; i < 32; ++i) {
                uint8_t v = 0xee;
                br_read8(p.obs.va, &p.arb, apid, 0x08 + i, &v);
                native_wifi_pon[1 + i / 4] |= (uint32_t)v << (8 * (i % 4));
            }
            return;
        }
}

// Every peripheral (SIDs 0 to 7) whose voltage setpoint reads 1.8 V (1792 to 1808 mV): the VDDIO rail's peripheral address is not known. Up to six per call,
// before (out[0..11]) and after (out[12..23]) the rail votes; two words each as in pmicScan.
void pmicFind1800(Pmic &p, volatile uint32_t *out) {
    unsigned n = 0;
    for (uint32_t sid = 0; sid < 8 && n < 6; ++sid)
        for (uint32_t page = 0x40; page < 0x100 && n < 6; ++page) {
            int owner = -1;
            const int apid = pmicApid(p, sid << 8 | page, &owner);
            uint8_t type = 0, sub = 0, st = 0, en = 0, lo = 0, hi = 0;
            if (apid < 0 || br_read8(p.obs.va, &p.arb, apid, 0x04, &type) || type == 0x09 || type == 0x2c) continue;
            if (br_read8(p.obs.va, &p.arb, apid, 0x40, &lo) || br_read8(p.obs.va, &p.arb, apid, 0x41, &hi)) continue;
            const uint32_t v = (uint32_t)hi << 8 | lo;
            if (v < 1792 || v > 1808) continue;
            br_read8(p.obs.va, &p.arb, apid, 0x05, &sub);
            br_read8(p.obs.va, &p.arb, apid, 0x08, &st);
            br_read8(p.obs.va, &p.arb, apid, 0x46, &en);
            out[2 * n] = (sid << 8 | page) << 16 | (uint32_t)type << 8 | sub;
            out[2 * n + 1] = (uint32_t)st << 24 | (uint32_t)en << 16 | v;
            ++n;
        }
}

// Before the PMU is powered: look at the sleep clock pin and switch it on when it is not already an enabled output of function 2.
void pmicPrepare() {
    Pmic &p = gPmic;
    if (!pmicOpen(p)) {native_wifi_pmic[10] = 1; return;}
    pmicSurvey(false);
    pmicPon(p);
    pmicFind1800(p, native_wifi_ldo);
    int owner = -1;
    const int apid = pmicApid(p, kSleepClkPpid, &owner);
    native_wifi_pmic[3] = apid < 0 ? 0xffff : (uint32_t)apid | (uint32_t)owner << 16;
    if (apid < 0) {native_wifi_pmic[10] = 2; return;}
    pmicSleepClk(p, apid, native_wifi_pmic + 4);
    const uint32_t mode = native_wifi_pmic[4] >> 16 & 0xff, source = native_wifi_pmic[5] >> 16 & 0xff, enable = native_wifi_pmic[6] & 0xff;
    if ((enable & 0x80) && (mode & 3) == 1 && (source & 0xf) == 2) {native_wifi_pmic[10] = 0xff;}
    else if (owner != 0) native_wifi_pmic[10] = 3;
    else if ((native_wifi_pmic[4] >> 8 & 0xff) < 0x10 || (native_wifi_pmic[4] >> 8 & 0xff) > 0x13) native_wifi_pmic[10] = 4;   // not a LV/MV pin: other register layout
    else {
        // pull disabled, power source 0, source function, output mode, master enable (the order the pinctrl driver applies them)
        int rc = br_write8(p.chn.va, apid, 0x42, 5);
        if (!rc) rc = br_write8(p.chn.va, apid, 0x41, 0);
        if (!rc) rc = br_write8(p.chn.va, apid, 0x44, 2);
        if (!rc) rc = br_write8(p.chn.va, apid, 0x40, 1);
        if (!rc) rc = br_write8(p.chn.va, apid, 0x46, 0x80);
        native_wifi_pmic[10] = rc ? 0x10u + (uint32_t)rc : 0;
    }
    pmicSleepClk(p, apid, native_wifi_pmic + 7);
}

// sy_supplies on apps RSC DRV-2. Voltage at +0, enable at +4. An LDO also votes mode 7 at +8.
// TCS 0 and TCS 1 on this DRV are touch and GPU. WLAN uses TCS 2.
struct Rail {const char *name; uint32_t millivolts; uint32_t minUv, maxUv; bool ldo;};
constexpr Rail kRails[] = {
    {"S1J_E1", 852, 400000, 1000000, false}, {"S2J_E1", 852, 500000, 1100000, false},
    {"L3G_E0", 1200, 1200000, 1200000, true}, {"L2G_E0", 1800, 1800000, 1800000, true},
    {"S8F_E0", 1856, 1800000, 2000000, false}, {"S7F_E0", 1256, 1224000, 1372000, false},
    {"L3K_E1", 1200, 1200000, 1200000, true},
};
constexpr uint64_t kRscDrv = 0x18920000, kRscTcs = 0xd00, kCmdDb = 0x81c60000, kCmdDbSize = 0x20000;

// [20] bit i: rail i found in the command database and voted | 1 << (8 + i) request done; [21] RSC id | TCS << 24 (negative stage on failure)
int powerRails() {
    static uint8_t db[kCmdDbSize];
    Window dbw = {}, rsc = {};
    if (!mapWindow(dbw, kCmdDb, kCmdDbSize)) return -1;
    for (uint64_t i = 0; i < kCmdDbSize; i += 4) {const uint32_t w = dbw.va[i / 4]; memcpy(db + i, &w, 4);}
    release(dbw);
    if (!mapWindow(rsc, kRscDrv, 0x10000)) return -2;
    volatile uint32_t *r = rsc.va;
    const QcomRscLayout layout = qcom_rsc_layout(r[0]);
    const unsigned tcs = 2;
    native_wifi_stats[21] = r[0] | tcs << 24;
    native_wifi_stats[26] = r[0x0c / 4];
    native_wifi_stats[28] = r[0];
    uint32_t result = 0;
    const uint64_t t0 = mach_absolute_time();
    auto keep = [&](uint32_t off, uint32_t v) {
        r[off / 4] = v;
        for (unsigned n = 0; n < 1000; ++n) {if (r[off / 4] == v) return true; IODelay(1);}
        return false;
    };
    // sy_rpmh_lookup: exactly one slave-4 hit. VRM also requires (addr >> 16) & 7 == 4. The clock lookup does not.
    auto findRes = [&](const char *name, uint32_t *address, bool clock) {
        char id[8] = {};
        for (unsigned i = 0; i < 8 && name[i]; ++i) id[i] = name[i];
        unsigned matches = 0;
        uint32_t found = 0;
        const uint32_t magic = db[4] | db[5] << 8 | db[6] << 16 | db[7] << 24;
        if (magic != QCOM_CMD_DB_MAGIC) return false;
        for (unsigned s = 0; s < 8; ++s) {
            const uint8_t *slave = db + 8 + 16 * s;
            const unsigned sid = slave[0] | slave[1] << 8;
            if (!sid) break;
            const size_t heads = 144u + (slave[2] | slave[3] << 8);
            const unsigned count = slave[6] | slave[7] << 8;
            if (heads + (size_t)count * 24 > sizeof db) return false;
            for (unsigned e = 0; e < count; ++e) {
                const uint8_t *entry = db + heads + 24 * e;
                bool same = true;
                for (unsigned i = 0; i < 8; ++i) if (entry[i] != (uint8_t)id[i]) same = false;
                if (!same) continue;
                const uint32_t addr = entry[16] | entry[17] << 8 | entry[18] << 16 | entry[19] << 24;
                const bool hit = sid == 4 && !(addr & 15u) && (clock || ((addr >> 16) & 7u) == 4);
                if (hit) {found = addr; ++matches;}
            }
        }
        if (matches != 1) return false;
        *address = found;
        return true;
    };
    auto vote = [&](uint32_t address, const uint32_t *vals, unsigned ncmd) {
        const uint32_t base = kRscTcs + layout.tcs_stride * tcs;
        const uint32_t bit = 1u << tcs;
        if (r[(base + layout.control) / 4] || r[(base + layout.cmd_enable) / 4] || r[(base + layout.wait_for_cmpl) / 4]) return false;
        bool ok = keep(base + layout.control, 0);
        r[(kRscTcs + layout.irq_clear) / 4] = bit;
        static const uint32_t offs[3] = {0, 4, 8};
        for (unsigned c = 0; c < ncmd && ok; ++c) {
            const uint32_t cmd = base + c * layout.cmd_stride;
            ok = keep(cmd + layout.cmd_msgid, 8u | 1u << 8 | 1u << 16)
                && keep(cmd + layout.cmd_addr, address + offs[c])
                && keep(cmd + layout.cmd_data, vals[c]);
        }
        const uint32_t mask = (1u << ncmd) - 1u;
        if (ok) ok = keep(base + layout.wait_for_cmpl, mask) && keep(base + layout.cmd_enable, mask);
        bool done = false;
        if (ok) {
            r[kRscTcs / 4] |= bit;
            ok = keep(base + layout.control, 1u << 16);
            if (ok) r[(base + layout.control) / 4] = (1u << 16) | (1u << 24);
            for (unsigned us = 0; us < 10000 && !done; ++us) {
                done = (r[(kRscTcs + layout.irq_status) / 4] & bit) && r[(base + layout.status) / 4];
                if (!done) IODelay(1);
            }
        }
        keep(base + layout.control, 1u << 16);
        keep(base + layout.control, 0);
        keep(base + layout.cmd_enable, 0);
        keep(base + layout.wait_for_cmpl, 0);
        r[(kRscTcs + layout.irq_clear) / 4] = bit;
        r[kRscTcs / 4] &= ~bit;
        native_wifi_stats[27] = r[kRscTcs / 4];
        return ok && done;
    };
    int failed = 0;
    for (unsigned i = 0; i < sizeof kRails / sizeof kRails[0]; ++i) {
        const Rail &rail = kRails[i];
        if (rail.millivolts * 1000u < rail.minUv || rail.millivolts * 1000u > rail.maxUv) {failed = -3; break;}
        uint32_t address = 0;
        if (!findRes(rail.name, &address, false)) {failed = -3; break;}
        if (i < 6) native_wifi_pmic[11 + i] = address;
        else native_wifi_stats[22] = address;
        const uint32_t vals[3] = {rail.millivolts, 1u, 7u};
        if (!vote(address, vals, rail.ldo ? 3u : 2u)) {failed = -3; break;}
        result |= 1u << i;
    }
    uint32_t clock = 0;
    const uint32_t enable = 1;
    if (!failed && findRes("C1A_E0", &clock, true) && vote(clock + 4, &enable, 1)) native_wifi_stats[24] = clock;
    native_wifi_pon[9] = 0;
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
    native_wifi_pmic[23] = uint32_t(ns / 1000000);
    native_wifi_pmic[22] = result;
    native_wifi_stats[20] = result;
    release(rsc);
    return failed;
}

// The MEM window of pcie@1c00000: CPU and PCI addresses are the same, 0x40300000 for 0x3d00000 bytes (WIFI.md §1.2, patch SY_MEM_BASE).
constexpr uint64_t kMemBase = 0x40300000, kMemSize = 0x3d00000;

// ---- stage 5: the firmware into the endpoint (SMMU stream, DMA buffers, BHI secondary boot loader, MHI contexts, BHIe full image) ----
constexpr uint32_t kSegment = 128 * 1024, kSbl = 512 * 1024, kMaxSegments = 48;

// DMA window from sy_mem_setup (patch 1091-1100, 4650-4716). IOVA 0x18000000 is L1 index 0.
// The other window, SY_IOVA_BASE 0xAF800000 size 0x800000, is the MHI buffer and is not this one.
// Usable bytes are 0x2000000 - 0x1000 - 0x100000 = 0x1EFF000. Page 0 is the MSI sink. The last 1 MiB is SY_MEM_FILES.
constexpr uint64_t kDmaIova = 0x18000000ull, kDmaPte = 0x0060000000000e47ull, kSmmuPhys = 0x15000000ull;
constexpr uint32_t kDmaBytes = 0x02000000u, kDmaFiles = 0x100000u, kDmaSid = 0x1401u;
static_assert(kDmaIova + kDmaBytes == 0x1A000000ull, "window end");
static_assert(((kDmaIova >> 30) & 511) == 0, "l1 index");
static_assert(((kDmaIova >> 21) & 511) == 192, "l2 start");
static_assert((kDmaBytes >> 21) == 16, "l2 entries");
static_assert((kDmaBytes >> 12) == 8192, "l3 pages");
static_assert(kDmaBytes - 4096u - kDmaFiles == 0x1EFF000u, "usable");
struct DmaWin {
    uint8_t *va; vm_offset_t l1, l2, l3; uint64_t iova; uint32_t bytes;
    uintptr_t gr0, gr1, cb; uint32_t asid, slot, bank, stream; int ready, err;
};
DmaWin gDma;
Window gSmmuKeep[4];
unsigned gSmmuKeepN;
uint32_t gDmaUsed;
struct Dma {uint8_t *va; uint64_t pa; size_t bytes;};

uintptr_t map4k(Window &w, uint64_t phys) {
    w = {};
    if (phys & 0xfffu) return 0;
    if (!mapWindow(w, phys, 0x1000)) return 0;
    return (uintptr_t)w.va;
}
uint32_t smmuRd(uintptr_t b, uint32_t o) {return *(volatile uint32_t *)(b + o);}
void smmuWr(uintptr_t b, uint32_t o, uint32_t v) {*(volatile uint32_t *)(b + o) = v;}
uint64_t dmaPa(vm_offset_t va) {
    const ppnum_t pn = pmap_find_phys(kernel_pmap, va);
    return pn ? ptoa_64(pn) : 0;
}
int paOk(uint64_t pa) {return pa && !(pa & 0xfffu) && !(pa >> 36);}

// Page tables for this window only. Descriptor bits are the patch's: table entries end in 3,
// leaf entries are PA | 0x0060000000000e47 (patch 4685-4692). The 8 MiB window uses 0xe43; this one does not.
int buildPt(vm_offset_t buf, vm_offset_t l1, vm_offset_t l2, vm_offset_t l3) {
    uint64_t pa = dmaPa(l2);
    if (!paOk(pa)) return 12;
    ((uint64_t *)l1)[(kDmaIova >> 30) & 511] = pa | 3ull;
    for (unsigned i = 0; i < (kDmaBytes >> 21); ++i) {
        pa = dmaPa(l3 + ((vm_offset_t)i << 12));
        if (!paOk(pa)) return 12;
        ((uint64_t *)l2)[((kDmaIova >> 21) + i) & 511] = pa | 3ull;
    }
    for (unsigned i = 0; i < (kDmaBytes >> 12); ++i) {
        pa = dmaPa(buf + ((vm_offset_t)i << 12));
        if (!paOk(pa)) return 12;
        ((uint64_t *)(l3 + (((vm_offset_t)i >> 9) << 12)))[i & 0x1ff] = (pa & 0x0000fffffffff000ull) | kDmaPte;
    }
    for (unsigned i = 0; i < 16; ++i) flush_dcache64(dmaPa(l3 + ((vm_offset_t)i << 12)), 4096, 1);
    flush_dcache64(dmaPa(l2), 4096, 1);
    flush_dcache64(dmaPa(l1), 4096, 1);
    __asm__ volatile("dsb sy" ::: "memory");
    return 0;
}

// Stream 0x1401, same scan and same register values as sy_smmu_install (patch 2506-2696).
// No register is written until a free slot, a free bank and an ASID are in hand. A stream that
// already covers 0x1401 returns 8 and is left alone.
int smmuSelect() {
    Window w = {};
    const uintptr_t gr0 = map4k(w, kSmmuPhys);
    if (!gr0) return 1;
    gSmmuKeep[gSmmuKeepN++] = w;
    gDma.gr0 = gr0;
    const uint32_t cr0 = smmuRd(gr0, 0x00), id0 = smmuRd(gr0, 0x20), id1 = smmuRd(gr0, 0x24), id2 = smmuRd(gr0, 0x28);
    const uint32_t streams = id0 & 0xffu, banks = id1 & 0xffu, ext = (cr0 >> 3) & 1u;
    const uint32_t shift = (id1 >> 31) ? 16u : 12u;
    const uint32_t pages = 2u << ((id1 >> 28) & 7u);
    const uint32_t ns2 = (id1 >> 16) & 0xffu;
    IOLog("wifi dma: smmu cr0 %x id0 %x id1 %x id2 %x streams %u banks %u shift %u pages %u ns2 %u\n",
          cr0, id0, id1, id2, streams, banks, shift, pages, ns2);
    if ((cr0 & 0x401u) != 0x400u || !streams || !banks || streams > 128u || banks > 128u || ns2 >= banks) return 2;
    w = {};
    const uintptr_t gr1 = map4k(w, kSmmuPhys + (1ull << shift));
    if (!gr1) return 3;
    gSmmuKeep[gSmmuKeepN++] = w;
    gDma.gr1 = gr1;
    unsigned slot = ~0u;
    uint8_t used[128];
    uint64_t usedAsid[256];
    for (unsigned i = 0; i < 128u; ++i) used[i] = 0;
    for (unsigned i = 0; i < 256u; ++i) usedAsid[i] = 0;
    for (unsigned i = 0; i < streams; ++i) {
        const uint32_t smr = smmuRd(gr0, 0x800 + 4 * i), s2cr = smmuRd(gr0, 0xc00 + 4 * i);
        if (ext ? !(s2cr & (1u << 10)) : !(smr & 0x80000000u)) {
            if (slot == ~0u) slot = i;
            continue;
        }
        if ((s2cr & 0xffu) < banks) used[s2cr & 0xffu] = 1;
        const uint32_t mask = (smr >> 16) & (ext ? 0xffffu : 0x7fffu);
        const uint32_t id = smr & (ext ? 0xffffu : 0x7fffu);
        if ((kDmaSid & ~mask) == (id & ~mask)) {
            IOLog("wifi dma: sid 0x1401 covered by stream %u smr %x s2cr %x\n", i, smr, s2cr);
            return 8;
        }
    }
    if (slot == ~0u) return 4;
    for (unsigned i = 0; i < banks; ++i) {
        const uint32_t v = smmuRd(gr1, 4 * i);
        if (((v >> 16) & 3u) == 3u && ((v >> 8) & 0xffu) < banks) used[(v >> 8) & 0xffu] = 1;
        if (((v >> 16) & 3u) == 0u) continue;
        Window one = {};
        const uintptr_t t = map4k(one, kSmmuPhys + ((uint64_t)pages + i) * (1ull << shift));
        if (!t) continue;
        const uint64_t tt0 = ((uint64_t)smmuRd(t, 0x24) << 32) | smmuRd(t, 0x20);
        const uint64_t tt1 = ((uint64_t)smmuRd(t, 0x2c) << 32) | smmuRd(t, 0x28);
        usedAsid[2 * i] = tt0 >> 48;
        usedAsid[2 * i + 1] = tt1 >> 48;
        release(one);
    }
    uintptr_t cb = 0;
    for (unsigned i = ns2; i < banks; ++i) {
        if (used[i]) continue;
        Window one = {};
        const uintptr_t t = map4k(one, kSmmuPhys + ((uint64_t)pages + i) * (1ull << shift));
        if (!t) continue;
        if (smmuRd(t, 0) & 1u) {release(one); continue;}
        const uint32_t v = smmuRd(t, 0x58);
        if (!(v & 0xc00001feu) && !(v & (~(0xc00001feu | 0x600u)))) {
            cb = t; gDma.bank = i;
            gSmmuKeep[gSmmuKeepN++] = one;
            break;
        }
        release(one);
    }
    if (!cb) return 5;
    uint32_t asid = 0;
    for (unsigned i = 0xff00u; i > 0xfd00u; --i) {
        unsigned j = 0;
        for (; j < 2 * banks; ++j) if (usedAsid[j] == i) break;
        if (j == 2 * banks) {asid = i; break;}
    }
    if (!asid) return 6;
    if (smmuRd(cb, 0) & 1u) return 7;
    gDma.cb = cb; gDma.asid = asid; gDma.slot = slot;
    return 0;
}

int smmuProgram(vm_offset_t l1) {
    const uint64_t ttbrPa = dmaPa(l1);
    if (!paOk(ttbrPa)) return 9;
    const uint32_t cr0 = smmuRd(gDma.gr0, 0x00);
    const uint32_t ext = (cr0 >> 3) & 1u;
    const uint32_t smr = ext ? kDmaSid : (0x80000000u | kDmaSid);
    const uint32_t s2cr = gDma.bank | (ext << 10);
    const uint64_t ttbr = ttbrPa | ((uint64_t)gDma.asid << 48);
    smmuWr(gDma.cb, 0x00, 0);
    smmuWr(gDma.gr1, 0x800 + 4 * gDma.bank, 1);
    smmuWr(gDma.gr1, 4 * gDma.bank, 0x1f300);
    smmuWr(gDma.cb, 0x10, 0x38011u);
    smmuWr(gDma.cb, 0x30, 0x80802020u);
    smmuWr(gDma.cb, 0x24, (uint32_t)(ttbr >> 32));
    smmuWr(gDma.cb, 0x20, (uint32_t)ttbr);
    smmuWr(gDma.cb, 0x28, 0);
    smmuWr(gDma.cb, 0x2c, 0);
    smmuWr(gDma.cb, 0x38, 0xff44u);
    smmuWr(gDma.cb, 0x3c, 0);
    __asm__ volatile("dsb sy" ::: "memory");
    smmuWr(gDma.cb, 0x610, gDma.asid);
    smmuWr(gDma.cb, 0x7f0, 0xffffffffu);
    for (unsigned i = 0; i < 100u; ++i) {
        if (smmuRd(gDma.cb, 0x7f4) & 1u) break;
        IODelay(1000);
    }
    smmuWr(gDma.cb, 0x00, 0x167u);
    __asm__ volatile("dsb sy" ::: "memory");
    smmuWr(gDma.gr0, 0xc00 + 4 * gDma.slot, s2cr);
    smmuWr(gDma.gr0, 0x800 + 4 * gDma.slot, kDmaSid);
    __asm__ volatile("dsb sy" ::: "memory");
    smmuWr(gDma.gr0, 0x800 + 4 * gDma.slot, smr);
    __asm__ volatile("dsb sy" ::: "memory");
    IOLog("wifi dma: installed slot %u bank %u asid %x iova %x sctlr %x tcr %x tcr2 %x mair %x cbar %x smr %x s2cr %x fsr %x\n",
          gDma.slot, gDma.bank, gDma.asid, (unsigned)kDmaIova, smmuRd(gDma.cb, 0x00), smmuRd(gDma.cb, 0x30), smmuRd(gDma.cb, 0x10),
          smmuRd(gDma.cb, 0x38), smmuRd(gDma.gr1, 4 * gDma.bank), smmuRd(gDma.gr0, 0x800 + 4 * gDma.slot),
          smmuRd(gDma.gr0, 0xc00 + 4 * gDma.slot), smmuRd(gDma.cb, 0x58));
    return 0;
}

int dmaWindow() {
    if (gDma.ready) return 0;
    if (gDma.err) return gDma.err;
    const int sel = smmuSelect();
    if (sel) {gDma.err = sel; IOLog("wifi dma: select %d\n", sel); return sel;}
    vm_offset_t buf = 0, l1 = 0, l2 = 0, l3 = 0;
    kern_return_t kr = kmem_alloc(kernel_map, &buf, kDmaBytes, KMA_DATA | KMA_ZERO | KMA_NOPAGEWAIT, VM_KERN_MEMORY_IOKIT);
    if (kr != KERN_SUCCESS || !buf) {gDma.err = 10; IOLog("wifi dma: buffer %d\n", kr); return 10;}
    kr = kmem_alloc(kernel_map, &l3, 0x10000, KMA_DATA | KMA_ZERO | KMA_NOPAGEWAIT, VM_KERN_MEMORY_IOKIT);
    if (kr != KERN_SUCCESS || !l3) {gDma.err = 11; IOLog("wifi dma: l3 %d\n", kr); return 11;}
    kr = kmem_alloc(kernel_map, &l2, 0x1000, KMA_DATA | KMA_ZERO | KMA_NOPAGEWAIT, VM_KERN_MEMORY_IOKIT);
    if (kr != KERN_SUCCESS || !l2) {gDma.err = 11; IOLog("wifi dma: l2 %d\n", kr); return 11;}
    kr = kmem_alloc(kernel_map, &l1, 0x1000, KMA_DATA | KMA_ZERO | KMA_NOPAGEWAIT, VM_KERN_MEMORY_IOKIT);
    if (kr != KERN_SUCCESS || !l1) {gDma.err = 11; IOLog("wifi dma: l1 %d\n", kr); return 11;}
    const int pt = buildPt(buf, l1, l2, l3);
    if (pt) {gDma.err = pt; IOLog("wifi dma: pt %d\n", pt); return pt;}
    const int prog = smmuProgram(l1);
    if (prog) {gDma.err = prog; return prog;}
    gDma.va = (uint8_t *)buf; gDma.l1 = l1; gDma.l2 = l2; gDma.l3 = l3;
    gDma.iova = kDmaIova; gDma.bytes = kDmaBytes;
    gDma.stream = gDma.slot | (gDma.bank << 16) | 0x80000000u;
    gDma.ready = 1;
    if (!gDmaUsed) gDmaUsed = 4096;
    return 0;
}

bool dmaAlloc(Dma &b, size_t bytes) {
    if (dmaWindow()) {native_wifi_mhi[11] = 1; return false;}
    const uint32_t take = (uint32_t)((bytes + 4095u) & ~4095u);
    if (take < bytes || gDmaUsed + take > gDma.bytes - kDmaFiles) {native_wifi_mhi[11] = 1; return false;}
    b.va = gDma.va + gDmaUsed;
    b.pa = gDma.iova + gDmaUsed;
    b.bytes = bytes;
    gDmaUsed += take;
    bzero(b.va, bytes);
    return true;
}

struct Fw {uint32_t lun; uint64_t first; uint32_t amssOffset, amssSize;};
// The container (local-tools/make-wifi-fw.py) in the partition named oops: 'WFW1', count, then name[24] offset size.
bool findFirmware(Fw &f) {
    if (&native_ufs_part_find == nullptr || &native_ufs_part_read == nullptr) return false;
    uint64_t blocks = 0;
    for (unsigned i = 0; i < 240; ++i) {   // the storage driver comes up a while into the boot
        if (native_ufs_part_find("oops", &f.lun, &f.first, &blocks)) break;
        IOSleep(500);
    }
    if (!blocks) return false;
    native_wifi_mhi[2] = f.lun | (uint32_t)f.first << 8;
    uint32_t which = 1;
    PE_parse_boot_argn("sheng-wifi-fw", &which, sizeof which);   // 2: the second image of the container (amss2.bin)
    const char *wanted = which == 2 ? "amss2.bin" : "amss.bin";
    uint8_t *header = static_cast<uint8_t *>(IOMallocData(4096));
    if (!header) return false;
    bool found = false;
    if (native_ufs_part_read(f.lun, f.first, 1, header) && !memcmp(header, "WFW1", 4)) {
        uint32_t count;
        memcpy(&count, header + 4, 4);
        for (uint32_t i = 0; i < count && i < 100; ++i) {
            const uint8_t *e = header + 8 + 32 * i;
            if (!strncmp((const char *)e, wanted, 24)) {memcpy(&f.amssOffset, e + 24, 4); memcpy(&f.amssSize, e + 28, 4); found = true;}
        }
    }
    IOFreeData(header, 4096);
    return found && f.amssSize && !(f.amssOffset & 4095);
}

struct BarIo {volatile uint32_t *m;};
uint32_t barRead(void *ctx, uint32_t reg) {return static_cast<BarIo *>(ctx)->m[reg / 4];}
uint32_t gTraceWrites, gSkipReg;   // a register whose write is left out (the boot loader's doorbell)
uint32_t gSkipped;
   // while non-zero every register write is announced in the log and followed by a pause long enough for a log slot to be written
void barWrite(void *ctx, uint32_t reg, uint32_t value) {
    if (gTraceWrites) {
        native_wifi_mhi[14] = (gTraceWrites << 16) | (reg & 0xffff); native_wifi_mhi[13] = value;
        IOSleep(2500);
        ++gTraceWrites;
    }
    if (gSkipReg && reg == gSkipReg) {gSkipped = value | 0x80000000u; return;}
    static_cast<BarIo *>(ctx)->m[reg / 4] = value; __asm__ volatile("dsb sy" ::: "memory");
}

void put64(volatile uint32_t *regs, uint32_t lowOffset, uint64_t value) {regs[lowOffset / 4] = (uint32_t)value; regs[(lowOffset + 4) / 4] = (uint32_t)(value >> 32);}

// The device context arrays and rings (mhi_init_dev_ctxt): 128 channel contexts left disabled, two event rings (control 32 elements, data 256) and the
// primary command ring, in one buffer; then the registers that point the device at them (mhi_init_mmio).
uint8_t *gEvVa[2]; uint8_t *gCtxVa; uint8_t *gSinkVa;
void programContexts(volatile uint32_t *m, Dma &c, uint64_t mapped) {
    constexpr uint32_t kCtx = 44, kChannels = 128, kEvents = 2, kCmdEl = 8, kEl = 16;
    const uint32_t evLen[kEvents] = {32 * kEl, 256 * kEl};
    uint8_t *va = c.va;
    gCtxVa = va;
    const uint64_t pa = c.pa;
    const uint32_t chOff = 0, evOff = kChannels * kCtx, cmdOff = evOff + kEvents * kCtx;
    uint32_t ringOff = (cmdOff + kCtx + 4095) & ~4095u;
    uint32_t cur = ringOff;
    auto w32 = [&](uint32_t at, uint32_t v) {memcpy(va + at, &v, 4);};
    auto w64 = [&](uint32_t at, uint64_t v) {memcpy(va + at, &v, 8);};
    uint64_t evBase[kEvents];
    for (uint32_t i = 0; i < kEvents; ++i) {
        evBase[i] = pa + cur;
        gEvVa[i] = va + cur;
        const uint32_t at = evOff + i * kCtx;
        w32(at, i == 0 ? 0 : 1u << 16);          // interrupt moderation 0 / 1 ms
        w32(at + 4, 1);                          // MHI_ER_TYPE_VALID
        w32(at + 8, i + 1);                      // MSI vector
        w64(at + 12, evBase[i]);
        w64(at + 20, evLen[i]);
        w64(at + 28, evBase[i]);                 // read pointer
        w64(at + 36, evBase[i] + evLen[i] - kEl);   // write pointer: the whole ring is free for the device
        cur += evLen[i];
    }
    const uint64_t cmdBase = pa + cur;
    w64(cmdOff + 12, cmdBase);
    w64(cmdOff + 20, kCmdEl * kEl);
    w64(cmdOff + 28, cmdBase);
    w64(cmdOff + 36, cmdBase);
    __asm__ volatile("dsb sy" ::: "memory");
    put64(m, 0x58, pa + chOff);                  // CCABAP
    put64(m, 0x60, pa + evOff);                  // ECABAP
    put64(m, 0x68, pa + cmdOff);                 // CRCBAP
    put64(m, 0x80, 0); put64(m, 0x88, 0xffffffffull);   // control base and limit
    put64(m, 0x98, 0); put64(m, 0xa0, 0xffffffffull);   // data base and limit
    m[0x10 / 4] = (m[0x10 / 4] & ~0xff000000u & ~0x00ff0000u) | (kEvents << 16);   // MHICFG: NER = 2, NHWER = 0
    // event ring doorbells (ERDBOFF + 8 * ring): the write pointers
    const uint32_t erdb = m[0x20 / 4];
    if (erdb + 8 * kEvents <= mapped) for (uint32_t i = 0; i < kEvents; ++i) put64(m, erdb + 8 * i, evBase[i] + evLen[i] - kEl);
    __asm__ volatile("dsb sy" ::: "memory");
}

// The endpoint's and the root port's configuration view, kept for looking at the link while the boot loader runs. When the endpoint answers in
// configuration space but its BAR0 or memory enable was cleared (its PCIe core restarting), they are written back.
volatile uint32_t *gCfg, *gDbi;
uint32_t gBar0, gRestored;
void linkSnapshot(volatile uint32_t *out) {
    out[0] = gCfg[0]; out[1] = gCfg[1]; out[2] = gCfg[0x10 / 4]; out[3] = gCfg[0x14 / 4];
    out[4] = gDbi[0x80 / 4]; out[5] = gDbi[kPortDebug0 / 4]; out[6] = gDbi[0x20 / 4]; out[7] = gDbi[1];
}
void linkRecover() {
    if (gCfg[0] != 0x110e17cb) return;
    if ((gCfg[0x10 / 4] & ~0xfu) != (gBar0 & ~0xfu) || (gCfg[1] & 6u) != 6u) {
        gCfg[0x10 / 4] = gBar0; gCfg[0x14 / 4] = 0;
        gCfg[1] = (gCfg[1] & 0xffffu) | 0x6u;
        __asm__ volatile("dsb sy" ::: "memory");
        ++gRestored;
    }
}

// The chip's own power-up recipe (ath12k_pci_power_up): the boot loader is only started on a chip whose power control, interrupts and warm-boot marker were
// put in order first. Chip registers above 0x80000 are seen through a window: this phone is 17cb:110e, and its cnss2
// (reg.h line 11, PEACH_PCIE_REMAP_BAR_CTRL_OFFSET) writes the window at BAR + 0x3278. The value is 0x40000000 | (offset bits 24:19).
// The register is then at BAR + 0x80000 + (offset & 0x7ffff); the MHI block (0x1e0e100 and up) is at the start of the BAR.
constexpr uint32_t kRemapBar = 0x3278;
uint32_t gWindow = 0xffffffffu;
uint32_t chipRead(volatile uint32_t *m, uint32_t off) {
    if (off < 0x80000) return m[off / 4];
    const uint32_t window = (off >> 19) & 0x3fu;
    if (window != gWindow) {m[kRemapBar / 4] = 0x40000000u | window; (void)m[kRemapBar / 4]; gWindow = window;}
    if (off >= 0x1e0e100u && off <= 0x1e0effcu) return m[((off - 0x1e0e100u) & 0x7ffffu) / 4];
    return m[(0x80000u + (off & 0x7ffffu)) / 4];
}
void chipWrite(volatile uint32_t *m, uint32_t off, uint32_t value) {
    if (off < 0x80000) {m[off / 4] = value; __asm__ volatile("dsb sy" ::: "memory"); return;}
    const uint32_t window = (off >> 19) & 0x3fu;
    if (window != gWindow) {m[kRemapBar / 4] = 0x40000000u | window; (void)m[kRemapBar / 4]; gWindow = window;}
    if (off >= 0x1e0e100u && off <= 0x1e0effcu) m[((off - 0x1e0e100u) & 0x7ffffu) / 4] = value;
    else m[(0x80000u + (off & 0x7ffffu)) / 4] = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
// ath12k_pci_sw_reset(power on). Values seen along the way go to native_wifi_chip: [0] LTSSM word, [1] hot reset word, [2] Q6 cookie, [3] warm entry,
// [4] warm entry after clearing, [5] reset cause, [6] global reset word before, [7] after, [8] MHISTATUS before the MHI reset, [9] MHISTATUS after, [10] EE after.
void chipPowerUp(volatile uint32_t *m) {
    volatile uint32_t *c = native_wifi_chip;
    gWindow = 0xffffffffu;
    uint32_t v = chipRead(m, 0x1e081b0u);
    for (unsigned i = 0; v != 0x111u && i < 5; ++i) {
        if (v == 0xffffffffu) IOSleep(5);
        chipWrite(m, 0x1e081b0u, 0x111u);
        v = chipRead(m, 0x1e081b0u);
    }
    c[0] = v;
    v = chipRead(m, 0x1e40304u); v |= 0x10u; chipWrite(m, 0x1e40304u, v); c[1] = chipRead(m, 0x1e40304u);
    IOSleep(5);
    chipWrite(m, 0x1e08228u, 0xffffffffu);   // interrupts left over from a hot reset
    v = chipRead(m, 0x1f8031cu); v &= ~0x4u; chipWrite(m, 0x1f8031cu, v);
    chipWrite(m, 0x360, 0); chipWrite(m, 0x368, 0); chipWrite(m, 0x394, 0); chipWrite(m, 0x39c, 0);   // BHIe vector doorbells and status
    c[2] = chipRead(m, 0x1f80500u);
    c[3] = chipRead(m, 0x1f80504u);
    IOSleep(10);
    chipWrite(m, 0x1f80504u, 0);             // keeps the Q6 off the warm boot path
    IOSleep(10);
    c[4] = chipRead(m, 0x1f80504u);
    c[5] = chipRead(m, 0x1f8060cu);          // read-clear
    v = chipRead(m, 0x3008u); c[6] = v;
    chipWrite(m, 0x3008u, v | 1u); (void)chipRead(m, 0x3008u);
    IOSleep(10);
    chipWrite(m, 0x3008u, v & ~1u); (void)chipRead(m, 0x3008u);
    IOSleep(10);
    c[7] = chipRead(m, 0x3008u);
    m[kRemapBar / 4] = 0x40000000u | (gWindow == 0xffffffffu ? 0u : gWindow);   // the window register is cleared by the reset; put the cached window back
    (void)m[kRemapBar / 4];
    c[8] = m[0x48 / 4];
    m[0x38 / 4] = 2;                         // MHICTRL reset: clears a SYSERR left by the global reset
    __asm__ volatile("dsb sy" ::: "memory");
    IOSleep(10);
    c[9] = m[0x48 / 4];
    c[10] = m[(m[0x28 / 4] + 0x28) / 4];
}
// Without ASPM during the load, and MSI enabled (the endpoint raises its completions as MSI writes; they land in a sink in the DMA buffer, polled later).
void endpointLinkSetup(uint64_t sinkAddress) {
    uint8_t cap = (uint8_t)(gCfg[0x34 / 4] & 0xff);
    for (unsigned guard = 0; cap >= 0x40 && guard < 48; ++guard) {
        const uint32_t w = gCfg[cap / 4];
        const uint8_t id = (uint8_t)w, next = (uint8_t)(w >> 8);
        if (id == 0x10) {   // PCI Express: link control, ASPM bits
            const uint32_t lc = gCfg[(cap + 0x10) / 4];
            native_wifi_chip[11] = lc;
            gCfg[(cap + 0x10) / 4] = lc & ~3u;
        } else if (id == 0x05) {   // MSI: address, data, then 4 vectors and enable
            const bool is64 = (w >> 16) & 0x80;
            gCfg[(cap + 4) / 4] = (uint32_t)sinkAddress;
            if (is64) {gCfg[(cap + 8) / 4] = (uint32_t)(sinkAddress >> 32); gCfg[(cap + 0xc) / 4] = 0;}
            else gCfg[(cap + 8) / 4] = 0;
            gCfg[cap / 4] = (w & 0xffffu) | (((w >> 16) & ~0x70u) | 0x20u | 1u) << 16;
            native_wifi_chip[12] = gCfg[cap / 4];
        }
        cap = next;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}


// ---- Stage 6: the WiFi module (Ath12kWifi) on this board. It does its own chip power-up, MHI boot, QRTR/QMI and driver start; the board provides
// register access, configuration space, DMA memory from the endpoint's SMMU window, firmware files from the container, a clock and a log.
struct Arena {uint8_t *va; uint64_t iova; size_t bytes; struct Block {uint32_t off, size; bool used;} blocks[16384]; unsigned count;} gArena;
bool arenaInit() {
    if (dmaWindow()) return false;
    gArena.va = gDma.va; gArena.iova = gDma.iova; gArena.bytes = gDma.bytes;
    gArena.blocks[0] = {4096, gDma.bytes - 4096u - kDmaFiles, false}; gArena.count = 1;   // page 0 is the MSI sink, the last 1 MiB is the file tail
    bzero(gArena.va, gArena.bytes);
    return true;
}
// Buffers the endpoint reads or writes sit at addresses aligned to their size rounded up to a power of two, with no upper limit: the boot loader
// refuses an image address that is only page aligned, before it makes any transfer (the registers that name the address and size of a failed transfer
// stay zero), the firmware's host memory regions (several MiB) go through the same address translation, and an IOMMU-backed DMA allocator hands out
// addresses aligned that way.
void pLog(int level, const char *line);
void *arenaAlloc(size_t n, uint64_t *bus) {
    n = (n + 63) & ~(size_t)63;
    if (n >= 4096) n = (n + 4095) & ~(size_t)4095;
    uint32_t align = 64;
    if (n >= 4096) {align = 4096; while (align < n) align <<= 1;}
    for (unsigned i = 0; i < gArena.count; ++i) {
        Arena::Block &b = gArena.blocks[i];
        if (b.used) continue;
        const uint32_t start = (b.off + align - 1) & ~(align - 1), pad = start - b.off;
        if (b.size < pad + n) continue;
        const bool tail = b.size > pad + n;
        if (gArena.count + (pad ? 1 : 0) + (tail ? 1 : 0) > 16384) continue;
        const unsigned extra = (pad ? 1u : 0u) + (tail ? 1u : 0u);
        const Arena::Block whole = b;
        memmove(&gArena.blocks[i + extra], &gArena.blocks[i], (gArena.count - i) * sizeof(Arena::Block));
        gArena.count += extra;
        unsigned at = i;
        if (pad) {gArena.blocks[at] = {whole.off, pad, false}; ++at;}
        gArena.blocks[at] = {start, (uint32_t)n, true};
        if (tail) gArena.blocks[at + 1] = {start + (uint32_t)n, whole.size - pad - (uint32_t)n, false};
        uint8_t *p = gArena.va + start;
        bzero(p, n);
        if (bus) *bus = gArena.iova + start;
        return p;
    }
    uint32_t freeTotal = 0, freeMax = 0;
    for (unsigned i = 0; i < gArena.count; ++i) if (!gArena.blocks[i].used) {freeTotal += gArena.blocks[i].size; if (gArena.blocks[i].size > freeMax) freeMax = gArena.blocks[i].size;}
    char line[96];
    snprintf(line, sizeof line, "dma window full: want %zu free %u largest %u blocks %u", n, freeTotal, freeMax, gArena.count);
    pLog(3, line);
    // used blocks by size class: up to 4 KiB, 64 KiB, 512 KiB, larger (count and KiB)
    uint32_t cnt[4] = {0, 0, 0, 0}, kib[4] = {0, 0, 0, 0}, big[4] = {0, 0, 0, 0};
    for (unsigned i = 0; i < gArena.count; ++i) if (gArena.blocks[i].used) {
        const uint32_t sz = gArena.blocks[i].size;
        const unsigned c = sz <= 4096 ? 0 : sz <= 65536 ? 1 : sz <= 524288 ? 2 : 3;
        ++cnt[c]; kib[c] += sz / 1024;
        for (unsigned k = 0; k < 4; ++k) if (sz > big[k]) {for (unsigned m = 3; m > k; --m) big[m] = big[m - 1]; big[k] = sz; break;}
    }
    snprintf(line, sizeof line, "dma used: <=4K %u/%uK <=64K %u/%uK <=512K %u/%uK more %u/%uK", cnt[0], kib[0], cnt[1], kib[1], cnt[2], kib[2], cnt[3], kib[3]);
    pLog(3, line);
    snprintf(line, sizeof line, "dma largest used: %u %u %u %u", big[0], big[1], big[2], big[3]);
    pLog(3, line);
    return nullptr;
}
void arenaFree(void *p) {
    if (!p || (uint8_t *)p < gArena.va || (uint8_t *)p >= gArena.va + gArena.bytes) return;
    const uint32_t off = (uint32_t)((uint8_t *)p - gArena.va);
    for (unsigned i = 0; i < gArena.count; ++i) {
        if (gArena.blocks[i].off != off || !gArena.blocks[i].used) continue;
        gArena.blocks[i].used = false;
        if (i + 1 < gArena.count && !gArena.blocks[i + 1].used) {gArena.blocks[i].size += gArena.blocks[i + 1].size; memmove(&gArena.blocks[i + 1], &gArena.blocks[i + 2], (gArena.count - i - 2) * sizeof(Arena::Block)); --gArena.count;}
        if (i > 0 && !gArena.blocks[i - 1].used) {gArena.blocks[i - 1].size += gArena.blocks[i].size; memmove(&gArena.blocks[i], &gArena.blocks[i + 1], (gArena.count - i - 1) * sizeof(Arena::Block)); --gArena.count;}
        return;
    }
}

uint64_t pNow() {uint64_t ns = 0; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}
void pSleep(unsigned ms) {IOSleep(ms);}
void pPoll() {}
// Each line carries the milliseconds since the first one; a line equal to one of the last sixteen is only counted, so that the messages the driver
// repeats every round (the seven interrupt groups it polls each millisecond, a send that waits for credits) cannot push the history out of the ring.
uint32_t gLogRepeats;
volatile uint32_t *gNwBar;   // the endpoint's registers, for the RAM dump reader
void rddmScan();
extern "C" int boot_log_export_put(uint32_t generation, uint32_t total, uint32_t offset, const void *data, uint32_t size) __attribute__((weak_import));
bool has(const char *s, const char *w) {for (; *s; ++s) {size_t k = 0; while (w[k] && s[k] == w[k]) ++k; if (!w[k]) return true;} return false;}
bool hasRddm(const char *s) {return has(s, "MHI_CB_EE_RDDM");}
void pLog(int level, const char *line) {
    // one line per channel would push the start-up messages out of the ring
    if (has(line, "chan scan list") || has(line, "mac channel [")) {++gLogRepeats; return;}
    // nor may the lines of every received buffer and interrupt (the beacon count is in [wifi-nw])
    if (level >= 7 && (has(line, "htc rx completion") || has(line, "rx ce pipe") || has(line, "ext irq:") || has(line, "event mgmt rx") ||
                       has(line, "mgmt rx event") || has(line, "chan info vdev_id") || has(line, "scan event foreign channel"))) {++gLogRepeats; return;}
    static uint64_t first;
    static uint32_t recent[16];
    uint32_t h = 2166136261u;
    for (const char *c = line; *c; ++c) h = (h ^ static_cast<uint8_t>(*c)) * 16777619u;
    h ^= static_cast<uint32_t>(level);
    for (uint32_t r : recent) if (r == h) {++gLogRepeats; return;}
    memmove(&recent[1], &recent[0], sizeof recent - sizeof recent[0]);
    recent[0] = h;
    const uint64_t now = pNow();
    if (!first) first = now;
    const uint32_t i = __atomic_fetch_add(const_cast<uint32_t *>(&native_wifi_text_n), 1u, __ATOMIC_RELAXED) % 512;
    snprintf(native_wifi_text[i], sizeof native_wifi_text[i], "%u %d %.100s", static_cast<uint32_t>(now - first), level, line);
    if (level <= 6) {
        const uint32_t j = __atomic_fetch_add(const_cast<uint32_t *>(&native_wifi_note_n), 1u, __ATOMIC_RELAXED) % 128;
        memcpy(native_wifi_note[j], native_wifi_text[i], sizeof native_wifi_note[j]);
    }
    // when the driver reports a command stuck: what the root port saw (device status, link, advanced error reporting)
    static bool rootShown;
    if (!rootShown && gDbi && has(line, "stuck: wmi credits")) {
        rootShown = true;
        char text[112];
        snprintf(text, sizeof text, "root port: devctl/sta %08x link %08x ue %08x mask %08x ce %08x rootsta %08x src %08x",
                 gDbi[0x78 / 4], gDbi[0x80 / 4], gDbi[0x104 / 4], gDbi[0x108 / 4], gDbi[0x110 / 4], gDbi[0x130 / 4], gDbi[0x134 / 4]);
        pLog(3, text);
        snprintf(text, sizeof text, "root port: aer header %08x %08x %08x %08x ltssm-debug %08x", gDbi[0x11c / 4], gDbi[0x120 / 4], gDbi[0x124 / 4], gDbi[0x128 / 4], gDbi[kPortDebug0 / 4]);
        pLog(3, text);
    }
    // the first time the device reports its RAM dump state: the context bank this window was installed on
    static bool faultShown;
    if (!faultShown && gDma.cb && hasRddm(line)) {
        faultShown = true;
        char text[112];
        snprintf(text, sizeof text, "smmu at rddm: cb-fsr %08x sctlr %08x iova %08x", smmuRd(gDma.cb, 0x58), smmuRd(gDma.cb, 0x00), (unsigned)gDma.iova);
        pLog(3, text);
        rddmScan();
    }
}

// The device uploads its RAM dump into the buffers of the receive vector table the MHI core programmed at power up. Once the transfer is complete
// the dump is searched for the messages a firmware error leaves (an assertion or fatal error text, or a source file with a line number).
bool printable(uint8_t c) {return c >= 0x20 && c < 0x7f;}
bool looksLikeError(const char *t, size_t n) {
    for (size_t i = 0; i < n; ++i) if (t[i] == '%') return false;   // a format string of the image, not a message
    for (size_t i = 0; i + 3 < n; ++i) {
        if (t[i] == '.' && t[i + 1] == 'c' && t[i + 2] == ':' && t[i + 3] >= '0' && t[i + 3] <= '9') return true;
        if (i + 5 <= n && (!memcmp(t + i, "ssert", 5) || !memcmp(t + i, "SSERT", 5) || !memcmp(t + i, "FATAL", 5))) return true;
        if (i + 8 <= n && !memcmp(t + i, "rr_fatal", 8)) return true;
    }
    return false;
}
void rddmScan() {
    volatile uint32_t *m = gNwBar;
    if (!m) return;
    const uint32_t bhie = m[0x2c / 4];
    if (bhie == 0 || bhie > 0x1000 || (bhie & 3)) return;
    uint32_t status = 0;
    unsigned waited = 0;
    for (; waited < 400; ++waited) {status = m[(bhie + 0x78) / 4]; if ((status >> 30) >= 2) break; IOSleep(10);}
    const uint64_t vec = m[(bhie + 0x60) / 4] | (uint64_t)m[(bhie + 0x64) / 4] << 32;
    const uint32_t vecBytes = m[(bhie + 0x68) / 4];
    char text[112];
    snprintf(text, sizeof text, "rddm upload: status %08x after %u ms, table %llx %u bytes", status, waited * 10, (unsigned long long)vec, vecBytes);
    pLog(3, text);
    auto toVa = [](uint64_t iova, uint64_t n) -> const uint8_t * {
        if (iova < gArena.iova || iova + n > gArena.iova + gArena.bytes) return nullptr;
        return gArena.va + (iova - gArena.iova);
    };
    const uint8_t *table = toVa(vec, vecBytes);
    if (!table || (status >> 30) != 2) return;
    unsigned found = 0;
    uint64_t scanned = 0;
    uint32_t seen[48] = {};
    for (uint32_t e = 0; e + 16 <= vecBytes && found < 40; e += 16) {
        uint64_t addr, size;
        memcpy(&addr, table + e, 8); memcpy(&size, table + e + 8, 8);
        const uint8_t *seg = toVa(addr, size);
        if (!seg) continue;
        scanned += size;
        for (uint64_t i = 0; i < size && found < 40;) {
            if (!printable(seg[i])) {++i; continue;}
            uint64_t j = i;
            while (j < size && printable(seg[j]) && j - i < 200) ++j;
            const size_t n = (size_t)(j - i);
            if (n >= 10 && looksLikeError(reinterpret_cast<const char *>(seg + i), n)) {
                uint32_t h = 2166136261u;
                for (size_t k = 0; k < n; ++k) h = (h ^ seg[i + k]) * 16777619u;
                bool dup = false;
                for (unsigned k = 0; k < found && k < 48; ++k) if (seen[k] == h) dup = true;
                if (!dup) {
                    seen[found] = h;
                    ++found;
                    char body[85];
                    const size_t len = n < 84 ? n : 84;
                    memcpy(body, seg + i, len); body[len] = 0;
                    snprintf(text, sizeof text, "rddm %u+%llx: %s", e / 16, (unsigned long long)i, body);
                    for (char *c = text; *c; ++c) if (!printable((uint8_t)*c)) *c = '?';
                    pLog(3, text);
                }
            }
            i = j;
        }
    }
    snprintf(text, sizeof text, "rddm scan: %llu bytes, %u messages", (unsigned long long)scanned, found);
    pLog(3, text);
    // Context: every text in the crash record (segment 0 up to 0x1000), and the runtime log the firmware kept before it (segment 1, 0x78000 on): the
    // first as it is, the second without format strings, 4 bytes per word of the record shown where no text is.
    struct Range {uint32_t seg; uint64_t from, to; size_t min; bool formats;};
    const Range ranges[] = {{0, 0, 0x1000, 4, true}, {0, 0xec00, 0xf400, 4, true}, {1, 0x78000, 0x80000, 12, false}};
    unsigned lines = 0;
    for (const Range &r : ranges) {
        if ((r.seg + 1) * 16 > vecBytes) continue;
        uint64_t addr, size;
        memcpy(&addr, table + r.seg * 16, 8); memcpy(&size, table + r.seg * 16 + 8, 8);
        const uint8_t *seg = toVa(addr, size);
        if (!seg) continue;
        const uint64_t to = r.to < size ? r.to : size;
        for (uint64_t i = r.from; i < to && lines < 160;) {
            if (!printable(seg[i])) {++i; continue;}
            uint64_t j = i;
            while (j < to && printable(seg[j]) && j - i < 200) ++j;
            const size_t n = (size_t)(j - i);
            bool format = false;
            for (size_t k = 0; k < n; ++k) if (seg[i + k] == '%') format = true;
            if (n >= r.min && (r.formats || !format)) {
                char body[85];
                const size_t len = n < 84 ? n : 84;
                memcpy(body, seg + i, len); body[len] = 0;
                snprintf(text, sizeof text, "rddm-ctx %u+%llx: %s", r.seg, (unsigned long long)i, body);
                pLog(3, text);
                ++lines;
            }
            i = j;
        }
    }
    // the crash record's first 0x200 bytes as words (registers and addresses sit there)
    {
        uint64_t addr, size;
        memcpy(&addr, table, 8); memcpy(&size, table + 8, 8);
        const uint8_t *seg = toVa(addr, size);
        if (seg && size >= 0x200) for (uint32_t o = 0; o < 0x200; o += 32) {
            uint32_t w[8];
            memcpy(w, seg + o, 32);
            snprintf(text, sizeof text, "rddm-rec %03x: %08x %08x %08x %08x %08x %08x %08x %08x", o, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
            pLog(3, text);
        }
    }
    // The whole dump goes to the boot log's export area, read back on the host: the receive vector table as table.bin and each of its buffers as
    // segNN.bin, in the container the log reader unpacks (SHEXR001 holding an M2VCACH1 file list). Its generation has the top bit set, which keeps
    // a later export from replacing it.
    if (!&boot_log_export_put) return;
    static const char root[] = "/rddm";
    uint64_t total = 8 + 4 + sizeof root - 1 + 4, files = 1;
    total += 24 + 9 + vecBytes;
    for (uint32_t e = 0; e + 16 <= vecBytes; e += 16) {
        uint64_t addr, size;
        memcpy(&addr, table + e, 8); memcpy(&size, table + e + 8, 8);
        if (toVa(addr, size)) {total += 24 + 9 + size; ++files;}
    }
    const uint64_t blobBytes = 16 + total;
    uint8_t *blob = blobBytes < (64u << 20) ? static_cast<uint8_t *>(IOMallocData(blobBytes)) : nullptr;
    if (!blob) {pLog(3, "rddm export: no memory"); return;}
    uint8_t *w = blob;
    auto put = [&w](const void *d, size_t n) {memcpy(w, d, n); w += n;};
    auto put32 = [&put](uint32_t v) {put(&v, 4);};
    auto file = [&](const char *name, const uint8_t *d, uint64_t n) {
        const uint32_t head[4] = {(uint32_t)strlen(name), 0644, 0, 0};
        put(head, 16); put(&n, 8); put(name, strlen(name)); put(d, n);
    };
    put("SHEXR001", 8); put32((uint32_t)total); put32(0);
    put("M2VCACH1", 8); put32(sizeof root - 1); put(root, sizeof root - 1); put32((uint32_t)files);
    file("table.bin", table, vecBytes);
    for (uint32_t e = 0; e + 16 <= vecBytes; e += 16) {
        uint64_t addr, size;
        memcpy(&addr, table + e, 8); memcpy(&size, table + e + 8, 8);
        const uint8_t *seg = toVa(addr, size);
        if (!seg) continue;
        char name[16];
        snprintf(name, sizeof name, "seg%02u.bin", e / 16);
        file(name, seg, size);
    }
    const uint32_t generation = 0x80000000u | (uint32_t)(pNow() & 0x7fffffff);
    const int rc = (uint64_t)(w - blob) == blobBytes ? boot_log_export_put(generation, (uint32_t)blobBytes, 0, blob, (uint32_t)blobBytes) : -100;
    IOFreeData(blob, blobBytes);
    snprintf(text, sizeof text, "rddm export: %llu bytes in %llu files, result %d", (unsigned long long)blobBytes, (unsigned long long)files, rc);
    pLog(3, text);
}
void pHalt() {for (;;) IOSleep(1000);}
void *hAlloc(size_t n) {
    uint8_t *p = static_cast<uint8_t *>(IOMallocData(n + 16));
    if (!p) return nullptr;
    *reinterpret_cast<size_t *>(p) = n + 16;
    return p + 16;
}
void hFree(void *q) {
    if (!q) return;
    uint8_t *p = static_cast<uint8_t *>(q) - 16;
    IOFreeData(p, *reinterpret_cast<size_t *>(p));
}
void *pDmaAlloc(size_t n, uint64_t *bus) {return arenaAlloc(n, bus);}
void pDmaFree(size_t, void *p, uint64_t) {arenaFree(p);}
uint64_t pDmaMap(void *p, size_t, int) {
    if ((uint8_t *)p < gArena.va || (uint8_t *)p >= gArena.va + gArena.bytes) return 0;
    return gArena.iova + ((uint8_t *)p - gArena.va);
}
void pDmaSync(uint64_t, size_t, int) {}

// Firmware files: PANDFW01 in the pinned userdata pack. Header is sy_fw_find (patch 2327-2369):
// "PANDFW01", u32 count at byte 8 (1..63), then count entries of {name[48], u64 offset, u64 size}
// at byte 16. Extents are /chosen pandora-fw-extents, partition-relative 4 KiB triples, on the
// userdata partition (patch 0144 and 0088). The stage-5 WFW1 reader is a different path.
constexpr unsigned kFwExtMax = 256;
constexpr uint32_t kFwBlock = 4096;
static_assert(48 + 8 + 8 == 64, "PANDFW01 entry");
static_assert(16 + 63 * 64 <= 4096, "directory fits in one block");
struct FwExt {uint64_t file, part, count;};
struct FwPack {uint32_t lun; uint64_t first, blocks; FwExt ext[kFwExtMax]; unsigned n;} gFwPack;
struct FwCache {char name[48]; uint8_t *data; size_t size; bool tried;} gFw[8];

int fwPackOpen() {
    if (gFwPack.n) return 0;
    if (&native_ufs_part_find == nullptr || &native_ufs_part_read == nullptr) return -1;
    uint64_t partBlocks = 0;
    if (!native_ufs_part_find("userdata", &gFwPack.lun, &gFwPack.first, &partBlocks) || !partBlocks) return -1;
    IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
    OSData *ext = chosen ? OSDynamicCast(OSData, chosen->getProperty("pandora-fw-extents")) : nullptr;
    OSData *blocks = chosen ? OSDynamicCast(OSData, chosen->getProperty("pandora-fw-blocks")) : nullptr;
    const unsigned size = ext ? ext->getLength() : 0, bsize = blocks ? blocks->getLength() : 0;
    const unsigned n = size / (3 * (unsigned)sizeof(uint64_t));
    if (!ext || !blocks || bsize != sizeof(uint64_t) || !size || size % (3 * sizeof(uint64_t)) || n > kFwExtMax) {
        OSSafeReleaseNULL(chosen);
        return -1;
    }
    const uint64_t *e = static_cast<const uint64_t *>(ext->getBytesNoCopy());
    uint64_t want = 0, pos = 0;
    memcpy(&want, blocks->getBytesNoCopy(), sizeof want);
    unsigned kept = 0;
    for (unsigned i = 0; i < n && pos < want; ++i) {
        uint64_t file = e[3 * i], part = e[3 * i + 1], count = e[3 * i + 2];
        if (file != pos || !count || part >= partBlocks || count > partBlocks - part) {OSSafeReleaseNULL(chosen); return -1;}
        if (count > want - pos) count = want - pos;
        gFwPack.ext[kept++] = {file, part, count};
        pos += gFwPack.ext[kept - 1].count;
    }
    OSSafeReleaseNULL(chosen);
    if (pos != want || !kept) return -1;
    gFwPack.blocks = want;
    gFwPack.n = kept;
    return 0;
}
bool fwMap(uint64_t blk, uint64_t *lba, uint64_t *run) {
    for (unsigned i = 0; i < gFwPack.n; ++i) {
        const FwExt &e = gFwPack.ext[i];
        if (blk < e.file || blk - e.file >= e.count) continue;
        const uint64_t off = blk - e.file;
        *lba = gFwPack.first + e.part + off;
        *run = e.count - off;
        return true;
    }
    return false;
}
int fwRead(uint64_t off, void *dst, uint64_t len) {
    if (fwPackOpen()) return -1;
    if (!len || off + len < off || off + len > gFwPack.blocks * (uint64_t)kFwBlock) return -1;
    uint8_t *out = static_cast<uint8_t *>(dst);
    uint8_t *bounce = nullptr;
    while (len) {
        const uint64_t blk = off / kFwBlock;
        uint64_t lba = 0, run = 0;
        const uint32_t in = (uint32_t)(off % kFwBlock);
        if (!fwMap(blk, &lba, &run)) {if (bounce) IOFreeData(bounce, kFwBlock); return -1;}
        if (!in && len >= kFwBlock) {
            uint32_t n = (uint32_t)(len / kFwBlock);
            if ((uint64_t)n > run) n = (uint32_t)run;
            if (n > 256) n = 256;
            if (!native_ufs_part_read(gFwPack.lun, lba, n, out)) {if (bounce) IOFreeData(bounce, kFwBlock); return -1;}
            const uint64_t got = (uint64_t)n * kFwBlock;
            out += got; off += got; len -= got;
            continue;
        }
        if (!bounce) bounce = static_cast<uint8_t *>(IOMallocData(kFwBlock));
        if (!bounce || !native_ufs_part_read(gFwPack.lun, lba, 1, bounce)) {if (bounce) IOFreeData(bounce, kFwBlock); return -1;}
        uint64_t got = kFwBlock - in;
        if (got > len) got = len;
        memcpy(out, bounce + in, (size_t)got);
        out += got; off += got; len -= got;
    }
    if (bounce) IOFreeData(bounce, kFwBlock);
    return 0;
}
int fwFind(const char *name, uint64_t *off, uint64_t *size) {
    uint8_t blk[4096];
    uint32_t n = 0;
    if (fwRead(0, blk, sizeof blk)) return -1;
    if (memcmp(blk, "PANDFW01", 8)) return -1;
    for (unsigned i = 0; i < 4; ++i) n |= (uint32_t)blk[8 + i] << (8 * i);
    if (!n || n > 63) return -1;
    unsigned nl = 0;
    while (name[nl] && nl < 47) ++nl;
    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t *e = blk + 16 + i * 64;
        unsigned k = 0;
        for (; k < nl; ++k) if (e[k] != (uint8_t)name[k]) break;
        if (k != nl || e[nl]) continue;
        uint64_t eo = 0, es = 0;
        for (k = 0; k < 8; ++k) {
            eo |= (uint64_t)e[48 + k] << (8 * k);
            es |= (uint64_t)e[56 + k] << (8 * k);
        }
        *off = eo;
        *size = es;
        return 0;
    }
    return -1;
}
// The radio's address (wlan_mac.bin): this board's firmware reports none, so a locally administered address is made from /chosen/unique-chip-id,
// the same on every boot of the same image.
// The address itself: made from /chosen/unique-chip-id (FNV-1a, locally administered, unicast), the same at every boot.
bool boardMac(uint8_t m[6]) {
    IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
    OSData *id = chosen ? OSDynamicCast(OSData, chosen->getProperty("unique-chip-id")) : nullptr;
    uint64_t seed = 0;
    if (id && id->getLength() >= sizeof seed) memcpy(&seed, id->getBytesNoCopy(), sizeof seed);
    OSSafeReleaseNULL(chosen);
    if (!seed) return false;
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned i = 0; i < 8; ++i) h = (h ^ ((seed >> (8 * i)) & 0xff)) * 0x100000001b3ull;
    for (unsigned i = 0; i < 6; ++i) m[i] = (uint8_t)(h >> (8 * i));
    m[0] = (uint8_t)((m[0] & 0xfe) | 0x02);
    return true;
}
int macFile(const uint8_t **data, size_t *size) {
    static char text[40]; static size_t len;
    if (!len) {
        uint8_t m[6];
        if (!boardMac(m)) return -1;
        len = (size_t)snprintf(text, sizeof text, "address=%02x%02x%02x%02x%02x%02x\n", m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    *data = (const uint8_t *)text; *size = len;
    return 0;
}
// The system's own record of the Wi-Fi address, as a boot loader leaves it in the device tree: the wireless daemon derives every network's
// private address from it (MobileGestalt WifiAddressData) and aborts a join without it.
void publishMac() {
    uint8_t m[6];
    if (!boardMac(m)) return;
    if (IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane)) {
        if (!chosen->getProperty("mac-address-wifi0")) chosen->setProperty("mac-address-wifi0", m, sizeof m);
        chosen->release();
    }
    if (IORegistryEntry *wlan = IORegistryEntry::fromPath("/arm-io/wlan", gIODTPlane)) {
        if (!wlan->getProperty("local-mac-address")) wlan->setProperty("local-mac-address", m, sizeof m);
        wlan->release();
    }
}
// Request basename to the name sy_fw_find uses in 0350-pandora-varcache.patch.
// board-2.bin stays unmapped. core.c api 2 requests that file and requires the
// QCA-ATH12K-BOARD magic; only after that call fails does fetch_bdf ask for
// board.bin, which api 1 returns unmodified.
static const char *fwPackName(const char *base) {
    static const struct { const char *req; const char *pack; } map[] = {
        { "amss.bin", "amss20.bin" },
        { "m3.bin", "phy_ucode20.elf" },
        { "aux_ucode.bin", "aux_ucode20.elf" },
        { "regdb.bin", "regdb_xiaomi.bin" },
        { "board.bin", "bd_songyuan.elf" },
    };
    for (unsigned i = 0; i < sizeof map / sizeof map[0]; ++i)
        if (!strcmp(base, map[i].req)) return map[i].pack;
    return base;
}
int pFwGet(const char *path, const uint8_t **data, size_t *size) {
    const char *base = path;
    for (const char *c = path; *c; ++c) if (*c == '/') base = c + 1;
    if (!strcmp(base, "wlan_mac.bin")) return macFile(data, size);
    char want[48];
    uint32_t which = 1;
    PE_parse_boot_argn("sheng-wifi-fw", &which, sizeof which);
    if (!strcmp(base, "amss.bin") && which == 2) strlcpy(want, "amss2.bin", sizeof want);
    else strlcpy(want, fwPackName(base), sizeof want);
    FwCache *slot = nullptr;
    for (auto &f : gFw) {if (f.tried && !strncmp(f.name, want, sizeof f.name)) {slot = &f; break;} }
    if (!slot) for (auto &f : gFw) if (!f.tried) {slot = &f; strlcpy(f.name, want, sizeof f.name); f.tried = true; break;}
    if (!slot) return -1;
    if (!slot->data && slot->size == 0) {
        uint64_t offset = 0, bytes = 0;
        if (fwFind(want, &offset, &bytes) || !bytes || offset + bytes < offset || offset + bytes > gFwPack.blocks * (uint64_t)kFwBlock) return -1;
        uint8_t *buf = static_cast<uint8_t *>(IOMallocData((size_t)bytes));
        if (!buf) return -1;
        if (fwRead(offset, buf, bytes)) {IOFreeData(buf, (size_t)bytes); return -1;}
        slot->data = buf; slot->size = (size_t)bytes;
    }
    if (!slot->data) return -1;
    *data = slot->data; *size = slot->size;
    return 0;
}

// Configuration space of the endpoint: 32-bit accesses through the iATU window; offsets with bit 31 set are relative to the PCI Express capability.
uint32_t gPcieCap, gMsiCap;
int findCaps() {
    uint8_t cap = (uint8_t)(gCfg[0x34 / 4] & 0xff);
    for (unsigned guard = 0; cap >= 0x40 && guard < 48; ++guard) {
        const uint32_t w = gCfg[cap / 4];
        if ((uint8_t)w == 0x10) gPcieCap = cap;
        if ((uint8_t)w == 0x05) gMsiCap = cap;
        cap = (uint8_t)(w >> 8);
    }
    return gPcieCap && gMsiCap;
}
int pCfgRead(int off, int size, uint32_t *v) {
    uint32_t o = (uint32_t)off;
    if (o & 0x80000000u) o = gPcieCap + (o & 0x7fffffffu);
    const uint32_t w = gCfg[(o & ~3u) / 4];
    *v = size == 4 ? w : size == 2 ? (w >> ((o & 2) * 8)) & 0xffff : (w >> ((o & 3) * 8)) & 0xff;
    return 0;
}
int pCfgWrite(int off, int size, uint32_t v) {
    uint32_t o = (uint32_t)off;
    if (o & 0x80000000u) o = gPcieCap + (o & 0x7fffffffu);
    if (size == 4) {gCfg[o / 4] = v; return 0;}
    const unsigned shift = (size == 2 ? (o & 2) : (o & 3)) * 8;
    const uint32_t mask = (size == 2 ? 0xffffu : 0xffu) << shift;
    gCfg[(o & ~3u) / 4] = (gCfg[(o & ~3u) / 4] & ~mask) | ((v << shift) & mask);
    return 0;
}
int pMsiTarget(uint32_t *lo, uint32_t *hi, uint32_t *data) {*lo = (uint32_t)gArena.iova; *hi = (uint32_t)(gArena.iova >> 32); *data = 0; return 0;}

// The network side (WifiNet.cpp): its classes are registered with the kernel's module like the other late classes, then the interface is made.
extern "C" void native_wifi_logf(int level, const char *fmt, ...) {
    char line[160]; va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof line, fmt, ap); va_end(ap);
    pLog(level, line);
}
void netStart() {
    struct nw_wifi_status status; nw_wifi_status_get(&status);
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {native_wifi_logf(3, "wifi: network classes: no module handle"); return;}
    native_wifi_net_static_init();
    const OSReturn r = OSMetaClass::postModLoad(handle);
    if (r) {native_wifi_logf(3, "wifi: network classes not registered: %d", (int)r); return;}
    sheng_wifi_net_start(status.mac);
}

// This board has no endpoint SPI. WIFI.md lists 140/149/150/151/152 on the root complex,
// and those are not the endpoint's MSI. nw_wifi_step raises the driver's handlers.
// The endpoint still writes MSI into the arena's first page, which arenaInit keeps free.
volatile uint32_t gMsiWake;
}  // namespace
// The network side calls this after it queued a frame: the WiFi thread sends it now instead of at its next wake.
extern "C" void sheng_wifi_wake(void) {gMsiWake = 1; thread_wakeup(event_t(&gMsiWake));}
namespace {

void wifiStack(volatile uint32_t *m, uint64_t barBytes) {
    volatile uint32_t *st = native_wifi_mhi;
    if (!arenaInit()) {st[0] = (uint32_t)-11; st[1] = (uint32_t)gDma.err; return;}
    pLog(6, "wifi: DMA window 18000000/2000000 cached");
    if (!findCaps()) {st[0] = (uint32_t)-12; return;}
    static const struct lx_platform plat = {pNow, pSleep, pPoll, pLog, pHalt, hAlloc, hFree, pDmaAlloc, pDmaFree, pDmaMap, pDmaSync, pFwGet};
    static struct nw_board board;
    gNwBar = m;
    board.bar = const_cast<uint32_t *>(m); board.bar_len = barBytes; board.device_id = (uint16_t)(gCfg[0] >> 16); board.msi_cap = (uint16_t)gMsiCap; board.irq_base = 1000;
    board.cfg_read = pCfgRead; board.cfg_write = pCfgWrite; board.msi_target = pMsiTarget; board.map_phys = nullptr;
    // the board's address window for the endpoint to reach sleep clock and sibling blocks is already set up by the earlier stages
    st[0] = 20;
    const int r = nw_wifi_start(&plat, &board);
    st[0] = r ? (uint32_t)(0x80000000u | (uint32_t)(-r & 0xffff)) : 21;
    bool netDone = false;
    // Poll. nw_wifi_step marks every registered handler pending, then sleeps 1 ms while idle.
    pLog(6, "wifi: polling, no SPI");
    for (uint64_t n = 0;; ++n) {
        const int busy = nw_wifi_step();
        if (!netDone && nw_wifi_stats.stage >= 4) {netDone = true; netStart();}
        if (!busy) IOSleep(1);
        if ((n & 63) == 0) st[1] = nw_wifi_stats.stage | nw_wifi_stats.scans << 8 | nw_wifi_stats.bss_seen << 16;
    }
}

// The host's half of the event rings (mhi_process_ctrl_ev_ring): the device advances the ring context's read pointer past each event it posts; the host takes
// them and tells the device the ring is free again by moving the write pointer to just behind that and ringing the ring's doorbell. Returns how many rings moved.
unsigned eventAck(volatile uint32_t *m) {
    constexpr uint32_t kCtx = 44, kChannels = 128;
    unsigned moved = 0;
    const uint32_t erdb = m[0x20 / 4];
    static uint64_t acked[2];
    for (unsigned r = 0; r < 2; ++r) {
        uint8_t *ctx = gCtxVa + kChannels * kCtx + r * kCtx;
        uint64_t base, len, rp;
        memcpy(&base, ctx + 12, 8); memcpy(&len, ctx + 20, 8); memcpy(&rp, ctx + 28, 8);
        if (rp == acked[r] || rp < base || rp >= base + len) continue;
        acked[r] = rp;
        const uint64_t wp = rp == base ? base + len - 16 : rp - 16;
        memcpy(ctx + 36, &wp, 8);
        __asm__ volatile("dc civac, %0\n dsb sy" :: "r"(ctx + 36) : "memory");
        m[(erdb + 8 * r) / 4] = (uint32_t)wp; m[(erdb + 8 * r + 4) / 4] = (uint32_t)(wp >> 32);
        ++moved;
    }
    return moved;
}

// What the device left for the host: the first events of both event rings, the event ring contexts' read and write pointers, the MSI sink and the BHIe block.
void eventSnapshot(volatile uint32_t *m, uint32_t slot) {
    volatile uint32_t *o = native_wifi_ev + slot * 24;
    if (!gCtxVa || !gEvVa[0]) return;
    for (unsigned i = 0; i < 8; ++i) {uint32_t w; memcpy(&w, gEvVa[0] + 4 * i, 4); o[i] = w;}      // ring 0: two entries
    for (unsigned i = 0; i < 4; ++i) {uint32_t w; memcpy(&w, gEvVa[1] + 4 * i, 4); o[8 + i] = w;}   // ring 1: one entry
    for (unsigned r = 0; r < 2; ++r) {
        uint32_t rp, wp;
        memcpy(&rp, gCtxVa + 128 * 44 + r * 44 + 28, 4); memcpy(&wp, gCtxVa + 128 * 44 + r * 44 + 36, 4);
        o[12 + 2 * r] = rp; o[13 + 2 * r] = wp;
    }
    for (unsigned i = 0; i < 2; ++i) {uint32_t w; memcpy(&w, gSinkVa + 4 * i, 4); o[16 + i] = w;}
    const uint32_t bhie = m[0x2c / 4];
    for (unsigned i = 0; i < 6; ++i) o[18 + i] = m[(bhie + 0x2c + 4 * i + (i >= 4 ? 4 : 0)) / 4];   // ADDR_LOW, ADDR_HIGH, SIZE, (0x38), DB, STATUS
}

void mhiBring(volatile uint32_t *m, uint64_t barBytes) {
    volatile uint32_t *st = native_wifi_mhi;
    const uint64_t t0 = mach_absolute_time();
    auto elapsed = [&]() {uint64_t ns = 0; absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns); return (uint32_t)(ns / 1000000);};
    static Dma seg[kMaxSegments], sbl, table, ctx;
    // IOVA 0x18000000 fits in the endpoint's 32-bit DMA mask. 32 MiB is more than the 7 MiB the load asks for.
    const int win = dmaWindow();
    if (win) {st[0] = (uint32_t)-4; st[1] = (uint32_t)win; return;}
    const bool low = gDma.bytes >= 7u << 20 && gDma.iova <= 0xffffffffull;
    native_wifi_mhi_ms[7] = low ? (uint32_t)gDma.iova : 0xdead0000u;
    for (uint32_t i = 0; i < kMaxSegments; ++i)
        if (!dmaAlloc(seg[i], kSegment)) {st[0] = (uint32_t)-3; st[10] = 100 + i; return;}
    sbl = seg[0];   // the boot loader is the first 512 KiB of the image, consecutive segments
    if (!dmaAlloc(table, 4096)) {st[0] = (uint32_t)-3; st[10] = 201; return;}
    if (!dmaAlloc(ctx, 32 * 1024)) {st[0] = (uint32_t)-3; st[10] = 202; return;}
    st[15] = (uint32_t)(seg[0].pa >> 12);   // where the buffers landed (page numbers), kept until the registers are read after the load
    st[8] = (uint32_t)(sbl.pa >> 12); st[9] = (uint32_t)(table.pa >> 12); st[12] = (uint32_t)(ctx.pa >> 12);
    st[0] = 1; native_wifi_mhi_ms[0] = elapsed();
    Fw fw = {};
    if (!findFirmware(fw)) {st[0] = (uint32_t)-1; return;}
    native_wifi_mhi_ms[1] = elapsed();
    st[3] = fw.amssSize;
    const uint32_t segments = (fw.amssSize + kSegment - 1) / kSegment;
    st[4] = segments;
    if (segments > kMaxSegments) {st[0] = (uint32_t)-2; return;}
    constexpr size_t kChunk = 128 * 1024;
    uint8_t *bounce = static_cast<uint8_t *>(IOMallocData(kChunk));
    if (!bounce) {st[0] = (uint32_t)-3; st[10] = 2; return;}
    for (uint32_t done = 0; done < fw.amssSize; done += kChunk) {
        const uint32_t left = fw.amssSize - done, bytes = left < kChunk ? left : kChunk;
        if (!native_ufs_part_read(fw.lun, fw.first + (fw.amssOffset + done) / 4096, (bytes + 4095) / 4096, bounce)) {IOFreeData(bounce, kChunk); st[0] = (uint32_t)-2; return;}
        for (uint32_t off = 0; off < bytes; off += kSegment) memcpy(seg[(done + off) / kSegment].va, bounce + off, bytes - off < kSegment ? bytes - off : kSegment);
    }
    IOFreeData(bounce, kChunk);
    st[0] = 2; native_wifi_mhi_ms[2] = elapsed();
    uint64_t addresses[kMaxSegments];
    for (uint32_t i = 0; i < segments; ++i) addresses[i] = seg[i].pa;
    const uint32_t entries = mhi_bhie_table(reinterpret_cast<MhiBhieEntry *>(table.va), 4096 / sizeof(MhiBhieEntry), addresses, fw.amssSize, kSegment);
    if (!entries) {st[0] = (uint32_t)-3; st[10] = 3; return;}
    {
        uint32_t sum = 0;
        for (uint32_t off = 0; off < kSbl; off += 4) {uint32_t w; memcpy(&w, seg[0].va + off, 4); sum += w;}
        uint32_t first; memcpy(&first, seg[0].va, 4);
        native_wifi_cfg[28] = sum; native_wifi_cfg[29] = first; native_wifi_cfg[31] = fw.amssSize;
    }
    for (uint32_t off = 0; off < gDmaUsed; off += 64) __asm__ volatile("dc civac, %0" :: "r"(gDma.va + off) : "memory");   // the endpoint reads DRAM
    __asm__ volatile("dsb sy" ::: "memory");
    st[0] = 3;
    // slot | bank << 16, bit 31 set. The SMR valid bit itself depends on CR0 and is written in smmuProgram.
    st[1] = gDma.stream;
    if (!(st[1] & 0x80000000u)) {st[0] = (uint32_t)-4; return;}
    st[0] = 4; native_wifi_mhi_ms[3] = elapsed();
    static Dma sink;
    if (!dmaAlloc(sink, 4096)) {st[0] = (uint32_t)-3; st[10] = 203; return;}
    gSinkVa = sink.va;
    uint32_t level0 = 0;
    PE_parse_boot_argn("sheng-wifi", &level0, sizeof level0);
    if (level0 >= 4) {chipPowerUp(m); endpointLinkSetup(sink.pa);}
    BarIo io = {m};
    MhiBoot boot = {};
    MhiIo mio = {barRead, barWrite, &io, (uint32_t)barBytes};
    if (mhi_boot_open(&boot, &mio)) {st[0] = (uint32_t)-5; return;}
    // the log writer needs a while to start; the next step is the one that can end the boot, so let it write a few slots first
    st[0] = 4; st[13] = elapsed();
    uint32_t level = 0;
    if (!PE_parse_boot_argn("sheng-wifi", &level, sizeof level) || level < 3 || !low) return;   // the load itself starts with sheng-wifi=3
    IOSleep(8000);
    programContexts(m, ctx, barBytes);
    for (uint32_t off = 0; off < gDmaUsed; off += 64) __asm__ volatile("dc civac, %0" :: "r"(gDma.va + off) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");   // the driver for this chip sets the device's context arrays up before the boot loader is started
    gTraceWrites = 1;
    gSkipReg = level >= 4 ? 0 : m[0x28 / 4] + MHI_BHI_IMGTXDB;   // sheng-wifi=3 sets everything up but leaves the doorbell alone
    st[14] = 1;   // about to write the boot loader's address and size into BHI
    uint32_t phase = mhi_boot_begin(&boot, 0, sbl.pa, kSbl < fw.amssSize ? kSbl : fw.amssSize, table.pa, entries * (uint32_t)sizeof(MhiBhieEntry), 0x5eed);
    gTraceWrites = 0;
    // the doorbell is written: stay off the endpoint for a while, each marker telling the log how far the boot got without touching it
    for (uint32_t quiet = 0; quiet < 3; ++quiet) {st[14] = 0x200 + quiet; IOSleep(2500);}
    st[14] = 0x203;
    st[14] = 2; st[0] = 5; native_wifi_mhi_ms[4] = elapsed();
    // one step per millisecond; every phase gets two seconds
    uint32_t last = phase, since = 0;
    for (unsigned i = 0; i < 20000 && phase != MhiDone && phase != MhiFailed; ++i) {   // five milliseconds a step, twenty seconds a phase (the driver for this chip waits as long)
        if (phase == MhiNeedContexts) {programContexts(m, ctx, barBytes); /* the boot loader resets the MHI registers when it starts: the contexts go in again once the device reports READY */ st[0] = 6; native_wifi_mhi_ms[5] = elapsed(); phase = mhi_boot_contexts_done(&boot);}
        else {
            if (i % 100 == 0 && (m[0x48 / 4] == 0xffffffffu)) {   // not answering: look at the link, put a cleared BAR back
                if (!native_wifi_cfg[16]) {linkSnapshot(native_wifi_cfg); native_wifi_cfg[16] = i + 1;}
                linkRecover();
                native_wifi_cfg[17] = gRestored;
            }
            phase = mhi_boot_step(&boot);
        }
        st[5] = boot.phase | boot.error << 8 | boot.failed_phase << 16; st[6] = boot.ee; st[7] = boot.state; st[14] = 100 + i;   // live, for a log slot written just before a reset
        if (phase != MhiNeedContexts && phase != MhiDone && phase != MhiFailed) native_wifi_chip[13] += eventAck(m);
        if (phase == MhiImageLoading && since == 600) eventSnapshot(m, 0);
        if (phase != last) {last = phase; since = 0;}
        else if (++since > 4000) {phase = mhi_boot_timeout(&boot); last = phase; since = 0;}
        IOSleep(5);
    }
    if (phase == MhiDone) st[0] = 7;
    eventSnapshot(m, 1);
    IOSleep(3000);
    linkSnapshot(native_wifi_cfg + 8);
    {uint32_t sum = 0; for (uint32_t off = 0; off < kSbl; off += 4) {uint32_t w; memcpy(&w, seg[0].va + off, 4); sum += w;} native_wifi_cfg[30] = sum;}
    if (gDma.cb) native_wifi_cfg[20] = smmuRd(gDma.cb, 0x58);
    st[5] = boot.phase | boot.error << 8 | boot.failed_phase << 16;
    st[6] = mhi_boot_ee(&boot); st[7] = mhi_boot_state(&boot);
    const uint32_t bhi = m[0x28 / 4], bhie = m[0x2c / 4];
    st[8] = m[(bhi + 0x2c) / 4]; st[9] = m[(bhi + 0x30) / 4];
    st[10] = m[(bhi + 0x34) / 4]; st[11] = m[(bhi + 0x38) / 4]; st[12] = m[(bhi + 0x3c) / 4];
    st[13] = m[0x48 / 4]; st[14] = elapsed(); st[15] = m[(bhie + 0x44) / 4]; native_wifi_mhi_ms[6] = st[14];
}

// Stage 4: forward the window to bus 1 (the root port's memory base and limit, outbound iATU region 1), give the endpoint's BAR0 an address in it, turn
// memory decoding and bus mastering on and read the MHI registers the BAR starts with. The endpoint's configuration space is reached through
// the 1 MiB window iATU region 0 maps to bus 1 (c), the root port's through the DBI (d), the iATU through a (unrolled, 0x200 per region).
void endpointProbe(volatile uint32_t *a, volatile uint32_t *c, volatile uint32_t *d) {
    gCfg = c; gDbi = d;
    volatile uint32_t *ep = native_wifi_ep;
    // region 1: memory, CPU window to the same PCI addresses
    volatile uint32_t *r1 = a + 0x200 / 4;
    r1[0x08 / 4] = uint32_t(kMemBase);
    r1[0x0c / 4] = uint32_t(kMemBase >> 32);
    r1[0x10 / 4] = uint32_t(kMemBase + kMemSize - 1);
    r1[0x20 / 4] = uint32_t((kMemBase + kMemSize - 1) >> 32);
    r1[0x14 / 4] = uint32_t(kMemBase);
    r1[0x18 / 4] = uint32_t(kMemBase >> 32);
    r1[0x00 / 4] = 0x0;
    r1[0x04 / 4] = 1u << 31;
    for (unsigned i = 0; i < 5 && !(r1[0x04 / 4] & (1u << 31)); ++i) IODelay(10);
    // type 1 header memory base (bits 15:4 = address bits 31:20) and limit (31:20)
    d[0x20 / 4] = uint32_t(((kMemBase + kMemSize - 1) & 0xfff00000u) | ((kMemBase >> 16) & 0xfff0u));
    ep[4] = d[0x20 / 4];
    ep[0] = 1;
    // BAR0: size it, put it at the start of the window
    const uint32_t saved = c[0x10 / 4];
    c[0x10 / 4] = 0xffffffffu;
    const uint32_t mask = c[0x10 / 4];
    c[0x10 / 4] = saved;
    ep[1] = mask;
    const uint32_t size = ~(mask & ~0xfu) + 1;
    if (!(mask & ~0xfu) || !size || size > kMemSize) return;
    // a BAR sits at an address aligned to its size: the first such address of the window
    const uint32_t address = uint32_t((kMemBase + size - 1) & ~uint64_t(size - 1));
    ep[1] = mask;
    c[0x10 / 4] = address | (mask & 0xfu);
    if ((mask & 6u) == 4u) c[0x14 / 4] = 0;
    c[0x04 / 4] = (c[0x04 / 4] & 0xffffu) | 0x6u;
    gBar0 = address | (mask & 0xfu);
    ep[2] = c[0x10 / 4];
    ep[3] = c[0x04 / 4];
    ep[0] = 2;
    Window bar = {};
    if (!mapWindow(bar, address, size < 0x200000 ? size : 0x200000)) return;
    volatile uint32_t *m = bar.va;
    for (unsigned i = 0; i < 7; ++i) {
        static const uint32_t regs[7] = {0x00, 0x08, 0x10, 0x28, 0x2c, 0x38, 0x48};
        ep[5 + i] = m[regs[i] / 4];
    }
    ep[0] = 3;
    uint32_t level5 = 0;
    PE_parse_boot_argn("sheng-wifi", &level5, sizeof level5);
    if (level5 >= 5) {ep[0] = 4; wifiStack(m, size < 0x200000 ? size : 0x200000); release(bar); return;}
    const uint32_t bhi = m[0x28 / 4];
    if (bhi && bhi < 0x20000 - 0x40) {
        ep[12] = m[bhi / 4 + 1] | m[bhi / 4] << 16;
        ep[13] = m[(bhi + 0x28) / 4];
        ep[14] = m[(bhi + 0x2c) / 4];
        ep[15] = m[(bhi + 0x30) / 4];
        ep[0] = 4;
        mhiBring(m, size < 0x20000 ? size : 0x20000);
    }
    release(bar);
}

// Stage 2; the step reached stays in native_wifi_stats[0], a negative value names the step that failed.
void bringUp(Window &gcc, Pins const &pins) {
    volatile uint32_t *g = gcc.va;
    pmicPrepare();
    const int rails = powerRails();
    IOSleep(20);
    if (gPmic.obs.va) {pmicSurvey(true); pmicFind1800(gPmic, native_wifi_ldo + 12);}
    if (rails) {native_wifi_stats[0] = -9; return;}
    IOSleep(5);
    // PERST# stays low. rails and rf_clk already ran. cnss power.c WLAN_ENABLE_DELAY is 1000 us,
    // once, before the RC clocks (WIFI.md rule 21).
    setPin(pins, kPerst, 1u | 1u << 9, 0);
    setPin(pins, kClkReq, 3u | 1u << 2, -1);
    setPin(pins, kWlanEn, 1u | 3u << 6 | 1u << 9, 1);
    IODelay(1000);
    native_wifi_stats[10] = pinWord(pins);
    // what the pin registers hold now (a write TZ or the hypervisor drops reads back as written-nothing)
    native_wifi_stats[17] = pinSnap(pins, kWlanEn);
    native_wifi_stats[18] = pinSnap(pins, kPerst);
    native_wifi_stats[19] = pinSnap(pins, kClkReq);
    native_wifi_stats[0] = 10;

    // gcc-canoe.c: 0x5214c bit 0 collapses the root complex, bit 2 the PHY. sy_gdsc_on polls PWR_UP_DONE.
    g[kCollapse / 4] &= ~(1u << 0 | 1u << 2);
    auto powerUp = [&](uint32_t gdscr) {
        for (unsigned i = 0; i < 50; ++i) {if (g[(gdscr + 4) / 4] & (1u << 16)) return true; IODelay(2000);}
        g[gdscr / 4] &= ~1u;
        for (unsigned i = 0; i < 50; ++i) {if (g[(gdscr + 4) / 4] & (1u << 16)) return true; IODelay(2000);}
        return false;
    };
    if (!powerUp(kGdsc) || !powerUp(kPhyGdsc)) {native_wifi_stats[0] = -11; return;}
    uint32_t power = (g[kGdsc / 4] >> 31) | (g[kPhyGdsc / 4] >> 31) << 1;
    native_wifi_stats[11] = power;
    native_wifi_stats[0] = 11;

    // sy_clocks_on: aux at TCXO, rchng at 100 MHz, clkref, pipe parent 0, then the votes. Pipe bit 4 waits for the PHY.
    if (!rcgSet(g, kAuxRcg, 0) || !rcgSet(g, kRchngRcg, 6u << 8 | 2u)) {native_wifi_stats[0] = -12; return;}
    Window clkref = {};
    if (!mapWindow(clkref, kTcsrClkref, 0x1000)) {native_wifi_stats[0] = -12; return;}
    clkref.va[kTcsrClkrefEn / 4] |= 1u;
    release(clkref);
    g[kPipeMux / 4] &= ~3u;
    g[kVote8 / 4] |= kVote8Bits;
    g[kVote0 / 4] |= kVote0Bits;
    g[kVote18 / 4] |= kVote18Bits;
    IODelay(200);
    uint32_t off = 0;
    for (unsigned i = 0; i < sizeof kBranches / sizeof kBranches[0]; ++i)
        if (g[kBranches[i] / 4] & (1u << 31)) off |= 1u << i;
    native_wifi_stats[11] = power | off << 8;
    if (off & ~(1u << 4)) {native_wifi_stats[0] = -12; return;}   // the pipe branch runs only once the PHY drives it
    pulseReset(g, kPcieBcr);
    pulseReset(g, kPhyBcr);
    native_wifi_stats[0] = 12;

    // qcom_pcie_init_2_7_0 after its resets, before the PHY powers on (qcom_pcie_host_init): PHY_CTRL's test power-down bit holds the PHY
    Window parf = {}, dbi = {};
    if (!mapWindow(parf, kParf, 0x3000) || !mapWindow(dbi, kDbi, 0x1000)) {release(parf); native_wifi_stats[0] = -13; return;}
    volatile uint32_t *f = parf.va, *d = dbi.va;
    f[kParfDeviceType / 4] = 4u;
    f[kParfPhyCtrl / 4] &= ~1u;
    f[kParfDbiBase / 4] = uint32_t(kDbi);
    f[kParfDbiBaseHi / 4] = uint32_t(kDbi >> 32);
    f[kParfAtuBase / 4] = uint32_t(kAtu);
    f[kParfAtuBaseHi / 4] = uint32_t(kAtu >> 32);
    f[kParfSlvSpace / 4] = 0xffffffffu;       // WIFI.md §10.3: U64_MAX, not the 2 GiB mirror
    f[kParfSlvSpaceHi / 4] = 0xffffffffu;
    f[kParfSysCtrl / 4] &= ~(1u << 29);
    f[kParfMhiClockReset / 4] |= 1u << 4;
    f[kParfPmCtrl / 4] &= ~(1u << 5);
    f[kParfAxiHalt / 4] |= 1u << 31;
    native_wifi_stats[0] = 13;

    // sy_phy_start: the 199-word sequence, then the pipe clock, then PHYSTATUS at 0x414 bit 7.
    Window phy = {};
    if (!mapWindow(phy, kPhy, 0x2000)) {release(dbi); release(parf); native_wifi_stats[0] = -14; return;}
    volatile uint32_t *p = phy.va;
    using namespace wifi_phy;
    for (const Init &e : kSeq) p[e.offset / 4] = e.value;
    IODelay(1000);
    g[kPipeMux / 4] &= ~3u;
    g[kVote8 / 4] |= 1u << 4;
    g[kPhyNocsrBcr / 4] |= 1u;
    IODelay(1000);
    g[kPhyNocsrBcr / 4] &= ~1u;
    unsigned polls = 0;
    const bool ready = waitClear(&p[kPhyStatus / 4], kPhyStatusBit, 2000, &polls);
    native_wifi_stats[12] = (p[kPhyStatus / 4] & 0xffffu) | polls << 16;
    native_wifi_pmic[17] = g[kBranches[4] / 4];
    release(phy);
    if (!ready) {release(dbi); release(parf); native_wifi_stats[0] = -14; return;}

    // qcom_pcie_config_sid_1_9_0: the controller tells the SMMU which stream a requester's DMA belongs to from a hash table of requester IDs
    // (0x2000, 256 words: BDF << 16 | stream offset << 8 | next) once its bypass bit (0x2c00 bit 0) is off. The iommu-map is requester 0 -> stream 0x1400 and
    // requester 0x100 (bus 1, the endpoint) -> 0x1401; the hash is a CRC-8 (polynomial 7, MSB first) of the requester ID's two bytes, high one first.
    {
        volatile uint32_t *f = parf.va;
        for (unsigned i = 0; i < 256; ++i) f[(0x2000 + 4 * i) / 4] = 0;
        f[0x2c00 / 4] &= ~1u;
        auto crc8 = [](uint16_t bdf) {
            uint8_t crc = 0;
            const uint8_t bytes[2] = {(uint8_t)(bdf >> 8), (uint8_t)bdf};
            for (uint8_t byte : bytes) {
                crc ^= byte;
                for (int bit = 0; bit < 8; ++bit) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 7) : (uint8_t)(crc << 1);
            }
            return crc;
        };
        const uint8_t hash = crc8(0x100);
        f[(0x2000 + 4 * hash) / 4] = 0x100u << 16 | 1u << 8;   // requester 0 hashes to entry 0 with offset 0: the zeroed word already says that
        __asm__ volatile("dsb sy" ::: "memory");
        native_wifi_pon[10] = hash | f[0x2c00 / 4] << 8 | (f[(0x2000 + 4 * hash) / 4] != 0) << 31;   // hash, bypass register, entry written
    }

    // qcom_pcie_post_init_2_7_0 and the root port's bus numbers and command (dw_pcie_setup_rc)
    d[kBusNumbers / 4] = 0x00010100;   // primary 0, secondary 1, subordinate 1
    d[kCommand / 4] |= 0x6u;           // memory space and bus master
    native_wifi_stats[7] = d[0];
    native_wifi_stats[0] = 14;

    // qcom_ep_reset_deassert: PERST# low for at least 100 ms, then high, then the LTSSM (qcom_pcie_2_3_2_ltssm_enable)
    IOSleep(100);
    setPin(pins, kPerst, 1u | 1u << 9, 1);
    IODelay(1000);
    native_wifi_stats[0] = 15;
    f[kParfLtssm / 4] |= 1u << 8;
    const uint64_t start = mach_absolute_time();
    unsigned ms = 0;
    for (; ms < 1000; ++ms) {
        if ((d[kPortDebug1 / 4] & (1u << 4)) && (d[kPortDebug0 / 4] & 0x3f) == 0x11) break;
        if (ms == 20) {native_wifi_pmic[19] = d[kPortDebug0 / 4]; native_wifi_pmic[21] = d[kPortDebug1 / 4];}
        IOSleep(1);
    }
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
    native_wifi_stats[16] = uint32_t(ns / 1000000);
    native_wifi_pmic[18] = pinSnap(pins, kPerst);
    native_wifi_pmic[20] = g[kPipeMux / 4];
    native_wifi_stats[13] = d[kPortDebug0 / 4];
    native_wifi_stats[14] = d[kPortDebug1 / 4];
    native_wifi_stats[6] = f[kParfLtssm / 4];
    native_wifi_stats[8] = d[kLinkControl / 4];
    native_wifi_stats[9] = f[kParfPhyCtrl / 4];
    if (ms == 1000) {release(dbi); release(parf); native_wifi_stats[0] = -16; return;}
    native_wifi_stats[0] = 16;

    // outbound iATU region 0 (unrolled at the ATU base) turns the 1 MiB config window into type 0 accesses to bus 1 device 0
    Window atu = {}, config = {};
    if (mapWindow(atu, kAtu, 0x10000) && mapWindow(config, kConfig, 0x1000)) {
        volatile uint32_t *a = atu.va;
        a[0x08 / 4] = uint32_t(kConfig);
        a[0x0c / 4] = uint32_t(kConfig >> 32);
        a[0x10 / 4] = uint32_t(kConfig + 0xfffff);
        a[0x20 / 4] = uint32_t((kConfig + 0xfffff) >> 32);
        a[0x14 / 4] = 1u << 24;
        a[0x18 / 4] = 0;
        a[0x00 / 4] = 0x4;
        a[0x04 / 4] = 1u << 31;
        for (unsigned i = 0; i < 5 && !(a[0x04 / 4] & (1u << 31)); ++i) IODelay(10);
        native_wifi_stats[15] = config.va[0];
        native_wifi_stats[0] = 17;
        if ((config.va[0] & 0xffff) == 0x17cb) endpointProbe(a, config.va, d);
    } else native_wifi_stats[0] = -17;
    release(config); release(atu); release(dbi); release(parf);
}

void wifiThread(void *arg, wait_result_t) {
    thread_set_thread_name(current_thread(), "sheng-wifi");
    publishMac();
    Window gcc = {}, tlmmLow = {}, tlmmHi = {}, parf = {}, dbi = {};
    if (!mapWindow(gcc, kGcc, 0x80000) || !mapWindow(tlmmLow, kTlmmLow, 0x4000) || !mapWindow(tlmmHi, kTlmmHi, 0x3000)) {
        native_wifi_stats[0] = -1; release(gcc); release(tlmmLow); release(tlmmHi); return;
    }
    Pins pins = {tlmmLow.va, tlmmHi.va};
    native_wifi_stats[0] = 1;
    native_wifi_stats[1] = gcc.va[kVote8 / 4];
    native_wifi_stats[2] = gcc.va[kVote0 / 4];
    const uint32_t gdsc = gcc.va[kGdsc / 4];
    native_wifi_stats[3] = gdsc;
    uint32_t off = 0;
    for (unsigned i = 0; i < sizeof kBranches / sizeof kBranches[0]; ++i)
        if (gcc.va[kBranches[i] / 4] & (1u << 31)) off |= 1u << i;
    native_wifi_stats[4] = off;
    native_wifi_stats[5] = pinWord(pins);
    native_wifi_stats[0] = 2;
    if (reinterpret_cast<uintptr_t>(arg) >= 2) {bringUp(gcc, pins); release(tlmmHi); release(tlmmLow); release(gcc); return;}
    if ((gdsc & (1u << 31)) && !(off & kNeeded) && mapWindow(parf, kParf, 0x3000) && mapWindow(dbi, kDbi, 0x1000)) {
        native_wifi_stats[6] = parf.va[kParfLtssm / 4];
        native_wifi_stats[9] = parf.va[kParfPhyCtrl / 4];
        native_wifi_stats[7] = dbi.va[0];
        native_wifi_stats[8] = dbi.va[kLinkControl / 4];
        native_wifi_stats[0] = 3;
    }
    release(dbi); release(parf); release(tlmmHi); release(tlmmLow); release(gcc);
}
}  // namespace

// A file of the firmware container by name, for the board's own test hooks (not the driver).
extern "C" int sheng_wifi_file(const char *name, const uint8_t **data, size_t *size) {return pFwGet(name, data, size);}

extern "C" void sheng_wifi_probe(void) {
    unsigned stage = 0;
    if (!PE_parse_boot_argn("sheng-wifi", &stage, sizeof stage) || stage < 1 || native_wifi_stats[0]) return;
    thread_t thread = nullptr;
    if (kernel_thread_start(wifiThread, reinterpret_cast<void *>(uintptr_t(stage)), &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else native_wifi_stats[0] = -7;
}
