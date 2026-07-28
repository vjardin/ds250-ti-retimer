// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2026 Free Mobile — Vincent Jardin
 *
 * ds250 — TI DS250DF410 / DS250DF810 retimer monitor + configuration tool.
 *
 * Run: be root, check the device's SMBus address is that is strapped per board
 * via ADDR[1:0]:
 *   ds250 /dev/i2c-26 0x18 probe
 *   ds250 /dev/i2c-26 0x18 status              # every channel
 *   ds250 /dev/i2c-26 0x18 heo -c 2            # quick HEO/VEO
 *   ds250 /dev/i2c-26 0x18 eye -c 2 [-r 1] [-o eye.csv]
 *   ds250 /dev/i2c-26 0x18 dump -c 0 0x00 0x2f
 *   ds250 /dev/i2c-26 0x18 dump -g             # globals 0xEF..0xFF
 *   ds250 /dev/i2c-26 0x18 rd 0x01 -c 3
 *   ds250 /dev/i2c-26 0x18 wr 0x03 0x55 -c 3   # raw write (careful)
 *   ds250 /dev/i2c-26 0x18 reset -c 3 vco
 */

#define _DEFAULT_SOURCE

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define EXIT_USAGE 2

/* global registers */
#define REG_CHAN_CONFIG_ID 0xEF
#define REG_VERSION        0xF0
#define REG_DEVICE_ID      0xF1
#define REG_CHAN_SHARE_VER 0xF3
#define REG_CHAN_SEL       0xFC
#define REG_VENDOR_ID      0xFE
#define REG_PAGE_CTRL      0xFF

#define VENDOR_ID_TI       0x03

/* channel registers */
#define CH_RESET           0x00
#define CH_STATUS          0x01
#define CH_EQ_BST          0x03
#define CH_EOM_VRANGE      0x11    /* [7:6] range, [5] EOM power-down */
#define CH_EOM_CTRL        0x24    /* [7] fast EOM, [0] start (self-clear) */
#define CH_EOM_MSB         0x25
#define CH_EOM_LSB         0x26
#define CH_HEO             0x27
#define CH_VEO             0x28
#define CH_DFE_CFG         0x2C    /* [6] VEO_SCALE (SM controls range) */
#define CH_EQ_OV           0x2D    /* [3] REG_EQ_BST_OV */
#define CH_EOM_LOCKMON     0x67    /* [5] EOM lock monitoring */

/* channel registers -- equalization / CDR / PRBS (Table 10/11) */
#define CH_DES_PD          0x0D    /* [7] deserializer (PRBS checker) power-down */
#define CH_DFE_POL         0x11    /* [3] tap2 .. [0] tap5 polarity */
#define CH_DFE_TAP1        0x12    /* [7] tap1 pol, [4:0] tap1 weight */
#define CH_DFE_FORCE       0x15    /* [7] DFE_FORCE_EN (manual taps) */
#define CH_PFD_CFG         0x1E    /* [4] SER_EN, [3] DFE_PD, [1] EN_PARTIAL_DFE */
#define CH_DFE_WT45        0x20    /* [7:4] tap5 wt, [3:0] tap4 wt */
#define CH_DFE_WT23        0x21    /* [7:4] tap3 wt, [3:0] tap2 wt */
#define CH_PRBS_SEL_MSB    0x2E    /* [2] PRBS_PATTERN_SEL[2] */
#define CH_RATE            0x2F    /* [6:4] RATE, [3] INDEX_OV, [0] CTLE_ADAPT (RWSC) */
#define CH_PRBS_CFG        0x30    /* [3] PRBS_EN_DIG_CLK, [1:0] PATTERN_SEL[1:0] */
#define CH_ADAPT           0x31    /* [6:5] ADAPT_MODE, [4:3] EQ_SM_FOM */
#define CH_CTLE_STATUS     0x37
#define CH_FIR_C0          0x3D    /* [7] EN_FIR_CURSOR, [6] sign, [4:0] main */
#define CH_FIR_CN1         0x3E    /* [7] FIR_PD_TX, [6] sign, [3:0] pre */
#define CH_FIR_CP1         0x3F    /* [6] sign, [3:0] post */
#define CH_LINK_STATUS     0x78    /* [5] SD_STATUS, [4] CDR_LOCK_STATUS (live) */
#define CH_PRBS_EN         0x79    /* [6] PRBS_CHKR_EN, [5] PRBS_GEN_EN */
#define CH_PRBS_CHK        0x82    /* [7] freeze, [6] rst cnt, [5] patt ov, [4:2] patt */
#define CH_PRBS_ERR_HI     0x83    /* [2:0] = PRBS_ERR_CNT[10:8] */
#define CH_PRBS_ERR_LO     0x84    /* PRBS_ERR_CNT[7:0] */
#define CH_PSTLCK_MUX      0xA5    /* [7:5] TX output while locked (001=retimed, 100=gen) */

/* shared registers, reachable with 0xFF[0]=0 (Table 9) */
#define SH_SMBUS_ADDR      0x00    /* [7:4] strap offset from 0x18 */
#define SH_EE_STAT         0x05    /* [4] EEPROM_READ_DONE */
#define SH_INT             0x08    /* [3:0] interrupt from ch3..ch0 of the quad */
#define SH_REFCLK          0x0B    /* [6] REFCLK_DET (25 MHz CAL_CLK_IN) */
#define SH_EECFG           0x11    /* [7:6] 10=EEPROM ok 01=fail, [5:0] attempts */

#define EYE_DIM            64
#define EYE_JUNK_WORDS     4

struct dev {
    int      fd;
    uint16_t addr;
    int      nch;       /* 4 or 8 */
    int      cur_ch;    /* currently selected channel page, -1 unknown */
};

static int xfer(struct dev *d, struct i2c_msg *msgs, int n)
{
    struct i2c_rdwr_ioctl_data ic = { .msgs = msgs, .nmsgs = (uint32_t)n };
    return ioctl(d->fd, I2C_RDWR, &ic) < 0 ? -errno : 0;
}

static int rd8(struct dev *d, uint8_t reg)
{
    uint8_t v = 0;
    struct i2c_msg m[2] = {
        { .addr = d->addr, .flags = 0,        .len = 1, .buf = &reg },
        { .addr = d->addr, .flags = I2C_M_RD, .len = 1, .buf = &v },
    };
    int rc = xfer(d, m, 2);
    return rc ? rc : v;
}

static int wr8(struct dev *d, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    struct i2c_msg m = { .addr = d->addr, .flags = 0, .len = 2, .buf = b };
    return xfer(d, &m, 1);
}

/* read-modify-write of masked bits in the current page */
static int rmw8(struct dev *d, uint8_t reg, uint8_t mask, uint8_t val)
{
    int v = rd8(d, reg);
    if (v < 0)
        return v;
    return wr8(d, reg, ((uint8_t)v & ~mask) | (val & mask));
}

/* select one channel's register page (reads require a single channel) */
static int sel_ch(struct dev *d, int ch)
{
    if (ch < 0 || ch >= d->nch)
        errx(EXIT_USAGE, "channel %d out of range (device has %d)", ch, d->nch);
    if (d->cur_ch == ch)
        return 0;
    int rc = wr8(d, REG_CHAN_SEL, (uint8_t)(1u << ch));
    if (!rc)
        rc = wr8(d, REG_PAGE_CTRL, 0x01);   /* bit0 = channel page */
    if (!rc)
        d->cur_ch = ch;
    return rc;
}

/* select the shared-register page of one quad (DF410: quad 0 only) */
static int sel_shared(struct dev *d, int quad)
{
    int rc = wr8(d, REG_PAGE_CTRL, quad ? 0x20 : 0x10);
    if (!rc)
        d->cur_ch = -1;             /* next sel_ch() re-pages */
    return rc;
}

static void die_rc(int rc, const char *what)
{
    if (rc < 0)
        errx(EXIT_FAILURE, "%s failed: %s", what, strerror(-rc));
}

static int cmd_probe(struct dev *d)
{
    int ven = rd8(d, REG_VENDOR_ID);
    die_rc(ven, "vendor-id read");
    int cfg = rd8(d, REG_CHAN_CONFIG_ID);
    int ver = rd8(d, REG_VERSION);
    int id  = rd8(d, REG_DEVICE_ID);
    int csv = rd8(d, REG_CHAN_SHARE_VER);

    printf("addr 0x%02x: vendor=0x%02x %s  chan_config_id=0x%02x  "
           "version=0x%02x  device_id=0x%02x  chan/share_ver=0x%02x\n",
           d->addr, ven, ven == VENDOR_ID_TI ? "(TI)" : "(NOT TI!)",
           cfg, ver, id, csv);
    printf("  -> %d channels (%s)%s\n", d->nch,
           d->nch == 8 ? "DS250DF810" : "DS250DF410",
           d->nch == 8 ? "" : " -- or forced with -n");
    return ven == VENDOR_ID_TI ? 0 : 1;
}

static const char *prbs_name(unsigned code)
{
    switch (code) {
    case 0x8: return "PRBS7";
    case 0x9: return "PRBS9";
    case 0xa: return "PRBS11";
    case 0xb: return "PRBS15";
    case 0xc: return "PRBS23";
    case 0xd: return "PRBS31";
    case 0xe: return "PRBS58";
    case 0xf: return "PRBS63";
    default:  return "-";
    }
}

/*
 * Register decoder: bit-level field descriptions from the TI
 * datasheets (DS250DF410 SNLS456 Tables 8/9/10/11; the DF810 shares
 * the register model and adds quad paging). Only fields the
 * datasheet names are listed, unlisted bits are reserved.
 */

struct field {
    uint8_t     hi, lo;
    const char *name;
    const char *desc;
    const char *const *enums;   /* value names, index 0..2^width-1, or NULL */
};

struct regdesc {
    uint8_t             addr;
    const char         *name;
    const struct field *f;
    uint8_t             nf;
};

#define F(hi, lo, nm, ds)      { hi, lo, nm, ds, NULL }
#define FE(hi, lo, nm, ds, en) { hi, lo, nm, ds, en }
#define REG(a, nm, tab)        { a, nm, tab, (uint8_t)(sizeof(tab)/sizeof((tab)[0])) }

static const char *const e_prbs8[8] = {
    "PRBS7", "PRBS9", "PRBS11", "PRBS15", "PRBS23", "PRBS31", "PRBS58", "PRBS63"
};
static const char *const e_seqdet[16] = {
    "no detect", "no detect", "no detect", "no detect",
    "no detect", "no detect", "no detect", "no detect",
    "PRBS7", "PRBS9", "PRBS11", "PRBS15", "PRBS23", "PRBS31", "PRBS58", "PRBS63"
};
static const char *const e_vrange[4] = {
    "+/-100mV", "+/-200mV", "+/-300mV", "+/-400mV"
};
static const char *const e_adapt[4] = {
    "none", "CTLE only", "CTLE->DFE->CTLE", "CTLE->DFE->EQ"
};
static const char *const e_fom[4] = {
    "HEO+VEO", "HEO only", "VEO only", "HEO+VEO"
};
static const char *const e_dfefom[4] = {
    "invalid", "HEO only", "VEO only", "HEO+VEO"
};
static const char *const e_prelock[8] = {
    "raw data", NULL, NULL, NULL, NULL, NULL, NULL, "mute"
};
static const char *const e_postlock[8] = {
    "raw data", "retimed", NULL, NULL, "PRBS/fixed gen", "10M clock", NULL, "mute"
};
static const char *const e_eomrate[4] = {
    NULL, NULL, "half rate", "full rate"
};
static const char *const e_eecfg[4] = {
    "in progress/none", "load FAILED", "loaded OK", "invalid"
};
static const char *const e_cfgid[16] = {
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, "DS250DF810", NULL, "DS250DF410", NULL
};

/* channel registers (Tables 10 + 11) */
static const struct field f_c00[] = {
    F(3, 3, "RST_CORE",   "reset core clock domain (all state machines)"),
    F(2, 2, "RST_REGS",   "reset channel registers to defaults"),
    F(1, 1, "RST_VCO",    "reset CDR/S2P clock domain (PPM+EOM counters)"),
    F(0, 0, "RST_REFCLK", "reset 25MHz refclk domain"),
};
static const struct field f_c01[] = {
    F(7, 7, "SIGDET",            "raw signal detect"),
    F(6, 6, "POL_INV_DET",       "PRBS checker saw inverted polarity"),
    F(5, 5, "CDR_LOCK_LOSS_INT", "sticky lock-loss, clears on read (en 0x31[1])"),
    FE(4, 1, "PRBS_SEQ_DET",     "pattern detected on input", e_seqdet),
    F(0, 0, "SIG_DET_LOSS_INT",  "sticky signal-loss, clears on read (en 0x31[0])"),
};
static const struct field f_c02[] = {
    F(7, 0, "CDR_STATUS", "internal debug bus (selected via 0x0C[7:4])"),
};
static const struct field f_c03[] = {
    F(7, 6, "EQ_BST0", "CTLE stage 0 boost (forced when 0x2D[3]=1)"),
    F(5, 4, "EQ_BST1", "CTLE stage 1 boost"),
    F(3, 2, "EQ_BST2", "CTLE stage 2 boost"),
    F(1, 0, "EQ_BST3", "CTLE stage 3 boost"),
};
static const struct field f_c0d[] = {
    F(7, 7, "DES_PD", "1: deserializer (PRBS checker) powered down"),
};
static const struct field f_c11[] = {
    FE(7, 6, "EOM_SEL_VRANGE", "manual EOM range (with 0x2C[6]=0)", e_vrange),
    F(5, 5, "EOM_PD",          "1: EOM duty-cycled, 0: force-enabled"),
    F(3, 3, "DFE_TAP2_POL",    "1 = negative/boost"),
    F(2, 2, "DFE_TAP3_POL",    "1 = negative/boost"),
    F(1, 1, "DFE_TAP4_POL",    "1 = negative/boost"),
    F(0, 0, "DFE_TAP5_POL",    "1 = negative/boost"),
};
static const struct field f_c12[] = {
    F(7, 7, "DFE_TAP1_POL", "1 = negative/boost"),
    F(4, 0, "DFE_WT1",      "tap1 weight (manual w/ 0x15[7], else adapt seed)"),
};
static const struct field f_c13[] = {
    F(7, 7, "EQ_PD_PEAKDETECT", "1 = normal, 0 = test mode"),
    F(6, 6, "EQ_PD_SD",         "1: power down signal detect"),
    F(5, 5, "EQ_HI_GAIN",       "1: high DC gain CTLE mode"),
    F(4, 4, "EQ_EN_DC_OFF",     "1 = normal, 0 = no DC-offset compensation"),
    F(2, 2, "EQ_LIMIT_EN",      "1: final CTLE stage limiting"),
};
static const struct field f_c14[] = {
    F(7, 7, "EQ_SD_PRESET", "force sigdet HIGH + enable channel"),
    F(6, 6, "EQ_SD_RESET",  "force sigdet LOW + disable channel"),
    F(5, 4, "EQ_REFA_SEL",  "signal-detect assert level"),
    F(3, 2, "EQ_REFD_SEL",  "signal-detect de-assert level"),
};
static const struct field f_c15[] = {
    F(7, 7, "DFE_FORCE_EN", "1: manual DFE tap settings take effect"),
    F(3, 3, "DRV_PD",       "1: power down high-speed output driver"),
};
static const struct field f_c1e[] = {
    FE(7, 5, "PFD_SEL_DATA_PRELCK", "TX output while CDR unlocked", e_prelock),
    F(4, 4, "SER_EN",         "serializer enable (for PRBS generator)"),
    F(3, 3, "DFE_PD",         "1: DFE disabled"),
    F(2, 2, "PFD_PD_PD",      "1: PFD phase detector powered down"),
    F(1, 1, "EN_PARTIAL_DFE", "1: enable DFE taps 3-5"),
    F(0, 0, "PFD_EN_FD",      "1: PFD frequency detector enabled"),
};
static const struct field f_c20[] = {
    F(7, 4, "DFE_WT5", "tap5 weight (manual w/ 0x15[7])"),
    F(3, 0, "DFE_WT4", "tap4 weight"),
};
static const struct field f_c21[] = {
    F(7, 4, "DFE_WT3", "tap3 weight"),
    F(3, 0, "DFE_WT2", "tap2 weight"),
};
static const struct field f_c22[] = {
    F(7, 7, "EOM_OV",          "EOM manual-control override"),
    F(6, 6, "EOM_SEL_RATE_OV", "EOM rate-select override (rate in 0x39)"),
};
static const struct field f_c23[] = {
    F(7, 7, "EOM_GET_HEO_VEO_OV", "manual HEO/VEO trigger override"),
    F(6, 6, "DFE_OV",             "1 = normal (DFE also needs 0x1E[3]=0)"),
};
static const struct field f_c24[] = {
    F(7, 7, "FAST_EOM",             "auto-step 64x64 matrix into 0x25/0x26"),
    F(6, 6, "DFE_EQ_ERROR_NO_LOCK", "adapt SM quit due to loss of lock"),
    F(5, 5, "HEO_VEO_ERR_NO_HITS",  "get_heo_veo: no hits at zero crossing"),
    F(4, 4, "HEO_VEO_ERR_NO_OPEN",  "get_heo_veo: no vertical eye opening"),
    F(2, 2, "DFE_ADAPT",            "start DFE adaptation (self-clearing)"),
    F(1, 1, "EOM_GET_HEO_VEO",      "trigger HEO/VEO measure (needs 0x23[7])"),
    F(0, 0, "EOM_START",            "start EOM counter (self-clearing)"),
};
static const struct field f_c25[] = {
    F(7, 0, "EOM_COUNT[15:8]", "eye-monitor hit counter MSB"),
};
static const struct field f_c26[] = {
    F(7, 0, "EOM_COUNT[7:0]", "eye-monitor hit counter LSB"),
};
static const struct field f_c27[] = {
    F(7, 0, "HEO", "horizontal eye opening, /32 = UI (bit8 in 0x29[0])"),
};
static const struct field f_c28[] = {
    F(7, 0, "VEO", "vertical eye opening, x3.125 = mV (bit8 in 0x29[1])"),
};
static const struct field f_c29[] = {
    FE(6, 5, "EOM_VRANGE_SETTING", "currently set EOM range (RO)", e_vrange),
    F(1, 1, "VEO[8]", "VEO MSB"),
    F(0, 0, "HEO[8]", "HEO MSB"),
};
static const struct field f_c2a[] = {
    F(7, 4, "EOM_TIMER_THR",    "EOM dwell time per eye point"),
    F(3, 0, "VEO_MIN_REQ_HITS", "hit filter for the VEO measurement"),
};
static const struct field f_c2b[] = {
    F(3, 0, "EOM_MIN_REQ_HITS", "hit filter for the HEO measurement"),
};
static const struct field f_c2c[] = {
    F(7, 7, "RELOAD_DFE_TAPS",   "load taps from last adapted values"),
    F(6, 6, "VEO_SCALE",         "1: scale VEO based on EOM vrange"),
    FE(5, 4, "DFE_SM_FOM",       "DFE adaptation figure of merit", e_dfefom),
    F(3, 0, "DFE_ADAPT_COUNTER", "DFE look-beyond count"),
};
static const struct field f_c2d[] = {
    F(3, 3, "REG_EQ_BST_OV", "1: 0x03 forces the EQ boost"),
};
static const struct field f_c2e[] = {
    F(5, 5, "EQ_BST3_BIT2_TO_EQ",  "readback of eq_BST3[2] driving the EQ (RO)"),
    F(2, 2, "PRBS_PATTERN_SEL[2]", "generator pattern MSB (LSBs 0x30[1:0])"),
};
static const struct field f_c2f[] = {
    F(6, 4, "RATE",         "PPM/divider standard-rate group (TI prog guide)"),
    F(3, 3, "INDEX_OV",     "1: 0x39[3:0] indexes EQ array 0x40..0x4F"),
    F(2, 2, "EN_PPM_CHECK", "PPM check as lock-detect qualifier"),
    F(0, 0, "CTLE_ADAPT",   "restart CTLE adaptation (self-clearing)"),
};
static const struct field f_c30[] = {
    F(7, 7, "FREEZE_PPM_CNT",       "freeze PPM counter for safe read"),
    F(6, 6, "EQ_SEARCH_OV_EN",      "EQ search bit forced by 0x13[2]"),
    F(5, 5, "EN_PATT_INV",          "invert fixed pattern every 16 bits"),
    F(4, 4, "RELOAD_PRBS_CHKR",     "reload checker LFSR seed"),
    F(3, 3, "PRBS_EN_DIG_CLK",      "clock for PRBS gen/checker (toggle = reset)"),
    F(2, 2, "PRBS_PROGPATT_EN",     "fixed pattern from 0x7C/0x97 (needs 0x1E[4])"),
    F(1, 0, "PRBS_PATTERN_SEL[1:0]", "generator pattern LSBs (MSB 0x2E[2])"),
};
static const struct field f_c31[] = {
    F(7, 7, "PRBS_INT_EN",        "enable PRBS interrupt (status 0x71[7])"),
    FE(6, 5, "ADAPT_MODE",        "", e_adapt),
    FE(4, 3, "EQ_SM_FOM",         "CTLE adapt figure of merit", e_fom),
    F(1, 1, "CDR_LOCK_LOSS_INT_EN", "enable sticky 0x01[5]"),
    F(0, 0, "SIG_DET_LOSS_INT_EN",  "enable sticky 0x01[0]"),
};
static const struct field f_c32[] = {
    F(7, 4, "HEO_INT_THRESH", "HEO interrupt threshold (x8 counts)"),
    F(3, 0, "VEO_INT_THRESH", "VEO interrupt threshold (x8 counts)"),
};
static const struct field f_c33[] = {
    F(7, 4, "HEO_THRESH", "min HEO before DFE adapt (adapt mode 3)"),
    F(3, 0, "VEO_THRESH", "min VEO before DFE adapt (adapt mode 3)"),
};
static const struct field f_c34[] = {
    F(7, 7, "PPM_ERR_RDY",         "PPM error count ready in 0x3B/0x3C"),
    F(6, 6, "LOW_POWER_MODE_DIS",  "1: keep blocks powered while sigdet low"),
    F(5, 4, "LOCK_COUNTER",        "lock re-checks before out-of-lock (+3 max)"),
    F(3, 0, "DFE_MAX_TAP2_5",      "max adapt step for taps 2-5"),
};
static const struct field f_c35[] = {
    F(7, 6, "DATA_LOCK_PPM", "PPM delta tolerance scaling (with 0x64)"),
    F(5, 5, "GET_PPM_ERROR", "trigger PPM error capture (self-clears)"),
    F(4, 0, "DFE_MAX_TAP1",  "tap1 maximum magnitude limit"),
};
static const struct field f_c36[] = {
    F(6, 6, "HEO_VEO_INT_EN", "enable HEO/VEO interrupt (status 0x78[0])"),
    F(5, 4, "REF_MODE",       "11 = normal operation (reference mode 3)"),
};
static const struct field f_c37[] = {
    F(7, 0, "CTLE_STATUS", "reserved for future use"),
};
static const struct field f_c38[] = {
    F(7, 0, "DFE_STATUS", "reserved for future use"),
};
static const struct field f_c39[] = {
    FE(6, 5, "MR_EOM_RATE", "EOM rate when 0x22[6]=1", e_eomrate),
    F(3, 0, "START_INDEX",  "EQ adaptation start index"),
};
static const struct field f_c3a[] = {
    F(7, 6, "FIXED_EQ_BST0", "fixed EQ used when divider >2 (vs 0x6F[7])"),
    F(5, 4, "FIXED_EQ_BST1", ""),
    F(3, 2, "FIXED_EQ_BST2", ""),
    F(1, 0, "FIXED_EQ_BST3", ""),
};
static const struct field f_c3b[] = {
    F(7, 0, "PPM_COUNT[15:8]", "PPM count MSB (RO)"),
};
static const struct field f_c3c[] = {
    F(7, 0, "PPM_COUNT[7:0]", "PPM count LSB (RO)"),
};
static const struct field f_c3d[] = {
    F(7, 7, "EN_FIR_CURSOR", "enable pre/post-cursor FIR"),
    F(6, 6, "FIR_C0_SGN",    "main-cursor sign (1 = negative)"),
    F(4, 0, "FIR_C0",        "main-cursor magnitude"),
};
static const struct field f_c3e[] = {
    F(7, 7, "FIR_PD_TX",   "TX FIR power down"),
    F(6, 6, "FIR_CN1_SGN", "pre-cursor sign (1 = negative)"),
    F(3, 0, "FIR_CN1",     "pre-cursor magnitude"),
};
static const struct field f_c3f[] = {
    F(6, 6, "FIR_CP1_SGN", "post-cursor sign (1 = negative)"),
    F(3, 0, "FIR_CP1",     "post-cursor magnitude"),
};
static const struct field f_ceqarr[] = {
    F(7, 6, "EQ_ARRAY_BST0", "EQ search array entry, stage 0"),
    F(5, 4, "EQ_ARRAY_BST1", "stage 1"),
    F(3, 2, "EQ_ARRAY_BST2", "stage 2"),
    F(1, 0, "EQ_ARRAY_BST3", "stage 3"),
};
static const struct field f_c60[] = {
    F(7, 0, "GRP0_OV_CNT[7:0]", "group-0 rate count LSB"),
};
static const struct field f_c61[] = {
    F(7, 7, "CNT_DLTA_OV_0",     "group-0 manual data-rate override"),
    F(6, 0, "GRP0_OV_CNT[14:8]", "group-0 rate count MSB"),
};
static const struct field f_c62[] = {
    F(7, 0, "GRP1_OV_CNT[7:0]", "group-1 rate count LSB"),
};
static const struct field f_c63[] = {
    F(7, 7, "CNT_DLTA_OV_1",     "group-1 manual data-rate override"),
    F(6, 0, "GRP1_OV_CNT[14:8]", "group-1 rate count MSB"),
};
static const struct field f_c64[] = {
    F(7, 4, "GRP0_OV_DLTA[3:0]", "PPM delta tolerance grp0 (bit4 = 0x67[7])"),
    F(3, 0, "GRP1_OV_DLTA[3:0]", "PPM delta tolerance grp1 (bit4 = 0x67[6])"),
};
static const struct field f_c67[] = {
    F(7, 7, "GRP0_OV_DLTA[4]", ""),
    F(6, 6, "GRP1_OV_DLTA[4]", ""),
    F(5, 5, "HV_LOCKMON_EN",   "periodic HEO/VEO lock qualification"),
};
static const struct field f_c6a[] = {
    F(7, 4, "VEO_LCK_THRSH", "VEO needed before lock (x4 counts)"),
    F(3, 0, "HEO_LCK_THRSH", "HEO needed before lock (x4 counts)"),
};
static const struct field f_c6b[] = {
    F(6, 0, "FOM_A", "alternate-FOM weight A (= value/128)"),
};
static const struct field f_c6c[] = {
    F(7, 0, "FOM_B", "alternate-FOM HEO offset B"),
};
static const struct field f_c6d[] = {
    F(7, 0, "FOM_C", "alternate-FOM VEO offset C"),
};
static const struct field f_c6e[] = {
    F(7, 7, "EN_NEW_FOM_CTLE", "CTLE SM uses alt FOM (A/B/C = 0x6B..0x6D)"),
    F(6, 6, "EN_NEW_FOM_DFE",  "DFE SM uses alt FOM"),
};
static const struct field f_c70[] = {
    F(3, 0, "EQ_LB_CNT", "CTLE look-beyond count for adaptation"),
};
static const struct field f_c71[] = {
    F(7, 7, "PRBS_INT",      "PRBS detect/error irq, clears on read (en 0x31[7])"),
    F(5, 5, "DFE_POL_1_OBS", "tap1 polarity observation"),
    F(4, 0, "DFE_WT1_OBS",   "tap1 weight observation"),
};
static const struct field f_c76[] = {
    F(7, 4, "POST_LOCK_VEO_THR", "VEO threshold after lock established"),
    F(3, 0, "POST_LOCK_HEO_THR", "HEO threshold after lock established"),
};
static const struct field f_c77[] = {
    F(7, 7, "PRBS_GEN_POL_EN", "invert generated PRBS polarity"),
};
static const struct field f_c78[] = {
    F(5, 5, "SD_STATUS",        "signal detect (primary observation)"),
    F(4, 4, "CDR_LOCK_STATUS",  "CDR lock (primary observation)"),
    F(3, 3, "CDR_LOCK_INT",     "sticky lock-acquired, clears on read (en 0x79[1])"),
    F(2, 2, "SD_INT",           "sticky sigdet-change, clears on read (en 0x79[0])"),
    F(1, 1, "EOM_VRANGE_LIMIT", "HEO/VEO hit vrange limit during adapt"),
    F(0, 0, "HEO_VEO_INT",      "eye below 0x76 limits (en 0x36[6])"),
};
static const struct field f_c79[] = {
    F(6, 6, "PRBS_CHKR_EN",    "PRBS checker enable"),
    F(5, 5, "PRBS_GEN_EN",     "PRBS generator enable"),
    F(1, 1, "CDR_LOCK_INT_EN", "enable sticky 0x78[3]"),
    F(0, 0, "SD_INT_EN",       "enable sticky 0x78[2]"),
};
static const struct field f_c7c[] = {
    F(7, 0, "PRBS_FIXED[7:0]", "fixed-pattern LSB (MSB at 0x97)"),
};
static const struct field f_c82[] = {
    F(7, 7, "FREEZE_PRBS_CNTR", "freeze error counter for readback"),
    F(6, 6, "RST_PRBS_CNTS",    "1: hold error counter in reset"),
    F(5, 5, "PRBS_PATT_OV",     "force checker pattern from [4:2]"),
    FE(4, 2, "PRBS_PATT",       "forced checker pattern", e_prbs8),
    F(1, 1, "PRBS_POL_OV",      "force checker polarity from [0]"),
    F(0, 0, "PRBS_POL",         "forced polarity (1 = inverted)"),
};
static const struct field f_c83[] = {
    F(2, 0, "PRBS_ERR_CNT[10:8]", "error count MSBs (freeze 0x82[7] first)"),
};
static const struct field f_c84[] = {
    F(7, 0, "PRBS_ERR_CNT[7:0]", "error count LSB"),
};
static const struct field f_c8e[] = {
    F(0, 0, "VGA_SEL_GAIN", "1: VGA high-gain mode"),
};
static const struct field f_c95[] = {
    F(7, 7, "SD_ENABLE",      "force enable signal detect"),
    F(6, 6, "SD_DISABLE",     "force disable signal detect"),
    F(5, 5, "DC_OFF_ENABLE",  "force enable DC-offset compensation"),
    F(4, 4, "DC_OFF_DISABLE", "force disable DC-offset compensation"),
    F(3, 3, "EQ_ENABLE",      "force enable CTLE"),
    F(2, 2, "EQ_DISABLE",     "force disable CTLE"),
};
static const struct field f_c96[] = {
    F(3, 3, "EQ_EN_LOCAL",  "enable ebuf toward local output"),
    F(2, 2, "EQ_EN_FANOUT", "enable ebuf toward fanout"),
    F(1, 1, "EQ_SEL_XPNT",  "1: data from crosspoint, 0: local"),
};
static const struct field f_c97[] = {
    F(7, 0, "PRBS_FIXED[15:8]", "fixed-pattern MSB"),
};
static const struct field f_ca5[] = {
    FE(7, 5, "PFD_SEL_DATA_PSTLCK", "TX output while CDR locked", e_postlock),
};
static const struct field f_ca6[] = {
    F(7, 7, "INCR_HIST_TMR",      "EOM histogram timer +8"),
    F(6, 6, "EOM_TMR_ABRT_ON_HIT", "fast eye scan: next point on first hit"),
};

static const struct regdesc chan_regs[] = {
    REG(0x00, "channel reset (self-clearing pulses)", f_c00),
    REG(0x01, "channel status (sticky bits clear on read)", f_c01),
    REG(0x02, "CDR status debug bus", f_c02),
    REG(0x03, "CTLE boost force value", f_c03),
    REG(0x0D, "deserializer power", f_c0d),
    REG(0x11, "EOM range / DFE tap polarity", f_c11),
    REG(0x12, "DFE tap 1", f_c12),
    REG(0x13, "CTLE aux control", f_c13),
    REG(0x14, "signal-detect force / thresholds", f_c14),
    REG(0x15, "DFE manual mode / driver power", f_c15),
    REG(0x1E, "PFD / serializer / DFE power", f_c1e),
    REG(0x20, "DFE taps 4-5 weight", f_c20),
    REG(0x21, "DFE taps 2-3 weight", f_c21),
    REG(0x22, "EOM overrides", f_c22),
    REG(0x23, "HEO/VEO + DFE overrides", f_c23),
    REG(0x24, "EOM control / adapt errors", f_c24),
    REG(0x25, "EOM counter MSB", f_c25),
    REG(0x26, "EOM counter LSB", f_c26),
    REG(0x27, "horizontal eye opening", f_c27),
    REG(0x28, "vertical eye opening", f_c28),
    REG(0x29, "EOM range readback / HEO-VEO bit8", f_c29),
    REG(0x2A, "EOM timing / VEO hit filter", f_c2a),
    REG(0x2B, "HEO hit filter", f_c2b),
    REG(0x2C, "DFE adapt config", f_c2c),
    REG(0x2D, "EQ boost override enable", f_c2d),
    REG(0x2E, "EQ readback / PRBS pattern MSB", f_c2e),
    REG(0x2F, "rate select / CTLE adapt", f_c2f),
    REG(0x30, "PRBS generator config / PPM freeze", f_c30),
    REG(0x31, "adaptation mode / interrupt enables", f_c31),
    REG(0x32, "HEO-VEO interrupt thresholds", f_c32),
    REG(0x33, "CTLE-adapt minimum eye", f_c33),
    REG(0x34, "PPM ready / lock counter / DFE limits", f_c34),
    REG(0x35, "PPM tolerance / DFE tap1 limit", f_c35),
    REG(0x36, "HEO-VEO irq enable / ref mode", f_c36),
    REG(0x37, "CTLE status", f_c37),
    REG(0x38, "DFE status", f_c38),
    REG(0x39, "EOM rate / EQ start index", f_c39),
    REG(0x3A, "fixed EQ boost", f_c3a),
    REG(0x3B, "PPM count MSB", f_c3b),
    REG(0x3C, "PPM count LSB", f_c3c),
    REG(0x3D, "TX FIR main cursor", f_c3d),
    REG(0x3E, "TX FIR pre cursor", f_c3e),
    REG(0x3F, "TX FIR post cursor", f_c3f),
    REG(0x40, "EQ search array [0]", f_ceqarr),
    REG(0x41, "EQ search array [1]", f_ceqarr),
    REG(0x42, "EQ search array [2]", f_ceqarr),
    REG(0x43, "EQ search array [3]", f_ceqarr),
    REG(0x44, "EQ search array [4]", f_ceqarr),
    REG(0x45, "EQ search array [5]", f_ceqarr),
    REG(0x46, "EQ search array [6]", f_ceqarr),
    REG(0x47, "EQ search array [7]", f_ceqarr),
    REG(0x48, "EQ search array [8]", f_ceqarr),
    REG(0x49, "EQ search array [9]", f_ceqarr),
    REG(0x4A, "EQ search array [10]", f_ceqarr),
    REG(0x4B, "EQ search array [11]", f_ceqarr),
    REG(0x4C, "EQ search array [12]", f_ceqarr),
    REG(0x4D, "EQ search array [13]", f_ceqarr),
    REG(0x4E, "EQ search array [14]", f_ceqarr),
    REG(0x4F, "EQ search array [15]", f_ceqarr),
    REG(0x60, "rate group 0 count LSB", f_c60),
    REG(0x61, "rate group 0 override / count MSB", f_c61),
    REG(0x62, "rate group 1 count LSB", f_c62),
    REG(0x63, "rate group 1 override / count MSB", f_c63),
    REG(0x64, "PPM delta tolerances", f_c64),
    REG(0x67, "PPM delta MSBs / lock monitor", f_c67),
    REG(0x6A, "pre-lock eye thresholds", f_c6a),
    REG(0x6B, "alternate FOM A", f_c6b),
    REG(0x6C, "alternate FOM B", f_c6c),
    REG(0x6D, "alternate FOM C", f_c6d),
    REG(0x6E, "alternate FOM enables", f_c6e),
    REG(0x70, "CTLE look-beyond count", f_c70),
    REG(0x71, "PRBS irq / DFE tap1 observation", f_c71),
    REG(0x76, "post-lock eye thresholds", f_c76),
    REG(0x77, "PRBS generator polarity", f_c77),
    REG(0x78, "link status / sticky interrupts", f_c78),
    REG(0x79, "PRBS enables / interrupt enables", f_c79),
    REG(0x7C, "fixed pattern LSB", f_c7c),
    REG(0x82, "PRBS checker control", f_c82),
    REG(0x83, "PRBS error count MSB", f_c83),
    REG(0x84, "PRBS error count LSB", f_c84),
    REG(0x8E, "VGA gain select", f_c8e),
    REG(0x95, "analog force enables", f_c95),
    REG(0x96, "output buffer / crosspoint", f_c96),
    REG(0x97, "fixed pattern MSB", f_c97),
    REG(0xA5, "post-lock output mux", f_ca5),
    REG(0xA6, "EOM histogram / scan speed", f_ca6),
};

/* global registers (Table 8) */
static const struct field f_gef[] = {
    FE(3, 0, "CHAN_CONFIG_ID", "TI device ID (quad count)", e_cfgid),
};
static const struct field f_gf0[] = {
    F(7, 0, "VERSION", "version ID"),
};
static const struct field f_gf1[] = {
    F(7, 0, "DEVICE_ID", "full device ID"),
};
static const struct field f_gf3[] = {
    F(7, 4, "CHAN_VERSION",  "digital channel version"),
    F(3, 0, "SHARE_VERSION", "digital share version"),
};
static const struct field f_gfc[] = {
    F(7, 0, "EN_CH[7:0]", "channel-select bitmask (CH7..4 DF810 only)"),
};
static const struct field f_gfe[] = {
    F(7, 0, "VENDOR_ID", "0x03 = TI"),
};
static const struct field f_gff[] = {
    F(5, 5, "EN_SHARE_Q1", "select quad-1 shared regs (DF810 only)"),
    F(4, 4, "EN_SHARE_Q0", "select quad-0 shared regs"),
    F(1, 1, "WRITE_ALL_CH", "broadcast writes to all 0xFC channels"),
    F(0, 0, "EN_CH_SMB",   "1: channel regs, 0: shared regs"),
};

static const struct regdesc glob_regs[] = {
    REG(0xEF, "device family ID", f_gef),
    REG(0xF0, "version", f_gf0),
    REG(0xF1, "device ID", f_gf1),
    REG(0xF3, "digital versions", f_gf3),
    REG(0xFC, "channel select", f_gfc),
    REG(0xFE, "vendor ID", f_gfe),
    REG(0xFF, "page control", f_gff),
};

/* shared registers (Table 9), one bank per quad */
static const struct field f_s00[] = {
    F(7, 4, "SMBUS_ADDR", "strap offset: 7-bit address = 0x18 + value"),
};
static const struct field f_s04[] = {
    F(6, 6, "RST_I2C_REGS", "reset shared registers (self-clearing)"),
    F(5, 5, "RST_I2C_MAS",  "reset SMBus/I2C master (self-clearing)"),
    F(4, 4, "FRC_EEPRM_RD", "force EEPROM configuration"),
};
static const struct field f_s05[] = {
    F(7, 7, "DISAB_EEPRM_CFG",    "disable master-mode EEPROM config"),
    F(4, 4, "EEPROM_READ_DONE",   "EEPROM read complete (RO)"),
    F(3, 3, "TEST0_AS_CAL_CLK_IN", "TEST0 pin as 25MHz CAL_CLK (quad0 only)"),
    F(2, 2, "CAL_CLK_INV_DIS",    "disable CAL_CLK_OUT inversion"),
};
static const struct field f_s08[] = {
    F(3, 0, "INT_Q0C[3:0]", "per-channel interrupt (quad per 0xFF[5:4])"),
};
static const struct field f_s0a[] = {
    F(0, 0, "DIS_REFCLK_OUT", "1: CAL_CLK_OUT high-Z"),
};
static const struct field f_s0b[] = {
    F(6, 6, "REFCLK_DET",        "25MHz detected on CAL_CLK_IN (RO)"),
    F(3, 3, "MR_REFCLK_DET_DIS", "1: disable refclk detection"),
};
static const struct field f_s11[] = {
    FE(7, 6, "EECFG",     "EEPROM self-load status", e_eecfg),
    F(5, 0, "EECFG_ATMPT", "EEPROM load attempts"),
};
static const struct field f_s12[] = {
    F(7, 7, "REG_I2C_FAST", "EEPROM load at 400kHz (0 = 100kHz)"),
};

static const struct regdesc shar_regs[] = {
    REG(0x00, "SMBus address strap", f_s00),
    REG(0x04, "I2C resets / EEPROM force", f_s04),
    REG(0x05, "EEPROM / CAL_CLK config", f_s05),
    REG(0x08, "channel interrupt status", f_s08),
    REG(0x0A, "CAL_CLK_OUT control", f_s0a),
    REG(0x0B, "refclk detect", f_s0b),
    REG(0x11, "EEPROM self-load status", f_s11),
    REG(0x12, "EEPROM I2C speed", f_s12),
};

static const struct regdesc *find_reg(const struct regdesc *tab, size_t n,
                                      uint8_t addr)
{
    for (size_t i = 0; i < n; i++)
        if (tab[i].addr == addr)
            return &tab[i];
    return NULL;
}

/* space: 'c' channel page, 'g' global, 's' shared page */
static void print_decoded(char space, uint8_t addr, uint8_t val)
{
    const struct regdesc *r = NULL;
    switch (space) {
    case 'c':
        r = find_reg(chan_regs, sizeof(chan_regs) / sizeof(chan_regs[0]), addr);
        break;
    case 'g':
        r = find_reg(glob_regs, sizeof(glob_regs) / sizeof(glob_regs[0]), addr);
        break;
    case 's':
        r = find_reg(shar_regs, sizeof(shar_regs) / sizeof(shar_regs[0]), addr);
        break;
    default:
        break;
    }
    if (!r) {
        printf("  0x%02x = 0x%02x  (no documented fields)\n", addr, val);
        return;
    }
    printf("  0x%02x = 0x%02x  %s\n", addr, val, r->name);
    for (int i = 0; i < r->nf; i++) {
        const struct field *f = &r->f[i];
        unsigned w = (unsigned)(f->hi - f->lo) + 1u;
        unsigned v = ((unsigned)val >> f->lo) & ((1u << w) - 1u);
        char bits[16], vs[16];
        if (f->hi == f->lo)
            snprintf(bits, sizeof(bits), "[%u]", f->hi);
        else
            snprintf(bits, sizeof(bits), "[%u:%u]", f->hi, f->lo);
        if (w >= 5)
            snprintf(vs, sizeof(vs), "0x%02x/%u", v, v);
        else
            snprintf(vs, sizeof(vs), "%u", v);
        printf("        %-6s %-22s = %-8s", bits, f->name, vs);
        if (f->enums && f->enums[v])
            printf(" (%s)", f->enums[v]);
        if (f->desc && f->desc[0])
            printf("  %s", f->desc);
        putchar('\n');
    }
    /* computed interpretations */
    if (space == 'c' && addr == CH_HEO)
        printf("        -> HEO = %u/32 = %.3f UI\n", val, val / 32.0);
    else if (space == 'c' && addr == CH_VEO)
        printf("        -> VEO = %u x 3.125 = %.1f mV\n", val, val * 3.125);
    else if (space == 's' && addr == SH_SMBUS_ADDR)
        printf("        -> strapped 7-bit address = 0x%02x\n",
               0x18 + (((unsigned)val >> 4) & 0xf));
}

static int status_one(struct dev *d, int ch)
{
    die_rc(sel_ch(d, ch), "channel select");
    int st = rd8(d, CH_STATUS);
    die_rc(st, "status read");
    int heo = rd8(d, CH_HEO);
    int veo = rd8(d, CH_VEO);
    int lk  = rd8(d, CH_LINK_STATUS);      /* live SD + CDR lock */

    bool sig  = st & 0x80;
    bool cdr  = lk >= 0 && (lk & 0x10);
    bool pol  = st & 0x40;
    bool loss = st & 0x20;                 /* sticky, clears on read */
    unsigned prbs = ((unsigned)st >> 1) & 0xf;

    printf("ch%-2d sigdet=%d  cdr_lock=%d  heo=%.3f UI  veo=%.1f mV  pol_inv=%d  "
           "lock_loss(sticky)=%d  prbs_det=%s  [0x01=0x%02x 0x78=0x%02x]\n",
           ch, sig, cdr, heo / 32.0, veo * 3.125, pol, loss,
           prbs_name(prbs), st, lk < 0 ? 0 : lk);
    return sig ? 0 : 1;
}

static int cmd_status(struct dev *d, int ch)
{
    int bad = 0;
    if (ch >= 0)
        return status_one(d, ch);
    for (int c = 0; c < d->nch; c++)
        bad += status_one(d, c);
    return bad ? 1 : 0;
}

static int cmd_heo(struct dev *d, int ch)
{
    die_rc(sel_ch(d, ch), "channel select");
    int heo = rd8(d, CH_HEO);
    die_rc(heo, "HEO read");
    int veo = rd8(d, CH_VEO);
    die_rc(veo, "VEO read");
    printf("ch%d: HEO = %d/32 = %.3f UI   VEO = %d x 3.125 = %.1f mV%s\n",
           ch, heo, heo / 32.0, veo, veo * 3.125,
           (heo == 0 && veo == 0) ? "   (0/0 -- CDR probably not locked)" : "");
    return heo == 0 && veo == 0;
}

/*
 * Full 64x64 eye capture per datasheet Table 4. Renders an ASCII eye
 * (phase = X, voltage = Y, top = +V) and optionally dumps the raw
 * 16-bit hit counts as CSV.
 */
static int cmd_eye(struct dev *d, int ch, int vrange, const char *csv_path)
{
    static uint16_t eye[EYE_DIM][EYE_DIM];   /* [phase][voltage] */

    die_rc(sel_ch(d, ch), "channel select");

    int heo = rd8(d, CH_HEO), veo = rd8(d, CH_VEO);
    if (heo == 0 && veo == 0)
        warnx("ch%d: HEO/VEO both 0 -- CDR may be unlocked, eye may be empty", ch);

    /* steps 1-4 */
    die_rc(rmw8(d, CH_EOM_LOCKMON, 0x20, 0x00), "step1 0x67[5]=0");
    die_rc(rmw8(d, CH_DFE_CFG, 0x40, 0x00), "step2 0x2C[6]=0");
    die_rc(rmw8(d, CH_EOM_VRANGE, 0xC0, (uint8_t)(vrange << 6)), "step2 0x11[7:6]");
    die_rc(rmw8(d, CH_EOM_VRANGE, 0x20, 0x00), "step3 0x11[5]=0");
    die_rc(rmw8(d, CH_EOM_CTRL, 0x80, 0x80), "step4 0x24[7]=1");
    die_rc(rmw8(d, CH_EOM_CTRL, 0x01, 0x01), "step4 0x24[0]=1");

    /* step 5: 4 junk words then 4096 words, voltage minor, phase major */
    fprintf(stderr, "capturing 64x64 eye on ch%d (8200 reads, be patient)...\n", ch);
    for (int w = 0; w < EYE_JUNK_WORDS + EYE_DIM * EYE_DIM; w++) {
        int msb = rd8(d, CH_EOM_MSB);
        die_rc(msb, "EOM MSB read");
        int lsb = rd8(d, CH_EOM_LSB);
        die_rc(lsb, "EOM LSB read");
        if (w >= EYE_JUNK_WORDS) {
            int i = w - EYE_JUNK_WORDS;
            eye[i / EYE_DIM][i % EYE_DIM] =
                (uint16_t)(((unsigned)msb << 8) | (unsigned)lsb);
        }
    }

    /* step 7: restore */
    rmw8(d, CH_EOM_LOCKMON, 0x20, 0x20);
    rmw8(d, CH_DFE_CFG, 0x40, 0x40);
    rmw8(d, CH_EOM_VRANGE, 0x20, 0x20);
    rmw8(d, CH_EOM_CTRL, 0x82, 0x00);

    /* stats */
    uint32_t max = 0;
    uint64_t total = 0;
    int zero_cells = 0;
    for (int p = 0; p < EYE_DIM; p++)
        for (int v = 0; v < EYE_DIM; v++) {
            if (eye[p][v] > max)
                max = eye[p][v];
            if (eye[p][v] == 0)
                zero_cells++;
            total += eye[p][v];
        }

    static const int vr_mv[4] = { 100, 200, 300, 400 };
    printf("ch%d eye: HEO=%.3f UI  VEO=%.1f mV  vrange=+/-%d mV  "
           "max_hits=%u  open_cells=%d/4096\n",
           ch, heo / 32.0, veo * 3.125, vr_mv[vrange], max, zero_cells);

    /* ASCII render: 64 phases wide, voltage pairs folded -> 32 rows,
     * +V at the top. Hits ramp ' ' '.' ':' 'o' '#' '@' (log-ish).   */
    const char ramp[] = " .:o#@";
    for (int row = EYE_DIM / 2 - 1; row >= 0; row--) {
        putchar('|');
        for (int p = 0; p < EYE_DIM; p++) {
            uint32_t h = (uint32_t)eye[p][2 * row] + eye[p][2 * row + 1];
            int idx = 0;
            if (h > 0 && max > 0) {
                idx = 1;
                for (uint32_t t = max; t > 1 && idx < 5; t /= 8)
                    if (h >= t)
                        break;
                    else
                        idx++;
                idx = 6 - idx;
                if (idx < 1) idx = 1;
                if (idx > 5) idx = 5;
            }
            putchar(ramp[idx]);
        }
        puts("|");
    }
    printf("+%.*s+  phase 0..63 (early->late), blank = no hits (open eye)\n",
           EYE_DIM, "----------------------------------------------------------------");

    if (csv_path) {
        FILE *f = fopen(csv_path, "w");
        if (!f)
            err(EXIT_FAILURE, "fopen %s", csv_path);
        for (int v = 0; v < EYE_DIM; v++) {
            for (int p = 0; p < EYE_DIM; p++)
                fprintf(f, "%u%c", eye[p][v], p == EYE_DIM - 1 ? '\n' : ',');
        }
        fclose(f);
        printf("raw 64x64 hit counts written to %s (rows = voltage 0..63)\n",
               csv_path);
    }
    (void)total;
    return 0;
}

static int cmd_dump(struct dev *d, int ch, bool globals, bool shared,
                    bool verbose, int first, int last)
{
    char space = 'c';

    if (globals) {
        space = 'g';
        first = 0xEF;
        last = 0xFF;
        printf("globals%s:\n", verbose ? " (decoded)" : "");
    } else if (shared) {
        space = 's';
        int quad = ch < 0 ? 0 : ch;
        if (quad >= d->nch / 4)
            errx(EXIT_USAGE, "quad %d out of range (device has %d)", quad,
                 d->nch / 4);
        die_rc(sel_shared(d, quad), "shared page select");
        if (first == 0x00 && last == 0x7f)
            last = 0x12;                     /* documented shared window */
        printf("quad%d shared registers 0x%02x..0x%02x%s:\n", quad, first,
               last, verbose ? " (decoded)" : "");
    } else {
        die_rc(sel_ch(d, ch), "channel select");
        printf("ch%d registers 0x%02x..0x%02x%s:\n", ch, first, last,
               verbose ? " (decoded)" : "");
    }
    for (int r = first; r <= last; r++) {
        int v = rd8(d, (uint8_t)r);
        if (verbose) {
            if (v < 0)
                printf("  0x%02x = read error (%s)\n", r, strerror(-v));
            else
                print_decoded(space, (uint8_t)r, (uint8_t)v);
            continue;
        }
        if ((r - first) % 8 == 0)
            printf("  0x%02x:", r);
        if (v < 0)
            printf("  err");
        else
            printf("  0x%02x", v);
        if ((r - first) % 8 == 7 || r == last)
            putchar('\n');
    }
    return 0;
}

static int cmd_reset(struct dev *d, int ch, uint8_t bits)
{
    die_rc(sel_ch(d, ch), "channel select");
    die_rc(wr8(d, CH_RESET, bits), "reset assert");
    usleep(10 * 1000);
    die_rc(wr8(d, CH_RESET, 0x00), "reset deassert");
    printf("ch%d: reset bits 0x%02x pulsed\n", ch, bits);
    return 0;
}

/* PRBS pattern length -> 3-bit select code (0x30[1:0] + 0x2E[2], 0x82[4:2]) */
static unsigned prbs_code(const char *s)
{
    static const int tab[8] = { 7, 9, 11, 15, 23, 31, 58, 63 };
    int n = atoi(s);
    for (unsigned i = 0; i < 8; i++)
        if (tab[i] == n)
            return i;
    errx(EXIT_USAGE, "unknown PRBS pattern '%s' (7|9|11|15|23|31|58|63)", s);
}

static int cmd_prbs(struct dev *d, int ch, int npos, char **posv)
{
    const char *sub = npos ? posv[0] : "errors";

    die_rc(sel_ch(d, ch), "channel select");

    if (!strcmp(sub, "check")) {
        if (npos > 1) {
            unsigned c = prbs_code(posv[1]);
            die_rc(rmw8(d, CH_PRBS_CHK, 0x3C, (uint8_t)(0x20 | (c << 2))),
                   "0x82 pattern force");
        } else {
            die_rc(rmw8(d, CH_PRBS_CHK, 0x20, 0x00), "0x82 auto-detect");
        }
        die_rc(rmw8(d, CH_DES_PD, 0x80, 0x00), "0x0D[7] deserializer on");
        die_rc(rmw8(d, CH_PRBS_CFG, 0x08, 0x08), "0x30[3] PRBS clock");
        die_rc(rmw8(d, CH_PRBS_EN, 0x40, 0x40), "0x79[6] checker enable");
        die_rc(rmw8(d, CH_PRBS_CHK, 0x40, 0x40), "0x82[6] counter reset");
        die_rc(rmw8(d, CH_PRBS_CHK, 0x40, 0x00), "0x82[6] counter release");
        printf("ch%d: PRBS checker on (%s pattern), counters zeroed -- "
               "read with 'prbs -c %d errors'\n",
               ch, npos > 1 ? posv[1] : "auto-detect", ch);
        return 0;
    }
    if (!strcmp(sub, "gen")) {
        unsigned c = npos > 1 ? prbs_code(posv[1]) : 5;   /* default PRBS31 */
        die_rc(rmw8(d, CH_PRBS_SEL_MSB, 0x04, (c & 4) ? 0x04 : 0x00),
               "0x2E[2] pattern msb");
        die_rc(rmw8(d, CH_PRBS_CFG, 0x03, (uint8_t)(c & 3)), "0x30[1:0] pattern");
        die_rc(rmw8(d, CH_PFD_CFG, 0x10, 0x10), "0x1E[4] serializer");
        die_rc(rmw8(d, CH_PRBS_CFG, 0x08, 0x08), "0x30[3] PRBS clock");
        die_rc(rmw8(d, CH_PRBS_EN, 0x20, 0x20), "0x79[5] generator enable");
        /* route the generator to TX while the CDR is locked (0xA5[7:5]=100) */
        die_rc(rmw8(d, CH_PSTLCK_MUX, 0xE0, 0x80), "0xA5 post-lock mux -> gen");
        printf("ch%d: PRBS%s generator on -- WARNING: TX now carries the test "
               "pattern instead of retimed data ('prbs -c %d off' to stop)\n",
               ch, npos > 1 ? posv[1] : "31", ch);
        return 0;
    }
    if (!strcmp(sub, "errors") || !strcmp(sub, "clear")) {
        die_rc(rmw8(d, CH_PRBS_CHK, 0x80, 0x80), "0x82[7] freeze");
        int hi = rd8(d, CH_PRBS_ERR_HI);
        int lo = rd8(d, CH_PRBS_ERR_LO);
        die_rc(hi < 0 ? hi : (lo < 0 ? lo : 0), "error-counter read");
        die_rc(rmw8(d, CH_PRBS_CHK, 0x80, 0x00), "0x82[7] unfreeze");
        int st = rd8(d, CH_STATUS);
        unsigned e = (((unsigned)hi & 7) << 8) | (unsigned)lo;
        printf("ch%d: prbs_det=%s  errors=%u%s\n", ch,
               prbs_name(((unsigned)st >> 1) & 0xf), e,
               e == 0x7ff ? " (saturated)" : "");
        if (!strcmp(sub, "clear")) {
            die_rc(rmw8(d, CH_PRBS_CHK, 0x40, 0x40), "counter reset");
            die_rc(rmw8(d, CH_PRBS_CHK, 0x40, 0x00), "counter release");
        }
        return e ? 1 : 0;
    }
    if (!strcmp(sub, "off")) {
        die_rc(rmw8(d, CH_PSTLCK_MUX, 0xE0, 0x20), "0xA5 post-lock mux -> retimed");
        die_rc(rmw8(d, CH_PRBS_EN, 0x60, 0x00), "0x79 gen+chk off");
        die_rc(rmw8(d, CH_PRBS_CFG, 0x08, 0x00), "0x30[3] PRBS clock off");
        die_rc(rmw8(d, CH_PFD_CFG, 0x10, 0x00), "0x1E[4] serializer off");
        die_rc(rmw8(d, CH_DES_PD, 0x80, 0x80), "0x0D[7] deserializer off");
        printf("ch%d: PRBS generator + checker off, datapath restored\n", ch);
        return 0;
    }
    errx(EXIT_USAGE, "prbs: check [PATT] | gen [PATT] | errors | clear | off");
}

static int cmd_eq(struct dev *d, int ch, int npos, char **posv)
{
    die_rc(sel_ch(d, ch), "channel select");

    if (npos && !strcmp(posv[0], "adapt")) {
        die_rc(rmw8(d, CH_RATE, 0x01, 0x01), "0x2F[0] CTLE_ADAPT");
        printf("ch%d: CTLE adaptation restarted\n", ch);
        return 0;
    }
    if (npos && !strcmp(posv[0], "set")) {
        if (npos < 2)
            errx(EXIT_USAGE, "eq set needs a boost byte (stage pairs, 0x00..0xFF)");
        uint8_t v = (uint8_t)strtol(posv[1], NULL, 0);
        die_rc(rmw8(d, CH_EQ_OV, 0x08, 0x08), "0x2D[3] EQ override");
        die_rc(wr8(d, CH_EQ_BST, v), "0x03 EQ boost");
        printf("ch%d: EQ boost forced to 0x%02x (stages %u/%u/%u/%u)\n", ch, v,
               (v >> 6) & 3, (v >> 4) & 3, (v >> 2) & 3, v & 3);
        return 0;
    }
    if (npos && !strcmp(posv[0], "auto")) {
        die_rc(rmw8(d, CH_EQ_OV, 0x08, 0x00), "0x2D[3] EQ override");
        printf("ch%d: EQ boost back under adaptation control\n", ch);
        return 0;
    }

    static const char *const adapt_names[4] = {
        "none", "CTLE only", "CTLE->DFE->CTLE", "CTLE->DFE->EQ (full)"
    };
    int bst  = rd8(d, CH_EQ_BST);
    int ov   = rd8(d, CH_EQ_OV);
    int rate = rd8(d, CH_RATE);
    int ad   = rd8(d, CH_ADAPT);
    int ctle = rd8(d, CH_CTLE_STATUS);
    die_rc(bst < 0 ? bst : (ad < 0 ? ad : 0), "EQ register read");
    printf("ch%d: adapt_mode=%u (%s)  eq_sm_fom=%u  rate_sel=%u  index_ov=%d\n",
           ch, ((unsigned)ad >> 5) & 3, adapt_names[((unsigned)ad >> 5) & 3],
           ((unsigned)ad >> 3) & 3, ((unsigned)rate >> 4) & 7, (rate >> 3) & 1);
    printf("ch%d: boost(0x03)=0x%02x stages %u/%u/%u/%u  override(0x2D[3])=%d  "
           "ctle_status(0x37)=0x%02x\n",
           ch, bst, ((unsigned)bst >> 6) & 3, ((unsigned)bst >> 4) & 3,
           ((unsigned)bst >> 2) & 3, (unsigned)bst & 3, (ov >> 3) & 1, ctle);
    return 0;
}

static int cmd_dfe(struct dev *d, int ch, int npos, char **posv)
{
    die_rc(sel_ch(d, ch), "channel select");

    if (npos && !strcmp(posv[0], "on")) {
        die_rc(rmw8(d, CH_PFD_CFG, 0x0A, 0x02), "0x1E DFE enable + taps 3-5");
        printf("ch%d: DFE enabled (taps 1-5)\n", ch);
        return 0;
    }
    if (npos && !strcmp(posv[0], "off")) {
        die_rc(rmw8(d, CH_PFD_CFG, 0x0A, 0x08), "0x1E DFE power-down");
        printf("ch%d: DFE disabled\n", ch);
        return 0;
    }

    int pfd = rd8(d, CH_PFD_CFG);
    int frc = rd8(d, CH_DFE_FORCE);
    int t1  = rd8(d, CH_DFE_TAP1);
    int pol = rd8(d, CH_DFE_POL);
    int w45 = rd8(d, CH_DFE_WT45);
    int w23 = rd8(d, CH_DFE_WT23);
    die_rc(pfd < 0 ? pfd : (t1 < 0 ? t1 : 0), "DFE register read");
    printf("ch%d: dfe=%s  taps3-5=%s  manual_force(0x15[7])=%d\n", ch,
           (pfd & 0x08) ? "off" : "on", (pfd & 0x02) ? "on" : "off",
           (frc >> 7) & 1);
    printf("ch%d: tap1=%c%u  tap2=%c%u  tap3=%c%u  tap4=%c%u  tap5=%c%u  "
           "(-=boost, +=attenuate)\n", ch,
           (t1 & 0x80) ? '-' : '+', (unsigned)t1 & 0x1f,
           (pol & 0x08) ? '-' : '+', ((unsigned)w23 >> 0) & 0xf,
           (pol & 0x04) ? '-' : '+', ((unsigned)w23 >> 4) & 0xf,
           (pol & 0x02) ? '-' : '+', ((unsigned)w45 >> 0) & 0xf,
           (pol & 0x01) ? '-' : '+', ((unsigned)w45 >> 4) & 0xf);
    return 0;
}

static int cmd_fir(struct dev *d, int ch, int npos, char **posv)
{
    die_rc(sel_ch(d, ch), "channel select");

    if (npos && !strcmp(posv[0], "set")) {
        if (npos < 4)
            errx(EXIT_USAGE, "fir set needs PRE MAIN POST (signed, e.g. -2 26 -4)");
        int pre = atoi(posv[1]), main_c = atoi(posv[2]), post = atoi(posv[3]);
        if (abs(pre) > 15 || abs(main_c) > 31 || abs(post) > 15)
            errx(EXIT_USAGE, "cursor range: pre/post -15..15, main -31..31");
        die_rc(wr8(d, CH_FIR_C0, (uint8_t)(0x80 | (main_c < 0 ? 0x40 : 0) |
                                           (abs(main_c) & 0x1f))), "0x3D main");
        die_rc(rmw8(d, CH_FIR_CN1, 0x4F, (uint8_t)((pre < 0 ? 0x40 : 0) |
                                                   (abs(pre) & 0xf))), "0x3E pre");
        die_rc(rmw8(d, CH_FIR_CP1, 0x4F, (uint8_t)((post < 0 ? 0x40 : 0) |
                                                   (abs(post) & 0xf))), "0x3F post");
        printf("ch%d: TX FIR set pre=%d main=%d post=%d (cursors enabled)\n",
               ch, pre, main_c, post);
        return 0;
    }
    if (npos && !strcmp(posv[0], "off")) {
        die_rc(rmw8(d, CH_FIR_C0, 0x80, 0x00), "0x3D[7] cursor disable");
        printf("ch%d: pre/post FIR cursors disabled (lower power)\n", ch);
        return 0;
    }

    int c0  = rd8(d, CH_FIR_C0);
    int cn1 = rd8(d, CH_FIR_CN1);
    int cp1 = rd8(d, CH_FIR_CP1);
    die_rc(c0 < 0 ? c0 : (cp1 < 0 ? cp1 : 0), "FIR register read");
    printf("ch%d: fir_cursors=%s  tx_pd=%d  pre=%c%u  main=%c%u  post=%c%u\n",
           ch, (c0 & 0x80) ? "on" : "off", (cn1 >> 7) & 1,
           (cn1 & 0x40) ? '-' : '+', (unsigned)cn1 & 0xf,
           (c0 & 0x40) ? '-' : '+', (unsigned)c0 & 0x1f,
           (cp1 & 0x40) ? '-' : '+', (unsigned)cp1 & 0xf);
    return 0;
}

static int cmd_shared(struct dev *d)
{
    int nquad = d->nch / 4;
    for (int q = 0; q < nquad; q++) {
        die_rc(sel_shared(d, q), "shared page select");
        int sa  = rd8(d, SH_SMBUS_ADDR);
        int ee  = rd8(d, SH_EE_STAT);
        int in  = rd8(d, SH_INT);
        int rk  = rd8(d, SH_REFCLK);
        int cfg = rd8(d, SH_EECFG);
        die_rc(sa < 0 ? sa : (cfg < 0 ? cfg : 0), "shared register read");
        unsigned eest = ((unsigned)cfg >> 6) & 3;
        printf("quad%d: smbus_addr=0x%02x  cal_clk_25mhz=%s  eeprom=%s "
               "(attempts=%u, read_done=%d)  int_ch[3..0]=%u%u%u%u\n",
               q, 0x18 + (((unsigned)sa >> 4) & 0xf),
               (rk & 0x40) ? "detected" : "MISSING",
               eest == 2 ? "loaded" : (eest == 1 ? "FAILED" : "none/in-progress"),
               (unsigned)cfg & 0x3f, (ee >> 4) & 1,
               (in >> 3) & 1, (in >> 2) & 1, (in >> 1) & 1, in & 1);
    }
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s <i2c-dev> <addr> <command> [options]\n"
        "\n"
        "  <i2c-dev>   /dev/i2c-N (or just N)\n"
        "  <addr>      7-bit SMBus address (family window 0x18..0x27, per ADDR straps)\n"
        "\n"
        "Commands:\n"
        "  probe                     identify the device (vendor/version/id)\n"
        "  status [-c CH]            per-channel sigdet/HEO/VEO/PRBS (default: all)\n"
        "  heo -c CH                 quick HEO/VEO readout\n"
        "  eye -c CH [-r 0..3] [-o F.csv]\n"
        "                            full 64x64 EOM capture, ASCII eye + CSV\n"
        "                            -r vertical range 0=+/-100mV .. 3=+/-400mV (default 1)\n"
        "  prbs -c CH check [PATT]   enable PRBS checker (auto-detect or 7|9|11|15|23|31|58|63)\n"
        "  prbs -c CH errors|clear   read (and optionally zero) the 11-bit PRBS error counter\n"
        "  prbs -c CH gen [PATT]     enable PRBS generator on TX (DISRUPTS traffic!)\n"
        "  prbs -c CH off            generator + checker off, datapath restored\n"
        "  eq -c CH [adapt|auto|set BYTE]\n"
        "                            show/restart/force CTLE boost + adaptation mode\n"
        "  dfe -c CH [on|off]        show or switch the 5-tap DFE\n"
        "  fir -c CH [set -- PRE MAIN POST | off]\n"
        "                            show or set TX FIR cursors (signed; '--' ends\n"
        "                            option parsing so negative cursors pass through)\n"
        "  shared                    per-quad shared page: CAL_CLK detect, EEPROM, addr strap\n"
        "  dump [-c CH] [FIRST LAST] raw register dump (default 0x00 0x7f)\n"
        "  dump -g                   global registers 0xEF..0xFF\n"
        "  dump -s [-c QUAD]         shared-page registers 0x00..0x12\n"
        "  dump ... -v               decoded dump: per-bit field names + meanings\n"
        "  rd REG [-c CH|-s]         read one register (decoded when page is known)\n"
        "  wr REG VAL [-c CH|-s]     write one register, decode the readback\n"
        "  reset -c CH [core|regs|vco|refclk|all]\n"
        "\n"
        "Options (short/long):\n"
        "  -c, --channel CH          channel to operate on\n"
        "  -r, --range 0..3          eye vertical range\n"
        "  -o, --output F.csv        eye CSV output file\n"
        "  -n, --nch 4|8             force channel count (default: from 0xEF)\n"
        "  -g, --globals             dump global registers 0xEF..0xFF\n"
        "  -s, --shared              address the shared page (quad via -c, default 0)\n"
        "  -v, --verbose             decode registers: bit fields, names, meanings\n"
        "  -h, --help                this help\n"
        "\n"
        "Exit codes: 0 ok; 1 device/channel unhealthy or verify failed; 2 usage\n",
        argv0);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        usage(argv[0]);
        return EXIT_USAGE;
    }

    char path[32];
    if (argv[1][0] == '/')
        snprintf(path, sizeof(path), "%s", argv[1]);
    else
        snprintf(path, sizeof(path), "/dev/i2c-%s", argv[1]);

    struct dev d = { .fd = -1, .cur_ch = -1, .nch = 0 };
    d.addr = (uint16_t)strtoul(argv[2], NULL, 0);
    if (d.addr < 0x08 || d.addr > 0x7f)
        errx(EXIT_USAGE, "addr 0x%x out of 7-bit range", d.addr);

    d.fd = open(path, O_RDWR);
    if (d.fd < 0)
        err(EXIT_FAILURE, "open %s", path);

    const char *cmd = argv[3];

    static const struct option longopts[] = {
        { "channel", required_argument, NULL, 'c' },
        { "range",   required_argument, NULL, 'r' },
        { "output",  required_argument, NULL, 'o' },
        { "nch",     required_argument, NULL, 'n' },
        { "globals", no_argument,       NULL, 'g' },
        { "shared",  no_argument,       NULL, 's' },
        { "verbose", no_argument,       NULL, 'v' },
        { "help",    no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };

    int ch = -1, vrange = 1, nch_force = 0, first = 0x00, last = 0x7f;
    bool globals = false, shared = false, verbose = false;
    const char *csv = NULL;

    /* options start after the three positionals; GNU getopt permutes
     * any remaining positionals (REG/VAL, FIRST/LAST, reset domain)
     * to the tail, collected below via optind. */
    optind = 4;
    int opt;
    while ((opt = getopt_long(argc, argv, "c:r:o:n:gsvh", longopts, NULL)) != -1) {
        switch (opt) {
        case 'c': ch = (int)strtol(optarg, NULL, 0); break;
        case 'r': vrange = (int)strtol(optarg, NULL, 0) & 3; break;
        case 'o': csv = optarg; break;
        case 'n': nch_force = (int)strtol(optarg, NULL, 0); break;
        case 'g': globals = true; break;
        case 's': shared = true; break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return EXIT_SUCCESS;
        default:  usage(argv[0]); return EXIT_USAGE;
        }
    }
    int npos = argc - optind;
    char **posv = argv + optind;
    long pos[2] = { 0, 0 };
    if (npos > 0)
        pos[0] = strtol(posv[0], NULL, 0);
    if (npos > 1)
        pos[1] = strtol(posv[1], NULL, 0);

    /* identify + channel count (0xEF CHAN_CONFIG_ID; bench: DF810=0x0c) */
    int ven = rd8(&d, REG_VENDOR_ID);
    if (ven < 0)
        errx(EXIT_FAILURE, "no response from 0x%02x on %s: %s",
             d.addr, path, strerror(-ven));
    int cfg = rd8(&d, REG_CHAN_CONFIG_ID);
    d.nch = nch_force ? nch_force : ((cfg & 0x0f) == 0x0c ? 8 : 4);
    if (d.nch != 4 && d.nch != 8)
        errx(EXIT_USAGE, "-n must be 4 or 8");

    if (!strcmp(cmd, "probe"))
        return cmd_probe(&d);
    if (!strcmp(cmd, "status"))
        return cmd_status(&d, ch);
    if (!strcmp(cmd, "heo")) {
        if (ch < 0)
            errx(EXIT_USAGE, "heo needs -c CH");
        return cmd_heo(&d, ch);
    }
    if (!strcmp(cmd, "eye")) {
        if (ch < 0)
            errx(EXIT_USAGE, "eye needs -c CH");
        return cmd_eye(&d, ch, vrange, csv);
    }
    if (!strcmp(cmd, "prbs")) {
        if (ch < 0)
            errx(EXIT_USAGE, "prbs needs -c CH");
        return cmd_prbs(&d, ch, npos, posv);
    }
    if (!strcmp(cmd, "eq")) {
        if (ch < 0)
            errx(EXIT_USAGE, "eq needs -c CH");
        return cmd_eq(&d, ch, npos, posv);
    }
    if (!strcmp(cmd, "dfe")) {
        if (ch < 0)
            errx(EXIT_USAGE, "dfe needs -c CH");
        return cmd_dfe(&d, ch, npos, posv);
    }
    if (!strcmp(cmd, "fir")) {
        if (ch < 0)
            errx(EXIT_USAGE, "fir needs -c CH");
        return cmd_fir(&d, ch, npos, posv);
    }
    if (!strcmp(cmd, "shared"))
        return cmd_shared(&d);
    if (!strcmp(cmd, "dump")) {
        if (npos == 2) {
            first = (int)pos[0];
            last = (int)pos[1];
        }
        if (!globals && !shared && ch < 0)
            errx(EXIT_USAGE, "dump needs -c CH (or -g / -s)");
        return cmd_dump(&d, ch, globals, shared, verbose, first, last);
    }
    if (!strcmp(cmd, "rd")) {
        if (npos < 1)
            errx(EXIT_USAGE, "rd needs REG");
        uint8_t reg = (uint8_t)pos[0];
        if (shared)
            die_rc(sel_shared(&d, ch < 0 ? 0 : ch), "shared page select");
        else if (ch >= 0)
            die_rc(sel_ch(&d, ch), "channel select");
        int v = rd8(&d, reg);
        die_rc(v, "read");
        if (reg >= 0xEF)
            print_decoded('g', reg, (uint8_t)v);
        else if (shared)
            print_decoded('s', reg, (uint8_t)v);
        else if (ch >= 0)
            print_decoded('c', reg, (uint8_t)v);
        else
            printf("0x%02x = 0x%02x\n", reg, v);
        return 0;
    }
    if (!strcmp(cmd, "wr")) {
        if (npos < 2)
            errx(EXIT_USAGE, "wr needs REG VAL");
        uint8_t reg = (uint8_t)pos[0];
        if (shared)
            die_rc(sel_shared(&d, ch < 0 ? 0 : ch), "shared page select");
        else if (ch >= 0)
            die_rc(sel_ch(&d, ch), "channel select");
        die_rc(wr8(&d, reg, (uint8_t)pos[1]), "write");
        int v = rd8(&d, reg);
        printf("0x%02x <- 0x%02lx (reads back 0x%02x)\n", reg, pos[1],
               v < 0 ? 0xFFF : v);
        if (v >= 0) {
            if (reg >= 0xEF)
                print_decoded('g', reg, (uint8_t)v);
            else if (shared)
                print_decoded('s', reg, (uint8_t)v);
            else if (ch >= 0)
                print_decoded('c', reg, (uint8_t)v);
        }
        return 0;
    }
    if (!strcmp(cmd, "reset")) {
        if (ch < 0)
            errx(EXIT_USAGE, "reset needs -c CH");
        uint8_t bits = 0x0A;   /* default: core + vco */
        for (int i = 0; i < npos; i++) {
            if (!strcmp(posv[i], "core"))   bits = 0x08;
            if (!strcmp(posv[i], "regs"))   bits = 0x04;
            if (!strcmp(posv[i], "vco"))    bits = 0x02;
            if (!strcmp(posv[i], "refclk")) bits = 0x01;
            if (!strcmp(posv[i], "all"))    bits = 0x0F;
        }
        return cmd_reset(&d, ch, bits);
    }

    usage(argv[0]);
    return EXIT_USAGE;
}
