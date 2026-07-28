# ds250-tool - TI DS250 retimer management

Monitor and configure TI DS250 series retimer.
It was tested with DS250DF410 (4-channel) and DS250DF810
(8-channel) 25 Gbps multi-rate retimers from Linux userland.

It is designed to be kernel and board agnostic: the only dependency is
a `/dev/i2c-N` device node and the device's 7-bit SMBus address.

The register models (channel paging via 0xFC/0xFF, HEO/VEO at 0x27/0x28,
the eye capture sequence, the status bits in 0x01/0x78) comes
from the TI datasheets SNLS513 (DS250DF810) and  SNLS456 (DS250DF410).

## Build

```sh
meson setup build
meson compile -C build
```

Statically linked cross build:

```sh
meson setup build-aarch64-static --cross-file <cross-file> -Dstatic=true
meson compile -C build-aarch64-static
```

## Usage

```
ds250 <i2c-dev> <addr> <command> [options]
```

## Commands and example output

All examples below are real captures from a DS250DF810 at `0x1a` and a
DS250DF410 at `0x22` on `/dev/i2c-26`.

### probe — identify the device

```
# ds250 /dev/i2c-26 0x1a probe
addr 0x1a: vendor=0x03 (TI)  chan_config_id=0x0c  version=0x32  device_id=0x10  chan/share_ver=0x00
  -> 8 channels (DS250DF810)

# ds250 /dev/i2c-26 0x22 probe
addr 0x22: vendor=0x03 (TI)  chan_config_id=0x0e  version=0x32  device_id=0x10  chan/share_ver=0x00
  -> 4 channels (DS250DF410) -- or forced with -n
```

TBC: `CHAN_CONFIG_ID` (`0xEF`) is the only register that distinguishes the
two models (`0x0C` = DF810, `0x0E` = DF410); VERSION/DEVICE_ID are
identical across the family.

### status — per-channel health (all channels, or one with `-c`)

```
# ds250 /dev/i2c-26 0x1a status
ch0  sigdet=1  cdr_lock=1  heo=0.375 UI  veo=206.2 mV  pol_inv=0  lock_loss(sticky)=0  prbs_det=-  [0x01=0x80 0x78=0x30]
ch1  sigdet=1  cdr_lock=0  heo=0.000 UI  veo=0.0 mV  pol_inv=0  lock_loss(sticky)=0  prbs_det=-  [0x01=0x80 0x78=0x20]
ch2  sigdet=1  cdr_lock=1  heo=0.375 UI  veo=231.2 mV  pol_inv=0  lock_loss(sticky)=0  prbs_det=-  [0x01=0x80 0x78=0x30]
ch7  sigdet=1  cdr_lock=1  heo=0.125 UI  veo=31.2 mV  pol_inv=0  lock_loss(sticky)=0  prbs_det=-  [0x01=0x80 0x78=0x30]
```

### heo — quick eye-opening readout

```
# ds250 /dev/i2c-26 0x1a heo -c 0
ch0: HEO = 12/32 = 0.375 UI   VEO = 66 x 3.125 = 206.2 mV
```

### eye — full eye capture (ASCII art on the console + optional CSV)

You can add `-o/--output` to dump the raw counts as CSV
for offline plotting.

A healthy channel — open center:

```
# ds250 /dev/i2c-26 0x1a eye --channel 0 --range 1 --output /tmp/eye_ch0.csv
capturing 64x64 eye on ch0 (8200 reads, be patient)...
ch0 eye: HEO=0.375 UI  VEO=206.2 mV  vrange=+/-200 mV  max_hits=160  open_cells=366/4096
|@@@@@@####@@@@@@@@@@@@@@@@@@@@@@@@@@########@@@@@@@@@@@@@@@@@@@@|
|@@@@############@@@@@@@@@@@@@@@@@@################@@@@@@@@@@@@@@|
|######################@@@@@@@@#########################@@@@@@@@#|
|##o:    oo##############@@######o       o################@@#####|
|         o###############@####o          o###############@@####o|
|o        o#####################o         o###############@@#####|
|####o  oo#################@@#######o  :oo#################@@####|
|######################@@@@@@@@@#######################@@@@@@@@@@|
|@@@@@########@@@@@@@@@@@@@@@@@@@@@@#############@@@@@@@@@@@@@@@@|
+----------------------------------------------------------------+
phase 0..63 (early->late), blank = no hits (open eye)
raw 64x64 hit counts written to /tmp/eye_ch0.csv (rows = voltage 0..63)
```

A marginal channel — nearly closed, zero open cells:

```
# ds250 /dev/i2c-26 0x1a eye -c 7 -r 1
ch7 eye: HEO=0.125 UI  VEO=31.2 mV  vrange=+/-200 mV  max_hits=148  open_cells=0/4096
|#####################@@@@@@@@#########################@@@@@@@###|
|#######################@@@@#############################@@@#####|
|#######ooo############@@@@#############oooo############@@@######|
|#######################@@@##############################@@######|
+----------------------------------------------------------------+
phase 0..63 (early->late), blank = no hits (open eye)
```

(Rows elided in this README; the tool always prints all 32 rendered rows.)

### prbs — error metrics (checker) and pattern generation

```
# ds250 /dev/i2c-26 0x1a prbs -c 3 check
ch3: PRBS checker on (auto-detect pattern), counters zeroed, read with 'prbs -c 3 errors'

# ds250 /dev/i2c-26 0x1a prbs -c 3 errors
ch3: prbs_det=-  errors=510

# ds250 /dev/i2c-26 0x1a prbs -c 3 off
ch3: PRBS generator + checker off, datapath restored
```

`check [PATT]` powers the checker's deserializer (`0x0D[7]=0`, it is off by
default) and it enables the checker (`0x79[6]`), auto-detecting the
pattern or forcing one of `7|9|11|15|23|31|58|63` via `0x82`.

`errors` freezes the counter (`0x82[7]`), reads the 11-bit count
(`0x83`/`0x84`; 2047 = saturated) and unfreezes.

`clear` also zeroes it.

`gen [PATT]` (default PRBS31) turns the channel TX into a pattern
source: serializer on (`0x1E[4]`), generator on (`0x79[5]`), and the
post-lock output mux routed to the generator (`0xA5[7:5]=100`),
it replaces the retimed data on the wire.

`off` restores the retimed datapath (`0xA5[7:5]=001`) and powers the PRBS
blocks back to down.

On live non-PRBS traffic the checker simply counts mismatches
(for example the ch3 above), a real BER run needs a PRBS partner at the far end
(for example using `gen` on the peer retimer's channel).

### eq — CTLE boost / adaptation control

```
# ds250 /dev/i2c-26 0x1a eq -c 0
ch0: adapt_mode=1 (CTLE only)  eq_sm_fom=0  rate_sel=5  index_ov=0
ch0: boost(0x03)=0x00 stages 0/0/0/0  override(0x2D[3])=0  ctle_status(0x37)=0x00

# ds250 /dev/i2c-26 0x1a eq -c 0 adapt        # restart CTLE adaptation (0x2F[0])
# ds250 /dev/i2c-26 0x1a eq -c 0 set 0x55     # force boost stages (0x2D[3]=1 + 0x03)
# ds250 /dev/i2c-26 0x1a eq -c 0 auto         # boost back under adaptation control
```

### dfe — 5-tap decision-feedback equalizer

```
# ds250 /dev/i2c-26 0x1a dfe -c 0
ch0: dfe=off  taps3-5=off  manual_force(0x15[7])=0
ch0: tap1=-3  tap2=+0  tap3=+0  tap4=+0  tap5=+0  (-=boost, +=attenuate)

# ds250 /dev/i2c-26 0x1a dfe -c 0 on          # 0x1E[3]=0, taps 3-5 via 0x1E[1]=1
# ds250 /dev/i2c-26 0x1a dfe -c 0 off
```

### fir — TX FIR pre/main/post cursors

```
# ds250 /dev/i2c-26 0x1a fir -c 0
ch0: fir_cursors=off  tx_pd=0  pre=-0  main=+26  post=-0

# ds250 /dev/i2c-26 0x1a fir -c 0 set -- -2 26 -4   # signed cursors -> 0x3D/0x3E/0x3F
# ds250 /dev/i2c-26 0x1a fir -c 0 off               # disable pre/post cursors (lower power)
```

Note the `--` marker ends getopt option parsing so negative cursor values
are not mistaken for option flags.

### shared — per-quad shared page (refclk, EEPROM, address strap)

```
# ds250 /dev/i2c-26 0x1a shared
quad0: smbus_addr=0x18  cal_clk_25mhz=detected  eeprom=none/in-progress (attempts=0, read_done=0)  int_ch[3..0]=0000
quad1: smbus_addr=0x18  cal_clk_25mhz=detected  eeprom=none/in-progress (attempts=0, read_done=0)  int_ch[3..0]=0000
```

`cal_clk_25mhz` is `0x0B[6]` REFCLK_DET — if it reads MISSING, the
25 MHz calibration clock input is absent and adaptation/EOM will not
work.

`eeprom` decodes shared `0x11[7:6]` (self-configuration from an
attached EEPROM)

"none/in-progress" with 0 attempts means the device is
host-configured.

### dump / rd / wr — raw and decoded register access

By default, it is a compact hex matrix:

```
# ds250 /dev/i2c-26 0x1a dump -g
globals:
  0xef:  0x0c  0x32  0x10  0x00  0x00  0x00  0x00  0x00
  0xf7:  0x00  0x00  0x00  0x00  0x04  0x01  0x00  0x03
  0xff:  0x01

# ds250 /dev/i2c-26 0x1a dump -c 0 0x00 0x2f
ch0 registers 0x00..0x2f:
  0x00:  0x00  0x80  0xd8  0x00  0x01  0x01  0x01  0x01
  0x08:  0x73  0x00  0x00  0x63  0x00  0x80  0x93  0x69
  ...
```

With `-v/--verbose` the dump is decoded with the registers
we could find from the TI public documentation.

```
# ds250 /dev/i2c-26 0x1a dump -c 0 -v 0x2f 0x31
ch0 registers 0x2f..0x31 (decoded):
  0x2f = 0x54  rate select / CTLE adapt
        [6:4]  RATE                   = 5         PPM/divider standard-rate group (TI prog guide)
        [3]    INDEX_OV               = 0         1: 0x39[3:0] indexes EQ array 0x40..0x4F
        [2]    EN_PPM_CHECK           = 1         PPM check as lock-detect qualifier
        [0]    CTLE_ADAPT             = 0         restart CTLE adaptation (self-clearing)
  0x30 = 0x00  PRBS generator config / PPM freeze
        [7]    FREEZE_PPM_CNT         = 0         freeze PPM counter for safe read
        [3]    PRBS_EN_DIG_CLK        = 0         clock for PRBS gen/checker (toggle = reset)
        [1:0]  PRBS_PATTERN_SEL[1:0]  = 0         generator pattern LSBs (MSB 0x2E[2])
  0x31 = 0x20  adaptation mode / interrupt enables
        [6:5]  ADAPT_MODE             = 1        (CTLE only)
        [4:3]  EQ_SM_FOM              = 0        (HEO+VEO)  CTLE adapt figure of merit
        ...

# ds250 /dev/i2c-26 0x1a dump -g -v
globals (decoded):
  0xef = 0x0c  device family ID
        [3:0]  CHAN_CONFIG_ID         = 12       (DS250DF810)  TI device ID (quad count)
  ...
  0xff = 0x01  page control
        [5]    EN_SHARE_Q1            = 0         select quad-1 shared regs (DF810 only)
        [4]    EN_SHARE_Q0            = 0         select quad-0 shared regs
        [1]    WRITE_ALL_CH           = 0         broadcast writes to all 0xFC channels
        [0]    EN_CH_SMB              = 1         1: channel regs, 0: shared regs
```

`-s/--shared` addresses the shared page (quad selected with `-c`,
default 0) for `dump`, `rd` and `wr`:

```
# ds250 /dev/i2c-26 0x1a dump -s -v 0x0b 0x11
quad0 shared registers 0x0b..0x11 (decoded):
  0x0b = 0x40  refclk detect
        [6]    REFCLK_DET             = 1         25MHz detected on CAL_CLK_IN (RO)
        [3]    MR_REFCLK_DET_DIS      = 0         1: disable refclk detection
  ...
  0x11 = 0x00  EEPROM self-load status
        [7:6]  EECFG                  = 0        (in progress/none)  EEPROM self-load status
        [5:0]  EECFG_ATMPT            = 0x00/0    EEPROM load attempts
```

`rd` decodes automatically whenever the page is known (`-c` channel,
`-s` shared, or a global address `>= 0xEF`), with computed
interpretations for the eye registers

`wr` decodes the readback:

```
# ds250 /dev/i2c-26 0x1a rd 0x78 -c 7
  0x78 = 0x30  link status / sticky interrupts
        [5]    SD_STATUS              = 1         signal detect (primary observation)
        [4]    CDR_LOCK_STATUS        = 1         CDR lock (primary observation)
        [3]    CDR_LOCK_INT           = 0         sticky lock-acquired, clears on read (en 0x79[1])
        [2]    SD_INT                 = 0         sticky sigdet-change, clears on read (en 0x79[0])
        [1]    EOM_VRANGE_LIMIT       = 0         HEO/VEO hit vrange limit during adapt
        [0]    HEO_VEO_INT            = 0         eye below 0x76 limits (en 0x36[6])

# ds250 /dev/i2c-26 0x1a rd 0x27 -c 7
  0x27 = 0x04  horizontal eye opening
        [7:0]  HEO                    = 0x04/4    horizontal eye opening, /32 = UI (bit8 in 0x29[0])
        -> HEO = 4/32 = 0.125 UI

# ds250 /dev/i2c-26 0x1a wr 0xfc 0x01
0xfc <- 0x01 (reads back 0x01)
  0xfc = 0x01  channel select
        [7:0]  EN_CH[7:0]             = 0x01/1    channel-select bitmask (CH7..4 DF810 only)
```

Beware that decoding registers with sticky clear-on-read bits (channel
`0x01`, `0x71`, `0x78`) consumes those bits, like any read does.

### reset — per-channel reset domains

```
# ds250 /dev/i2c-26 0x22 reset -c 0
ch0: reset bits 0x0a pulsed
```

Domains (channel `0x00` bits): `core` (0x08), `regs` (0x04), `vco`
(0x02), `refclk` (0x01), `all` (0x0F); the default pulses core+vco
(0x0A).

## Testing

```
tests/testplan.sh /dev/i2c-26 0x1a 0x22 0x18
```

Last run: 77/77 PASS (see the Results section in `TESTPLAN.md`).

## TODO

Some registers may be missing like PPM frequency-error, CDR
debug metrics, or the EEPROM self-load logics.
