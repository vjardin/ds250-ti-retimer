# ds250-tool — datasheet-based test plan

It is a validation plan for every tool feature against the behavior documented
in the TI datasheets: DS250DF410 (SNLS456) and DS250DF810 (SNLS513).

## Test environment

- One or more DS250DFx10 devices on a Linux I2C bus. The reference run
  uses a DS250DF810 with live 25G lanes (a mix of CDR-locked and
  signal-only channels) plus a second, all-quiet device (DF410 or
  DF810) that is safe for write tests.
- Runner: use `testplan.sh <i2c-dev> <live-addr> [quiet-addr] [second-810-addr]`

### Channel-role discovery (self-configuring)

The runner parses `status` on the live device and derives:

| role  | definition                                    | used by                          |
|-------|-----------------------------------------------|----------------------------------|
| LIVE  | locked channel with the largest HEO           | read-only + adaptation tests     |
| MARG  | locked channel with the smallest HEO          | eye capture, sticky/relock test  |
| QUIET | first channel without CDR lock                | PRBS checker state tests         |

Write tests run on the quiet device only. If no locked channel exists,
the lock-dependent tests report `SKIP`.

### Safety rules, to be changed, why do we care ?

1. Register writes only on the quiet device or on unlocked channels of
   the live device, except: the sticky/relock test (C5) and the CTLE
   re-adaptation test (I3), which intentionally disturb a locked lane
   and verify it relocks.
2. Every test that changes a register saves the prior value and
   restores it (or proves the reset-to-default path, D2).
3. The PRBS generator is only ever enabled on the quiet device (its TX
   goes to an unused lane); `off` must restore the retimed datapath.

## Test matrix

### A — device identification (Table 8: 0xEF/0xF0/0xF1/0xFE)

| ID | Feature                       | Procedure                     | Expected                                            |
|----|-------------------------------|-------------------------------|-----------------------------------------------------|
| A1 | DF810 identification          | `probe` on live DF810         | `vendor=0x03 (TI)`, `-> 8 channels (DS250DF810)`, rc 0 |
| A2 | second DF810                  | `probe` on second DF810       | same, rc 0                                          |
| A3 | DF410 identification          | `probe` on DF410              | `chan_config_id=0x0e`, `-> 4 channels`, rc 0        |
| A4 | CHAN_CONFIG_ID decode         | `rd 0xEF` on DF410            | decoded line contains `(DS250DF410)`                |
| A5 | absent device                 | `probe` at an empty address   | error (`no response` or `NOT TI`), rc 1             |
| A6 | `-n` channel-count force      | `probe -n 8` on DF410; `-n 5` | reports 8 channels; `-n 5` rejected with rc 2       |

### B — register paging (Table 8: 0xFC/0xFF quad+channel select)

| ID | Feature                       | Procedure                                   | Expected                                  |
|----|-------------------------------|---------------------------------------------|-------------------------------------------|
| B1 | channel paging isolates regs  | `rd 0x27` on LIVE vs MARG                   | different HEO values (LIVE > MARG)        |
| B2 | globals visible from any page | `rd 0xF0` after selecting a channel         | still `0x32` (VERSION)                    |
| B3 | DF810 quad paging             | `dump -s -c 0` and `dump -s -c 1`           | both quads readable                       |
| B4 | quad range check on DF410     | `dump -s -c 1` on DF410                     | `quad 1 out of range`, rc 2               |

### C — status & observability (Table 10: 0x01; Table 11: 0x78; 0x27/0x28)

| ID | Feature                          | Procedure                                                      | Expected                                                       |
|----|----------------------------------|----------------------------------------------------------------|----------------------------------------------------------------|
| C1 | per-channel status, healthy dev  | `status` on live DF810                                         | 8 rows, all `sigdet=1`, rc 0                                   |
| C2 | status exit code, quiet dev      | `status` on quiet device                                       | `sigdet=0` rows present, rc 1                                  |
| C3 | live CDR lock split (0x78[4])    | `status -c LIVE` / `-c QUIET`                                  | `cdr_lock=1` vs `cdr_lock=0`                                   |
| C4 | HEO/VEO scaling (0x27/0x28)      | `heo -c LIVE`                                                  | HEO within 0.20–0.50 UI (sane live value)                      |
| C5 | sticky lock-loss + relock        | enable `0x31[1]`; `reset -c MARG vco`; read `0x01` twice; poll `0x78[4]`; restore | 1st read `[5]=1`, 2nd read `[5]=0` (clears on read); relocks |

### D — reset domains (Table 10: 0x00)

| ID | Feature                         | Procedure                                             | Expected                                    |
|----|---------------------------------|-------------------------------------------------------|---------------------------------------------|
| D1 | reset pulse leaves channel sane | `reset` (core+vco) on an unlocked live-device channel | rc 0; `0x00` reads back `0x00`              |
| D2 | RST_REGS restores defaults      | quiet dev: `wr 0x32 0xAA`, verify, `reset regs`, read | `0x32` back to datasheet default `0x11`     |

### E — raw access + decoder (Tables 8/9/10/11 field definitions)

| ID | Feature                        | Procedure                          | Expected                                                    |
|----|--------------------------------|------------------------------------|-------------------------------------------------------------|
| E1 | wr + readback                  | quiet dev: save `0x33`, `wr 0x33 0x5A`, restore | `reads back 0x5a`                              |
| E2 | dump matrix format             | `dump -c 0 0x00 0x0f`              | two rows of eight values                                    |
| E3 | decoded dump, enums            | `dump -c LIVE -v 0x1e 0x1e`        | `PFD_SEL_DATA_PRELCK` line with `(mute)` + field breakdown  |
| E4 | unknown-register fallback      | `rd 0x05 -c 0`                     | `(no documented fields)`                                    |
| E5 | global decode                  | `dump -g -v` on DF810              | `(DS250DF810)` enum + `EN_CH_SMB` field                     |
| E6 | computed interpretation        | `rd 0x27 -c LIVE`                  | `-> HEO = N/32 = x UI` line                                 |

### F — shared page (Table 9: 0x00/0x05/0x08/0x0B/0x11)

| ID | Feature                     | Procedure                    | Expected                                          |
|----|-----------------------------|------------------------------|---------------------------------------------------|
| F1 | shared summary, DF810       | `shared` on live DF810       | 2 quad rows, `cal_clk_25mhz=detected` on both     |
| F2 | shared summary, DF410       | `shared` on DF410            | 1 quad row, CAL_CLK detected                      |
| F3 | REFCLK_DET decode (0x0B[6]) | `dump -s -v 0x0b 0x0b`       | `REFCLK_DET = 1`                                  |
| F4 | EEPROM self-load status     | `shared` output              | `eeprom=none/in-progress (attempts=0 ...)`        |
| F5 | SMBUS_ADDR strap readback   | `dump -s -v 0x00 0x00`       | INFO only — record silicon value vs strap         |

### G — eye monitor (Table 4 procedure; 0x11/0x22-0x29 EOM registers)

| ID | Feature                        | Procedure                                | Expected                                                      |
|----|--------------------------------|------------------------------------------|---------------------------------------------------------------|
| G1 | full eye, healthy channel      | `eye -c LIVE -r 1 -o csv`                | HEO ≥ 0.2 UI, `open_cells` > 0, 32 ASCII rows + footer        |
| G2 | CSV export shape               | inspect CSV from G1                      | 64 rows × 64 comma-separated values                           |
| G3 | vrange select (0x11[7:6])      | `eye -c MARG -r 0`                       | completes; banner says `vrange=+/-100 mV`; VEO ≤ 100 mV       |
| G4 | HEO consistency eye vs status  | compare G1 HEO with `heo -c LIVE`        | difference ≤ 0.11 UI (values update continuously)             |

### H — PRBS generator/checker (0x0D, 0x1E[4], 0x30, 0x79, 0x82-0x84, 0xA5)

| ID | Feature                          | Procedure                                | Expected                                                                  |
|----|----------------------------------|------------------------------------------|---------------------------------------------------------------------------|
| H1 | checker bring-up state           | `prbs -c QUIET check`, then rd 0x79/0x0D/0x30 | `PRBS_CHKR_EN=1`, `DES_PD=0` (deserializer powered), `PRBS_EN_DIG_CLK=1` |
| H2 | error counter freeze/read        | `prbs -c QUIET errors` twice             | `errors=N` printed each time (2047 = saturated allowed)                   |
| H3 | forced pattern select            | `prbs -c QUIET check 31`, rd 0x82        | `PRBS_PATT_OV=1`, `PRBS_PATT` decodes `(PRBS31)`                          |
| H4 | generator datapath (quiet dev)   | `prbs -c 2 gen 7`, rd 0x79/0x1E/0xA5     | `PRBS_GEN_EN=1`, `SER_EN=1`, post-lock mux `(PRBS/fixed gen)`             |
| H5 | off restores datapath            | `prbs off` on both channels, rd back     | gen+chk 0, `SER_EN=0`, `DES_PD=1`, mux `(retimed)`                        |

### I — CTLE equalizer (0x03, 0x2D[3], 0x2F, 0x31, 0x37)

| ID | Feature                         | Procedure                                        | Expected                                             |
|----|---------------------------------|--------------------------------------------------|------------------------------------------------------|
| I1 | eq show                         | `eq -c LIVE`                                     | adapt_mode/rate_sel/boost/override lines             |
| I2 | boost force + auto (0x2D/0x03)  | quiet dev: save 0x03; `eq set 0xE4`; `eq auto`   | `stages 3/2/1/0`, `0x03=0xE4`, override 1 then 0     |
| I3 | CTLE_ADAPT self-clear (0x2F[0]) | `eq -c LIVE adapt`; rd 0x2F; poll 0x78[4]        | restart msg; `CTLE_ADAPT=0` after; channel relocks   |

### J — DFE (0x1E[3]/[1], 0x11/0x12/0x20/0x21 taps)

| ID | Feature                | Procedure                                | Expected                                        |
|----|------------------------|------------------------------------------|-------------------------------------------------|
| J1 | dfe show + tap decode  | `dfe -c LIVE`                            | `dfe=off`, five signed taps listed              |
| J2 | dfe on/off             | quiet dev: `dfe on`, rd 0x1E, `dfe off`  | `DFE_PD=0`+`EN_PARTIAL_DFE=1`, then `DFE_PD=1`  |

### K — TX FIR (0x3D/0x3E/0x3F)

| ID | Feature                     | Procedure                                             | Expected                                        |
|----|-----------------------------|-------------------------------------------------------|-------------------------------------------------|
| K1 | fir show, silicon defaults  | `fir -c LIVE`                                         | `main=+26` (datasheet default), cursors=off     |
| K2 | fir set / off (closed loop) | quiet dev: save regs; `fir set -- -2 26 -4`; `off`; restore | show reports `pre=-2 main=+26 post=-4`; then cursors=off |
| K3 | cursor range check          | `fir set -- -20 26 -4`                                | rejected, rc 2                                  |

Note: negative cursors need the POSIX `--` end-of-options marker so
getopt does not eat `-2`/`-4` as option flags.

### L — CLI behavior

| ID | Feature              | Procedure                    | Expected                       |
|----|----------------------|------------------------------|--------------------------------|
| L1 | help                 | `--help` (any device args)   | usage text, rc 0               |
| L2 | missing arguments    | run with no arguments        | usage, rc 2                    |
| L3 | eye needs channel    | `eye` without `-c`           | rc 2                           |
| L4 | channel range check  | `status -c 9` on DF410       | `out of range`, rc 2           |

## Out of scope on this setup (and rationals)

- True BER / PRBS closed loop: it needs a physical loopback or a
  PRBS partner at the far end of a lane
- EEPROM self-configuration (master mode): I do not have such setup
- INT pin behavior: the interrupt output is not integrated from
  the tool, the enables/status bits are covered (C5, H tests).
- Crosspoint routing (0x96): TODO
- CDR_STATUS debug bus (0x02 via 0x0C[7:4]): the 0x0C select values
  are not enumerated in the public datasheet ("Programming Guide").

## How to run

```
testplan.sh /dev/i2c-26 0x1a 0x22 0x18
```

Example:
```
== ds250 test plan: bus=/dev/i2c-26 live=0x1a quiet=0x22 ==
== roles: LIVE=ch4 MARG=ch7 QUIET=ch1 ==
PASS A1 A2 A3 A4 A5 A6a A6b
PASS B1 B2 B3a B3b B4
PASS C1 C2 C3a C3b C4 C5a C5b C5c
PASS D1a D1b D2a D2b
PASS E1 E2 E3 E4 E5a E5b E6
PASS F1 F2 F3 F4          INFO F5 SMBUS_ADDR strap readback: 0
PASS G1 G2 G3 G4
PASS H1a H1b H1c H1d H2a H2b H3
PASS H4a H4b H4c H4d H5a H5b H5c H5d H5e
PASS I1 I2a I2b I2c I2d I3a I3b I3c
PASS J1 J2a J2b J2c J2d
PASS K1 K2a K2b K2c K3
PASS L1 L2 L3 L4
== summary: PASS=77 FAIL=0 SKIP=0 ==
```

The total run time is about 8 minutes, most of the time is spent
with the two 64×64 eye captures and the two relock polls.
