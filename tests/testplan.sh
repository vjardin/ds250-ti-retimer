#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Free Mobile - Vincent Jardin
#
# ds250-tool datasheet test-plan runner
#
#   testplan.sh <i2c-dev> <live-addr> [quiet-addr] [second-810-addr]
#
# <live-addr>  device with active lanes (locked channels preferred)
# [quiet-addr] all-quiet device, safe for register-write tests
# Prints PASS/FAIL/SKIP/INFO per check; exit 0 iff no FAIL.

BUS=${1:?i2c-dev}
LIVEDEV=${2:?live device address}
QDEV=$3
DEV810B=$4

PASS=0; FAIL=0; SKIP=0
OUT=""; RC=0

r() { OUT=$(ds250 "$@" 2>&1); RC=$?; }

ok()   { PASS=$((PASS + 1)); echo "PASS $1"; }
fail() { FAIL=$((FAIL + 1)); echo "FAIL $1 (rc=$RC)"; echo "$OUT" | sed 's/^/     | /' | head -6; }
skip() { SKIP=$((SKIP + 1)); echo "SKIP $1"; }

# chk ID expected-rc grep-pattern, it evaluates the last r() call
chk() {
    if [ "$RC" = "$2" ] && echo "$OUT" | grep -q "$3"; then
        ok "$1"
    else
        fail "$1 [want rc=$2 pat='$3']"
    fi
}

# hex value of one register: rdv ADDR REG [CH]
rdv() {
    if [ -n "$3" ]; then
        ds250 "$BUS" "$1" rd "$2" -c "$3" 2>/dev/null | head -1 | awk '{print $3}'
    else
        ds250 "$BUS" "$1" rd "$2" 2>/dev/null | head -1 | awk '{print $3}'
    fi
}

echo "== ds250 test plan: bus=$BUS live=$LIVEDEV quiet=${QDEV:-none} =="

# channel-role discovery on the live device
ROLES=$(ds250 "$BUS" "$LIVEDEV" status 2>/dev/null | awk '
/^ch/ {
    ch = substr($1, 3) + 0
    lk = ($3 == "cdr_lock=1")
    split($4, h, "="); heo = h[2] + 0
    if (lk && heo > bh) { bh = heo; live = ch; found = 1 }
    if (lk && (mh == "" || heo < mh)) { mh = heo; marg = ch }
    if (!lk && q == "") q = ch
}
END {
    if (!found) live = -1
    if (mh == "") marg = -1
    if (q == "") q = -1
    print live, marg, q
}')
LIVE=${ROLES%% *}; rest=${ROLES#* }; MARG=${rest%% *}; QCH=${rest##* }
echo "== roles: LIVE=ch$LIVE MARG=ch$MARG QUIET=ch$QCH =="

# A: identification
r "$BUS" "$LIVEDEV" probe;                  chk A1 0 "DS250DF810"
if [ -n "$DEV810B" ]; then
    r "$BUS" "$DEV810B" probe;              chk A2 0 "DS250DF810"
else skip "A2 (no second DF810)"; fi
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" probe;                 chk A3 0 "4 channels"
    r "$BUS" "$QDEV" rd 0xEF;               chk A4 0 "(DS250DF410)"
else skip A3; skip A4; fi
r "$BUS" 0x20 probe
if [ "$RC" = 1 ]; then ok A5; else fail "A5 [want rc=1]"; fi
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" probe -n 8;            chk A6a 0 "8 channels"
    r "$BUS" "$QDEV" probe -n 5
    if [ "$RC" = 2 ]; then ok A6b; else fail "A6b [want rc=2]"; fi
else skip A6a; skip A6b; fi

# B: paging
if [ "$LIVE" -ge 0 ] && [ "$MARG" -ge 0 ] && [ "$LIVE" != "$MARG" ]; then
    H1=$(rdv "$LIVEDEV" 0x27 "$LIVE"); H2=$(rdv "$LIVEDEV" 0x27 "$MARG")
    if [ "$H1" != "$H2" ] && [ $((H1)) -gt $((H2)) ]; then ok B1; else
        RC=-; OUT="LIVE=$H1 MARG=$H2"; fail B1; fi
else skip "B1 (need two distinct locked channels)"; fi
V=$(rdv "$LIVEDEV" 0xF0 "$MARG")
if [ "$V" = "0x32" ]; then ok B2; else RC=-; OUT="0xF0=$V"; fail B2; fi
r "$BUS" "$LIVEDEV" dump -s -c 0 0x0b 0x0b; chk B3a 0 "quad0"
r "$BUS" "$LIVEDEV" dump -s -c 1 0x0b 0x0b; chk B3b 0 "quad1"
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" dump -s -c 1 0x0b 0x0b
    if [ "$RC" = 2 ]; then ok B4; else fail "B4 [want rc=2]"; fi
else skip B4; fi

# C: status/observability
r "$BUS" "$LIVEDEV" status;                 chk C1 0 "ch7 "
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" status
    if [ "$RC" = 1 ] && echo "$OUT" | grep -q "sigdet=0"; then ok C2; else fail C2; fi
else skip C2; fi
if [ "$LIVE" -ge 0 ] && [ "$QCH" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" status -c "$LIVE";  chk C3a 0 "cdr_lock=1"
    r "$BUS" "$LIVEDEV" status -c "$QCH";   chk C3b 0 "cdr_lock=0"
else skip C3a; skip C3b; fi
if [ "$LIVE" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" heo -c "$LIVE"
    HEOV=$(echo "$OUT" | sed -n 's/.*= \([0-9.]*\) UI.*/\1/p')
    if [ "$RC" = 0 ] && awk -v h="$HEOV" 'BEGIN{exit !(h >= 0.20 && h <= 0.50)}'; then
        ok C4
    else fail "C4 [HEO=$HEOV]"; fi
else skip C4; fi
# C5: sticky lock-loss (0x01[5], en 0x31[1]) + relock after vco reset
if [ "$MARG" -ge 0 ]; then
    S31=$(rdv "$LIVEDEV" 0x31 "$MARG")
    ds250 "$BUS" "$LIVEDEV" wr 0x31 $((S31 | 2)) -c "$MARG" >/dev/null 2>&1
    ds250 "$BUS" "$LIVEDEV" rd 0x01 -c "$MARG" >/dev/null 2>&1   # arm: clear stale stickies
    ds250 "$BUS" "$LIVEDEV" reset -c "$MARG" vco >/dev/null 2>&1
    V1=$(rdv "$LIVEDEV" 0x01 "$MARG"); V2=$(rdv "$LIVEDEV" 0x01 "$MARG")
    if [ $((V1 & 0x20)) -ne 0 ]; then ok C5a; else RC=-; OUT="0x01=$V1"; fail C5a; fi
    if [ $((V2 & 0x20)) -eq 0 ]; then ok C5b; else RC=-; OUT="0x01=$V2"; fail C5b; fi
    ds250 "$BUS" "$LIVEDEV" wr 0x31 "$S31" -c "$MARG" >/dev/null 2>&1
    n=0; RELOCK=0
    while [ $n -lt 10 ]; do
        L=$(rdv "$LIVEDEV" 0x78 "$MARG")
        if [ $((L & 0x10)) -ne 0 ]; then RELOCK=1; break; fi
        sleep 1; n=$((n + 1))
    done
    if [ "$RELOCK" = 1 ]; then ok C5c; else RC=-; OUT="0x78=$L after ${n}s"; fail C5c; fi
else skip C5a; skip C5b; skip C5c; fi

# D: reset domains
if [ "$QCH" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" reset -c "$QCH";    chk D1a 0 "pulsed"
    V=$(rdv "$LIVEDEV" 0x00 "$QCH")
    if [ "$V" = "0x00" ]; then ok D1b; else RC=-; OUT="0x00=$V"; fail D1b; fi
else skip D1a; skip D1b; fi
if [ -n "$QDEV" ]; then
    ds250 "$BUS" "$QDEV" wr 0x32 0xAA -c 0 >/dev/null 2>&1
    V=$(rdv "$QDEV" 0x32 0)
    if [ "$V" = "0xaa" ]; then ok D2a; else RC=-; OUT="0x32=$V"; fail D2a; fi
    ds250 "$BUS" "$QDEV" reset -c 0 regs >/dev/null 2>&1
    V=$(rdv "$QDEV" 0x32 0)
    if [ "$V" = "0x11" ]; then ok D2b; else RC=-; OUT="0x32=$V (want default 0x11)"; fail D2b; fi
else skip D2a; skip D2b; fi

# E: raw access + decoder
if [ -n "$QDEV" ]; then
    SV=$(rdv "$QDEV" 0x33 0)
    r "$BUS" "$QDEV" wr 0x33 0x5A -c 0;     chk E1 0 "reads back 0x5a"
    ds250 "$BUS" "$QDEV" wr 0x33 "$SV" -c 0 >/dev/null 2>&1
else skip E1; fi
r "$BUS" "$LIVEDEV" dump -c 0 0x00 0x0f
if [ "$RC" = 0 ] && [ "$(echo "$OUT" | grep -c '^  0x0')" = 2 ]; then ok E2; else fail E2; fi
r "$BUS" "$LIVEDEV" dump -c "${LIVE:-0}" -v 0x1e 0x1e; chk E3 0 "PFD_SEL_DATA_PRELCK"
r "$BUS" "$LIVEDEV" rd 0x05 -c 0;           chk E4 0 "no documented fields"
r "$BUS" "$LIVEDEV" dump -g -v;             chk E5a 0 "(DS250DF810)"
echo "$OUT" | grep -q "EN_CH_SMB" && ok E5b || { RC=-; fail E5b; }
r "$BUS" "$LIVEDEV" rd 0x27 -c "${LIVE:-0}"; chk E6 0 "\-> HEO ="

# F: shared page
r "$BUS" "$LIVEDEV" shared
if [ "$RC" = 0 ] && [ "$(echo "$OUT" | grep -c cal_clk_25mhz=detected)" = 2 ]; then
    ok F1; else fail F1; fi
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" shared;                chk F2 0 "cal_clk_25mhz=detected"
    echo "$OUT" | grep -q "eeprom=none" && ok F4 || { RC=-; fail F4; }
else skip F2; skip F4; fi
r "$BUS" "$LIVEDEV" dump -s -v 0x0b 0x0b;   chk F3 0 "REFCLK_DET *= 1"
r "$BUS" "$LIVEDEV" dump -s -v 0x00 0x00
echo "INFO F5 SMBUS_ADDR strap readback: $(echo "$OUT" | grep SMBUS_ADDR | awk '{print $4}')"

# G: eye monitor
if [ "$LIVE" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" eye -c "$LIVE" -r 1 -o /tmp/tp_eye.csv
    EH=$(echo "$OUT" | sed -n 's/.*HEO=\([0-9.]*\) UI.*/\1/p')
    OC=$(echo "$OUT" | sed -n 's/.*open_cells=\([0-9]*\)\/.*/\1/p')
    NR=$(echo "$OUT" | grep -c '^|')
    if [ "$RC" = 0 ] && [ "$NR" = 32 ] && [ "${OC:-0}" -gt 0 ] &&
       awk -v h="$EH" 'BEGIN{exit !(h >= 0.2)}'; then ok G1; else
        RC=$RC; OUT="HEO=$EH open=$OC rows=$NR"; fail G1; fi
    ROWS=$(wc -l < /tmp/tp_eye.csv)
    COLS=$(awk -F, 'NR==1{print NF}' /tmp/tp_eye.csv)
    if [ "$ROWS" = 64 ] && [ "$COLS" = 64 ]; then ok G2; else
        RC=-; OUT="csv ${ROWS}x${COLS}"; fail G2; fi
else skip G1; skip G2; fi
if [ "$MARG" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" eye -c "$MARG" -r 0
    MV=$(echo "$OUT" | sed -n 's/.*VEO=\([0-9.]*\) mV.*/\1/p')
    if [ "$RC" = 0 ] && echo "$OUT" | grep -q "vrange=+/-100 mV" &&
       awk -v v="$MV" 'BEGIN{exit !(v <= 100)}'; then ok G3; else
        OUT="VEO=$MV"; fail G3; fi
else skip G3; fi
if [ "$LIVE" -ge 0 ] && [ -n "$EH" ]; then
    r "$BUS" "$LIVEDEV" heo -c "$LIVE"
    SH=$(echo "$OUT" | sed -n 's/.*= \([0-9.]*\) UI.*/\1/p')
    if awk -v a="$EH" -v b="$SH" 'BEGIN{d=a-b; if (d<0) d=-d; exit !(d <= 0.11)}'; then
        ok G4; else RC=-; OUT="eye=$EH heo=$SH"; fail G4; fi
else skip G4; fi

# H: PRBS
if [ "$QCH" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" prbs -c "$QCH" check; chk H1a 0 "checker on"
    V=$(rdv "$LIVEDEV" 0x79 "$QCH")
    [ $((V & 0x40)) -ne 0 ] && ok H1b || { RC=-; OUT="0x79=$V"; fail H1b; }
    V=$(rdv "$LIVEDEV" 0x0D "$QCH")
    [ $((V & 0x80)) -eq 0 ] && ok H1c || { RC=-; OUT="0x0D=$V"; fail H1c; }
    V=$(rdv "$LIVEDEV" 0x30 "$QCH")
    [ $((V & 0x08)) -ne 0 ] && ok H1d || { RC=-; OUT="0x30=$V"; fail H1d; }
    r "$BUS" "$LIVEDEV" prbs -c "$QCH" errors
    echo "$OUT" | grep -q "errors=" && ok H2a || fail H2a
    r "$BUS" "$LIVEDEV" prbs -c "$QCH" errors
    echo "$OUT" | grep -q "errors=" && ok H2b || fail H2b
    r "$BUS" "$LIVEDEV" prbs -c "$QCH" check 31
    V=$(rdv "$LIVEDEV" 0x82 "$QCH")
    if [ $((V & 0x20)) -ne 0 ] && [ $(((V >> 2) & 7)) = 5 ]; then ok H3; else
        RC=-; OUT="0x82=$V"; fail H3; fi
    ds250 "$BUS" "$LIVEDEV" prbs -c "$QCH" off >/dev/null 2>&1
    ds250 "$BUS" "$LIVEDEV" wr 0x82 0x00 -c "$QCH" >/dev/null 2>&1
else skip H1a; skip H1b; skip H1c; skip H1d; skip H2a; skip H2b; skip H3; fi
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" prbs -c 2 gen 7;       chk H4a 0 "generator on"
    V=$(rdv "$QDEV" 0x79 2)
    [ $((V & 0x20)) -ne 0 ] && ok H4b || { RC=-; OUT="0x79=$V"; fail H4b; }
    V=$(rdv "$QDEV" 0x1E 2)
    [ $((V & 0x10)) -ne 0 ] && ok H4c || { RC=-; OUT="0x1E=$V"; fail H4c; }
    r "$BUS" "$QDEV" rd 0xA5 -c 2;          chk H4d 0 "PRBS/fixed gen"
    r "$BUS" "$QDEV" prbs -c 2 off;         chk H5a 0 "datapath restored"
    r "$BUS" "$QDEV" rd 0xA5 -c 2;          chk H5b 0 "(retimed)"
    V=$(rdv "$QDEV" 0x79 2)
    [ $((V & 0x60)) -eq 0 ] && ok H5c || { RC=-; OUT="0x79=$V"; fail H5c; }
    V=$(rdv "$QDEV" 0x0D 2)
    [ $((V & 0x80)) -ne 0 ] && ok H5d || { RC=-; OUT="0x0D=$V"; fail H5d; }
    V=$(rdv "$QDEV" 0x1E 2)
    [ $((V & 0x10)) -eq 0 ] && ok H5e || { RC=-; OUT="0x1E=$V"; fail H5e; }
else skip H4a; skip H4b; skip H4c; skip H4d
     skip H5a; skip H5b; skip H5c; skip H5d; skip H5e; fi

# I: CTLE EQ
r "$BUS" "$LIVEDEV" eq -c "${LIVE:-0}";     chk I1 0 "adapt_mode="
if [ -n "$QDEV" ]; then
    SB=$(rdv "$QDEV" 0x03 1)
    r "$BUS" "$QDEV" eq -c 1 set 0xE4;      chk I2a 0 "stages 3/2/1/0"
    V=$(rdv "$QDEV" 0x03 1); OV=$(rdv "$QDEV" 0x2D 1)
    if [ "$V" = "0xe4" ] && [ $((OV & 8)) -ne 0 ]; then ok I2b; else
        RC=-; OUT="0x03=$V 0x2D=$OV"; fail I2b; fi
    r "$BUS" "$QDEV" eq -c 1 auto;          chk I2c 0 "adaptation control"
    OV=$(rdv "$QDEV" 0x2D 1)
    [ $((OV & 8)) -eq 0 ] && ok I2d || { RC=-; OUT="0x2D=$OV"; fail I2d; }
    ds250 "$BUS" "$QDEV" wr 0x03 "$SB" -c 1 >/dev/null 2>&1
else skip I2a; skip I2b; skip I2c; skip I2d; fi
if [ "$LIVE" -ge 0 ]; then
    r "$BUS" "$LIVEDEV" eq -c "$LIVE" adapt; chk I3a 0 "restarted"
    sleep 1
    V=$(rdv "$LIVEDEV" 0x2F "$LIVE")
    [ $((V & 1)) -eq 0 ] && ok I3b || { RC=-; OUT="0x2F=$V"; fail I3b; }
    n=0; RELOCK=0
    while [ $n -lt 10 ]; do
        L=$(rdv "$LIVEDEV" 0x78 "$LIVE")
        if [ $((L & 0x10)) -ne 0 ]; then RELOCK=1; break; fi
        sleep 1; n=$((n + 1))
    done
    [ "$RELOCK" = 1 ] && ok I3c || { RC=-; OUT="0x78=$L"; fail I3c; }
else skip I3a; skip I3b; skip I3c; fi

# J: DFE
r "$BUS" "$LIVEDEV" dfe -c "${LIVE:-0}";    chk J1 0 "tap1="
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" dfe -c 3 on;           chk J2a 0 "DFE enabled"
    V=$(rdv "$QDEV" 0x1E 3)
    if [ $((V & 8)) -eq 0 ] && [ $((V & 2)) -ne 0 ]; then ok J2b; else
        RC=-; OUT="0x1E=$V"; fail J2b; fi
    r "$BUS" "$QDEV" dfe -c 3 off;          chk J2c 0 "DFE disabled"
    V=$(rdv "$QDEV" 0x1E 3)
    [ $((V & 8)) -ne 0 ] && ok J2d || { RC=-; OUT="0x1E=$V"; fail J2d; }
else skip J2a; skip J2b; skip J2c; skip J2d; fi

# K: TX FIR
r "$BUS" "$LIVEDEV" fir -c "${LIVE:-0}";    chk K1 0 "main=+26"
if [ -n "$QDEV" ]; then
    S3D=$(rdv "$QDEV" 0x3D 0); S3E=$(rdv "$QDEV" 0x3E 0); S3F=$(rdv "$QDEV" 0x3F 0)
    r "$BUS" "$QDEV" fir -c 0 set -- -2 26 -4; chk K2a 0 "pre=-2 main=26 post=-4"
    r "$BUS" "$QDEV" fir -c 0
    echo "$OUT" | grep -q "fir_cursors=on.*pre=-2.*main=+26.*post=-4" && ok K2b || fail K2b
    r "$BUS" "$QDEV" fir -c 0 off;          chk K2c 0 "disabled"
    ds250 "$BUS" "$QDEV" wr 0x3D "$S3D" -c 0 >/dev/null 2>&1
    ds250 "$BUS" "$QDEV" wr 0x3E "$S3E" -c 0 >/dev/null 2>&1
    ds250 "$BUS" "$QDEV" wr 0x3F "$S3F" -c 0 >/dev/null 2>&1
    r "$BUS" "$QDEV" fir -c 0 set -- -20 26 -4
    [ "$RC" = 2 ] && ok K3 || fail "K3 [want rc=2]"
else skip K2a; skip K2b; skip K2c; skip K3; fi

# L: CLI
r "$BUS" "$LIVEDEV" probe --help;           chk L1 0 "Usage"
OUT=$(ds250 2>&1); RC=$?
[ "$RC" = 2 ] && ok L2 || fail "L2 [want rc=2]"
r "$BUS" "$LIVEDEV" eye
[ "$RC" = 2 ] && ok L3 || fail "L3 [want rc=2]"
if [ -n "$QDEV" ]; then
    r "$BUS" "$QDEV" status -c 9
    [ "$RC" = 2 ] && ok L4 || fail "L4 [want rc=2]"
else skip L4; fi

echo "== summary: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" = 0 ]
