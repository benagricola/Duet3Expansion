# TmcSPI: TMC2240 vs TMC5160 Complete Register Difference (Datasheet-Verified)

This document lists every register address in the TMC2240 and TMC5160 families and classifies them as:
- Present on both with compatible semantics
- Present on both with differing semantics/bitfields
- Present only on TMC5160
- Present only on TMC2240

Sources: `docs/tmc_2240_datasheet.txt` (ADI TMC2240 Rev 2, 11/23) and `docs/tmc_5160_datasheet.txt` (Trinamic TMC5160 Rev 1.17, 2022-05-25).

Notes
- All addresses are the SPI register numbers (0x00..0x7F); write accesses add 0x80.
- Vague, generic examples in datasheets may reference registers not present on a given chip—this list is strictly from the chips’ Register Map tables.
- Where fields differ, the exact per-chip bit placement is noted.

## Legend
- Both (same): Present on both, same purpose/bitfields (minor reserved differences ignored)
- Both (diff): Present on both, address matches but fields/meaning differ
- 5160-only: Exists on 5160; not in 2240 map
- 2240-only: Exists on 2240; not in 5160 map

## General Configuration (0x00–0x0F)

- 0x00 GCONF — Both (diff)
  - 5160: recalibrate (bit0), faststandstill(1), en_pwm_mode(2), multistep_filt(3), shaft(4), diag0_error(5), diag0_otpw(6), diag0_stall/diag0_step(7), diag1_stall/diag1_dir(8), diag1_index(9), diag1_onstate(10), diag1_steps_skipped(11), diag pushpull bits are in separate names; direct_mode(15/16 region per table); test_mode present.
  - 2240: fast_standstill(1), en_pwm_mode(2), multistep_filt(3), shaft(4), diag0_error(5), diag0_otpw(6), diag0_stall(7), diag1_stall(8), diag1_index(9), diag1_onstate(10), pushpull selects diag0/diag1 at bits 12/13, stop_enable(15), direct_mode(16). No recalibrate, no test_mode, no diag1_steps_skipped.
- 0x01 GSTAT — Both (same)
  - Bits: reset(0), drv_err(1), uv_cp(2). 2240 adds vm_uvlo(4) and register_reset(3) flags explicitly; semantic superset. Clear-on-write-1 behavior same.
- 0x02 IFCNT — Both (same)
  - UART write counter; disabled in SPI.
- 0x03 — Both (diff)
  - 5160: SLAVECONF (SLAVEADDR, SENDDELAY) for UART node and NEXTADDR.
  - 2240: NODECONF (NODEADDR, SENDDELAY) with slightly different naming; functional equivalent.
- 0x04 IOIN/OUTPUT — Both (diff)
  - 5160: Address 0x04 serves two purposes: reading returns IOIN (pin states, VERSION=0x30); writing controls OUTPUT polarity for SDO/NAO in UART chain mode.
  - 2240: IOIN is at 0x04 with similar purpose (VERSION=0x40). The OUTPUT control is integrated as bit 12 within IOIN and is read/write at the same address.
- 0x05 X_COMPARE — 5160-only
- 0x06 OTP_PROG — 5160-only
- 0x07 OTP_READ — 5160-only
- 0x08 FACTORY_CONF — 5160-only
- 0x09 SHORT_CONF — 5160-only
- 0x0A DRV_CONF — Both (diff)
  - 5160: External gate driver settings: BBMTIME, BBMCLKS, OTSELECT, DRVSTRENGTH, FILT_ISENSE.
  - 2240: Integrated current sense settings: CURRENT_RANGE[1:0] (1A/2A/3A) and SLOPE_CONTROL[1:0] (100/200/400/800 V/μs). All other bits reserved.
- 0x0B GLOBAL_SCALER — Both (same)
- 0x0C OFFSET_READ — 5160-only
- 0x0D..0x0F — 5160-only (no mapped registers on 2240)

## Velocity-Dependent (0x10–0x1F)

- 0x10 IHOLD_IRUN — Both (same)
- 0x11 TPOWERDOWN — Both (same)
- 0x12 TSTEP — Both (same)
- 0x13 TPWMTHRS — Both (same)
- 0x14 TCOOLTHRS — Both (same)
- 0x15 THIGH — Both (same)
- 0x16..0x1F — Unused gap (no mapped registers on either)

## Ramp Generator (0x20–0x2F)

- 5160 has full motion controller at 0x20–0x2C and XTARGET at 0x2D; plus VDCMIN 0x33 and SW_MODE/RAMP_STAT/XLATCH at 0x34–0x36.
- 2240 does not implement a ramp generator block.

- 0x20 RAMPMODE — 5160-only
- 0x21 XACTUAL — 5160-only
- 0x22 VACTUAL — 5160-only
- 0x23 VSTART — 5160-only
- 0x24 A1 — 5160-only
- 0x25 V1 — 5160-only
- 0x26 AMAX — 5160-only
- 0x27 VMAX — 5160-only
- 0x28 DMAX — 5160-only
- 0x29 — 5160-only (reserved/unused in map)
- 0x2A D1 — 5160-only
- 0x2B VSTOP — 5160-only
- 0x2C TZEROWAIT — 5160-only
- 0x2D DIRECT register — Both (same address, different name)
  - 5160: XTARGET is repurposed when GCONF.direct_mode=1; bits 8:0 coil A, 24:16 coil B signed.
  - 2240: DIRECT_MODE is an explicit register at 0x2D with the same packed fields: DIRECT_COIL_A[8:0], DIRECT_COIL_B[8:0].
- 0x2E DCCTRL — 5160-only
- 0x2F — Unused gap (no mapped register)

## Encoder (0x38–0x3D)

- 0x38 ENCMODE — Both (diff)
  - 5160: includes latch_x_act; edge selection fields are pos_edge/neg_edge; otherwise similar.
  - 2240: enc_sel_decimal, clr_enc_x, pos_neg_edge[1:0] and other flags; semantics are equivalent with naming/packing differences.
- 0x39 X_ENC — Both (same)
- 0x3A ENC_CONST — Both (same)
- 0x3B ENC_STATUS — Both (diff)
  - 5160: n_event (bit0) and deviation_warn (bit1). Clear-on-write-1. Also IRQ ORed.
  - 2240: only n_event bit. No deviation_warn.
- 0x3C ENC_LATCH — Both (same, RO)
- 0x3D ENC_DEVIATION — 5160-only

## Ramp Generator Driver Features (0x30–0x36)

- 0x33 VDCMIN — 5160-only
- 0x34 SW_MODE — 5160-only
- 0x35 RAMP_STAT — 5160-only
- 0x36 XLATCH — 5160-only

## ADC (0x50–0x52/0x53)

- 2240-only block:
  - 0x50 ADC_VSUPPLY_AIN — RO; ADC_VSUPPLY[12:0], ADC_AIN[12:0]
  - 0x51 ADC_TEMP — RO; ADC_TEMP[12:0]
  - 0x52 OTW_OV_VTH — RW; programmable thresholds for OT prewarn and overvoltage
  - 0x53 — not defined in the 2240 map (placeholder in some summaries); ignore
- 5160: no ADC block at these addresses.

## Motor Driver and PWM/StallGuard (0x60–0x76)

- 0x60–0x67 MSLUT[0..7] — Both (same)
- 0x68 MSLUTSEL — Both (same)
- 0x69 MSLUTSTART — Both (same)
- 0x6A MSCNT — Both (same)
- 0x6B MSCURACT — Both (same bit packing: CUR_A[8:0] at 24:16, CUR_B[8:0] at 8:0)
- 0x6C CHOPCONF — Both (diff)
  - 5160: external MOSFET fields like vhighfs/vhighchm, TBL, HEND/HSTRT, MRES, etc. Default 0x10410150.
  - 2240: same register structure and fields for StealthChop/SpreadCycle, but external MOSFET-specific interactions aren’t applicable; defaults differ.
- 0x6D COOLCONF — Both (same)
  - SGT bitfield is [23:16] on both chips; SGT sign and semantics align. Other fields semin/semax/sedn/seup/seimin/sfilt align.
- 0x6E DCCTRL — 5160-only
- 0x6F DRV_STATUS — Both (diff)
  - Both provide SG_RESULT[9:0], CS_ACTUAL[4:0], flags like stst, olb, ola, s2ga/s2gb, otpw, ot, stallguard, stealth, fsactive, s2vsa/s2vsb.
  - Minor naming differences; field positions match datasheet tables for each chip.
- 0x70 PWMCONF — Both (same purpose; field names largely match)
- 0x71 PWM_SCALE — Both (same)
- 0x72 PWM_AUTO — Both (same)
- 0x73 LOST_STEPS — 5160-only
- 0x74 SG4_THRS — 2240-only
- 0x75 SG4_RESULT — 2240-only
- 0x76 SG4_IND — 2240-only

## Summary Tables

Counts (by category):
- Both (same or diff): 0x00, 0x01, 0x02, 0x03, 0x04, 0x0A, 0x0B, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x2D, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x60–0x6D, 0x6F–0x72
- 5160-only: 0x05, 0x06, 0x07, 0x08, 0x09, 0x0C, 0x20–0x2C, 0x2E, 0x33–0x36, 0x6E, 0x73
- 2240-only: 0x50–0x52, 0x74–0x76

Key semantic differences to account for in code:
- GCONF: bit layout changes; diag outputs/SD_MODE coupling on 5160 not present on 2240; remove 5160-only features (recalibrate, test_mode, diag1_steps_skipped) when building for 2240.
- DRV_CONF: external gate driver tuning (5160) vs ICS CURRENT_RANGE and SLOPE_CONTROL (2240).
- COOLCONF: SGT in [23:16] on both.
- DIRECT at 0x2D: identical packing for direct coil currents; safe to unify implementation.
- Ramp generator and DcStep: entire block is 5160-only; must be excluded for 2240 builds; likewise LOST_STEPS.
- ADC and SG4: 2240 adds ADC monitoring and StallGuard4 configuration/results; add diagnostics and setup paths only for 2240.

Acceptance criteria
- No references to 5160-only registers when TMC_TYPE==2240.
- No references to 2240-only registers when TMC_TYPE==5160.
- Shared fields with identical bit locations use common masks (e.g., COOLCONF.SGT).
- DIRECT_MODE/XDIRECT uses a single path targeting 0x2D packing.
- M122 diagnostics reflect chip-specific extras: LOST_STEPS (5160), ADC and SG4_* (2240).
