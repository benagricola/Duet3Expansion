# TMC2240 SPI Driver Implementation Plan (Datasheet-Verified)

This plan adds TMC2240 support to the existing unified `TmcSPI` driver while keeping current support for TMC5160/5161/2160 and TMC5130 unchanged. It is grounded in docs/TmcSPI_2240_vs_5160_Register_Diff.md and the current `TmcSPI.*` code.

## 1) Guardrails and scope

- Do not modify behavior for 5160/5161/2160/5130 unless required for compile-time isolation.
- Only add code for 2240 behind `#if TMC_TYPE == 2240` (or small shared helpers behind neutral names).
- Never reference 5160-only registers in 2240 builds; never reference 2240-only registers in 5160/5130 builds.

## 2) Register contracts to enforce

Shared and stable
- 0x00 GCONF, 0x01 GSTAT, 0x02 IFCNT, 0x10–0x15 (IHOLD_IRUN, TPOWERDOWN, TSTEP, TPWMTHRS, TCOOLTHRS, THIGH), 0x2D DIRECT, 0x60–0x6B (MSLUT*, MSCNT, MSCURACT), 0x6C CHOPCONF, 0x6D COOLCONF, 0x6F DRV_STATUS, 0x70–0x72 (PWM*).
- COOLCONF.SGT is [23:16] on both 2240 and 5160.
- DIRECT at 0x2D uses the same packing: A[8:0] at 8:0, B[8:0] at 24:16.

5160-only (exclude for 2240)
- Motion/ramp and DcStep: 0x20–0x2C, 0x33–0x36, 0x6E, 0x73; also 0x05 X_COMPARE, 0x06 OTP_PROG, 0x07 OTP_READ, 0x08 FACTORY_CONF, 0x09 SHORT_CONF, 0x0C OFFSET_READ.

2240-only (exclude for 5160)
- ADC: 0x50–0x52; SG4: 0x74–0x76.

GCONF and DRV_CONF shape
- 5160 has recalibrate/test_mode/diag1_steps_skipped; 2240 does not.
- 2240 has CURRENT_RANGE and SLOPE_CONTROL in DRV_CONF; 5160 has BBM*, DRVSTRENGTH, FILT_ISENSE, OTSELECT.

## 3) Code changes in `TmcSPI.*`

3.1 Constants and masks
- Add a 2240 section with:
  - REGNUM_ADC_VSUPPLY_AIN=0x50, REGNUM_ADC_TEMP=0x51, REGNUM_OTW_OV_VTH=0x52.
  - REGNUM_SG4_THRS=0x74, REGNUM_SG4_RESULT=0x75, REGNUM_SG4_IND=0x76.
  - DRV_CONF fields: CURRENT_RANGE_SHIFT/MASK, SLOPE_CONTROL_SHIFT/MASK.
- Ensure shared masks use one definition when addresses and bits match (e.g., COOLCONF_SGT_SHIFT=16, COOLCONF_SGT_MASK=0x7F << 16).
- For 5160-only constants already present (SHORT_CONF, DCCTRL, LOST_STEPS, etc.), keep them under `#if TMC_TYPE == 5160`.

3.2 Default GCONF per chip
- 5160/5130/2160: keep as-is.
- 2240: construct `DefaultGConfReg` using only fields that exist on 2240: fast_standstill(1), en_pwm_mode(2), shaft(4), diag0/1 bits, push-pull bits (12/13), stop_enable(15) if desired, direct_mode(16) disabled by default.
- Remove 5160-only bits from the 2240 default (no recalibrate/test_mode/diag1_steps_skipped/multistep_filt differences beyond what the 2240 map supports).

3.3 DRV_CONF write path
- 5160 path: keep existing BBM*, STRENGTH, FILT_ISENSE, OTSELECT programming.
- 2240 path: write CURRENT_RANGE and SLOPE_CONTROL only; gate other 5160 fields out with `#if TMC_TYPE == 5160`.
- Add a board-level way (existing config or new defines) to choose CURRENT_RANGE for 2240.

3.4 StallGuard configuration and APIs
- Keep API surface unchanged: SetStallThreshold, SetStallFilter, SetStallMinimumStepsPerSecond, AppendStallConfig.
- Implementation detail:
  - Write SGT to COOLCONF[23:16] on both chips; preserve sign. Respect `sfilt` on 2240.
  - Keep TCOOLTHRS/THIGH programming common (present on both).
- Status parsing: DRV_STATUS provides SG_RESULT[9:0] on both; report consistently.

3.5 Direct mode
- Reuse current code paths for closed loop/phase stepping that write 0x2D packed current. No chip split needed.
- Ensure the enable bit for direct_mode in GCONF is correctly addressed for 2240.

3.6 ADC and SG4 (2240 only)
- New optional diagnostics for M122:
  - Read 0x50 ADC_VSUPPLY_AIN (report VSUPPLY and AIN), 0x51 ADC_TEMP, 0x52 OTW_OV_VTH settings.
  - Read 0x74–0x76 SG4_THRS/RESULT/IND when available and include a brief SG4 summary.
- All the above must compile out for non-2240 builds.

3.7 Excluding 5160-only features for 2240
- Wrap access to: SHORT_CONF (0x09), OFFSET_READ(0x0C), motion/ramp (0x20–0x2C), DCCTRL(0x6E), LOST_STEPS(0x73), X_COMPARE(0x05), OTP_* (0x06/0x07), FACTORY_CONF(0x08).
- Keep public APIs stable; just no-op or omit when not compiled for that chip.

## 4) Board/config integration

- Keep existing TMC_TYPE selection per board. Valid: 5130, 5160 (covers 5161/2160), 2240.
- For 2240 add board defines for:
  - MaxTmc2240Current, Tmc2240SenseResistor (already referenced in code), and TMC2240_CURRENT_RANGE default.
  - Optional: default SLOPE_CONTROL.
- If a board needs SPI/UART mode strap on boot, use the existing pin init hook (or add a small PinInit table) without changing the M-code surface.

## 5) Testing and validation

Phase 1: Open-loop (SpreadCycle/StealthChop)
- Compile for a 2240 board; verify movement, current setting, microstepping, interpolation.
- M122: verify common registers, DRV_STATUS, CHOPCONF, COOLCONF.

Phase 2: Diagnostics (2240 extras)
- Read and report ADC values and thresholds; sanity check ranges.
- Optionally surface SG4 values in M122 for tuning.

Phase 3: Direct mode
- Enable GCONF.direct_mode, stream 0x2D currents in control loop; check holding torque and response.
- Confirm register packing and timing are identical across 5160 and 2240.

Phase 4: StallGuard
- Tune SGT and `sfilt`; compare behavior to 5160 and document sensitivity/threshold deltas.

Notes
- Build guards must ensure no 5160-only registers appear in 2240 builds and vice versa.
- COOLCONF.SGT position was verified as [23:16] on both.
- The register diff file is the single source of truth for addresses and presence.

## TODO checklist (driver implementation)

Core wiring
- [x] Compile-time chip selection includes `TMC_TYPE == 2240` path (basic scaffolding and current calculations present).
- [x] Reuse of common open-loop paths (IHOLD_IRUN, CHOPCONF, microstepping, interpolation) across chip families.
- [ ] Add 2240-specific DRV_CONF writer for CURRENT_RANGE and SLOPE_CONTROL (guarded under `TMC_TYPE == 2240`).
- [ ] Ensure `DefaultGConfReg` for 2240 uses only 2240-valid bits (no 5160-only flags) and includes relevant options (e.g., fast_standstill, en_pwm_mode, push-pull, etc.).

StallGuard
- [x] COOLCONF/SGT write/read flow present for 51xx family.
- [ ] Confirm and use the same SGT mask/shift ([23:16]) for 2240; adjust any 2240 build guards accordingly.
- [x] DRV_STATUS parsing exists and reports SG_RESULT/CS_ACTUAL.
- [ ] Review/add any 2240-specific flag nuances to status reporting if needed.

Direct mode (closed-loop/phase stepping)
- [x] DIRECT (0x2D) write path and packing used in closed-loop/phase stepping.
- [ ] Ensure enabling `GCONF.direct_mode` works for 2240 and is exercised in closed-loop mode.

Diagnostics (M122)
- [ ] Read and report 2240 ADC: 0x50 (VSUPPLY/AIN), 0x51 (TEMP), 0x52 (OTW_OV_VTH).
- [ ] Read and report 2240 SG4: 0x74 (THRS), 0x75 (RESULT), 0x76 (IND).
- [ ] Suppress 5160-only diagnostics (LOST_STEPS 0x73, DCCTRL 0x6E) when `TMC_TYPE == 2240`.

Feature gating / exclusions
- [x] 5160-only blocks (SHORT_CONF, OFFSET_READ, ramp gen, DCCTRL, LOST_STEPS, OTP/FACTORY) are guarded and excluded from 2240 builds.
- [ ] Audit code paths to ensure no accidental 5160-only register access remains under 2240.

Board/config
- [x] `MaxTmc2240Current` and `Tmc2240SenseResistor` used for current scaling.
- [ ] Add default/config for `TMC2240_CURRENT_RANGE` and optional `TMC2240_SLOPE_CONTROL` per board.
- [ ] Document expected defaults in board config templates.

Testing
- [ ] Phase 1: Open-loop motion on hardware (SpreadCycle/StealthChop); verify M122 common fields.
- [ ] Phase 2: ADC diagnostics sanity (voltage/temp thresholds) and optional SG4 readouts.
- [ ] Phase 3: DIRECT mode exercise (hold/response) and timing validation.
- [ ] Phase 4: StallGuard tuning (SGT/sfilt) and comparison notes vs 5160.
