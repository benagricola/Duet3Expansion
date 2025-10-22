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

3.7 Current scaling (2240 vs 5160 differences)
- TMC5160 approach:
  - Uses external sense resistor with 325mV reference voltage
  - Formula: `FullScaleCurrent = 325.0 / SenseResistor` (mA)
  - Two-stage control allows scaling below fixed hardware maximum
- TMC2240 approach:
  - Uses Integrated Current Sensing (ICS) with RREF and KIFS
  - Formula: `FullScaleCurrent = (KIFS × 1000) / Rref` (mA peak)
  - KIFS determined by DRV_CONF[1:0]: 11.75, 24, or 36 A×kΩ
  - RMS current = Peak / √2 ≈ Peak / 1.414
- Shared algorithm with chip-specific full-scale calculation:
  - Calculate `GLOBALSCALER = MotorCurrent × 256 × RecipFullScaleCurrent × csRecip`
  - Clamp GLOBALSCALER to 32-256 range (0 means 256, 1-31 invalid)
  - If out of range, adjust IRUN to compensate while keeping GLOBALSCALER valid
  - Calculate IHOLD using standstill fraction with MaximumStandstillCurrent limit
  - Write both IHOLD_IRUN and GLOBALSCALER registers
- Benefits of GLOBALSCALER approach over UART's IRUN-only:
  - **Resolution**: ~1% (256 steps) vs ~3.125% (32 steps)
  - **Microstep quality**: Keeps IRUN in optimal 16-31 range per datasheet recommendation
  - **Accuracy**: Fine-tuning allows precise current setting even when motor << board maximum
  - Example: 2A motor on 3A hardware → GLOBALSCALER=171, IRUN=31 achieves ±0.2% accuracy with best microstep performance

3.8 Excluding 5160-only features for 2240
- Wrap access to: SHORT_CONF (0x09), OFFSET_READ(0x0C), motion/ramp (0x20–0x2C), DCCTRL(0x6E), LOST_STEPS(0x73), X_COMPARE(0x05), OTP_* (0x06/0x07), FACTORY_CONF(0x08).
- Keep public APIs stable; just no-op or omit when not compiled for that chip.

## 4) Board/config integration

- Keep existing TMC_TYPE selection per board. Valid: 5130, 5160 (covers 5161/2160), 2240.
- **TmcSPI driver is a drop-in replacement for TMC51xx driver** - board configs remain unchanged.
- Naming convention (must match TMC51xx.cpp for compatibility):
  - For TMC5160 boards:
    - `MaxTmc5160Current` - maximum allowed motor current in mA (matches TMC51xx.cpp)
    - `Tmc5160SenseResistor` - external sense resistor value in Ω (matches TMC51xx.cpp)
    - `DefaultStandstillCurrentPercent` - default standstill percentage (typically 71-75%)
  - For TMC2240 boards:
    - `MaximumMotorCurrent` - maximum allowed motor current in mA (used by UART drivers)
    - `Tmc2240Rref` - reference resistor in kΩ (chip-specific for ICS)
    - `Tmc2240CurrentRange` - DRV_CONF[1:0] setting: 0x00=11.75, 0x01=24, 0x02/0x03=36 A×kΩ
    - `Tmc2240SlopeControl` - DRV_CONF[5:4] for dV/dt: 0x00=120V/µs, 0x01=200V/µs, 0x02=300V/µs, 0x03=480V/µs
    - `DefaultStandstillCurrentPercent` - default standstill percentage (typically 75%)
- Current calculation differences:
  - TMC5160 (external sense resistor, 325mV ref):
    - `RecipFullScaleCurrent = Tmc5160SenseResistor / 325.0`
    - Uses `MaxTmc5160Current` for limits
  - TMC2240 (ICS with RREF + KIFS):
    - `KIFS` from `Tmc2240CurrentRange` mapping: 0b00=11.75, 0b01=24, 0b10/0b11=36 A×kΩ
    - `RecipFullScaleCurrent = Tmc2240Rref / (KIFS × 1000)`
    - Uses `MaximumMotorCurrent` for limits
    - Example: `Tmc2240Rref=12kΩ`, `KIFS=36` → `IFS_peak=3000mA`, `I_RMS≈2121mA`
- Both chips use an identical two-stage GLOBALSCALER + IRUN algorithm for optimal resolution and microstep quality.
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
- [x] Add 2240-specific DRV_CONF writer for CURRENT_RANGE and SLOPE_CONTROL (guarded under `TMC_TYPE == 2240`).
- [x] Ensure `DefaultGConfReg` for 2240 uses only 2240-valid bits (no 5160-only flags) and includes relevant options (e.g., fast_standstill, en_pwm_mode, push-pull, etc.).
- [x] Ensure TCOOLTHRS/THIGH are present and used on 2240 (parity with 5160).

StallGuard
- [x] COOLCONF/SGT write/read flow present for 51xx family.
- [x] Confirm and use the same SGT mask/shift ([23:16]) for 2240; adjust any 2240 build guards accordingly.
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
- [x] Board configs use existing constant names for drop-in compatibility with TMC51xx driver:
  - TMC5160: `MaxTmc5160Current`, `Tmc5160SenseResistor`, `DefaultStandstillCurrentPercent`
  - TMC2240: `MaximumMotorCurrent`, `Tmc2240Rref`, `Tmc2240CurrentRange`, `Tmc2240SlopeControl`, `DefaultStandstillCurrentPercent`
- [x] Current calculation uses chip-appropriate formula with board-defined constants
- [x] TmcSPI.cpp uses MaxTmc5160Current for TMC5160 (matches TMC51xx.cpp exactly)
- [x] TmcSPI.cpp uses MaximumMotorCurrent for TMC2240 (matches existing UART board configs)

Current setting implementation
- [x] Update RecipFullScaleCurrent calculation to be chip-specific:
  - TMC5160: `RecipFullScaleCurrent = Tmc5160SenseResistor / 325.0` (external sense resistor)
  - TMC2240: `RecipFullScaleCurrent = Tmc2240Rref / (KIFS × 1000)` where KIFS determined by `Tmc2240CurrentRange`
- [x] TMC5160 uses `MaxTmc5160Current` for current limits (matches TMC51xx.cpp)
- [x] TMC2240 uses `MaximumMotorCurrent` for current limits (matches existing UART configs)
- [x] Verify UpdateCurrent() algorithm works correctly with both calculation methods
- [x] Remove any dead code or constants related to incorrect TMC2240 sense resistor approach
- [x] Ensure `MaximumStandstillCurrent` calculated from chip-appropriate max current constant

Testing
- [ ] Phase 1: Open-loop motion on hardware (SpreadCycle/StealthChop); verify M122 common fields.
- [ ] Phase 2: ADC diagnostics sanity (voltage/temp thresholds) and optional SG4 readouts.
- [ ] Phase 3: DIRECT mode exercise (hold/response) and timing validation.
- [ ] Phase 4: StallGuard tuning (SGT/sfilt) and comparison notes vs 5160.
