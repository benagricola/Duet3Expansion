# Motor control on core 1

## Target architecture

**Core 1 = the motor-output engine.** It turns the trajectory core 0 plans into physical motion, by
whichever method the mode selects, and nothing else. **Core 0 = planning + management** (move planning,
tuning/calibration, NVM, reporting, comms). They talk only through `MotorControlBlock`.

Both motor-drive methods live on core 1 so that "motor control" is one coherent thing, not split by mode:

| Mode | What core 1 does each iteration |
|---|---|
| `openLoopStep` | Poll the step deadline for the current trajectory point and toggle STEP/DIR; the TMC chip makes the coil currents. Publishes `motorPosition`. |
| `closedLoop` | Read encoder → PID + feedforward against the trajectory → write coil currents (XDIRECT). Publishes encoder/error. |
| `directCommand` | Apply core 0's commanded phase/current verbatim (used by the tuning/calibration sequencer). |
| `idle` | De-energised/holding; publish encoder only. |

## Why both methods on core 1

- **Coherence:** one home for all real-time motor output; core 0 never touches STEP pins or coil currents.
- **Frees core 0:** the step ISR is heavy during fast moves; moving it off core 0 reclaims that time.
- **Timing:** closed loop *needs* the deterministic 80 µs cycle (that is the whole reason for core 1).
  Open-loop stepping does *not* need it — the TMC chip smooths µs-level step jitter into the currents —
  so moving stepping to core 1 is a cleanliness + CPU win, not a motion-quality one. Done as a busy-poll
  it can actually improve step timing (no interrupt latency).

## The rule that ends the whack-a-mole

`MotorControlLoop.cpp` (core 1) includes only: `MotorControlBlock.h`, the TMC + encoder SPI HAL, the
trig/PID math, and `StepTimer`. It does **not** include or call FreeRTOS, the NVM/flash API, CAN, or the
object model. A stray call to any of them is a **link error at build time**, not a reset at 200 MHz on
the bench. The compiler enforces the separation we kept violating by hand.

## What moves off the hot path onto core 0

- `PerformTune` becomes a core-0 state machine that sequences `directCommand`s and reads back the encoder
  from the block (the maneuver runs at core-0 pace; the kernel just does I/O).
- Encoder calibration and its NVM writes (`Calibrate`/`ScrubLUT` → `EnsureWritten`) — core 0 only.
- Fault/stall *reporting* (`NewDriverFault`, events), sample *draining* + CAN send, statistics *reporting*
  — all driven from flags/accumulators/ring the kernel writes, drained on core 0 (reusing the deferred
  drain already in `Move::Spin`).

## Staging

1. **Interface + closed-loop kernel** — `MotorControlBlock.h` (done), `MotorControlLoop.cpp` running
   `closedLoop` + `directCommand` + `idle`. Core 0 populates inputs, reads outputs. *Proves the pattern
   and makes closed-loop holding work with zero FreeRTOS on core 1 — the part that needs the determinism.*
2. **Tuning/calibration state machine on core 0** — removes the last flash-write-from-core-1 path.
3. **Fold in open-loop step generation** — move `Move::Interrupt`/`CalcNextStepTime` stepping onto core 1
   as the `openLoopStep` mode; core 1 becomes the sole owner of motor output and motor position. (Bigger,
   more core-0-entangled; deferred until the closed-loop pattern is proven.)
4. **Streaming + reporting**, then **validate + data-driven compare** of core-0 vs core-1 builds.

## Kept from the current core-1 work

`Core1Runtime` (bare-metal host, park/resume, launch), RAM pinning, CAN-on-core-0
(`SPICAN_CORE0_SERVICE`), cross-core spinlocks. Only the motor-control loop body is restructured.
