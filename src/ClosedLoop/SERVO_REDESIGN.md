# Closed-loop servo on core 1 — structural redesign (Option 2)

## Why

Running the existing `ClosedLoop::InstanceControlLoop` on bare-metal core 1 kept resetting the board
because the loop, written as a FreeRTOS task, reaches into FreeRTOS in ways that are illegal off core 0:
task notifications, critical sections (scheduler suspend), NVM/flash writes, and potentially `malloc`.
Fixing them one at a time (find-a-call, defer-it, rebuild, crash on the next one) is whack-a-mole and
fragile — miss one and the board resets at 200 MHz on the bench.

The structural fix: **split the cycle into a tiny hot kernel that runs on core 1 and a management layer
that stays on core 0, talking only through one shared struct — and make the illegal calls impossible by
construction** (the core-1 unit does not link against FreeRTOS/flash/CAN, so a stray call is a build
error, not a runtime reset).

## The split

| Layer | Core | Contents |
|---|---|---|
| **Servo kernel** (`ServoLoop.cpp`) | 1, bare metal | Per 80 µs cycle: read encoder → read inputs from `servoBlock` → *servo mode*: PID against the trajectory / *directCommand mode*: apply core 0's command → set coil currents → publish outputs + stats to `servoBlock` → push a sample if streaming. Nothing else. |
| **Management** (`ClosedLoop.cpp` on core 0) | 0, FreeRTOS | Tuning + calibration state machines (drive the kernel via `directCommand`, read feedback from `servoBlock`), NVM writes, fault/stall reporting, sample draining + CAN send, statistics reporting, all M569.x command handling. |

The interface is `ServoControlBlock.h` — the only thing that crosses the boundary.

## What moves where

- **Stays hot (core 1):** encoder read, trajectory evaluation (`GetCurrentMotion` via the cross-core
  *spinlock* — a hardware spinlock, legal from core 1, not FreeRTOS), PID + feedforward, phase advance,
  flux braking, coil-current output, stall *detection* (sets a flag).
- **Moves to core 0:** `PerformTune` becomes a state machine that sequences `directCommand`s and reads
  back the encoder — the maneuver runs at core-0 pace, the kernel just does I/O; encoder calibration and
  its NVM writes; sample *draining* and CAN transmission (kernel only pushes to the ring); fault/stall
  *reporting* (`Heat::NewDriverFault`, event raising); statistics *reporting*.

## Isolation enforcement (the key idea)

`ServoLoop.cpp` includes only: `ServoControlBlock.h`, the TMC SPI HAL, the encoder SPI HAL, the
trig/PID math, and `StepTimer` for the deadline. It does **not** include or call FreeRTOS, the NVM/flash
API, CAN, or the object model. If a future edit adds such a call it fails to link — the compiler enforces
the rule we kept violating by hand.

## Phases

1. **Interface + kernel skeleton** — `ServoControlBlock.h` (done), `ServoLoop.cpp` running the servo +
   directCommand modes and publishing outputs. Core 0 populates the inputs. *This alone makes closed-loop
   holding work with zero FreeRTOS on core 1.*
2. **Tuning/calibration state machine on core 0** — reimplement `PerformTune` as a core-0 sequencer over
   `directCommand`; move the calibration NVM writes to core 0.
3. **Sample streaming** — kernel pushes to the ring; core 0 drains + sends. (Diagnostics only.)
4. **Reporting** — faults/stalls/stats flow from flags/accumulators, drained on core 0.
5. **Validate + compare** — bench battery, and a side-by-side of the core-0 vs core-1 build (loop rate,
   CPU headroom, tracking) so the push/no-push decision is data-driven.

## What is kept from the current core-1 work

`Core1Runtime` (the bare-metal loop host, park/resume, launch), the RAM pinning, CAN-on-core-0
(`SPICAN_CORE0_SERVICE`), and the cross-core spinlocks for the motion segments. Only the *control loop
body* is restructured.
