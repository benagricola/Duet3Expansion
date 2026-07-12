/*
 * MotorControlBlock.h
 *
 * The single, explicit interface between the two cores when motor control runs on core 1.
 *
 * DESIGN:
 *   The 80us control cycle is split into two layers that talk ONLY through this block:
 *
 *   - CORE 1 (the servo, bare metal, in MotorControlLoop.cpp): a tiny pure kernel. Every cycle it reads the
 *     encoder, reads its inputs from this block, either runs the PID or applies a direct command, sets
 *     the coil currents, and writes its outputs back to this block. It calls NO FreeRTOS, NO flash, NO
 *     allocation, NO notifications - none of it is reachable from MotorControlLoop.cpp, so a stray call is a
 *     build error, not a reset at 200MHz on the bench. That is the whole point.
 *
 *   - CORE 0 (management, a normal FreeRTOS task/handlers): owns everything that is not the hot cycle -
 *     tuning and calibration state machines (which drive the servo via the direct-command inputs and
 *     read the feedback outputs), NVM writes, fault/stall reporting, sample streaming, statistics
 *     reporting. All of it may use FreeRTOS freely because it runs on core 0.
 *
 *   Two narrow handles cross the block by design rather than being marshalled through it:
 *   - the encoder object pointer (the kernel calls its TakeReading()/GetCurrentCount() interface - the
 *     encoder SPI HAL is part of the kernel's allowed surface). Core 0 writes it only while core 1 is
 *     parked, and nulls it before destroying the encoder.
 *   - the trajectory: the kernel calls MotorControlGetTrajectory() (implemented by the motion system),
 *     which evaluates and advances the motion segments exactly as the proven closed-loop code always
 *     has, under the cross-core lock. The block carries configuration, commands and telemetry; the
 *     12.5kHz trajectory query is a direct call so the proven segment logic is not reimplemented.
 *
 * SYNCHRONISATION:
 *   Scalar fields are plain volatile: single-writer per field, and a torn read of one control cycle is
 *   harmless for a servo (the next cycle corrects it). Grouped inputs that must be consistent (the PID
 *   gain set, a direct command) use the sequence-lock counters so the reader retries on a mid-update
 *   snapshot - lock-free, no FreeRTOS. The statistics accumulators are written by core 1 and
 *   read-and-reset by core 0; the races there lose at most one cycle's contribution (diagnostic only).
 *   Diagnostic samples go straight into the shared SampleBuffer, which was already designed as
 *   single-producer (the control cycle, on core 1) / single-consumer (the transmission task, core 0).
 *   This header pulls in nothing from FreeRTOS/flash so it is safe to include from either side.
 */

#ifndef SRC_CLOSEDLOOP_MOTORCONTROLBLOCK_H_
#define SRC_CLOSEDLOOP_MOTORCONTROLBLOCK_H_

#include <cstdint>
#include <Core.h>			// defines the RPXXXX macro tested below (it is not a command-line define)

// Trajectory sample passed from the motion system to the control code. Also used by the pre-existing
// (non-core-1) closed-loop code, so it lives outside the TMC_ON_CORE1 gate.
struct MotionParameters
{
	float position = 0.0;
	float speed = 0.0;
	float acceleration = 0.0;
};

#if RPXXXX && TMC_ON_CORE1

class Encoder;
class SampleBuffer;

// Kernel-side sample-streaming state, published in the block for core 0 to mirror (see below)
enum class MotorSampleState : uint8_t
{
	idle = 0,
	waitingForMove,		// armed; recording starts when a movement command begins executing
	recording,
	complete,			// requested number of samples collected
	overflowed,			// the sample buffer filled before the requested number was collected
};

// Tuning sweep manoeuvres the kernel can execute (see the tuning sweep fields below)
enum class MotorSweepKind : uint8_t
{
	none = 0,
	basicTuning,		// move forward/back ~4 full steps collecting linear-regression data (relative encoders)
	calibrate,			// full-revolution sweep collecting calibration data, clearing the LUT first
	calibrationCheck,	// as calibrate but keeps the LUT (measures residual errors)
};

enum class MotorSweepState : uint8_t
{
	idle = 0,
	running,
	done,				// sweep finished; core 0 performs the completion (result processing, calibration task)
};

// What the servo does each cycle, chosen by core 0.
enum class MotorMode : uint32_t
{
	idle = 0,			// motor de-energised / holding; the kernel does nothing (core 0 owns the encoder in this mode)
	openLoopStep,		// open loop: generate STEP pulses from the trajectory, TMC makes the currents (staging 3)
	closedLoop,			// closed loop: firmware computes coil currents by PID against the trajectory
	assistedOpen,		// phase follows the commanded position open-loop; the encoder error only boosts the current
	directCommand,		// apply commandedPhase/commandedCurrentFraction verbatim (used by tuning/calibration)
};

struct MotorControlBlock
{
	// ---- Inputs: written by core 0, read by core 1 --------------------------------------------------
	volatile MotorMode mode = MotorMode::idle;

	// The encoder the kernel reads each cycle. Written only while core 1 is parked; nulled before the
	// encoder object is destroyed. The kernel does nothing while this is null.
	Encoder *volatile encoder = nullptr;

	// Bumped by core 0 when the kernel must clear its integrator, derivative/speed filters and stall
	// latches (on entering closed loop, and when the PID gains change).
	volatile uint32_t resetSeq = 0;

	// PID + feedforward gain set. Written as a group under paramSeq (odd = update in progress).
	volatile uint32_t paramSeq = 0;
	volatile float Kp = 0, Ki = 0, Kd = 0, Kv = 0, Ka = 0;
	volatile float currentLimitFraction = 1.0;					// max fraction of configured motor current (not yet applied in staging 1)
	volatile float preErrorThreshold = 0, errorThreshold = 0;	// in full steps; 0 disables that check
	volatile float holdCurrentFraction = 0.25;					// the current floor in assistedOpen mode (from the standstill current percentage)

	// Phase offset for assistedOpen mode: aligns the commanded-position phase with the rotor's
	// measured phase at mode entry. Written only while core 1 is parked (the mode transition).
	volatile uint16_t phaseOffset = 0;

	// Feature config (phase advance / flux braking), read every cycle. (Not yet used by the kernel:
	// the ported control law has its own fixed phase feedforward, matching the proven behaviour.)
	volatile bool phaseAdvanceEnabled = false;
	volatile float phaseAdvanceStartStepsPerSec = 0, phaseAdvanceCountsPerStepPerSec = 0;
	volatile uint32_t phaseAdvanceMaxCounts = 0;
	volatile bool fluxBrakeEnabled = false;

	// Direct command (directCommand mode). Written as a group under commandSeq.
	volatile uint32_t commandSeq = 0;
	volatile uint16_t commandedPhase = 0;
	volatile float commandedCurrentFraction = 0;

	// ---- Outputs: written by core 1, read by core 0 ------------------------------------------------
	volatile int32_t encoderCount = 0;							// latest raw encoder count
	volatile float positionError = 0;							// full steps
	volatile float currentFraction = 0;							// commanded current fraction this cycle
	volatile uint16_t commandedStepPhase = 0;
	volatile uint16_t measuredStepPhase = 0;
	volatile bool encoderReadOk = true;							// false if the last encoder read failed
	volatile int32_t motorPosition = 0;							// steps taken so far (open-loop-step path; staging 3). Core 0 reads for reporting/next-move planning.

	// Event flags: set by core 1, cleared by core 0 after it has actioned them.
	volatile bool faultPending = false;							// position error exceeded errorThreshold
	volatile bool stall = false;								// latched stall state (level)
	volatile bool preStall = false;

	// Statistics: core 1 accumulates, core 0 reads-and-resets. A one-period race is benign (diagnostic).
	volatile float statMaxAbsError = 0;
	volatile float statSumSqError = 0;
	volatile float statMaxCurrentFraction = 0;
	volatile float statSumCurrentFraction = 0;
	volatile uint32_t statSampleCount = 0;

	// Loop-health counters and timing (for M122), core 1 writes; core 0 resets after reporting (benign race).
	volatile uint32_t cycleCount = 0;
	volatile uint32_t cycleOverruns = 0;
	volatile uint32_t minCycleInterval = 0xFFFFFFFF;			// step clocks between cycle starts
	volatile uint32_t maxCycleInterval = 0;
	volatile uint32_t minCycleRuntime = 0xFFFFFFFF;				// step clocks spent in the cycle
	volatile uint32_t maxCycleRuntime = 0;

	// ---- Tuning sweep (M569.6): armed by core 0, executed by the kernel at the native step pacing ---
	// The manoeuvre state machines run inside the kernel (in directCommand mode) so the step rate is
	// paced by step-clock ticks rather than the FreeRTOS tick; core 0 arms one manoeuvre at a time and
	// waits for done, then performs the completion (which needs FreeRTOS/flash).
	volatile uint32_t sweepArmSeq = 0;							// bumped by core 0 to arm (or, with kind none, abort); kernel latches sweepKind when it changes
	volatile MotorSweepKind sweepKind = MotorSweepKind::none;
	volatile MotorSweepState sweepState = MotorSweepState::idle;	// written by the kernel

	// ---- Sample streaming (M569.5): armed by core 0, executed by the kernel ------------------------
	// The kernel packs samples straight into the shared SampleBuffer, exactly as the pre-kernel
	// control loop did (SampleBuffer was already single-producer core 1 / single-consumer core 0);
	// only the arming/progress state crosses through the block. Core 0 mirrors the progress into the
	// ClosedLoop sampling state machine that the CAN transmission task runs on.
	SampleBuffer *volatile sampleBuffer = nullptr;				// set once by core 0 at init; the kernel does not sample while it is null
	volatile uint32_t sampleArmSeq = 0;							// bumped by core 0 to (re)arm or stop; the kernel latches the group below when it changes
	volatile uint16_t sampleFilter = 0;							// CL_RECORD_* bitmask (from Duet3Common.h)
	volatile uint16_t samplesRequested = 0;
	volatile uint32_t sampleIntervalTicks = 1;					// step clocks between samples
	volatile uint8_t sampleStartMode = 0;						// 0 = stop, 1 = start now, 2 = start on the next movement command
	volatile uint16_t samplesCollected = 0;						// kernel: samples written to the buffer so far
	volatile MotorSampleState sampleState = MotorSampleState::idle;	// kernel: progress/completion
};

// The one shared instance lives in ordinary RAM (not the flash-cached region). Defined in MotorControlLoop.cpp.
extern MotorControlBlock motorBlock;

// The kernel's trajectory query, implemented by the motion system (Move.cpp). Evaluates - and in closed
// loop mode advances - the motion segments for the closed-loop driver at time 'when' (movement time).
// Returns true if a movement command is current. Runs on core 1 under the cross-core motion lock.
bool MotorControlGetTrajectory(uint32_t when, MotionParameters& mParams) noexcept;

namespace MotorControl
{
	// One control cycle; called from the core-1 TMC loop between SPI transfers.
	void Cycle() noexcept;
}

#endif	// RPXXXX && TMC_ON_CORE1

#endif /* SRC_CLOSEDLOOP_MOTORCONTROLBLOCK_H_ */
