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
 *     LINK ERROR at build time, not a reset at 200MHz on the bench. That is the whole point.
 *
 *   - CORE 0 (management, a normal FreeRTOS task/handlers): owns everything that is not the hot cycle -
 *     tuning and calibration state machines (which drive the servo via the direct-command inputs and
 *     read the feedback outputs), NVM writes, fault/stall reporting, sample streaming, statistics
 *     reporting. All of it may use FreeRTOS freely because it runs on core 0.
 *
 * SYNCHRONISATION:
 *   Scalar fields are plain volatile: single-writer per field, and a torn read of one control cycle is
 *   harmless for a servo (the next cycle corrects it). Grouped inputs that must be consistent (the PID
 *   gain set, a direct command) use the sequence-lock counters so the reader retries on a mid-update
 *   snapshot - lock-free, no FreeRTOS. The sample ring is single-producer (core 1) / single-consumer
 *   (core 0). This header pulls in nothing from FreeRTOS/flash so it is safe to include from either side.
 */

#ifndef SRC_CLOSEDLOOP_MOTORCONTROLBLOCK_H_
#define SRC_CLOSEDLOOP_MOTORCONTROLBLOCK_H_

#include <cstdint>

#if RPXXXX && TMC_ON_CORE1

// What the servo does each cycle, chosen by core 0.
enum class MotorMode : uint32_t
{
	idle = 0,			// motor de-energised / holding; publish encoder only
	openLoopStep,		// open loop: generate STEP pulses from the trajectory, TMC makes the currents (Phase 2)
	closedLoop,			// closed loop: firmware computes coil currents by PID against the trajectory
	directCommand,		// apply commandedPhase/commandedCurrentFraction verbatim (used by tuning/calibration)
};

struct MotorControlBlock
{
	// ---- Inputs: written by core 0, read by core 1 --------------------------------------------------
	volatile MotorMode mode = MotorMode::idle;

	// PID + feedforward gain set. Written as a group under paramSeq (odd = update in progress).
	volatile uint32_t paramSeq = 0;
	volatile float Kp = 0, Ki = 0, Kd = 0, Kv = 0, Ka = 0;
	volatile float currentLimitFraction = 1.0;					// max fraction of configured motor current
	volatile float preErrorThreshold = 0, errorThreshold = 0;	// in full steps; 0 disables that check

	// Feature config (phase advance / flux braking), read every cycle.
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
	volatile uint16_t encoderReading = 0;						// latest raw encoder angle
	volatile float positionError = 0;							// full steps
	volatile float currentFraction = 0;							// commanded current fraction this cycle
	volatile uint16_t commandedStepPhase = 0;
	volatile uint16_t measuredStepPhase = 0;
	volatile bool encoderReadOk = true;							// false if the last encoder read failed
	volatile int32_t motorPosition = 0;							// steps taken so far (open-loop-step path; Phase 2). Core 0 reads for reporting/next-move planning.

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

	// Loop-health counters (for the M122 rate-degradation warning), core 1 writes.
	volatile uint32_t cycleCount = 0;
	volatile uint32_t cycleOverruns = 0;

	// ---- Sample ring: single-producer (core 1) / single-consumer (core 0) --------------------------
	// Used only while streaming diagnostic samples. Core 1 pushes, core 0 drains and sends over CAN.
	static constexpr uint32_t SampleRingSize = 256;				// power of two
	volatile uint32_t sampleHead = 0;							// core 1 increments after writing
	volatile uint32_t sampleTail = 0;							// core 0 increments after reading
	volatile uint32_t sampleRing[SampleRingSize] = { };			// packed sample words (format agreed out of band)
};

// The one shared instance lives in ordinary RAM (not the flash-cached region). Defined in MotorControlLoop.cpp.
extern MotorControlBlock motorBlock;

#endif	// RPXXXX && TMC_ON_CORE1

#endif /* SRC_CLOSEDLOOP_MOTORCONTROLBLOCK_H_ */
