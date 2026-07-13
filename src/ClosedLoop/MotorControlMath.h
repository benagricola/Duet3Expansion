/*
 * MotorControlMath.h
 *
 * The closed-loop and assisted-open-loop control law, shared between the two hosts that can
 * execute it:
 *  - the pre-existing TmcLoop path (ClosedLoop::ControlMotorCurrents, ClosedLoop.cpp), used when
 *    the motor kernel is not in use;
 *  - the core-1 motor kernel (MotorControlLoop.cpp), which runs it inside the control cycle when
 *    TMC_ON_CORE1.
 * There is ONE copy of the control mathematics - this file - taken verbatim from the proven
 * ClosedLoop::ControlMotorCurrents implementation. Unlike the tuning manoeuvres (TuningMoves.h)
 * the law is pure arithmetic, so no context type is needed: every input is an argument and the
 * PID terms are written through references that each host binds to its own diagnostic state
 * (ClosedLoop members feeding CollectSample; kernel statics feeding SampleStream). Torque mode
 * stays in ClosedLoop.cpp: it is not available in the kernel.
 *
 * This header is included by the core-1 kernel, so it must not (transitively) include FreeRTOS,
 * CAN, the NVM/flash API or the object model, and must stay free of globals.
 */

#ifndef SRC_CLOSEDLOOP_MOTORCONTROLMATH_H_
#define SRC_CLOSEDLOOP_MOTORCONTROLMATH_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

#include <Movement/StepTimer.h>
#include "Trigonometry.h"

namespace MotorControlMath
{
	constexpr float PIDIlimit = 80.0;

	// Closed-loop control: a PID controller computes the required 'torque' as a control signal in
	// the arbitrary range -256..256. The phase of the motor current is always +/- 1 full step
	// relative to the measured position, with a speed-dependent feedforward, and the current
	// magnitude follows the PID result. iTerm is the integrator and is updated in place; the other
	// term references receive the values used, for diagnostics and data collection.
	static inline void ComputeClosedLoop(float Kp, float Ki, float Kd, float Kv, float Ka,
										float positionError, float errorDerivative, float speedDerivative,
										float trajectorySpeed, float trajectoryAcceleration,
										StepTimer::Ticks ticksSinceLastCall, uint32_t measuredStepPhase,
										float& pTerm, float& iTerm, float& dTerm, float& vTerm, float& aTerm, float& controlSignal,
										uint16_t& commandedStepPhase, float& currentFraction) noexcept
	{
		pTerm = constrain<float>(Kp * positionError, -256.0, 256.0);
		dTerm = constrain<float>(Kd * errorDerivative * StepTimer::StepClockRate, -256.0, 256.0);	// constrain D so that we can graph it more sensibly after a sudden step input
		const float timeDelta = (float)ticksSinceLastCall * (1.0/(float)StepTimer::StepClockRate);	// get the time delta in seconds
		iTerm = constrain<float>(iTerm + Ki * positionError * timeDelta, -PIDIlimit, PIDIlimit);	// constrain I to prevent it running away
		vTerm = trajectorySpeed * Kv * ticksSinceLastCall;
		aTerm = trajectoryAcceleration * Ka * fsquare(ticksSinceLastCall);
		controlSignal = constrain<float>(pTerm + iTerm + dTerm + vTerm + aTerm, -256.0, 256.0);		// clamp the sum between +/- 256

		// Phase of motor current is always +/- 1 full step relative to current position, but motor
		// current is adjusted according to the PID result.
		// The following assumes that signed arithmetic is 2's complement
		const float PhaseFeedForwardFactor = 1000.0;
		const int16_t phaseFeedForward = (int16_t)lrintf(constrain<float>(speedDerivative * ticksSinceLastCall * PhaseFeedForwardFactor, -256.0, 256.0));
		const uint16_t adjustedStepPhase = (uint16_t)((int16_t)measuredStepPhase + phaseFeedForward) % 4096u;
		commandedStepPhase = (((controlSignal < 0.0) ? (3 * 1024) : 1024) + adjustedStepPhase) % 4096u;
		currentFraction = fabsf(controlSignal) * (1.0/256.0);
	}

	// Assisted open loop: the phase follows the commanded position; the I term is not used and the
	// A and V terms are independent of the loop time. The error only boosts the current above the
	// standstill floor.
	static inline void ComputeAssistedOpen(float Kp, float Kd, float Kv, float Ka,
										float positionError, float errorDerivative,
										float trajectorySpeed, float trajectoryAcceleration,
										float targetPosition, uint16_t phaseOffset, float holdCurrentFraction,
										float& pTerm, float& dTerm, float& vTerm, float& aTerm, float& controlSignal,
										uint16_t& commandedStepPhase, float& currentFraction) noexcept
	{
		pTerm = constrain<float>(Kp * positionError, -256.0, 256.0);
		dTerm = constrain<float>(Kd * errorDerivative * StepTimer::StepClockRate, -256.0, 256.0);
		constexpr float scalingFactor = 100.0;
		vTerm = trajectorySpeed * Kv * scalingFactor;
		aTerm = trajectoryAcceleration * Ka * fsquare(scalingFactor);
		controlSignal = min<float>(fabsf(pTerm + dTerm) + fabsf(vTerm) + fabsf(aTerm), 256.0);

		const uint16_t stepPhase = (uint16_t)llrintf(targetPosition * 1024.0);		// we use llrintf so that we can guarantee to convert the float operand to integer. We only care about the lowest 12 bits.
		commandedStepPhase = (stepPhase + phaseOffset) % 4096u;
		currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(controlSignal * (1.0/256.0), 1.0);
	}

	// Coil currents for a torque-producing vector (phase/magnitude, about 90deg electrical to the
	// rotor) plus an optional zero-torque flux-braking vector along the measured rotor axis
	// (rotorPhase/brakeMagnitude). Both magnitudes are 0.0..1.0 and their squares must sum to no
	// more than 1; the components are clamped to the +/-248 swing FastSinCos is scaled for anyway.
	static inline void ComputeCoilCurrents(uint16_t phase, float magnitude, uint16_t rotorPhase, float brakeMagnitude,
										int16_t& coilA, int16_t& coilB) noexcept
	{
		float sine, cosine;
		Trigonometry::FastSinCos(phase, sine, cosine);
		if (brakeMagnitude > 0.0)
		{
			float brakeSine, brakeCosine;
			Trigonometry::FastSinCos(rotorPhase, brakeSine, brakeCosine);
			coilA = (int16_t)constrain<int32_t>(lrintf(cosine * magnitude + brakeCosine * brakeMagnitude), -248, 248);
			coilB = (int16_t)constrain<int32_t>(lrintf(sine * magnitude + brakeSine * brakeMagnitude), -248, 248);
		}
		else
		{
			coilA = (int16_t)lrintf(cosine * magnitude);
			coilB = (int16_t)lrintf(sine * magnitude);
		}
	}

#if SUPPORT_PHASE_ADVANCE
	// Speed-proportional phase advance (field weakening): at speed, winding inductance and the
	// shrinking voltage headroom make the actual coil currents lag behind the rotating command,
	// which costs torque; compensate by advancing the command in the direction of rotation,
	// proportionally to speed above the onset threshold. This is distinct from the phase
	// feedforward in ComputeClosedLoop, which only compensates the one-cycle control latency.
	// Returns the adjusted phase; advanceCountsApplied reports the advance used (diagnostics).
	static inline uint16_t ApplyPhaseAdvance(float speedStepsPerSec, float onsetStepsPerSec, float countsPerStepPerSec,
										uint16_t maxCounts, uint16_t commandedStepPhase, uint16_t& advanceCountsApplied) noexcept
	{
		const float advance = (fabsf(speedStepsPerSec) - onsetStepsPerSec) * countsPerStepPerSec;
		if (advance > 0.0)
		{
			const uint16_t advanceCounts = (uint16_t)min<float>(advance, (float)maxCounts);
			advanceCountsApplied = advanceCounts;
			const int32_t signedAdvance = (speedStepsPerSec < 0.0) ? -(int32_t)advanceCounts : (int32_t)advanceCounts;
			return (uint16_t)(((int32_t)commandedStepPhase + signedAdvance) & 0x0FFF);
		}
		advanceCountsApplied = 0;
		return commandedStepPhase;
	}
#endif

#if SUPPORT_FLUX_BRAKING
	constexpr uint16_t FluxBrakeSnapDefaultMv = 8000;			// default single-sample jump treated as the supply stepping up rather than regeneration.
																// Power-on itself does not rely on this: readings below the caller's plausibility floor never
																// touch the baseline, and the first plausible reading pulls the baseline DOWN from its init.
																// Hard regeneration can exceed 8V between ADC refreshes and would then be misclassified and
																// skipped, so the threshold is runtime-adjustable (M569.2 feature register 136).
	constexpr uint16_t FluxBrakeBaselineDriftMv = 10;			// upward drift step, applied every 64 control cycles (about 2V/s at the 12.5kHz loop rate)

	// Maintain the supply-voltage baseline and return the d-axis current fraction to inject. Call at
	// the control loop frequency with a plausible reading (the caller applies its ADC's validity
	// window). The baseline follows a falling supply immediately, drifts upwards only slowly (so that
	// regeneration, which raises the bus over milliseconds, registers as overshoot) and snaps up on a
	// large single-sample jump (the supply being switched on, which no motor could produce). The
	// drift step never overtakes the current reading: a step past it would make the unsigned
	// subtraction wrap and read as a huge phantom overshoot (seen on the bench as spurious braking
	// about once a second with the maximum overshoot recorded as 0xFFFF). The torque demand keeps
	// priority: braking only gets the headroom left in the current budget. overshootOut reports the
	// overshoot above the baseline (diagnostics), whether or not braking is enabled.
	static inline float ComputeFluxBrakeFraction(uint16_t vsMv, uint16_t& baselineMv, uint32_t& baselineDivider,
										bool enabled, uint16_t onsetDeltaMv, float recipRangeMv, float maxFraction,
										uint16_t snapMv, float torqueCurrentFraction, uint16_t& overshootOut) noexcept
	{
		overshootOut = 0;
		if (vsMv <= baselineMv)
		{
			baselineMv = vsMv;										// follow a falling supply immediately
			return 0.0;
		}
		if (vsMv - baselineMv >= snapMv)
		{
			baselineMv = vsMv;										// supply stepped up, not regeneration
			return 0.0;
		}
		if (((++baselineDivider) & 0x3F) == 0 && baselineMv < vsMv)
		{
			baselineMv = min<uint16_t>(baselineMv + FluxBrakeBaselineDriftMv, vsMv);
		}
		const uint16_t overshoot = vsMv - baselineMv;
		overshootOut = overshoot;
		if (!enabled || overshoot <= onsetDeltaMv)					// the baseline is maintained even while disabled, so enabling at runtime behaves cleanly
		{
			return 0.0;
		}
		const float requested = min<float>((float)(overshoot - onsetDeltaMv) * recipRangeMv, 1.0) * maxFraction;
		const float headroom = fastSqrtf(max<float>(1.0 - fsquare(torqueCurrentFraction), 0.0));
		return min<float>(requested, headroom);
	}
#endif

	// Stall / pre-stall detection with hysteresis: the stall flag resets when the position error
	// falls to below half the tolerance, to avoid generating too many stall events. Returns true
	// when a NEW stall event fires; the host raises the driver-fault event in its own way (a
	// FreeRTOS notification on core 0, a deferred flag in the kernel).
	static inline bool UpdateStallDetection(float absPositionError, float preErrorThreshold, float errorThreshold,
										bool& stall, bool& preStall) noexcept
	{
		if (stall)
		{
			//TODO do we need a minimum delay before resetting too?
			if (errorThreshold <= 0 || absPositionError < errorThreshold/2)
			{
				stall = false;
			}
			return false;
		}
		stall = errorThreshold > 0 && absPositionError > errorThreshold;
		if (stall)
		{
			return true;
		}
		preStall = preErrorThreshold > 0 && absPositionError > preErrorThreshold;
		return false;
	}
}

#endif	// SUPPORT_CLOSED_LOOP

#endif	// SRC_CLOSEDLOOP_MOTORCONTROLMATH_H_
