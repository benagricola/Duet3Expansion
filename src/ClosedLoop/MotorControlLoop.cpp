/*
 * MotorControlLoop.cpp
 *
 * The core-1 motor-control kernel (see MOTOR_CONTROL_ON_CORE1.md and MotorControlBlock.h).
 *
 * THE RULE: this translation unit includes only MotorControlBlock.h, the encoder interface, the
 * trig/PID math and StepTimer. FreeRTOS, the NVM/flash API, CAN and the object model are not visible
 * here, so a stray call to any of them is a build error rather than a crash at speed on the bench.
 * Keep it that way: do not add includes without checking what they drag in.
 *
 * The control law is a faithful port of the proven ClosedLoop::InstanceControlLoop /
 * ClosedLoop::ControlMotorCurrents closed-loop path (the v8-validated code); only where the state
 * lives has changed (this file's statics and the shared MotorControlBlock instead of the ClosedLoop
 * instance). Any change to the control law must be made in both places or the core-0 and core-1
 * builds will disagree.
 */

#include "MotorControlBlock.h"

#if RPXXXX && TMC_ON_CORE1

#include <Movement/StepTimer.h>
#include "Trigonometry.h"
#include "DerivativeAveragingFilter.h"
#include "Encoders/Encoder.h"

// Narrow shim into the TMC driver HAL. Declared here rather than by including the driver header
// (which drags in the whole platform); the C++ mangled name makes a signature mismatch a link error.
namespace SmartDrivers
{
	bool SetMotorPhases(size_t driver, uint32_t regVal) noexcept;
}

MotorControlBlock motorBlock;

namespace MotorControl
{
	// These mirror the constants private to ClosedLoop
	constexpr float PIDIlimit = 80.0;
	constexpr unsigned int DerivativeFilterSize = 8;
	constexpr unsigned int SpeedFilterSize = 8;
	constexpr size_t driverNumber = 0;								// single closed-loop driver per board

	// Kernel-private control state (core 1 only)
	static DerivativeAveragingFilter<DerivativeFilterSize> errorDerivativeFilter;
	static DerivativeAveragingFilter<SpeedFilterSize> speedFilter;
	static float PIDITerm = 0.0;
	static uint32_t lastResetSeq = 0;
	static uint32_t lastParamSeq = 0;
	static StepTimer::Ticks prevCycleStartTime = 0;
	static bool prevCycleTimeValid = false;

	// Local copies of the seqlocked gain group (refreshed when paramSeq changes)
	static float Kp = 0.0, Ki = 0.0, Kd = 0.0, Kv = 0.0, Ka = 0.0;
	static float preErrorThreshold = 0.0, errorThreshold = 0.0;

	// Set the coil currents for the given phase and magnitude (0.0..1.0), and publish what we commanded
	TIME_CRITICAL static void SetMotorPhase(uint16_t phase, float magnitude) noexcept
	{
		float sine, cosine;
		Trigonometry::FastSinCos(phase, sine, cosine);
		const int16_t coilA = (int16_t)lrintf(cosine * magnitude);
		const int16_t coilB = (int16_t)lrintf(sine * magnitude);
		(void)SmartDrivers::SetMotorPhases(driverNumber, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
		motorBlock.commandedStepPhase = phase;
	}

	// Refresh the local gain/threshold copies if core 0 has published a new set. Seqlock: retry once,
	// else keep the previous consistent copy and try again next cycle.
	TIME_CRITICAL static void RefreshParams() noexcept
	{
		for (unsigned int attempt = 0; attempt < 2; ++attempt)
		{
			const uint32_t seqBefore = motorBlock.paramSeq;
			if (seqBefore == lastParamSeq)
			{
				return;												// nothing new
			}
			if ((seqBefore & 1u) != 0)
			{
				continue;											// update in progress
			}
			const float p = motorBlock.Kp, i = motorBlock.Ki, d = motorBlock.Kd, v = motorBlock.Kv, a = motorBlock.Ka;
			const float pre = motorBlock.preErrorThreshold, err = motorBlock.errorThreshold;
			if (motorBlock.paramSeq == seqBefore)
			{
				Kp = p; Ki = i; Kd = d; Kv = v; Ka = a;
				preErrorThreshold = pre; errorThreshold = err;
				lastParamSeq = seqBefore;
				return;
			}
		}
	}

	// One control cycle. Ported from ClosedLoop::InstanceControlLoop + ControlMotorCurrents
	// (ClosedLoopMode::closed branch); see the file header comment.
	TIME_CRITICAL void Cycle() noexcept
	{
		const MotorMode mode = motorBlock.mode;
		Encoder *const encoder = motorBlock.encoder;
		if (mode == MotorMode::idle || encoder == nullptr)
		{
			prevCycleTimeValid = false;								// don't count the idle gap as a cycle interval
			return;													// core 0 owns the encoder and the motor while we are idle
		}

		// Cycle timing statistics (these are the loop-health numbers M122 reports)
		const StepTimer::Ticks cycleStartTime = StepTimer::GetTimerTicks();
		StepTimer::Ticks timeElapsed = 0;
		if (prevCycleTimeValid)
		{
			timeElapsed = cycleStartTime - prevCycleStartTime;
			if (timeElapsed < motorBlock.minCycleInterval) { motorBlock.minCycleInterval = timeElapsed; }
			if (timeElapsed > motorBlock.maxCycleInterval) { motorBlock.maxCycleInterval = timeElapsed; }
		}
		prevCycleStartTime = cycleStartTime;
		prevCycleTimeValid = true;

		// Handle a reset request (mode entry or gain change): clear the integrator, filters and latches
		const uint32_t rs = motorBlock.resetSeq;
		if (rs != lastResetSeq)
		{
			lastResetSeq = rs;
			PIDITerm = 0.0;
			errorDerivativeFilter.Reset();
			speedFilter.Reset();
			motorBlock.stall = motorBlock.preStall = false;
		}

		RefreshParams();

		// Read the encoder; the rest of the cycle is gated on this succeeding, as in the ported code
		if (!encoder->TakeReading())
		{
			motorBlock.encoderReadOk = false;
			return;
		}
		motorBlock.encoderReadOk = true;

		// Evaluate the trajectory and compute the position error in full steps
		const uint32_t now = StepTimer::ConvertLocalToMovementTime(cycleStartTime);
		MotionParameters mParams;
		if (mode == MotorMode::closedLoop)
		{
			(void)MotorControlGetTrajectory(now, mParams);
		}
		else
		{
			// directCommand: no trajectory; report the error relative to wherever we are (zero)
			mParams.position = (float)encoder->GetCurrentCount() * encoder->GetStepsPerCount();
		}

		const float targetEncoderReading = rintf(mParams.position * encoder->GetCountsPerStep());
		const float currentPositionError = (float)(targetEncoderReading - (float)encoder->GetCurrentCount()) * encoder->GetStepsPerCount();
		errorDerivativeFilter.ProcessReading(currentPositionError, now);
		speedFilter.ProcessReading((float)encoder->GetCurrentCount() * encoder->GetStepsPerCount(), now);

		const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
		motorBlock.encoderCount = encoder->GetCurrentCount();
		motorBlock.positionError = currentPositionError;
		motorBlock.measuredStepPhase = (uint16_t)measuredStepPhase;

		float currentFraction = 0.0;
		if (mode == MotorMode::directCommand)
		{
			// Apply core 0's commanded phase and current verbatim (seqlock read; skip on a torn snapshot)
			const uint32_t seqBefore = motorBlock.commandSeq;
			if ((seqBefore & 1u) == 0)
			{
				const uint16_t phase = motorBlock.commandedPhase;
				const float fraction = motorBlock.commandedCurrentFraction;
				if (motorBlock.commandSeq == seqBefore)
				{
					SetMotorPhase(phase, fraction);
					currentFraction = fraction;
				}
			}
		}
		else
		{
			// Closed loop: PID + feedforward, ported verbatim from ClosedLoop::ControlMotorCurrents.
			// The control signal is chosen to be in the range -256..256 (arbitrary, as in the original).
			const float PIDPTerm = constrain<float>(Kp * currentPositionError, -256.0, 256.0);
			const float PIDDTerm = constrain<float>(Kd * errorDerivativeFilter.GetDerivative() * StepTimer::StepClockRate, -256.0, 256.0);
			const float timeDelta = (float)timeElapsed * (1.0/(float)StepTimer::StepClockRate);
			PIDITerm = constrain<float>(PIDITerm + Ki * currentPositionError * timeDelta, -PIDIlimit, PIDIlimit);
			const float PIDVTerm = mParams.speed * Kv * timeElapsed;
			const float PIDATerm = mParams.acceleration * Ka * fsquare((float)timeElapsed);
			const float PIDControlSignal = constrain<float>(PIDPTerm + PIDITerm + PIDDTerm + PIDVTerm + PIDATerm, -256.0, 256.0);

			// Phase of the motor current is always +/- 1 full step relative to the measured position,
			// with a speed-dependent feedforward, and the current magnitude follows the PID result.
			const float PhaseFeedForwardFactor = 1000.0;
			const int16_t phaseFeedForward = (int16_t)lrintf(constrain<float>(speedFilter.GetDerivative() * timeElapsed * PhaseFeedForwardFactor, -256.0, 256.0));
			const uint16_t adjustedStepPhase = (uint16_t)((int16_t)measuredStepPhase + phaseFeedForward) % 4096u;
			const uint16_t commandedStepPhase = (((PIDControlSignal < 0.0) ? (3 * 1024) : 1024) + adjustedStepPhase) % 4096u;
			currentFraction = fabsf(PIDControlSignal) * (1.0/256.0);
			SetMotorPhase(commandedStepPhase, currentFraction);

			// Stall / pre-stall detection, ported from InstanceControlLoop
			const float positionErr = fabsf(currentPositionError);
			if (motorBlock.stall)
			{
				// Reset the stall flag when the position error falls below half the tolerance, to avoid generating too many stall events
				if (errorThreshold <= 0 || positionErr < errorThreshold/2)
				{
					motorBlock.stall = false;
				}
			}
			else
			{
				const bool nowStalled = errorThreshold > 0 && positionErr > errorThreshold;
				if (nowStalled)
				{
					motorBlock.stall = true;
					motorBlock.faultPending = true;					// core 0 turns this into the driver-fault event (it needs FreeRTOS)
				}
				else
				{
					motorBlock.preStall = preErrorThreshold > 0 && positionErr > preErrorThreshold;
				}
			}
		}
		motorBlock.currentFraction = currentFraction;

		// Statistics for the periodic report (core 0 reads and resets these)
		const float absPositionError = fabsf(currentPositionError);
		if (absPositionError > motorBlock.statMaxAbsError) { motorBlock.statMaxAbsError = absPositionError; }
		motorBlock.statSumSqError += fsquare(currentPositionError);
		if (currentFraction > motorBlock.statMaxCurrentFraction) { motorBlock.statMaxCurrentFraction = currentFraction; }
		motorBlock.statSumCurrentFraction += currentFraction;
		++motorBlock.statSampleCount;
		++motorBlock.cycleCount;

		// Cycle runtime statistics
		const StepTimer::Ticks cycleRuntime = StepTimer::GetTimerTicks() - cycleStartTime;
		if (cycleRuntime < motorBlock.minCycleRuntime) { motorBlock.minCycleRuntime = cycleRuntime; }
		if (cycleRuntime > motorBlock.maxCycleRuntime) { motorBlock.maxCycleRuntime = cycleRuntime; }
	}
}

#endif	// RPXXXX && TMC_ON_CORE1

// End
