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
#include <hardware/structs/sio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/regs/io_bank0.h>
#include "Trigonometry.h"
#include "DerivativeAveragingFilter.h"
#include "Encoders/Encoder.h"
#include "SampleBuffer.h"
#include "TuningMoves.h"
#include "ControlLaw.h"

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
	static float holdCurrentFraction = 0.25;

	// Last-cycle control values kept for diagnostic sampling (the pre-kernel code sampled the
	// equivalent ClosedLoop instance fields)
	static float lastPIDControlSignal = 0.0, lastPIDPTerm = 0.0, lastPIDDTerm = 0.0, lastPIDVTerm = 0.0, lastPIDATerm = 0.0;
	static int16_t lastCoilA = 0, lastCoilB = 0;

	// Sample-streaming state (latched from the block when sampleArmSeq changes)
	static uint32_t lastSampleArmSeq = 0;
	static uint16_t sampleFilter = 0;
	static uint16_t samplesWanted = 0;
	static uint32_t sampleIntervalTicks = 1;
	static uint32_t whenNextSampleDue = 0;
	static uint32_t sampleStartTicks = 0;

	// Tuning-sweep state (latched from the block when sweepArmSeq changes). The sweep runs at the
	// pre-kernel-split step pacing: one step per stepTicksPerTuningStep, i.e. 2kHz.
	constexpr unsigned int tuningStepsPerSecond = 2000;			// mirrors the constant private to ClosedLoop
	constexpr uint32_t stepTicksPerTuningStep = StepTimer::StepClockRate/tuningStepsPerSecond;
	static uint32_t lastSweepArmSeq = 0;
	static MotorSweepKind sweepKind = MotorSweepKind::none;
	static bool sweepFirstIteration = true;
	static uint32_t whenLastSweepStep = 0;
	static uint16_t sweepPhase = 0;								// the manoeuvres' working phase (was ClosedLoop::desiredStepPhase)

	// Set the coil currents for the given phase and magnitude (0.0..1.0), and publish what we commanded
	TIME_CRITICAL static void SetMotorPhase(uint16_t phase, float magnitude) noexcept
	{
		float sine, cosine;
		Trigonometry::FastSinCos(phase, sine, cosine);
		const int16_t coilA = (int16_t)lrintf(cosine * magnitude);
		const int16_t coilB = (int16_t)lrintf(sine * magnitude);
		(void)SmartDrivers::SetMotorPhases(driverNumber, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
		motorBlock.commandedStepPhase = phase;
		lastCoilA = coilA;
		lastCoilB = coilB;
	}

	// Diagnostic sample streaming, ported from ClosedLoop::CollectSample and the scheduling logic
	// around it. The kernel packs samples straight into the shared SampleBuffer (the same
	// single-producer/single-consumer arrangement the pre-kernel core-1 loop used); core 0 mirrors
	// the progress fields into the transmission state machine.
	TIME_CRITICAL static void SampleStream(uint32_t now, bool hasMove, Encoder *encoder, const MotionParameters& mParams, float positionError) noexcept
	{
		const uint32_t armSeq = motorBlock.sampleArmSeq;
		if (armSeq != lastSampleArmSeq)
		{
			lastSampleArmSeq = armSeq;
			sampleFilter = motorBlock.sampleFilter;
			samplesWanted = motorBlock.samplesRequested;
			sampleIntervalTicks = motorBlock.sampleIntervalTicks;
			motorBlock.samplesCollected = 0;
			switch (motorBlock.sampleStartMode)
			{
			case 1:
				sampleStartTicks = whenNextSampleDue = now;
				motorBlock.sampleState = MotorSampleState::recording;
				break;
			case 2:
				motorBlock.sampleState = MotorSampleState::waitingForMove;
				break;
			default:
				motorBlock.sampleState = MotorSampleState::idle;
				break;
			}
		}

		if (motorBlock.sampleState == MotorSampleState::waitingForMove && hasMove)
		{
			sampleStartTicks = whenNextSampleDue = now;
			motorBlock.sampleState = MotorSampleState::recording;
		}

		if (motorBlock.sampleState == MotorSampleState::recording && (int32_t)(now - whenNextSampleDue) >= 0)
		{
			SampleBuffer *const buf = motorBlock.sampleBuffer;
			if (buf == nullptr)
			{
				motorBlock.sampleState = MotorSampleState::idle;
				return;
			}
			if (buf->IsFull())
			{
				motorBlock.sampleState = MotorSampleState::overflowed;			// tell core 0 to stop the collection
				return;
			}

			// Pack the sample in exactly the order the pre-kernel CollectSample used (the wire format)
			buf->PutF32((float)(now - sampleStartTicks) * StepTimer::StepClocksToMillis);	// always collect the timestamp
			if (sampleFilter & CL_RECORD_RAW_ENCODER_READING) 	{ buf->PutI32(encoder->GetCurrentCount()); }
			if (sampleFilter & CL_RECORD_CURRENT_MOTOR_STEPS) 	{ buf->PutF32((float)encoder->GetCurrentCount() * encoder->GetStepsPerCount()); }
			if (sampleFilter & CL_RECORD_TARGET_MOTOR_STEPS)  	{ buf->PutF32(mParams.position); }
			if (sampleFilter & CL_RECORD_CURRENT_ERROR) 		{ buf->PutF32(positionError); }
			if (sampleFilter & CL_RECORD_PID_CONTROL_SIGNAL)  	{ buf->PutF16(lastPIDControlSignal); }
			if (sampleFilter & CL_RECORD_PID_P_TERM)  			{ buf->PutF16(lastPIDPTerm); }
			if (sampleFilter & CL_RECORD_PID_I_TERM)  			{ buf->PutF16(PIDITerm); }
			if (sampleFilter & CL_RECORD_PID_D_TERM)  			{ buf->PutF16(lastPIDDTerm); }
			if (sampleFilter & CL_RECORD_PID_V_TERM)  			{ buf->PutF16(lastPIDVTerm); }
			if (sampleFilter & CL_RECORD_PID_A_TERM)  			{ buf->PutF16(lastPIDATerm); }
			if (sampleFilter & CL_RECORD_CURRENT_STEP_PHASE)  	{ buf->PutU16((uint16_t)encoder->GetCurrentPhasePosition()); }
			if (sampleFilter & CL_RECORD_DESIRED_STEP_PHASE)  	{ buf->PutU16(motorBlock.commandedStepPhase); }
			if (sampleFilter & CL_RECORD_PHASE_SHIFT)  			{ buf->PutU16(0); }
			if (sampleFilter & CL_RECORD_COIL_A_CURRENT) 		{ buf->PutI16(lastCoilA); }
			if (sampleFilter & CL_RECORD_COIL_B_CURRENT) 		{ buf->PutI16(lastCoilB); }
			buf->FinishSample();

			__asm volatile("dmb" ::: "memory");									// the sample data must be visible to core 0 before the count that publishes it
			const uint16_t collected = motorBlock.samplesCollected + 1;
			motorBlock.samplesCollected = collected;
			if (collected == samplesWanted)
			{
				motorBlock.sampleState = MotorSampleState::complete;
			}
			whenNextSampleDue += sampleIntervalTicks;
		}
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
			const float hold = motorBlock.holdCurrentFraction;
			if (motorBlock.paramSeq == seqBefore)
			{
				Kp = p; Ki = i; Kd = d; Kv = v; Ka = a;
				preErrorThreshold = pre; errorThreshold = err;
				holdCurrentFraction = hold;
				lastParamSeq = seqBefore;
				return;
			}
		}
	}
	// The tuning manoeuvre state machines are shared with the core-0 path: see TuningMoves.h. This
	// context supplies the kernel-side hooks. Completion side effects are no-ops here because the
	// kernel signals completion through the block and core 0 performs them (they need FreeRTOS/flash).
	struct KernelTuningContext
	{
		Encoder *encoder;
		uint16_t GetPhase() const noexcept { return sweepPhase; }
		void SetPhase(uint32_t phase) noexcept { SetMotorPhase((uint16_t)phase, 1.0); sweepPhase = (uint16_t)phase; }
		bool IsCalibrating() const noexcept { return sweepKind == MotorSweepKind::calibrate; }
		void FinishedBasic() const noexcept { }
		void ReadyToCalibrate() const noexcept { }
	};

	// Run the armed tuning sweep, one manoeuvre step per stepTicksPerTuningStep. Called from Cycle()
	// in directCommand mode. Arms/aborts are latched from the block via sweepArmSeq. Pacing uses the
	// LOCAL step clock (localNow): movement time can jump backwards when the movement delay changes,
	// which would stall a comparison-based limiter.
	TIME_CRITICAL static void RunTuningSweep(uint32_t localNow, Encoder *encoder) noexcept
	{
		const uint32_t armSeq = motorBlock.sweepArmSeq;
		if (armSeq != lastSweepArmSeq)
		{
			lastSweepArmSeq = armSeq;
			sweepKind = motorBlock.sweepKind;
			if (sweepKind == MotorSweepKind::none)
			{
				motorBlock.sweepState = MotorSweepState::idle;
				return;
			}
			sweepFirstIteration = true;
			sweepPhase = motorBlock.commandedStepPhase;				// continue from the last commanded phase (core 0 energised it before arming)
			whenLastSweepStep = localNow;							// the first step happens one interval after arming
			motorBlock.sweepIterations = 0;
			motorBlock.sweepState = MotorSweepState::running;
		}

		if (motorBlock.sweepState != MotorSweepState::running)
		{
			return;
		}

		// Limit the rate at which we command tuning steps, as the pre-kernel-split code did
		if ((int32_t)(localNow - whenLastSweepStep) < (int32_t)stepTicksPerTuningStep)
		{
			return;
		}
		whenLastSweepStep = localNow;

		KernelTuningContext ctx { encoder };
		const bool finished = (sweepKind == MotorSweepKind::basicTuning)
								? BasicTuningMove(ctx, sweepFirstIteration)
									: EncoderCalibrationMove(ctx, sweepFirstIteration);
		sweepFirstIteration = false;
		motorBlock.sweepIterations = motorBlock.sweepIterations + 1;
		if (finished)
		{
			motorBlock.sweepState = MotorSweepState::done;			// core 0 performs the completion
		}
	}



	// Continuous poll hook (registered with Core1Runtime by ClosedLoop::InitInstance): open-loop step
	// generation. In openLoopStep mode this runs at the host loop's poll rate, microseconds apart.
	TIME_CRITICAL void KernelYieldPoll() noexcept
	{
		if (motorBlock.stepTestRequest == 1)
		{
			// Bench diagnostic: emit test pulses from THIS core via the SIO GPIO registers (the same
			// mechanism the ISR stepping path uses from core 0) so cross-core GPIO problems can be
			// discriminated from step-generation problems. Earlier "core-1 SIO doesn't reach the
			// pads" conclusions were artifacts: MSCNT deltas alias to zero for x16 pulse counts that
			// are multiples of 64, and the G1-path pulses were too narrow for the TMC's filtered
			// STEP input. Read back CTRL/STATUS/SIO so core 0 can report write delivery regardless.
			const uint32_t mask = motorBlock.stepPollMask;
			unsigned int pin = 0;
			while (pin < 31 && (mask & (1u << pin)) == 0) { ++pin; }
			// 500us high/low paced by the step timer: slow enough for the rotor to physically follow,
			// so the encoder read between tests gives ground truth (MSCNT proved unreliable at speed)
			constexpr uint32_t halfPeriodTicks = (uint64_t)StepTimer::StepClockRate * 500 / 1000000;
			for (unsigned int n = 0; n < 1600; ++n)
			{
				sio_hw->gpio_set = mask;
				if (n == 0)
				{
					motorBlock.stepTestCtrlWritten = mask;
					motorBlock.stepTestCtrlReadback = io_bank0_hw->io[pin].ctrl;
					motorBlock.stepTestStatusHigh = io_bank0_hw->io[pin].status;
					motorBlock.stepTestSioReadback = sio_hw->gpio_out;
				}
				uint32_t t0 = StepTimer::GetTimerTicks();
				while (StepTimer::GetTimerTicks() - t0 < halfPeriodTicks) { }
				sio_hw->gpio_clr = mask;
				t0 = StepTimer::GetTimerTicks();
				while (StepTimer::GetTimerTicks() - t0 < halfPeriodTicks) { }
			}
			motorBlock.stepTestRequest = 2;
		}
		if (motorBlock.mode == MotorMode::openLoopStep)
		{
			MotorControlStepPoll();
		}
	}

	// One control cycle. Ported from ClosedLoop::InstanceControlLoop + ControlMotorCurrents
	// (ClosedLoopMode::closed branch); see the file header comment.
	TIME_CRITICAL void Cycle() noexcept
	{
		const MotorMode mode = motorBlock.mode;
		Encoder *const encoder = motorBlock.encoder;
		if (mode == MotorMode::idle || mode == MotorMode::openLoopStep || encoder == nullptr)
		{
			// idle: core 0 owns the encoder and the motor. openLoopStep: stepping happens in
			// KernelYieldPoll, and core 0 owns the encoder (the kernel must not read it in open loop)
			prevCycleTimeValid = false;								// don't count the gap as a cycle interval
			return;
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
		bool hasMove = false;
		if (mode == MotorMode::closedLoop || mode == MotorMode::assistedOpen)
		{
			hasMove = MotorControlGetTrajectory(now, mParams);
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
			RunTuningSweep(cycleStartTime, encoder);				// paced on the local step clock, not movement time
			const MotorSweepState sweepState = motorBlock.sweepState;
			if (sweepState == MotorSweepState::running)
			{
				currentFraction = 1.0;								// the sweep commands full current
			}
			else if (sweepState == MotorSweepState::idle)
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
			// A completed sweep (done) holds its final phase: applying the stale pre-sweep command
			// here would snap the rotor back; core 0 changes the mode as part of the completion.
		}
		else
		{
			// Closed loop / assisted open loop - the control mathematics live in ControlLaw.h,
			// shared verbatim with ClosedLoop::ControlMotorCurrents. The last* statics bound here
			// feed the M569.5 sample stream.
			uint16_t commandedStepPhase;
			if (mode == MotorMode::closedLoop)
			{
				MotorControlLaw::ComputeClosedLoop(Kp, Ki, Kd, Kv, Ka,
									currentPositionError, errorDerivativeFilter.GetDerivative(), speedFilter.GetDerivative(),
									mParams.speed, mParams.acceleration, timeElapsed, measuredStepPhase,
									lastPIDPTerm, PIDITerm, lastPIDDTerm, lastPIDVTerm, lastPIDATerm, lastPIDControlSignal,
									commandedStepPhase, currentFraction);
			}
			else
			{
				MotorControlLaw::ComputeAssistedOpen(Kp, Kd, Kv, Ka,
									currentPositionError, errorDerivativeFilter.GetDerivative(),
									mParams.speed, mParams.acceleration,
									mParams.position, motorBlock.phaseOffset, holdCurrentFraction,
									lastPIDPTerm, lastPIDDTerm, lastPIDVTerm, lastPIDATerm, lastPIDControlSignal,
									commandedStepPhase, currentFraction);
			}
			SetMotorPhase(commandedStepPhase, currentFraction);

			// Stall / pre-stall detection - shared hysteresis; the fault event is deferred to core 0
			bool stall = motorBlock.stall, preStall = motorBlock.preStall;
			if (MotorControlLaw::UpdateStallDetection(fabsf(currentPositionError), preErrorThreshold, errorThreshold, stall, preStall))
			{
				motorBlock.faultPending = true;						// core 0 turns this into the driver-fault event (it needs FreeRTOS)
			}
			motorBlock.stall = stall;
			motorBlock.preStall = preStall;
		}
		motorBlock.currentFraction = currentFraction;

		SampleStream(now, hasMove, encoder, mParams, currentPositionError);

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
