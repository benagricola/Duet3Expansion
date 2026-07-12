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
#include "SampleBuffer.h"

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

	// One iteration of the basic tuning manoeuvre, ported verbatim from ClosedLoop::BasicTuning
	// (Tuning.cpp) with ClosedLoop::desiredStepPhase replaced by sweepPhase. Returns true when finished.
	TIME_CRITICAL static bool BasicTuningStep(bool firstIteration, Encoder *encoder) noexcept
	{
		enum class BasicTuningState { forwardInitial = 0, forwards, reverseInitial, reverse };

		static BasicTuningState state;
		static uint16_t initialStepPhase;
		static unsigned int stepCounter;
		static int32_t initialCount;
		static float regressionAccumulator;
		static float readingAccumulator;

		constexpr unsigned int LinearEncoderIncreaseFactor = 4;	// mirrors the constant in ClosedLoop.h
		const unsigned int BasicPhaseIncrement = 4;
		const unsigned int NumDummySteps = 256/BasicPhaseIncrement;
		const unsigned int NumSamples = 4096/BasicPhaseIncrement;
		const float HalfNumSamplesMinusOne = (float)(NumSamples - 1) * 0.5;

		const uint16_t PhaseIncrement = (encoder->GetType() == EncoderType::linearComposite)
										? (uint16_t)(LinearEncoderIncreaseFactor * BasicPhaseIncrement)
											: (uint16_t)BasicPhaseIncrement;
		const float Denominator = (float)PhaseIncrement * (fcube((float)NumSamples) - (float)NumSamples)/12.0;

		if (!encoder->UsesBasicTuning())
		{
			return true;
		}

		if (firstIteration)
		{
			state = BasicTuningState::forwardInitial;
			stepCounter = 0;
			encoder->SetTuningBackwards(false);					// the kernel owns the encoder here, so no parking is needed
			encoder->ClearFullRevs();
		}

		const uint32_t currentPosition = sweepPhase;

		switch (state)
		{
		case BasicTuningState::forwardInitial:
			SetMotorPhase(currentPosition + PhaseIncrement, 1.0);
			sweepPhase = (uint16_t)(currentPosition + PhaseIncrement);
			++stepCounter;
			if (stepCounter == NumDummySteps)
			{
				regressionAccumulator = readingAccumulator = 0.0;
				stepCounter = 0;
				state = BasicTuningState::forwards;
			}
			break;

		case BasicTuningState::forwards:
			{
				int32_t reading = encoder->GetCurrentCount();
				if (stepCounter == 0)
				{
					initialCount = reading;
					initialStepPhase = currentPosition;
				}
				reading -= initialCount;
				readingAccumulator += (float)reading;
				regressionAccumulator += (float)reading * ((float)stepCounter - HalfNumSamplesMinusOne);
			}
			++stepCounter;
			if (stepCounter == NumSamples)
			{
				const float slope = regressionAccumulator / Denominator;
				const float xMean = (float)initialStepPhase + (float)PhaseIncrement * HalfNumSamplesMinusOne;
				const float yMean = readingAccumulator/NumSamples + (float)initialCount;
				encoder->SetForwardTuningResults(slope, xMean, yMean);
				stepCounter = 0;
				state = BasicTuningState::reverseInitial;
			}
			else
			{
				SetMotorPhase(currentPosition + PhaseIncrement, 1.0);
				sweepPhase = (uint16_t)(currentPosition + PhaseIncrement);
			}
			break;

		case BasicTuningState::reverseInitial:
			SetMotorPhase(currentPosition - PhaseIncrement, 1.0);
			sweepPhase = (uint16_t)(currentPosition - PhaseIncrement);
			++stepCounter;
			if (stepCounter == NumDummySteps)
			{
				regressionAccumulator = readingAccumulator = 0.0;
				stepCounter = 0;
				state = BasicTuningState::reverse;
			}
			break;

		case BasicTuningState::reverse:
			{
				int32_t reading = encoder->GetCurrentCount();
				if (stepCounter == 0)
				{
					initialCount = reading;
					initialStepPhase = currentPosition;
				}
				reading -= initialCount;
				readingAccumulator += (float)reading;
				regressionAccumulator += (float)reading * ((float)stepCounter - HalfNumSamplesMinusOne);
			}
			++stepCounter;
			if (stepCounter == NumSamples)
			{
				const float slope = regressionAccumulator / (-Denominator);
				const float xMean = (float)initialStepPhase - (float)PhaseIncrement * HalfNumSamplesMinusOne;
				const float yMean = readingAccumulator/NumSamples + (float)initialCount;
				encoder->SetReverseTuningResults(slope, xMean, yMean);
				return true;										// finished; core 0 calls FinishedBasicTuning()
			}
			else
			{
				SetMotorPhase(currentPosition - PhaseIncrement, 1.0);
				sweepPhase = (uint16_t)(currentPosition - PhaseIncrement);
			}
			break;
		}

		return false;
	}

	// One iteration of the encoder calibration (or calibration check) manoeuvre, ported verbatim from
	// ClosedLoop::EncoderCalibration (Tuning.cpp). Returns true when finished.
	TIME_CRITICAL static bool EncoderCalibrationStep(bool firstIteration, Encoder *encoder) noexcept
	{
		enum class EncoderCalibrationState { setup = 0, forwards, backwards };

		static EncoderCalibrationState state = EncoderCalibrationState::setup;
		static uint32_t positionsPerRev;
		static uint32_t positionsTillStart;
		static unsigned int phaseIncrementShift;
		static uint32_t positionCounter;

		if (!encoder->UsesCalibration())
		{
			return true;
		}

		const uint32_t currentPosition = sweepPhase;

		if (firstIteration)
		{
			positionsPerRev = encoder->GetPhasePositionsPerRev();

			phaseIncrementShift = 0;
			while ((positionsPerRev >> phaseIncrementShift) > Encoder::MaxCalibrationDataPoints)
			{
				++phaseIncrementShift;
			}

			encoder->ClearDataCollection(positionsPerRev >> phaseIncrementShift);

			// If calibrating (not checking), clear the mapping table. The kernel owns the encoder in
			// this mode, so unlike the core-0 version no parking is needed around these.
			if (sweepKind == MotorSweepKind::calibrate)
			{
				encoder->ClearLUT();
				encoder->SetCalibrationBackwards(false);
			}

			positionsTillStart = 4096 - currentPosition;
			if (positionsTillStart < 256)
			{
				positionsTillStart += 4096;
			}

			state = EncoderCalibrationState::setup;
		}

		const int32_t currentCount = encoder->GetCurrentShaftCount();

		switch (state)
		{
		case EncoderCalibrationState::setup:
			if (positionsTillStart != 0)
			{
				const uint32_t phaseChange = (positionsTillStart % (1u << phaseIncrementShift)) + (1u << phaseIncrementShift);
				positionsTillStart -= phaseChange;
				SetMotorPhase(currentPosition + phaseChange, 1.0);
				sweepPhase = (uint16_t)(currentPosition + phaseChange);
				return false;
			}

			positionCounter = 0;
			state = EncoderCalibrationState::forwards;
			[[fallthrough]];

		case EncoderCalibrationState::forwards:
			if (positionCounter < positionsPerRev)
			{
				encoder->RecordDataPoint(positionCounter >> phaseIncrementShift, currentCount, false);
			}

			SetMotorPhase(currentPosition + (1u << phaseIncrementShift), 1.0);
			sweepPhase = (uint16_t)(currentPosition + (1u << phaseIncrementShift));
			positionCounter += 1u << phaseIncrementShift;
			if (positionCounter == positionsPerRev + 256)
			{
				state = EncoderCalibrationState::backwards;
			}
			break;

		case EncoderCalibrationState::backwards:
			if (positionCounter < positionsPerRev)
			{
				encoder->RecordDataPoint(positionCounter >> phaseIncrementShift, currentCount, true);

				if (positionCounter == 0)
				{
					return true;									// finished; core 0 calls ReadyToCalibrate()
				}
			}

			SetMotorPhase(currentPosition - (1u << phaseIncrementShift), 1.0);
			sweepPhase = (uint16_t)(currentPosition - (1u << phaseIncrementShift));
			positionCounter -= 1u << phaseIncrementShift;
			break;
		}
		return false;
	}

	// Run the armed tuning sweep, one manoeuvre step per stepTicksPerTuningStep. Called from Cycle()
	// in directCommand mode. Arms/aborts are latched from the block via sweepArmSeq.
	TIME_CRITICAL static void RunTuningSweep(uint32_t now, Encoder *encoder) noexcept
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
			whenLastSweepStep = now;								// the first step happens one interval after arming
			motorBlock.sweepState = MotorSweepState::running;
		}

		if (motorBlock.sweepState != MotorSweepState::running)
		{
			return;
		}

		// Limit the rate at which we command tuning steps, as the pre-kernel-split code did
		if ((int32_t)(now - whenLastSweepStep) < (int32_t)stepTicksPerTuningStep)
		{
			return;
		}
		whenLastSweepStep = now;

		const bool finished = (sweepKind == MotorSweepKind::basicTuning)
								? BasicTuningStep(sweepFirstIteration, encoder)
									: EncoderCalibrationStep(sweepFirstIteration, encoder);
		sweepFirstIteration = false;
		if (finished)
		{
			motorBlock.sweepState = MotorSweepState::done;			// core 0 performs the completion
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
			RunTuningSweep(now, encoder);
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
			// Closed loop / assisted open loop, ported verbatim from ClosedLoop::ControlMotorCurrents.
			// The control signal is chosen to be in the range -256..256 (arbitrary, as in the original).
			const float PIDPTerm = constrain<float>(Kp * currentPositionError, -256.0, 256.0);
			const float PIDDTerm = constrain<float>(Kd * errorDerivativeFilter.GetDerivative() * StepTimer::StepClockRate, -256.0, 256.0);
			float PIDControlSignal, PIDVTerm, PIDATerm;
			uint16_t commandedStepPhase;
			if (mode == MotorMode::closedLoop)
			{
				const float timeDelta = (float)timeElapsed * (1.0/(float)StepTimer::StepClockRate);
				PIDITerm = constrain<float>(PIDITerm + Ki * currentPositionError * timeDelta, -PIDIlimit, PIDIlimit);
				PIDVTerm = mParams.speed * Kv * timeElapsed;
				PIDATerm = mParams.acceleration * Ka * fsquare((float)timeElapsed);
				PIDControlSignal = constrain<float>(PIDPTerm + PIDITerm + PIDDTerm + PIDVTerm + PIDATerm, -256.0, 256.0);

				// Phase of the motor current is always +/- 1 full step relative to the measured position,
				// with a speed-dependent feedforward, and the current magnitude follows the PID result.
				const float PhaseFeedForwardFactor = 1000.0;
				const int16_t phaseFeedForward = (int16_t)lrintf(constrain<float>(speedFilter.GetDerivative() * timeElapsed * PhaseFeedForwardFactor, -256.0, 256.0));
				const uint16_t adjustedStepPhase = (uint16_t)((int16_t)measuredStepPhase + phaseFeedForward) % 4096u;
				commandedStepPhase = (((PIDControlSignal < 0.0) ? (3 * 1024) : 1024) + adjustedStepPhase) % 4096u;
				currentFraction = fabsf(PIDControlSignal) * (1.0/256.0);
			}
			else
			{
				// Assisted open loop: the phase follows the commanded position; the I term is not used
				// and the A and V terms are independent of the loop time. The error only boosts the
				// current above the standstill floor.
				constexpr float scalingFactor = 100.0;
				PIDVTerm = mParams.speed * Kv * scalingFactor;
				PIDATerm = mParams.acceleration * Ka * fsquare(scalingFactor);
				PIDControlSignal = min<float>(fabsf(PIDPTerm + PIDDTerm) + fabsf(PIDVTerm) + fabsf(PIDATerm), 256.0);

				const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);	// llrintf so the float operand is always convertible; only the low 12 bits matter
				commandedStepPhase = (stepPhase + motorBlock.phaseOffset) % 4096u;
				currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(PIDControlSignal * (1.0/256.0), 1.0);
			}
			SetMotorPhase(commandedStepPhase, currentFraction);

			// Keep the control values for diagnostic sampling
			lastPIDControlSignal = PIDControlSignal;
			lastPIDPTerm = PIDPTerm;
			lastPIDDTerm = PIDDTerm;
			lastPIDVTerm = PIDVTerm;
			lastPIDATerm = PIDATerm;

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
