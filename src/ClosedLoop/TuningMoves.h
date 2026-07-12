/*
 * TuningMoves.h
 *
 * The tuning manoeuvre state machines (basic tuning and encoder calibration), shared between the
 * two hosts that can execute them:
 *  - the pre-existing core-0 path (ClosedLoop::PerformTune, Tuning.cpp), used when the motor kernel
 *    is not in use;
 *  - the core-1 motor kernel (MotorControlLoop.cpp), which runs them inside the control cycle at the
 *    native step pacing when TMC_ON_CORE1.
 * There is ONE copy of the manoeuvre logic - this file - taken verbatim from the proven Tuning.cpp
 * implementations; the hosts differ only through the small context type:
 *
 *   struct Ctx {
 *       Encoder *encoder;
 *       uint16_t GetPhase();           // the working step phase (was ClosedLoop::desiredStepPhase)
 *       void SetPhase(uint32_t p);     // command the coils to phase p at full current and update the working phase
 *       bool IsCalibrating();          // encoder calibration (clear + store the LUT) vs calibration check
 *       void FinishedBasic();          // completion side effects; no-ops in the kernel, which signals
 *       void ReadyToCalibrate();       //   completion through the motor control block instead
 *   };
 *
 * Each function performs one iteration of its manoeuvre and returns true when the manoeuvre has
 * finished. The caller is responsible for pacing the iterations (one per tuning step interval).
 *
 * This header is included by the core-1 kernel, so it must not (transitively) include FreeRTOS, CAN,
 * the NVM/flash API or the object model, and the manoeuvre code must not call them. The encoder
 * methods used here are arithmetic and RAM-only. (The DEBUG prints the core-0 version once had are
 * omitted for this reason.)
 *
 * The linear-regression derivation for basic tuning is documented in Tuning.cpp.
 */

#ifndef SRC_CLOSEDLOOP_TUNINGMOVES_H_
#define SRC_CLOSEDLOOP_TUNINGMOVES_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

#include "Encoders/Encoder.h"

constexpr unsigned int TuningLinearEncoderIncreaseFactor = 4;	// power of 2; linear composite encoders have more backlash, so move further

// Perform one iteration of the basic tuning manoeuvre. Returns true when finished.
template<class Ctx> bool BasicTuningMove(Ctx& ctx, bool firstIteration) noexcept
{
	enum class BasicTuningState { forwardInitial = 0, forwards, reverseInitial, reverse };

	static BasicTuningState state;									// state machine control
	static uint16_t initialStepPhase;								// the step phase we started at
	static unsigned int stepCounter;								// a counter to use within a state
	static int32_t initialCount;
	static float regressionAccumulator;
	static float readingAccumulator;

	// The following parameters define how much we move the motor before taking samples (to overcome backlash) and how far we move the motor.
	const unsigned int BasicPhaseIncrement = 4;						// how much we normally increase the motor phase by on each step
	static_assert(4096 % BasicPhaseIncrement == 0);
	const unsigned int NumDummySteps = 256/BasicPhaseIncrement;		// how many dummy increments to use before we start collecting data, to overcome backlash. (normally 1/4 step)
	const unsigned int NumSamples = 4096/BasicPhaseIncrement;		// the number of samples we take to do the linear regression (normally 4 full steps)
	const float HalfNumSamplesMinusOne = (float)(NumSamples - 1) * 0.5;

	// When using linear composite encoders we expect more backlash, therefore we increase the size of the phase increment.
	const uint16_t PhaseIncrement = (ctx.encoder->GetType() == EncoderType::linearComposite)
									? (uint16_t)(TuningLinearEncoderIncreaseFactor * BasicPhaseIncrement)
										: (uint16_t)BasicPhaseIncrement;
	static_assert(4096 % (TuningLinearEncoderIncreaseFactor * BasicPhaseIncrement) == 0);

	const float Denominator = (float)PhaseIncrement * (fcube((float)NumSamples) - (float)NumSamples)/12.0;

	if (!ctx.encoder->UsesBasicTuning())
	{
		return true;
	}

	if (firstIteration)
	{
		state = BasicTuningState::forwardInitial;
		stepCounter = 0;
		ctx.encoder->SetTuningBackwards(false);
		ctx.encoder->ClearFullRevs();
	}

	const uint32_t currentPosition = ctx.GetPhase();

	switch (state)
	{
	case BasicTuningState::forwardInitial:
		// In this state we move forwards a few microsteps to allow the motor to settle down
		ctx.SetPhase(currentPosition + PhaseIncrement);
		++stepCounter;
		if (stepCounter == NumDummySteps)
		{
			regressionAccumulator = readingAccumulator = 0.0;
			stepCounter = 0;
			state = BasicTuningState::forwards;
		}
		break;

	case BasicTuningState::forwards:
		// Collect data and move forwards, until we have moved 4 full steps
		{
			int32_t reading = ctx.encoder->GetCurrentCount();
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
			// Save the accumulated data
			const float slope = regressionAccumulator / Denominator;										// the average encoder counts per phase position
			const float xMean = (float)initialStepPhase + (float)PhaseIncrement * HalfNumSamplesMinusOne;	// the average phase
			const float yMean = readingAccumulator/NumSamples + (float)initialCount;						// the average count
			ctx.encoder->SetForwardTuningResults(slope, xMean, yMean);
			stepCounter = 0;
			state = BasicTuningState::reverseInitial;
		}
		else
		{
			ctx.SetPhase(currentPosition + PhaseIncrement);
		}
		break;

	case BasicTuningState::reverseInitial:
		// In this state we move backwards a few microsteps to allow the motor to settle down
		ctx.SetPhase(currentPosition - PhaseIncrement);
		++stepCounter;
		if (stepCounter == NumDummySteps)
		{
			regressionAccumulator = readingAccumulator = 0.0;
			stepCounter = 0;
			state = BasicTuningState::reverse;
		}
		break;

	case BasicTuningState::reverse:
		// Collect data and move backwards, until we have moved 4 full steps
		{
			int32_t reading = ctx.encoder->GetCurrentCount();
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
			// Save the accumulated data
			const float slope = regressionAccumulator / (-Denominator);			// negate the denominator because the phase increment was negative this time
			const float xMean = (float)initialStepPhase - (float)PhaseIncrement * HalfNumSamplesMinusOne;
			const float yMean = readingAccumulator/NumSamples + (float)initialCount;
			ctx.encoder->SetReverseTuningResults(slope, xMean, yMean);
			ctx.FinishedBasic();
			return true;																// finished tuning
		}
		else
		{
			ctx.SetPhase(currentPosition - PhaseIncrement);
		}
		break;
	}

	return false;
}

// Perform one iteration of the magnetic encoder calibration or calibration check manoeuvre.
// Returns true when finished.
template<class Ctx> bool EncoderCalibrationMove(Ctx& ctx, bool firstIteration) noexcept
{
	enum class EncoderCalibrationState { setup = 0, forwards, backwards };

	static EncoderCalibrationState state = EncoderCalibrationState::setup;
	static uint32_t positionsPerRev;			// this gets set to 1024 * the number of full steps per revolution, i.e. 204800 or 409600
	static uint32_t positionsTillStart;			// the position we advance to before we start tuning proper
	static unsigned int phaseIncrementShift;	// we increase the phase position by one << this value for each sample
	static uint32_t positionCounter;			// how many positions we have moved

	if (!ctx.encoder->UsesCalibration())
	{
		return true;							// we don't do this tuning for relative encoders
	}

	const uint32_t currentPosition = ctx.GetPhase();

	if (firstIteration)
	{
		// Set up some variables
		positionsPerRev = ctx.encoder->GetPhasePositionsPerRev();

		// Decide how many phase positions to advance at a time. This is down to the steps/rev ands the size of our calibration data storage array.
		phaseIncrementShift = 0;
		while ((positionsPerRev >> phaseIncrementShift) > Encoder::MaxCalibrationDataPoints)
		{
			++phaseIncrementShift;
		}

		ctx.encoder->ClearDataCollection(positionsPerRev >> phaseIncrementShift);

		// If calibrating (not checking), clear the mapping table
		if (ctx.IsCalibrating())
		{
			ctx.encoder->ClearLUT();
			ctx.encoder->SetCalibrationBackwards(false);
		}

		// To counter any backlash, start by advancing a bit. Then advance to the next position which is a multiple of 4 full steps so that the phase position is zero.
		positionsTillStart = 4096 - currentPosition;
		if (positionsTillStart < 256)
		{
			positionsTillStart += 4096;
		}

		state = EncoderCalibrationState::setup;
	}

	const int32_t currentCount = ctx.encoder->GetCurrentShaftCount();

	switch (state)
	{
	case EncoderCalibrationState::setup:
		// Advancing to a suitable full step position
		if (positionsTillStart != 0)
		{
			const uint32_t phaseChange = (positionsTillStart % (1u << phaseIncrementShift)) + (1u << phaseIncrementShift);
			positionsTillStart -= phaseChange;
			ctx.SetPhase(currentPosition + phaseChange);
			return false;
		}

		positionCounter = 0;
		state = EncoderCalibrationState::forwards;
		[[fallthrough]];

	case EncoderCalibrationState::forwards:
		// Advancing slowly and recording positions
		if (positionCounter < positionsPerRev)
		{
			ctx.encoder->RecordDataPoint(positionCounter >> phaseIncrementShift, currentCount, false);
		}

		// Move to the next position. After a complete revolution we continue another 256 positions without recording data, ready for the reverse pass.
		ctx.SetPhase(currentPosition + (1u << phaseIncrementShift));
		positionCounter += 1u << phaseIncrementShift;
		if (positionCounter == positionsPerRev + 256)
		{
			state = EncoderCalibrationState::backwards;
		}
		break;

	case EncoderCalibrationState::backwards:
		if (positionCounter < positionsPerRev)
		{
			ctx.encoder->RecordDataPoint(positionCounter >> phaseIncrementShift, currentCount, true);

			if (positionCounter == 0)
			{
				// We are finished
				ctx.ReadyToCalibrate();
				return true;
			}
		}

		// Move to the next position
		ctx.SetPhase(currentPosition - (1u << phaseIncrementShift));
		positionCounter -= 1u << phaseIncrementShift;
		break;
	}
	return false;
}

#endif	// SUPPORT_CLOSED_LOOP

#endif /* SRC_CLOSEDLOOP_TUNINGMOVES_H_ */
