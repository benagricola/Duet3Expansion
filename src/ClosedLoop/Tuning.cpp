
#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

#include "Encoders/Encoder.h"
#include "TuningMoves.h"

// NOTE: the BasicTuning and EncoderCalibration manoeuvre state machines live in TuningMoves.h,
// shared with the core-1 motor kernel (which runs them at the native step pacing under
// TMC_ON_CORE1); the functions here are the core-0 host wrappers, used when the kernel is not.

# if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support closed loop with the specified hardware
# endif

/*
 * The following sections, delimited by comments, implement different tuning moves.
 * The comment gives the name of the move, and a description for how it is implemented
 * for both relative and absolute encoders.
 * Each function should perform one 'iteration' of the tuning move, and return true if
 * the iteration was it's last. It will also be supplied with an argument representing
 * if this is it's first iteration.
 *
 * At the bottom of the file, ClosedLoop::PerformTune() is implemented to take advantage
 * of these function
 */


/*
 * Basic tuning
 * ------------------
 *
 *  - Increase the step phase by a little over 4096 counts and back again
 *  - ignore the points near the start position
 *  - feed the remaining (phase, encoder reading) points into a linear regression algorithm, separately on the forward and the reverse movements
 *  - the linear regression gives us the encoder offset and the counts per step
 *  - pass these figures to the ClosedLoop module. It will check the counts per step, set the forward/reverse encoder polarity flag, and set the zero position
 *
 *  Notes on linear regression:
 *  From https://en.wikipedia.org/wiki/Simple_linear_regression the formula to fit a straight line y = mx + c to a set of N (x, y) points is:
 *   m = sigma(i=0..(N-1): (xi - xm) * (yi - ym)) / sigma(i=0..(N-1): (xi - xm)^2)
 *   c = ym - m * xm
 *  where xi is the ith x, yi is the ith y, xm is the mean x, ym is the mean y
 *  In our case the x values are the motor phase values selected, which are spaced uniformly, so that xi = x0 + p*i
 *  So xm = (x0 + (x0 + p*(N-1)))/2 = x0 + p*(N-1)/2
 *  and (xi - xm) = x0 + p*i - x0 - p*(N-1)/2 = p*(i - (N-1)/2)
 *
 *  Expand the numerator in the equation for m to:
 *   sigma(i=0..(N-1): yi*(xi - xm)) - ym*sigma(i=0..(N-1): (xi - xm))
 *  Simplify this to:
 *   sigma(i=0..N-1): yi*p*(i - (N-1)/2)) - ym*sigma(i=0..(N-1): p*(i - (N-1)/2))
 *  and further to:
 *   p * sigma(i=0..N-1): yi*(i - (N-1)/2)) - ym*p*(N * (N-1)/2 - N * (N-1)/2)
 *  and even further to:
 *   p * (sigma(i=0..N-1): yi*(i - (N-1)/2))
 *  We can accumulate the first term as we take readings, and we can accumulate the sum of the y values so we can calculate ym at the end.
 *
 *  The denominator in the equation for m expands to:
 *   sigma(i=0..(N-1): p^2*(i - (N-1)/2)^2)
 *  Expand this to:
 *   p^2 * sigma(i=0..(N-1): i^2) - sigma(i=0..(N-1): i * (N-1)) + sigma(i=0..(N-1): ((N-1)/2)^2)
 *  Simplify to:
 *   p^2 * sigma(i=0..(N-1): i^2) - (N-1) * sigma(i=0..(N-1): i) + N * ((N-1)/2)^2
 *  Using sigma(i=0..(N-1): i) = N * (N-1)/2, sigma(i=0..(N-1): i^2) = (N * (N-1) * (2*N - 1))/6 we get:
 *   p^2 * ((N * (N-1) * (2*N - 1))/6 - (N-1)*N * (N-1)/2 + N * ((N-1)/2)^2)
 *  which simplifies to:
 *   p^2*(N^3-N)/12
 *  So numerator/denominator is:
 *   (sigma(i=0..N-1): yi*(i - (N-1)/2))) / (p*(N^3-N)/12)
 */

bool ClosedLoop::BasicTuning(bool firstIteration) noexcept
{
	// The manoeuvre state machine itself is shared with the core-1 motor kernel: see TuningMoves.h
	struct Ctx
	{
		ClosedLoop& cl;
		Encoder *encoder;
		uint16_t GetPhase() const noexcept { return cl.desiredStepPhase; }
		void SetPhase(uint32_t phase) noexcept { cl.SetMotorPhase((uint16_t)phase, 1.0); }
		bool IsCalibrating() const noexcept { return (cl.tuning & ENCODER_CALIBRATION_MANOEUVRE) != 0; }
		void FinishedBasic() noexcept { cl.FinishedBasicTuning(); }
		void ReadyToCalibrate() noexcept { cl.ReadyToCalibrate(IsCalibrating()); }
	};
	Ctx ctx { *this, encoder };
	return BasicTuningMove(ctx, firstIteration);
}


/*
 * Magnetic encoder calibration or calibration check
 * -------------------------------------------------
 *
 * Absolute:
 * 	- Move forwards somewhat (to counter any backlash) and then to the next full step position (to give (hopefully) consistent results)
 * 	- Move forwards at a constant rate. At each position, take the current encoder reading and update the Fourier coefficients
 * 	- Store the Fourier coefficients in the encoder LUT
 */

bool ClosedLoop::EncoderCalibration(bool firstIteration) noexcept
{
	// The manoeuvre state machine itself is shared with the core-1 motor kernel: see TuningMoves.h
	struct Ctx
	{
		ClosedLoop& cl;
		Encoder *encoder;
		uint16_t GetPhase() const noexcept { return cl.desiredStepPhase; }
		void SetPhase(uint32_t phase) noexcept { cl.SetMotorPhase((uint16_t)phase, 1.0); }
		bool IsCalibrating() const noexcept { return (cl.tuning & ENCODER_CALIBRATION_MANOEUVRE) != 0; }
		void FinishedBasic() noexcept { cl.FinishedBasicTuning(); }
		void ReadyToCalibrate() noexcept { cl.ReadyToCalibrate(IsCalibrating()); }
	};
	Ctx ctx { *this, encoder };
	return EncoderCalibrationMove(ctx, firstIteration);
}


/*
 * Continuous Phase Increase
 * -------------
 *
 * Absolute:
 * Relative:
 *  - TODO!
 *
 */

#if 0	// not implemented

bool ClosedLoop::ContinuousPhaseIncrease(bool firstIteration) noexcept
{
	return true;
}

#endif

/*
 * Step
 * -------------
 *
 * Absolute:
 * Relative:
 *  - Increase the target motor steps by 4
 *
 */

bool ClosedLoop::Step(bool firstIteration) noexcept
{
	AdjustTargetMotorSteps(4.0);
	return true;
}


/*
 * Ziegler Nichols Manoeuvre
 * -------------
 *
 * Absolute:
 * Relative:
 *  - TODO!
 *
 */

#if 1
// TODO: Implement ziegler-Nichols move

#else
bool ClosedLoop::ZieglerNichols(bool firstIteration) noexcept
{

	// We will need to restore these afterwards...
	const float prevKp = Kp;
	const float prevKi = Ki;
	const float prevKd = Kd;

	// Reset the PID controller
	Ki = 0;
	Kd = 0;
	Kp = 0;
	PIDITerm = 0;

	ultimateGain = 0;		// Reset the ultimate gain value
	int direction = 1;		// Which direction are we moving in

	float lowerBound = 0;
	float upperBound = 10000;

	while (upperBound - lowerBound > 100) {

		Kp = lowerBound + (upperBound - lowerBound) / 2;

		targetMotorSteps = currentMotorSteps + (direction * 10);

		// Flip the direction
		direction = -direction;

		unsigned int initialRiseTime = 0;		// The time it takes to initially meet the target

		float peakError = 0;			// The peak of the current oscillation
		float prevPeakError = 0;		// The peak of the previous oscillation
		unsigned int prevTimestamp = 0;			// The previous time of oscillation

		unsigned int oscillationCount = 0;		// The number of oscillations that have occurred

		float ewmaDecayFraction = 0;	// An EWMA of the decay fraction of oscillations
		float ewmaOscillationPeriod = 0;// An EWMA of the oscillation period

		// Run up to a maximum of 4096
		for (unsigned int time=0; time<16384; time++) {
			TaskBase::Take(10);		// TODO: Use delayuntil here? And run at PID frequency

			ControlMotorCurrents();

			float currentPosition = direction * currentMotorSteps;
			float targetPosition = direction * targetMotorSteps;
			float error = abs(currentPosition - targetPosition);

			// Search for the initial rise time
			if (initialRiseTime == 0) {
				if (currentPosition > targetPosition) {
					initialRiseTime = time;
				} else {
					continue;
				}
			}

			// Wait another two initial rise times for oscillations to occur
			if (time < 3 * initialRiseTime) {continue;}

			// We're now in the prime time for oscillations - check if they are actually happening:

			// Record data if we are above the target
			if (currentPosition > targetPosition) {
				peakError = max<float>(peakError, error);
				continue;
			}
			// Process data if we have just crossed the target
			float decayFraction;
			if (peakError > 0) {
				if (prevPeakError > 0) {
					decayFraction = peakError / prevPeakError;
					ewmaDecayFraction =
							ewmaDecayFraction == 0
							? decayFraction
							: 0.7 * ewmaDecayFraction + 0.3 * decayFraction;
					if (oscillationCount > 5) {
						ewmaOscillationPeriod =
								ewmaOscillationPeriod == 0
								? (time - prevTimestamp)
								: 0.3 * ewmaOscillationPeriod + 0.7 * (time - prevTimestamp);
					}
				}
				oscillationCount++;
				prevPeakError = peakError;
				peakError = 0;
				prevTimestamp = time;
			}

			PIDPTerm = ewmaOscillationPeriod;
			PIDDTerm = (time - prevTimestamp);

			// Wait for at least 5 oscillations
			if (oscillationCount < 5) {
				continue;
			}

			// Check that the next 5 oscillations all keep the average decay fraction above 98%
			if (ewmaDecayFraction < 0.98) {
				// No oscillations, this is the new lower bound.
				lowerBound = Kp;
				break;
			}
			if (oscillationCount >= 10) {
				// Oscillations found! This is the new upper bound.
				upperBound = Kp;
				oscillationPeriod = ewmaOscillationPeriod;
				break;
			}

			// If we time out of this loop, assume no oscillations
			if (time == 16383) {
				lowerBound = Kp;
			}

		}
	}

	ultimateGain = upperBound;
	Kp = prevKp;
	Ki = prevKi;
	Kd = prevKd;

	tuning &= ~ZIEGLER_NICHOLS_MANOEUVRE;
}
#endif


/*
 * ClosedLoop::PerformTune()
 * -------------------------
 *
 * Makes use of the above tuning functions.
 *
 */

// This is called from the closed loop control loop every (1/tuningStepsPerSecond) seconds if tuning is enabled, currently every 0.5ms
void ClosedLoop::PerformTune() noexcept
{
	static bool newTuningMove = true;						// indicates if a tuning move has just finished

	// Check we are in direct drive mode and we have an encoder
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct || encoder == nullptr)
	{
		tuningError |= TuningError::SystemError;
		tuning = 0;
		return;
	}

	// Run one iteration of the one, highest priority, tuning move
	if (tuning & BASIC_TUNING_MANOEUVRE)
	{
		newTuningMove = BasicTuning(newTuningMove);
		if (newTuningMove)
		{
			tuning &= ~BASIC_TUNING_MANOEUVRE;				// we can do encoder calibration after basic tuning
		}
	}
	else if (tuning & (ENCODER_CALIBRATION_MANOEUVRE | ENCODER_CALIBRATION_CHECK))
	{
		if (tuningError & (TuningError::TooMuchMotion | TuningError::TooLittleMotion | TuningError::InconsistentMotion))
		{
			// Basic tuning failed, so don't attempt encoder calibration because it may not complete
			tuning = 0;
		}
		else
		{
			newTuningMove = EncoderCalibration(newTuningMove);
			if (newTuningMove)
			{
				tuning = 0;
			}
		}
	}
	else if (tuning & STEP_MANOEUVRE)
	{
		newTuningMove = Step(newTuningMove);
		if (newTuningMove)
		{
			tuning = 0;
		}
#if 0	// not implemented
	} else if (tuning & CONTINUOUS_PHASE_INCREASE_MANOEUVRE) {
		newTuningMove = ContinuousPhaseIncrease(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
	} else if (tuning & ZIEGLER_NICHOLS_MANOEUVRE) {
		newTuningMove = ZieglerNichols(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
#endif
	}
	else
	{
		tuning = 0;
		newTuningMove = true;								// ready for next time
	}
}

#endif	// #if SUPPORT_CLOSED_LOOP
