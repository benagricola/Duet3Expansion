/*
 * Core1Runtime.cpp
 *
 * See Core1Runtime.h. The core-1 main loop and everything it calls must be RAM-resident
 * (TIME_CRITICAL): core 1 must not fetch from flash, both because flash operations on core 0
 * require it, and because a control loop on core 1 must not stall on the shared XIP cache.
 */

#include "Core1Runtime.h"

#if RPXXXX && SPICAN_CORE0_SERVICE

#include <pico/multicore.h>
#include <pico/platform.h>
#include <hardware/structs/timer.h>
#include <hardware/structs/psm.h>
#include <hardware/structs/sio.h>
#include <hardware/structs/watchdog.h>
#include <hardware/irq.h>
#include <Platform/Tasks.h>
#include <Movement/StepTimer.h>
#include <atomic>

namespace Core1Runtime
{
	// Mailbox and state shared between the cores. All fields are written by exactly one side
	// except where noted; the sequence-number handshake orders the accesses.
	static volatile uint32_t heartbeat = 0;					// written by core 1
	static volatile bool parkRequested = false;				// written by core 0
	static volatile bool parked = false;					// written by core 1
	static volatile uint32_t cmdSeq = 0;					// written by core 0; a new value submits the command
	static volatile uint32_t cmdAckSeq = 0;					// written by core 1 when the command has been executed
	static volatile Command cmdOp = Command::none;			// written by core 0 before bumping cmdSeq
	static volatile uint32_t cmdArg0 = 0;					// written by core 0 before bumping cmdSeq
	static volatile uint32_t cmdResult = 0;					// written by core 1 before bumping cmdAckSeq

	static spin_lock_t *crossCoreLock = nullptr;
	static spin_lock_t *segmentPoolLock = nullptr;
	static bool started = false;
	static std::atomic<int> parkDepth(0);
	static Core1EntryFn core1Entry = nullptr;
	static volatile uint32_t parkTimeouts = 0;				// times Park() gave up waiting for the acknowledgement (its caller proceeded unparked)
	static volatile uint32_t parkCount = 0;					// total park acquisitions (diagnostics)

	// Diagnostics for the core-1 relaunch (surfaced by the 'D' USB report). On RP2350 a watchdog
	// reset does not power-cycle core 1, so after a firmware update or software reset core 1 has been
	// running continuously since the last cold boot; the stock relaunch handshake then hangs on that
	// asymmetry. RobustResetCore1() hard power-cycles core 1 with a bounded, retried handshake so the
	// reset cannot hang, and Start() boots without core 1 (setting launchFailed) rather than lock up
	// if core 1 never acknowledges.
	static volatile bool launchFailed = false;			// core 1 could not be brought up; core 0 booted without it
	static volatile uint32_t lastResetAttempts = 0;		// how many force-off/on attempts the last reset needed (0 = not yet run)

	// The SDK's multicore_launch_core1() blocks unboundedly in a FIFO handshake and has been seen to
	// hang on a warm boot; it cannot be bounded or reimplemented (its core-1 side is static in the
	// SDK). The escape is the hardware watchdog: vApplicationTickHook suspends its kick while
	// launchInProgress is set, so a hang inside the launch window reboots in ~1s instead of wedging
	// silently. Launch attempts are counted in a watchdog scratch register (which survives warm
	// resets) so the reboot loop is bounded: after MaxLaunchAttempts the next boot gives up and boots
	// without core 1, leaving the board responsive. A progress code in another scratch register
	// records how far the last attempt got, for the 'D' report.
	static volatile bool launchInProgress = false;		// true only inside Start()'s reset/launch window (read by vApplicationTickHook)
	static uint32_t priorLaunchAttempts = 0;			// launch attempts consumed by watchdog reboots before this boot's Start()

	constexpr uint32_t MaxLaunchAttempts = 3;
	constexpr size_t LaunchAttemptScratchIndex = 1;		// scratch[0] is the firmware-update magic word; [4..7] belong to the SDK watchdog code
	constexpr size_t LaunchProgressScratchIndex = 3;

#if MNB_USB_DIAG
	constexpr size_t LaunchHangTestScratchIndex = 2;	// armed by the bench-only 1202-baud USB handler (SerialCDC_tusb.cpp)
	constexpr uint32_t LaunchHangTestMagic = 0x7E57CAFE;
#endif

	// Progress codes written to scratch[LaunchProgressScratchIndex]
	constexpr uint32_t LaunchProgressEntered = 1;		// Start() entered the launch window
	constexpr uint32_t LaunchProgressResetOk = 2;		// core-1 hard reset acknowledged
	constexpr uint32_t LaunchProgressLaunching = 3;		// about to call multicore_launch_core1 (a hang here is the known failure)
	constexpr uint32_t LaunchProgressDone = 4;			// multicore_launch_core1 returned; core 1 running
	constexpr uint32_t LaunchProgressResetFailed = 0x20;	// core-1 hard reset never acknowledged

	// Bounded, retrying replacement for the SDK's multicore_reset_core1() (which pop_blocking-waits
	// forever for core 1's readiness word). Hard power-cycles core 1 through the PSM and waits, bounded,
	// for the bootrom to signal ready; retries the whole power-cycle if it does not. Returns false if
	// core 1 never acknowledges, so the caller can boot without it instead of hanging.
	static bool RobustResetCore1(uint32_t perAttemptMillis, unsigned int maxAttempts) noexcept
	{
		const uint irqNum = SIO_FIFO_IRQ_NUM(0);
		const bool irqWasEnabled = irq_is_enabled(irqNum);
		irq_set_enabled(irqNum, false);						// the FIFO handshake must not race the FIFO IRQ

		bool acknowledged = false;
		unsigned int attempt = 0;
		for (; attempt < maxAttempts && !acknowledged; ++attempt)
		{
			hw_set_bits(&psm_hw->frce_off, PSM_FRCE_OFF_PROC1_BITS);			// hard power-cycle core 1
			while ((psm_hw->frce_off & PSM_FRCE_OFF_PROC1_BITS) == 0) { tight_loop_contents(); }
			multicore_fifo_drain();												// clear stale core-1 -> core-0 words before release
			hw_clear_bits(&psm_hw->frce_off, PSM_FRCE_OFF_PROC1_BITS);			// release: core 1 runs its bootrom and pushes a readiness word

			const uint32_t start = millis();
			while (millis() - start < perAttemptMillis)
			{
				if (multicore_fifo_rvalid())
				{
					(void)sio_hw->fifo_rd;										// consume the readiness word
					acknowledged = true;
					break;
				}
			}
		}

		lastResetAttempts = attempt;
		irq_set_enabled(irqNum, irqWasEnabled);
		return acknowledged;
	}

	static volatile YieldPollFn yieldPoll = nullptr;		// polled continuously from Yield's wait loop (e.g. open-loop step generation)

	void SetYieldPoll(YieldPollFn fn) noexcept
	{
		yieldPoll = fn;
	}

	// Service housekeeping until the deadline. This is the only place core 1 parks, so a worker entry
	// (e.g. the TMC control loop) must call it between cycles. Bare metal: no scheduler on this core.
	TIME_CRITICAL void Yield(uint32_t untilStepTicks) noexcept
	{
		do
		{
			++heartbeat;
			{
				const YieldPollFn poll = yieldPoll;
				if (poll != nullptr)
				{
					poll();									// e.g. generate open-loop steps; not called while parked (see below)
				}
			}
			if (parkRequested)
			{
				parked = true;
				while (parkRequested)
				{
					__wfe();								// cheap wait; Resume() or any SEV wakes us to recheck
				}
				parked = false;
			}
			else if (cmdSeq != cmdAckSeq)
			{
				switch (cmdOp)
				{
				case Command::ping:
					cmdResult = cmdArg0 + 1;
					break;
				default:
					cmdResult = 0;
					break;
				}
				__dmb();									// result must be visible before the acknowledgement
				cmdAckSeq = cmdSeq;
				__sev();
			}
		} while ((int32_t)(untilStepTicks - StepTimer::GetTimerTicks()) > 0);
	}

	// The default core-1 entry: nothing to do but housekeeping
	[[noreturn]] TIME_CRITICAL static void MainLoop() noexcept
	{
		for (;;)
		{
			Yield(StepTimer::GetTimerTicks() + 1000);
		}
	}

	extern "C" [[noreturn]] TIME_CRITICAL void Core1RuntimeEntry() noexcept
	{
		(core1Entry != nullptr ? core1Entry : MainLoop)();
		for (;;) { }										// entries never return; keep the compiler happy
	}

	void Init() noexcept
	{
		if (crossCoreLock == nullptr)
		{
			crossCoreLock = spin_lock_instance((uint)spin_lock_claim_unused(true));
			segmentPoolLock = spin_lock_instance((uint)spin_lock_claim_unused(true));
		}
	}

	void Start(Core1EntryFn entry) noexcept
	{
		if (!started)
		{
			core1Entry = entry;
			Init();

			// Bound the watchdog-reboot retry loop (see the launchInProgress comment above). The
			// scratch counter survives warm resets; it is zeroed on every path that leaves Start().
			priorLaunchAttempts = watchdog_hw->scratch[LaunchAttemptScratchIndex];
			if (priorLaunchAttempts >= MaxLaunchAttempts)
			{
				watchdog_hw->scratch[LaunchAttemptScratchIndex] = 0;
				launchFailed = true;								// boot without core 1; board stays responsive and re-flashable
				return;
			}
			watchdog_hw->scratch[LaunchAttemptScratchIndex] = priorLaunchAttempts + 1;
			watchdog_hw->scratch[LaunchProgressScratchIndex] = LaunchProgressEntered;
			launchInProgress = true;								// a hang from here on watchdog-reboots in ~1s instead of wedging

			// Bounded, retried hard reset rather than the SDK's multicore_reset_core1(), which can hang
			// forever on a warm (non-cold) boot. If core 1 will not come up, boot without it instead of
			// locking up: the 'D' report shows launchFailed and the board stays alive and re-flashable.
			if (!RobustResetCore1(50, 5))
			{
				watchdog_hw->scratch[LaunchProgressScratchIndex] = LaunchProgressResetFailed;
				watchdog_hw->scratch[LaunchAttemptScratchIndex] = 0;
				launchInProgress = false;
				launchFailed = true;
				return;
			}
			watchdog_hw->scratch[LaunchProgressScratchIndex] = LaunchProgressResetOk;
			multicore_fifo_drain();									// clear any stale inter-core FIFO data before the launch handshake
			delay(100);												// match the proven CAN-core-1 launch timing
			watchdog_hw->scratch[LaunchProgressScratchIndex] = LaunchProgressLaunching;
#if MNB_USB_DIAG
			if (watchdog_hw->scratch[LaunchHangTestScratchIndex] == LaunchHangTestMagic)
			{
				// Bench self-test of the watchdog escape: fake the observed hang at its real location,
				// once (the magic is consumed). The gated kick must let the watchdog reboot us in ~1s,
				// and the retry must then show up as a nonzero retry count in the 'D' report.
				watchdog_hw->scratch[LaunchHangTestScratchIndex] = 0;
				for (;;) { }
			}
#endif
			multicore_launch_core1(Core1RuntimeEntry);
			watchdog_hw->scratch[LaunchProgressScratchIndex] = LaunchProgressDone;
			launchInProgress = false;
			watchdog_hw->scratch[LaunchAttemptScratchIndex] = 0;
			WatchdogReset();										// top the watchdog back up after the un-kicked launch window
			launchFailed = false;
			started = true;
		}
	}

	void Start() noexcept
	{
		Start(nullptr);
	}

	void HaltForReset() noexcept
	{
		if (started)
		{
			multicore_reset_core1();								// put core 1 back in the bootrom, halted
			started = false;
		}
	}

	bool IsStarted() noexcept { return started; }

	bool Park() noexcept
	{
		if (get_core_num() != 0)
		{
			// Called from core 1 itself. We cannot park ourselves, and any FreeRTOS call from here (the
			// flash-write path uses delay()) would assert. Reaching here means core 1 tried to initiate a
			// flash/NVM write, which it must not; return without blocking so we do not crash.
			return false;
		}
		if (parkDepth.fetch_add(1) == 0)
		{
			if (!started)
			{
				return true;									// nothing running on core 1, so it is trivially parked
			}
			parkRequested = true;
			__sev();
		}
		else if (!started)
		{
			return true;
		}
		// Wait for the acknowledgement even when another parker requested the park first: it may still
		// be acquiring, and returning early would let this caller mutate core-1-shared state (encoder,
		// flash/XIP, LUT) while core 1 is still running. That race was seen on the bench as concurrent
		// M122 handling during a calibration store leaving the encoder broken and the store incomplete.
		const uint32_t startTime = millis();
		while (!parked)
		{
			if (millis() - startTime >= 50)
			{
				parkTimeouts = parkTimeouts + 1;
				return false;									// proceed anyway; no worse than not having waited
			}
			delay(1);
		}
		parkCount = parkCount + 1;
		return true;
	}

	void Resume() noexcept
	{
		if (get_core_num() != 0)
		{
			return;
		}
		if (parkDepth.fetch_sub(1) == 1)
		{
			parkRequested = false;
			__sev();
		}
	}

	bool SendCommand(Command cmd, uint32_t arg0, uint32_t& result, uint32_t timeoutMillis) noexcept
	{
		if (!started || parked)
		{
			return false;
		}
		cmdOp = cmd;
		cmdArg0 = arg0;
		__dmb();												// operands must be visible before the submission
		const uint32_t seq = cmdSeq + 1;
		cmdSeq = seq;
		__sev();
		const uint32_t startTime = millis();
		while (cmdAckSeq != seq)
		{
			if (millis() - startTime >= timeoutMillis)
			{
				return false;
			}
			delay(1);
		}
		result = cmdResult;
		return true;
	}

	uint32_t GetHeartbeat() noexcept { return heartbeat; }
	bool IsParked() noexcept { return parked; }

	void GetParkDiagnostics(int& depth, bool& requested, bool& isParked, uint32_t& acquisitions, uint32_t& timeouts) noexcept
	{
		depth = parkDepth.load();
		requested = parkRequested;
		isParked = parked;
		acquisitions = parkCount;
		timeouts = parkTimeouts;
	}
	bool LaunchFailed() noexcept { return launchFailed; }
	uint32_t GetResetAttempts() noexcept { return lastResetAttempts; }
	bool LaunchInProgress() noexcept { return launchInProgress; }
	uint32_t GetLaunchRetries() noexcept { return priorLaunchAttempts; }
	uint32_t GetLaunchProgress() noexcept { return watchdog_hw->scratch[LaunchProgressScratchIndex]; }

	// RAM-resident: called from core 1 inside the motion critical section every control cycle, so a
	// flash fetch here would defeat the point of running the loop on core 1
	TIME_CRITICAL spin_lock_t *GetCrossCoreLock() noexcept { return crossCoreLock; }
	TIME_CRITICAL spin_lock_t *GetSegmentPoolLock() noexcept { return segmentPoolLock; }
}

// Strong overrides of the weak CoreN2G hooks: flash operations park our core-1 runtime through the
// same DisableCore1Processing()/EnableCore1Processing() path that all existing callers already use.
void ApplicationPauseCore1() noexcept
{
	(void)Core1Runtime::Park();
}

void ApplicationResumeCore1() noexcept
{
	Core1Runtime::Resume();
}

#endif	// RPXXXX && SPICAN_CORE0_SERVICE

// End
