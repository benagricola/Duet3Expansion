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

	// Service housekeeping until the deadline. This is the only place core 1 parks, so a worker entry
	// (e.g. the TMC control loop) must call it between cycles. Bare metal: no scheduler on this core.
	TIME_CRITICAL void Yield(uint32_t untilStepTicks) noexcept
	{
		do
		{
			++heartbeat;
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
			multicore_reset_core1();
			delay(100);												// match the proven CAN-core-1 launch timing; a short delay can leave core 1 not fully reset after a software reboot (M997), hanging the relaunch
			multicore_launch_core1(Core1RuntimeEntry);
			started = true;
		}
	}

	void Start() noexcept
	{
		Start(nullptr);
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
		if (parkDepth.fetch_add(1) != 0)
		{
			return parked || !started;							// someone else already requested the park
		}
		if (!started)
		{
			return true;										// nothing running on core 1, so it is trivially parked
		}
		parkRequested = true;
		__sev();
		const uint32_t startTime = millis();
		while (!parked)
		{
			if (millis() - startTime >= 50)
			{
				return false;									// proceed anyway; no worse than not having waited
			}
			delay(1);
		}
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
