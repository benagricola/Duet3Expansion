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
#include <hardware/structs/timer.h>
#include <Platform/Tasks.h>

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
	static bool started = false;

	// The core-1 main loop. Bare metal: no scheduler, no interrupts expected on this core.
	// Phase C replaces the body of the active branch with the TMC control cycle.
	[[noreturn]] TIME_CRITICAL static void MainLoop() noexcept
	{
		for (;;)
		{
			++heartbeat;
			if (parkRequested)
			{
				parked = true;
				while (parkRequested)
				{
					__wfe();									// cheap wait; Resume() follows a spin_unlock or SEV soon enough
				}
				parked = false;
			}
			else
			{
				if (cmdSeq != cmdAckSeq)
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
			}
		}
	}

	extern "C" [[noreturn]] TIME_CRITICAL void Core1RuntimeEntry() noexcept
	{
		MainLoop();
	}

	void Init() noexcept
	{
		if (crossCoreLock == nullptr)
		{
			crossCoreLock = spin_lock_instance((uint)spin_lock_claim_unused(true));
		}
	}

	void Start() noexcept
	{
		if (!started)
		{
			Init();
			multicore_reset_core1();
			delay(2);
			multicore_launch_core1(Core1RuntimeEntry);
			started = true;
		}
	}

	bool IsStarted() noexcept { return started; }

	bool Park() noexcept
	{
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
		parkRequested = false;
		__sev();
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

	spin_lock_t *GetCrossCoreLock() noexcept { return crossCoreLock; }
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
