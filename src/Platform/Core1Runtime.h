/*
 * Core1Runtime.h
 *
 * Application-owned runtime for the RP2350's second core. Available only when the CAN chip is
 * serviced from core 0 (SPICAN_CORE0_SERVICE), which leaves core 1 free. Provides:
 *  - a bare-metal, RAM-resident core-1 main loop (no FreeRTOS on core 1)
 *  - a park/resume handshake, wired into the DisableCore1Processing()/EnableCore1Processing()
 *    flash-operation path via the weak ApplicationPauseCore1()/ApplicationResumeCore1() hooks
 *  - a simple command mailbox for core-0 -> core-1 requests
 *  - a cross-core critical section backed by an RP2350 hardware spinlock, for data structures
 *    that both cores mutate (interrupt masking alone only protects within one core)
 */

#ifndef SRC_PLATFORM_CORE1RUNTIME_H_
#define SRC_PLATFORM_CORE1RUNTIME_H_

#include <RepRapFirmware.h>

#if RPXXXX && SPICAN_CORE0_SERVICE

#include <hardware/sync.h>

namespace Core1Runtime
{
	// Commands that core 0 can send to the core-1 loop. Phase C adds the real ones.
	enum class Command : uint32_t
	{
		none = 0,
		ping,										// reply with arg0 + 1 in the result, to prove the mailbox works
	};

	typedef void (*Core1EntryFn)() noexcept;

	void Init() noexcept;							// claim the spinlocks etc.; call before Start and before any cross-core locker use
	void Start() noexcept;							// launch the default core-1 main loop (idle: heartbeat/park/mailbox only)
	void Start(Core1EntryFn entry) noexcept;		// launch a specific core-1 entry (e.g. the TMC control loop); it must call Yield() regularly
	void HaltForReset() noexcept;					// reset core 1 into the bootrom and mark it not started (call before a system reset)
	bool IsStarted() noexcept;

	// Service core-1 housekeeping (heartbeat, park requests, mailbox) until the given step-timer
	// deadline. Called only from core-1 code, between control cycles.
	void Yield(uint32_t untilStepTicks) noexcept;

	// Register a function to be polled continuously from Yield's wait loop (RAM-resident, core 1,
	// microsecond-scale granularity; not called while parked). Used by the motor kernel to generate
	// open-loop steps. Pass nullptr to remove.
	typedef void (*YieldPollFn)() noexcept;
	void SetYieldPoll(YieldPollFn fn) noexcept;

	bool Park() noexcept;							// ask core 1 to park in its idle loop; returns true when it acknowledged (bounded wait). Nestable.
	void Resume() noexcept;

	// Send a command and wait (bounded) for completion. Returns false on timeout.
	bool SendCommand(Command cmd, uint32_t arg0, uint32_t& result, uint32_t timeoutMillis) noexcept;

	uint32_t GetHeartbeat() noexcept;				// increments continuously while the core-1 loop is alive
	bool IsParked() noexcept;
	bool LaunchFailed() noexcept;					// true if Start() could not bring core 1 up and booted without it
	uint32_t GetResetAttempts() noexcept;			// force-off/on attempts the last core-1 reset needed (diagnostics)
	bool LaunchInProgress() noexcept;				// true inside Start()'s reset/launch window; vApplicationTickHook suspends the watchdog kick while set
	uint32_t GetLaunchRetries() noexcept;			// watchdog-reboot launch retries consumed before this boot's Start() (0 = first try worked)
	uint32_t GetLaunchProgress() noexcept;			// progress code of the most recent launch attempt (see Core1Runtime.cpp)

	spin_lock_t *GetCrossCoreLock() noexcept;		// guards the motion segment structures; initialised by Init()
	spin_lock_t *GetSegmentPoolLock() noexcept;		// guards the MoveSegment freelist (separate lock: the motion lock is held while segments are released)
}

// Scoped park of the core-1 loop, for core-0 sequences that must not run concurrently with it
// (encoder reconfiguration, closed-loop mode switches, encoder diagnostics). Nestable.
class Core1ParkLocker
{
public:
	Core1ParkLocker() noexcept { (void)Core1Runtime::Park(); }
	~Core1ParkLocker() { Core1Runtime::Resume(); }
	Core1ParkLocker(const Core1ParkLocker&) = delete;
	Core1ParkLocker& operator=(const Core1ParkLocker&) = delete;
};

// Scoped cross-core critical section: takes the hardware spinlock with interrupts disabled on the
// calling core. Both cores must use this (not AtomicCriticalSectionLocker) around any structure
// they both mutate.
class CrossCoreCriticalSectionLocker
{
public:
	CrossCoreCriticalSectionLocker() noexcept
		: savedIrqState(spin_lock_blocking(Core1Runtime::GetCrossCoreLock()))
	{
	}

	~CrossCoreCriticalSectionLocker()
	{
		spin_unlock(Core1Runtime::GetCrossCoreLock(), savedIrqState);
	}

	CrossCoreCriticalSectionLocker(const CrossCoreCriticalSectionLocker&) = delete;
	CrossCoreCriticalSectionLocker& operator=(const CrossCoreCriticalSectionLocker&) = delete;

private:
	uint32_t savedIrqState;
};

#endif	// RPXXXX && SPICAN_CORE0_SERVICE

#endif /* SRC_PLATFORM_CORE1RUNTIME_H_ */
