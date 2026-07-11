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

	void Init() noexcept;							// claim the spinlock etc.; call before Start and before any CrossCoreCriticalSection use
	void Start() noexcept;							// launch the core-1 main loop
	bool IsStarted() noexcept;

	bool Park() noexcept;							// ask core 1 to park in its idle loop; returns true when it acknowledged (bounded wait)
	void Resume() noexcept;

	// Send a command and wait (bounded) for completion. Returns false on timeout.
	bool SendCommand(Command cmd, uint32_t arg0, uint32_t& result, uint32_t timeoutMillis) noexcept;

	uint32_t GetHeartbeat() noexcept;				// increments continuously while the core-1 loop is alive
	bool IsParked() noexcept;

	spin_lock_t *GetCrossCoreLock() noexcept;		// initialised by Init()
}

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
