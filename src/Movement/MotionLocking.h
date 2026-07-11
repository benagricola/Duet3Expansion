/*
 * MotionLocking.h
 *
 * Locking primitives for the motion segment structures and the MoveSegment freelist.
 *
 * In the normal build, mutual exclusion between the writers (the Move task and the step interrupt,
 * and the closed-loop control task) only has to hold within one core, so masking interrupts is
 * sufficient and cheap. When the TMC control loop runs on core 1 (TMC_ON_CORE1), it advances and
 * frees segments concurrently with core 0, so these sections must take a hardware spinlock instead.
 *
 * Two separate locks are used because the control loop releases segments to the freelist while it
 * holds the motion lock: motion -> pool nesting is allowed, the inverse never occurs.
 */

#ifndef SRC_MOVEMENT_MOTIONLOCKING_H_
#define SRC_MOVEMENT_MOTIONLOCKING_H_

#include <RepRapFirmware.h>

#if TMC_ON_CORE1

# include <Platform/Core1Runtime.h>

static inline uint32_t MotionLockSave() noexcept { return spin_lock_blocking(Core1Runtime::GetCrossCoreLock()); }
static inline void MotionLockRestore(uint32_t flags) noexcept { spin_unlock(Core1Runtime::GetCrossCoreLock(), flags); }
static inline uint32_t SegmentPoolLockSave() noexcept { return spin_lock_blocking(Core1Runtime::GetSegmentPoolLock()); }
static inline void SegmentPoolLockRestore(uint32_t flags) noexcept { spin_unlock(Core1Runtime::GetSegmentPoolLock(), flags); }

using MotionCriticalSectionLocker = CrossCoreCriticalSectionLocker;

#else

static inline uint32_t MotionLockSave() noexcept { return IrqSave(); }
static inline void MotionLockRestore(uint32_t flags) noexcept { IrqRestore(flags); }
static inline uint32_t SegmentPoolLockSave() noexcept { return IrqSave(); }
static inline void SegmentPoolLockRestore(uint32_t flags) noexcept { IrqRestore(flags); }

using MotionCriticalSectionLocker = AtomicCriticalSectionLocker;

#endif

#endif /* SRC_MOVEMENT_MOTIONLOCKING_H_ */
