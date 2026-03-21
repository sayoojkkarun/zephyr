/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Kernel timeout subsystem - min-heap backend
 */

#ifndef CONFIG_TIMEOUT_USE_MIN_HEAP
#error "timeout_heap.c requires CONFIG_TIMEOUT_USE_MIN_HEAP=y"
#endif

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/min_heap.h>
#include <ksched.h>
#include <timeout_q.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/llext/symbol.h>

#define TIMEOUT_HEAP_MAX  CONFIG_TIMEOUT_HEAP_MAX_ENTRIES

static struct _timeout *timeout_storage[TIMEOUT_HEAP_MAX];
static struct min_heap timeout_heap_inst;

static struct k_spinlock timeout_lock;

/* Absolute tick counter: number of ticks since boot */
static uint64_t curr_tick;

/* Ticks left to process in the currently executing sys_clock_announce(). */
static int announce_remaining;

static uint32_t timeout_seq;

static int timeout_cmp(const void *a, const void *b)
{
	const struct _timeout *ta = *(const struct _timeout *const *)a;
	const struct _timeout *tb = *(const struct _timeout *const *)b;

	if (ta->abs_ticks < tb->abs_ticks) {
		return -1;
	}

	if (ta->abs_ticks > tb->abs_ticks) {
		return 1;
	}

	/* For equal deadlines preserve the insertion order */
	return (ta->seq < tb->seq) ? -1 : (ta->seq > tb->seq) ? 1 : 0;
}

/*
 * timeout_on_move: position update callback
 *
 * Callback gets invoked when pointer-slot moves to a new index. heap_idx
 * in struct _timeout needs to be updatedso that z_abort_timeout() can locate
 * the element in O(1) without linear scan.
 */
static void timeout_on_move(void *elem, size_t index, void *user_data)
{
	ARG_UNUSED(user_data);

	struct _timeout *to = *(struct _timeout **)elem;

	to->heap_idx = (uint16_t)(index + 1U); /* 1-indexed */
}

/**
 * Helper functions
 */

static int32_t elapsed(void)
{
	return announce_remaining == 0 ? sys_clock_elapsed() : 0U;
}

static int32_t heap_next_ticks(int32_t ticks_elapsed)
{
	if (min_heap_is_empty(&timeout_heap_inst)) {
		return SYS_CLOCK_MAX_WAIT;
	}

	const struct _timeout *t =
		*(const struct _timeout **)min_heap_peek(&timeout_heap_inst);
	int64_t remaining = t->abs_ticks - (int64_t)curr_tick - ticks_elapsed;

	if (remaining > (int64_t)INT32_MAX) {
		return SYS_CLOCK_MAX_WAIT;
	}

	return (int32_t)MAX(0, remaining);
}

static int timeout_heap_init(void)
{
	min_heap_init_ex(&timeout_heap_inst,
		timeout_storage,
		TIMEOUT_HEAP_MAX,
		sizeof(struct _timeout *),
		timeout_cmp,
		timeout_on_move,
		NULL);

		return 0;
}

SYS_INIT(timeout_heap_init, PRE_KERNEL_1, 0);

/**
 * Public kernel timeout API
 */

k_ticks_t z_add_timeout(struct _timeout *to, _timeout_func_t fn,
			k_timeout_t timeout)
{
	k_ticks_t ret;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		return 0;
	}

#ifdef CONFIG_KERNEL_COHERENCE
	__ASSERT_NO_MSG(sys_cache_is_mem_coherent(to));
#endif

	__ASSERT_NO_MSG(z_is_inactive_timeout(to));

	to->fn = fn;

	K_SPINLOCK(&timeout_lock) {
		int32_t ticks_elapsed;
		bool reprogram = false;

		to->seq = timeout_seq++;

		if (Z_IS_TIMEOUT_RELATIVE(timeout)) {
			ticks_elapsed = elapsed();
			to->abs_ticks = (int64_t)curr_tick + ticks_elapsed
				+ timeout.ticks + 1;
			ret = to->abs_ticks;
		} else {
			int64_t abs = Z_TICK_ABS(timeout.ticks);

			to->abs_ticks = MAX(abs, (int64_t)curr_tick + 1);
			ret = to->abs_ticks;
			ticks_elapsed = elapsed();
		}

		int rc = min_heap_push(&timeout_heap_inst, &to);

		ARG_UNUSED(rc);
		__ASSERT_NO_MSG(rc == 0);

		const struct _timeout *root =
			*(const struct _timeout **)min_heap_peek(&timeout_heap_inst);

		if (root == to && announce_remaining == 0) {
			reprogram = true;
		}

		if (reprogram) {
			sys_clock_set_timeout(heap_next_ticks(ticks_elapsed), false);
		}
	}

	return ret;
}

int z_abort_timeout(struct _timeout *to)
{
	int ret = -EINVAL;

	K_SPINLOCK(&timeout_lock) {
		if (!z_is_inactive_timeout(to)) {
			/* convert head_idx to slot */
			size_t id = (size_t)(to->heap_idx - 1U);
			bool was_min = (id == 0);
			struct _timeout *popped;

			min_heap_remove(&timeout_heap_inst, id, &popped);
			to->heap_idx = TIMEOUT_HEAP_IDX_ABORTED;
			ret = 0;

			if (was_min && announce_remaining == 0) {
				sys_clock_set_timeout(heap_next_ticks(elapsed()), false);
			}
		}
	}

	return ret;
}

k_ticks_t z_timeout_remaining(const struct _timeout *timeout)
{
	k_ticks_t ticks = 0;

	K_SPINLOCK(&timeout_lock) {
		if (!z_is_inactive_timeout(timeout)) {
			int64_t rem = timeout->abs_ticks
					- (int64_t)curr_tick - elapsed();

			ticks = (k_ticks_t)MAX(0, rem);
		}
	}

	return ticks;
}
EXPORT_SYMBOL(_timeout_remaining);

k_ticks_t z_timeout_expires(const struct _timeout *timeout)
{
	k_ticks_t ticks = 0;

	K_SPINLOCK(&timeout_lock) {
		ticks = z_is_inactive_timeout(timeout)
			? (k_ticks_t)curr_tick
			: (k_ticks_t)timeout->abs_ticks;
	}

	return ticks;
}
EXPORT_SYMBOL(z_timeout_expires);

int32_t z_get_next_timeout_expiry(void)
{
	int32_t ret = (int32_t)K_TICKS_FOREVER;

	K_SPINLOCK(&timeout_lock) {
		ret = heap_next_ticks(elapsed());
	}

	return ret;
}

void sys_clock_announce(int32_t ticks)
{
	k_spinlock_key_t key = k_spin_lock(&timeout_lock);

	if (IS_ENABLED(CONFIG_SMP) && announce_remaining != 0) {
		announce_remaining += ticks;
		k_spin_unlock(&timeout_lock, key);

		return;
	}

	announce_remaining = ticks;

	while (!min_heap_is_empty(&timeout_heap_inst)) {
		struct _timeout *t =
			*(struct _timeout **)
			min_heap_peek(&timeout_heap_inst);

		if (t->abs_ticks > (int64_t)(curr_tick + announce_remaining)) {
			break;
		}

		int32_t dt = (int32_t)(t->abs_ticks - curr_tick);

		curr_tick = t->abs_ticks;
		announce_remaining -= dt;

		/**
		 * Pop before releasing the lock. on_move is NOT called for
		 * the popped element (min_heap_pop copies it to an
		 * out-buffer but does not notify the removed element's final
		 * position - which is "no longer in the heap"). We therefore
		 * set heap_idx to IDLE explicitly before firing the callback.
		 */
		struct _timeout *popped;

		min_heap_pop(&timeout_heap_inst, &popped);
		t->heap_idx = TIMEOUT_HEAP_IDX_IDLE;

		key = k_spin_unlock(&timeout_lock, key);
		t->fn(t);
		key = k_spin_lock(&timeout_lock);
	}

	curr_tick += announce_remaining;
	announce_remaining = 0;

	sys_clock_set_timeout(heap_next_ticks(0), false);

	k_spin_unlock(&timeout_lock, key);

#ifdef CONFIG_TIMESLICING
	z_time_slice();
#endif
}

/* Tick and timepoint helper functions */

int64_t sys_clock_tick_get(void)
{
	uint64_t t = 0U;

	K_SPINLOCK(&timeout_lock) {
		t = curr_tick + elapsed();
	}

	return t;
}

uint32_t sys_clock_tick_get_32(void)
{
#ifdef CONFIG_TICKLESS_KERNEL
	return (uint32_t)sys_clock_tick_get();
#else
	return (uint32_t)curr_tick;
#endif
}

int64_t z_imp_k_uptime_ticks(void)
{
	return sys_clock_tick_get();
}

#ifdef CONFIG_USERSPACE
static inline int64_t z_vrfy_k_uptime_ticks(void)
{
	return z_impl_k_uptime_ticks();
}
#include <zephyr/syscalls/k_uptime_ticks_mrsh.c>
#endif

k_timepoint_t sys_timepoint_calc(k_timeout_t timeout)
{
	k_timepoint_t timepoint;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		timepoint.tick = UINT64_MAX;
	} else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		timeout.tick = 0;
	} else {
		k_ticks_t dt = timeout.ticks;

		if (Z_IS_TIMEOUT_RELATIVE(timeout)) {
			timepoint.tick = sys_clock_tick_get() + MAX(1, dt);
		} else {
			timepoint.tick = Z_TICK_ABS(dt);
		}
	}

	return timepoint;
}

k_timeout_t sys_timepoint_timeout(k_timepoint_t timepoint)
{
	uint64_t now, remaining;

	if (timepoint.tick == UINT64_MAX) {
		return K_FOREVER;
	}

	if (timepoint.tick == 0) {
		return K_NO_WAIT;
	}

	now = sys_clock_tick_get();
	remaining = (timepoint.tick > now) ? (timepoint.tick - now) : 0;

	return K_TICKS(remaining);
}

#ifdef CONFIG_ZTEST
void z_impl_sys_clock_tick_set(uint64_t tick)
{
	curr_tick = tick;
}

void z_vrfy_sys_clock_tick_set(uint64_t tick)
{
	z_impl_sys_clock_tick_set(tick);
}
#endif /* CONFIG_ZTEST */
