/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_KERNEL_INCLUDE_TIMEOUT_Q_H_
#define ZEPHYR_KERNEL_INCLUDE_TIMEOUT_Q_H_

/**
 * @file
 * @brief timeout queue for threads on kernel objects
 */

#include <zephyr/kernel.h>

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_SYS_CLOCK_EXISTS

#ifdef CONFIG_TIMEOUT_USE_MIN_HEAP

/*
 * heap_idx sentinels : heap_idx is 1-indexed so that zero maps to IDLE.
 * This will make heap_idx match the behaviour of dlist implementation.
 *
 * IDLE: heap_idx == 0 - Never inserted or already fired.
 * Active: heap_idx in [1, heap_size] - actual slot is (heap_idx - 1).
 * ABORTED: heap_idx == UINT16_MAX - cancelled while pending.
 *
 */
#define TIMEOUT_HEAP_IDX_IDLE    ((uint16_t)0U)
#define TIMEOUT_HEAP_IDX_ABORTED ((uint16_t)UINT16_MAX)

static inline void z_init_timeout(struct _timeout *to)
{
	to->heap_idx = TIMEOUT_HEAP_IDX_IDLE;
}

/**
 * Adds the timeout to the queue.
 *
 * @return Absolute tick value when timeout will expire.
 */
k_ticks_t z_add_timeout(struct _timeout *to, _timeout_func_t fn, k_timeout_t timeout);

int z_abort_timeout(struct _timeout *to);

/**
 * Checks if a timeout node is active
 *
 * @return Return true if inactive (idle or aborted)
 */
static inline bool z_is_inactive_timeout(const struct _timeout *to)
{
	return to->heap_idx == TIMEOUT_HEAP_IDX_IDLE ||
		to->heap_idx == TIMEOUT_HEAP_IDX_ABORTED;
}

/**
 * Checks if the timeout was aborted.
 *
 * @return Return true if aborted.
 */
static inline bool z_is_aborted_timeout(const struct _timeout *to)
{
	return to->heap_idx == TIMEOUT_HEAP_IDX_ABORTED;
}

#else /* !CONFIG_TIMEOUT_USE_MIN_HEAP : dlist backend */

/* Value written to dticks when timeout is aborted. */
#define TIMEOUT_DTICKS_ABORTED (IS_ENABLED(CONFIG_TIMEOUT_64BIT) ? INT64_MIN : INT32_MIN)

static inline void z_init_timeout(struct _timeout *to)
{
	sys_dnode_init(&to->node);
}

/* Adds the timeout to the queue.
 *
 * @return Absolute tick value when timeout will expire.
 */
k_ticks_t z_add_timeout(struct _timeout *to, _timeout_func_t fn, k_timeout_t timeout);

int z_abort_timeout(struct _timeout *to);

static inline bool z_is_inactive_timeout(const struct _timeout *to)
{
	return !sys_dnode_is_linked(&to->node);
}

static inline bool z_is_aborted_timeout(const struct _timeout *to)
{
	/* When timeout is aborted then dticks is set to special value. */
	return to->dticks == TIMEOUT_DTICKS_ABORTED;
}

#endif /* CONFIG_TIMEOUT_USE_MIN_HEAP */

static inline void z_init_thread_timeout(struct _thread_base *thread_base)
{
	z_init_timeout(&thread_base->timeout);
}

extern void z_thread_timeout(struct _timeout *timeout);

static inline k_ticks_t z_add_thread_timeout(struct k_thread *thread, k_timeout_t ticks)
{
	return z_add_timeout(&thread->base.timeout, z_thread_timeout, ticks);
}

static inline void z_abort_thread_timeout(struct k_thread *thread)
{
	z_abort_timeout(&thread->base.timeout);
}

static inline bool z_is_aborted_thread_timeout(struct k_thread *thread)
{

	return z_is_aborted_timeout(&thread->base.timeout);
}

int32_t z_get_next_timeout_expiry(void);

k_ticks_t z_timeout_remaining(const struct _timeout *timeout);

#else

/* Stubs when !CONFIG_SYS_CLOCK_EXISTS */
#define z_init_thread_timeout(thread_base) do {} while (false)
#define z_abort_thread_timeout(to) do {} while (false)
#define z_is_aborted_thread_timeout(to) false
#define z_is_inactive_timeout(to) 1
#define z_is_aborted_timeout(to) false
#define z_get_next_timeout_expiry() ((int32_t) K_TICKS_FOREVER)
#define z_set_timeout_expiry(ticks, is_idle) do {} while (false)

static inline k_ticks_t z_add_thread_timeout(struct k_thread *thread, k_timeout_t ticks)
{
	ARG_UNUSED(thread);
	ARG_UNUSED(ticks);
	return 0;
}

#endif /* CONFIG_SYS_CLOCK_EXISTS */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_KERNEL_INCLUDE_TIMEOUT_Q_H_ */
