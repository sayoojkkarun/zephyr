/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System Health Monitor implementation
 */

#include <zephyr/health/health.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>

#ifdef CONFIG_STATS
#include <zephyr/stats/stats.h>
#include <zephyr/kernel/stats.h>
#endif

#ifdef CONFIG_THREAD_ANALYZER
#include <zephyr/debug/thread_analyzer.h>
#endif

#ifdef CONFIG_DEBUG_CPU_LOAD
#include <zephyr/debug/cpu_load.h>
#endif

LOG_MODULE_REGISTER(health, CONFIG_HEALTH_MONITOR_LOG_LEVEL);

/* Maximum number of built-in metrics */
#define HEALTH_BUILTIN_METRIC_COUNT 7

/* Built-in metric array indices */
enum health_builtin_metric_idx {
	HEALTH_IDX_CPU_USAGE = 0,
	HEALTH_IDX_MEMORY_USAGE = 1,
	HEALTH_IDX_STACK_USAGE = 2,
	HEALTH_IDX_THREAD_COUNT = 3,
	HEALTH_IDX_THREADS_HIGH_STACK = 4,
	HEALTH_IDX_FREE_HEAP = 5,
	HEALTH_IDX_TOTAL_HEAP = 6,
};

/* Custom metric entry structure */
struct health_custom_metric {
	const char *name;
	health_metric_collect_fn collect_fn;
	void *user_data;
	uint32_t metric_id;
	bool active;
};

/* Threshold callback entry */
struct health_callback {
	enum health_metric_type type;
	enum health_status threshold_level;
	health_threshold_cb cb;
	void *user_data;
};

/* History buffer entry */
struct health_history {
	uint32_t *values;
	size_t size;
	size_t count;
	size_t head;
};

/* Built-in metrics storage */
static struct health_metric builtin_metrics[HEALTH_BUILTIN_METRIC_COUNT];

/* Custom metrics storage */
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
static struct health_custom_metric
	custom_metrics[CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS];
static uint32_t next_custom_id = HEALTH_METRIC_CUSTOM_START;
#endif

/* Callbacks storage */
#define MAX_CALLBACKS 16
static struct health_callback callbacks[MAX_CALLBACKS];
static size_t callback_count;

/* History buffers */
#ifdef CONFIG_HEALTH_MONITOR_HISTORY
static struct health_history history_buffers[HEALTH_BUILTIN_METRIC_COUNT +
					     CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS];
#endif

/* Timer for periodic collection */
static struct k_timer collection_timer;

/* Thread safety */
static struct k_spinlock health_lock;

/* Initialization flag */
static bool health_initialized;

/* Stack analysis data */
#ifdef CONFIG_HEALTH_MONITOR_STACK
struct stack_analysis_data {
	uint32_t total_stack_used;
	uint32_t total_stack_size;
	uint32_t thread_count;
	uint32_t high_stack_count;
};
#endif

/* Forward declarations */
static void health_collect_metrics(struct k_timer *timer);
#ifdef CONFIG_HEALTH_MONITOR_STACK
static void stack_analyzer_cb(struct thread_analyzer_info *info);
#endif

#ifdef CONFIG_HEALTH_MONITOR_STACK
static struct stack_analysis_data stack_data;
#endif

/**
 * @brief Initialize built-in metrics with default values
 */
static void health_init_builtin_metrics(void)
{
	memset(builtin_metrics, 0, sizeof(builtin_metrics));

	builtin_metrics[HEALTH_IDX_CPU_USAGE].type = HEALTH_METRIC_CPU_USAGE;
	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].type = HEALTH_METRIC_MEMORY_USAGE;
	builtin_metrics[HEALTH_IDX_STACK_USAGE].type = HEALTH_METRIC_STACK_USAGE;
	builtin_metrics[HEALTH_IDX_THREAD_COUNT].type = HEALTH_METRIC_THREAD_COUNT;
	builtin_metrics[HEALTH_IDX_THREADS_HIGH_STACK].type = HEALTH_METRIC_THREADS_HIGH_STACK;
	builtin_metrics[HEALTH_IDX_FREE_HEAP].type = HEALTH_METRIC_FREE_HEAP;
	builtin_metrics[HEALTH_IDX_TOTAL_HEAP].type = HEALTH_METRIC_TOTAL_HEAP;

#ifdef CONFIG_HEALTH_MONITOR_DEFAULT_THRESHOLDS
#ifdef CONFIG_HEALTH_MONITOR_CPU
	builtin_metrics[HEALTH_IDX_CPU_USAGE].threshold_warning =
		CONFIG_HEALTH_MONITOR_CPU_WARNING_THRESHOLD;
	builtin_metrics[HEALTH_IDX_CPU_USAGE].threshold_critical =
		CONFIG_HEALTH_MONITOR_CPU_CRITICAL_THRESHOLD;
#endif
#ifdef CONFIG_HEALTH_MONITOR_MEMORY
	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].threshold_warning =
		CONFIG_HEALTH_MONITOR_MEMORY_WARNING_THRESHOLD;
	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].threshold_critical =
		CONFIG_HEALTH_MONITOR_MEMORY_CRITICAL_THRESHOLD;
#endif
#ifdef CONFIG_HEALTH_MONITOR_STACK
	builtin_metrics[HEALTH_IDX_STACK_USAGE].threshold_warning =
		CONFIG_HEALTH_MONITOR_STACK_WARNING_THRESHOLD;
	builtin_metrics[HEALTH_IDX_STACK_USAGE].threshold_critical =
		CONFIG_HEALTH_MONITOR_STACK_CRITICAL_THRESHOLD;
#endif
#endif
}

/**
 * @brief Collect CPU usage metric
 */
#ifdef CONFIG_HEALTH_MONITOR_CPU
static void health_collect_cpu_usage(void)
{
	uint32_t cpu_usage = 0;

#ifdef CONFIG_DEBUG_CPU_LOAD
	int load = cpu_load_get(false);

	if (load >= 0) {
		/* cpu_load_get returns per mille (0-1000), convert to percent */
		cpu_usage = (uint32_t)(load / 10);
	}
#elif defined(CONFIG_STATS) && defined(CONFIG_SCHED_THREAD_USAGE)
	/* Use kernel stats if available */
	struct k_thread *idle_thread = z_get_idle_thread();
	struct k_cycle_stats *idle_stats = &idle_thread->base.usage;
	uint64_t total_cycles = k_cycle_get_64();
	uint64_t idle_cycles = idle_stats->total;

	if (total_cycles > 0 && idle_cycles < total_cycles) {
		uint64_t used_cycles = total_cycles - idle_cycles;
		cpu_usage = (uint32_t)((used_cycles * 100ULL) / total_cycles);
	}
#endif

	builtin_metrics[HEALTH_IDX_CPU_USAGE].value = cpu_usage;
	builtin_metrics[HEALTH_IDX_CPU_USAGE].timestamp = k_uptime_ticks();
}
#else
static void health_collect_cpu_usage(void)
{
	/* CPU monitoring not enabled */
}
#endif

/**
 * @brief Collect memory usage metric
 */
#ifdef CONFIG_HEALTH_MONITOR_MEMORY
static void health_collect_memory_usage(void)
{
	/* Simple approach: use kernel heap if available */
	/* For now, set to 0 if we can't get accurate stats */
	uint32_t memory_usage = 0;
	uint32_t free_heap = 0;
	uint32_t total_heap = 0;

	/* Try to get heap statistics */
	/* Note: This is a simplified implementation */
	/* In a full implementation, we would iterate over all heaps */
	extern struct k_heap _system_heap;

#ifdef CONFIG_KERNEL_MEM_POOLS
	struct k_mem_slab *slab;
	size_t num_blocks;
	size_t block_size;
	size_t used_blocks = 0;

	/* Count used blocks in system heap */
	/* This is a placeholder - actual implementation would need
	 * access to heap internals or use a different approach
	 */
	total_heap = CONFIG_HEAP_MEM_POOL_SIZE;
	free_heap = total_heap; /* Placeholder */
	memory_usage = total_heap > 0 ? ((total_heap - free_heap) * 100) / total_heap : 0;
#else
	/* Fallback: set to 0 if we can't determine */
	memory_usage = 0;
	free_heap = 0;
	total_heap = 0;
#endif

	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].value = memory_usage;
	builtin_metrics[HEALTH_IDX_FREE_HEAP].value = free_heap;
	builtin_metrics[HEALTH_IDX_TOTAL_HEAP].value = total_heap;
	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].timestamp = k_uptime_ticks();
	builtin_metrics[HEALTH_IDX_FREE_HEAP].timestamp = k_uptime_ticks();
	builtin_metrics[HEALTH_IDX_TOTAL_HEAP].timestamp = k_uptime_ticks();
}
#else
static void health_collect_memory_usage(void)
{
	/* Memory monitoring not enabled */
}
#endif

/**
 * @brief Thread analyzer callback for stack usage
 */
#ifdef CONFIG_HEALTH_MONITOR_STACK
static void stack_analyzer_cb(struct thread_analyzer_info *info)
{
	if (info == NULL) {
		return;
	}

	stack_data.thread_count++;
	stack_data.total_stack_size += info->stack_size;
	stack_data.total_stack_used += info->stack_used;

	/* Count threads with high stack usage (>80%) */
	if (info->stack_size > 0) {
		uint32_t usage_percent = (info->stack_used * 100) / info->stack_size;

		if (usage_percent > 80) {
			stack_data.high_stack_count++;
		}
	}
}

/**
 * @brief Collect stack usage metrics
 */
static void health_collect_stack_usage(void)
{
	memset(&stack_data, 0, sizeof(stack_data));

	thread_analyzer_run(stack_analyzer_cb, 0);

	if (stack_data.thread_count > 0) {
		uint32_t avg_stack_usage = (stack_data.total_stack_used * 100) /
					    stack_data.total_stack_size;

		builtin_metrics[HEALTH_IDX_STACK_USAGE].value = avg_stack_usage;
		builtin_metrics[HEALTH_IDX_THREAD_COUNT].value = stack_data.thread_count;
		builtin_metrics[HEALTH_IDX_THREADS_HIGH_STACK].value = stack_data.high_stack_count;
	} else {
		builtin_metrics[HEALTH_IDX_STACK_USAGE].value = 0;
		builtin_metrics[HEALTH_IDX_THREAD_COUNT].value = 0;
		builtin_metrics[HEALTH_IDX_THREADS_HIGH_STACK].value = 0;
	}

	builtin_metrics[HEALTH_IDX_STACK_USAGE].timestamp = k_uptime_ticks();
	builtin_metrics[HEALTH_IDX_THREAD_COUNT].timestamp = k_uptime_ticks();
	builtin_metrics[HEALTH_IDX_THREADS_HIGH_STACK].timestamp = k_uptime_ticks();
}
#else
static void health_collect_stack_usage(void)
{
	/* Stack monitoring not enabled */
}
#endif

/**
 * @brief Collect thread count (if stack monitoring not available)
 */
#ifdef CONFIG_HEALTH_MONITOR_THREADS
#ifndef CONFIG_HEALTH_MONITOR_STACK
static void health_collect_thread_count(void)
{
	/* Simple thread count - iterate threads */
	uint32_t count = 0;
	struct k_thread *thread;

	/* Count all threads */
	STRUCT_SECTION_FOREACH(k_thread, thread) {
		count++;
	}

	builtin_metrics[HEALTH_IDX_THREAD_COUNT].value = count;
	builtin_metrics[HEALTH_IDX_THREAD_COUNT].timestamp = k_uptime_ticks();
}
#else
static void health_collect_thread_count(void)
{
	/* Already collected in stack usage */
}
#endif
#else
static void health_collect_thread_count(void)
{
	/* Thread monitoring not enabled */
}
#endif

/**
 * @brief Check thresholds and invoke callbacks
 */
static void health_check_thresholds(enum health_metric_type type, uint32_t value)
{
	k_spinlock_key_t key = k_spin_lock(&health_lock);

	for (size_t i = 0; i < callback_count; i++) {
		if (callbacks[i].cb == NULL) {
			continue;
		}

		if (callbacks[i].type != type) {
			continue;
		}

		/* Find the metric to get thresholds */
		struct health_metric *metric = NULL;

		if (type < HEALTH_METRIC_CUSTOM_START) {
			for (size_t j = 0; j < HEALTH_BUILTIN_METRIC_COUNT; j++) {
				if (builtin_metrics[j].type == type) {
					metric = &builtin_metrics[j];
					break;
				}
			}
		}
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
		else {
			for (size_t j = 0; j < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; j++) {
				if (custom_metrics[j].active &&
				    custom_metrics[j].metric_id == type) {
					/* Get metric from storage */
					/* For now, we'll pass the value directly */
					break;
				}
			}
		}
#endif

		if (metric == NULL) {
			continue;
		}

		enum health_status status = HEALTH_STATUS_OK;

		if (value >= metric->threshold_critical) {
			status = HEALTH_STATUS_CRITICAL;
		} else if (value >= metric->threshold_warning) {
			status = HEALTH_STATUS_WARNING;
		}

		if (status != HEALTH_STATUS_OK &&
		    callbacks[i].threshold_level == status) {
			health_threshold_cb cb = callbacks[i].cb;
			void *user_data = callbacks[i].user_data;

			k_spin_unlock(&health_lock, key);
			cb(type, value, status, user_data);
			key = k_spin_lock(&health_lock);
		}
	}

	k_spin_unlock(&health_lock, key);
}

/**
 * @brief Timer callback for periodic metric collection
 */
static void health_collect_metrics(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Collect built-in metrics */
#ifdef CONFIG_HEALTH_MONITOR_CPU
	health_collect_cpu_usage();
	health_check_thresholds(HEALTH_METRIC_CPU_USAGE,
				builtin_metrics[HEALTH_IDX_CPU_USAGE].value);
#endif

#ifdef CONFIG_HEALTH_MONITOR_MEMORY
	health_collect_memory_usage();
	health_check_thresholds(HEALTH_METRIC_MEMORY_USAGE,
				builtin_metrics[HEALTH_IDX_MEMORY_USAGE].value);
#endif

#ifdef CONFIG_HEALTH_MONITOR_STACK
	health_collect_stack_usage();
	health_check_thresholds(HEALTH_METRIC_STACK_USAGE,
				builtin_metrics[HEALTH_IDX_STACK_USAGE].value);
#endif

#ifdef CONFIG_HEALTH_MONITOR_THREADS
#ifndef CONFIG_HEALTH_MONITOR_STACK
	health_collect_thread_count();
#endif
#endif

	/* Collect custom metrics */
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		if (!custom_metrics[i].active || custom_metrics[i].collect_fn == NULL) {
			continue;
		}

		uint32_t value = custom_metrics[i].collect_fn(custom_metrics[i].user_data);

		/* Update metric value */
		/* Find metric in storage and update */
		/* For now, we'll store in a simplified way */
		health_check_thresholds(custom_metrics[i].metric_id, value);
	}
#endif

	k_spin_unlock(&health_lock, key);
}

int health_monitor_init(void)
{
	if (health_initialized) {
		return -EALREADY;
	}

	/* Initialize built-in metrics */
	health_init_builtin_metrics();

	/* Initialize timer */
	k_timer_init(&collection_timer, health_collect_metrics, NULL);

	/* Start periodic collection */
	k_timer_start(&collection_timer,
		      K_MSEC(CONFIG_HEALTH_MONITOR_UPDATE_INTERVAL_MS),
		      K_MSEC(CONFIG_HEALTH_MONITOR_UPDATE_INTERVAL_MS));

	health_initialized = true;

	LOG_INF("Health monitor initialized");

	return 0;
}

enum health_status health_get_status(void)
{
	enum health_status status = HEALTH_STATUS_OK;
	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Check all built-in metrics */
	for (size_t i = 0; i < HEALTH_BUILTIN_METRIC_COUNT; i++) {
		struct health_metric *m = &builtin_metrics[i];

		if (m->threshold_critical == 0 && m->threshold_warning == 0) {
			continue;
		}

		if (m->value >= m->threshold_critical) {
			status = HEALTH_STATUS_CRITICAL;
			k_spin_unlock(&health_lock, key);
			return status;
		}

		if (m->value >= m->threshold_warning) {
			status = HEALTH_STATUS_WARNING;
		}
	}

#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	/* Check custom metrics */
	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		if (!custom_metrics[i].active) {
			continue;
		}

		/* Get metric value and thresholds */
		/* Simplified: would need to store thresholds per custom metric */
	}
#endif

	k_spin_unlock(&health_lock, key);
	return status;
}

int health_get_metric(enum health_metric_type type, struct health_metric *metric)
{
	if (metric == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	if (type < HEALTH_METRIC_CUSTOM_START) {
		/* Built-in metric */
		for (size_t i = 0; i < HEALTH_BUILTIN_METRIC_COUNT; i++) {
			if (builtin_metrics[i].type == type) {
				memcpy(metric, &builtin_metrics[i], sizeof(*metric));
				k_spin_unlock(&health_lock, key);
				return 0;
			}
		}
		k_spin_unlock(&health_lock, key);
		return -ENODATA;
	}

#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	/* Custom metric */
	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		if (custom_metrics[i].active &&
		    custom_metrics[i].metric_id == type) {
			/* Fill metric structure */
			metric->type = type;
			metric->name = custom_metrics[i].name;
			metric->user_data = custom_metrics[i].user_data;
			/* Get current value by calling collect function */
			if (custom_metrics[i].collect_fn) {
				metric->value = custom_metrics[i].collect_fn(custom_metrics[i].user_data);
			}
			metric->timestamp = k_uptime_ticks();
			k_spin_unlock(&health_lock, key);
			return 0;
		}
	}
	k_spin_unlock(&health_lock, key);
	return -ENODATA;
#else
	k_spin_unlock(&health_lock, key);
	return -ENOTSUP;
#endif
}

int health_get_all_metrics(struct health_metric *metrics, size_t *count)
{
	if (metrics == NULL || count == NULL) {
		return -EINVAL;
	}

	size_t total_count = 0;
	size_t max_count = *count;

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Copy built-in metrics */
	for (size_t i = 0; i < HEALTH_BUILTIN_METRIC_COUNT && total_count < max_count; i++) {
		memcpy(&metrics[total_count], &builtin_metrics[i], sizeof(*metrics));
		total_count++;
	}

#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	/* Copy custom metrics */
	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS && total_count < max_count; i++) {
		if (custom_metrics[i].active) {
			metrics[total_count].type = custom_metrics[i].metric_id;
			metrics[total_count].name = custom_metrics[i].name;
			metrics[total_count].user_data = custom_metrics[i].user_data;
			if (custom_metrics[i].collect_fn) {
				metrics[total_count].value =
					custom_metrics[i].collect_fn(custom_metrics[i].user_data);
			}
			metrics[total_count].timestamp = k_uptime_ticks();
			total_count++;
		}
	}
#endif

	*count = total_count;
	k_spin_unlock(&health_lock, key);

	return 0;
}

int health_set_threshold(enum health_metric_type type,
			 uint32_t warning_threshold,
			 uint32_t critical_threshold)
{
	if (critical_threshold < warning_threshold) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	if (type < HEALTH_METRIC_CUSTOM_START) {
		/* Built-in metric */
		for (size_t i = 0; i < HEALTH_BUILTIN_METRIC_COUNT; i++) {
			if (builtin_metrics[i].type == type) {
				builtin_metrics[i].threshold_warning = warning_threshold;
				builtin_metrics[i].threshold_critical = critical_threshold;
				k_spin_unlock(&health_lock, key);
				return 0;
			}
		}
		k_spin_unlock(&health_lock, key);
		return -EINVAL;
	}

#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	/* Custom metric - would need to store thresholds */
	/* For now, return not supported */
	k_spin_unlock(&health_lock, key);
	return -ENOTSUP;
#else
	k_spin_unlock(&health_lock, key);
	return -ENOTSUP;
#endif
}

int health_register_callback(enum health_metric_type type,
			     enum health_status threshold_level,
			     health_threshold_cb cb,
			     void *user_data)
{
	if (threshold_level == HEALTH_STATUS_OK) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Find empty slot or existing callback to replace */
	size_t slot = SIZE_MAX;
	for (size_t i = 0; i < MAX_CALLBACKS; i++) {
		if (callbacks[i].cb == NULL) {
			slot = i;
			break;
		}
	}

	if (slot == SIZE_MAX) {
		k_spin_unlock(&health_lock, key);
		return -ENOMEM;
	}

	callbacks[slot].type = type;
	callbacks[slot].threshold_level = threshold_level;
	callbacks[slot].cb = cb;
	callbacks[slot].user_data = user_data;

	if (slot >= callback_count) {
		callback_count = slot + 1;
	}

	k_spin_unlock(&health_lock, key);
	return 0;
}

#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
int health_register_custom_metric(const char *name,
				   health_metric_collect_fn collect_fn,
				   uint32_t warning_threshold,
				   uint32_t critical_threshold,
				   void *user_data)
{
	if (name == NULL || collect_fn == NULL) {
		return -EINVAL;
	}

	if (critical_threshold < warning_threshold) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Find empty slot */
	size_t slot = SIZE_MAX;
	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		if (!custom_metrics[i].active) {
			slot = i;
			break;
		}
	}

	if (slot == SIZE_MAX) {
		k_spin_unlock(&health_lock, key);
		return -ENOMEM;
	}

	/* Assign ID */
	uint32_t metric_id = next_custom_id++;
	if (next_custom_id >= (HEALTH_METRIC_CUSTOM_START + CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS)) {
		next_custom_id = HEALTH_METRIC_CUSTOM_START;
	}

	custom_metrics[slot].name = name;
	custom_metrics[slot].collect_fn = collect_fn;
	custom_metrics[slot].user_data = user_data;
	custom_metrics[slot].metric_id = metric_id;
	custom_metrics[slot].active = true;

	k_spin_unlock(&health_lock, key);

	return (int)metric_id;
}

int health_unregister_custom_metric(uint32_t metric_id)
{
	if (metric_id < HEALTH_METRIC_CUSTOM_START) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		if (custom_metrics[i].active &&
		    custom_metrics[i].metric_id == metric_id) {
			custom_metrics[i].active = false;
			custom_metrics[i].collect_fn = NULL;
			custom_metrics[i].name = NULL;
			custom_metrics[i].user_data = NULL;
			k_spin_unlock(&health_lock, key);
			return 0;
		}
	}

	k_spin_unlock(&health_lock, key);
	return -ENOENT;
}
#endif /* CONFIG_HEALTH_MONITOR_CUSTOM_METRICS */

#ifdef CONFIG_HEALTH_MONITOR_HISTORY
int health_enable_history(enum health_metric_type type, size_t history_size)
{
	if (history_size == 0 || history_size > CONFIG_HEALTH_MONITOR_HISTORY_SIZE) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Find history buffer for this metric */
	size_t idx = SIZE_MAX;
	if (type < HEALTH_METRIC_CUSTOM_START) {
		idx = type;
	} else {
		/* Find custom metric index */
		for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
			if (custom_metrics[i].active &&
			    custom_metrics[i].metric_id == type) {
				idx = HEALTH_BUILTIN_METRIC_COUNT + i;
				break;
			}
		}
	}

	if (idx == SIZE_MAX || idx >= ARRAY_SIZE(history_buffers)) {
		k_spin_unlock(&health_lock, key);
		return -EINVAL;
	}

	/* Allocate history buffer */
	if (history_buffers[idx].values != NULL) {
		k_free(history_buffers[idx].values);
	}

	history_buffers[idx].values = k_malloc(history_size * sizeof(uint32_t));
	if (history_buffers[idx].values == NULL) {
		k_spin_unlock(&health_lock, key);
		return -ENOMEM;
	}

	history_buffers[idx].size = history_size;
	history_buffers[idx].count = 0;
	history_buffers[idx].head = 0;

	k_spin_unlock(&health_lock, key);
	return 0;
}

int health_get_history(enum health_metric_type type,
		       uint32_t *values,
		       size_t *count)
{
	if (values == NULL || count == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	/* Find history buffer */
	size_t idx = SIZE_MAX;
	if (type < HEALTH_METRIC_CUSTOM_START) {
		idx = type;
	} else {
		for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
			if (custom_metrics[i].active &&
			    custom_metrics[i].metric_id == type) {
				idx = HEALTH_BUILTIN_METRIC_COUNT + i;
				break;
			}
		}
	}

	if (idx == SIZE_MAX || idx >= ARRAY_SIZE(history_buffers)) {
		k_spin_unlock(&health_lock, key);
		return -EINVAL;
	}

	struct health_history *hist = &history_buffers[idx];
	if (hist->values == NULL || hist->count == 0) {
		*count = 0;
		k_spin_unlock(&health_lock, key);
		return 0;
	}

	size_t copy_count = MIN(*count, hist->count);
	for (size_t i = 0; i < copy_count; i++) {
		size_t src_idx = (hist->head - copy_count + i + hist->size) % hist->size;
		values[i] = hist->values[src_idx];
	}

	*count = copy_count;
	k_spin_unlock(&health_lock, key);
	return 0;
}
#endif /* CONFIG_HEALTH_MONITOR_HISTORY */

