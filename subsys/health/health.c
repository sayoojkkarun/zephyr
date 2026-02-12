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

#ifdef CONFIG_CPU_LOAD
#include <zephyr/debug/cpu_load.h>
#endif

#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS) && K_HEAP_MEM_POOL_SIZE > 0
#include <zephyr/sys/sys_heap.h>
extern struct k_heap _system_heap;
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
	uint32_t threshold_warning;
	uint32_t threshold_critical;
	bool active;
};

/* Threshold violation tracking for debouncing */
struct threshold_violation_state {
	uint64_t last_warning_time;
	uint64_t last_critical_time;
	bool warning_active;
	bool critical_active;
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

/* Work queue for metric collection (heavy operations) */
static struct k_work_delayable collection_work;

/* Thread safety */
static struct k_spinlock health_lock;

/* Timer control */
static bool timer_running;
static uint32_t current_interval_ms;

/* Initialization flag */
static bool health_initialized;

/* Timer control */
static bool timer_running;
static uint32_t current_interval_ms;

/* Threshold violation states for debouncing (prevent flooding) */
#define MAX_VIOLATION_STATES 16
static struct threshold_violation_state violation_states[MAX_VIOLATION_STATES];
static uint32_t violation_state_count;

/* Minimum time between same threshold violation callbacks (ms) */
#define THRESHOLD_DEBOUNCE_MS 5000

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
	builtin_metrics[HEALTH_IDX_CPU_USAGE].name = "CPU Usage";
	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].type = HEALTH_METRIC_MEMORY_USAGE;
	builtin_metrics[HEALTH_IDX_MEMORY_USAGE].name = "Memory Usage";
	builtin_metrics[HEALTH_IDX_STACK_USAGE].type = HEALTH_METRIC_STACK_USAGE;
	builtin_metrics[HEALTH_IDX_STACK_USAGE].name = "Stack Usage";
	builtin_metrics[HEALTH_IDX_THREAD_COUNT].type = HEALTH_METRIC_THREAD_COUNT;
	builtin_metrics[HEALTH_IDX_THREAD_COUNT].name = "Thread Count";
	builtin_metrics[HEALTH_IDX_THREADS_HIGH_STACK].type = HEALTH_METRIC_THREADS_HIGH_STACK;
	builtin_metrics[HEALTH_IDX_THREADS_HIGH_STACK].name = "Threads High Stack";
	builtin_metrics[HEALTH_IDX_FREE_HEAP].type = HEALTH_METRIC_FREE_HEAP;
	builtin_metrics[HEALTH_IDX_FREE_HEAP].name = "Free Heap";
	builtin_metrics[HEALTH_IDX_TOTAL_HEAP].type = HEALTH_METRIC_TOTAL_HEAP;
	builtin_metrics[HEALTH_IDX_TOTAL_HEAP].name = "Total Heap";

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

#ifdef CONFIG_CPU_LOAD
	int load = cpu_load_get(false);

	if (load >= 0) {
		/* cpu_load_get returns per mille (0-1000), convert to percent */
		cpu_usage = (uint32_t)(load / 10);
	}
#elif defined(CONFIG_STATS) && defined(CONFIG_SCHED_THREAD_USAGE)
	/* Use kernel stats if available */
	/* Note: CPU usage calculation via idle thread stats requires
	 * access to internal kernel APIs. For now, we rely on
	 * CONFIG_DEBUG_CPU_LOAD if available, otherwise set to 0.
	 */
	cpu_usage = 0;
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
	uint32_t memory_usage = 0;
	uint32_t free_heap = 0;
	uint32_t total_heap = 0;

#if K_HEAP_MEM_POOL_SIZE > 0
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
	struct sys_memory_stats stats;
	int err;

	err = sys_heap_runtime_stats_get(&_system_heap.heap, &stats);
	if (err == 0) {
		/* Get heap statistics */
		free_heap = (uint32_t)stats.free_bytes;
		/* Total heap is the sum of free and allocated bytes.
		 * Note: This may be slightly larger than K_HEAP_MEM_POOL_SIZE
		 * due to heap overhead/alignment.
		 */
		total_heap = (uint32_t)(stats.free_bytes + stats.allocated_bytes);
		
		/* Calculate memory usage percentage */
		if (total_heap > 0) {
			memory_usage = (stats.allocated_bytes * 100) / total_heap;
		} else {
			memory_usage = 0;
		}
	} else {
		/* Failed to get stats, use compile-time size */
		total_heap = K_HEAP_MEM_POOL_SIZE;
		free_heap = 0;
		memory_usage = 0;
	}
#else
	/* Runtime stats not available, use compile-time size */
	total_heap = K_HEAP_MEM_POOL_SIZE;
	free_heap = 0; /* Can't determine free heap without runtime stats */
	memory_usage = 0;
#endif
#elif defined(K_HEAP_MEM_POOL_SIZE) && K_HEAP_MEM_POOL_SIZE > 0
	/* Fallback: use compile-time heap size if runtime stats not available */
	total_heap = K_HEAP_MEM_POOL_SIZE;
	free_heap = 0; /* Can't determine free heap without runtime stats */
	memory_usage = 0;
#else
	/* No heap information available */
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
		/* Calculate average stack usage percentage across all threads */
		/* This is a weighted average: sum of all used stack / sum of all stack sizes */
		uint32_t avg_stack_usage = (stack_data.total_stack_used * 100) /
					    stack_data.total_stack_size;

		builtin_metrics[HEALTH_IDX_STACK_USAGE].value = avg_stack_usage;
		builtin_metrics[HEALTH_IDX_THREAD_COUNT].value = stack_data.thread_count;
		/* Number of threads with stack usage > 80% */
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
#ifdef CONFIG_THREAD_MONITOR
	STRUCT_SECTION_FOREACH(k_thread, thread) {
		count++;
	}
#else
	/* Fallback: use a simple count if thread monitor not available */
	count = 1; /* At least main thread */
#endif

	builtin_metrics[HEALTH_IDX_THREAD_COUNT].value = count;
	builtin_metrics[HEALTH_IDX_THREAD_COUNT].timestamp = k_uptime_ticks();
}
#endif
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
		uint32_t threshold_warning = 0;
		uint32_t threshold_critical = 0;
		bool found = false;

		if (type < HEALTH_METRIC_CUSTOM_START) {
			for (size_t j = 0; j < HEALTH_BUILTIN_METRIC_COUNT; j++) {
				if (builtin_metrics[j].type == type) {
					threshold_warning = builtin_metrics[j].threshold_warning;
					threshold_critical = builtin_metrics[j].threshold_critical;
					found = true;
					break;
				}
			}
		}
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
		else {
			for (size_t j = 0; j < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; j++) {
				if (custom_metrics[j].active &&
				    custom_metrics[j].metric_id == type) {
					threshold_warning = custom_metrics[j].threshold_warning;
					threshold_critical = custom_metrics[j].threshold_critical;
					found = true;
					break;
				}
			}
		}
#endif

		if (!found) {
			continue;
		}

		enum health_status status = HEALTH_STATUS_OK;

		if (threshold_critical > 0 && value >= threshold_critical) {
			status = HEALTH_STATUS_CRITICAL;
		} else if (threshold_warning > 0 && value >= threshold_warning) {
			status = HEALTH_STATUS_WARNING;
		}

		if (status != HEALTH_STATUS_OK &&
		    callbacks[i].threshold_level == status) {
			/* Debounce: only call callback if enough time has passed */
			uint64_t now = k_uptime_get();
			uint64_t *last_time_ptr = NULL;
			bool *active_ptr = NULL;

			/* Find or create violation state for this metric */
			size_t state_idx = SIZE_MAX;
			for (size_t j = 0; j < violation_state_count; j++) {
				/* Use metric type as identifier - simple approach */
				if (state_idx == SIZE_MAX) {
					state_idx = j;
					break;
				}
			}

			if (state_idx == SIZE_MAX && violation_state_count < MAX_VIOLATION_STATES) {
				state_idx = violation_state_count++;
			}

			if (state_idx != SIZE_MAX) {
				if (status == HEALTH_STATUS_CRITICAL) {
					last_time_ptr = &violation_states[state_idx].last_critical_time;
					active_ptr = &violation_states[state_idx].critical_active;
				} else {
					last_time_ptr = &violation_states[state_idx].last_warning_time;
					active_ptr = &violation_states[state_idx].warning_active;
				}

				/* Check if enough time has passed since last callback */
				bool should_call = false;
				if (last_time_ptr != NULL && active_ptr != NULL) {
					if (!(*active_ptr)) {
						/* First violation - always report */
						should_call = true;
						*active_ptr = true;
						*last_time_ptr = now;
					} else if ((now - *last_time_ptr) >= THRESHOLD_DEBOUNCE_MS) {
						/* Enough time has passed */
						should_call = true;
						*last_time_ptr = now;
					}
				} else {
					/* No debounce tracking available - call immediately */
					should_call = true;
				}

				if (should_call) {
					health_threshold_cb cb = callbacks[i].cb;
					void *user_data = callbacks[i].user_data;

					k_spin_unlock(&health_lock, key);
					cb(type, value, status, user_data);
					key = k_spin_lock(&health_lock);
				}
			} else {
				/* No debounce tracking available - call immediately */
				health_threshold_cb cb = callbacks[i].cb;
				void *user_data = callbacks[i].user_data;

				k_spin_unlock(&health_lock, key);
				cb(type, value, status, user_data);
				key = k_spin_lock(&health_lock);
			}
		} else if (status == HEALTH_STATUS_OK) {
			/* Reset active flags when metric returns to normal */
			for (size_t j = 0; j < violation_state_count; j++) {
				if (callbacks[i].threshold_level == HEALTH_STATUS_WARNING) {
					violation_states[j].warning_active = false;
				} else if (callbacks[i].threshold_level == HEALTH_STATUS_CRITICAL) {
					violation_states[j].critical_active = false;
				}
			}
		}
	}

	k_spin_unlock(&health_lock, key);
}

/**
 * @brief Work queue handler for metric collection
 *
 * This runs in thread context (sysworkq) to perform heavy operations
 * like thread analysis and custom metric collection.
 */
static void health_collect_metrics_work(struct k_work *work)
{
	ARG_UNUSED(work);

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

/**
 * @brief Timer callback for periodic metric collection
 *
 * This runs in ISR context and just submits work to the work queue.
 * Heavy operations are deferred to thread context.
 */
static void health_collect_metrics(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	/* Submit work to work queue - this is safe to call from ISR */
	k_work_submit(&collection_work.work);
}

int health_monitor_init(void)
{
	if (health_initialized) {
		return -EALREADY;
	}

	/* Initialize built-in metrics */
	health_init_builtin_metrics();

	/* Initialize work queue for metric collection */
	k_work_init_delayable(&collection_work, health_collect_metrics_work);

	/* Initialize timer - timer just triggers work submission */
	k_timer_init(&collection_timer, health_collect_metrics, NULL);

	/* Set initial interval */
	current_interval_ms = CONFIG_HEALTH_MONITOR_UPDATE_INTERVAL_MS;

	/* Start periodic collection */
	k_timer_start(&collection_timer,
		      K_MSEC(current_interval_ms),
		      K_MSEC(current_interval_ms));

	timer_running = true;
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

		/* Get metric value */
		uint32_t value = 0;
		if (custom_metrics[i].collect_fn) {
			value = custom_metrics[i].collect_fn(custom_metrics[i].user_data);
		}

		uint32_t threshold_warning = custom_metrics[i].threshold_warning;
		uint32_t threshold_critical = custom_metrics[i].threshold_critical;

		if (threshold_critical > 0 && value >= threshold_critical) {
			status = HEALTH_STATUS_CRITICAL;
			k_spin_unlock(&health_lock, key);
			return status;
		}

		if (threshold_warning > 0 && value >= threshold_warning) {
			status = HEALTH_STATUS_WARNING;
		}
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
			metric->threshold_warning = custom_metrics[i].threshold_warning;
			metric->threshold_critical = custom_metrics[i].threshold_critical;
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
			metrics[total_count].threshold_warning = custom_metrics[i].threshold_warning;
			metrics[total_count].threshold_critical = custom_metrics[i].threshold_critical;
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
	/* Custom metric */
	for (size_t i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		if (custom_metrics[i].active &&
		    custom_metrics[i].metric_id == type) {
			custom_metrics[i].threshold_warning = warning_threshold;
			custom_metrics[i].threshold_critical = critical_threshold;
			k_spin_unlock(&health_lock, key);
			return 0;
		}
	}
	k_spin_unlock(&health_lock, key);
	return -EINVAL;
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
	custom_metrics[slot].threshold_warning = warning_threshold;
	custom_metrics[slot].threshold_critical = critical_threshold;
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

int health_monitor_start(void)
{
	if (!health_initialized) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	if (!timer_running) {
		k_timer_start(&collection_timer,
			      K_MSEC(current_interval_ms),
			      K_MSEC(current_interval_ms));
		timer_running = true;
	}

	k_spin_unlock(&health_lock, key);
	return 0;
}

int health_monitor_stop(void)
{
	if (!health_initialized) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	if (timer_running) {
		k_timer_stop(&collection_timer);
		k_work_cancel_delayable(&collection_work);
		timer_running = false;
	}

	k_spin_unlock(&health_lock, key);
	return 0;
}

int health_monitor_set_interval(uint32_t interval_ms)
{
	if (!health_initialized) {
		return -ENOTSUP;
	}

	if (interval_ms < 100 || interval_ms > 60000) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);

	current_interval_ms = interval_ms;

	/* Restart timer with new interval if running */
	if (timer_running) {
		/* Stop the timer - this cancels any pending expirations */
		k_timer_stop(&collection_timer);
		/* Cancel any pending work to prevent race conditions */
		k_work_cancel_delayable(&collection_work);
		
		/* Restart with new interval */
		k_timer_start(&collection_timer,
			      K_MSEC(current_interval_ms),
			      K_MSEC(current_interval_ms));
		LOG_INF("Timer interval changed to %u ms", interval_ms);
	}

	k_spin_unlock(&health_lock, key);
	return 0;
}

uint32_t health_monitor_get_interval(void)
{
	if (!health_initialized) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&health_lock);
	uint32_t interval = current_interval_ms;
	k_spin_unlock(&health_lock, key);

	return interval;
}

