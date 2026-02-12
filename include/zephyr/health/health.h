/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System Health Monitor API
 *
 * This header file declares the System Health Monitor APIs for monitoring
 * system health metrics including CPU usage, memory usage, stack usage,
 * and thread states. The subsystem also supports user-defined custom
 * metrics.
 */

#ifndef ZEPHYR_INCLUDE_HEALTH_HEALTH_H_
#define ZEPHYR_INCLUDE_HEALTH_HEALTH_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Health Monitor APIs
 * @defgroup health_api Health Monitor APIs
 * @ingroup os_services
 * @{
 */

/**
 * @brief Health metric types
 */
enum health_metric_type {
	/** CPU usage percentage (0-100) */
	HEALTH_METRIC_CPU_USAGE,
	/** Memory usage percentage (0-100) */
	HEALTH_METRIC_MEMORY_USAGE,
	/** Average stack usage percentage across all threads (0-100).
	 *  This is a weighted average: (sum of all used stack) / (sum of all stack sizes) * 100
	 */
	HEALTH_METRIC_STACK_USAGE,
	/** Number of active threads */
	HEALTH_METRIC_THREAD_COUNT,
	/** Number of threads with individual stack usage > 80%.
	 *  This counts threads where (thread_stack_used / thread_stack_size) > 80%
	 */
	HEALTH_METRIC_THREADS_HIGH_STACK,
	/** Free heap memory in bytes */
	HEALTH_METRIC_FREE_HEAP,
	/** Total heap size in bytes */
	HEALTH_METRIC_TOTAL_HEAP,
	/** User-defined metrics start from this value */
	HEALTH_METRIC_CUSTOM_START = 100,
};

/**
 * @brief Overall system health status
 */
enum health_status {
	/** All metrics within normal ranges */
	HEALTH_STATUS_OK,
	/** One or more metrics exceed warning thresholds */
	HEALTH_STATUS_WARNING,
	/** One or more metrics exceed critical thresholds */
	HEALTH_STATUS_CRITICAL,
};

/**
 * @brief Health metric value and thresholds
 */
struct health_metric {
	/** Metric type (or custom metric ID) */
	enum health_metric_type type;
	/** Metric name (for custom metrics) */
	const char *name;
	/** Current value (percentage or absolute) */
	uint32_t value;
	/** Warning threshold */
	uint32_t threshold_warning;
	/** Critical threshold */
	uint32_t threshold_critical;
	/** Timestamp of last update (system ticks) */
	uint64_t timestamp;
	/** User data pointer (for custom metrics) */
	void *user_data;
};

/**
 * @brief User-defined metric collection function
 *
 * This function is called periodically by the health monitor to collect
 * the current value of a custom metric.
 *
 * @param user_data User data provided during registration
 * @return Current metric value (0-100 for percentages, or absolute value)
 */
typedef uint32_t (*health_metric_collect_fn)(void *user_data);

/**
 * @brief Initialize the health monitor subsystem
 *
 * Must be called before using any other health monitor APIs.
 * Typically called during system initialization.
 *
 * @return 0 on success, negative errno on error
 */
int health_monitor_init(void);

/**
 * @brief Get overall system health status
 *
 * Aggregates all monitored metrics to determine overall health.
 *
 * @return Current health status
 */
enum health_status health_get_status(void);

/**
 * @brief Get a specific health metric
 *
 * @param type Metric type to retrieve
 * @param metric Pointer to structure to fill with metric data
 * @return 0 on success, -EINVAL if type is invalid, -ENODATA if metric not available
 */
int health_get_metric(enum health_metric_type type, struct health_metric *metric);

/**
 * @brief Get all available metrics
 *
 * Fills an array with all currently available metrics.
 *
 * @param metrics Array to fill with metric data
 * @param count Size of array (input) / number of metrics returned (output)
 * @return 0 on success, negative errno on error
 */
int health_get_all_metrics(struct health_metric *metrics, size_t *count);

/**
 * @brief Set threshold for a metric
 *
 * @param type Metric type
 * @param warning_threshold Warning threshold value
 * @param critical_threshold Critical threshold value
 * @return 0 on success, negative errno on error
 */
int health_set_threshold(enum health_metric_type type,
			 uint32_t warning_threshold,
			 uint32_t critical_threshold);

/**
 * @brief Threshold violation callback function
 *
 * @param type Metric type that violated threshold
 * @param value Current metric value
 * @param status Health status (WARNING or CRITICAL)
 * @param user_data User data provided during registration
 */
typedef void (*health_threshold_cb)(enum health_metric_type type,
				    uint32_t value,
				    enum health_status status,
				    void *user_data);

/**
 * @brief Register a callback for threshold violations
 *
 * The callback is invoked when a metric exceeds its threshold.
 * Multiple callbacks can be registered for the same metric type.
 *
 * @param type Metric type to monitor
 * @param threshold_level Which threshold to monitor (WARNING or CRITICAL)
 * @param cb Callback function (NULL to unregister)
 * @param user_data User data passed to callback
 * @return 0 on success, negative errno on error
 */
int health_register_callback(enum health_metric_type type,
			     enum health_status threshold_level,
			     health_threshold_cb cb,
			     void *user_data);

/**
 * @brief Register a user-defined metric
 *
 * Allows applications to register custom health metrics that are collected
 * and monitored alongside built-in metrics. Custom metrics are assigned IDs
 * starting from HEALTH_METRIC_CUSTOM_START.
 *
 * @param name Human-readable name for the metric (e.g., "Battery Level")
 * @param collect_fn Function to call to collect the metric value
 * @param warning_threshold Warning threshold value
 * @param critical_threshold Critical threshold value
 * @param user_data User data passed to collect_fn
 * @return Custom metric ID (>= HEALTH_METRIC_CUSTOM_START) on success,
 *         negative errno on error
 */
int health_register_custom_metric(const char *name,
				    health_metric_collect_fn collect_fn,
				    uint32_t warning_threshold,
				    uint32_t critical_threshold,
				    void *user_data);

/**
 * @brief Unregister a user-defined metric
 *
 * @param metric_id Custom metric ID returned by health_register_custom_metric
 * @return 0 on success, negative errno on error
 */
int health_unregister_custom_metric(uint32_t metric_id);

/**
 * @brief Enable metric history tracking
 *
 * Maintains a circular buffer of recent metric values for trend analysis.
 *
 * @param type Metric type
 * @param history_size Number of samples to maintain
 * @return 0 on success, negative errno on error
 */
int health_enable_history(enum health_metric_type type, size_t history_size);

/**
 * @brief Get metric history
 *
 * @param type Metric type
 * @param values Array to fill with historical values
 * @param count Size of array (input) / number of samples returned (output)
 * @return 0 on success, negative errno on error
 */
int health_get_history(enum health_metric_type type,
		       uint32_t *values,
		       size_t *count);

/**
 * @brief Start health monitor collection
 *
 * Starts or resumes periodic metric collection.
 *
 * @return 0 on success, negative errno on error
 */
int health_monitor_start(void);

/**
 * @brief Stop health monitor collection
 *
 * Stops periodic metric collection. Metrics can still be queried
 * manually, but automatic collection is paused.
 *
 * @return 0 on success, negative errno on error
 */
int health_monitor_stop(void);

/**
 * @brief Set health monitor update interval
 *
 * Changes the interval at which metrics are collected. Takes effect
 * on the next collection cycle.
 *
 * @param interval_ms Update interval in milliseconds
 * @return 0 on success, negative errno on error
 */
int health_monitor_set_interval(uint32_t interval_ms);

/**
 * @brief Get health monitor update interval
 *
 * @return Current update interval in milliseconds, or 0 if not initialized
 */
uint32_t health_monitor_get_interval(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_HEALTH_HEALTH_H_ */

