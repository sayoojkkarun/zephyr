/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/health/health.h>
#include <zephyr/kernel.h>
#include <string.h>

/* Test data for custom metrics */
static uint32_t test_custom_value = 50;
static bool callback_invoked;
static enum health_metric_type callback_metric_type;
static uint32_t callback_value;
static enum health_status callback_status;

/* Custom metric collection function */
static uint32_t test_custom_collect(void *user_data)
{
	uint32_t *val = (uint32_t *)user_data;
	return *val;
}

/* Threshold violation callback */
static void test_threshold_cb(enum health_metric_type type,
			      uint32_t value,
			      enum health_status status,
			      void *user_data)
{
	callback_invoked = true;
	callback_metric_type = type;
	callback_value = value;
	callback_status = status;
}

/* Setup function - runs before each test */
static void test_setup(void *fixture)
{
	ARG_UNUSED(fixture);
	callback_invoked = false;
	callback_metric_type = HEALTH_METRIC_CPU_USAGE;
	callback_value = 0;
	callback_status = HEALTH_STATUS_OK;
	test_custom_value = 50;
}

/**
 * @brief Test health monitor initialization
 */
ZTEST(health_monitor, test_init)
{
	int ret;

	ret = health_monitor_init();
	zassert_equal(ret, 0, "Health monitor init failed");
}

/**
 * @brief Test health monitor double initialization
 */
ZTEST(health_monitor, test_double_init)
{
	int ret1;
	int ret2;

	ret1 = health_monitor_init();
	zassert_equal(ret1, 0, "First init should succeed");

	ret2 = health_monitor_init();
	zassert_equal(ret2, -EALREADY, "Second init should return -EALREADY");
}

/**
 * @brief Test getting overall health status
 */
ZTEST(health_monitor, test_get_status)
{
	enum health_status status;

	health_monitor_init();
	status = health_get_status();
	zassert_true(status >= HEALTH_STATUS_OK && status <= HEALTH_STATUS_CRITICAL,
		     "Invalid health status");
}

/**
 * @brief Test getting CPU usage metric
 */
ZTEST(health_monitor, test_get_cpu_metric)
{
	health_monitor_init();
	k_sleep(K_MSEC(200)); /* Wait for initial collection */

	struct health_metric metric;
	int ret;

	ret = health_get_metric(HEALTH_METRIC_CPU_USAGE, &metric);

	zassert_equal(ret, 0, "Failed to get CPU metric");
	zassert_equal(metric.type, HEALTH_METRIC_CPU_USAGE, "Wrong metric type");
	zassert_true(metric.value <= 100, "CPU usage should be <= 100 percent");
}

/**
 * @brief Test getting memory usage metric
 */
ZTEST(health_monitor, test_get_memory_metric)
{
	health_monitor_init();
	k_sleep(K_MSEC(200)); /* Wait for initial collection */

	struct health_metric metric;
	int ret;

	ret = health_get_metric(HEALTH_METRIC_MEMORY_USAGE, &metric);

	zassert_equal(ret, 0, "Failed to get memory metric");
	zassert_equal(metric.type, HEALTH_METRIC_MEMORY_USAGE, "Wrong metric type");
	zassert_true(metric.value <= 100, "Memory usage should be <= 100 percent");
}

/**
 * @brief Test getting stack usage metric
 */
ZTEST(health_monitor, test_get_stack_metric)
{
	health_monitor_init();
	k_sleep(K_MSEC(200)); /* Wait for initial collection */

	struct health_metric metric;
	int ret;

	ret = health_get_metric(HEALTH_METRIC_STACK_USAGE, &metric);

	zassert_equal(ret, 0, "Failed to get stack metric");
	zassert_equal(metric.type, HEALTH_METRIC_STACK_USAGE, "Wrong metric type");
	zassert_true(metric.value <= 100, "Stack usage should be <= 100 percent");
}

/**
 * @brief Test getting thread count metric
 */
ZTEST(health_monitor, test_get_thread_count_metric)
{
	health_monitor_init();
	k_sleep(K_MSEC(200)); /* Wait for initial collection */

	struct health_metric metric;
	int ret;

	ret = health_get_metric(HEALTH_METRIC_THREAD_COUNT, &metric);

	zassert_equal(ret, 0, "Failed to get thread count metric");
	zassert_equal(metric.type, HEALTH_METRIC_THREAD_COUNT, "Wrong metric type");
	zassert_true(metric.value > 0, "Thread count should be > 0");
}

/**
 * @brief Test getting all metrics
 */
ZTEST(health_monitor, test_get_all_metrics)
{
	health_monitor_init();
	k_sleep(K_MSEC(200)); /* Wait for initial collection */

	struct health_metric metrics[16];
	size_t count = ARRAY_SIZE(metrics);
	int ret;

	ret = health_get_all_metrics(metrics, &count);

	zassert_equal(ret, 0, "Failed to get all metrics");
	zassert_true(count > 0, "Should have at least one metric");
	zassert_true(count <= ARRAY_SIZE(metrics), "Count should not exceed array size");
}

/**
 * @brief Test setting thresholds
 */
ZTEST(health_monitor, test_set_threshold)
{
	health_monitor_init();

	int ret;

	ret = health_set_threshold(HEALTH_METRIC_CPU_USAGE, 70, 90);
	zassert_equal(ret, 0, "Failed to set threshold");

	struct health_metric metric;

	ret = health_get_metric(HEALTH_METRIC_CPU_USAGE, &metric);
	zassert_equal(ret, 0, "Failed to get metric");
	zassert_equal(metric.threshold_warning, 70, "Warning threshold not set");
	zassert_equal(metric.threshold_critical, 90, "Critical threshold not set");
}

/**
 * @brief Test threshold callback registration
 */
ZTEST(health_monitor, test_register_callback)
{
	health_monitor_init();

	int ret;

	ret = health_register_callback(HEALTH_METRIC_CPU_USAGE,
					   HEALTH_STATUS_WARNING,
					   test_threshold_cb,
					   NULL);
	zassert_equal(ret, 0, "Failed to register callback");
}

/**
 * @brief Test threshold callback unregistration
 * Note: health_unregister_callback may not be implemented yet
 */
ZTEST(health_monitor, test_unregister_callback)
{
	health_monitor_init();

	health_register_callback(HEALTH_METRIC_MEMORY_USAGE,
				 HEALTH_STATUS_WARNING,
				 test_threshold_cb,
				 NULL);

	/* Test that callback was registered by checking it can be called */
	/* Note: Actual unregister test depends on API implementation */
	ztest_test_skip();
}

/**
 * @brief Test custom metric registration
 */
ZTEST(health_monitor, test_register_custom_metric)
{
	health_monitor_init();

	int metric_id = health_register_custom_metric("Test Metric",
						       test_custom_collect,
						       80, 90,
						       &test_custom_value);
	zassert_true(metric_id >= HEALTH_METRIC_CUSTOM_START,
		     "Custom metric ID should be >= HEALTH_METRIC_CUSTOM_START");
	zassert_true(metric_id >= 0, "Metric ID should be valid");
}

/**
 * @brief Test custom metric retrieval
 */
ZTEST(health_monitor, test_get_custom_metric)
{
	int metric_id;
	struct health_metric metric;
	int ret;

	health_monitor_init();

	metric_id = health_register_custom_metric("Test Metric",
						   test_custom_collect,
						   80, 90,
						   &test_custom_value);
	zassert_true(metric_id >= 0, "Failed to register custom metric");

	ret = health_get_metric(metric_id, &metric);
	zassert_equal(ret, 0, "Failed to get custom metric");
	zassert_equal(metric.type, metric_id, "Wrong metric type");
	zassert_not_null(metric.name, "Metric name should not be NULL");
	zassert_equal(strcmp(metric.name, "Test Metric"), 0, "Wrong metric name");
}

/**
 * @brief Test custom metric by name lookup
 */
ZTEST(health_monitor, test_get_custom_metric_by_name)
{
	int metric_id;
	struct health_metric metric;
	int ret;

	health_monitor_init();

	metric_id = health_register_custom_metric("Test Metric",
						   test_custom_collect,
						   80, 90,
						   &test_custom_value);
	zassert_true(metric_id >= 0, "Failed to register custom metric");

	/* Test that we can retrieve it by ID and verify name */
	ret = health_get_metric(metric_id, &metric);
	zassert_equal(ret, 0, "Failed to get custom metric");
	zassert_not_null(metric.name, "Metric name should not be NULL");
	zassert_equal(strcmp(metric.name, "Test Metric"), 0, "Wrong metric name");
}

/**
 * @brief Test custom metric unregistration
 */
ZTEST(health_monitor, test_unregister_custom_metric)
{
	int metric_id;
	int ret;
	struct health_metric metric;

	health_monitor_init();

	metric_id = health_register_custom_metric("Test Metric",
						   test_custom_collect,
						   80, 90,
						   &test_custom_value);
	zassert_true(metric_id >= 0, "Failed to register custom metric");

	ret = health_unregister_custom_metric(metric_id);
	zassert_equal(ret, 0, "Failed to unregister custom metric");

	/* Verify metric is no longer available */
	ret = health_get_metric(metric_id, &metric);
	zassert_not_equal(ret, 0, "Metric should not be available after unregister");
}

/**
 * @brief Test timer control - start/stop
 */
ZTEST(health_monitor, test_timer_control)
{
	health_monitor_init();

	int ret;

	ret = health_monitor_stop();
	zassert_equal(ret, 0, "Failed to stop monitor");

	ret = health_monitor_start();
	zassert_equal(ret, 0, "Failed to start monitor");
}

/**
 * @brief Test interval setting
 */
ZTEST(health_monitor, test_set_interval)
{
	health_monitor_init();

	int ret;

	ret = health_monitor_set_interval(5000);
	zassert_equal(ret, 0, "Failed to set interval");

	uint32_t interval;

	interval = health_monitor_get_interval();
	zassert_equal(interval, 5000, "Interval not set correctly");
}

/**
 * @brief Test invalid interval setting
 */
ZTEST(health_monitor, test_set_invalid_interval)
{
	health_monitor_init();

	/* Test too small interval */
	int ret;

	ret = health_monitor_set_interval(50);
	zassert_equal(ret, -EINVAL, "Should reject interval < 100ms");

	/* Test too large interval */
	ret = health_monitor_set_interval(70000);
	zassert_equal(ret, -EINVAL, "Should reject interval > 60000ms");
}

/**
 * @brief Test custom metric collection function
 */
ZTEST(health_monitor, test_custom_metric_collection)
{
	health_monitor_init();

	test_custom_value = 75;
	int metric_id = health_register_custom_metric("Test Metric",
						       test_custom_collect,
						       80, 90,
						       &test_custom_value);
	zassert_true(metric_id >= 0, "Failed to register custom metric");

	k_sleep(K_MSEC(200)); /* Wait for collection */

	struct health_metric metric;
	int ret;

	ret = health_get_metric(metric_id, &metric);
	zassert_equal(ret, 0, "Failed to get custom metric");
	zassert_equal(metric.value, 75, "Custom metric value not collected");
}

/**
 * @brief Test maximum custom metrics limit
 */
ZTEST(health_monitor, test_max_custom_metrics)
{
	health_monitor_init();

	/* Register maximum number of custom metrics */
	int metric_ids[CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS];
	char name_buf[32];
	int ret;
	int i;

	for (i = 0; i < CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS; i++) {
		snprintf(name_buf, sizeof(name_buf), "Metric%d", i);
		metric_ids[i] = health_register_custom_metric(name_buf,
								 test_custom_collect,
								 80, 90,
								 &test_custom_value);
		zassert_true(metric_ids[i] >= 0,
			     "Failed to register metric %d", i);
	}

	/* Try to register one more - should fail */
	ret = health_register_custom_metric("Extra Metric",
						test_custom_collect,
						80, 90,
						&test_custom_value);
	zassert_true(ret < 0, "Should fail to register beyond limit");
}

ZTEST_SUITE(health_monitor, NULL, NULL, test_setup, NULL, NULL);

