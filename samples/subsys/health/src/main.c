/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Health Monitor Sample Application
 *
 * This sample demonstrates the Health Monitor subsystem with built-in
 * metrics and custom metrics. It shows how to:
 * - Initialize the health monitor
 * - Register custom metrics
 * - Set up threshold callbacks
 * - Query health status
 * - Use shell commands to interact with the health monitor
 */

#include <zephyr/kernel.h>
#include <zephyr/health/health.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>

/* Simulated battery level data */
struct battery_data {
	uint8_t level;
	bool discharging;
};

static struct battery_data battery_state = {
	.level = 100,
	.discharging = true,
};

/* Simulated network latency data */
static uint32_t network_latency = 50;

/* Custom metric: Battery level collection function */
static uint32_t battery_collect_fn(void *user_data)
{
	struct battery_data *bat = (struct battery_data *)user_data;

	if (bat == NULL) {
		return 0;
	}

	/* Simulate battery discharge/charge */
	if (bat->discharging) {
		if (bat->level > 0) {
			bat->level--;
		} else {
			bat->discharging = false;
		}
	} else {
		if (bat->level < 100) {
			bat->level++;
		} else {
			bat->discharging = true;
		}
	}

	return bat->level;
}

/* Custom metric: Network latency collection function */
static uint32_t latency_collect_fn(void *user_data)
{
	ARG_UNUSED(user_data);

	/* Simulate network latency variation (50-150ms) */
	network_latency = 50 + (k_cycle_get_32() % 100);

	return network_latency;
}

/* Threshold callback for battery level */
static void battery_warning_cb(enum health_metric_type type,
				uint32_t value,
				enum health_status status,
				void *user_data)
{
	ARG_UNUSED(type);
	ARG_UNUSED(user_data);

	if (status == HEALTH_STATUS_CRITICAL) {
		printk("WARNING: Battery level critical: %u%%\n", value);
	} else if (status == HEALTH_STATUS_WARNING) {
		printk("WARNING: Battery level low: %u%%\n", value);
	}
}

/* Threshold callback for network latency */
static void latency_warning_cb(enum health_metric_type type,
				uint32_t value,
				enum health_status status,
				void *user_data)
{
	ARG_UNUSED(type);
	ARG_UNUSED(user_data);

	if (status == HEALTH_STATUS_CRITICAL) {
		printk("WARNING: Network latency critical: %u ms\n", value);
	} else if (status == HEALTH_STATUS_WARNING) {
		printk("WARNING: Network latency high: %u ms\n", value);
	}
}

/* Work item for periodic status printing */
static struct k_work_delayable status_work;

static void print_health_status(struct k_work *work)
{
	ARG_UNUSED(work);

	enum health_status status = health_get_status();
	struct health_metric metrics[8];
	size_t count = ARRAY_SIZE(metrics);

	printk("\n=== Health Status ===\n");
	printk("Overall Status: ");

	switch (status) {
	case HEALTH_STATUS_OK:
		printk("OK\n");
		break;
	case HEALTH_STATUS_WARNING:
		printk("WARNING\n");
		break;
	case HEALTH_STATUS_CRITICAL:
		printk("CRITICAL\n");
		break;
	default:
		printk("UNKNOWN\n");
		break;
	}

	if (health_get_all_metrics(metrics, &count) == 0) {
		printk("Metrics:\n");
		for (size_t i = 0; i < count; i++) {
			const char *name = metrics[i].name;

			if (name == NULL) {
				switch (metrics[i].type) {
				case HEALTH_METRIC_CPU_USAGE:
					name = "CPU Usage";
					break;
				case HEALTH_METRIC_MEMORY_USAGE:
					name = "Memory Usage";
					break;
				case HEALTH_METRIC_STACK_USAGE:
					name = "Stack Usage";
					break;
				case HEALTH_METRIC_THREAD_COUNT:
					name = "Thread Count";
					break;
				default:
					name = "Unknown";
					break;
				}
			}

			printk("  %s: %u", name, metrics[i].value);

			if (metrics[i].threshold_warning > 0 ||
			    metrics[i].threshold_critical > 0) {
				printk(" (W:%u, C:%u)", metrics[i].threshold_warning,
				       metrics[i].threshold_critical);
			}
			printk("\n");
		}
	}

	printk("Use 'health status' shell command for more details\n");
	printk("==================\n\n");

	/* Schedule next status print */
	k_work_schedule(&status_work, K_SECONDS(10));
}

int main(void)
{
	int ret;
	uint32_t battery_metric_id;
	uint32_t latency_metric_id;

	printk("Health Monitor Sample Application\n");
	printk("==================================\n\n");

	/* Initialize health monitor */
	ret = health_monitor_init();
	if (ret != 0) {
		printk("Failed to initialize health monitor: %d\n", ret);
		return 0;
	}

	printk("Health monitor initialized\n");

	/* Register custom metrics */
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	battery_metric_id = health_register_custom_metric(
		"Battery Level",
		battery_collect_fn,
		20,  /* Warning at 20% */
		10,  /* Critical at 10% */
		&battery_state
	);

	if (battery_metric_id < 0) {
		printk("Failed to register battery metric: %d\n", battery_metric_id);
	} else {
		printk("Registered battery metric (ID: %u)\n", battery_metric_id);
	}

	latency_metric_id = health_register_custom_metric(
		"Network Latency",
		latency_collect_fn,
		100, /* Warning at 100ms */
		200, /* Critical at 200ms */
		NULL
	);

	if (latency_metric_id < 0) {
		printk("Failed to register latency metric: %d\n", latency_metric_id);
	} else {
		printk("Registered network latency metric (ID: %u)\n",
		       latency_metric_id);
	}

	/* Register threshold callbacks */
	if (battery_metric_id >= HEALTH_METRIC_CUSTOM_START) {
		health_register_callback(battery_metric_id, HEALTH_STATUS_WARNING,
					 battery_warning_cb, NULL);
		health_register_callback(battery_metric_id, HEALTH_STATUS_CRITICAL,
					 battery_warning_cb, NULL);
		printk("Registered battery threshold callbacks\n");
	}

	if (latency_metric_id >= HEALTH_METRIC_CUSTOM_START) {
		health_register_callback(latency_metric_id, HEALTH_STATUS_WARNING,
					 latency_warning_cb, NULL);
		health_register_callback(latency_metric_id, HEALTH_STATUS_CRITICAL,
					 latency_warning_cb, NULL);
		printk("Registered latency threshold callbacks\n");
	}
#endif

	printk("\nAvailable shell commands:\n");
	printk("  health status       - Show overall health status\n");
	printk("  health metric <id>  - Show specific metric\n");
	printk("  health threshold <id> <w> <c> - Set thresholds\n");
#ifdef CONFIG_HEALTH_MONITOR_HISTORY
	printk("  health history <id> - Show metric history\n");
#endif
	printk("\n");

	/* Start periodic status printing */
	k_work_init_delayable(&status_work, print_health_status);
	k_work_schedule(&status_work, K_SECONDS(5));

	/* Main loop - just keep running */
	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

