/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Health Monitor Shell Commands
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/health/health.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

/* Helper function to get metric type from name or ID */
static enum health_metric_type parse_metric_id(const char *str)
{
	char *endptr;
	long val = strtol(str, &endptr, 0);

	/* If it's a number, treat as metric ID */
	if (*endptr == '\0') {
		return (enum health_metric_type)val;
	}

	/* Try to match by name */
	if (strcmp(str, "cpu") == 0 || strcmp(str, "CPU") == 0) {
		return HEALTH_METRIC_CPU_USAGE;
	}
	if (strcmp(str, "memory") == 0 || strcmp(str, "mem") == 0) {
		return HEALTH_METRIC_MEMORY_USAGE;
	}
	if (strcmp(str, "stack") == 0) {
		return HEALTH_METRIC_STACK_USAGE;
	}
	if (strcmp(str, "threads") == 0 || strcmp(str, "thread_count") == 0) {
		return HEALTH_METRIC_THREAD_COUNT;
	}
	if (strcmp(str, "high_stack") == 0) {
		return HEALTH_METRIC_THREADS_HIGH_STACK;
	}
	if (strcmp(str, "free_heap") == 0) {
		return HEALTH_METRIC_FREE_HEAP;
	}
	if (strcmp(str, "total_heap") == 0) {
		return HEALTH_METRIC_TOTAL_HEAP;
	}

	/* Check custom metrics by name - try to get all metrics and
	 * match by name
	 */
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
	struct health_metric metrics[16];
	size_t count = ARRAY_SIZE(metrics);

	if (health_get_all_metrics(metrics, &count) == 0) {
		for (size_t i = 0; i < count; i++) {
			if (metrics[i].name != NULL && strcmp(metrics[i].name, str) == 0) {
				return metrics[i].type;
			}
		}
	}
#endif

	return HEALTH_METRIC_CUSTOM_START; /* Invalid */
}

/* Helper function to get metric name */
static const char *get_metric_name(enum health_metric_type type)
{
	switch (type) {
	case HEALTH_METRIC_CPU_USAGE:
		return "CPU Usage";
	case HEALTH_METRIC_MEMORY_USAGE:
		return "Memory Usage";
	case HEALTH_METRIC_STACK_USAGE:
		return "Stack Usage";
	case HEALTH_METRIC_THREAD_COUNT:
		return "Thread Count";
	case HEALTH_METRIC_THREADS_HIGH_STACK:
		return "Threads High Stack";
	case HEALTH_METRIC_FREE_HEAP:
		return "Free Heap";
	case HEALTH_METRIC_TOTAL_HEAP:
		return "Total Heap";
	default:
#ifdef CONFIG_HEALTH_MONITOR_CUSTOM_METRICS
		/* Try to get the metric to find its name */
		struct health_metric metric;

		if (health_get_metric(type, &metric) == 0 && metric.name != NULL) {
			return metric.name;
		}
#endif
		return "Unknown";
	}
}

/* Helper function to get status string */
static const char *get_status_string(enum health_status status)
{
	switch (status) {
	case HEALTH_STATUS_OK:
		return "OK";
	case HEALTH_STATUS_WARNING:
		return "WARNING";
	case HEALTH_STATUS_CRITICAL:
		return "CRITICAL";
	default:
		return "UNKNOWN";
	}
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enum health_status overall_status = health_get_status();
	struct health_metric metrics[16];
	size_t count = ARRAY_SIZE(metrics);
	int ret;

	ret = health_get_all_metrics(metrics, &count);
	if (ret != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Failed to get metrics: %d\n", ret);
		return ret;
	}

	shell_fprintf(sh, SHELL_NORMAL, "System Health: %s\n", get_status_string(overall_status));

	for (size_t i = 0; i < count; i++) {
		const char *name = metrics[i].name;

		if (name == NULL) {
			name = get_metric_name(metrics[i].type);
		}

		if (metrics[i].threshold_warning > 0 || metrics[i].threshold_critical > 0) {
			shell_fprintf(sh, SHELL_NORMAL,
				      "  %s: %u (Warning: %u, Critical: %u)\n",
				      name, metrics[i].value,
				      metrics[i].threshold_warning,
				      metrics[i].threshold_critical);
		} else {
			shell_fprintf(sh, SHELL_NORMAL, "  %s: %u\n", name, metrics[i].value);
		}
	}

	return 0;
}

static int cmd_metric(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_fprintf(sh, SHELL_ERROR, "Usage: health metric <name|id>\n");
		return -EINVAL;
	}

	enum health_metric_type type = parse_metric_id(argv[1]);

	/* Validate metric by trying to get it */
	struct health_metric test_metric;

	if (health_get_metric(type, &test_metric) != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Invalid metric: %s\n", argv[1]);
		return -EINVAL;
	}

	struct health_metric metric;
	int ret = health_get_metric(type, &metric);

	if (ret != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Failed to get metric: %d\n", ret);
		return ret;
	}

	const char *name = metric.name;

	if (name == NULL) {
		name = get_metric_name(type);
	}

	shell_fprintf(sh, SHELL_NORMAL, "%s: %u\n", name, metric.value);

	if (metric.threshold_warning > 0 || metric.threshold_critical > 0) {
		shell_fprintf(sh, SHELL_NORMAL,
			      "  Warning Threshold: %u\n", metric.threshold_warning);
		shell_fprintf(sh, SHELL_NORMAL,
			      "  Critical Threshold: %u\n", metric.threshold_critical);
	}

	shell_fprintf(sh, SHELL_NORMAL, "  Last Updated: %llu ticks\n",
		      (unsigned long long)metric.timestamp);

	return 0;
}

#ifdef CONFIG_HEALTH_MONITOR_HISTORY
static int cmd_history(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_fprintf(sh, SHELL_ERROR, "Usage: health history <name|id>\n");
		return -EINVAL;
	}

	enum health_metric_type type = parse_metric_id(argv[1]);

	/* Validate metric by trying to get it */
	struct health_metric test_metric;

	if (health_get_metric(type, &test_metric) != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Invalid metric: %s\n", argv[1]);
		return -EINVAL;
	}

	uint32_t values[CONFIG_HEALTH_MONITOR_HISTORY_SIZE];
	size_t count = ARRAY_SIZE(values);
	int ret = health_get_history(type, values, &count);

	if (ret != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Failed to get history: %d\n", ret);
		return ret;
	}

	if (count == 0) {
		shell_fprintf(sh, SHELL_INFO, "No history available for this metric\n");
		return 0;
	}

	const char *name = get_metric_name(type);

	shell_fprintf(sh, SHELL_NORMAL, "%s History (last %zu samples):\n", name, count);

	for (size_t i = 0; i < count; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "  %u", values[i]);
		if (i < count - 1) {
			shell_fprintf(sh, SHELL_NORMAL, ", ");
		}
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");

	return 0;
}
#endif /* CONFIG_HEALTH_MONITOR_HISTORY */

static int cmd_threshold(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_fprintf(sh, SHELL_ERROR,
			      "Usage: health threshold <name|id> <warning> <critical>\n");
		return -EINVAL;
	}

	enum health_metric_type type = parse_metric_id(argv[1]);

	/* Validate metric by trying to get it */
	struct health_metric test_metric;

	if (health_get_metric(type, &test_metric) != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Invalid metric: %s\n", argv[1]);
		return -EINVAL;
	}

	char *endptr;
	unsigned long warning = strtoul(argv[2], &endptr, 0);

	if (*endptr != '\0' || warning > UINT32_MAX) {
		shell_fprintf(sh, SHELL_ERROR, "Invalid warning threshold: %s\n", argv[2]);
		return -EINVAL;
	}

	unsigned long critical = strtoul(argv[3], &endptr, 0);

	if (*endptr != '\0' || critical > UINT32_MAX) {
		shell_fprintf(sh, SHELL_ERROR, "Invalid critical threshold: %s\n", argv[3]);
		return -EINVAL;
	}

	int ret = health_set_threshold(type, (uint32_t)warning, (uint32_t)critical);

	if (ret != 0) {
		shell_fprintf(sh, SHELL_ERROR, "Failed to set threshold: %d\n", ret);
		return ret;
	}

	const char *name = get_metric_name(type);

	shell_fprintf(sh, SHELL_INFO, "Threshold set for %s: warning=%lu, critical=%lu\n",
		      name, warning, critical);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_health,
	SHELL_CMD(status, NULL,
		  SHELL_HELP("Show overall health status and all metrics", NULL),
		  cmd_status),
	SHELL_CMD(metric, NULL,
		  SHELL_HELP("Show specific metric details", "<name|id>"),
		  cmd_metric),
#ifdef CONFIG_HEALTH_MONITOR_HISTORY
	SHELL_CMD(history, NULL,
		  SHELL_HELP("Show metric history", "<name|id>"),
		  cmd_history),
#endif
	SHELL_CMD(threshold, NULL,
		  SHELL_HELP("Set metric thresholds", "<name|id> <warning> <critical>"),
		  cmd_threshold),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(health, &sub_health, "Health monitor commands", NULL);

