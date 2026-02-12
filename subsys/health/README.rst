.. _health_monitor:

Health Monitor Subsystem
#########################

Overview
********

The Health Monitor subsystem provides continuous runtime monitoring of system
health metrics including CPU usage, memory usage, stack usage, and thread states.
It complements the existing POST subsystem by providing continuous runtime
monitoring rather than one-time boot tests, enabling proactive system health
management in embedded applications.

The subsystem supports:

* **Built-in Metrics**: CPU usage, memory usage, stack usage, thread count,
  and heap statistics
* **Custom Metrics**: User-defined application-specific metrics
* **Threshold Management**: Configurable warning and critical thresholds per metric
* **Callback Support**: Automatic notification when thresholds are exceeded
* **Health Status Aggregation**: Overall system health status based on all metrics
* **Shell Integration**: Interactive commands for querying health status
* **History Tracking**: Optional historical data for trend analysis

Features
********

Built-in Metrics
================

The following built-in metrics are available:

* **CPU Usage**: Percentage of CPU utilization (0-100%)
* **Memory Usage**: Percentage of heap memory used (0-100%)
* **Stack Usage**: Average stack usage percentage across all threads (0-100%)
* **Thread Count**: Number of active threads
* **Threads High Stack**: Number of threads with stack usage > 80%
* **Free Heap**: Available heap memory in bytes
* **Total Heap**: Total heap size in bytes

Custom Metrics
==============

Applications can register custom metrics using
:c:func:`health_register_custom_metric`. Custom metrics are assigned IDs
starting from :c:macro:`HEALTH_METRIC_CUSTOM_START` (100). Each custom metric
requires:

* A human-readable name
* A collection function that returns the current metric value
* Warning and critical thresholds
* Optional user data

Example:

.. code-block:: c

   static uint32_t battery_level = 100;

   static uint32_t battery_collect(void *user_data)
   {
       return battery_level;
   }

   int metric_id = health_register_custom_metric("Battery Level",
                                                   battery_collect,
                                                   20,  /* warning at 20% */
                                                   10,  /* critical at 10% */
                                                   NULL);
   if (metric_id >= 0) {
       printk("Registered battery metric (ID: %d)\n", metric_id);
   }

Threshold Management
====================

Each metric can have configurable warning and critical thresholds. When a
metric exceeds its threshold, callbacks can be registered to receive
notifications:

.. code-block:: c

   static void cpu_warning_cb(enum health_metric_type type,
                              uint32_t value,
                              enum health_status status,
                              void *user_data)
   {
       printk("CPU usage is %u%% - status: %d\n", value, status);
   }

   health_register_callback(HEALTH_METRIC_CPU_USAGE,
                           HEALTH_STATUS_WARNING,
                           cpu_warning_cb,
                           NULL);

The subsystem includes debouncing to prevent excessive callback invocations
when metrics fluctuate around threshold values.

Health Status
=============

The overall system health status is determined by aggregating all monitored
metrics:

* **OK**: All metrics are within normal ranges
* **WARNING**: One or more metrics exceed warning thresholds
* **CRITICAL**: One or more metrics exceed critical thresholds

The status can be queried using :c:func:`health_get_status`.

Configuration
*************

The subsystem is configured via Kconfig options:

* :kconfig:option:`CONFIG_HEALTH_MONITOR`: Enable the health monitor subsystem
* :kconfig:option:`CONFIG_HEALTH_MONITOR_UPDATE_INTERVAL_MS`: Update interval
  in milliseconds (default: 60000ms)
* :kconfig:option:`CONFIG_HEALTH_MONITOR_CPU`: Enable CPU usage monitoring
* :kconfig:option:`CONFIG_HEALTH_MONITOR_MEMORY`: Enable memory usage monitoring
* :kconfig:option:`CONFIG_HEALTH_MONITOR_STACK`: Enable stack usage monitoring
* :kconfig:option:`CONFIG_HEALTH_MONITOR_THREADS`: Enable thread state monitoring
* :kconfig:option:`CONFIG_HEALTH_MONITOR_CUSTOM_METRICS`: Enable custom metrics
* :kconfig:option:`CONFIG_HEALTH_MONITOR_MAX_CUSTOM_METRICS`: Maximum number
  of custom metrics (default: 8)
* :kconfig:option:`CONFIG_HEALTH_MONITOR_SHELL`: Enable shell commands
* :kconfig:option:`CONFIG_HEALTH_MONITOR_HISTORY`: Enable metric history tracking

Dependencies
************

The health monitor subsystem requires:

* :kconfig:option:`CONFIG_STATS` or :kconfig:option:`CONFIG_THREAD_ANALYZER`
  for basic functionality
* :kconfig:option:`CONFIG_CPU_LOAD` for CPU usage monitoring
* :kconfig:option:`CONFIG_SYS_HEAP_RUNTIME_STATS` for accurate memory monitoring
* :kconfig:option:`CONFIG_SHELL` for shell command support

Usage
*****

Initialization
==============

Initialize the health monitor during system startup:

.. code-block:: c

   #include <zephyr/health/health.h>

   void main(void)
   {
       int ret = health_monitor_init();
       if (ret != 0) {
           printk("Failed to initialize health monitor: %d\n", ret);
           return;
       }

       /* Health monitor is now running and collecting metrics */
   }

Querying Health Status
======================

Query the overall system health status:

.. code-block:: c

   enum health_status status = health_get_status();
   switch (status) {
   case HEALTH_STATUS_OK:
       printk("System health: OK\n");
       break;
   case HEALTH_STATUS_WARNING:
       printk("System health: WARNING\n");
       break;
   case HEALTH_STATUS_CRITICAL:
       printk("System health: CRITICAL\n");
       break;
   }

Getting Metric Values
=====================

Retrieve a specific metric:

.. code-block:: c

   struct health_metric metric;
   int ret = health_get_metric(HEALTH_METRIC_CPU_USAGE, &metric);
   if (ret == 0) {
       printk("CPU Usage: %u%% (W:%u, C:%u)\n",
              metric.value,
              metric.threshold_warning,
              metric.threshold_critical);
   }

Setting Thresholds
==================

Configure thresholds for a metric:

.. code-block:: c

   health_set_threshold(HEALTH_METRIC_CPU_USAGE, 80, 95);
   /* Warning at 80%, Critical at 95% */

Timer Control
==============

Control the health monitor collection timer:

.. code-block:: c

   /* Stop collection */
   health_monitor_stop();

   /* Start collection */
   health_monitor_start();

   /* Change update interval */
   health_monitor_set_interval(30000); /* 30 seconds */

Shell Commands
**************

When :kconfig:option:`CONFIG_HEALTH_MONITOR_SHELL` is enabled, the following
shell commands are available:

* ``health status`` - Display overall health status and all metrics
* ``health metric <id>`` - Display details for a specific metric
* ``health threshold <id> <warning> <critical>`` - Set thresholds for a metric
* ``health start`` - Start health monitor collection
* ``health stop`` - Stop health monitor collection
* ``health interval`` - Display current update interval
* ``health interval <ms>`` - Set update interval in milliseconds

Example shell session:

.. code-block:: console

   uart:~$ health status
   === Health Status ===
   Overall Status: OK
   Metrics:
     CPU Usage: 2% (W:80, C:95)
     Memory Usage: 22% (W:80, C:95)
     Stack Usage: 37% (W:80, C:95)
     Thread Count: 4
     Free Heap: 6392 bytes
     Total Heap: 8208 bytes
     Battery Level: 75% (W:20, C:10)
     Network Latency: 130 ms (W:100, C:200)

   uart:~$ health metric 0
   CPU Usage: 2%
     Warning Threshold: 80%
     Critical Threshold: 95%
     Last Updated: 401620 ticks

API Reference
*************

.. doxygengroup:: health_api

