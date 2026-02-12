.. zephyr:code-sample:: health
   :name: Health Monitor
   :relevant-api: health_api

   Monitor system health using the Health Monitor subsystem.

Overview
********

This sample demonstrates the :ref:`Health Monitor <health_api>` subsystem
which provides continuous runtime monitoring of system health metrics
including CPU usage, memory usage, stack usage, and thread states. The
sample also shows how to register and monitor custom application-specific
metrics.

Building and Running
********************

The sample can be built and executed for any board that supports the
required dependencies (STATS and/or THREAD_ANALYZER).

Building and Running for QEMU x86
==================================
The sample can be built and executed for QEMU x86 as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/health
   :board: qemu_x86
   :goals: build run
   :compact:

For other boards just replace the board name.

Sample Output
=============

The following output is printed during initialization:

.. code-block:: console

   Health Monitor Sample Application
   ==================================

   Health monitor initialized
   Registered battery metric (ID: 100)
   Registered network latency metric (ID: 101)
   Registered battery threshold callbacks
   Registered latency threshold callbacks

   Available shell commands:
     health status       - Show overall health status
     health metric <id>  - Show specific metric
     health threshold <id> <w> <c> - Set thresholds

   === Health Status ===
   Overall Status: OK
   Metrics:
     CPU Usage: 5 (W:80, C:95)
     Memory Usage: 45 (W:80, C:95)
     Stack Usage: 35 (W:80, C:95)
     Thread Count: 3
     Battery Level: 99 (W:20, C:10)
     Network Latency: 87 (W:100, C:200)
   Use 'health status' shell command for more details
   ==================

Shell Commands
**************

The sample enables the health monitor shell module, providing the following
commands:

* ``health status`` - Display overall health status and all metrics
* ``health metric <name|id>`` - Show details for a specific metric
* ``health threshold <name|id> <warning> <critical>`` - Set thresholds for a metric
* ``health history <name|id>`` - Show metric history (if enabled)

Example shell session:

.. code-block:: console

   uart:~$ health status
   System Health: OK
     CPU Usage: 5 (Warning: 80, Critical: 95)
     Memory Usage: 45 (Warning: 80, Critical: 95)
     Stack Usage: 35 (Warning: 80, Critical: 95)
     Thread Count: 3
     Battery Level: 99 (Warning: 20, Critical: 10)
     Network Latency: 87 (Warning: 100, Critical: 200)

   uart:~$ health metric battery
   Battery Level: 99
     Warning Threshold: 20
     Critical Threshold: 10
     Last Updated: 12345 ticks

   uart:~$ health metric 0
   CPU Usage: 5
     Warning Threshold: 80
     Critical Threshold: 95
     Last Updated: 12350 ticks

Custom Metrics
**************

The sample demonstrates registering custom metrics:

1. **Battery Level** - Simulates a battery that discharges and charges
   cyclically, with thresholds at 20% (warning) and 10% (critical).

2. **Network Latency** - Simulates network latency that varies between
   50-150ms, with thresholds at 100ms (warning) and 200ms (critical).

Both custom metrics register threshold callbacks that print warnings when
thresholds are exceeded.

You can register your own custom metrics by:

1. Creating a collection function that returns the current metric value
2. Calling ``health_register_custom_metric()`` with the function and thresholds
3. Optionally registering callbacks for threshold violations

See the sample source code for complete examples.

