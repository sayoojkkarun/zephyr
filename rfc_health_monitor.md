# RFC: System Health Monitor Subsystem

## Abstract
This RFC proposes a new subsystem `subsys/health` that provides a lightweight framework for monitoring system health metrics (CPU usage, memory usage, stack usage, thread states) and reporting them via a unified API. It complements the existing POST subsystem by providing continuous runtime monitoring rather than one-time boot tests, enabling proactive system health management in embedded applications.

## Problem Description
Currently, Zephyr provides several mechanisms for system monitoring, but they lack integration and continuous operation:

- `subsys/debug/thread_analyzer`: Provides thread analysis but requires manual invocation via shell commands. It's designed for debugging, not continuous monitoring.
- `subsys/stats`: Provides basic statistics collection but has limited scope and no health status aggregation.
- `subsys/post`: Excellent for boot-time self-tests but doesn't address runtime health monitoring.

Real-world embedded systems need:
1. **Continuous Monitoring**: Track CPU load, memory usage, stack usage over time without manual intervention.
2. **Health Status Aggregation**: Combine multiple metrics into a single "health score" that applications can query.
3. **Threshold-Based Alerting**: Automatically notify when metrics exceed configurable thresholds.
4. **Historical Tracking**: Maintain recent history of health metrics for trend analysis and debugging.
5. **Integration Points**: Easy integration with logging, shell commands, and management protocols (e.g., MCUboot, device management).

Developers currently implement ad-hoc solutions:
- Custom periodic tasks that query thread analyzer
- Manual memory usage calculations
- Application-specific health monitoring code
- No standardized way to report system health to management layers

This leads to code duplication, inconsistent behavior, and missed opportunities for proactive system management.

## Proposed Change
Introduce `subsys/health` which provides:

1. **Health Metrics Collection**: Automatic, periodic collection of CPU usage, memory usage, stack usage, and thread states.
2. **Health Status API**: Unified API to query overall system health status and individual metrics.
3. **Threshold Management**: Configurable warning and critical thresholds per metric with callback support.
4. **Health Reporting**: Integration with logging, shell commands, and optional management protocol backends.
5. **Lightweight Design**: Minimal overhead suitable for resource-constrained embedded systems.

### Architecture
High-level view:
```
[ Application / Management Layer ]
       |
       v
[ Health Monitor Subsystem (subsys/health) ]
       |
       +---> [ Metrics Collector (Periodic Timer) ]
       |         |
       |         +---> CPU Usage Monitor
       |         +---> Memory Usage Monitor
       |         +---> Stack Usage Monitor
       |         +---> Thread State Monitor
       |
       +---> [ Threshold Manager ]
       |         |
       |         +---> Warning Thresholds
       |         +---> Critical Thresholds
       |         +---> Callback Invocation
       |
       +---> [ Health Status Aggregator ]
                 |
                 +---> Overall Health Score
                 +---> Metric History (optional)
```

### API Proposal

#### 1. Health Metric Types
```c
/**
 * @brief Health metric types
 */
enum health_metric_type {
    /** CPU usage percentage (0-100) */
    HEALTH_METRIC_CPU_USAGE,
    /** Memory usage percentage (0-100) */
    HEALTH_METRIC_MEMORY_USAGE,
    /** Average stack usage percentage across all threads (0-100) */
    HEALTH_METRIC_STACK_USAGE,
    /** Number of active threads */
    HEALTH_METRIC_THREAD_COUNT,
    /** Number of threads with high stack usage (>80%) */
    HEALTH_METRIC_THREADS_HIGH_STACK,
    /** Free heap memory in bytes */
    HEALTH_METRIC_FREE_HEAP,
    /** Total heap size in bytes */
    HEALTH_METRIC_TOTAL_HEAP,
    /** User-defined metrics start from this value */
    HEALTH_METRIC_CUSTOM_START = 100,
};
```

#### 2. Health Status
```c
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
```

#### 3. Health Metric Structure
```c
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
```

#### 3a. User-Defined Metric Callback
```c
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
```

#### 4. Core API
```c
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
                               void (*cb)(enum health_metric_type type,
                                         uint32_t value,
                                         enum health_status status,
                                         void *user_data),
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
```

#### 5. Optional: Metric History API
```c
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
```

### Configuration

#### Kconfig Options
```kconfig
# Health Monitor Subsystem
config HEALTH_MONITOR
    bool "Enable Health Monitor subsystem"
    depends on STATS || THREAD_ANALYZER
    help
      Enable the system health monitoring subsystem that provides
      continuous monitoring of CPU usage, memory usage, stack usage,
      and thread states.

if HEALTH_MONITOR

config HEALTH_MONITOR_UPDATE_INTERVAL_MS
    int "Health monitor update interval (ms)"
    default 1000
    range 100 60000
    help
      Interval at which health metrics are collected and updated.

config HEALTH_MONITOR_CPU
    bool "Enable CPU usage monitoring"
    default y
    depends on STATS
    help
      Enable monitoring of CPU usage percentage.

config HEALTH_MONITOR_MEMORY
    bool "Enable memory usage monitoring"
    default y
    help
      Enable monitoring of memory usage (heap).

config HEALTH_MONITOR_STACK
    bool "Enable stack usage monitoring"
    default y
    depends on THREAD_ANALYZER
    help
      Enable monitoring of thread stack usage.

config HEALTH_MONITOR_THREADS
    bool "Enable thread state monitoring"
    default y
    help
      Enable monitoring of thread count and states.

config HEALTH_MONITOR_CUSTOM_METRICS
    bool "Enable user-defined custom metrics"
    default y
    help
      Allow applications to register custom health metrics via
      health_register_custom_metric() API.

config HEALTH_MONITOR_MAX_CUSTOM_METRICS
    int "Maximum number of custom metrics"
    default 8
    depends on HEALTH_MONITOR_CUSTOM_METRICS
    range 1 32
    help
      Maximum number of user-defined custom metrics that can be
      registered simultaneously.

config HEALTH_MONITOR_HISTORY
    bool "Enable metric history tracking"
    default n
    help
      Maintain a history of metric values for trend analysis.
      Increases memory usage.

config HEALTH_MONITOR_HISTORY_SIZE
    int "Number of history samples per metric"
    default 10
    depends on HEALTH_MONITOR_HISTORY
    range 5 100
    help
      Number of historical samples to maintain per metric.

config HEALTH_MONITOR_SHELL
    bool "Enable health monitor shell commands"
    default y
    depends on SHELL
    help
      Enable shell commands for querying health status and metrics.

config HEALTH_MONITOR_DEFAULT_THRESHOLDS
    bool "Use default thresholds"
    default y
    help
      Use sensible default thresholds for common metrics.
      Can be overridden at runtime via API.

config HEALTH_MONITOR_CPU_WARNING_THRESHOLD
    int "CPU usage warning threshold (%)"
    default 80
    depends on HEALTH_MONITOR_CPU && HEALTH_MONITOR_DEFAULT_THRESHOLDS
    range 0 100

config HEALTH_MONITOR_CPU_CRITICAL_THRESHOLD
    int "CPU usage critical threshold (%)"
    default 95
    depends on HEALTH_MONITOR_CPU && HEALTH_MONITOR_DEFAULT_THRESHOLDS
    range 0 100

config HEALTH_MONITOR_MEMORY_WARNING_THRESHOLD
    int "Memory usage warning threshold (%)"
    default 80
    depends on HEALTH_MONITOR_MEMORY && HEALTH_MONITOR_DEFAULT_THRESHOLDS
    range 0 100

config HEALTH_MONITOR_MEMORY_CRITICAL_THRESHOLD
    int "Memory usage critical threshold (%)"
    default 95
    depends on HEALTH_MONITOR_MEMORY && HEALTH_MONITOR_DEFAULT_THRESHOLDS
    range 0 100

config HEALTH_MONITOR_STACK_WARNING_THRESHOLD
    int "Stack usage warning threshold (%)"
    default 80
    depends on HEALTH_MONITOR_STACK && HEALTH_MONITOR_DEFAULT_THRESHOLDS
    range 0 100

config HEALTH_MONITOR_STACK_CRITICAL_THRESHOLD
    int "Stack usage critical threshold (%)"
    default 95
    depends on HEALTH_MONITOR_STACK && HEALTH_MONITOR_DEFAULT_THRESHOLDS
    range 0 100

endif # HEALTH_MONITOR
```

### Shell Commands (Optional)
If `CONFIG_HEALTH_MONITOR_SHELL` is enabled:
```
uart:~$ health status
System Health: OK
  CPU Usage: 45% (Warning: 80%, Critical: 95%)
  Memory Usage: 62% (Warning: 80%, Critical: 95%)
  Stack Usage: 35% (Warning: 80%, Critical: 95%)
  Thread Count: 8
  Battery Level: 78% (Warning: 20%, Critical: 10%)
  Network Latency: 45ms (Warning: 100ms, Critical: 200ms)

uart:~$ health metric cpu
CPU Usage: 45%
  Warning Threshold: 80%
  Critical Threshold: 95%
  Last Updated: 12345 ticks

uart:~$ health metric "Battery Level"
Battery Level: 78%
  Warning Threshold: 20%
  Critical Threshold: 10%
  Last Updated: 12350 ticks

uart:~$ health history cpu
CPU Usage History (last 10 samples):
  42%, 44%, 45%, 43%, 45%, 46%, 44%, 45%, 43%, 45%
```

### Implementation Details

#### Metrics Collection
- Uses a kernel timer (`k_timer`) to periodically collect metrics
- Collection runs in timer ISR context (must be fast)
- Heavy computations (e.g., thread analysis) can be deferred to a work queue
- Metrics are stored in a simple array indexed by metric type

#### CPU Usage Calculation
- Uses `CONFIG_STATS` if available for accurate CPU usage
- Falls back to idle thread analysis if stats not available
- Calculated as: `(1 - idle_time / total_time) * 100`

#### Memory Usage Calculation
- Uses kernel heap statistics (`k_heap_stats_get()` or similar)
- Calculated as: `(total_heap - free_heap) / total_heap * 100`

#### Stack Usage Calculation
- Uses thread analyzer API (`thread_analyzer_get()`)
- Calculates average stack usage across all threads
- Also tracks number of threads with high stack usage

#### Custom Metrics
- Custom metrics are stored in a separate array/dictionary structure
- Each custom metric has:
  - Unique ID (assigned sequentially starting from `HEALTH_METRIC_CUSTOM_START`)
  - Name string (for display and lookup)
  - Collection function pointer
  - User data pointer
  - Thresholds (warning and critical)
- During periodic collection, all registered custom metric functions are called
- Custom metrics are included in overall health status calculation
- Custom metrics can be queried by ID or by name (via shell/API)

#### Health Status Aggregation
- Overall status is the worst status across all metrics (including custom metrics)
- `HEALTH_STATUS_CRITICAL` > `HEALTH_STATUS_WARNING` > `HEALTH_STATUS_OK`
- All metrics (built-in and custom) contribute equally to overall status

## Alternatives

### Status Quo
Continue using manual thread analyzer invocations and custom monitoring code. This leads to:
- Code duplication across projects
- Inconsistent monitoring approaches
- No standardized health reporting
- Missed opportunities for proactive management

### Extend Existing Subsystems
- **Extend `subsys/stats`**: Stats is designed for generic statistics, not health monitoring. Adding health concepts would bloat it.
- **Extend `subsys/debug/thread_analyzer`**: Thread analyzer is debug-focused and requires manual invocation. Making it continuous would change its purpose.
- **Extend `subsys/post`**: POST is for boot-time tests, not continuous monitoring. Different use case.

### Application-Level Implementation
Each application implements its own health monitoring. This is what developers do today, but it:
- Duplicates effort
- Lacks standardization
- Makes integration with management layers difficult

## Dependencies
- **Required**: Either `CONFIG_STATS` (for CPU usage) or `CONFIG_THREAD_ANALYZER` (for stack monitoring)
- **Optional**: `CONFIG_SHELL` for health monitor shell commands
- **Optional**: `CONFIG_LOG` for health status logging
- **Optional**: `CONFIG_HEALTH_MONITOR_CUSTOM_METRICS` for user-defined metrics support
- No external dependencies

## Example Usage

### Custom Metric Example
```c
#include <zephyr/health/health.h>

/* Example: Battery level monitoring */
struct battery_data {
    uint8_t current_level;  /* 0-100 */
};

static uint32_t battery_collect_fn(void *user_data)
{
    struct battery_data *bat = (struct battery_data *)user_data;
    
    /* Read battery level from ADC or fuel gauge */
    bat->current_level = read_battery_level();
    
    return bat->current_level;  /* Return 0-100 */
}

static void battery_warning_cb(enum health_metric_type type,
                               uint32_t value,
                               enum health_status status,
                               void *user_data)
{
    LOG_WRN("Battery level critical: %d%%", value);
    /* Trigger low battery actions */
}

void init_battery_monitoring(void)
{
    static struct battery_data bat_data = {0};
    uint32_t metric_id;
    
    /* Register battery level as a custom metric */
    metric_id = health_register_custom_metric(
        "Battery Level",
        battery_collect_fn,
        20,  /* Warning at 20% */
        10,  /* Critical at 10% */
        &bat_data
    );
    
    if (metric_id < 0) {
        LOG_ERR("Failed to register battery metric");
        return;
    }
    
    /* Register callback for battery warnings */
    health_register_callback(
        metric_id,
        HEALTH_STATUS_CRITICAL,
        battery_warning_cb,
        NULL
    );
}

/* Example: Network latency monitoring */
static uint32_t network_latency_collect_fn(void *user_data)
{
    /* Measure network latency */
    return measure_ping_latency_ms();  /* Return latency in milliseconds */
}

void init_network_monitoring(void)
{
    uint32_t metric_id;
    
    metric_id = health_register_custom_metric(
        "Network Latency",
        network_latency_collect_fn,
        100,  /* Warning at 100ms */
        200,  /* Critical at 200ms */
        NULL
    );
    
    if (metric_id < 0) {
        LOG_ERR("Failed to register network latency metric");
    }
}
```

### Querying Custom Metrics
```c
/* Get custom metric by ID */
struct health_metric metric;
int ret = health_get_metric(custom_metric_id, &metric);
if (ret == 0) {
    printk("Metric: %s = %u\n", metric.name, metric.value);
}

/* Custom metrics are included in overall health status */
enum health_status status = health_get_status();
if (status == HEALTH_STATUS_CRITICAL) {
    /* Check which metrics are causing the issue */
    struct health_metric metrics[16];
    size_t count = 16;
    health_get_all_metrics(metrics, &count);
    
    for (size_t i = 0; i < count; i++) {
        if (metrics[i].value >= metrics[i].threshold_critical) {
            LOG_ERR("Critical: %s = %u", metrics[i].name, metrics[i].value);
        }
    }
}
```

## Testing

### Unit Tests
- Test metric collection for each metric type
- Test threshold violation detection
- Test callback invocation
- Test health status aggregation logic
- Test metric history (if enabled)
- Test custom metric registration/unregistration
- Test custom metric collection function invocation
- Test custom metrics included in health status calculation

### Integration Tests
- Test with `CONFIG_STATS` enabled/disabled
- Test with `CONFIG_THREAD_ANALYZER` enabled/disabled
- Test shell commands (if enabled)
- Test callback registration/unregistration
- Test threshold configuration
- Test custom metrics with various collection functions
- Test custom metric name lookup
- Test maximum custom metrics limit

### Sample Application
Create a sample application (`samples/subsys/health`) that:
- Demonstrates health monitor initialization
- Shows threshold configuration
- Demonstrates callback usage
- Shows shell command usage
- Displays health metrics periodically
- Includes examples of custom metric registration (e.g., simulated battery level, network latency)
- Demonstrates custom metric integration with overall health status

## Future Enhancements (Out of Scope)
These are potential future enhancements but not part of the initial RFC:
- Integration with MCUboot for bootloader health reporting
- Integration with device management protocols (LwM2M, CoAP, etc.)
- Machine learning-based anomaly detection
- Health metrics export to external monitoring systems
- Per-thread health monitoring
- Network stack health metrics

## References
- Zephyr `subsys/stats` documentation
- Zephyr `subsys/debug/thread_analyzer` documentation
- Zephyr `subsys/post` documentation (complementary subsystem)
- Zephyr `subsys/task_wdt` (related: system monitoring)

