/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Timeout Queue Benchmark
 * Measures the performance of the kernel timeout queue backend
 * across three phases:
 * INSERT: N k_timer_start() calls with far future deadlines in increasing order.
 * ABORT: N/2 k_timer_stop() calls on live timers scattered through the queue
 * FIRE: N timers with staggered 10 ms deadlines. per-callback latency
 *       (actual expiry tick - expected expiry tick) is recorded and summarised
 *       as min/max/avg
 */

#include <zephyr/kernel.h>
#include <stdio.h>

#define N CONFIG_BENCH_NUM_TIMERS

#define INSERT_DEADLINE_SPACING_TICKS	1000
#define FIRE_SPACING_MS	10

static inline uint32_t bench_now(void)
{
	return k_cycle_get_32();
}

static inline uint64_t bench_elapsed(uint32_t start, uint32_t end)
{
	return (uint64_t)(end - start);
}

static inline uint64_t bench_to_ns(uint64_t cycles)
{
	uint32_t c32 = (cycles > UINT32_MAX) ? UINT32_MAX : (uint32_t)cycles;

	return (uint64_t)k_cyc_to_ns_near32(c32);
}

static struct k_timer timers[N];
static int64_t fire_expected_ticks[N];
static int64_t fire_actual_ticks[N];
static atomic_t fires_done;

static void fire_expiry(struct k_timer *t)
{
	int idx = (int)(uintptr_t)k_timer_user_data_get(t);

	fire_actual_ticks[idx] = k_uptime_ticks();
	atomic_inc(&fires_done);
}

static uint64_t bench_insert(void)
{
	int64_t base = k_uptime_ticks() + INSERT_DEADLINE_SPACING_TICKS;
	uint32_t t0, t1;

	t0 = bench_now();
	for (int i = 0; i < N; i++) {
		k_timer_start(&timers[i],
			K_TIMEOUT_ABS_TICKS(base + (int64_t)i * INSERT_DEADLINE_SPACING_TICKS),
			K_NO_WAIT);
	}
	t1 = bench_now();

	for (int i = 0; i < N; i++) {
		k_timer_stop(&timers[i]);
	}

	return bench_elapsed(t0, t1);
}

static uint64_t bench_abort(void)
{
	int64_t base = k_uptime_ticks() + INSERT_DEADLINE_SPACING_TICKS;
	uint32_t t0, t1;

	for (int i = 0; i < N; i++) {
		k_timer_start(&timers[i],
			K_TIMEOUT_ABS_TICKS(base + (int64_t)i * INSERT_DEADLINE_SPACING_TICKS),
			K_NO_WAIT);
	}

	t0 = bench_now();
	for (int i = 0; i < N; i += 2) {
		k_timer_stop(&timers[i]);
	}
	t1 = bench_now();

	for (int i = 1; i < N; i += 2) {
		k_timer_stop(&timers[i]);
	}

	return bench_elapsed(t0, t1);
}

static void bench_fire(int64_t *out_min, int64_t *out_max, int64_t *out_avg)
{
	int64_t base = k_uptime_ticks() + k_ms_to_ticks_ceil32(50);

	atomic_set(&fires_done, 0);
	for (int i = 0; i < N; i++) {
		fire_expected_ticks[i] = base
			+ (int64_t)i * k_ms_to_ticks_ceil32(FIRE_SPACING_MS);
		k_timer_user_data_set(&timers[i], (void *)(uintptr_t)i);
		k_timer_start(&timers[i],
			K_TIMEOUT_ABS_TICKS(fire_expected_ticks[i]),
			K_NO_WAIT);
	}

	while (atomic_get(&fires_done) < N) {
		k_yield();
	}

	int64_t min = INT64_MAX;
	int64_t max = -INT64_MAX;
	int64_t sum = 0;

	for (int i = 0; i < N; i++) {
		int64_t delta = fire_actual_ticks[i] - fire_expected_ticks[i];

		if (delta < min) {
			min = delta;
		}
		if (delta > max) {
			max = delta;
		}
		sum += delta;
	}

	*out_min = min;
	*out_max = max;
	*out_avg = sum / N;
}

int main(void)
{
	for (int i = 0; i < N; i++) {
		k_timer_init(&timers[i], fire_expiry, NULL);
	}

	printf("Timeout Queue Benchmark\n");
	printf("Backend : %s\n",
		IS_ENABLED(CONFIG_TIMEOUT_USE_MIN_HEAP) ? "MIN_HEAP" : "DLIST");
	printf("N	: %d timers\n", N);
	printf("Tick Hz : %d\n", CONFIG_SYS_CLOCK_TICKS_PER_SEC);
	printf("\n");

	/* INSERT */
	uint64_t ins_cyc = bench_insert();
	uint64_t ins_ns = bench_to_ns(ins_cyc);

	printf("INSERT %3d ops : %10llu cycles | %7llu ns/op\n", N, (unsigned long long)ins_cyc,
		(unsigned long long)(ins_ns / N));

	/* ABORT */
	uint64_t abt_cyc = bench_abort();
	uint64_t abt_ns = bench_to_ns(abt_cyc);

	printf("ABORT %3d ops : %10llu cycles | %7llu ns/op\n", N / 2,
		(unsigned long long)abt_cyc, (unsigned long long)(abt_ns / (N / 2)));

	/* FIRE */
	int64_t min, max, avg;

	bench_fire(&min, &max, &avg);
	printf("FIRE : %3d ops : min_late=%lld ticks | max_late=%lld ticks"
		" | avg_late=%lld ticks\n", N, (long long)min, (long long)max, (long long)avg);

	printf("\n BENCHMARK DONE\n");

	return 0;
}
