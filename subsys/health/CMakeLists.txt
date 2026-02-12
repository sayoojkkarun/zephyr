# Copyright (c) 2026 Aerlync Labs Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_HEALTH_MONITOR health.c)
zephyr_sources_ifdef(CONFIG_HEALTH_MONITOR_SHELL health_shell.c)

