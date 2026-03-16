# Copyright (c) 2026 Aerlync Labs Inc.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(nmea.c)

zephyr_sources_ifdef(CONFIG_NMEA_SENTENCE_GGA sentences/gga.c)
zephyr_sources_ifdef(CONFIG_NMEA_SENTENCE_RMC sentences/rmc.c)
zephyr_sources_ifdef(CONFIG_NMEA_SENTENCE_GSA sentences/gsa.c)
zephyr_sources_ifdef(CONFIG_NMEA_SENTENCE_GSV sentences/gsv.c)
zephyr_sources_ifdef(CONFIG_NMEA_SENTENCE_VTG sentences/vtg.c)
zephyr_sources_ifdef(CONFIG_NMEA_SENTENCE_GLL sentences/gll.c)
