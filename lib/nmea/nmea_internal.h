/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_NMEA_NMEA_INTERNAL_H_
#define ZEPHYR_LIB_NMEA_NMEA_INTERNAL_H_

#include <zephyr/nmea/nmea.h>

/**
 * @brief Parse NMEA time field (HHMMSS.sss) into struct nmea_time
 *
 * @param str  Null-terminated time string
 * @param time Output time struct
 *
 * @retval 0       Success
 * @retval -EINVAL Invalid input
 */
int nmea_parse_time(const char *str, struct nmea_time *time);

/**
 * @brief Parse NMEA date field (DDMMYY) into struct nmea_date
 *
 * @param str  Null-terminated date string
 * @param date Output date struct
 *
 * @retval 0       Success
 * @retval -EINVAL Invalid input
 */
int nmea_parse_date(const char *str, struct nmea_date *date);

/**
 * @brief Convert NMEA DDDMM.MMMM coordinate to degrees * 1e7
 *
 * @param str    Null-terminated coordinate string
 * @param hemi   Hemisphere character: 'N', 'S', 'E', or 'W'
 * @param is_lon True if longitude (3 degree digits), false for latitude (2)
 *
 * @return Signed coordinate in degrees * 1e7
 */
int32_t nmea_coord_to_e7(const char *str, char hemi, bool is_lon);

/**
 * @brief Parse a decimal string to an integer scaled by 100
 *
 * E.g. "1.03" -> 103, "12.5" -> 1250, "0.9" -> 90
 *
 * @param str   Null-terminated decimal string
 * @param out   Output value
 *
 * @retval 0       Success
 * @retval -EINVAL Invalid input
 */
int nmea_parse_decimal_e2(const char *str, int32_t *out);

/**
 * @brief Parse a decimal string to an integer scaled by 1000
 *
 * E.g. "61.7" -> 61700, "545.4" -> 545400
 *
 * @param str   Null-terminated decimal string
 * @param out   Output value
 *
 * @retval 0       Success
 * @retval -EINVAL Invalid input
 */
int nmea_parse_decimal_e3(const char *str, int32_t *out);

/**
 * @brief Parse a positive integer string
 *
 * @param str   Null-terminated digit string
 * @param out   Output value
 *
 * @retval 0       Success
 * @retval -EINVAL Invalid input
 */
int nmea_parse_int(const char *str, int32_t *out);

/* Sentence-specific parsers (conditionally compiled) */

#ifdef CONFIG_NMEA_SENTENCE_GGA
int nmea_parse_gga(char **fields, int nfields, struct nmea_gga *out);
#endif

#ifdef CONFIG_NMEA_SENTENCE_RMC
int nmea_parse_rmc(char **fields, int nfields, struct nmea_rmc *out);
#endif

#ifdef CONFIG_NMEA_SENTENCE_GSA
int nmea_parse_gsa(char **fields, int nfields, struct nmea_gsa *out);
#endif

#ifdef CONFIG_NMEA_SENTENCE_GSV
int nmea_parse_gsv(char **fields, int nfields, struct nmea_gsv *out);
#endif

#ifdef CONFIG_NMEA_SENTENCE_VTG
int nmea_parse_vtg(char **fields, int nfields, struct nmea_vtg *out);
#endif

#ifdef CONFIG_NMEA_SENTENCE_GLL
int nmea_parse_gll(char **fields, int nfields, struct nmea_gll *out);
#endif

#endif /* ZEPHYR_LIB_NMEA_NMEA_INTERNAL_H_ */
