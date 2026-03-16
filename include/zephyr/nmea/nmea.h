/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief NMEA 0183 sentence parser public API
 *
 */

#ifndef ZEPHYR_INCLUDE_NMEA_NMEA_H_
#define ZEPHYR_INCLUDE_NMEA_NMEA_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nmea_api NMEA 0183 Parser
 * @ingroup utilities
 * @{
 */

/** Maximum length of an NMEA 0183 sentence including '$' and CR/LF */
#define NMEA_MAX_SENTENCE_LEN 82

/** Maximum number of comma-separated fields in a sentence */
#define NMEA_MAX_FIELDS 25

/** Sentence type identifiers */
enum nmea_sentence_type {
	NMEA_SENTENCE_UNKNOWN = 0,
	NMEA_SENTENCE_GGA,
	NMEA_SENTENCE_RMC,
	NMEA_SENTENCE_GSA,
	NMEA_SENTENCE_GSV,
	NMEA_SENTENCE_VTG,
	NMEA_SENTENCE_GLL,
};

/** UTC time parsed from NMEA time fields */
struct nmea_time {
	/** Hours, 0-23 */
	uint8_t hours;
	/** Minutes, 0-59 */
	uint8_t minutes;
	/** Seconds, 0-60 (60 for leap second) */
	uint8_t seconds;
	/** Milliseconds, 0-999 */
	uint16_t milliseconds;
};

/** UTC date parsed from NMEA date fields */
struct nmea_date {
	/** Day of month, 1-31 */
	uint8_t day;
	/** Month, 1-12 */
	uint8_t month;
	/** Full year, e.g. 2026 */
	uint16_t year;
};

/**
 * @brief GGA — Global Positioning System Fix Data
 *
 * Contains position (latitude, longitude, altitude), fix quality,
 * satellite count, and horizontal dilution of precision.
 */
struct nmea_gga {
	/** UTC time of fix */
	struct nmea_time time;
	/** Latitude in degrees * 1e7, negative for south */
	int32_t latitude;
	/** Longitude in degrees * 1e7, negative for west */
	int32_t longitude;
	/** Fix quality: 0=none, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float */
	uint8_t fix_quality;
	/** Number of satellites used in fix, 0-12+ */
	uint8_t satellites_used;
	/** Horizontal dilution of precision * 100 (e.g. 120 = 1.20) */
	uint16_t hdop;
	/** Altitude above MSL in millimetres */
	int32_t altitude_mm;
	/** Geoid separation in millimetres */
	int32_t geoid_sep_mm;
};

/**
 * @brief RMC — Recommended Minimum Specific GNSS Data
 *
 * Contains position, speed, course, and date.
 */
struct nmea_rmc {
	/** UTC time of fix */
	struct nmea_time time;
	/** UTC date */
	struct nmea_date date;
	/** True if status is 'A' (active/valid) */
	bool valid;
	/** Latitude in degrees * 1e7, negative for south */
	int32_t latitude;
	/** Longitude in degrees * 1e7, negative for west */
	int32_t longitude;
	/** Speed over ground in knots * 100 (e.g. 1520 = 15.20 kn) */
	int32_t speed_knots_e2;
	/** Course over ground in degrees * 100 (e.g. 8440 = 84.40 deg) */
	int32_t course_e2;
};

/**
 * @brief GSA — GNSS DOP and Active Satellites
 */
struct nmea_gsa {
	/** Selection mode: 'A' = auto, 'M' = manual */
	uint8_t mode;
	/** Fix type: 1 = no fix, 2 = 2D, 3 = 3D */
	uint8_t fix_type;
	/** PRN of satellites used (0 if slot unused) */
	uint8_t sv_ids[12];
	/** Position DOP * 100 */
	uint16_t pdop;
	/** Horizontal DOP * 100 */
	uint16_t hdop;
	/** Vertical DOP * 100 */
	uint16_t vdop;
};

/** Single satellite entry within a GSV message */
struct nmea_gsv_sv {
	/** Satellite PRN number */
	uint16_t prn;
	/** Elevation in degrees, 0-90 */
	int8_t elevation;
	/** Azimuth in degrees, 0-359 */
	uint16_t azimuth;
	/** Signal-to-noise ratio in dB-Hz, 0 = not tracking */
	uint8_t snr;
};

/**
 * @brief GSV — GNSS Satellites in View
 *
 * Each GSV message contains up to 4 satellite entries.
 * A full sky view requires multiple GSV messages.
 */
struct nmea_gsv {
	/** Total number of GSV messages in this group */
	uint8_t total_msgs;
	/** Sequence number of this message (1-based) */
	uint8_t msg_num;
	/** Total satellites in view */
	uint8_t total_svs;
	/** Satellite data (up to 4 per message) */
	struct nmea_gsv_sv sv[4];
	/** Number of valid entries in sv[] */
	uint8_t sv_count;
};

/**
 * @brief VTG — Course Over Ground and Ground Speed
 */
struct nmea_vtg {
	/** Course referenced to true north in degrees * 100 */
	int32_t course_true_e2;
	/** Course referenced to magnetic north in degrees * 100 */
	int32_t course_mag_e2;
	/** Speed in knots * 100 */
	int32_t speed_knots_e2;
	/** Speed in km/h * 100 */
	int32_t speed_kph_e2;
};

/**
 * @brief GLL — Geographic Position Latitude/Longitude
 */
struct nmea_gll {
	/** Latitude in degrees * 1e7, negative for south */
	int32_t latitude;
	/** Longitude in degrees * 1e7, negative for west */
	int32_t longitude;
	/** UTC time */
	struct nmea_time time;
	/** True if status is 'A' (active/valid) */
	bool valid;
};

/**
 * @brief Tagged union containing a parsed NMEA sentence
 */
struct nmea_msg {
	/** Identifies which member of the union is populated */
	enum nmea_sentence_type type;
	/** Talker ID from the sentence (e.g. "GP", "GN", "GL") */
	char talker[3];
	union {
		struct nmea_gga gga;
		struct nmea_rmc rmc;
		struct nmea_gsa gsa;
		struct nmea_gsv gsv;
		struct nmea_vtg vtg;
		struct nmea_gll gll;
	};
};

/**
 * @brief Parse a single null-terminated NMEA 0183 sentence.
 *
 * The sentence must begin with '$' and should not include the
 * trailing CR/LF. Checksum validation is always performed.
 *
 * @param sentence Null-terminated NMEA sentence string.
 * @param msg      Output struct populated on success.
 *
 * @retval 0        Sentence parsed successfully.
 * @retval -EINVAL  Sentence is NULL, malformed, or too short.
 * @retval -EBADMSG Checksum mismatch.
 * @retval -ENOTSUP Sentence type not supported by this build configuration.
 * @retval -EPROTO  Field count or content does not match expected format.
 */
int nmea_parse(const char *sentence, struct nmea_msg *msg);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NMEA_NMEA_H_ */
