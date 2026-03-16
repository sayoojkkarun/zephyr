/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/nmea/nmea.h>

#include "nmea_internal.h"

int nmea_parse_time(const char *str, struct nmea_time *time)
{
	if (str == NULL || strlen(str) < 6) {
		return -EINVAL;
	}

	for (int i = 0; i < 6; i++) {
		if (str[i] < '0' || str[i] > '9') {
			return -EINVAL;
		}
	}

	time->hours   = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
	time->minutes = (uint8_t)((str[2] - '0') * 10 + (str[3] - '0'));
	time->seconds = (uint8_t)((str[4] - '0') * 10 + (str[5] - '0'));

	time->milliseconds = 0;
	if (str[6] == '.') {
		uint16_t ms = 0;
		int digits = 0;
		const char *p = str + 7;

		while (*p >= '0' && *p <= '9' && digits < 3) {
			ms = ms * 10 + (uint16_t)(*p - '0');
			p++;
			digits++;
		}
		while (digits < 3) {
			ms *= 10;
			digits++;
		}
		time->milliseconds = ms;
	}

	return 0;
}

int nmea_parse_date(const char *str, struct nmea_date *date)
{
	if (str == NULL || strlen(str) < 6) {
		return -EINVAL;
	}

	for (int i = 0; i < 6; i++) {
		if (str[i] < '0' || str[i] > '9') {
			return -EINVAL;
		}
	}

	date->day   = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
	date->month = (uint8_t)((str[2] - '0') * 10 + (str[3] - '0'));

	uint8_t yy = (uint8_t)((str[4] - '0') * 10 + (str[5] - '0'));

	date->year = (yy >= 80) ? (uint16_t)(1900 + yy) : (uint16_t)(2000 + yy);

	return 0;
}

int32_t nmea_coord_to_e7(const char *str, char hemi, bool is_lon)
{
	int deg_digits = is_lon ? 3 : 2;
	int32_t degrees = 0;
	const char *p = str;

	if (str == NULL || *str == '\0') {
		return 0;
	}

	for (int i = 0; i < deg_digits; i++) {
		if (*p < '0' || *p > '9') {
			return 0;
		}
		degrees = degrees * 10 + (*p - '0');
		p++;
	}

	/* Parse integer part of minutes */
	int32_t min_int = 0;

	while (*p >= '0' && *p <= '9') {
		min_int = min_int * 10 + (*p - '0');
		p++;
	}

	/* Parse fractional minutes, normalize to 5 decimal places */
	int32_t min_frac = 0;
	int frac_digits = 0;

	if (*p == '.') {
		p++;
		while (*p >= '0' && *p <= '9' && frac_digits < 5) {
			min_frac = min_frac * 10 + (*p - '0');
			p++;
			frac_digits++;
		}
		while (frac_digits < 5) {
			min_frac *= 10;
			frac_digits++;
		}
	}

	/* Convert minutes to degrees * 1e7 using integer arithmetic */
	int32_t minutes_e5 = min_int * 100000 + min_frac;
	int32_t minutes_e7 = (minutes_e5 * 10 + 3) / 6;

	int32_t result = degrees * 10000000 + minutes_e7;

	if (hemi == 'S' || hemi == 'W') {
		result = -result;
	}

	return result;
}

int nmea_parse_decimal_e2(const char *str, int32_t *out)
{
	bool negative = false;
	int32_t integer_part = 0;
	int32_t frac_part = 0;
	int frac_digits = 0;
	const char *p = str;

	if (str == NULL || *str == '\0') {
		return -EINVAL;
	}

	if (*p == '-') {
		negative = true;
		p++;
	}

	while (*p >= '0' && *p <= '9') {
		integer_part = integer_part * 10 + (*p - '0');
		p++;
	}

	if (*p == '.') {
		p++;
		while (*p >= '0' && *p <= '9' && frac_digits < 2) {
			frac_part = frac_part * 10 + (*p - '0');
			p++;
			frac_digits++;
		}
		while (frac_digits < 2) {
			frac_part *= 10;
			frac_digits++;
		}
	}

	*out = integer_part * 100 + frac_part;
	if (negative) {
		*out = -(*out);
	}

	return 0;
}

int nmea_parse_decimal_e3(const char *str, int32_t *out)
{
	bool negative = false;
	int32_t integer_part = 0;
	int32_t frac_part = 0;
	int frac_digits = 0;
	const char *p = str;

	if (str == NULL || *str == '\0') {
		return -EINVAL;
	}

	if (*p == '-') {
		negative = true;
		p++;
	}

	while (*p >= '0' && *p <= '9') {
		integer_part = integer_part * 10 + (*p - '0');
		p++;
	}

	if (*p == '.') {
		p++;
		while (*p >= '0' && *p <= '9' && frac_digits < 3) {
			frac_part = frac_part * 10 + (*p - '0');
			p++;
			frac_digits++;
		}
		while (frac_digits < 3) {
			frac_part *= 10;
			frac_digits++;
		}
	}

	*out = integer_part * 1000 + frac_part;
	if (negative) {
		*out = -(*out);
	}

	return 0;
}

int nmea_parse_int(const char *str, int32_t *out)
{
	int32_t val = 0;
	const char *p = str;

	if (str == NULL || *str == '\0') {
		return -EINVAL;
	}

	while (*p >= '0' && *p <= '9') {
		val = val * 10 + (*p - '0');
		p++;
	}

	*out = val;
	return 0;
}

static uint8_t compute_checksum(const char *sentence)
{
	uint8_t csum = 0;
	const char *p = sentence + 1; /* skip '$' */

	while (*p != '\0' && *p != '*') {
		csum ^= (uint8_t)*p;
		p++;
	}

	return csum;
}

static int validate_checksum(const char *sentence)
{
	const char *star = strchr(sentence, '*');

	if (star == NULL || strlen(star) < 3) {
		return -EINVAL;
	}

	uint8_t expected = 0;

	for (int i = 1; i <= 2; i++) {
		char c = star[i];
		uint8_t nibble;

		if (c >= '0' && c <= '9') {
			nibble = (uint8_t)(c - '0');
		} else if (c >= 'A' && c <= 'F') {
			nibble = (uint8_t)(c - 'A' + 10);
		} else if (c >= 'a' && c <= 'f') {
			nibble = (uint8_t)(c - 'a' + 10);
		} else {
			return -EINVAL;
		}

		expected = (expected << 4) | nibble;
	}

	uint8_t computed = compute_checksum(sentence);

	if (computed != expected) {
		return -EBADMSG;
	}

	return 0;
}

static int tokenise(char *buf, char **fields, int *nfields)
{
	int count = 0;

	fields[count++] = buf;

	while (*buf != '\0') {
		if (*buf == ',' || *buf == '*') {
			*buf = '\0';
			if (count >= NMEA_MAX_FIELDS) {
				break;
			}
			fields[count++] = buf + 1;
		}
		buf++;
	}

	*nfields = count;
	return 0;
}

static enum nmea_sentence_type identify_type(const char *type_str)
{
	if (strncmp(type_str, "GGA", 3) == 0) {
		return NMEA_SENTENCE_GGA;
	} else if (strncmp(type_str, "RMC", 3) == 0) {
		return NMEA_SENTENCE_RMC;
	} else if (strncmp(type_str, "GSA", 3) == 0) {
		return NMEA_SENTENCE_GSA;
	} else if (strncmp(type_str, "GSV", 3) == 0) {
		return NMEA_SENTENCE_GSV;
	} else if (strncmp(type_str, "VTG", 3) == 0) {
		return NMEA_SENTENCE_VTG;
	} else if (strncmp(type_str, "GLL", 3) == 0) {
		return NMEA_SENTENCE_GLL;
	}

	return NMEA_SENTENCE_UNKNOWN;
}

int nmea_parse(const char *sentence, struct nmea_msg *msg)
{
	char buf[NMEA_MAX_SENTENCE_LEN + 1];
	char *fields[NMEA_MAX_FIELDS];
	int nfields;
	int ret;
	size_t len;

	if (sentence == NULL || msg == NULL) {
		return -EINVAL;
	}

	len = strlen(sentence);
	if (len < 7 || sentence[0] != '$') {
		return -EINVAL;
	}

	ret = validate_checksum(sentence);
	if (ret != 0) {
		return ret;
	}

	/* Copy to stack buffer for in-place tokenisation */
	if (len > NMEA_MAX_SENTENCE_LEN) {
		len = NMEA_MAX_SENTENCE_LEN;
	}
	memcpy(buf, sentence, len);
	buf[len] = '\0';

	/* Strip trailing CR/LF */
	while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n')) {
		buf[--len] = '\0';
	}

	memset(msg, 0, sizeof(*msg));

	/* Extract talker ID (2 chars after '$') */
	msg->talker[0] = buf[1];
	msg->talker[1] = buf[2];
	msg->talker[2] = '\0';

	/* Sentence type starts at position 3 (after $XX) */
	const char *type_str = buf + 3;

	msg->type = identify_type(type_str);

	/* Tokenise from first field (after $XXYYY,) */
	tokenise(buf + 1, fields, &nfields);

	switch (msg->type) {
#ifdef CONFIG_NMEA_SENTENCE_GGA
	case NMEA_SENTENCE_GGA:
		return nmea_parse_gga(fields, nfields, &msg->gga);
#endif
#ifdef CONFIG_NMEA_SENTENCE_RMC
	case NMEA_SENTENCE_RMC:
		return nmea_parse_rmc(fields, nfields, &msg->rmc);
#endif
#ifdef CONFIG_NMEA_SENTENCE_GSA
	case NMEA_SENTENCE_GSA:
		return nmea_parse_gsa(fields, nfields, &msg->gsa);
#endif
#ifdef CONFIG_NMEA_SENTENCE_GSV
	case NMEA_SENTENCE_GSV:
		return nmea_parse_gsv(fields, nfields, &msg->gsv);
#endif
#ifdef CONFIG_NMEA_SENTENCE_VTG
	case NMEA_SENTENCE_VTG:
		return nmea_parse_vtg(fields, nfields, &msg->vtg);
#endif
#ifdef CONFIG_NMEA_SENTENCE_GLL
	case NMEA_SENTENCE_GLL:
		return nmea_parse_gll(fields, nfields, &msg->gll);
#endif
	default:
		return -ENOTSUP;
	}
}
