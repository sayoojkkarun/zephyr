/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_NXP_LPC_LPC84X_PINCTRL_SOC_H_
#define ZEPHYR_SOC_NXP_LPC_LPC84X_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <soc.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t pinctrl_soc_pin_t;

#define Z_PINCTRL_IOCON_PINCFG(node_id)							\
	(COND_CODE_1(DT_PROP(node_id, bias_pull_down), (IOCON_PIO_MODE(0x1) |), ())	\
	 COND_CODE_1(DT_PROP(node_id, bias_pull_up), (IOCON_PIO_MODE(0x2) |), ())		\
	 COND_CODE_1(DT_PROP(node_id, drive_push_pull), (IOCON_PIO_MODE(0x3) |), ())	\
	 COND_CODE_1(DT_PROP(node_id, input_schmitt_enable), (IOCON_PIO_HYS(1) |), ())	\
	 COND_CODE_1(DT_PROP(node_id, nxp_invert), (IOCON_PIO_INV(1) |), ())		\
	 COND_CODE_1(DT_PROP(node_id, drive_open_drain), (IOCON_PIO_OD(1) |), ())	\
	 COND_CODE_1(DT_PROP(node_id, nxp_digital_filter), (IOCON_PIO_S_MODE(1) |), ())	\
	 0)

#define Z_PINCTRL_IOCON_D_PIN_MASK (IOCON_PIO_MODE_MASK |				\
				    IOCON_PIO_HYS_MASK |					\
				    IOCON_PIO_INV_MASK |					\
				    IOCON_PIO_OD_MASK |					\
				    IOCON_PIO_S_MODE_MASK)

#define Z_PINCTRL_IOCON_A_PIN_MASK Z_PINCTRL_IOCON_D_PIN_MASK

#define Z_PINCTRL_IOCON_I_PIN_MASK (IOCON_PIO_I2CMODE_MASK | IOCON_PIO_S_MODE_MASK)

#define Z_PINCTRL_STATE_PIN_INIT(group, pin_prop, idx)					\
	DT_PROP_BY_IDX(group, pin_prop, idx) | Z_PINCTRL_IOCON_PINCFG(group),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)					\
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),				\
				DT_FOREACH_PROP_ELEM, pinmux,				\
				Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_NXP_LPC_LPC84X_PINCTRL_SOC_H_ */
