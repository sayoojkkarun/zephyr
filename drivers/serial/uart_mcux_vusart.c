/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_vusart

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <fsl_clock.h>
#include <fsl_swm.h>
#include <fsl_usart.h>
#include <soc.h>

#ifndef VFIFO_USART_CTLSETUSART_RXTHINTEN_MASK
#define VFIFO_USART_CTLSETUSART_RXTHINTEN_MASK 0U
#endif
#ifndef VFIFO_USART_CTLSETUSART_TXTHINTEN_MASK
#define VFIFO_USART_CTLSETUSART_TXTHINTEN_MASK 0U
#endif
#ifndef VFIFO_USART_CTLSETUSART_RXTIMEOUTINTEN_MASK
#define VFIFO_USART_CTLSETUSART_RXTIMEOUTINTEN_MASK 0U
#endif
#ifndef VFIFO_USART_STATUSART_RXTH_MASK
#define VFIFO_USART_STATUSART_RXTH_MASK 0U
#endif
#ifndef VFIFO_USART_STATUSART_TXTH_MASK
#define VFIFO_USART_STATUSART_TXTH_MASK 0U
#endif
#ifndef VFIFO_USART_STATUSART_RXTIMEOUT_MASK
#define VFIFO_USART_STATUSART_RXTIMEOUT_MASK 0U
#endif
#ifndef VFIFO_USART_STATUSART_BUSERR_MASK
#define VFIFO_USART_STATUSART_BUSERR_MASK 0U
#endif
#ifndef VFIFO_USART_STATUSART_RXEMPTY_MASK
#define VFIFO_USART_STATUSART_RXEMPTY_MASK 0U
#endif
#ifndef VFIFO_USART_STATUSART_TXEMPTY_MASK
#define VFIFO_USART_STATUSART_TXEMPTY_MASK 0U
#endif

#ifndef USART_STAT_RXRDY_MASK
#define USART_STAT_RXRDY_MASK BIT(0)
#endif
#ifndef USART_STAT_TXRDY_MASK
#define USART_STAT_TXRDY_MASK BIT(2)
#endif
#ifndef USART_STAT_TXIDLE_MASK
#define USART_STAT_TXIDLE_MASK BIT(3)
#endif
#ifndef USART_STAT_OVERRUNINT_MASK
#define USART_STAT_OVERRUNINT_MASK BIT(8)
#endif
#ifndef USART_STAT_FRAMERRINT_MASK
#define USART_STAT_FRAMERRINT_MASK BIT(13)
#endif
#ifndef USART_STAT_PARITYERRINT_MASK
#define USART_STAT_PARITYERRINT_MASK BIT(14)
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_mcux_vusart, CONFIG_UART_LOG_LEVEL);

struct uart_mcux_vusart_config {
	USART_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint32_t baud_rate;
	const struct pinctrl_dev_config *pincfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
	uint8_t swm_tx_func;
	uint8_t swm_tx_pin;
	uint8_t swm_rx_func;
	uint8_t swm_rx_pin;
	uint8_t swm_rts_func;
	uint8_t swm_rts_pin;
	uint8_t swm_cts_func;
	uint8_t swm_cts_pin;
};

struct uart_mcux_vusart_data {
	struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

static inline void uart_mcux_vusart_apply_swm(const struct uart_mcux_vusart_config *config)
{
	if ((config->swm_tx_pin != 0xFFU) && (config->swm_tx_func != 0xFFU)) {
		SWM_SetMovablePinSelect(SWM0,
			(swm_select_movable_t)config->swm_tx_func,
			(swm_port_pin_type_t)config->swm_tx_pin);
	}

	if ((config->swm_rx_pin != 0xFFU) && (config->swm_rx_func != 0xFFU)) {
		SWM_SetMovablePinSelect(SWM0,
			(swm_select_movable_t)config->swm_rx_func,
			(swm_port_pin_type_t)config->swm_rx_pin);
	}

	if ((config->swm_rts_pin != 0xFFU) && (config->swm_rts_func != 0xFFU)) {
		SWM_SetMovablePinSelect(SWM0,
			(swm_select_movable_t)config->swm_rts_func,
			(swm_port_pin_type_t)config->swm_rts_pin);
	}

	if ((config->swm_cts_pin != 0xFFU) && (config->swm_cts_func != 0xFFU)) {
		SWM_SetMovablePinSelect(SWM0,
			(swm_select_movable_t)config->swm_cts_func,
			(swm_port_pin_type_t)config->swm_cts_pin);
	}
}

#if defined(CONFIG_SOC_SERIES_LPC84X)
static void uart_mcux_vusart_select_clock(const struct uart_mcux_vusart_config *config)
{
	clock_select_t select;

	if (config->base == USART0) {
		select = kUART0_Clk_From_MainClk;
	} else if (config->base == USART1) {
		select = kUART1_Clk_From_MainClk;
	} else if (config->base == USART2) {
		select = kUART2_Clk_From_MainClk;
	} else if (config->base == USART3) {
		select = kUART3_Clk_From_MainClk;
	} else if (config->base == USART4) {
		select = kUART4_Clk_From_MainClk;
	} else {
		return;
	}

	CLOCK_Select(select);
}
#else
static void uart_mcux_vusart_select_clock(const struct uart_mcux_vusart_config *config)
{
	ARG_UNUSED(config);
}
#endif

#if defined(CONFIG_SOC_SERIES_LPC84X)
static void uart_mcux_vusart_enable_ahb_clock(const struct uart_mcux_vusart_config *config)
{
	if (config->base == USART0) {
		CLOCK_EnableClock(kCLOCK_Uart0);
	} else if (config->base == USART1) {
		CLOCK_EnableClock(kCLOCK_Uart1);
	} else if (config->base == USART2) {
		CLOCK_EnableClock(kCLOCK_Uart2);
	} else if (config->base == USART3) {
		CLOCK_EnableClock(kCLOCK_Uart3);
	} else if (config->base == USART4) {
		CLOCK_EnableClock(kCLOCK_Uart4);
	}
}
#else
static void uart_mcux_vusart_enable_ahb_clock(const struct uart_mcux_vusart_config *config)
{
	ARG_UNUSED(config);
}
#endif

static int uart_mcux_vusart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_Type *base = config->base;

	if ((USART_GetStatusFlags(base) & kUSART_RxReady) == 0U) {
		return -1;
	}

	*c = USART_ReadByte(base);

	return 0;
}

static void uart_mcux_vusart_poll_out(const struct device *dev, unsigned char out_char)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_Type *base = config->base;

	while ((USART_GetStatusFlags(base) & kUSART_TxReady) == 0U) {
	}

	USART_WriteByte(base, out_char);
}

static int uart_mcux_vusart_err_check(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_Type *base = config->base;
	int status = 0;
	uint32_t flags = USART_GetStatusFlags(base);
	uint32_t err_flags = 0U;

	if ((flags & kUSART_FramErrorFlag) != 0U) {
		status |= UART_ERROR_FRAMING;
		err_flags |= kUSART_FramErrorFlag;
	}

	if ((flags & kUSART_ParityErrorFlag) != 0U) {
		status |= UART_ERROR_PARITY;
		err_flags |= kUSART_ParityErrorFlag;
	}

	if ((flags & kUSART_HardwareOverrunFlag) != 0U) {
		status |= UART_ERROR_OVERRUN;
		err_flags |= kUSART_HardwareOverrunFlag;
	}

	if (err_flags != 0U) {
		USART_ClearStatusFlags(base, err_flags);
	}

	return status;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_mcux_vusart_configure(const struct device *dev,
					 const struct uart_config *cfg)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	struct uart_mcux_vusart_data *data = dev->data;
	usart_config_t usart_cfg;
	uint32_t clk_freq;
	int ret;
	status_t hal_status;

	if ((cfg->data_bits != UART_CFG_DATA_BITS_8) &&
	    (cfg->data_bits != UART_CFG_DATA_BITS_7)) {
		return -ENOTSUP;
	}

	if ((cfg->stop_bits != UART_CFG_STOP_BITS_1) &&
	    (cfg->stop_bits != UART_CFG_STOP_BITS_2)) {
		return -ENOTSUP;
	}

	if (cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	USART_GetDefaultConfig(&usart_cfg);
	usart_cfg.baudRate_Bps = cfg->baudrate;
	usart_cfg.enableRx = true;
	usart_cfg.enableTx = true;
	usart_cfg.stopBitCount = (cfg->stop_bits == UART_CFG_STOP_BITS_2) ?
		kUSART_TwoStopBit : kUSART_OneStopBit;
	usart_cfg.bitCountPerChar = (cfg->data_bits == UART_CFG_DATA_BITS_7) ?
		kUSART_7BitsPerChar : kUSART_8BitsPerChar;

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		usart_cfg.parityMode = kUSART_ParityDisabled;
		break;
	case UART_CFG_PARITY_EVEN:
		usart_cfg.parityMode = kUSART_ParityEven;
		break;
	case UART_CFG_PARITY_ODD:
		usart_cfg.parityMode = kUSART_ParityOdd;
		break;
	default:
		return -ENOTSUP;
	}

	ret = clock_control_get_rate(config->clock_dev,
					  config->clock_subsys,
					  &clk_freq);
	if (ret < 0) {
		return ret;
	}

	hal_status = USART_Init(config->base, &usart_cfg, clk_freq);
	if (hal_status != kStatus_Success) {
		LOG_ERR("Failed to init vUSART (err %d)", hal_status);
		return -EIO;
	}

	data->uart_cfg = *cfg;

	return 0;
}

static int uart_mcux_vusart_config_get(const struct device *dev,
					   struct uart_config *cfg)
{
	struct uart_mcux_vusart_data *data = dev->data;

	*cfg = data->uart_cfg;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_mcux_vusart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_Type *base = config->base;
	int n = 0;

	while ((n < size) && ((USART_GetStatusFlags(base) & kUSART_TxReady) != 0U)) {
		USART_WriteByte(base, tx_data[n++]);
	}

	return n;
}

static int uart_mcux_vusart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_Type *base = config->base;
	int n = 0;

	while ((n < size) && ((USART_GetStatusFlags(base) & kUSART_RxReady) != 0U)) {
		rx_data[n++] = USART_ReadByte(base);
	}

	return n;
}

static void uart_mcux_vusart_irq_tx_enable(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_EnableInterrupts(config->base, kUSART_TxReadyInterruptEnable);
}

static void uart_mcux_vusart_irq_tx_disable(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_DisableInterrupts(config->base, kUSART_TxReadyInterruptEnable);
}

static int uart_mcux_vusart_irq_tx_ready(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;

	return (USART_GetStatusFlags(config->base) & kUSART_TxReady) != 0U;
}

static int uart_mcux_vusart_irq_tx_complete(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;

	return (USART_GetStatusFlags(config->base) & kUSART_TxIdleFlag) != 0U;
}

static void uart_mcux_vusart_irq_rx_enable(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_EnableInterrupts(config->base, kUSART_RxReadyInterruptEnable);
}

static void uart_mcux_vusart_irq_rx_disable(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_DisableInterrupts(config->base, kUSART_RxReadyInterruptEnable);
}

static int uart_mcux_vusart_irq_rx_ready(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;

	return (USART_GetStatusFlags(config->base) & kUSART_RxReady) != 0U;
}

static void uart_mcux_vusart_irq_err_enable(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_EnableInterrupts(config->base,
		kUSART_HardwareOverRunInterruptEnable |
		kUSART_FramErrorInterruptEnable |
		kUSART_ParityErrorInterruptEnable);
}

static void uart_mcux_vusart_irq_err_disable(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	USART_DisableInterrupts(config->base,
		kUSART_HardwareOverRunInterruptEnable |
		kUSART_FramErrorInterruptEnable |
		kUSART_ParityErrorInterruptEnable);
}

static int uart_mcux_vusart_irq_is_pending(const struct device *dev)
{
	return uart_mcux_vusart_irq_tx_ready(dev) || uart_mcux_vusart_irq_rx_ready(dev);
}

static int uart_mcux_vusart_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void uart_mcux_vusart_irq_callback_set(const struct device *dev,
					       uart_irq_callback_user_data_t cb,
					       void *cb_data)
{
	struct uart_mcux_vusart_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_mcux_vusart_isr(const struct device *dev)
{
	struct uart_mcux_vusart_data *data = dev->data;

	if ((data->callback != NULL) && uart_mcux_vusart_irq_is_pending(dev)) {
		data->callback(dev, data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_mcux_vusart_init(const struct device *dev)
{
	const struct uart_mcux_vusart_config *config = dev->config;
	struct uart_mcux_vusart_data *data = dev->data;
	int ret;

	uart_mcux_vusart_select_clock(config);
	uart_mcux_vusart_enable_ahb_clock(config);

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl apply failed (%d)", ret);
		return ret;
	}

	uart_mcux_vusart_apply_swm(config);

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("clock on failed (%d)", ret);
		return ret;
	}

	data->uart_cfg.baudrate = config->baud_rate;
	data->uart_cfg.parity = UART_CFG_PARITY_NONE;
	data->uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
	data->uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
	data->uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	ret = uart_mcux_vusart_configure(dev, &data->uart_cfg);
	if (ret < 0) {
		return ret;
	}
#else
	usart_config_t usart_cfg;
	uint32_t clk_freq;

	USART_GetDefaultConfig(&usart_cfg);
	usart_cfg.baudRate_Bps = data->uart_cfg.baudrate;
	usart_cfg.enableRx = true;
	usart_cfg.enableTx = true;

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys,
		&clk_freq);
	if (ret < 0) {
		return ret;
	}

	if (USART_Init(config->base, &usart_cfg, clk_freq) != kStatus_Success) {
		return -EIO;
	}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (config->irq_config_func != NULL) {
		config->irq_config_func(dev);
	}
#endif

	return 0;
}

static const struct uart_driver_api uart_mcux_vusart_driver_api = {
	.poll_in = uart_mcux_vusart_poll_in,
	.poll_out = uart_mcux_vusart_poll_out,
	.err_check = uart_mcux_vusart_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_mcux_vusart_configure,
	.config_get = uart_mcux_vusart_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_mcux_vusart_fifo_fill,
	.fifo_read = uart_mcux_vusart_fifo_read,
	.irq_tx_enable = uart_mcux_vusart_irq_tx_enable,
	.irq_tx_disable = uart_mcux_vusart_irq_tx_disable,
	.irq_tx_ready = uart_mcux_vusart_irq_tx_ready,
	.irq_tx_complete = uart_mcux_vusart_irq_tx_complete,
	.irq_rx_enable = uart_mcux_vusart_irq_rx_enable,
	.irq_rx_disable = uart_mcux_vusart_irq_rx_disable,
	.irq_rx_ready = uart_mcux_vusart_irq_rx_ready,
	.irq_err_enable = uart_mcux_vusart_irq_err_enable,
	.irq_err_disable = uart_mcux_vusart_irq_err_disable,
	.irq_is_pending = uart_mcux_vusart_irq_is_pending,
	.irq_update = uart_mcux_vusart_irq_update,
	.irq_callback_set = uart_mcux_vusart_irq_callback_set,
#endif
};

#define UART_MCUX_VUSART_SWM_FUNC(n, signal) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, nxp_swm_##signal), \
		(DT_INST_PROP_BY_IDX(n, nxp_swm_##signal, 0)), \
		(0xFFU))

#define UART_MCUX_VUSART_SWM_PIN(n, signal) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, nxp_swm_##signal), \
		(DT_INST_PROP_BY_IDX(n, nxp_swm_##signal, 1)), \
		(0xFFU))

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_MCUX_VUSART_IRQ_CFG_FUNC(n) \
	static void uart_mcux_vusart_irq_config_##n(const struct device *dev) \
	{ \
		ARG_UNUSED(dev); \
		IRQ_CONNECT(DT_INST_IRQN(n), \
			 DT_INST_IRQ(n, priority), \
			 uart_mcux_vusart_isr, \
			 DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	}
#define UART_MCUX_VUSART_IRQ_INIT(n) \
	.irq_config_func = uart_mcux_vusart_irq_config_##n,
#else
#define UART_MCUX_VUSART_IRQ_CFG_FUNC(n)
#define UART_MCUX_VUSART_IRQ_INIT(n)
#endif

#define UART_MCUX_VUSART_INIT(n) \
	PINCTRL_DT_INST_DEFINE(n); \
	UART_MCUX_VUSART_IRQ_CFG_FUNC(n) \
	static const struct uart_mcux_vusart_config uart_mcux_vusart_config_##n = { \
		.base = (USART_Type *)DT_INST_REG_ADDR(n), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name), \
		.baud_rate = DT_INST_PROP_OR(n, current_speed, 115200), \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
		UART_MCUX_VUSART_IRQ_INIT(n) \
		.swm_tx_func = (uint8_t)UART_MCUX_VUSART_SWM_FUNC(n, tx), \
		.swm_tx_pin = (uint8_t)UART_MCUX_VUSART_SWM_PIN(n, tx), \
		.swm_rx_func = (uint8_t)UART_MCUX_VUSART_SWM_FUNC(n, rx), \
		.swm_rx_pin = (uint8_t)UART_MCUX_VUSART_SWM_PIN(n, rx), \
		.swm_rts_func = (uint8_t)UART_MCUX_VUSART_SWM_FUNC(n, rts), \
		.swm_rts_pin = (uint8_t)UART_MCUX_VUSART_SWM_PIN(n, rts), \
		.swm_cts_func = (uint8_t)UART_MCUX_VUSART_SWM_FUNC(n, cts), \
		.swm_cts_pin = (uint8_t)UART_MCUX_VUSART_SWM_PIN(n, cts), \
	}; \
	static struct uart_mcux_vusart_data uart_mcux_vusart_data_##n; \
	DEVICE_DT_INST_DEFINE(n, \
			uart_mcux_vusart_init, \
			NULL, \
			&uart_mcux_vusart_data_##n, \
			&uart_mcux_vusart_config_##n, \
			PRE_KERNEL_1, \
			CONFIG_SERIAL_INIT_PRIORITY, \
			&uart_mcux_vusart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_MCUX_VUSART_INIT)
