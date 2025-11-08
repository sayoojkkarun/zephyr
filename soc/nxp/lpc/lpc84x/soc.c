/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <fsl_clock.h>
#include <fsl_common.h>
#include <fsl_power.h>
#include <soc.h>
#include <string.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#define CLOCK_SETUP_NODE DT_NODELABEL(clock_setup)

#if DT_NODE_HAS_STATUS(CLOCK_SETUP_NODE, okay)

static int configure_fro_frequency(void)
{
#if DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, fro_freq)
	const char *const freq = DT_PROP(CLOCK_SETUP_NODE, fro_freq);

	if (strcmp(freq, "18M") == 0) {
		CLOCK_SetFroOscFreq(kCLOCK_FroOscOut18M);
	} else if (strcmp(freq, "24M") == 0) {
		CLOCK_SetFroOscFreq(kCLOCK_FroOscOut24M);
	} else if (strcmp(freq, "30M") == 0) {
		CLOCK_SetFroOscFreq(kCLOCK_FroOscOut30M);
	} else {
		LOG_ERR("Unsupported FRO frequency selection: %s", freq);
		return -EINVAL;
	}
#endif /* DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, fro_freq) */

	return 0;
}

static int board_clock_init(void)
{
	int ret;

#if DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, enable_froout)
	POWER_DisablePD(kPDRUNCFG_PD_FRO_OUT);
#endif
#if DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, enable_fro)
	POWER_DisablePD(kPDRUNCFG_PD_FRO);
#endif

	ret = configure_fro_frequency();
	if (ret < 0) {
		return ret;
	}

	CLOCK_SetFroOutClkSrc(kCLOCK_FroSrcFroOsc);

#if DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, enable_sysosc)
	POWER_DisablePD(kPDRUNCFG_PD_SYSOSC);
#endif

#if DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, extclk_src)
	const char *const extclk_src = DT_PROP(CLOCK_SETUP_NODE, extclk_src);

	if (strcmp(extclk_src, "sysosc") == 0) {
		CLOCK_Select(kEXT_Clk_From_SysOsc);
	} else if (strcmp(extclk_src, "clkin") == 0) {
		CLOCK_Select(kEXT_Clk_From_ClkIn);
	} else {
		LOG_ERR("Unsupported external clock source: %s", extclk_src);
		return -EINVAL;
	}
#else
	CLOCK_Select(kEXT_Clk_From_SysOsc);
#endif

#if DT_NODE_HAS_PROP(CLOCK_SETUP_NODE, mainclk_src)
	const char *const mainclk_src = DT_PROP(CLOCK_SETUP_NODE, mainclk_src);

	if (strcmp(mainclk_src, "fro") == 0) {
		CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);
	} else if (strcmp(mainclk_src, "extclk") == 0) {
		CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcExtClk);
	} else if (strcmp(mainclk_src, "wdosc") == 0) {
		CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcWdtOsc);
	} else if (strcmp(mainclk_src, "frodiv") == 0) {
		CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFroDiv);
	} else if (strcmp(mainclk_src, "syspll") == 0) {
		CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcSysPll);
	} else {
		LOG_ERR("Unsupported main clock source: %s", mainclk_src);
		return -EINVAL;
	}
#else
	CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);
#endif

	CLOCK_SetCoreSysClkDiv(1U);

	SystemCoreClockUpdate();

	return 0;
}

#else

static int board_clock_init(void)
{
	return 0;
}

#endif /* DT_NODE_HAS_STATUS(CLOCK_SETUP_NODE, okay) */

static int nxp_lpc84x_init(void)
{
	int ret;

	ret = board_clock_init();
	if (ret < 0) {
		return ret;
	}

	CLOCK_EnableClock(kCLOCK_Iocon);
	CLOCK_EnableClock(kCLOCK_Swm);

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gpio0))
	CLOCK_EnableClock(kCLOCK_Gpio0);
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gpio1))
	CLOCK_EnableClock(kCLOCK_Gpio1);
#endif

	return 0;
}

SYS_INIT(nxp_lpc84x_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
