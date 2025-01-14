/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef DIGITAL_CDC_RSC_MGR_H
#define DIGITAL_CDC_RSC_MGR_H

#include <linux/clk.h>

#ifdef CONFIG_AUDIO_TRACE_PRINTK
#define TRACE_PRINTK(fmt, ...)				\
do {						\
	trace_printk(fmt, ##__VA_ARGS__); \
} while (0)
#else
#define TRACE_PRINTK(fmt, ...)				\
do {							\
} while (0)
#endif

#ifdef CONFIG_DIGITAL_CDC_RSC_MGR

int digital_cdc_rsc_mgr_hw_vote_enable(struct clk *vote_handle, struct device *dev);
void digital_cdc_rsc_mgr_hw_vote_disable(struct clk *vote_handle, struct device *dev);
void digital_cdc_rsc_mgr_hw_vote_reset(struct clk *vote_handle);

void digital_cdc_rsc_mgr_init(void);
void digital_cdc_rsc_mgr_exit(void);

#else

static inline int digital_cdc_rsc_mgr_hw_vote_enable(struct clk *vote_handle, struct device *dev)
{
	return 0;
}

static inline void digital_cdc_rsc_mgr_hw_vote_disable(struct clk *vote_handle, struct device *dev)
{
}

static inline void digital_cdc_rsc_mgr_hw_vote_reset(struct clk *vote_handle)
{
}

static inline void digital_cdc_rsc_mgr_init(void)
{
}

static inline void digital_cdc_rsc_mgr_exit(void)
{
}

#endif /* CONFIG_DIGITAL_CDC_RSC_MGR */

#endif /* DIGITAL_CDC_RSC_MGR_H */
