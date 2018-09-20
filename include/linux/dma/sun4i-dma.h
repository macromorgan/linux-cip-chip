/*
 * Sun4i DMA Engine drivers support header file
 *
 * Copyright (C) 2016 Free Electrons. All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _SUN4I_DMA_H
#define _SUN4I_DMA_H

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

/* Dedicated DMA parameter register layout */
#define SUN4I_DDMA_PARA_DST_DATA_BLK_SIZE(n)	(((n) - 1) << 24)
#define SUN4I_DDMA_PARA_DST_WAIT_CYCLES(n)	(((n) - 1) << 16)
#define SUN4I_DDMA_PARA_SRC_DATA_BLK_SIZE(n)	(((n) - 1) << 8)
#define SUN4I_DDMA_PARA_SRC_WAIT_CYCLES(n)	(((n) - 1) << 0)

/**
 * struct sun4i_dma_chan_config - DMA channel config
 *
 * @para: contains information about block size and time before checking
 *	  DRQ line. This is device specific and only applicable to dedicated
 *	  DMA channels
 */
struct sun4i_dma_chan_config {
	u32 para;
};

int sun4i_dma_set_chan_config(struct dma_chan *dchan,
			      const struct sun4i_dma_chan_config *cfg);

#endif /* _SUN4I_DMA_H */
