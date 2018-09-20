/*
 * Copyright (C) 2015 - Antoine Tenart <antoine.tenart@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __DIP_MANAGER_H
#define __DIP_MANAGER_H

struct dip_header {
        u32     magic;                  /* rsvd */
        u8      version;                /* spec version */
        u32     vendor_id;
        u16     product_id;
        u8      product_version;
        char    vendor_name[32];
        char    product_name[32];
        u8      rsvd[36];               /* rsvd for futre spec versions */
        u8      data[16];               /* user data, per-DIP specific */
} __packed;

void dip_manager_insert(struct device *dev, struct dip_header *header);

#endif
