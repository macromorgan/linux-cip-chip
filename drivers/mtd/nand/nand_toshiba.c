/*
 * Copyright (C) 2014 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>

static u8 tc58teg_read_retry_regs[] = {
	0x4, 0x5, 0x6, 0x7, 0xd
};

static u8 tc58teg_read_retry_values[] = {
	0x0, 0x0, 0x0, 0x0, 0x0,
	0x2, 0x4, 0x2, 0x0, 0x0,
	0x7c, 0x0, 0x7c, 0x7c, 0x0,
	0x7a, 0x0, 0x7a, 0x7a, 0x0,
	0x78, 0x2, 0x78, 0x7a, 0x0,
	0x7e, 0x4, 0x7e, 0x7a, 0x0,
	0x76, 0x4, 0x76, 0x78, 0x0,
	0x4, 0x4, 0x4, 0x76, 0x0,
	0x6, 0xa, 0x6, 0x2, 0x0,
	0x74, 0x7c, 0x74, 0x76, 0x0,
};

static int tc58teg_setup_read_retry(struct mtd_info *mtd, int retry_mode)
{
	struct nand_chip *chip = mtd->priv;
	int i, offset;

	offset = ARRAY_SIZE(tc58teg_read_retry_regs) * retry_mode;
	chip->cmdfunc(mtd, 0x5c, -1, -1);
	chip->cmdfunc(mtd, 0xc5, -1, -1);
	for (i = 0; i < ARRAY_SIZE(tc58teg_read_retry_regs); i++) {
		chip->cmdfunc(mtd, 0x55, tc58teg_read_retry_regs[i], -1);
		chip->write_byte(mtd, tc58teg_read_retry_values[offset + i]);
	}
	chip->cmdfunc(mtd, 0x26, -1, -1);
	chip->cmdfunc(mtd, 0x5d, -1, -1);

	if (!retry_mode)
		chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	return 0;
}

static int tc58teg_init(struct mtd_info *mtd, const uint8_t *id)
{
	struct nand_chip *chip = mtd->priv;

	chip->setup_read_retry = tc58teg_setup_read_retry;
	chip->read_retries = 10;
	chip->options |= NAND_NEED_SCRAMBLING;
	chip->onfi_timing_mode_default = 3;
	mtd->pairing = &dist3_pairing_scheme;

	return 0;
}

struct toshiba_nand_initializer {
	u8 id[6];
	int (*init)(struct mtd_info *mtd, const uint8_t *id);
};

static struct toshiba_nand_initializer initializers[] = {
	{
		.id = {NAND_MFR_TOSHIBA, 0xde, 0x94, 0x93, 0x76, 0x51},
		.init = tc58teg_init,
	},
	{
		.id = {NAND_MFR_TOSHIBA, 0xd7, 0x84, 0x93, 0x72, 0x51},
		.init = tc58teg_init,
	},
};

int toshiba_nand_init(struct mtd_info *mtd, const uint8_t *id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(initializers); i++) {
		struct toshiba_nand_initializer *initializer = &initializers[i];
		if (memcmp(id, initializer->id, sizeof(initializer->id)))
			continue;

		return initializer->init(mtd, id);
	}

	return 0;
}
EXPORT_SYMBOL(toshiba_nand_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boris BREZILLON <b.brezillon.dev@gmail.com>");
MODULE_DESCRIPTION("Toshiba NAND specific code");
