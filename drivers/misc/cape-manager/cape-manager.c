/*
 * Copyright (C) 2015 - Antoine Tenart <antoine.tenart@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>

#include <linux/dip-manager.h>

LIST_HEAD(dip_list);
DEFINE_SPINLOCK(dip_lock);

#define DIP_MANAGER_MAGIC	0x43484950
#define DIP_FIRMWARE_PREFIX	"nextthingco/chip/"

#define dip_convert(field)					\
	(							\
		(sizeof(field) == 1) ? field :			\
		(sizeof(field) == 2) ? be16_to_cpu(field) :	\
		(sizeof(field) == 4) ? be32_to_cpu(field) :	\
		-1						\
	)

struct dip {
	struct list_head	head;

	struct dip_header	*header;

	unsigned int		loaded:1;

	char			*dtbo;
	const struct firmware	*fw;
	struct device_node	*overlay;
	int			overlay_id;
};

static int dip_manager_check_overlay(struct device *dev, struct dip *dip)
{
	 struct property *p;
	 const char *s = NULL;
	 bool compatible = false;

	 p = of_find_property(dip->overlay, "compatible", NULL);
	 if (!p) {
		dev_err(dev, "Missing compatible property in %s\n",
			 dip->dtbo);
		return -EINVAL;
	 }

	 do {
		s = of_prop_next_string(p, s);
		if (of_machine_is_compatible(s)) {
			compatible = true;
			break;
		}
	 } while(s);

	if (!compatible) {
		dev_err(dev, "Incompatible overlay\n");
		return -EINVAL;
	 }

	return 0;
}

static int dip_manager_load(struct device *dev, struct dip *dip)
{
	int err;
	char *dtbo;

	if (dip->loaded) {
		dev_err(dev, "Overlay already loaded for dip %s\n",
			dip->header->product_name);
		return -EAGAIN;
	}

	dtbo = kasprintf(GFP_KERNEL, DIP_FIRMWARE_PREFIX "dip-%x-%x-%x.dtbo",
			 dip_convert(dip->header->vendor_id),
			 dip_convert(dip->header->product_id),
			 dip_convert(dip->header->product_version));
	if (!dtbo)
		return -ENOMEM;

	/* First try to request the overlay specific to the version */
	err = request_firmware_direct(&dip->fw, dtbo, dev);
	if (err) {
		/* If it fails, try with a generic one */
		kfree(dtbo);

		dtbo = kasprintf(GFP_KERNEL, DIP_FIRMWARE_PREFIX "dip-%x-%x.dtbo",
				 dip_convert(dip->header->vendor_id),
				 dip_convert(dip->header->product_id));
		err = request_firmware_direct(&dip->fw, dtbo, dev);
		if (err) {
			dev_err(dev, "Could not find overlay %s for dip %s\n",
				dtbo, dip->header->product_name);
			return err;
		}
	}
	dip->dtbo = dtbo;

	of_fdt_unflatten_tree((unsigned long *)dip->fw->data, &dip->overlay);
	if (!dip->overlay) {
		dev_err(dev, "Could not unflatten %s\n", dip->dtbo);
		err = -EINVAL;
		goto err;
	}

	of_node_set_flag(dip->overlay, OF_DETACHED);

	err = of_resolve_phandles(dip->overlay);
	if (err) {
		dev_err(dev, "Could not resolve phandles for overlay %s (%d)\n",
			dip->dtbo, err);
		goto err;
	}

	if (dip_manager_check_overlay(dev, dip))
		goto err;

	dip->overlay_id = of_overlay_create(dip->overlay);
	if (dip->overlay_id < 0) {
		dev_err(dev, "Could not apply overlay %s for dip %s\n",
			dip->dtbo, dip->header->product_name);
		err = dip->overlay_id;
		goto err;
	}

	dip->loaded = 1;
	return 0;

err:
	dip->overlay = NULL;
	release_firmware(dip->fw);
	return err;
}

/*
 * Called by an id provider when a new id is detected. The id is stored and an
 * overlay is applied if it matches the id.
 */
void dip_manager_insert(struct device *dev, struct dip_header *header)
{
	struct list_head *pos, *n;
	struct dip *dip;

	if (dip_convert(header->magic) != DIP_MANAGER_MAGIC) {
		dev_err(dev, "Bad magic value (%x)\n",
			dip_convert(header->magic));
		return;
	}

	/* a few sanitary modifications */
	header->vendor_name[31] = '\0';
	header->product_name[31] = '\0';

#if 0
	/* debug */
	pr_err("magic: 0x%x\n", dip_convert(header->magic));
	pr_err("version: %d\n", dip_convert(header->version));
	pr_err("vendor id: 0x%x\n", dip_convert(header->vendor_id));
	pr_err("product id: 0x%x\n", dip_convert(header->product_id));
	pr_err("product version: %d\n", dip_convert(header->product_version));
	pr_err("vendor name: %s\n", header->vendor_name);
	pr_err("product name: %s\n", header->product_name);
#endif

	spin_lock(&dip_lock);

	list_for_each_safe(pos, n, &dip_list) {
		dip = list_entry(pos, struct dip, head);

		if (dip_convert(dip->header->vendor_id) == dip_convert(header->vendor_id) &&
		    dip_convert(dip->header->product_id) == dip_convert(header->product_id) &&
		    dip_convert(dip->header->product_version) == dip_convert(header->product_version)) {
			dev_err(dev, "Dip already loaded\n");
			goto err;
		}
	}

	dip = devm_kzalloc(dev, sizeof(*dip), GFP_KERNEL);
	if (!dip)
		goto err;

	dip->loaded = 0;
	dip->header = header;

	if (dip_manager_load(dev, dip)) {
		dev_err(dev, "Couldn't load dip %s\n", header->product_name);
		goto err;
	}

	list_add_tail(&dip->head, &dip_list);

	dev_info(dev, "Overlay %s for dip %s applied!\n", dip->dtbo,
		 header->product_name);

err:
	spin_unlock(&dip_lock);
}
EXPORT_SYMBOL_GPL(dip_manager_insert);
