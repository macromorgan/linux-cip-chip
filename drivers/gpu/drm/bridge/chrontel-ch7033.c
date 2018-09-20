
#include <linux/module.h>
#include <linux/i2c.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>

#define CH7033_PAGE_SEL_REG		0x03

#define CH7033_POWER_STATE_4_REG	0x0a
#define CH7033_POWER_STATE_4_MEM_INIT		BIT(7)
#define CH7033_POWER_STATE_4_MEM_STOP		BIT(4)

#define CH7033_INPUT_TIMING_1_REG	0x0b
#define CH7033_INPUT_TIMING_1_HTI(val)		(((val >> 8) & 0xf) << 3)
#define CH7033_INPUT_TIMING_1_HAI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_2_REG	0x0c
#define CH7033_INPUT_TIMING_2_HAI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_3_REG	0x0d
#define CH7033_INPUT_TIMING_3_HTI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_4_REG	0x0e
#define CH7033_INPUT_TIMING_4_HWI(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_INPUT_TIMING_4_HOI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_5_REG	0x0f
#define CH7033_INPUT_TIMING_5_HOI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_6_REG	0x10
#define CH7033_INPUT_TIMING_6_HWI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_7_REG	0x11
#define CH7033_INPUT_TIMING_7_VTI(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_INPUT_TIMING_7_VAI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_8_REG	0x12
#define CH7033_INPUT_TIMING_8_VAI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_9_REG	0x13
#define CH7033_INPUT_TIMING_9_VTI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_10_REG	0x14
#define CH7033_INPUT_TIMING_10_VWI(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_INPUT_TIMING_10_VOI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_11_REG	0x15
#define CH7033_INPUT_TIMING_11_VOI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_12_REG	0x16
#define CH7033_INPUT_TIMING_12_VWI(val)		(val & 0xff)

#define CH7033_INPUT_POL_REG		0x19
#define CH7033_INPUT_POL_HSYNC_HI		BIT(5)
#define CH7033_INPUT_POL_VSYNC_HI		BIT(4)
#define CH7033_INPUT_POL_DE_HI			BIT(3)
#define CH7033_INPUT_POL_GCLK(val)		((val >> 16) & 0x3)

#define CH7033_GCLK_1_REG		0x1a
#define CH7033_GCLK_1_FREQ(val)			((val >> 8) & 0xff)

#define CH7033_GCLK_2_REG		0x1b
#define CH7033_GCLK_2_FREQ(val)			((val) & 0xff)

#define CH7033_OUTPUT_TIMING_1_REG	0x1f
#define CH7033_OUTPUT_TIMING_1_HTO(val)		(((val >> 8) & 0xf) << 3)
#define CH7033_OUTPUT_TIMING_1_HAO(val)		((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_2_REG	0x20
#define CH7033_OUTPUT_TIMING_2_HAO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_3_REG	0x21
#define CH7033_OUTPUT_TIMING_3_HTO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_7_REG	0x25
#define CH7033_OUTPUT_TIMING_7_VTO(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_OUTPUT_TIMING_7_VAO(val)		((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_8_REG	0x26
#define CH7033_OUTPUT_TIMING_8_VAO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_9_REG	0x27
#define CH7033_OUTPUT_TIMING_9_VTO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_4_REG	0x54
#define CH7033_OUTPUT_TIMING_4_HWO(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_OUTPUT_TIMING_4_HOO(val)		((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_5_REG	0x55
#define CH7033_OUTPUT_TIMING_5_HOO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_6_REG	0x56
#define CH7033_OUTPUT_TIMING_6_HWO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_10_REG	0x57
#define CH7033_OUTPUT_TIMING_10_VWO(val)	(((val >> 8) & 0x7) << 3)
#define CH7033_OUTPUT_TIMING_10_VOO(val)	((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_11_REG	0x58
#define CH7033_OUTPUT_TIMING_11_VOO(val)	(val & 0xff)

#define CH7033_OUTPUT_TIMING_12_REG	0x59
#define CH7033_OUTPUT_TIMING_12_VWO(val)	(val & 0xff)

struct ch7033 {
	struct drm_bridge	bridge;
	struct drm_connector	connector;

	struct i2c_client	*client;
};

static u8 ch7033_write(struct i2c_client *client, u8 page, u8 reg, u8 value)
{
	i2c_smbus_write_byte_data(client, CH7033_PAGE_SEL_REG, page);

	return i2c_smbus_write_byte_data(client, reg, value);
}

static int ch7033_update_bits(struct i2c_client *client, u8 page, u8 reg,
			      u8 mask, u8 bits)
{
	u8 data;

	i2c_smbus_write_byte_data(client, CH7033_PAGE_SEL_REG, page);

	data = i2c_smbus_read_byte_data(client, reg);
	data &= ~(mask);
	i2c_smbus_write_byte_data(client, reg, data | bits);

	return 0;
}


#define R007_PD_IO			0x20	// always ON
#define R007_DRI_PD			0x08	// HDMI(CH7033/5)/LVDS(CH7034) 4 serial drivers, postpine to power on it for LVDS
#define	R007_PDMIO			0x04	// SDRAM IO power, special sequence
#define R007_PDPLL1			0x02	// SDRAM PLL, special sequence, turn off last

#define	R009_SCLPD			0x10	// SDRAM clock, special sequence
#define	R009_SDPD			0x08	// SDRAM contol logic,  special sequence
#define R009_HDMI_PD			0x01

#define	R00A_MEMPD			0x20	// SDRAM PD, sepcial sequence

#define R16B_DRISER_PD		0x01	// always be 0

#define R16C_DRIPLL_PD			0x02

static void ch7033_memory_init(struct i2c_client *client)
{
	ch7033_update_bits(client, 0, CH7033_POWER_STATE_4_REG,
			   CH7033_POWER_STATE_4_MEM_INIT | CH7033_POWER_STATE_4_MEM_STOP,
			   CH7033_POWER_STATE_4_MEM_INIT | CH7033_POWER_STATE_4_MEM_STOP);

	ch7033_update_bits(client, 0, CH7033_POWER_STATE_4_REG,
			   CH7033_POWER_STATE_4_MEM_STOP,
			   0);
}

static void ch7033_unknown_init(struct i2c_client *client)
{

	ch7033_write(client, 0, 0x5e, 0x54);
	ch7033_write(client, 0, 0x74, 0x30);
	ch7033_write(client, 0, 0x7e, 0x8f);

	ch7033_write(client, 1, 0x07, 0x66);
	ch7033_write(client, 1, 0x0b, 0x75);
	ch7033_write(client, 1, 0x0c, 0x6a);
	ch7033_write(client, 1, 0x0d, 0x21);
	ch7033_write(client, 1, 0x0f, 0x9d);

	ch7033_write(client, 3, 0x28, 0x04);
	ch7033_write(client, 3, 0x2a, 0x28);
}

static int ch7033_power(struct i2c_client *client)
{
	/* Main Power Up */
	ch7033_update_bits(client, 0, 0x07,
			   R007_PD_IO | R007_PDPLL1 | R007_PDMIO | R007_DRI_PD,
			   0);

	/* Power up SDRAM clock and power domain, and HDMI block */
	ch7033_update_bits(client, 0, 0x09,
			   R009_SCLPD | R009_SDPD | R009_HDMI_PD,
			   0);

	ch7033_update_bits(client, 0, 0x0a, R00A_MEMPD, 0);

	/* Power up DRISER */
	ch7033_update_bits(client, 1, 0x6b, R16B_DRISER_PD, 0);

	/* Power up DRI PLL */
	ch7033_update_bits(client, 1, 0x6c, R16C_DRIPLL_PD, 0);

	return 0;
}

static void ch7033_reset(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(client, 3, 4);
	i2c_smbus_write_byte_data(client, 0x52, 0xC3);
	i2c_smbus_write_byte_data(client, 0x52, 0xC1);
	i2c_smbus_write_byte_data(client, 0x52, 0xC3);

	i2c_smbus_write_byte_data(client, 3, 0);
	i2c_smbus_write_byte_data(client, 0x1C, 0x69);
	i2c_smbus_write_byte_data(client, 0x1D, 0x78);

	i2c_smbus_write_byte_data(client, 3, 1);
	i2c_smbus_write_byte_data(client, 0x1E, 9);
}

static inline struct ch7033 *
drm_bridge_to_ch7033(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ch7033, bridge);
}

static inline struct ch7033 *
drm_connector_to_ch7033(struct drm_connector *connector)
{
	return container_of(connector, struct ch7033, connector);
}

static int ch7033_get_modes(struct drm_connector *connector)
{
	struct ch7033 *ch7033 = drm_connector_to_ch7033(connector);
	struct edid *edid;

	edid = drm_get_edid(connector, ch7033->client->adapter);
	if (!edid)
		return -ENODEV;

	drm_mode_connector_update_edid_property(connector, edid);
	return drm_add_edid_modes(connector, edid);
}

static struct drm_encoder *
ch7033_best_encoder(struct drm_connector *connector)
{
	struct ch7033 *ch7033 = drm_connector_to_ch7033(connector);

	return ch7033->bridge.encoder;
}

static struct drm_connector_helper_funcs ch7033_con_helper_funcs = {
	.get_modes	= ch7033_get_modes,
	.best_encoder	= ch7033_best_encoder,
};

static enum drm_connector_status
ch7033_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void
ch7033_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs ch7033_con_funcs = {
	.dpms			= drm_atomic_helper_connector_dpms,
	.detect			= ch7033_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= ch7033_connector_destroy,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static int ch7033_attach(struct drm_bridge *bridge)
{
	struct ch7033 *ch7033 = drm_bridge_to_ch7033(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Missing encoder\n");
		return -ENODEV;
	}

	drm_connector_helper_add(&ch7033->connector,
				 &ch7033_con_helper_funcs);
	ret = drm_connector_init(bridge->dev, &ch7033->connector,
				 &ch7033_con_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		DRM_ERROR("Failed to initialize connector\n");
		return ret;
	}

	drm_mode_connector_attach_encoder(&ch7033->connector,
					  bridge->encoder);

	return 0;
}

static void ch7033_mode_set(struct drm_bridge *bridge,
			    struct drm_display_mode *mode,
			    struct drm_display_mode *adj_mode)
{
	struct ch7033 *ch7033 = drm_bridge_to_ch7033(bridge);
	struct i2c_client *client = ch7033->client;
	u16 hbp = mode->hsync_start - mode->hdisplay;
	u16 hsync_len = mode->hsync_end - mode->hsync_start;
	u16 vbp = mode->vsync_start - mode->vdisplay;
	u16 vsync_len = mode->vsync_end - mode->vsync_start;
	u32 val;

	/* Setup the horizontal timings ... */
	ch7033_write(client, 0, CH7033_INPUT_TIMING_1_REG,
		     CH7033_INPUT_TIMING_1_HTI(mode->htotal) |
		     CH7033_INPUT_TIMING_1_HAI(mode->hdisplay));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_2_REG,
		     CH7033_INPUT_TIMING_2_HAI(mode->hdisplay));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_3_REG,
		     CH7033_INPUT_TIMING_3_HTI(mode->htotal));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_4_REG,
		     CH7033_INPUT_TIMING_4_HOI(hbp) |
		     CH7033_INPUT_TIMING_4_HWI(hsync_len));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_5_REG,
		     CH7033_INPUT_TIMING_5_HOI(hbp));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_6_REG,
		     CH7033_INPUT_TIMING_6_HWI(hsync_len));

	/* ... And the vertical ones */
	ch7033_write(client, 0, CH7033_INPUT_TIMING_7_REG,
		     CH7033_INPUT_TIMING_7_VTI(mode->vtotal) |
		     CH7033_INPUT_TIMING_7_VAI(mode->vdisplay));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_8_REG,
		     CH7033_INPUT_TIMING_8_VAI(mode->vdisplay));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_9_REG,
		     CH7033_INPUT_TIMING_9_VTI(mode->vtotal));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_10_REG,
		     CH7033_INPUT_TIMING_10_VOI(vbp) |
		     CH7033_INPUT_TIMING_10_VWI(vsync_len));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_11_REG,
		     CH7033_INPUT_TIMING_11_VOI(vbp));

	ch7033_write(client, 0, CH7033_INPUT_TIMING_12_REG,
		     CH7033_INPUT_TIMING_12_VWI(vsync_len));

	/* Setup polarities and clock */
	val = CH7033_INPUT_POL_DE_HI;
	val |= (mode->flags & DRM_MODE_FLAG_PHSYNC) ? CH7033_INPUT_POL_HSYNC_HI : 0;
	val |= (mode->flags & DRM_MODE_FLAG_PVSYNC) ? CH7033_INPUT_POL_VSYNC_HI : 0;
	val |= CH7033_INPUT_POL_GCLK(mode->clock);
	ch7033_write(client, 0, CH7033_INPUT_POL_REG, val);

	ch7033_write(client, 0, CH7033_GCLK_1_REG,
		     CH7033_GCLK_1_FREQ(mode->clock));

	ch7033_write(client, 0, CH7033_GCLK_2_REG,
		     CH7033_GCLK_2_FREQ(mode->clock));

	/* Horizontal output timings ... */
	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_1_REG,
		     CH7033_OUTPUT_TIMING_1_HTO(mode->htotal) |
		     CH7033_OUTPUT_TIMING_1_HAO(mode->hdisplay));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_2_REG,
		     CH7033_OUTPUT_TIMING_2_HAO(mode->hdisplay));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_3_REG,
		     CH7033_OUTPUT_TIMING_3_HTO(mode->htotal));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_4_REG,
		     CH7033_OUTPUT_TIMING_4_HOO(hbp) |
		     CH7033_OUTPUT_TIMING_4_HWO(hsync_len));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_5_REG,
		     CH7033_OUTPUT_TIMING_5_HOO(hbp));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_6_REG,
		     CH7033_OUTPUT_TIMING_6_HWO(hsync_len));

	/* ... And the vertical ones */
	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_7_REG,
		     CH7033_OUTPUT_TIMING_7_VTO(mode->vtotal) |
		     CH7033_OUTPUT_TIMING_7_VAO(mode->vdisplay));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_8_REG,
		     CH7033_OUTPUT_TIMING_8_VAO(mode->vdisplay));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_9_REG,
		     CH7033_OUTPUT_TIMING_9_VTO(mode->vtotal));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_10_REG,
		     CH7033_OUTPUT_TIMING_10_VOO(vbp) |
		     CH7033_OUTPUT_TIMING_10_VWO(vsync_len));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_11_REG,
		     CH7033_OUTPUT_TIMING_11_VOO(vbp));

	ch7033_write(client, 0, CH7033_OUTPUT_TIMING_12_REG,
		     CH7033_OUTPUT_TIMING_12_VWO(vsync_len));
}

static void ch7033_pre_enable(struct drm_bridge *bridge)
{
	struct ch7033 *ch7033 = drm_bridge_to_ch7033(bridge);
	struct i2c_client *client = ch7033->client;

	ch7033_unknown_init(client);
	ch7033_power(client);
}

static void ch7033_enable(struct drm_bridge *bridge)
{
	struct ch7033 *ch7033 = drm_bridge_to_ch7033(bridge);
	struct i2c_client *client = ch7033->client;

	ch7033_memory_init(client);

	i2c_smbus_write_byte_data(client, 0x08, 0x0F);
}

static void ch7033_nop(struct drm_bridge *bridge) {};

static struct drm_bridge_funcs ch7033_bridge_funcs = {
	.attach		= ch7033_attach,

	.mode_set	= ch7033_mode_set,

	.enable		= ch7033_enable,
	.disable	= ch7033_nop,
	.pre_enable	= ch7033_pre_enable,
	.post_disable	= ch7033_nop,
};

static struct drm_display_mode *ch7033_create_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	strncpy(mode->name, "XP", 3);

	mode->type = DRM_MODE_TYPE_DRIVER;
	mode->flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC;
	mode->vrefresh = 60;
	mode->clock = 65000;

	mode->hdisplay = 1024;
	mode->hsync_start = 1048;
	mode->hsync_end = 1184;
	mode->htotal = 1344;

	mode->vdisplay = 768;
	mode->vsync_start = 771;
	mode->vsync_end = 777;
	mode->vtotal = 806;

	return mode;
}

static int ch7033_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ch7033 *ch7033;
	struct drm_display_mode *mode;

	ch7033 = devm_kzalloc(&client->dev, sizeof(*ch7033), GFP_KERNEL);
	if (!ch7033) {
		return -ENOMEM;
	}
	i2c_set_clientdata(client, ch7033);
	ch7033->client = client;

	ch7033->bridge.funcs = &ch7033_bridge_funcs;
	ch7033->bridge.of_node = client->dev.of_node;

	ch7033_reset(client);

	return drm_bridge_add(&ch7033->bridge);
}

static int ch7033_remove(struct i2c_client *client)
{
	struct ch7033 *ch7033 = i2c_get_clientdata(client);

	drm_bridge_remove(&ch7033->bridge);

	return 0;
}

static const struct i2c_device_id ch7033_i2c_table[] = {
	{ "ch7033", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ch7033_i2c_table);

static const struct of_device_id ch7033_of_table[] = {
	{ .compatible = "chrontel,ch7033" },
	{},
};
MODULE_DEVICE_TABLE(of, ch7033_of_table);

struct i2c_driver ch7033_driver = {
	.probe 		= ch7033_probe,
	.remove		= ch7033_remove,
	.id_table	= ch7033_i2c_table,

	.driver		= {
		.name		= "ch7033",
		.of_match_table	= ch7033_of_table,
	},
};
module_i2c_driver(ch7033_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Chrontel CH7033 bridge driver");
MODULE_LICENSE("GPL");
