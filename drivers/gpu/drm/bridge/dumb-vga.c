
#include <linux/module.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>

struct dumb_vga {
	struct drm_bridge	bridge;
	struct drm_connector	connector;

	struct i2c_adapter	*ddc;
};

static inline struct dumb_vga *
drm_bridge_to_dumb_vga(struct drm_bridge *bridge)
{
	return container_of(bridge, struct dumb_vga, bridge);
}

static inline struct dumb_vga *
drm_connector_to_dumb_vga(struct drm_connector *connector)
{
	return container_of(connector, struct dumb_vga, connector);
}

static int dumb_vga_get_modes(struct drm_connector *connector)
{
	struct dumb_vga *vga = drm_connector_to_dumb_vga(connector);
	struct edid *edid;
	int ret;

	if (!vga->ddc)
		goto fallback;

	edid = drm_get_edid(connector, vga->ddc);
	if (!edid) {
		DRM_INFO("EDID readout failed, falling back to standard modes\n");
		goto fallback;
	}

	drm_mode_connector_update_edid_property(connector, edid);
	return drm_add_edid_modes(connector, edid);

fallback:
	/*
	 * In case we cannot retrieve the EDIDs (broken or missing i2c
	 * bus), fallback on the XGA standards
	 */
	ret = drm_add_modes_noedid(connector, 1920, 1200);

	/* And prefer a mode pretty much anyone can handle */
	drm_set_preferred_mode(connector, 1024, 768);

	return ret;
}

static struct drm_encoder *
dumb_vga_best_encoder(struct drm_connector *connector)
{
	struct dumb_vga *vga = drm_connector_to_dumb_vga(connector);

	return vga->bridge.encoder;
}

static struct drm_connector_helper_funcs dumb_vga_con_helper_funcs = {
	.get_modes	= dumb_vga_get_modes,
	.best_encoder	= dumb_vga_best_encoder,
};

static enum drm_connector_status
dumb_vga_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void
dumb_vga_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs dumb_vga_con_funcs = {
	.dpms			= drm_atomic_helper_connector_dpms,
	.detect			= dumb_vga_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= dumb_vga_connector_destroy,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static int dumb_vga_attach(struct drm_bridge *bridge)
{
	struct dumb_vga *vga = drm_bridge_to_dumb_vga(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Missing encoder\n");
		return -ENODEV;
	}

	drm_connector_helper_add(&vga->connector,
				 &dumb_vga_con_helper_funcs);
	ret = drm_connector_init(bridge->dev, &vga->connector,
				 &dumb_vga_con_funcs, DRM_MODE_CONNECTOR_VGA);
	if (ret) {
		DRM_ERROR("Failed to initialize connector\n");
		return ret;
	}

	drm_mode_connector_attach_encoder(&vga->connector,
					  bridge->encoder);

	return 0;
}

static void dumb_vga_nop(struct drm_bridge *bridge) {};

static struct drm_bridge_funcs dumb_vga_bridge_funcs = {
	.attach		= dumb_vga_attach,
	.enable		= dumb_vga_nop,
	.disable	= dumb_vga_nop,
	.pre_enable	= dumb_vga_nop,
	.post_disable	= dumb_vga_nop,
};

static int dumb_vga_probe(struct platform_device *pdev)
{
	struct dumb_vga *vga;
	struct device_node *ddc;

	vga = devm_kzalloc(&pdev->dev, sizeof(*vga), GFP_KERNEL);
	if (!vga)
		return -ENOMEM;
	platform_set_drvdata(pdev, vga);

	ddc = of_parse_phandle(pdev->dev.of_node, "ddc-i2c-bus", 0);
	if (ddc) {
		vga->ddc = of_find_i2c_adapter_by_node(ddc);
		of_node_put(ddc);

		if (!vga->ddc) {
			dev_err(&pdev->dev, "Couldn't retrieve i2c bus\n");
			return -EPROBE_DEFER;
		}
	} else {
		dev_info(&pdev->dev,
			 "No i2c bus specified... Disabling EDID readout\n");
	}

	vga->bridge.funcs = &dumb_vga_bridge_funcs;
	vga->bridge.of_node = pdev->dev.of_node;

	return drm_bridge_add(&vga->bridge);
}

static int dumb_vga_remove(struct platform_device *pdev)
{
	struct dumb_vga *vga = platform_get_drvdata(pdev);

	drm_bridge_remove(&vga->bridge);

	return 0;
}

static const struct of_device_id dumb_vga_match[] = {
	{ .compatible = "dumb-vga-bridge" },
	{},
};
MODULE_DEVICE_TABLE(of, dumb_vga_match);

struct platform_driver dumb_vga_driver = {
	.probe	= dumb_vga_probe,
	.remove	= dumb_vga_remove,
	.driver		= {
		.name		= "dumb-vga-bridge",
		.of_match_table	= dumb_vga_match,
	},
};
module_platform_driver(dumb_vga_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Dumb RGB to VGA bridge driver");
MODULE_LICENSE("GPL");
