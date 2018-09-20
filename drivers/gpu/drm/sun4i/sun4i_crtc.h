/*
 * Copyright (C) 2015 Free Electrons
 * Copyright (C) 2015 NextThing Co
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef _SUN4I_CRTC_H_
#define _SUN4I_CRTC_H_

struct sun4i_crtc {
	struct drm_crtc			crtc;
	struct drm_pending_vblank_event	*event;

	struct sun4i_drv		*drv;
};

struct sun4i_crtc_state {
	struct drm_crtc_state	base;

	u32			display_x_size;
	u32			display_y_size;

	u32			plane_x_offset;
	u32			plane_y_offset;

	bool			vga_hack;
};

static inline struct sun4i_crtc *drm_crtc_to_sun4i_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct sun4i_crtc, crtc);
}

static inline struct sun4i_crtc_state *
drm_crtc_state_to_sun4i_crtc_state(struct drm_crtc_state *state)
{
	return container_of(state, struct sun4i_crtc_state, base);
}

struct sun4i_crtc *sun4i_crtc_init(struct drm_device *drm);

#endif /* _SUN4I_CRTC_H_ */
