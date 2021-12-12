#ifndef __RK_EDP_PANEL_H__
#define	__RK_EDP_PANEL_H__
#include <drm/drmP.h>
#include <drm/drm_panel.h>
#include <dev/extres/regulator/regulator.h>
#include <dev/gpio/gpiobusvar.h>
#include "dev/drm/bridges/anxdp/anx_dp.h"
struct rk_edp_panel_softc {
	device_t dev;
	struct drm_panel panel;
	gpio_pin_t	pin;
	regulator_t     power_supply;
	int enabled;
};

DECLARE_CLASS(rk_edp_panel_driver);
#endif /* __RK_EDP_PANEL_H__ */
