#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_graph.h>
#include <dev/gpio/gpiobusvar.h>
#include <sys/gpio.h>

#include "rk_edp_panel.h"

	

static struct ofw_compat_data   rkedp_compat_data[] = {
	{"boe,nv140fhmn49",     1},
	{NULL,                  0}

};
static int
panel_prepare(struct drm_panel *d_panel)
{

	struct rk_edp_panel_softc *sc;
	sc =	container_of(d_panel, struct rk_edp_panel_softc, panel);
	if(sc->power_supply)
		regulator_enable(sc->power_supply);
	return 0;
}
static int
panel_enable(struct drm_panel * d_panel)
{
	int err=0;
	struct rk_edp_panel_softc *sc;
	sc =	container_of(d_panel, struct rk_edp_panel_softc, panel);
	struct gpiobus_pin lcdpin;
	lcdpin.dev = sc->pin->dev;
	lcdpin.flags = GPIO_PIN_OUTPUT;
	lcdpin.pin = 0;
	bool active;
	gpio_pin_is_active(sc->pin, &active);
	if(!active) {	
		err = gpio_pin_setflags(&lcdpin,lcdpin.flags);
		if(err) {
		device_printf(sc->dev,"Could set gpio pin to out error:%d\n",err);
		return err;
		}
		err=gpio_pin_set_active(&lcdpin,1);
		if(err)  {
		device_printf(sc->dev,"Could set gpio pin to high error:%d\n",err);
		}
	}
	return err;
}
static const struct drm_panel_funcs panel_funcs = {
	.disable = NULL,
	.enable = panel_enable,
	.get_modes = NULL,
	.get_timings = NULL,
	.prepare = panel_prepare,
	.unprepare = NULL,
};
static int
rk_edp_panel_probe(device_t dev)	
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, rkedp_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "RockChip lcd panel");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_edp_panel_attach(device_t dev)
{
	struct rk_edp_panel_softc *sc;
	sc = device_get_softc(dev);
	phandle_t node;
	int err;
	if (regulator_get_by_ofw_property(dev, 0, "power-supply",
	    &sc->power_supply) != 0) {
		device_printf(dev, "No power-supply property\n");
		return (ENXIO);
	}
	if ((node = ofw_bus_get_node(dev)) == -1) {
		device_printf(dev, "failed to get node\n");
		return ENXIO;
	}

	if (!OF_hasprop(node, "enable-gpios")) {
		device_printf(dev, "missing enable-gpios property\n");
		return ENXIO;
	}
	err = gpio_pin_get_by_ofw_property(dev, node, "enable-gpios",&sc->pin);
	if(err!=0) { 
		device_printf(sc->dev, "Cannot get 'enable-gpios' gpio error: %d\n",err);
		return ENXIO;
	}

	sc->dev=dev;
	
	drm_panel_init(&sc->panel,sc->dev,&panel_funcs,DRM_MODE_CONNECTOR_eDP);
	drm_panel_add(&sc->panel);
	drm_panel_prepare(&sc->panel);
	drm_panel_enable(&sc->panel);
	
	return 0;
}
static device_method_t rk_edp_panel_methods[] = {
	DEVMETHOD(device_probe,		rk_edp_panel_probe),
	DEVMETHOD(device_attach,	rk_edp_panel_attach),
	DEVMETHOD_END
};


static devclass_t rk_edp_panel_devclass;
DRIVER_MODULE(rk_edp_panel, simplebus, rk_edp_panel_driver,rk_edp_panel_devclass,NULL,NULL);
DEFINE_CLASS_0(rk_edp_panel, rk_edp_panel_driver, rk_edp_panel_methods,sizeof(struct rk_edp_panel_softc));
MODULE_VERSION(rk_edp_panel, 1);
