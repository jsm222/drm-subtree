/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Jesper Schmitz Mouridsen  <jsm@FreeBSD.org>
 * Copyright (c) 2019 Jonathan A. Kollasch <jakllsch@kollasch.net>

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include "rk_edp.h"
#include "syscon_if.h"
#include "dev/drm/bridges/anxdp/anx_dp.h"
#include "anx_edp_if.h"
#include "iicbus_if.h"
#define	 RK3399_GRF_SOC_CON20		0x6250
#define  EDP_LCDC_SEL			BIT(5)


static struct ofw_compat_data   rkedp_compat_data[] = {
  {"rockchip,rk3399-edp",     1},
  {NULL,                      0}

};

static struct resource_spec rk_edp_spec[]  = {
  { SYS_RES_MEMORY,	0,	RF_ACTIVE },
  { SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
  { -1,0}
};
static int rk_edp_probe(device_t dev);
static int rk_edp_attach(device_t dev);

static int rk_edp_add_encoder(device_t dev, struct drm_crtc *crtc, struct drm_device *drm);

#define	to_rk_anxdp_softc(x)	container_of(x, struct rk_edp_softc, sc_base)
#define	to_anxdp_encoder(x)	container_of(x, struct anxdp_softc, sc_encoder)


static void rk_anxdp_select_input(struct rk_edp_softc *sc, u_int crtc_index)
{
  const uint32_t write_mask = EDP_LCDC_SEL << 16;
  const uint32_t write_val = crtc_index == 0 ? EDP_LCDC_SEL : 0;
  SYSCON_WRITE_4(sc->grf, RK3399_GRF_SOC_CON20, write_mask | write_val);

}

static bool
rk_anxdp_encoder_mode_fixup(struct drm_encoder *encoder,
			    const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
  return true;
}

static void
rk_anxdp_encoder_mode_set(struct drm_encoder *encoder,
			  struct drm_display_mode *mode, struct drm_display_mode *adjusted)
{

}

static void
rk_anxdp_encoder_enable(struct drm_encoder *encoder)
{
}

static void
rk_anxdp_encoder_disable(struct drm_encoder *encoder)
{
}

static void
rk_anxdp_encoder_prepare(struct drm_encoder *encoder)
{
  struct anxdp_softc *sc_base;
  struct rk_edp_softc *sc;
  sc_base = container_of(encoder, struct anxdp_softc, sc_encoder);
  sc = container_of(sc_base, struct rk_edp_softc, sc_base);

  const u_int crtc_index = drm_crtc_index(encoder->crtc);

  rk_anxdp_select_input(sc, crtc_index);
}

static void
rk_anxdp_encoder_commit(struct drm_encoder *encoder)
{
}

static void
rk_anxdp_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static const struct drm_encoder_funcs rk_anxdp_encoder_funcs = {
  .destroy = drm_encoder_cleanup,
};

static const struct drm_encoder_helper_funcs rk_anxdp_encoder_helper_funcs = {
  .prepare = rk_anxdp_encoder_prepare,
  .mode_fixup = rk_anxdp_encoder_mode_fixup,
  .mode_set = rk_anxdp_encoder_mode_set,
  .enable = rk_anxdp_encoder_enable,
  .disable = rk_anxdp_encoder_disable,
  .commit = rk_anxdp_encoder_commit,
  .dpms = rk_anxdp_encoder_dpms,
};

#define	to_rk_edp_softc(x)	container_of(x, struct rk_edp_softc, sc_base)
#define	to_rk_edp_encoder(x)	container_of(x, struct rk_edp_softc, sc_encoder)

int rk_do_small_transfer(device_t dev, struct iic_msg msg,int k) {
  size_t j=0;
  size_t loop_timeout = 0;
  uint32_t val;
  ssize_t ret = 0;
  struct rk_edp_softc * csc;
  struct anxdp_softc * sc;
  csc = device_get_softc(dev);
  sc = &csc->sc_base;
  val = AUX_LENGTH(msg.len);
  if (msg.flags & I2C_M_RD)  {
    val |= AUX_TX_COMM_READ | AUX_TX_COMM_I2C_TRANSACTION;
  }
  bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_CTL_1, val);
  bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_ADDR_7_0,
		    AUX_ADDR_7_0(msg.slave>>1));
  bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_ADDR_15_8,
		    AUX_ADDR_15_8(msg.slave>>1 ));
  bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_ADDR_19_16,
		    AUX_ADDR_19_16(msg.slave>>1));
  bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_CTL_2,
		    AUX_EN | ((msg.len==0) ? ADDR_ONLY : 0));
  loop_timeout = 0;
  val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_CTL_2);
  while ((val & AUX_EN) != 0) {
    if (++loop_timeout > 20000) {
      ret = -ETIMEDOUT;
      goto out;
    }
    DELAY(25);
    val = bus_space_read_4(sc->sc_bst, sc->sc_bsh,
			   ANXDP_AUX_CH_CTL_2);
  }

  loop_timeout = 0;
  val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA);
  while (!(val & RPLY_RECEIV)) {
    if (++loop_timeout > 2000) {
      ret = -ETIMEDOUT;
      goto out;
    }
    DELAY(10);
    val = bus_space_read_4(sc->sc_bst, sc->sc_bsh,
			   ANXDP_DP_INT_STA);
  }

  bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA,
		    RPLY_RECEIV);
  val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA);
  if ((val & AUX_ERR) != 0) {
    bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA,
		      AUX_ERR);
    ret = -EREMOTEIO;
    goto out;
  }
  val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_STA);
  if (AUX_STATUS(val) != 0) {
    ret = -EREMOTEIO;
    goto out;
  }

  if(msg.flags & I2C_M_RD) {
    for (j = 0; j < 16;j++) {
      msg.buf[j+k*16] = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_BUF_DATA(j));
      ret++;
    }
    val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_RX_COMM);
  }
 out:
  if (ret < 0) {
    anxdp_init_aux(sc);
    return ret;
  }
  return 0;
}


int rk_test_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs) {

  struct rk_edp_softc * csc;
  struct anxdp_softc * sc;
  size_t k,i,j;
  size_t loop_timeout = 0;
  uint32_t val;
  csc = device_get_softc(dev);
  sc = &csc->sc_base;

  ssize_t ret = 0;
  for (i=0;i<nmsgs;i++) {
    if(msgs[i].len > 16) {
      for(k=0;k<msgs[i].len / 16;k++) {
	ret = rk_do_small_transfer(dev,msgs[i],k);
	if (ret!=0) {
	  return -1;
	}
      }

    } else {
      val = AUX_LENGTH(msgs[i].len);
      if (msgs[i].flags & I2C_M_RD)  {
	val |= AUX_TX_COMM_READ | AUX_TX_COMM_I2C_TRANSACTION;
      }

      bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_CTL_1, val);
      bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_ADDR_7_0,
			AUX_ADDR_7_0(msgs[i].slave>>1));
      bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_ADDR_15_8,
			AUX_ADDR_15_8(msgs[i].slave>>1 ));
      bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_ADDR_19_16,
			AUX_ADDR_19_16(msgs[i].slave>>1));

      if (!(msgs[i].flags & I2C_M_RD)) {
	for (j = 0; j < msgs[i].len; j++) {

	  bus_space_write_4(sc->sc_bst, sc->sc_bsh,
			    ANXDP_BUF_DATA(j),
			    msgs[i].buf[j]);
	  ret++;
	}
      }


      bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_CTL_2,
			AUX_EN | ((msgs[i].len==0) ? ADDR_ONLY : 0));
      loop_timeout = 0;
      val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_CTL_2);
      while ((val & AUX_EN) != 0) {
	if (++loop_timeout > 20000) {
	  ret = -ETIMEDOUT;
	  goto out;
	}
	DELAY(25);
	val = bus_space_read_4(sc->sc_bst, sc->sc_bsh,
			       ANXDP_AUX_CH_CTL_2);
      }

      loop_timeout = 0;
      val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA);
      while (!(val & RPLY_RECEIV)) {
	if (++loop_timeout > 2000) {
	  ret = -ETIMEDOUT;
	  goto out;
	}
	DELAY(10);
	val = bus_space_read_4(sc->sc_bst, sc->sc_bsh,
			       ANXDP_DP_INT_STA);
      }

      bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA,
			RPLY_RECEIV);

      val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA);
      if ((val & AUX_ERR) != 0) {

	bus_space_write_4(sc->sc_bst, sc->sc_bsh, ANXDP_DP_INT_STA,
			  AUX_ERR);
	ret = -EREMOTEIO;
	goto out;
      }

      val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_CH_STA);
      if (AUX_STATUS(val) != 0) {

	ret = -EREMOTEIO;
	goto out;
      }
      if (msgs[i].flags & I2C_M_RD) {
	for (j = 0; j < msgs[i].len;j++) {
	  uint32_t b = bus_space_read_4(sc->sc_bst, sc->sc_bsh, 0x7c0+4*j);
	  msgs[i].buf[j] = b;
	  ret++;
	}
      }

      val = bus_space_read_4(sc->sc_bst, sc->sc_bsh, ANXDP_AUX_RX_COMM);
    }
  }
 out:
  if (ret < 0) {
    anxdp_init_aux(sc);
  }
  return (0);
}

static device_method_t rk_edp_methods[] = {
  /* Device interface */
  DEVMETHOD(device_probe,		rk_edp_probe),
  DEVMETHOD(device_attach,	rk_edp_attach),
  DEVMETHOD(anx_edp_add_encoder,  rk_edp_add_encoder),
  DEVMETHOD(iicbus_transfer,	rk_test_transfer),
  DEVMETHOD_END
};
static int
rk_edp_probe(device_t dev)
{

  if (!ofw_bus_status_okay(dev))
    return (ENXIO);

  if (ofw_bus_search_compatible(dev, rkedp_compat_data)->ocd_data == 0)
    return (ENXIO);

  device_set_desc(dev, "RockChip edp");
  return (BUS_PROBE_DEFAULT);
}
static int rk_edp_attach(device_t dev)
{


  struct rk_edp_softc *sc;
  phandle_t node;
  int error;


  sc = device_get_softc(dev);
  node = ofw_bus_get_node(dev);
  if (bus_alloc_resources(dev,rk_edp_spec, sc->res)!=0) {
    device_printf(dev, "could not allocate resources\n");
    return (ENXIO);
  }
  sc->sc_base.sc_bst = rman_get_bustag(sc->res[0]);
  sc->sc_base.sc_bsh = rman_get_bushandle(sc->res[0]);

  sc->sc_base.sc_flags |= ANXDP_FLAG_ROCKCHIP;


  error = clk_get_by_ofw_name(dev, 0, "pclk", &sc->pclk);
  if (error!=0) {
    printf("could not get pclk error:%d\n",error);
    return -1;
  }
  error = clk_enable(sc->pclk);
  if (error!=0) {
    printf("could not enable pclk error:%d\n",error);
    return -1;
  }
  error = clk_get_by_ofw_name(dev, 0, "dp", &sc->dpclk);
  if (error!=0) {
    printf("could not get dp clock error:%d\n",error);
    return -1;
  }
  error = clk_enable(sc->dpclk);
  if (error!=0) {
    printf("could not enable dp error:%d\n",error);
    return -1;
  }
  error = clk_get_by_ofw_name(dev, 0, "grf", &sc->grfclk);
  if (error!=0) {
    printf("could not get grf clock error:%d\n",error);
    return -1;
  }
  error = clk_enable(sc->grfclk);
  if (error!=0) {
    printf("could not enabel grp clok error:%d\n",error);
    return -1;
  }
  error = syscon_get_by_ofw_property(dev, node, "rockchip,grf", &sc->grf);
  if (error != 0) {
    printf("cannot get grf syscon: %d\n", error);
    return (ENXIO);
  }

  sc->dev=dev;



  sc->sc_base.sc_dev=dev;
  anxdp_attach(&sc->sc_base);
  return (0);

}




static int rk_edp_add_encoder(device_t dev, struct drm_crtc *crtc, struct drm_device *drm)
{
  struct rk_edp_softc *sc;
  sc = device_get_softc(dev);

  sc->sc_base.sc_encoder.possible_crtcs = drm_crtc_mask(crtc);
  drm_encoder_helper_add(&sc->sc_base.sc_encoder,&rk_anxdp_encoder_helper_funcs);
  drm_encoder_init(drm, &sc->sc_base.sc_encoder, &rk_anxdp_encoder_funcs,
		   DRM_MODE_ENCODER_TMDS, NULL);

  sc->sc_base.sc_connector.connector_type = DRM_MODE_CONNECTOR_eDP;
  rk_anxdp_select_input(sc,crtc->index);
  anxdp_add_bridge(&sc->sc_base,&sc->sc_base.sc_encoder);
  return (0);
}







static devclass_t rk_edp_devclass;
DEFINE_CLASS_1(rk_edp, rk_edp_driver, rk_edp_methods,
	       sizeof(struct rk_edp_softc), anxdp_driver);

EARLY_DRIVER_MODULE(rk_edp, simplebus, rk_edp_driver,rk_edp_devclass,
		    0,0,BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(rk_edp, 1);
