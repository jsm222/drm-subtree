KMOD     = rk_dw_hdmi
DRMROOT?=./../../..
DWHDMIROOT?=${DRMROOT}/bridges/dw_hdmi
SRCTOP?=/usr/src
SYSDIR?=${SRCTOP}/sys
.include "${SYSDIR}/conf/kern.opts.mk"
CFLAGS= -I${SYSDIR} -I${DRMROOT} -I${DRMROOT}/drmkpi/include/  -I${DRMROOT}/core/include/ -include ${DRMROOT}/drmkpi/include/drm/drm_os_freebsd.h -I${DRMROOT}/core/include/uapi/ -Wno-error=gnu-pointer-arith -Wno-error=missing-prototypes -DFDT
SRCS=device_if.h bus_if.h vnode_if.h pci_if.h opt_platform.h ofw_bus_if.h clknode_if.h fb_if.h  syscon_if.h iicbus_if.h phynode_if.h ${DWHDMIROOT}/rk_dw_hdmi.c ${DWHDMIROOT}/dw_hdmi.c \
${DRMROOT}/dw_hdmi_if.h \
${DWHDMIROOT}/dw_hdmi_phy_if.h

.include <bsd.kmod.mk>
