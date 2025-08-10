KMOD     = rk_vop
DRMROOT?=./../../..
RKROOT?=${DRMROOT}/rockchip
SRCTOP?=/usr/src
SYSDIR?=${SRCTOP}/sys
.include "${SYSDIR}/conf/kern.opts.mk"
CFLAGS= -I${RKROOT}  -I${DRMROOT}/drmkpi/include/  -I${DRMROOT}/core/include/ -include ${DRMROOT}/drmkpi/include/drm/drm_os_freebsd.h -I${DRMROOT}/core/include/uapi/ -Wno-error=gnu-pointer-arith -DFDT -Wno-error=format
rk_vop_if.h:
	awk -f /usr/src/sys/tools/makeobjops.awk ${RKROOT}/rk_vop_if.m -h
dw_hdmi_if.h:
	awk -f /usr/src/sys/tools/makeobjops.awk ${DRMROOT}/dw_hdmi_if.m -h

SRCS=device_if.h bus_if.h vnode_if.h pci_if.h opt_platform.h ofw_bus_if.h clknode_if.h rk_vop_if.h fb_if.h phynode_if.h ${RKROOT}/rk_vop.c dw_hdmi_if.h ${RKROOT}/rk_plane.c



.include <bsd.kmod.mk>
