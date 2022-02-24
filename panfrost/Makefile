KMOD=   panfrost
DEBUG_FLAGS=-g
SRCS=   panfrost_mmu.c panfrost_job.c panfrost_drv.c panfrost_gem.c panfrost_device.c device_if.h bus_if.h opt_platform.h clknode_if.h vnode_if.h  pci_if.h ofw_bus_if.h
CFLAGS+=-I${SRC}/sys/dev/drm/core/include -I${SRC}/sys -I${SRC}/sys/dev/drm/drmkpi/include/ -I${SRC}/sys/dev/drm/core/include/uapi -I/${SRC}/sys/sys -DFDT=1
KERNCONFDIR=  /usr/src/sys/arm64/conf
.include <bsd.kmod.mk>
