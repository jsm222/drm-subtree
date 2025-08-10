KMOD     = drm_kmod
SRCTOP?= /usr/src
SYSDIR?=${SRCTOP}/sys
.include "${SYSDIR}/conf/kern.opts.mk"
./../../dw_hdmi_if.c:
	awk -f /usr/src/sys/tools/makeobjops.awk ./../../dw_hdmi_if.m -c -p
./../../dw_hdmi_if.h:
	awk -f /usr/src/sys/tools/makeobjops.awk ./../../dw_hdmi_if.m -h -p
rk_vop_if.c:
		awk -f /usr/src/sys/tools/makeobjops.awk ./../../rockchip/rk_vop_if.m -c 
rk_vop_if.h:
	awk -f /usr/src/sys/tools/makeobjops.awk ./../../rockchip/rk_vop_if.m -h 
./../../bridges/dw_hdmi/dw_hdmi_phy_if.c:
	awk -f /usr/src/sys/tools/makeobjops.awk ./../../bridges/dw_hdmi/dw_hdmi_phy_if.m -c -p
./../../bridges/dw_hdmi/dw_hdmi_phy_if.h:
	awk -f /usr/src/sys/tools/makeobjops.awk ./../../bridges/dw_hdmi/dw_hdmi_phy_if.m -h -p

CFLAGS= -I./../../drmkpi/include/  -I./../../core/include/ -include ./../../drmkpi/include/drm/drm_os_freebsd.h -I./../../core/include/uapi/ -Wno-error=gnu-pointer-arith -Wno-error=missing-prototypes -Wno-error=format -Wno-error=cast-qual -Wno-error=unused-but-set-variable '-DKBUILD_MODNAME="DRMv5.8"'
SRCS=device_if.h bus_if.h vnode_if.h pci_if.h opt_platform.h ofw_bus_if.h clknode_if.h phynode_if.h \
./../../dw_hdmi_if.c	rk_vop_if.h rk_vop_if.c ./../../dw_hdmi_if.h ./../../bridges/dw_hdmi/dw_hdmi_phy_if.c  ./../../bridges/dw_hdmi/dw_hdmi_phy_if.h \
./../../core/drm_atomic_state_helper.c \
./../../core/drm_atomic.c \
./../../core/drm_atomic_helper.c \
./../../core/drm_atomic_uapi.c \
./../../core/drm_auth.c \
./../../core/drm_blend.c \
./../../core/drm_bridge.c \
./../../core/drm_cache.c \
./../../core/drm_client.c \
./../../core/drm_client_modeset.c \
./../../core/drm_color_mgmt.c \
./../../core/drm_connector.c \
./../../core/drm_crtc.c \
./../../core/drm_crtc_helper.c \
./../../core/drm_dp_helper.c \
./../../core/drm_damage_helper.c \
./../../core/drm_drv.c \
./../../core/drm_dumb_buffers.c \
./../../core/drm_edid.c \
./../../core/drm_encoder.c \
./../../core/drm_fb_helper.c \
./../../core/drm_file.c \
./../../core/drm_flip_work.c \
./../../core/drm_fourcc.c \
./../../core/drm_framebuffer.c \
./../../core/drm_gem.c \
./../../core/drm_hashtab.c \
./../../core/drm_ioc32.c \
./../../core/drm_ioctl.c \
./../../core/drm_kms_helper_common.c \
./../../core/drm_memory.c \
./../../core/drm_mm.c \
./../../core/drm_mode_config.c \
./../../core/drm_mode_object.c \
./../../core/drm_modes.c \
./../../core/drm_modeset_helper.c \
./../../core/drm_modeset_lock.c \
./../../core/drm_panel.c \
./../../core/drm_panel_orientation_quirks.c \
./../../core/drm_plane.c \
./../../core/drm_plane_helper.c \
./../../core/drm_prime.c \
./../../core/drm_print.c \
./../../core/drm_probe_helper.c \
./../../core/drm_property.c \
./../../core/drm_rect.c \
./../../core/drm_self_refresh_helper.c \
./../../core/drm_scdc_helper.c \
./../../core/drm_syncobj.c \
./../../core/drm_vblank.c \
./../../core/drm_vma_manager.c \
./../../core/scheduler/sched_entity.c \
./../../core/scheduler/sched_fence.c \
./../../core/scheduler/sched_main.c \
./../../freebsd/drm_gem_cma_helper.c \
./../../freebsd/drm_gem_framebuffer_helper.c \
./../../freebsd/drm_fb_cma_helper.c \
./../../freebsd/drm_fbdev.c \
./../../freebsd/drm_os_freebsd.c \
./../../freebsd/drm_sysctl.c \
./../../freebsd/drm_sysfs.c \
./../../drmkpi/drmcompat_compat.c \
./../../drmkpi/drmcompat_completion.c \
./../../drmkpi/drmcompat_dma_buf.c \
./../../drmkpi/drmcompat_dma_fence.c \
./../../drmkpi/drmcompat_dma_fence_chain.c \
./../../drmkpi/drmcompat_dma_resv.c \
./../../drmkpi/drmcompat_idr.c \
./../../drmkpi/drmcompat_kthread.c \
./../../drmkpi/drmcompat_list_sort.c \
./../../drmkpi/drmcompat_lock.c \
./../../drmkpi/drmcompat_page.c \
./../../drmkpi/drmcompat_page1.c \
./../../drmkpi/drmcompat_rcu.c \
./../../drmkpi/drmcompat_schedule.c \
./../../drmkpi/drmcompat_slab.c \
./../../drmkpi/drmcompat_sync_file.c \
./../../drmkpi/drmcompat_timer.c \
./../../drmkpi/drmcompat_work.c \
./../../drmkpi/hdmi.c \
./../../drmkpi/drmcompat_sort.c \
./../../kmod.c

.include <bsd.kmod.mk>
