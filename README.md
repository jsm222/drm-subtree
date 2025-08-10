How to use :
For building as modules make sure your running kernels source is in /usr/src then go to drm-subtree/modules and type 

``` 
make && make install
```

on rockpro64: (the order of loading is significant)

```
kldload /boot/modules/drm_kmod.ko 
kldload /boot/modules/rk_dw_hdmi.ko
kldload /boot/modules/rk_vop.ko
kldload /boot/modules/rk_drm.ko
```


or set kld_list accordinly
on pinebook pro: (the order of loading is significant)

```
kldload /boot/modules/drm_kmod.ko 
kldload /boot/modules/rk_andxp.ko
kldload /boot/modules/rk_vop.ko
kldload /boot/modules/rk_drm.ko
```
Below is not yet tested on this branch..
or set kld_list accordinly
From a checkout freebsd git repo :
- git checkout -b drm-base-subtree
- git remote add drm-subtree https://github.com/evadot/drm-subtree.git
- git subtree add --prefix sys/dev/drm/ drm-subtree master
- git am sys/dev/drm/extra_patches/*.patch

To update:
 - git fetch drm-subtree
 - git subtree pull --prefix sys/dev/drm/ drm-subtree master
 - Check if there is any new patches in extra_patches and git am them

When working on the main freebsd branch every commit will be in the main freebsd
repository, this is how subtree works.
After doing a commit, to update the drm-subtree submodule do :
 - git subtree push --prefix sys/dev/drm/ drm-subtree master
And update again to have the latest changes :
 - git subtree pull --prefix sys/dev/drm/ drm-subtree master
Commit will appear twice in git log which is a bit weird so it might be better to commit
directly to this repository.

DRMKPI todos:
 - Remove struct task_struct and usage of td->td_lkpi_task
 - Finish checking that it doesn't conflict with linuxkpi
