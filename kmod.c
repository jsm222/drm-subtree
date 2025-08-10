#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>

static int drm_handler(module_t mod, int /*modeventtype_t*/ what,
                       void *arg) {
return 0;
}

static moduledata_t kmod_data= {
         "drm",
         drm_handler,
         NULL
};

MODULE_VERSION(drm_kmod, 1);

DECLARE_MODULE(drm_kmod, kmod_data, SI_SUB_EXEC, SI_ORDER_ANY);

