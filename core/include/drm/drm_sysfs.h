/* Public Domain */

void drm_sysfs_hotplug_event(struct drm_device *dev __unused);
int
drm_sysfs_connector_add(struct drm_connector *connector __unused);
void
drm_sysfs_connector_remove(struct drm_connector *connector __unused);

void
drm_sysfs_hotplug_event(struct drm_device *dev __unused);
struct cdev *
drm_sysfs_minor_alloc(struct drm_minor *minor);
