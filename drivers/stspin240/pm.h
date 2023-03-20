
#ifdef CONFIG_STSPIN240_DEVICE_PM
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
int inc_live_motor_count(const struct device *dev);
int dec_live_motor_count(const struct device *dev);
#endif

