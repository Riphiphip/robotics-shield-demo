
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "../motors/motor_group.h"
#include "./pm.h"

#define DT_DRV_COMPAT st_stspin240
#define STSPIN240_INIT_PRIORITY 61

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stspin240_driver, CONFIG_STSPIN240_DRIVER_LOG_LEVEL);

struct stspin240_data {
    uint8_t n_live_motors;  // Number of active motors. Used for power management
};

struct stspin240_conf {
    uint8_t n_motors;
    const struct device **motors;
    struct gpio_dt_spec enable_fault_gpio;
    struct gpio_dt_spec standby_reset_gpio;
    struct k_mutex *live_motor_update_mutex;
};

static size_t _get_motor_count(const struct device *dev) {
    const struct stspin240_conf *conf = (struct stspin240_conf *)dev->config;
    return conf->n_motors;
}

static int _drive_continous(const struct device *dev, struct motor_power_config power_configs[]) {
    const struct stspin240_conf *conf = (struct stspin240_conf *)dev->config;
    for (size_t i = 0; i < conf->n_motors; i++) {
        int err = motor_drive_continous(conf->motors[i], power_configs[i]);
        if (err != 0) {
            return err;
        }
    }
    return 0;
}

static int _drive_all_continous(const struct device *dev, struct motor_power_config power_config) {
    const struct stspin240_conf *conf = (struct stspin240_conf *)dev->config;
    for (size_t i = 0; i < conf->n_motors; i++) {
        int err = motor_drive_continous(conf->motors[i], power_config);
        if (err != 0) {
            return err;
        }
    }
    return 0;
}

static int _drive_single_continous(const struct device *dev, size_t index, struct motor_power_config power_config) {
    const struct stspin240_conf *conf = (struct stspin240_conf *)dev->config;
    if (index >= conf->n_motors) {
        return -EINVAL;
    }
    return motor_drive_continous(conf->motors[index], power_config);
}

static struct motor_group_api api = {
    .get_motor_count = _get_motor_count,
    .drive_continous = _drive_continous,
    .drive_all_continous = _drive_all_continous,
    .drive_single_continous = _drive_single_continous,
};

#ifdef CONFIG_STSPIN240_DEVICE_PM
// Device power management functionality
int inc_live_motor_count(const struct device *dev) {
    int err = 0;
    err = pm_device_runtime_get(dev);
    return err;
}

int dec_live_motor_count(const struct device *dev) {
    int err = 0;
    err = pm_device_runtime_put(dev);
    return err;
}

static int _pm_action(const struct device *dev, enum pm_device_action action) {
    const struct stspin240_conf *conf = (struct stspin240_conf *)dev->config;
    switch (action) {
        case PM_DEVICE_ACTION_SUSPEND:
            if (conf->standby_reset_gpio.port != NULL) {
                gpio_pin_set_dt(&conf->standby_reset_gpio, 0);
            }
            break;
        case PM_DEVICE_ACTION_RESUME:
            if (conf->standby_reset_gpio.port != NULL) {
                gpio_pin_set_dt(&conf->standby_reset_gpio, 1);
            }
            break;
        case PM_DEVICE_ACTION_TURN_ON:
            if (conf->enable_fault_gpio.port != NULL) {
                gpio_pin_set_dt(&conf->enable_fault_gpio, 1);
            }
            break;
        case PM_DEVICE_ACTION_TURN_OFF:
            if (conf->enable_fault_gpio.port != NULL) {
                gpio_pin_set_dt(&conf->enable_fault_gpio, 0);
            }
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}
#endif  // CONFIG_STSPIN240_DEVICE_PM

static int init(const struct device *dev) {
    const struct stspin240_conf *conf = (struct stspin240_conf *)dev->config;

    for (size_t i = 0; i < conf->n_motors; i++) {
        if (!device_is_ready(conf->motors[i])) {
            LOG_ERR("Motor %s in group %s is not ready", conf->motors[i]->name, dev->name);
            return -ENODEV;
        }
    }

    if (conf->enable_fault_gpio.port != NULL) {
        if (!device_is_ready(conf->enable_fault_gpio.port)) {
            LOG_ERR("Enable/fault GPIO of %s is not ready", dev->name);
            return -ENODEV;
        }
    }

    if (conf->standby_reset_gpio.port != NULL) {
        if (!device_is_ready(conf->standby_reset_gpio.port)) {
            LOG_ERR("Standby/reset GPIO of %s is not ready", dev->name);
            return -ENODEV;
        }
    }
    return 0;
}

// Used to count number of motors
#define CHILD_NODE_COUNTER(...) +1
#define CHILD_NODE_LIST_ELEMENT(child) DEVICE_DT_GET(child),

#define INIT_STSPIN240(inst)                                                                                  \
    PM_DEVICE_DT_INST_DEFINE(inst, _pm_action);                                                               \
    static const size_t motor_count_##inst = (0 DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, CHILD_NODE_COUNTER)); \
    static const struct device const *motors_##inst[] = {                                                     \
        DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, CHILD_NODE_LIST_ELEMENT)};                                    \
    K_MUTEX_DEFINE(live_motor_update_mutex_##inst);                                                           \
    static struct stspin240_conf conf_##inst = {                                                              \
        .n_motors = motor_count_##inst,                                                                       \
        .motors = motors_##inst,                                                                              \
        .enable_fault_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), enable_fault_gpios, {0}),                 \
        .standby_reset_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), standby_reset_gpios, {0}),               \
        .live_motor_update_mutex = &live_motor_update_mutex_##inst,                                           \
    };                                                                                                        \
    static struct stspin240_data data_##inst = {};                                                            \
    DEVICE_DT_INST_DEFINE(                                                                                    \
        inst,                                                                                                 \
        init,                                                                                                 \
        PM_DEVICE_DT_INST_GET(inst),                                                                             \
        &data_##inst,                                                                                         \
        &conf_##inst,                                                                                         \
        APPLICATION,                                                                                          \
        STSPIN240_INIT_PRIORITY,                                                                              \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INIT_STSPIN240)