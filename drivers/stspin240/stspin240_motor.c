
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "../motors/motor.h"
#include "./pm.h"

#define DT_DRV_COMPAT st_stspin240_motor
#define STSPIN240_MOTOR_INIT_PRIORITY 60

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stspin240_motor_driver, CONFIG_STSPIN240_MOTOR_DRIVER_LOG_LEVEL);

struct motor_data {
    struct motor_power_config power_config_current;
};

struct motor_conf {
    struct gpio_dt_spec phase_gpio;
    struct pwm_dt_spec pwm;
    const struct device *stspin240_dev;
};

static int _drive_continous(const struct device *dev, struct motor_power_config power_config) {
    int err;
    uint32_t pulse;

    const struct motor_conf *conf = (struct motor_conf *)dev->config;
    struct motor_data *data = (struct motor_data *)dev->data;

    uint8_t power_numerator = power_config.numerator;
    uint8_t power_denominator = power_config.denominator;
    enum motor_direction direction = power_config.direction;
    switch (direction) {
        case MOTOR_DIRECTION_CLOCKWISE:
            err = gpio_pin_set_dt(&conf->phase_gpio, 1);
            break;
        case MOTOR_DIRECTION_COUNTER_CLOCKWISE:
            err = gpio_pin_set_dt(&conf->phase_gpio, 0);
            break;
        default:
            LOG_ERR("Invalid motor direction %d", direction);
            return -EINVAL;
    }

    pulse = (conf->pwm.period / power_denominator) * power_numerator;

    err = pwm_set_pulse_dt(&conf->pwm, pulse);
    if (err) {
        LOG_ERR("Failed to set PWM pulse: Error %d", err);
        return err;
    }

    if (conf->stspin240_dev != NULL && IS_ENABLED(CONFIG_STSPIN240_DEVICE_PM)) {
        if (data->power_config_current.numerator == 0 && power_config.numerator > 0) {
            inc_live_motor_count(conf->stspin240_dev);
        } else if (data->power_config_current.numerator > 0 && power_config.numerator == 0) {
            dec_live_motor_count(conf->stspin240_dev);
        }
    }

    data->power_config_current = power_config;

    LOG_DBG("Setting power on motor %s to %d/%d with pulse width %d and period %d", dev->name, power_numerator, power_denominator, pulse, conf->pwm.period);
    return 0;
}

int _get_power_config(const struct device *dev, struct motor_power_config *power_config) {
    struct motor_data *data = (struct motor_data *)dev->data;
    *power_config = data->power_config_current;
    return 0;
}

static struct motor_api api = {
    .drive_continous = _drive_continous,
    .get_power_config = _get_power_config,
};

static int init_gpio(const struct device *dev) {
    struct motor_conf *conf = (struct motor_conf *)dev->config;
    int err = 0;

    if (conf->phase_gpio.port != NULL) {
        if (!device_is_ready(conf->phase_gpio.port)) {
            LOG_ERR("Phase gpio, %s, is not ready", conf->phase_gpio.port->name);
            return -ENODEV;
        }

        err = gpio_pin_configure_dt(&conf->phase_gpio, GPIO_OUTPUT);
        if (err) {
            LOG_ERR("Failed to configure phase gpio: Error %d", err);
            return err;
        }
    }

    return 0;
}

static int init_pwm(const struct device *dev) {
    int err;
    struct motor_conf *conf = (struct motor_conf *)dev->config;
    if (!device_is_ready(conf->pwm.dev)) {
        LOG_ERR("PWM device not found or not ready");
        return -ENODEV;
    }
    err = pwm_set_dt(&conf->pwm, conf->pwm.period, 0);
    if (err) {
        LOG_ERR("Failed to set PWM period %d ns: Error %d", conf->pwm.period, err);
        return err;
    }
    return 0;
}

static int init_motor(const struct device *dev) {
    int err;
    err = init_gpio(dev);
    if (err) {
        LOG_ERR("Error while initializig gpio: %d", err);
        return err;
    }
    err = init_pwm(dev);
    if (err) {
        LOG_ERR("Error while initializig pwm: %d", err);
        return err;
    }
    return 0;
}

#define INIT_STSPIN240_MOTOR(inst)                                                                                                              \
    BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_INST_PARENT(inst), st_stspin240), "Parent of st,stspin240-motor must have compatible=\"st,stspin240\""); \
    static struct motor_conf conf_##inst = {                                                                                                    \
        .phase_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, phase_gpios, {0}),                                                                         \
        .pwm = PWM_DT_SPEC_GET_BY_IDX(DT_INST(inst, DT_DRV_COMPAT), 0),                                                                         \
        .stspin240_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PARENT(inst)),                                                                           \
    };                                                                                                                                          \
    static struct motor_data data_##inst = {                                                                                                    \
        .power_config_current = {0},                                                                                                            \
    };                                                                                                                                          \
    DEVICE_DT_INST_DEFINE(                                                                                                                      \
        inst,                                                                                                                                   \
        init_motor,                                                                                                                             \
        NULL,                                                                                                                                   \
        &data_##inst,                                                                                                                           \
        &conf_##inst,                                                                                                                           \
        APPLICATION,                                                                                                                            \
        STSPIN240_MOTOR_INIT_PRIORITY,                                                                                                          \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INIT_STSPIN240_MOTOR);