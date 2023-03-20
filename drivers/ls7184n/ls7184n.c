#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#define DT_DRV_COMPAT lsi_ls7184n
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ls7184_driver, CONFIG_LS7184N_DRIVER_LOG_LEVEL);

#define LS7184N_INIT_PRIORITY 60

#define DEGREES_PER_ROTATION 360

struct ls7184n_data {
    const struct device *parent_dev;
    atomic_t position;
    struct gpio_callback clk_gpio_cb;
    struct ls7184n_attrs {
        int32_t resolution;  // Set using SENSOR_ATTR_SAMPLING_FREQUENCY.
    } attrs;
    struct sensor_value fetched_value;
};

struct ls7184n_conf {
    struct gpio_dt_spec clk_gpio;
    struct gpio_dt_spec up_down_gpio;
};

static int sample_fetch(const struct device *dev, enum sensor_channel chan) {
    if (chan != SENSOR_CHAN_ROTATION && chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }
    struct ls7184n_data *data = (struct ls7184n_data *)dev->data;
    int32_t position = atomic_get(&data->position);
    data->fetched_value.val1 = DEGREES_PER_ROTATION * position / data->attrs.resolution;
    data->fetched_value.val2 = DEGREES_PER_ROTATION * (position % data->attrs.resolution) * 1000000 / data->attrs.resolution;
    if (&data->position < 0) {
        data->fetched_value.val2 *= -1;
    }
    return 0;
}

static int channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct ls7184n_data *data = (struct ls7184n_data *)dev->data;
    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }
    *val = data->fetched_value;
    return 0;
}

static int attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val) {
    struct ls7184n_data *data = (struct ls7184n_data *)dev->data;
    if (chan == SENSOR_CHAN_ROTATION) {
        switch (attr) {
            case SENSOR_ATTR_SAMPLING_FREQUENCY:
                data->attrs.resolution = val->val1;
                return 0;
            default:
                LOG_DBG("Attribute not supported: %d", attr);
                return -ENOTSUP;
        }
    }
    LOG_DBG("Channel not supported: %d", chan);
    return -ENOTSUP;
}

static int attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value *val) {
    struct ls7184n_data *data = (struct ls7184n_data *)dev->data;
    if (chan == SENSOR_CHAN_ROTATION) {
        switch (attr) {
            case SENSOR_ATTR_SAMPLING_FREQUENCY:
                val->val1 = data->attrs.resolution;
                val->val2 = 0;
                return 0;
            default:
                LOG_DBG("Attribute not supported: %d", attr);
                return -ENOTSUP;
        }
    }
    LOG_DBG("Channel not supported: %d", chan);
    return -ENOTSUP;
}

static struct sensor_driver_api api = {
    .sample_fetch = sample_fetch,
    .channel_get = channel_get,
    .attr_set = attr_set,
    .attr_get = attr_get,
    .trigger_set = NULL,
};

static void clk_gpio_int_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct ls7184n_data *data = CONTAINER_OF(cb, struct ls7184n_data, clk_gpio_cb);
    const struct device *dev = data->parent_dev;
    const struct ls7184n_conf *conf = (const struct ls7184n_conf *)dev->config;
    if (gpio_pin_get_dt(&conf->up_down_gpio)) {
        atomic_inc(&data->position);
    } else {
        atomic_dec(&data->position);
    }
}

static int init(const struct device *dev) {
    const struct ls7184n_conf *conf = (struct ls7184n_conf *)dev->config;
    struct ls7184n_data *data = (struct ls7184n_data *)dev->data;
    data->parent_dev = dev;

    int err = gpio_pin_configure_dt(&conf->clk_gpio, GPIO_INPUT);
    if (err != 0) {
        LOG_ERR("Failed to configure clock gpio: %d", err);
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&conf->clk_gpio, GPIO_INT_EDGE_FALLING);
    if (err != 0) {
        LOG_ERR("Failed to configure clock gpio interrupt: %d", err);
        return err;
    }
    gpio_init_callback(&data->clk_gpio_cb, clk_gpio_int_handler, BIT(conf->clk_gpio.pin));
    err = gpio_add_callback(conf->clk_gpio.port, &data->clk_gpio_cb);
    if (err != 0) {
        LOG_ERR("Failed to add clock gpio callback: %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(&conf->up_down_gpio, GPIO_INPUT);
    if (err != 0) {
        LOG_ERR("Failed to configure up/down gpio: %d", err);
        return err;
    }
    return 0;
}

#define INIT_LS7184N_MOTOR(inst)                                            \
    static struct ls7184n_conf conf_##inst = {                              \
        .clk_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), clk_gpios),         \
        .up_down_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), up_down_gpios), \
    };                                                                      \
    static struct ls7184n_data data_##inst = {                              \
        .parent_dev = NULL,                                                 \
        .position = ATOMIC_INIT(0),                                         \
        .clk_gpio_cb = {0},                                                 \
        .attrs = {                                                          \
            .resolution = DT_INST_PROP_OR(inst, resolution, 0),             \
        },                                                                  \
        .fetched_value = {0},                                               \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(                                                  \
        inst,                                                               \
        init,                                                               \
        NULL,                                                               \
        &data_##inst,                                                       \
        &conf_##inst,                                                       \
        POST_KERNEL,                                                        \
        LS7184N_INIT_PRIORITY,                                              \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INIT_LS7184N_MOTOR)
