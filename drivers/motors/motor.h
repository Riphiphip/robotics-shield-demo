#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>

enum motor_direction {
    MOTOR_DIRECTION_CLOCKWISE,
    MOTOR_DIRECTION_COUNTER_CLOCKWISE,
};

struct motor_power_config {
    uint8_t numerator;
    uint8_t denominator;
    enum motor_direction direction;
};

typedef int (*drive_continous_t)(const struct device *dev, struct motor_power_config power_config);

typedef int (*set_position_t)(const struct device *dev, int position, bool hold, struct motor_power_config power_config);

typedef int (*get_power_config_t)(const struct device *dev, struct motor_power_config *power_config_dest);

struct motor_api {
    drive_continous_t drive_continous;
    set_position_t set_position;
    get_power_config_t get_power_config;
};

/**
 * @brief Set power of motor
 *
 * @param dev Motor device
 * @param power Power of the motor. Negative values give same power in oposite direction.
 *              Relationship between power parameter and actual output from the motor depends
 *              on the underlying driver
 * @return 0 on success, negative errno code otherwise.
 *         -ENOTSUP if motor does not support continous rotation.
 *         Other error codes are defined by the underlying driver.
 */
static inline int motor_drive_continous(const struct device *dev, struct motor_power_config power_config) {
    const struct motor_api *api = (struct motor_api *)dev->api;
    if (api->drive_continous == NULL) {
        return -ENOTSUP;
    }
    return api->drive_continous(dev, power_config);
}

/**
 * @brief Set position of motor.
 *
 * @param dev Motor device
 * @param position Position the motor shoul be moved to.
 * @param power Power with which to move the motor. Negative values move the motor towards
 *              Relationship between power parameter and actual output from the motor depends
 *              on the underlying driver
 * @param hold Attempts to hold position when reached if true. Powers off motor if false.
 * @return 0 on success, negative errno code otherwise.
 *         -ENOTSUP if motor does not support moving to a specific position.
 *         -ENOTSUP if hold == true and the motor does not support holding the motor position.
 *         Other error codes are defined by the underlying driver.
 */
static inline int motor_set_position(const struct device *dev, int position, bool hold, struct motor_power_config power_config) {
    const struct motor_api *api = (struct motor_api *)dev->api;

    if (api->set_position == NULL) {
        return -ENOTSUP;
    }

    return api->set_position(dev, position, hold, power_config);
}

/**
 * @brief Get current power configuration of motor.
 *
 * @param dev Motor device
 * @param power_config_dest Pointer to struct where the current power configuration will be stored.
 * @return 0 on success, negative errno code otherwise.
 *         -ENOTSUP if motor does not support getting the current power configuration.
 *         Other error codes are defined by the underlying driver.
 */
static inline int motor_get_power_config(const struct device *dev, struct motor_power_config *power_config_dest) {
    const struct motor_api *api = (struct motor_api *)dev->api;

    if (api->get_power_config == NULL) {
        return -ENOTSUP;
    }

    return api->get_power_config(dev, power_config_dest);
}