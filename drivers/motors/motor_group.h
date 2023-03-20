#pragma once

#include "motor.h"

typedef size_t (*motor_group_get_motor_count_t)(const struct device *dev);
typedef int (*motor_group_drive_continous_t)(const struct device *dev, struct motor_power_config power_configs[]);
typedef int (*motor_group_drive_all_continous_t)(const struct device *dev, struct motor_power_config power_config);
typedef int (*motor_group_drive_single_continous_t)(const struct device *dev, size_t index, struct motor_power_config power_config);

struct motor_group_api {
    motor_group_get_motor_count_t get_motor_count;
    motor_group_drive_continous_t drive_continous;
    motor_group_drive_all_continous_t drive_all_continous;
    motor_group_drive_single_continous_t drive_single_continous;
};

/**
 * @brief Get number of motors in group
 *
 * @param dev Motor group device
 * @return int Number of motors in group
 */
static inline size_t motor_group_get_motor_count(const struct device *dev) {
    const struct motor_group_api *api = (const struct motor_group_api *)dev->api;
    if (api->get_motor_count == NULL) {
        return -ENOTSUP;
    }
    return api->get_motor_count(dev);
}

/**
 * @brief Set power of all motors in the motor group. Number of power configs must match number of motors in group.
 *
 * @param dev Motor group device
 * @param power_configs Array of power configs. Number of power configs must match number of motors in group.
 * @return int 0 on success, negative errno code otherwise.
 */
static inline int motor_group_drive_continous(const struct device *dev, struct motor_power_config power_configs[]) {
    const struct motor_group_api *api = (const struct motor_group_api *)dev->api;
    if (api->drive_continous == NULL) {
        return -ENOTSUP;
    }
    return api->drive_continous(dev, power_configs);
}

/**
 * @brief Set power of all motors in the motor group to the same value.
 *
 * @param dev Motor group device
 * @param power_config Power config to be used for all motors in motor group.
 * @return int 0 on success, negative errno code otherwise.
 */
static inline int motor_group_drive_all_continous(const struct device *dev, struct motor_power_config power_config) {
    const struct motor_group_api *api = (const struct motor_group_api *)dev->api;
    if (api->drive_all_continous == NULL) {
        return -ENOTSUP;
    }
    return api->drive_all_continous(dev, power_config);
}

/**
 * @brief Set power of a single motor in the motor group.
 *
 * @param dev Motor group device
 * @param index Index of motor to set power for.
 * @param power_config Power config to be used for motor.
 * @return int 0 on success, negative errno code otherwise.
 */
static inline int motor_group_drive_single_continous(const struct device *dev, size_t index, struct motor_power_config power_config) {
    const struct motor_group_api *api = (const struct motor_group_api *)dev->api;
    if (api->drive_single_continous == NULL) {
        return -ENOTSUP;
    }
    return api->drive_single_continous(dev, index, power_config);
}