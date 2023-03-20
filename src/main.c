/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "../drivers/motors/motor_group.h"

#define MODULE main

LOG_MODULE_REGISTER(MODULE);

void main(void) {
    const struct device *motor_port_0_dev = DEVICE_DT_GET(DT_PATH(pca63557_motor_port_0));
    const struct device *motor_port_1_dev = DEVICE_DT_GET(DT_PATH(pca63557_motor_port_1));

    const struct device *rgb_led_dev = DEVICE_DT_GET(DT_PATH(pca63557_pwmleds));

    const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(pca63557_bmi270));

    if (!device_is_ready(motor_port_0_dev)) {
        LOG_ERR("Device %s is not ready", motor_port_0_dev->name);
        return;
    }

    if (!device_is_ready(motor_port_1_dev)) {
        LOG_ERR("Device %s is not ready", motor_port_1_dev->name);
        return;
    }

    if (!device_is_ready(rgb_led_dev)) {
        LOG_ERR("Device %s is not ready", rgb_led_dev->name);
        return;
    }

    bool imu_ready = device_is_ready(imu_dev);

    if (!imu_ready) {
        LOG_ERR("Device %s is not ready", imu_dev->name);
        return;
    }

    struct sensor_value full_scale, sampling_freq, oversampling;
    full_scale.val1 = 2;
    full_scale.val2 = 0;
    sampling_freq.val1 = 100;
    sampling_freq.val2 = 0;
    oversampling.val1 = 1;
    oversampling.val2 = 0;

    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    /* Setting scale in degrees/s to match the sensor scale */
    full_scale.val1 = 500; /* dps */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100; /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 1; /* Normal mode */
    oversampling.val2 = 0;

    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    const struct device *qdec_0 = DEVICE_DT_GET(DT_PATH(pca63557_qdec_port_0));
    if (!device_is_ready(qdec_0)) {
        LOG_ERR("Device %s is not ready", qdec_0->name);
        return;
    }

    struct sensor_value ticks_per_rot = {.val1 = 576, .val2 = 0};
    sensor_attr_set(qdec_0, SENSOR_CHAN_ROTATION, SENSOR_ATTR_SAMPLING_FREQUENCY, &ticks_per_rot);


    led_on(rgb_led_dev, 0);

    LOG_DBG("Motor port 0 device is %p, name is %s, and it controls %d motors", motor_port_0_dev, motor_port_0_dev->name, motor_group_get_motor_count(motor_port_0_dev));

    motor_group_drive_all_continous(motor_port_0_dev, (struct motor_power_config){
                                                          .numerator = 0,
                                                          .denominator = 100,
                                                          .direction = MOTOR_DIRECTION_CLOCKWISE,
                                                      });

    motor_group_drive_all_continous(motor_port_1_dev, (struct motor_power_config){
                                                          .numerator = 0,
                                                          .denominator = 100,
                                                          .direction = MOTOR_DIRECTION_CLOCKWISE,
                                                      });

    while (true) {
        sensor_sample_fetch(imu_dev);
        struct sensor_value accel[3];
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        struct sensor_value gyro[3];
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        LOG_DBG("Accelerometer: %d.%06d, %d.%06d, %d.%06d", accel[0].val1, NRFX_ABS(accel[0].val2), accel[1].val1, NRFX_ABS(accel[1].val2), accel[2].val1, NRFX_ABS(accel[2].val2));
        LOG_DBG("Gyroscope: %d.%06d, %d.%06d, %d.%06d", gyro[0].val1, NRFX_ABS(gyro[0].val2), gyro[1].val1, NRFX_ABS(gyro[1].val2), gyro[2].val1, NRFX_ABS(gyro[2].val2));
        sensor_sample_fetch(qdec_0);
        struct sensor_value qdec_0_val;
        sensor_channel_get(qdec_0, SENSOR_CHAN_ROTATION, &qdec_0_val);
        LOG_DBG("QDEC 0: %d", qdec_0_val.val1);
        k_sleep(K_MSEC(500));
    }
}
