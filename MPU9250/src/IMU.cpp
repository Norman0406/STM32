#include "IMU.h"

extern "C"
{
    #include "stm32f4xx_hal.h"
    #include "inv_mpu.h"
    #include "inv_mpu_dmp_motion_driver.h"
    #include "invensense.h"
    #include "invensense_adv.h"
    #include "eMPL_outputs.h"
    #include "mltypes.h"
    #include "mpu.h"
    #include "log.h"
}

unsigned char* mpl_key = (unsigned char*)"eMPL 5.1";

#define DEFAULT_MPU_HZ  (100)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

static struct hal_s hal = {0};

IMU::IMU()
{
}

void IMU::init()
{
    trace_puts("Configuring MPU9250");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    inv_error_t result;
    struct int_param_s int_param;

    result = mpu_init(&int_param);
    if (result)
    {
        trace_puts("Could not initialize gyro.\n");
    }

    result = inv_init_mpl();
    if (result)
    {
        trace_puts("Could not initialize MPL.\n");
    }

    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();

    inv_enable_fast_nomot();

    inv_enable_gyro_tc();

    // compass calibration
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();

    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        while (1)
        {
            MPL_LOGE("Not authorized.\n");
        }
    }

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    mpu_set_compass_sample_rate(COMPASS_READ_MS / COMPASS_READ_MS);

    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    unsigned short compass_fsr;
    mpu_get_compass_fsr(&compass_fsr);

    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);

    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);

    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;

    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    hal.dmp_features =
            DMP_FEATURE_6X_LP_QUAT |
            DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL;

    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    short lastAccel = 0;

    trace_puts("MPU9250 configured");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void IMU::process()
{
    unsigned long timestamp;
    timestamp = HAL_GetTick();

    int new_compass = 0;
    int new_data = 0;
    int new_temp = 1;
    unsigned long sensor_timestamp;

    if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
        hal.new_gyro && (hal.sensors & COMPASS_ON)) {
        hal.next_compass_ms = timestamp + COMPASS_READ_MS;
        new_compass = 1;
    }
    new_compass = 1;

    if (timestamp > hal.next_temp_ms) {
        hal.next_temp_ms = timestamp + TEMP_READ_MS;
        new_temp = 1;
    }

    if (hal.lp_accel_mode)
    {
        short accel_short[3];
        long accel[3];
        mpu_get_accel_reg(accel_short, &sensor_timestamp);
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
        hal.new_gyro = 0;
    }
    else if (hal.dmp_on)
    {
        short gyro[3], accel_short[3], sensors;
        unsigned char more;
        long accel[3], quat[4], temperature;

        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

        if (!more)
            hal.new_gyro = 0;

        if (sensors & INV_XYZ_GYRO) {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            new_data = 1;
            if (new_temp) {
                new_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }

        if (sensors & INV_XYZ_ACCEL) {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
        }

        if (sensors & INV_WXYZ_QUAT) {
            inv_build_quat(quat, 0, sensor_timestamp);
            new_data = 1;
        }
    }
    else
    {
        short gyro[3], accel_short[3];
        unsigned char sensors, more;
        long accel[3], temperature;

        hal.new_gyro = 0;
        mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
            &sensors, &more);
        if (more)
            hal.new_gyro = 1;
        if (sensors & INV_XYZ_GYRO) {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            new_data = 1;
            if (new_temp) {
                new_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }
        if (sensors & INV_XYZ_ACCEL) {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
        }
    }

    if (new_compass) {
        short compass_short[3];
        long compass[3];
        new_compass = 0;
        if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
            compass[0] = (long)compass_short[0];
            compass[1] = (long)compass_short[1];
            compass[2] = (long)compass_short[2];
            /* NOTE: If using a third-party compass calibration library,
             * pass in the compass data in uT * 2^16 and set the second
             * parameter to INV_CALIBRATED | acc, where acc is the
             * accuracy from 0 to 3.
             */
            inv_build_compass(compass, 0, sensor_timestamp);
        }
        new_data = 1;
    }

    if (new_data)
    {
//            long data[9];
//            int8_t accuracy;
//            unsigned long timestamp;
//            float float_data[3] = {0};
//
//            inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_compass(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_rot_mat(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp);
//            inv_get_sensor_type_gravity(float_data, &accuracy, (inv_time_t*)&timestamp);

        /*if (abs(lastAccel - accel_short[0]) > 1000)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }*/
    }
}
