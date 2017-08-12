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

/*#define DEFAULT_MPU_HZ  (20)
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

static struct hal_s hal = {0};*/

IMU::IMU()
{
    init();
}

void IMU::init()
{
    trace_puts("Configuring MPU9250");

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

    if (result)
    {
        MPL_LOGE("Could not start the MPL.\n");
    }

    /*mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

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

    //get_tick_count(&timestamp);

    //dmp_load_motion_driver_firmware();
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
    hal.dmp_on = 1;*/

    trace_puts("MPU9250 configured");
}
