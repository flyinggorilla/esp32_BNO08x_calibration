/**
 * @file BNO08xRptUncalGyro.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptUncalGyro.hpp"

/**
 * @brief Updates uncalibrated gyro data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptUncalGyro::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data = sensor_val->un.gyroscopeUncal;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    bias_data = sensor_val->un.gyroscopeUncal;
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables uncalibrated gyro reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptUncalGyro::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent uncalibrated gyroscope data, units are in rad/s.
 *
 * @param vel Reference to save velocity data.
 * @param bias Reference to save bias data.
 *
 * @return void, nothing to return
 */
void BNO08xRptUncalGyro::get(bno08x_gyro_t& vel, bno08x_gyro_bias_t& bias)
{
    lock_user_data();
    vel = data;
    bias = bias_data;
    unlock_user_data();
}

/**
 * @brief Grabs most recent uncalibrated gyroscope velocity data, units are in rad/s.
 *
 * @return Struct containing requested data.
 */
bno08x_gyro_t BNO08xRptUncalGyro::get_vel()
{
    lock_user_data();
    bno08x_gyro_t rqdata = data;
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs most recent uncalibrated gyroscope bias data, units are in rad/s.
 *
 * @return Struct containing requested data.
 */
bno08x_gyro_bias_t BNO08xRptUncalGyro::get_bias()
{
    lock_user_data();
    bno08x_gyro_bias_t rqdata = bias_data;
    unlock_user_data();
    return rqdata;
}