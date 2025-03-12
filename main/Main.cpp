#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "Main.h"
#include "BNO08x.hpp"
#include <driver/gpio.h>
#include <sys/dirent.h>
#include <sys/stat.h>

static const char tag[] = "Main";

// BNO086 SPI interface
#define CONFIG_BNO086_SPI2_CLOCK GPIO_NUM_9
#define CONFIG_BNO086_SPI2_DATA_OUT GPIO_NUM_8
#define CONFIG_BNO086_SPI2_DATA_IN GPIO_NUM_7
#define CONFIG_BNO086_SPI2_SELECT GPIO_NUM_6
#define CONFIG_BNO086_INT_PIN GPIO_NUM_41 // The H_INTN pin is the application interrupt line that indicates the BNO08X requires attention. This should be
                                          // tied to a GPIO with wake capability. The interrupt is active low.
#define CONFIG_BNO086_RESET_PIN GPIO_NUM_42

#define BNO08X_USE_ROTATION_VECTOR 1                                         // uses also gyro for more accurate rotation, but it might introduce drift
#define BNO08X_USE_GEOMAGNETIC_ROTATION_VECTOR (!BNO08X_USE_ROTATION_VECTOR) // uses only magnetometer and accelerometer

#define BNO08X_SPI_FREQ (1 * 1000 * 1000)          // BNO08X allows max 3 MHz
#define BNO08X_TIME_BETWEEN_REPORTS (200 * 1000UL) // 200ms
#define BNO08X_INSTALL_GLOBAL_ISR true            // other task will already have installed the ISR before this task starts

static BNO08x bno(bno08x_config_t(SPI2_HOST,
    CONFIG_BNO086_SPI2_DATA_OUT,
    CONFIG_BNO086_SPI2_DATA_IN,
    CONFIG_BNO086_SPI2_CLOCK,
    CONFIG_BNO086_SPI2_SELECT,
    CONFIG_BNO086_INT_PIN,
    CONFIG_BNO086_RESET_PIN,
    BNO08X_SPI_FREQ,
    BNO08X_INSTALL_GLOBAL_ISR));


extern "C"
{
    void app_main();
}

void app_main()
{
    Main main;
    main.Run();
}

void RotationVectorCallback()
{
    static int count = 0;
    float roll, pitch, yaw;
    bno08x_euler_angle_t euler = bno.rpt.rv.get_euler();

    float accuracyQuat = euler.rad_accuracy;

    // Pitch: The rotation around the lateral axis (side-to-side axis). 
    //        Positive pitch tilts the bow upward (nose up) and the stern downward, 
    //        while negative pitch points the bow downward (nose down) and the stern upward.
    // Roll:  The rotation around the longitudinal axis (front-to-back axis). 
    //        Positive roll tilts the starboard hull (right hull) downward and the port hull (left hull) upward, 
    //        akin to a clockwise motion when viewed from the rear.
    // Yaw:   The rotation around the vertical axis. 
    //        Positive yaw moves the bow to starboard (to the right) and the stern to port (to the left), 
    //        which adjusts the craft's heading without tilting or pitching.

    // clang-format off    
    #define MOUNTING_POSITION_COCKPIT 0
    #define MOUNTING_POSITION_COMPUTER 0

    #if MOUNTING_POSITION_COCKPIT
        pitch = euler.x + 270.0;
        pitch = pitch > 180.0 ? -pitch + 360.0 : -pitch;
        roll = -euler.y;
        yaw = 180.0 - euler.z;
        if (yaw < 0)
        {
            yaw += 360.0;
        }
    #elif MOUNTING_POSITION_COMPUTER
        pitch = euler.x + 180.0;
        pitch = pitch > 180.0 ? - pitch + 360.0 : - pitch;
        roll = -euler.y;
        yaw = 180.0 - euler.z;
        if (yaw < 0)
        {
            yaw += 360.0;
        }
    #else
        pitch = euler.x;
        roll = euler.y;
        yaw = euler.z;
        if (yaw < 0)
        {
            yaw += 360.0;
        }   
    #endif
    // clang-format on

    bno08x_magf_t mag = bno.rpt.cal_magnetometer.get();
    bno08x_gyro_t gyro = bno.rpt.cal_gyro.get();
    bno08x_accel_t accel = bno.rpt.accelerometer.get();


    if (count++ > 4)
    {
        printf("\rrv:: mag: %4s, accel: %4s, gyro: %4s, quat: %4s, roll: %5.1f°, pitch: %5.1f°, yaw: %5.1f°, accuracy: %5.1f°        ",
            BNO08x::accuracy_to_str(mag.accuracy),
            BNO08x::accuracy_to_str(accel.accuracy),
            BNO08x::accuracy_to_str(gyro.accuracy),
            BNO08x::accuracy_to_str(euler.accuracy),
               roll,
               pitch,
               yaw,
               accuracyQuat);
        fflush(stdout);
        fsync(fileno(stdout));
        count = 0;
    }
}


void Main::Run()
{



    if (bno.initialize())
    {
        ESP_LOGI(tag, "BNO08x initialized. press [boot] button to calibrate");
    }
    else
    {
        ESP_LOGE(tag, "BNO08x initialization failed");
    }

    // if (bno.hard_reset()) // delay is built
    // {
    //     ESP_LOGI(tag, "BNO08x hard reset done");
    // }
    // else
    // {
    //     ESP_LOGE(tag, "BNO08x hard reset failed");
    //     return;
    // }

    // vTaskDelay(100 / portTICK_PERIOD_MS);

    if (bno.dynamic_calibration_disable(BNO08xCalSel::all))
    {
        ESP_LOGI(tag, "Dynamic calibration disabled");
    }
    else
    {
        ESP_LOGE(tag, "Failed to disable dynamic calibration");
    }

    ESP_LOGI(tag, "Enabling Rotation Vector");
    bno.rpt.rv.register_cb(RotationVectorCallback);
    if (bno.rpt.rv.enable(BNO08X_TIME_BETWEEN_REPORTS))
    {
        ESP_LOGI(tag, "Rotation Vector enabled");
    }
    else
    {
        ESP_LOGE(tag, "Failed to enable Rotation Vector");
    }

    if (bno.rpt.cal_magnetometer.enable(BNO08X_TIME_BETWEEN_REPORTS))
    {
        ESP_LOGI(tag, "Calibrated Magnetometer enabled");
    }
    else
    {
        ESP_LOGE(tag, "Failed to enable Calibrated Magnetometer");
    }
    

    if (bno.rpt.cal_magnetometer.enable(BNO08X_TIME_BETWEEN_REPORTS))
    {
        ESP_LOGI(tag, "Calibrated Magnetometer enabled");
    }
    else
    {
        ESP_LOGE(tag, "Failed to enable Calibrated Magnetometer");
    }

    if (bno.rpt.cal_gyro.enable(BNO08X_TIME_BETWEEN_REPORTS))
    {
        ESP_LOGI(tag, "Calibrated Gyroscope enabled");
    }
    else
    {
        ESP_LOGE(tag, "Failed to enable Calibrated Gyroscope");
    }

    if (bno.rpt.accelerometer.enable(BNO08X_TIME_BETWEEN_REPORTS))
    {
        ESP_LOGI(tag, "Accelerometer enabled");
    }
    else
    {
        ESP_LOGE(tag, "Failed to enable Accelerometer");
    }

    bool isCalibrating = false;
    while (true)
    {
        // check boot button to start calibration
        if ((gpio_get_level(GPIO_NUM_0) == 0) && (isCalibrating == false))
        {

            ESP_LOGW(tag, "Boot button pressed, attempting to start IMU calibration...");
            if (bno.calibration_start(BNO08X_TIME_BETWEEN_REPORTS)) // 100ms
            {
                isCalibrating = true;
                ESP_LOGI(tag, "IMU calibration started");
            }
            else
            {
                isCalibrating = false;
                ESP_LOGE(tag, "Starting IMU calibration failed");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
