# ESP32 BNO08x Calibration

This project tests the calibration feature of the BNO08x library using an ESP32. For more details, visit the [GitHub repository](https://github.com/myles-parfeniuk/esp32_BNO08x).

## Summary

- Click the boot button of ESP32 board (GPIO0) to start calibration.
- calibration_start() should return, instead it hangs. so no chance to call calibration_stop().


