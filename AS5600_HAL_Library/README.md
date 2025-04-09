AS5600 Sensor Library
=========================

Overview
--------

The **AS5600 HAL LIBRARY** project demonstrates the initialization and calibration process of the AS5600 magnetic rotary position sensor using the ESP32-S3 microcontroller. This code configures the sensor via I²C and analog output (OUT pin), allowing precise angle readings through ADC integration.

Features
--------

* **Sensor Configuration:** Sets power mode, hysteresis, output stage, filter settings, and watchdog for optimal performance.
* **One-Shot Calibration Timer:** Uses `esp_timer` to automate a multi-step calibration sequence.
* **ADC Angle Measurement:** Reads analog output to estimate the magnetic angle.
* **Safe EEPROM Commands (Commented):** Burn configuration and angle commands included with clear caution.

Hardware Components
-------------------

* **AS5600 Magnetic Sensor:** Provides angular position based on magnetic field.
* **ESP32-S3:** Handles I²C communication, ADC reading, and timer-based task scheduling.
* **Pull-up Resistors (4.7kΩ):** Required on SDA and SCL lines for I²C communication.
* **External Magnet:** Placed above the sensor shaft to provide angular position data.

Software Design
---------------

* **Initialization:**
  * Sets up I²C and OUT GPIOs.
  * Applies sensor configuration via the `AS5600_SetConf()` function.
  
* **Calibration Process:**
  * Timer-based sequence controlling GPIO initialization and deinitialization.
  * Guides user to position magnet at maximum angle.
  * Reads and prints angle values over 100 samples using ADC.

* **Configuration Verification:**
  * Confirms the written configuration by reading back the config register.
  
* **EEPROM Burn Commands (Optional):**
  * `AS5600_BurnSettingCommand()` – Permanently saves configuration.
  * `AS5600_BurnAngleCommand()` – Permanently saves current position as start/end angle.

Power Management
----------------

The system utilizes the ESP32’s low-power timer to delay calibration steps, helping reduce active time and avoid unnecessary power consumption when idle.

Installation
------------

1. Clone the repository, and use the sensor driver you want:

```bash
git clone https://github.com/nparra-code/robosync-robocup.git
```

2. Navigate to the project directory:

    ```bash
    cd AS5600_HAL_LIBRARY
    ```

3. Build and flash using your preferred ESP-IDF environment.

Usage
-----

1. Power the ESP32-S3 board and ensure the AS5600 sensor is connected properly.

2. Follow the console instructions during calibration:
    * Move the magnet to the **maximum angle** when prompted.
    * The system will automatically control GPIOs and capture analog angle data.

3. Observe the printed angles on the serial monitor.

4. If satisfied with the configuration and angle, uncomment the EEPROM burn commands **with caution** to permanently store settings.

License
-------

This project is licensed under the MIT License. See the LICENSE file for details.

