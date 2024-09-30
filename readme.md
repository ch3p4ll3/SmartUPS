## **README: SmartUPS Project with INA226 Sensor**

**Overview**

This repository contains the code for a smart UPS project built using an ESP32 microcontroller, incorporating a relay, an INA226 current sensor, an DS3231 real-time clock (RTC) module, MQTT notifications for power outages and restores, and automatic time synchronization.

**Components**

* **ESP32 microcontroller:** The brain of the project, handling all the computations and communications.
* **Relay:** Connected directly to the grid to monitor power outages.
* **INA226 current sensor:** Measures the battery voltage and current to calculate power consumption and remaining battery life.
* **DS3231:** Provides accurate timekeeping and date information, even during power outages.

**Functionality**

1. **Power Outage Detection:**
    * The relay monitors the grid power supply.
    * When a power outage is detected, the last outage time is recorded using the DS3231.
    * An MQTT message is published to a designated topic indicating the start of the outage.

2. **Power Restore Detection:**
    * When the grid power is restored, an MQTT message is published to a designated topic indicating the end of the outage.

3. **Battery Monitoring:**
   * The INA226 sensor measures the battery voltage and current.
   * Calculations are made to determine power consumption, remaining battery life, and battery status.

4. **Data Transmission:**
   * Every 10 seconds, the ESP32 publishes an MQTT message containing the following information:
     * `lastOutageTime`: Timestamp of the last power outage (from the DS3231).
     * `isOutagePresent`: Boolean indicating if an outage is currently in progress.
     * `time`: Current timestamp (from the DS3231).
     * `voltage`: Battery voltage (V).
     * `current`: Battery current (A).
     * `power`: Power consumption (W).
     * `shuntVoltage`: Shunt voltage of the INA226 sensor (mV).
     * `batteryLeft`: Estimated remaining battery life (%).
     * `powerLeft`: Estimated remaining power (Wh).
     * `batteryStatus`: Battery status (Ah).

5. **Outage Duration Monitoring:**
   * If the power outage exceeds a predefined threshold, the ESP32 publishes a message to a "shutdown" topic, indicating that a shutdown action should be initiated.

6. **Automatic Time Synchronization:**

    * Upon ESP32 boot, the DS3231 module is automatically set to the current UTC time using an NTP server. This ensures accurate timekeeping even after power outages.

**Usage**

1. **Hardware Setup:**
   * Connect the ESP32, relay, INA226 sensor, and DS3231 module according to the provided schematics.
   * Ensure proper power supply and connections.

2. **Software Configuration:**
   * Modify the MQTT broker address, port, and topic names in the code to match your network setup.
   * Adjust the outage duration threshold as needed.

3. **Upload Code:**

**Additional Notes**

* The DS3231 module provides accurate timekeeping, even during power outages, ensuring reliable data logging and outage duration tracking.
* For more advanced features, consider adding a battery charger, temperature sensor, or remote control capabilities.
* Customize the code to fit your specific requirements and preferences.
