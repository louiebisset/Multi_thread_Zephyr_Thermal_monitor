# Multi-threaded Thermal Monitor (Zephyr RTOS)

A temperature monitor for the nRF54L15 DK, built on Zephyr RTOS. It reads an analog temperature sensor every 100 ms, keeps a one-minute average, and reports the result on an LED, the serial console and Bluetooth LE.

## What it does

- Samples the temperature sensor using the ADC and works out a rolling one-minute average
- Warns when the average goes above a temperature threshold (default 28 °C)
- Flags a fault if the sensor gives an invalid reading
- Detects slow long-term drift in the temperature baseline
- Shows the state on the LED: off = normal, blinking = warning or drift, solid = fault
- Prints a status line to the console every second
- Broadcasts the average temperature and state over BLE
- Press the button to cycle the warning threshold through 26, 28, 30 and 32 °C

## Thread architecture

| Thread | Priority | Responsibility |
|---|---|---|
| Acquisition | 1 (highest) | Wakes on a 100 ms timer semaphore, reads the ADC and sends the sample to the logic thread |
| Logic | 2 | Receives samples, updates the average, drift tracking and system state |
| LED | 3 | Drives the LED pattern for the current state |
| Reporting | 4 | Prints status every second and handles threshold changes from the button |
| BLE | 5 (lowest) | Updates the advertising payload with the latest average and state |


## Hardware

- nRF54L15 DK
- Analog temperature sensor
