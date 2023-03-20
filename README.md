# Verigo-thermometer

Overview:
The goal of this project is to develop a new open source firmware for Verigo branded temperature dataloggers. The dataloggers use the cc2541 microcontroller unit (MCU) and the Si705 temperature sensor. And another ble master like smartphone or pc gets the temperature readable by human.. The firmware will be hosted on GitHub and will be available for anyone to use and modify.

Thermometer is based on TI's demo "Thermometer" code, which is using sensorTag like hardware produced by TI.

The project is created by IAR 8051 version. The project file is "Thermometer\Projects\ble\Thermometer\CC2541DB\thermometer.eww"

Features:
The firmware has the following features:

    Accurate temperature sensing using the Si705 sensor
    Data advertasing at regular intervals
    Low-power operation for long battery life
 

Architecture:
The firmware will be designed using the C programming language and will be based on the Texas Instruments BLE-Stack SDK for the cc2541 MCU. The firmware will use the Si705 driver provided by Silicon Labs and will implement a custom data logging and USB communication protocol.

The firmware will be designed to run on low-power mode when not in use and wake up periodically to collect temperature data and write it to memory. The firmware will use the RTC to accurately timestamp each temperature reading.
