# TEST_RTC_CAM_V6
Test code of the payload camera prototype subsystem of the project Cubesat 3u Libertad 2 (Universidad Sergio Arboleda).
This code was created to test the VITA1300 CMOS SENSOR (Super Extended Graphics Array CMOS Image Sensor) installed
in a Printed Circuit Board design for the payload subsystem of the Cubesat 3U Libertad 2 project.

So this code initializes a CORTEX ARM M4 STM32F4xx, used for command and handling the VITA1300 CMOS SENSOR.
Communication between Microcontroller and CMOS sensor is made with a Serial Peripheral Interface (SPI).

A USART communication is configured to receive commands from a computer to control the microcontroller

Image data captured with the CMOS sensor is stored in memory management by the microcontroller and send to the computer with the USART.
