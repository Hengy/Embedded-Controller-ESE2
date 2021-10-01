# Embedded-Controller-ESE2

This repository contains project files for the 2nd year Conestoga ESE project: Embedded Robot Controller PCB.

The PCB is a 4-layer board with connections for a STM32F303 Nucleo development board, LCD display, RS232 port, uSD card slot, RTC backup battery, Stepper motor driver, Dual DC motor driver, CANbus transciever, Bluetooth module, ultrasonic distance sensor, encoder input, and GPIO. GPIO include I2C and SPI connections.

The board runs of 12-14V, and includes a 5V switch mode buck converter, and a 3V3 linear regulator. Voltage input includes reverse polarity protection.

Layer stackup: (1) signal with ground plane (2) 5V power (3) ground (4) signal with ground plane.

PCB features include thermally enhanced features for motor drivers, thermistors for motor drivers to monitor temperature, numerous and easily accessilble test points, mechanical support for LCD display module, GPIO headers compatible with Tektronics MSO digital probes, and 5V and 3V3 isolation for debugging.

Switch mode power supply adheres to SMPS layout best practices, including small loop area.

Majority of components are SMD. PCB is capable of being oven reflowed on both sides - bottom first. Hand soldering is possible.

Project includes full 3D model.

![PCB](https://github.com/Hengy/Embedded-Controller-ESE2/blob/master/controller%203D.png)
