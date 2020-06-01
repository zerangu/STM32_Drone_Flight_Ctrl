
 * @author Zeran Gu
 *
 * Copyright (c) 2020 Zeran Gu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.



# STM32 Drone Flight Controller

Project is based on STM CUBE IDE

uses STM32f103c8t6 (AKA bluepill) as main controller and MPU6050 as IMU

input ppm as user control signal ch1:Throttle ch2:Roll ch3:Pitch ch4:Yaw ch5:arm

ppm must be 8 channel and it is compatible with Frsky Radio system's default settings

using other radio might reqire modifing the code in main.c. Search keyword "FRsky" to find the code related to radio system


# Pin Mapping:

PPM   -  PA4 - receiver

(top view)

ESC1  -  PA6 - left  - front - cw

ESC2  -  PA7 - right - front - ccw

ESC3  -  PB0 - right - rear   - cw

ESC4  -  PB1 - left  - rear   - ccw

(ESC: electronic speed controller)

# MPU6050 Mounting & Pin Mapping

X-axis towards front, facing up, lay flat. 

Use earplug, other foam materials or hollow rubber balls as a damper is a good practice.

VCC - 5V

GND - GND

SCL - PB8

SDA - PB9


# Power Plant

ESC:    HOBBYWING SKYWALKER 20A

Motor:  DJI 2212 920KV

Prop:   1045

bettery: 3s lipo 2200mah



# Flight Control

Controller uses PID control without I.

D gain might be too high for you.

Controller can read TFluna laser range finder which allowing altitude hold function become possible in the future.

# Test Flight

1. connect main power

2. imu calibration will starts in 2 seconds 

3. calibrating starts and will take another 2 seconds, dont touch the drone during calibration

4. motor beep

5. flip ch5 switch to arm (PWM > 1800us) and fly the damn thing

# good luck in your project!
