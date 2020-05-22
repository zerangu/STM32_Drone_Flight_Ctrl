# STM32_Drone_Flight_Ctrl

uses stm32f103c8t6 bluepill and mpu6050

input ppm as user control signal ch1:Throttle ch2:Roll ch3:Pitch ch4:Yaw ch5:arm

ppm must be 8 channel

# pin mapping:

PPM   -  PA4 - recever
(top view)
ESC1  -  PA6 - left  - frount - cw
ESC2  -  PA7 - right - frount - ccw
ESC3  -  PB0 - right - rear   - cw
ESC4  -  PB1 - left  - rear   - ccw

# MPU6050   X-axis towards frount
VCC - 5V
GND - GND
SCL - PB8
SDA - PB9



# Lifting Power

ESC:    HOBBYWING SKYWALKER 20A
Motor:  DJI P4 (kv900? not sure)
Prop:   1045
bettery: 3s lipo 2200mah

Controller uses PID control without I


D gain might be too high for you

Controller can read TFluna laser range finder 

good luck in your project!
