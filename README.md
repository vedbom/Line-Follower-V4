# Line Following Robot
![Orthograhic Robot Image](Robot%20Images/Robot%20Image.jpg) 

## Objective
The objective of this project is to create a 4 wheeled robotic platform that can be used for experimentation with STM32 MCU's. In particular, this platform will be used to learn about the various peripherals onboard a mainstream STM32 MCU, such as the GPIO, interrupts, general purpose timers, UART and I2C. The basic functionality of the robot will be to follow a black path drawn on top of a white surface using IR sensors. Furthermore, it will have object avoidance capabilities by detecting blockages in it's path using a momentary switch mounted at the front of the robot. 

Here is a quick demo of the robot: [demo link](https://drive.google.com/file/d/1aaqpYK5WPP4vlI3YJaK1VRXlM3UIz-dd/view?usp=drive_link)

## Components
- STM32F0 Discovery board (utilizes the STM32F051R8 MCU)
- 4 x DC motors
- 4 x wheel encoders
- 5 x IR LED and photodiode pairs
- 2 x L298N motor driver modules
- momentary switch
- HC-12 wireless serial port communication module
- 6 x 1.5 V AA batteries

![Robot Top View Annotated Image](Robot%20Images/Robot%20Top%20View%20Annotated.PNG)
![Robot Bottom View Annotated Image](Robot%20Images/Robot%20Bottom%20View%20Annotated.PNG)
