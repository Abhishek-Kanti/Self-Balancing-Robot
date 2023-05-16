
# Self Balancing Robot

This robot is a closed loop control system capable of balancing on its own. It uses an Arduino UNO, L298N motor driver and MPU6050 as the IMU to keep track of its inclination and provide feedback to the contol algorithm.


![WhatsApp Image 2023-05-16 at 20 59 29](https://github.com/Abhishek-Kanti/Self-Balancing-Robot/assets/114488605/7414491b-8554-469a-ba67-d906eef4d916)


## ❖ How does the robot function ?


https://github.com/Abhishek-Kanti/Self-Balancing-Robot/assets/114488605/bb988bdd-2c85-4d5e-ae32-d0552d3e04c5


### Overview:
The self-balancing robot utilizes a combination of sensors, actuators, and intelligent control algorithms to maintain its equilibrium while in motion. It incorporates a DC gear motor with a rotational speed of 100 RPM, an Arduino Uno microcontroller board for processing, an L298N motor driver for controlling the motor, and an MPU6050 sensor module for measuring the robot's orientation.

### Control System:
The core of the self-balancing robot is its closed-loop control system, which implements a PID (Proportional-Integral-Derivative) control algorithm. This algorithm constantly adjusts the motor's speed based on the robot's deviation from the desired upright position. The PID controller calculates an error signal by comparing the measured angle from the MPU6050 sensor with the desired angle setpoint. It then adjusts the motor speed to reduce this error, enabling the robot to maintain balance.

### Angle Calculation:
To accurately determine the robot's angle, a complementary filter is employed. This filter combines the accelerometer and gyroscope data from the MPU6050 sensor. By fusing these measurements, the filter eliminates noise from the accelerometer and reduces gyro drift, providing a more precise estimation of the robot's tilt angle. The complementary filter algorithm is implemented in the software running on the Arduino Uno.

### Power Source:
The self-balancing robot is powered by lithium-ion batteries, which provide the necessary energy for its operation. The battery capacity and voltage are selected to ensure sufficient power supply for the motor, microcontroller, and sensor modules, enabling extended periods of autonomous operation.

### Some Optimisations:
The Self balancing balancing robot requires fast response. Generally we can at max go upto the sampling time of 10 microseconds (time taken by the void loop to execute the code), so we need to save time. Arduino uses a function called "analogWrite" to generate pwm signals which are used to control the speeed of the motors. This becomes a time taking function when we start to increase the lines of code to add more features like RC. 

To save time I directly controlled the arduino registers and timers to generate pwm signals. I used Arduino timer 2 which sends the pwm signal to the pin number 9 and 10 of the Arduino by default. Through this I also got control over the frequency of the signal. On increasing the frequency of the pwm signal we get more control over the motors, though it has some cons too such as heating of motors but that usually happens at very high frequencies.

## circuit Diagram:

![Circuit-Diagram-for-DIY-Self-Balancing-Robot-using-Arduino](https://github.com/Abhishek-Kanti/Self-Balancing-Robot/assets/114488605/ed9ff5af-5fbd-481c-88ec-1b16bd34e29a)

