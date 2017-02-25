# Motioncapture suit
Motion capture suit based on two MPU6050 sensors and Arduino Nano

Results - [Video](https://goo.gl/photos/qRytGaHFtdLTXg9C8)

## How it works:
We need to calibrate each MPU6050 sensor using **MPU6050 calibration sketch**. Connect them into Arduino on way described under and write included sketch into Arduino.

Arduino pools MPU sensors, read quaternions from DMP and sends them to PC over UART. On PC side, Processing sketch reads data and rotate two boxes on display. Left box represents first sensor and second box second sensor.
These quaternions could be used in Blender 3D or similar applications as input for motion capture.

## Arm mount
Arm mount was made using acryl. PDF was included, and this is how its look:  

[Image 1 - Mount bottom view](https://goo.gl/photos/QSwBPkb8pkvLgPxv7)  
[Image 2 - Mount top view](https://goo.gl/photos/MQADPsbznEXjRJXR9)  
[Image 3 - Mounted on arm](https://goo.gl/photos/EaGsDJzs3iWhQrSE8)

## Hardware connections:

Based on: [DIY Hacking](https://301o583r8shhildde3s0vcnh-wpengine.netdna-ssl.com/wp-content/uploads/2014/11/conn.png)

**First MPU6050**:  

| MPU6050       | Arduino       |
| ------------- |:-------------:|
| VCC           | +5V           |
| GND           | GND           |
| SCL           | A5            |
| SDA           | A4            |
| INT           | D2            |
| AD0           | **GND**       |

Address on I2C bus: **0x68**

**Second MPU6050**:  

| MPU6050       | Arduino       |
| ------------- |:-------------:|
| VCC           | +5V           |
| GND           | GND           |
| SCL           | A5            |
| SDA           | A4            |
| INT           | D2            |
| AD0           | **VCC**       |

Address on I2C bus: **0x69**


Sensors are connected via flat cable to Arduino Nano as shown in images above.
