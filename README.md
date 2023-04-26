# KineticCloud
KineticCloud is an innovative RTOS system based on the ESP32 microcontroller and designed to enable precise, real-time control of servo motors based on the system's inertial acceleration. KineticCloud incorporates inercial sensors that provide real-time data on the state of the system, which is sent to a cloud-based IoT platform for analysis and processing. The system is designed to facilitate the development of applications that require precise, real-time control of servo motors, such as robotics and automation systems.

This project was developed  with my computer engineering students at the [FIAP](https://www.fiap.com.br/) university - São Paulo, Brasil.

- [Setup](#setup)
- [Development](#development)
- [Run](#contribution-guide)

## Setup

The hardware used for this project was:
- mpu6050
- ultrasonic sensor
- servo motor
- esp32

The software configuration:

- esp-idf
- thingspeaks platform
- docker & docker-compose
- python3

## Development

The solution architecture:
![iot-architecture](docs/code-strategy.png)

We are using the esp-idf on vscode and the enviroment is linux based system. Please read the docs for more information.

## Run

First start a new project, using the esp-idf. You should have a folder named `template-app` on your `~/esp` folder.
Reeplace the main folder:

```sh
cp -r <where-you-clone-this-repo>/main ~/esp/template-app
```

Then build and flash the project using the esp-idf extencion on vscode, detail instructions on the docs.
