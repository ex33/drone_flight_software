# Flight software for Quadcopter using Platform.io

## Description
Programming flight software from scratch on Teensy 4.1. Complements drone_sim repo.

## Installation
1. Install PlatformIO extension in VS Code.
2. Clone this repo.
3. Run `pio run` to build the project.

## Usage
Connect your Teensy and sensors, then run `pio run --target upload`.

## Current Sensors
- ICM20649 IMU
- LIS2MDL Magnetometer
- BMP390 Altimeter
- MTK3333 GPS