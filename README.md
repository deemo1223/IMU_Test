# IMU

Standalone C++ CAN IMU reader migrated from `centaur_ws`.

The code now follows the same separation used in `BRT_test`:
- `CANInterface`: SocketCAN transport and Linux interface setup.
- `IMU_can`: passive IMU CAN frame decoding and per-device state tracking.
- `imu_reader`: minimal single-IMU monitor that prints roll, pitch, and yaw.

## Build

```bash
cd /home/harlab/code/IMU
cmake -S . -B build
cmake --build build
```

## CAN setup

```bash
./scripts/init_can.sh can0 1000000
```

`imu_reader` now configures the interface directly through the shared `CANInterface` pattern used in `BRT_test`, so the helper scripts are optional manual setup tools rather than part of the runtime path.

## Run

```bash
./build/imu_reader
./build/imu_reader --can can0 --sensor-id 3
./build/imu_reader --can1 --bitrate 1000000 --sensorID3
```

Use `--can` or `--can0`/`--can1` style flags to select the CAN interface, and `--sensor-id` or `--sensorID` to select the IMU id from `1..4`. The defaults are `can0`, `1000000`, and sensor `3`.

Printed data:

- `rpy[deg]`: decoded roll, pitch, yaw angles for the selected IMU, printed whenever a new sample arrives.
