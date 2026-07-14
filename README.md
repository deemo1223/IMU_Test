# IMU

Standalone C++ CAN IMU reader migrated from `centaur_ws`.

The code now follows a small split:
- `CANInterface`: SocketCAN transport and Linux interface setup.
- `IMU_can`: passive IMU CAN frame decoding and per-device state tracking.
- `imu_reader`: minimal single-IMU monitor hard-coded to `can0` and sensor `0`.

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
```

The current `main.cpp` is hard-coded to read from `can0` at `1000000` bps and print data for sensor `0`.

Printed data:

- `rpy[deg]`: decoded roll, pitch, yaw angles for sensor `0`, printed every 100 ms using the latest received sample.
