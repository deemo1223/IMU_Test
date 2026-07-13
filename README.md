# IMU

Standalone C++ CAN IMU reader migrated from `centaur_ws`.

## Build

```bash
cd /home/harlab/code/IMU
make
```

## CAN setup

```bash
./scripts/init_can.sh can0 1000000
```

`imu_reader` also runs `IMU/init_can.sh` automatically before opening the CAN socket, following the original `centaur_ws` startup flow. The manual command is useful when you want to test the CAN adapter before starting the reader.

## Run

```bash
./build/imu_reader
./build/imu_reader can0 3
```

The second argument selects the IMU id from `1..4`. The default is `3`, matching the torso/com IMU comment in the original `hardwareIMU.cpp`.

Printed data:

- `rpy[deg]`: roll, pitch, yaw solved from quaternion by migrated `math/orientation_tools_modify.h`, with the same torso IMU correction chain used in `stateestimator::run()`, initial yaw zeroed, and yaw unwrapped.
- `gyro`: body angular velocity with the same sign convention as `stateestimator::run()` (`x, -y, -z`).
- `acc`: body acceleration with the same sign convention as `stateestimator::run()` (`x, -y, -z`).
- `quat` and `freeAcc`: decoded CAN payload values.

## Logs

After the first valid IMU sample, `imu_reader` creates `logs/YYYYmmdd_HHMMSS.csv` under the current run directory.

CSV columns:

```text
frame,t_ms,t_s,rpy_x,rpy_y,rpy_z,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,quat_1,quat_2,quat_3,quat_4,freeAcc_x,freeAcc_y
```
