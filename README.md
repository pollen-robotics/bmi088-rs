# bmi088-rs

Rust driver for the Bosch BMI088 6-axis IMU (accelerometer + gyroscope) over I2C, with a built-in Madgwick filter for orientation estimation.

## Library

The driver is generic over any [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) I2C implementation.

```rust
use bmi088::{Bmi088, Bmi088Ahrs, Config};

let imu = Bmi088::new(i2c, Config::default())?;
let mut ahrs = Bmi088Ahrs::new(imu, /* beta */ 0.1);

// Raw sensor data
let (ax, ay, az) = ahrs.imu().read_accelerometer()?;  // g
let (gx, gy, gz) = ahrs.imu().read_gyroscope()?;      // rad/s
let temp         = ahrs.imu().read_temperature()?;     // °C

// Orientation quaternion [w, x, y, z]
let [w, x, y, z] = ahrs.get_quaternion(dt)?;
```

`Config` defaults to ±3 g / 100 Hz accelerometer and ±500 °/s / 100 Hz gyroscope. All ranges and bandwidths are configurable via `AccRange`, `AccBandwidth`, `GyroRange`, `GyroBandwidth`.

The Madgwick `beta` parameter controls convergence speed vs. noise. `0.1` is a good starting point.

## Examples

Both examples target `/dev/i2c-4` at 50 Hz by default.

### read_imu

Print quaternion and temperature in a loop, or stream data over TCP for visualization.

```sh
# Print mode
cargo run --example read_imu
cargo run --example read_imu -- --bus /dev/i2c-1 --freq 100

# Stream mode (see Visualization below)
cargo run --example read_imu -- --stream
cargo run --example read_imu -- --stream --freq 30
```

### bench_imu

Measure how well the loop holds its target rate and how much CPU it uses.

```sh
cargo run --example bench_imu
cargo run --example bench_imu -- --freq 200
```

Sample output:

```
=== results (998 samples) ===
  rate      99.90 Hz  (target 100 Hz)
  mean     10.010 ms  (target 10.000 ms)
  std dev   0.318 ms
  min       9.964 ms
  p95      10.011 ms  ← 95% of iterations finished within this time
  p99      10.017 ms  ← 99% of iterations finished within this time
  max      20.063 ms
  cpu        2.4 %
```

## Visualization

`imu_client.py` connects to a running `--stream` server and displays the live orientation using [FramesViewer](https://github.com/apirrone/FramesViewer).

**On the robot**, start the stream:
```sh
cargo run --example read_imu -- --stream --freq 50
```

**On your machine**, install dependencies and run the client:
```sh
pip install -r requirements.txt
python imu_client.py --ip <robot-ip>
```
