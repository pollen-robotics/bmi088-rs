use bmi088::{Bmi088, Config};
use linux_embedded_hal::I2cdev;
use std::thread;
use std::time::Duration;

fn main() {
    let bus = std::env::args().nth(1).unwrap_or_else(|| "/dev/i2c-1".to_string());
    let i2c = I2cdev::new(&bus).unwrap_or_else(|e| panic!("failed to open {bus}: {e}"));
    let mut imu = Bmi088::new(i2c, Config::default())
        .unwrap_or_else(|e| panic!("failed to initialise BMI088 on {bus}: {e:?}"));

    loop {
        let (ax, ay, az) = imu.read_accelerometer().expect("accel read failed");
        let (gx, gy, gz) = imu.read_gyroscope().expect("gyro read failed");
        let temp = imu.read_temperature().expect("temp read failed");

        println!(
            "accel [{:7.3} {:7.3} {:7.3}] g  |  gyro [{:8.4} {:8.4} {:8.4}] rad/s  |  {:.1} °C",
            ax, ay, az, gx, gy, gz, temp
        );

        thread::sleep(Duration::from_millis(100));
    }
}
