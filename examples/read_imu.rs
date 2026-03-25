use bmi088::{Bmi088Ahrs, Bmi088, Config};
use linux_embedded_hal::I2cdev;
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let bus = std::env::args().nth(1).unwrap_or_else(|| "/dev/i2c-4".to_string());
    let i2c = I2cdev::new(&bus).unwrap_or_else(|e| panic!("failed to open {bus}: {e}"));

    let imu = Bmi088::new(i2c, Config::default())
        .unwrap_or_else(|e| panic!("failed to initialise BMI088 on {bus}: {e:?}"));
    let mut ahrs = Bmi088Ahrs::new(imu, 0.1);

    let mut last = Instant::now();

    loop {
        let dt = last.elapsed().as_secs_f32();
        last = Instant::now();

        let [w, x, y, z] = ahrs.get_quaternion(dt).expect("sensor read failed");
        let temp = ahrs.imu().read_temperature().expect("temp read failed");

        println!(
            "quat [{:6.3} {:6.3} {:6.3} {:6.3}]  |  {:.1} °C",
            w, x, y, z, temp
        );

        thread::sleep(Duration::from_millis(10));
    }
}
