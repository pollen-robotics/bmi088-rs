use bmi088::{Bmi088, Bmi088Ahrs, Config};
use linux_embedded_hal::I2cdev;
use std::collections::HashMap;
use std::io::Write;
use std::net::TcpListener;
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let bus = args.iter().position(|a| a == "--bus")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("/dev/i2c-4")
        .to_string();

    let freq: f64 = args.iter().position(|a| a == "--freq")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(50.0);

    let stream = args.contains(&"--stream".to_string());

    let i2c = I2cdev::new(&bus).unwrap_or_else(|e| panic!("failed to open {bus}: {e}"));
    let imu = Bmi088::new(i2c, Config::default())
        .unwrap_or_else(|e| panic!("failed to initialise BMI088 on {bus}: {e:?}"));
    let ahrs = Bmi088Ahrs::new(imu, 0.1);

    if stream {
        run_stream(ahrs, freq);
    } else {
        run_print(ahrs, freq);
    }
}

fn run_print(mut ahrs: Bmi088Ahrs<I2cdev>, freq: f64) {
    let interval = Duration::from_secs_f64(1.0 / freq);
    let mut last = Instant::now();
    let mut next_tick = last + interval;

    loop {
        let now = Instant::now();
        let dt = now.duration_since(last).as_secs_f32();
        last = now;

        let [w, x, y, z] = ahrs.get_quaternion(dt).expect("sensor read failed");
        let temp = ahrs.imu().read_temperature().expect("temp read failed");

        println!(
            "quat [{:6.3} {:6.3} {:6.3} {:6.3}]  |  {:.1} °C",
            w, x, y, z, temp
        );

        next_tick += interval;
        let now = Instant::now();
        if now < next_tick {
            thread::sleep(next_tick - now);
        }
    }
}

fn run_stream(mut ahrs: Bmi088Ahrs<I2cdev>, freq: f64) {
    let listener = TcpListener::bind("0.0.0.0:1234").expect("failed to bind port 1234");
    println!("Waiting for connection on port 1234 at {freq} Hz...");

    let interval = Duration::from_secs_f64(1.0 / freq);

    for incoming in listener.incoming() {
        let mut stream = incoming.expect("connection error");
        stream.set_nodelay(true).ok();
        println!("Client connected: {}", stream.peer_addr().unwrap());

        let mut last = Instant::now();
        let mut next_tick = last + interval;

        loop {
            let now = Instant::now();
            let dt = now.duration_since(last).as_secs_f32();
            last = now;

            let [w, x, y, z] = match ahrs.get_quaternion(dt) {
                Ok(q) => q,
                Err(_) => break,
            };
            let (gx, gy, gz) = match ahrs.imu().read_gyroscope() {
                Ok(g) => g,
                Err(_) => break,
            };

            let mut data: HashMap<&str, Vec<f32>> = HashMap::new();
            data.insert("quaternion", vec![w, x, y, z]);
            data.insert("gyro", vec![gx, gy, gz]);

            let bytes = serde_pickle::to_vec(&data, Default::default())
                .expect("serialisation failed");

            if stream.write_all(&bytes).is_err() {
                println!("Client disconnected.");
                break;
            }

            next_tick += interval;
            let now = Instant::now();
            if now < next_tick {
                thread::sleep(next_tick - now);
            }
        }

        println!("Waiting for next connection...");
    }
}
