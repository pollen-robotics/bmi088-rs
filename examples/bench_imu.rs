//! What a sample costs, three ways.
//!
//! The point of running this is the CPU line, and the reason it has three modes is that on a
//! Radxa Zero 3 reading this chip at 100 Hz, ~90% of the cost is *system* time — the per-
//! transaction driver work — and almost none of it is the Madgwick update. So the number that
//! moves is the number of I²C transactions per sample, and the modes differ only in that:
//!
//! - `update`      — accel + gyro, returns gyro and the quaternion.      2 transactions
//! - `update_all`  — the same reads, and the accel comes back too.       2 transactions
//! - `three-reads` — `update`, then `read_accelerometer_ms2` for the     3 transactions
//!                   accel it just dropped. What a consumer publishing
//!                   a full sample had to do before `update_all`.
//!
//! Nothing else about the sensor differs between them, so the CPU spread between the last one
//! and the other two is the price of that extra transaction, measured rather than reasoned
//! about.
//!
//! Stop anything else that owns the bus first (on a duck: `sudo systemctl stop tofd`), or the
//! contention lands in these numbers.

use bmi088::{Bmi088, Bmi088Ahrs, Config};
use cpu_time::ProcessTime;
use linux_embedded_hal::I2cdev;
use std::time::{Duration, Instant};

const RUN_SECS: u64 = 10;

fn arg(args: &[String], name: &str) -> Option<String> {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .cloned()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let bus = arg(&args, "--bus").unwrap_or_else(|| "/dev/i2c-4".to_string());
    let freq: f64 = arg(&args, "--freq")
        .and_then(|s| s.parse().ok())
        .unwrap_or(50.0);

    let i2c = I2cdev::new(&bus).unwrap_or_else(|e| panic!("failed to open {bus}: {e}"));
    let imu = Bmi088::new(i2c, Config::default())
        .unwrap_or_else(|e| panic!("BMI088 init failed: {e:?}"));
    let mut ahrs = Bmi088Ahrs::new(imu, 0.1);

    println!("bus {bus}, {freq} Hz, {RUN_SECS}s a mode");

    bench("update      (2 reads)", freq, &mut ahrs, |ahrs, dt| {
        let _ = ahrs.update(dt).expect("read failed");
    });
    bench("update_all  (2 reads)", freq, &mut ahrs, |ahrs, dt| {
        let _ = ahrs.update_all(dt).expect("read failed");
    });
    bench("three-reads (3 reads)", freq, &mut ahrs, |ahrs, dt| {
        let _ = ahrs.update(dt).expect("read failed");
        let _ = ahrs.imu().read_accelerometer_ms2().expect("read failed");
    });
}

/// Drive `sample` at `freq` for [`RUN_SECS`], then say what it cost.
///
/// CPU is process time over wall time, so it is directly comparable to the `%CPU` a `ps -L` on
/// the daemon's `head-imu` thread reports.
fn bench(
    label: &str,
    freq: f64,
    ahrs: &mut Bmi088Ahrs<I2cdev>,
    mut sample: impl FnMut(&mut Bmi088Ahrs<I2cdev>, f32),
) {
    let target = Duration::from_secs_f64(1.0 / freq);
    let run_for = Duration::from_secs(RUN_SECS);
    let mut intervals: Vec<f64> = Vec::with_capacity((freq * RUN_SECS as f64) as usize);

    // Warm up off the clock: the first transactions after a mode switch pay for cold cache
    // lines and, on the first mode, the chip's own settling.
    for _ in 0..10 {
        sample(ahrs, 1.0 / freq as f32);
        std::thread::sleep(target);
    }

    let wall_start = Instant::now();
    let cpu_start = ProcessTime::now();
    let mut last = Instant::now();
    let mut next_tick = last + target;

    while wall_start.elapsed() < run_for {
        let now = Instant::now();
        let dt = now.duration_since(last).as_secs_f32();
        intervals.push(now.duration_since(last).as_secs_f64() * 1000.0);
        last = now;

        sample(ahrs, dt);

        next_tick += target;
        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        }
    }

    let wall_ms = wall_start.elapsed().as_secs_f64() * 1000.0;
    let cpu_ms = cpu_start.elapsed().as_secs_f64() * 1000.0;

    // Drop the first, which spans the warmup boundary.
    let intervals = &intervals[1..];
    let n = intervals.len();
    let mean = intervals.iter().sum::<f64>() / n as f64;
    let mut sorted = intervals.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p99 = sorted[(0.99 * n as f64) as usize];

    println!(
        "  {label}   cpu {:5.2} %   rate {:6.2} Hz   mean {:5.3} ms   p99 {:5.3} ms   ({n} samples)",
        cpu_ms / wall_ms * 100.0,
        1000.0 / mean,
        mean,
        p99,
    );
}
