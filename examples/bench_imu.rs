//! What a sample costs, and which part of it costs that.
//!
//! Five modes, run back to back at one rate, so the cost can be attributed instead of guessed
//! at. They differ only in what happens between two sleeps:
//!
//! - `sleep only`  — nothing at all. **The floor**: what it costs merely to  0 transactions
//!                   be woken at this rate.
//! - `filter only` — one Madgwick update on constant vectors, no bus.        0 transactions
//! - `update`      — accel + gyro, returns gyro and the quaternion.         2 transactions
//! - `update_all`  — the same reads, and the accel comes back too.          2 transactions
//! - `three-reads` — `update`, then `read_accelerometer_ms2` for the        3 transactions
//!                   accel it just dropped. What a consumer publishing a
//!                   full sample had to do before `update_all`.
//!
//! On a Radxa Zero 3 at 100 Hz the last three came out at 4.44%, 4.46% and 4.53% of a core:
//! a whole I²C transaction is worth ~0.09 points, or ~9 µs, against ~440 µs a sample. So the
//! bus is not what a sample costs, which is what the first two modes are here to pin down —
//! if `sleep only` is most of that 4.4%, the cost is being woken a hundred times a second and
//! the only levers are the rate or not running at all.
//!
//! Stop anything else that owns the bus first (on a duck: `sudo systemctl stop tofd`), or the
//! contention lands in these numbers.

use ahrs::{Ahrs, Madgwick};
use bmi088::{Bmi088, Bmi088Ahrs, Config};
use cpu_time::ProcessTime;
use linux_embedded_hal::I2cdev;
use nalgebra::Vector3;
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

    bench("sleep only  (0 reads)", freq, &mut ahrs, |_, _| {});

    // The same filter the driver runs, on vectors that never change: a gyro at rest and one g
    // of gravity. Madgwick does not care that the input is constant — it does the same
    // arithmetic either way — so this is the filter's cost with the bus taken out of it.
    let mut filter = Madgwick::new(1.0 / freq, 0.1);
    let gyro = Vector3::new(0.0f64, 0.0, 0.0);
    let accel = Vector3::new(0.0f64, 0.0, 1.0);
    bench("filter only (0 reads)", freq, &mut ahrs, |_, _| {
        let _ = filter.update_imu(&gyro, &accel);
    });

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
