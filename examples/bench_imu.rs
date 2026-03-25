use bmi088::{Bmi088, Bmi088Ahrs, Config};
use cpu_time::ProcessTime;
use linux_embedded_hal::I2cdev;
use std::time::{Duration, Instant};

const TARGET_HZ: f64 = 100.0;
const RUN_SECS: u64 = 10;

fn main() {
    let bus = std::env::args().nth(1).unwrap_or_else(|| "/dev/i2c-4".to_string());
    let i2c = I2cdev::new(&bus).unwrap_or_else(|e| panic!("failed to open {bus}: {e}"));
    let imu = Bmi088::new(i2c, Config::default())
        .unwrap_or_else(|e| panic!("BMI088 init failed: {e:?}"));
    let mut ahrs = Bmi088Ahrs::new(imu, 0.1);

    let target = Duration::from_secs_f64(1.0 / TARGET_HZ);
    let run_for = Duration::from_secs(RUN_SECS);
    let expected_samples = (TARGET_HZ * RUN_SECS as f64) as usize;
    let mut intervals: Vec<f64> = Vec::with_capacity(expected_samples);

    println!("Warming up...");
    for _ in 0..10 {
        let _ = ahrs.get_quaternion(0.01).expect("read failed");
        std::thread::sleep(target);
    }

    println!("Benchmarking at {TARGET_HZ} Hz for {RUN_SECS}s...");

    let wall_start = Instant::now();
    let cpu_start = ProcessTime::now();
    let mut last = Instant::now();

    while wall_start.elapsed() < run_for {
        let dt = last.elapsed().as_secs_f32();
        let _ = ahrs.get_quaternion(dt).expect("read failed");

        let now = Instant::now();
        let elapsed = now.duration_since(last);
        intervals.push(elapsed.as_secs_f64() * 1000.0);
        last = now;

        if elapsed < target {
            std::thread::sleep(target - elapsed);
        }
    }

    let wall_ms = wall_start.elapsed().as_secs_f64() * 1000.0;
    let cpu_ms = cpu_start.elapsed().as_secs_f64() * 1000.0;

    // Drop first sample (warmup boundary)
    let intervals = &intervals[1..];
    let n = intervals.len();

    let mean = intervals.iter().sum::<f64>() / n as f64;
    let std_dev = (intervals.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64).sqrt();
    let min = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let mut sorted = intervals.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p95 = sorted[(0.95 * n as f64) as usize];
    let p99 = sorted[(0.99 * n as f64) as usize];

    println!();
    println!("=== results ({n} samples) ===");
    println!("  rate     {:6.2} Hz  (target {TARGET_HZ:.0} Hz)", 1000.0 / mean);
    println!("  mean     {:6.3} ms  (target {:.3} ms)", mean, 1000.0 / TARGET_HZ);
    println!("  std dev  {:6.3} ms", std_dev);
    println!("  min      {:6.3} ms", min);
    println!("  p95      {:6.3} ms", p95);
    println!("  p99      {:6.3} ms", p99);
    println!("  max      {:6.3} ms", max);
    println!("  cpu      {:5.1} %   ({:.1} ms cpu / {:.1} ms wall)", cpu_ms / wall_ms * 100.0, cpu_ms, wall_ms);
}
