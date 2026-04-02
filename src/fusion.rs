use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;

use crate::{Bmi088, Error};
use embedded_hal::i2c::I2c;

/// Wraps [`Bmi088`] with a Madgwick filter to produce orientation quaternions.
pub struct Bmi088Ahrs<I2C> {
    imu: Bmi088<I2C>,
    filter: Madgwick<f64>,
    beta: f64,
}

impl<I2C: I2c> Bmi088Ahrs<I2C> {
    /// Create a new instance.
    ///
    /// * `beta` — filter gain; higher values converge faster but are noisier.
    ///   `0.1` is a reasonable starting point.
    pub fn new(imu: Bmi088<I2C>, beta: f64) -> Self {
        Self {
            imu,
            filter: Madgwick::new(1.0 / 100.0, beta),
            beta,
        }
    }

    /// Read accelerometer and gyroscope, update the Madgwick filter, and return
    /// both the gyroscope reading and the orientation quaternion in one call.
    ///
    /// * `dt` — time since last call, in seconds.
    ///
    /// Returns `(gyro, quaternion)` where:
    /// - `gyro` is `[gx, gy, gz]` in **rad/s** (after any axis remapping)
    /// - `quaternion` is `[w, x, y, z]` (body→world, scalar-first)
    pub fn update(&mut self, dt: f32) -> Result<([f32; 3], [f32; 4]), Error<I2C::Error>> {
        let (ax, ay, az) = self.imu.read_accelerometer()?;
        let (gx, gy, gz) = self.imu.read_gyroscope()?;

        // Rebuild with current dt while preserving quaternion state
        self.filter = Madgwick::new_with_quat(
            dt as f64,
            self.beta,
            self.filter.quat,
        );

        let q = match self.filter.update_imu(
            &Vector3::new(gx as f64, gy as f64, gz as f64),
            &Vector3::new(ax as f64, ay as f64, az as f64),
        ) {
            Ok(q) => *q,
            Err(_) => self.filter.quat,
        };

        let q = q.into_inner();
        Ok(([gx, gy, gz], [q.w as f32, q.i as f32, q.j as f32, q.k as f32]))
    }

    /// Read accelerometer and gyroscope, then update the Madgwick filter.
    ///
    /// * `dt` — time since last call, in seconds.
    ///
    /// Returns the orientation quaternion as `[w, x, y, z]`.
    pub fn get_quaternion(&mut self, dt: f32) -> Result<[f32; 4], Error<I2C::Error>> {
        let (_, quat) = self.update(dt)?;
        Ok(quat)
    }

    /// Access the underlying driver (e.g. to read temperature).
    pub fn imu(&mut self) -> &mut Bmi088<I2C> {
        &mut self.imu
    }

    /// Consume the wrapper and return the underlying I2C bus.
    pub fn release(self) -> I2C {
        self.imu.release()
    }
}
