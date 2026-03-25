//! Driver for the Bosch BMI088 6-axis IMU (accelerometer + gyroscope).
//!
//! Uses the `embedded-hal` I2C trait, making it portable across any HAL.
//!
//! # Example
//! ```ignore
//! let config = Config::default(); // 3G accel, 500 DPS gyro, 100 Hz bandwidth
//! let mut imu = Bmi088Ahrs::new(Bmi088::new(i2c, config)?, 0.1);
//!
//! let (ax, ay, az) = imu.imu().read_accelerometer()?;   // g
//! let (gx, gy, gz) = imu.read_gyroscope()?;       // rad/s
//! let temp = imu.read_temperature()?;              // °C
//! ```

mod fusion;
pub use fusion::Bmi088Ahrs;

use embedded_hal::i2c::I2c;

// I2C addresses
const ACC_ADDR: u8 = 0x18;
const GYRO_ADDR: u8 = 0x69;

// Accelerometer registers
const REG_ACC_CONF: u8 = 0x40;
const REG_ACC_RANGE: u8 = 0x41;
const REG_ACC_X_LSB: u8 = 0x12;
const REG_ACC_POWER_CONF: u8 = 0x7D;
const REG_ACC_PWR_CTRL: u8 = 0x7D;
const REG_TEMP_MSB: u8 = 0x22;

const ACC_ON: u8 = 0x04;

// Gyroscope registers
const REG_GYRO_RANGE: u8 = 0x0F;
const REG_GYRO_BANDWIDTH: u8 = 0x10;
const REG_GYRO_RATE_X_LSB: u8 = 0x02;

// ── Configuration enums ──────────────────────────────────────────────────────

/// Accelerometer measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccRange {
    /// ±3 g
    G3,
    /// ±6 g
    G6,
    /// ±12 g
    G12,
    /// ±24 g
    G24,
}

impl AccRange {
    fn reg_value(self) -> u8 {
        match self {
            AccRange::G3 => 0,
            AccRange::G6 => 1,
            AccRange::G12 => 2,
            AccRange::G24 => 3,
        }
    }

    /// LSB / g (sensitivity divisor)
    fn sensitivity(self) -> f32 {
        match self {
            AccRange::G3 => 10920.0,
            AccRange::G6 => 5460.0,
            AccRange::G12 => 2730.0,
            AccRange::G24 => 1365.0,
        }
    }
}

/// Accelerometer output data rate / filter bandwidth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccBandwidth {
    Hz12_5,
    Hz25,
    Hz50,
    Hz100,
    Hz200,
    Hz400,
    Hz800,
    Hz1600,
}

impl AccBandwidth {
    fn reg_value(self) -> u8 {
        match self {
            AccBandwidth::Hz12_5 => 0xA5,
            AccBandwidth::Hz25 => 0xA6,
            AccBandwidth::Hz50 => 0xA7,
            AccBandwidth::Hz100 => 0xA8,
            AccBandwidth::Hz200 => 0xA9,
            AccBandwidth::Hz400 => 0xAA,
            AccBandwidth::Hz800 => 0xAB,
            AccBandwidth::Hz1600 => 0xAC,
        }
    }
}

/// Gyroscope measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroRange {
    /// ±2000 °/s
    Dps2000,
    /// ±1000 °/s
    Dps1000,
    /// ±500 °/s
    Dps500,
    /// ±250 °/s
    Dps250,
    /// ±125 °/s
    Dps125,
}

impl GyroRange {
    fn reg_value(self) -> u8 {
        match self {
            GyroRange::Dps2000 => 0,
            GyroRange::Dps1000 => 1,
            GyroRange::Dps500 => 2,
            GyroRange::Dps250 => 3,
            GyroRange::Dps125 => 4,
        }
    }

    /// LSB / (°/s) (sensitivity divisor)
    fn sensitivity(self) -> f32 {
        match self {
            GyroRange::Dps2000 => 16.384,
            GyroRange::Dps1000 => 32.768,
            GyroRange::Dps500 => 65.536,
            GyroRange::Dps250 => 131.072,
            GyroRange::Dps125 => 262.144,
        }
    }
}

/// Gyroscope filter bandwidth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroBandwidth {
    Hz2000,
    Hz1000,
    Hz400,
    Hz200,
    Hz100,
}

impl GyroBandwidth {
    fn reg_value(self) -> u8 {
        match self {
            GyroBandwidth::Hz2000 => 0,
            GyroBandwidth::Hz1000 => 2,
            GyroBandwidth::Hz400 => 3,
            GyroBandwidth::Hz200 => 4,
            GyroBandwidth::Hz100 => 5,
        }
    }
}

// ── Config ───────────────────────────────────────────────────────────────────

/// Sensor configuration passed to [`Bmi088::new`].
pub struct Config {
    pub acc_range: AccRange,
    pub acc_bandwidth: AccBandwidth,
    pub gyro_range: GyroRange,
    pub gyro_bandwidth: GyroBandwidth,
}

impl Default for Config {
    /// Matches the Python library defaults: ±3 g / 100 Hz, ±500 °/s / 100 Hz.
    fn default() -> Self {
        Config {
            acc_range: AccRange::G3,
            acc_bandwidth: AccBandwidth::Hz100,
            gyro_range: GyroRange::Dps500,
            gyro_bandwidth: GyroBandwidth::Hz100,
        }
    }
}

// ── Error ────────────────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

// ── Driver ───────────────────────────────────────────────────────────────────

/// BMI088 driver.
pub struct Bmi088<I2C> {
    i2c: I2C,
    acc_range: AccRange,
    gyro_range: GyroRange,
}

impl<I2C: I2c> Bmi088<I2C> {
    /// Initialise the sensor with the given configuration.
    pub fn new(mut i2c: I2C, config: Config) -> Result<Self, Error<I2C::Error>> {
        // Accelerometer: bandwidth then range
        i2c.write(ACC_ADDR, &[REG_ACC_CONF, config.acc_bandwidth.reg_value()])?;
        i2c.write(ACC_ADDR, &[REG_ACC_RANGE, config.acc_range.reg_value()])?;

        // Gyroscope: range then bandwidth
        i2c.write(GYRO_ADDR, &[REG_GYRO_RANGE, config.gyro_range.reg_value()])?;
        i2c.write(GYRO_ADDR, &[REG_GYRO_BANDWIDTH, config.gyro_bandwidth.reg_value()])?;

        // Power on accelerometer (mirrors Python _start sequence)
        i2c.write(ACC_ADDR, &[REG_ACC_POWER_CONF, 0x00])?;
        i2c.write(ACC_ADDR, &[REG_ACC_PWR_CTRL, ACC_ON])?;

        Ok(Bmi088 {
            i2c,
            acc_range: config.acc_range,
            gyro_range: config.gyro_range,
        })
    }

    /// Read accelerometer. Returns `(x, y, z)` in **g**.
    pub fn read_accelerometer(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(ACC_ADDR, &[REG_ACC_X_LSB], &mut buf)?;

        let sens = self.acc_range.sensitivity();
        let x = i16::from_le_bytes([buf[0], buf[1]]) as f32 / sens;
        let y = i16::from_le_bytes([buf[2], buf[3]]) as f32 / sens;
        let z = i16::from_le_bytes([buf[4], buf[5]]) as f32 / sens;
        Ok((x, y, z))
    }

    /// Read accelerometer. Returns `(x, y, z)` in **m/s²**.
    pub fn read_accelerometer_ms2(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_accelerometer()?;
        Ok((x * 9.80665, y * 9.80665, z * 9.80665))
    }

    /// Read gyroscope. Returns `(x, y, z)` in **rad/s**.
    pub fn read_gyroscope(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(GYRO_ADDR, &[REG_GYRO_RATE_X_LSB], &mut buf)?;

        let sens = self.gyro_range.sensitivity();
        let to_rad = core::f32::consts::PI / 180.0;
        let x = i16::from_le_bytes([buf[0], buf[1]]) as f32 / sens * to_rad;
        let y = i16::from_le_bytes([buf[2], buf[3]]) as f32 / sens * to_rad;
        let z = i16::from_le_bytes([buf[4], buf[5]]) as f32 / sens * to_rad;
        Ok((x, y, z))
    }

    /// Read gyroscope. Returns `(x, y, z)` in **degrees/s**.
    pub fn read_gyroscope_dps(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(GYRO_ADDR, &[REG_GYRO_RATE_X_LSB], &mut buf)?;

        let sens = self.gyro_range.sensitivity();
        let x = i16::from_le_bytes([buf[0], buf[1]]) as f32 / sens;
        let y = i16::from_le_bytes([buf[2], buf[3]]) as f32 / sens;
        let z = i16::from_le_bytes([buf[4], buf[5]]) as f32 / sens;
        Ok((x, y, z))
    }

    /// Read temperature. Returns value in **°C**.
    ///
    /// Raw value is an 11-bit unsigned number; formula: `raw * 0.125 + 23`.
    pub fn read_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(ACC_ADDR, &[REG_TEMP_MSB], &mut buf)?;

        // buf[0] = TEMP_MSB (0x22), buf[1] = TEMP_LSB (0x23)
        let raw = ((buf[0] as u16) << 3) | ((buf[1] as u16) >> 5);
        Ok(raw as f32 * 0.125 + 23.0)
    }

    /// Consume the driver and return the underlying I2C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }
}
