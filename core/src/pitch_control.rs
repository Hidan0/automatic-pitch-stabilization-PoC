use std::time::SystemTime;

use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{OutputPin, PinDriver};
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::hal::peripheral::Peripheral;
use mpu6050::device::{AccelRange, GyroRange};
use mpu6050::Mpu6050;

use crate::Result;

type MpuDriver<'a> = Mpu6050<I2cDriver<'a>>;

const CALIBRATIONS: u16 = 2000;
const K: f32 = 0.035;

pub struct PitchEstimator<'a> {
    mpu: MpuDriver<'a>,
    pub gyro_calibration: f32,
    pub accel_calibration: f32,
    angle: f32,
}

impl<'a> PitchEstimator<'a> {
    fn setup_mpu<D: DelayNs>(delay: &mut D, i2c: I2cDriver<'a>) -> Result<MpuDriver<'a>> {
        let mut mpu = Mpu6050::new(i2c);

        mpu.init(delay).unwrap();

        mpu.set_temp_enabled(false).unwrap();
        mpu.set_gyro_range(GyroRange::D500).unwrap();
        mpu.set_accel_range(AccelRange::G8).unwrap();

        Ok(mpu)
    }

    fn calibrate_gyro(mpu: &mut MpuDriver<'a>) -> f32 {
        let mut p = 0_f32;

        let start_time = SystemTime::now();
        for _ in 0..CALIBRATIONS {
            p += mpu.get_gyro().unwrap().x;
        }
        let current = SystemTime::now();
        log::info!(
            "For {} measurements took {}ms",
            CALIBRATIONS,
            current
                .duration_since(start_time)
                .unwrap_or_default()
                .as_millis()
        );

        p / CALIBRATIONS as f32
    }

    fn calibrate_accel(mpu: &mut MpuDriver<'a>) -> f32 {
        let mut p = 0_f32;

        let start_time = SystemTime::now();
        for _ in 0..CALIBRATIONS {
            p += mpu.get_acc_angles().unwrap().x;
        }
        let current = SystemTime::now();
        log::info!(
            "For {} measurements took {}ms",
            CALIBRATIONS,
            current
                .duration_since(start_time)
                .unwrap_or_default()
                .as_millis()
        );

        p / CALIBRATIONS as f32
    }

    pub fn new(
        i2c: I2cDriver<'a>,
        cal_led: impl Peripheral<P = impl OutputPin>,
    ) -> Result<PitchEstimator<'a>> {
        let mut led = PinDriver::output(cal_led).unwrap();
        led.set_high().unwrap();

        let delay = &mut FreeRtos;
        log::info!("Starting Mpu set up...");
        let mut mpu = Self::setup_mpu(delay, i2c)?;
        log::info!("Finished Mpu set up.");

        let gyro_calibration = Self::calibrate_gyro(&mut mpu);
        let accel_calibration = Self::calibrate_accel(&mut mpu);

        led.set_low().unwrap();

        Ok(Self {
            mpu,
            gyro_calibration,
            accel_calibration,
            angle: 0.,
        })
    }

    fn get_pitch_rate(&mut self) -> Result<f32> {
        let gyro = self.mpu.get_gyro().unwrap();

        Ok(gyro.x - self.gyro_calibration)
    }

    fn get_accel_pitch(&mut self) -> Result<f32> {
        Ok(self.mpu.get_acc_angles().unwrap().x - self.accel_calibration)
    }

    pub fn update(&mut self, dt: &f32) -> Result<()> {
        let pitch_rate = self.get_pitch_rate()?;
        let accel_pitch = self.get_accel_pitch()?;

        self.angle = accel_pitch * K + (1. - K) * (self.angle + pitch_rate * dt);

        Ok(())
    }

    pub fn pitch(&mut self) -> Result<f32> {
        Ok(self.angle)
    }
}

pub struct PIController {
    k_p: f32,
    k_i: f32,
    i: f32,
    setpoint: f32,
}

impl PIController {
    pub fn new(k_p: f32, k_i: f32, setpoint: f32) -> PIController {
        Self {
            k_p,
            k_i,
            i: 0.,
            setpoint,
        }
    }

    pub fn u(&mut self, value: f32, dt: &f32) -> f32 {
        let error = self.setpoint - value;
        self.i += error * dt;
        (self.k_p * error) + (self.k_i * self.i)
    }
}
