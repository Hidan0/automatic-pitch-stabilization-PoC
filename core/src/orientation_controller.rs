use std::time::SystemTime;

use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::i2c::I2cDriver;
use mpu6050::device::{AccelRange, GyroRange};
use mpu6050::Mpu6050;

use crate::Result;

type MpuDriver<'a> = Mpu6050<I2cDriver<'a>>;

const CALIBRATIONS: u16 = 2000;

pub struct Controller<'a> {
    mpu: MpuDriver<'a>,
    pub calibrations: GyroCalibration,
}

pub struct GyroCalibration {
    r: f32,
}

impl<'a> Controller<'a> {
    fn setup_mpu<D: DelayNs>(delay: &mut D, i2c: I2cDriver<'a>) -> Result<MpuDriver<'a>> {
        let mut mpu = Mpu6050::new(i2c);

        mpu.init(delay).unwrap();

        mpu.set_temp_enabled(false).unwrap();
        mpu.set_gyro_range(GyroRange::D500).unwrap();
        mpu.set_accel_range(AccelRange::G8).unwrap();

        Ok(mpu)
    }

    fn calibrate_gyro(mpu: &mut MpuDriver<'a>) -> GyroCalibration {
        let mut r = 0_f32;

        let start_time = SystemTime::now();
        for _ in 0..CALIBRATIONS {
            r += mpu.get_gyro().unwrap().x;
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

        GyroCalibration {
            r: r / CALIBRATIONS as f32,
        }
    }

    pub fn new(i2c: I2cDriver<'a>) -> Result<Controller<'a>> {
        let delay = &mut FreeRtos;
        log::info!("Starting Mpu set up...");
        let mut mpu = Self::setup_mpu(delay, i2c)?;
        log::info!("Finished Mpu set up.");

        let calibrations = Self::calibrate_gyro(&mut mpu);

        Ok(Self { mpu, calibrations })
    }

    pub fn get_roll(&mut self) -> Result<f32> {
        let gyro = self.mpu.get_gyro().unwrap();

        Ok(gyro.x - self.calibrations.r)
    }
}
