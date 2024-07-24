use embedded_hal::delay::DelayNs;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::i2c::I2cDriver;
use mpu6050::device::{AccelRange, GyroRange};
use mpu6050::Mpu6050;

use crate::Result;

type MpuDriver<'a> = Mpu6050<I2cDriver<'a>>;

pub struct Controller<'a> {
    mpu: MpuDriver<'a>,
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

    pub fn new(i2c: I2cDriver<'a>) -> Result<Controller<'a>> {
        log::info!("Starting Mpu set up...");
        let mpu = Self::setup_mpu(&mut FreeRtos, i2c)?;
        log::info!("Finished Mpu set up.");

        Ok(Self { mpu })
    }

    pub fn get_roll(&mut self) -> Result<f32> {
        let gyro = self.mpu.get_gyro().unwrap();

        Ok(gyro.x)
    }
}
