use embedded_hal::blocking::delay::DelayMs;
use esp_idf_hal::delay::{self, FreeRtos};
use esp_idf_hal::gpio::{Gpio6, Output, PinDriver};
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_sys as _;
use log::info;
use mpu6050::Mpu6050;
use std::time::SystemTime;

pub struct GyroControls {
    mpu: Mpu6050<I2cDriver<'static>>,
    cal_r: f32,
    cal_p: f32,
    cal_y: f32,
}

impl GyroControls {
    pub fn init(i2c: I2cDriver<'static>, info_led: &mut PinDriver<Gpio6, Output>) -> GyroControls {
        info!("Starting GyroControls initialization...");

        info!("Starting Mpu setup...");
        let mut mpu = Self::setup_mpu(&mut FreeRtos, i2c);
        info!("Finished Mpu setup");

        let start_time = SystemTime::now();
        info!("Starting calibration...");
        let (cal_r, cal_p, cal_y) = Self::calibrate_gyro(&mut delay::Delay, &mut mpu, info_led);
        let end_time = SystemTime::now();
        info!(
            "Finished calibration, {}ms",
            end_time.duration_since(start_time).unwrap().as_millis()
        );

        info!("Finished GyroControls initialization.");
        Self {
            mpu,
            cal_r,
            cal_p,
            cal_y,
        }
    }

    pub fn get_gyro(&mut self) -> (f32, f32, f32) {
        let gyro = self.mpu.get_gyro().unwrap();
        (
            gyro.x - self.cal_r,
            gyro.y - self.cal_p,
            gyro.z - self.cal_y,
        )
    }

    fn calibrate_gyro<D: DelayMs<u16>>(
        delay: &mut D,
        mpu: &mut Mpu6050<I2cDriver>,
        info_led: &mut PinDriver<Gpio6, Output>,
    ) -> (f32, f32, f32) {
        let (mut cal_r, mut cal_p, mut cal_y) = (0., 0., 0.);
        const NUMBER_CAL: usize = 500;

        for _ in 0..NUMBER_CAL {
            cal_r += mpu.get_gyro().unwrap().x;
            cal_p += mpu.get_gyro().unwrap().y;
            cal_y += mpu.get_gyro().unwrap().z;
            delay.delay_ms(1);
        }

        info_led.set_low().unwrap();

        (
            cal_r / NUMBER_CAL as f32,
            cal_p / NUMBER_CAL as f32,
            cal_y / NUMBER_CAL as f32,
        )
    }

    fn setup_mpu<D: DelayMs<u8>>(
        delay: &mut D,
        i2c: I2cDriver<'static>,
    ) -> Mpu6050<I2cDriver<'static>> {
        let mut mpu = Mpu6050::new(i2c);
        mpu.init(delay).unwrap();

        mpu.set_temp_enabled(false).unwrap();
        mpu.set_gyro_range(mpu6050::device::GyroRange::D500)
            .unwrap();

        mpu
    }
}
