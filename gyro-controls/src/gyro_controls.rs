use embedded_hal::blocking::delay::DelayMs;
use esp_idf_hal::delay::{self, FreeRtos};
use esp_idf_hal::gpio::{Gpio6, Output, PinDriver};
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_sys as _;
use log::info;
use mpu6050::Mpu6050;
use std::time::SystemTime;

struct GyroCalibration {
    r: f32,
    p: f32,
    y: f32,
}

pub struct GyroControls {
    mpu: Mpu6050<I2cDriver<'static>>,
    gyro_cal: GyroCalibration,
}

impl GyroControls {
    pub fn init(i2c: I2cDriver<'static>, info_led: &mut PinDriver<Gpio6, Output>) -> GyroControls {
        info!("Starting GyroControls initialization...");

        info!("Starting Mpu setup...");
        let mut mpu = Self::setup_mpu(&mut FreeRtos, i2c);
        info!("Finished Mpu setup");

        let start_time = SystemTime::now();
        info!("Starting gyroscope calibration...");
        let gyro_cal = Self::calibrate_gyro(&mut delay::Delay, &mut mpu);
        let end_time = SystemTime::now();
        info!(
            "Finished gyroscope calibration, {}ms",
            end_time.duration_since(start_time).unwrap().as_millis()
        );

        info_led.set_low().unwrap();

        info!("Finished GyroControls initialization.");
        Self { mpu, gyro_cal }
    }

    pub fn get_gyro(&mut self) -> (f32, f32, f32) {
        let gyro = self.mpu.get_gyro().unwrap();
        (
            gyro.x - self.gyro_cal.r,
            gyro.y - self.gyro_cal.p,
            gyro.z - self.gyro_cal.y,
        )
    }

    pub fn get_orientation(&mut self) -> (f32, f32) {
        let angles = self.mpu.get_acc_angles().unwrap();

        (angles.x.to_degrees(), angles.y.to_degrees())
    }

    fn calibrate_gyro<D: DelayMs<u16>>(
        delay: &mut D,
        mpu: &mut Mpu6050<I2cDriver>,
    ) -> GyroCalibration {
        let (mut cal_r, mut cal_p, mut cal_y) = (0., 0., 0.);
        const NUMBER_CAL: usize = 500;

        for _ in 0..NUMBER_CAL {
            cal_r += mpu.get_gyro().unwrap().x;
            cal_p += mpu.get_gyro().unwrap().y;
            cal_y += mpu.get_gyro().unwrap().z;
            delay.delay_ms(1);
        }

        GyroCalibration {
            r: cal_r / NUMBER_CAL as f32,
            p: cal_p / NUMBER_CAL as f32,
            y: cal_y / NUMBER_CAL as f32,
        }
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

        mpu.set_accel_range(mpu6050::device::AccelRange::G8)
            .unwrap();

        mpu
    }
}
