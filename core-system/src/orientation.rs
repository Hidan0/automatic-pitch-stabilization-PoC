use crate::prelude::*;
use embedded_hal::blocking::delay::DelayMs;
use esp_idf_hal::delay::{self, FreeRtos};
use esp_idf_hal::gpio::{Gpio6, Output, PinDriver};
use esp_idf_hal::i2c::I2cDriver;
use log::info;
use mpu6050::device::{AccelRange, GyroRange};
use mpu6050::Mpu6050;
use nalgebra::Vector2;
use std::time::SystemTime;

const DELTA_T: f32 = 0.004;
const SD_R_RATE: f32 = 0.06; // Standard Deviation of the rotation rate
const SD_ACCEL: f32 = 0.05; // Standard Deviation of the accelerometer

struct GyroCalibration {
    r: f32,
    p: f32,
}

pub struct Orientation {
    mpu: Mpu6050<I2cDriver<'static>>,
    gyro_cal: GyroCalibration,
    angle: Vector2<f32>,
    uncertainty: Vector2<f32>,
}

impl Orientation {
    fn setup_mpu<D: DelayMs<u8>>(
        delay: &mut D,
        i2c: I2cDriver<'static>,
    ) -> Result<Mpu6050<I2cDriver<'static>>> {
        let mut mpu = Mpu6050::new(i2c);
        mpu.init(delay)
            .map_err(|_| Error::MpuInit("initialization".to_string()))?;

        mpu.set_temp_enabled(false)
            .map_err(|_| Error::MpuInit("disabling temperature".to_string()))?;
        mpu.set_gyro_range(GyroRange::D500)
            .map_err(|_| Error::MpuInit("setting gyro range".to_string()))?;
        mpu.set_accel_range(AccelRange::G8)
            .map_err(|_| Error::MpuInit("setting accelerometer range".to_string()))?;

        Ok(mpu)
    }

    fn calibrate_gyro<D: DelayMs<u16>>(
        delay: &mut D,
        mpu: &mut Mpu6050<I2cDriver>,
    ) -> Result<GyroCalibration> {
        let (mut cal_r, mut cal_p) = (0., 0.);
        const NUMBER_CAL: usize = 500;

        for _ in 0..NUMBER_CAL {
            cal_r += mpu.get_gyro().map_err(|_| Error::GyroCalibration)?.x;
            cal_p += mpu.get_gyro().map_err(|_| Error::GyroCalibration)?.y;
            delay.delay_ms(1);
        }

        Ok(GyroCalibration {
            r: cal_r / NUMBER_CAL as f32,
            p: cal_p / NUMBER_CAL as f32,
        })
    }

    pub fn init(
        i2c: I2cDriver<'static>,
        info_led: &mut PinDriver<Gpio6, Output>,
    ) -> Result<Orientation> {
        info!("Starting Orientation initialization...");

        info!("Starting Mpu setup...");
        let mut mpu = Self::setup_mpu(&mut FreeRtos, i2c)?;
        info!("Finished Mpu setup");

        let start_time = SystemTime::now();
        info!("Starting gyroscope calibration...");
        let gyro_cal = Self::calibrate_gyro(&mut delay::Delay, &mut mpu)?;
        let end_time = SystemTime::now();
        info!(
            "Finished gyroscope calibration, {}ms",
            end_time.duration_since(start_time).unwrap().as_millis()
        );

        info_led
            .set_low()
            .map_err(|_| Error::Generic("Can not set calibration led to low".to_string()))?;

        info!("Finished Orientation initialization");
        Ok(Self {
            mpu,
            gyro_cal,
            angle: Vector2::new(0., 0.),
            uncertainty: Vector2::new(0., 0.),
        })
    }

    fn get_gyro(&mut self) -> Result<Vector2<f32>> {
        let gyro = self.mpu.get_gyro().map_err(|_| Error::GyroRead)?;
        Ok(Vector2::new(
            gyro.x - self.gyro_cal.r,
            gyro.y - self.gyro_cal.p,
        ))
    }

    pub fn update_orientation(&mut self) -> Result<()> {
        let gyro = self.get_gyro()?;
        let accel = self.mpu.get_acc_angles().map_err(|_| Error::AccelRead)?;

        (self.angle, self.uncertainty) = kalman(self.angle, self.uncertainty, gyro, accel);

        Ok(())
    }

    pub fn get_orientation(&self) -> Vector2<f32> {
        self.angle
    }
}

fn kalman(
    mut state: Vector2<f32>,
    mut uncertainty: Vector2<f32>,
    input: Vector2<f32>,
    measurement: Vector2<f32>,
) -> (Vector2<f32>, Vector2<f32>) {
    state = Vector2::new(state.x + DELTA_T * input.x, state.y + DELTA_T * input.y);

    uncertainty = Vector2::new(
        uncertainty.x + DELTA_T * DELTA_T * SD_R_RATE * SD_R_RATE,
        uncertainty.y + DELTA_T * DELTA_T * SD_R_RATE * SD_R_RATE,
    );

    let gain = Vector2::new(
        uncertainty.x / (uncertainty.x + SD_ACCEL * SD_ACCEL),
        uncertainty.y / (uncertainty.y + SD_ACCEL * SD_ACCEL),
    );

    state = Vector2::new(
        state.x + gain.x * (measurement.x - state.x),
        state.y + gain.y * (measurement.y - state.y),
    );

    uncertainty = Vector2::new((1. - gain.x) * uncertainty.x, (1. - gain.y) * uncertainty.y);

    (state, uncertainty)
}
