use crate::servo::ServoSG90;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use pitch_control::{PIController, PitchEstimator};

mod error;
mod pitch_control;
mod servo;

pub use self::error::{Error, Result};

const UPDATE_TIME_MS: u8 = 4;
const ERROR_SIGNAL_UPDATE_TIME_MS: u32 = 500;
const DELTA_TIME: f32 = UPDATE_TIME_MS as f32 / 1000.;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = match Peripherals::take() {
        Ok(p) => p,
        Err(e) => {
            log::error!("Can not take system peripherals.\n{e}");
            return;
        }
    };

    let mut error_led = match PinDriver::input_output(peripherals.pins.gpio8) {
        Ok(e_r) => e_r,
        Err(e) => {
            log::error!("Can not create pin driver for error led.\n{e}");
            return;
        }
    };

    error_led
        .set_low()
        .unwrap_or_else(|e| log::warn!("Can not set low error led, skipping...\nError:\n{e}"));

    let mut error_signal_loop = || loop {
        error_led
            .toggle()
            .unwrap_or_else(|e| log::warn!("Can not set high error led, skipping...\nError:\n{e}"));
        FreeRtos::delay_ms(ERROR_SIGNAL_UPDATE_TIME_MS);
    };

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let cal_led = peripherals.pins.gpio7;

    let i2c_config = I2cConfig::default().baudrate(Hertz(200000));
    let i2c_driver = match I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config) {
        Ok(i2c) => i2c,
        Err(e) => {
            log::error!("Failed to create i2c driver.\n{e}");
            error_signal_loop();
            return;
        }
    };

    let mut pitch_estimator = match PitchEstimator::new(i2c_driver, cal_led) {
        Ok(pe) => pe,
        Err(e) => {
            log::error!("Can not create pitch estimator:\n{e}");
            error_signal_loop();
            return;
        }
    };

    let mut pitch_surface = match ServoSG90::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio6,
    ) {
        Ok(p) => p,
        Err(e) => {
            log::error!("Can not create servo driver for pitch surface:\n{e}");
            error_signal_loop();
            return;
        }
    };

    if let Err(e) = pitch_surface.write_angle(0) {
        log::error!("Can not move the pitch surface to neutral position:\n{e}");
        error_signal_loop();
        return;
    }

    const K_P: f32 = 3.;
    const K_I: f32 = 0.02;

    let mut pi_controller = match pitch_estimator.pitch() {
        Ok(initial_pitch) => PIController::new(K_P, K_I, initial_pitch.to_degrees()),
        Err(e) => {
            log::error!("Can not get initial pitch angle.\n{e}");
            error_signal_loop();
            return;
        }
    };

    loop {
        if let Err(e) = pitch_estimator.update(&DELTA_TIME) {
            log::error!("Something went wrong during pitch estimation:\n{e}");
            error_signal_loop();
            return;
        }

        let p_angle = match pitch_estimator.pitch() {
            Ok(p) => p.to_degrees(),
            Err(e) => {
                log::error!("Can not get current pitch angle:\n{e}");
                error_signal_loop();
                return;
            }
        };

        let output = pi_controller.u(p_angle, &DELTA_TIME);
        pitch_surface
            .write_angle(-(output.clamp(-90., 90.) as i16))
            .unwrap();

        FreeRtos::delay_ms(UPDATE_TIME_MS.into());
    }
}
