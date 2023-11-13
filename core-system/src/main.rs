use self::orientation::Orientation;
use self::servo::ServoSG90;
use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use log::{error, warn};

mod error;
mod orientation;
mod prelude;
mod servo;

fn main() {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = match Peripherals::take() {
        Some(p) => p,
        None => {
            error!("Failed to take peripherals");
            panic!();
        }
    };

    let mut cal_led = match PinDriver::output(peripherals.pins.gpio6) {
        Ok(pin) => pin,
        Err(_) => {
            error!("Failed to initialize calibration LED");
            panic!();
        }
    };

    match cal_led.set_high() {
        Ok(_) => (),
        Err(_) => warn!("Failed to set calibration LED to high"),
    }

    let mut elevator = match ServoSG90::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio7,
    ) {
        Ok(servo) => servo,
        Err(err) => {
            error!("{}", err);
            panic!();
        }
    };

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let config = I2cConfig::new().baudrate(400000.Hz());
    let i2c = match I2cDriver::new(peripherals.i2c0, sda, scl, &config) {
        Ok(i2c) => i2c,
        Err(_) => {
            error!("Failed to initialize I2C");
            panic!();
        }
    };

    let mut orientation = match Orientation::init(i2c, &mut cal_led) {
        Ok(orientation) => orientation,
        Err(err) => {
            error!("{}", err);
            panic!();
        }
    };

    const K: f32 = 1.0;
    const DESIRED_ANGLE: f32 = 0.;

    loop {
        match orientation.update_orientation() {
            Ok(_) => {}
            Err(err) => error!("{}", err),
        };

        let x_angle = orientation.get_orientation().x;
        let error = DESIRED_ANGLE - x_angle;

        let mut p_term = (K * error).to_degrees() + 90.;

        if p_term > 180. {
            p_term = 18.;
        } else if p_term < 0. {
            p_term = 0.;
        }

        println!("{p_term}");

        match elevator.write_angle(p_term as u32) {
            Ok(_) => {}
            Err(err) => error!("{}", err),
        };

        Delay::delay_ms(4);
    }
}
