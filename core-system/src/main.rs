use self::orientation::Orientation;
use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use log::{error, warn};

mod error;
mod orientation;
mod prelude;

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

    loop {
        match orientation.update_orientation() {
            Ok(_) => {}
            Err(err) => error!("{}", err),
        };

        let angle = orientation.get_orientation();
        println!("{},{}", angle.x, angle.y);

        Delay::delay_ms(4);
    }
}
