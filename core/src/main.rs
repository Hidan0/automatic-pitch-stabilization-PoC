use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::Hertz;
use orientation_controller::Controller;

type Error = Box<dyn std::error::Error>;
type Result<T> = std::result::Result<T, Error>;

mod orientation_controller;
mod servo;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let i2c_config = I2cConfig::default().baudrate(Hertz(400000));
    let i2c_driver = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config).unwrap();

    let mut controller = Controller::new(i2c_driver).unwrap();

    loop {
        println!("{}", controller.get_roll().unwrap());
    }
}
