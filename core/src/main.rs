use esp_idf_svc::hal::{
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use orientation_controller::Controller;
use std::time::SystemTime;

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
    let mut cal = 0.;

    let start_time = SystemTime::now();

    for i in 1..=2000 {
        if i == 500 {
            let current = SystemTime::now();
            println!(
                "For 500 measurements took {}ms",
                current
                    .duration_since(start_time)
                    .unwrap_or_default()
                    .as_millis()
            );
        } else if i == 1000 {
            let current = SystemTime::now();
            println!(
                "For 1000 measurements took {}ms",
                current
                    .duration_since(start_time)
                    .unwrap_or_default()
                    .as_millis()
            );
        } else if i == 2000 {
            let current = SystemTime::now();
            println!(
                "For 2000 measurements took {}ms",
                current
                    .duration_since(start_time)
                    .unwrap_or_default()
                    .as_millis()
            );
        }

        cal += controller.get_roll().unwrap();
    }

    let cal_500 = cal / 500_f32;
    let cal_1000 = cal / 1000_f32;
    let cal_2000 = cal / 2000_f32;

    log::info!("Done callibration...");

    loop {
        let val = controller.get_roll().unwrap();
        println!(
            "{},{},{},{}",
            val,
            val - cal_500,
            val - cal_1000,
            val - cal_2000
        );
    }
}
