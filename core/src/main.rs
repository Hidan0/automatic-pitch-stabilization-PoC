use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use orientation_controller::Controller;

type Error = Box<dyn std::error::Error>;
type Result<T> = std::result::Result<T, Error>;

mod orientation_controller;
mod servo;

const UPDATE_TIME_MS: u8 = 4;
const DELTA_TIME: f32 = UPDATE_TIME_MS as f32 / 1000.;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let i2c_config = I2cConfig::default().baudrate(Hertz(400000));
    let i2c_driver = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config).unwrap();

    let mut controller = Controller::new(i2c_driver).unwrap();

    let mut roll_angle = 0.;

    let compl_filters = [0.05, 0.02, 0.5, 2., 5.];
    let mut compl_angles = [0., 0., 0., 0., 0.];

    println!("gyro,accel,compl005,compl002,compl05,compl2,compl5");
    loop {
        let roll_rate = controller.get_roll().unwrap();
        roll_angle += roll_rate * DELTA_TIME;
        let abs_angle = controller.get_accel_roll().unwrap();

        for i in 0..compl_angles.len() {
            compl_angles[i] = abs_angle * compl_filters[i]
                + (1. - compl_filters[i]) * (compl_angles[i] + roll_rate * DELTA_TIME);
        }

        println!(
            "{},{},{},{},{},{},{}",
            roll_angle,
            abs_angle,
            compl_angles[0],
            compl_angles[1],
            compl_angles[2],
            compl_angles[3],
            compl_angles[4]
        );

        FreeRtos::delay_ms(UPDATE_TIME_MS.into());
    }
}
