use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use gyro_controls::gyro_controls::GyroControls;
use log::*;

fn main() {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut cal_led = PinDriver::output(peripherals.pins.gpio6).unwrap();
    cal_led.set_high().unwrap();

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let config = I2cConfig::new().baudrate(400000.Hz());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    let mut gyro_controls = GyroControls::init(i2c, &mut cal_led);

    let mut last_roll_rate: f32 = 0.;
    let mut roll_angle: f32 = 0.;

    const DELTA_TIME: f32 = 0.004;

    loop {
        let (roll_rate, _, _) = gyro_controls.get_gyro();

        roll_angle += 0.5 * (roll_rate + last_roll_rate) * DELTA_TIME;
        last_roll_rate = roll_rate;

        info!("Roll angle: {}", roll_angle.to_degrees());

        Delay::delay_ms(4)
    }
}
