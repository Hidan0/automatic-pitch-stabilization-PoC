use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use gyro_controls::gyro_controls::GyroControls;

const DELTA_T: f32 = 0.004;

const SD_R_RATE: f32 = 0.06; // Standard Deviation of the rotation rate r
const SD_ACCEL: f32 = 0.05; // Standard Deviation of the accelerometer

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

    let (mut predicted_angle, mut predicted_uncertainty) = (0., 0.);

    let mut integrated_angle = 0.;

    loop {
        let (rate_roll, _, _) = gyro_controls.get_gyro();
        let (roll_angle, _) = gyro_controls.get_orientation();

        (predicted_angle, predicted_uncertainty) = kalman_1d(
            predicted_angle,
            predicted_uncertainty,
            rate_roll,
            roll_angle,
        );

        integrated_angle = integrated_angle + rate_roll * DELTA_T;

        println!(
            "{},{},{}",
            predicted_angle.to_degrees(), // offset to make it visible
            roll_angle.to_degrees(),
            integrated_angle.to_degrees()
        );

        Delay::delay_ms(4);
    }
}

fn kalman_1d(mut state: f32, mut uncertainty: f32, input: f32, measurement: f32) -> (f32, f32) {
    state = state + DELTA_T * input;
    uncertainty = uncertainty + DELTA_T * DELTA_T * SD_R_RATE * SD_R_RATE;

    let gain = uncertainty / (uncertainty + SD_ACCEL * SD_ACCEL);

    state = state + gain * (measurement - state);
    uncertainty = (1. - gain) * uncertainty;

    (state, uncertainty)
}
