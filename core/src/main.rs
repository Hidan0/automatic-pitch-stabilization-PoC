use crate::servo::ServoSG90;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::Hertz,
};
use pitch_control::{PIController, PitchEstimator};

type Error = Box<dyn std::error::Error>;
type Result<T> = std::result::Result<T, Error>;

mod pitch_control;
mod servo;

const UPDATE_TIME_MS: u8 = 4;
const DELTA_TIME: f32 = UPDATE_TIME_MS as f32 / 1000.;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let cal_led = peripherals.pins.gpio7;

    let i2c_config = I2cConfig::default().baudrate(Hertz(200000));
    let i2c_driver = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config).unwrap();

    let mut pitch_estimator = PitchEstimator::new(i2c_driver, cal_led).unwrap();

    let mut roll = ServoSG90::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio6,
    )
    .unwrap();

    roll.write_angle(0).unwrap();

    const K_P: f32 = 2.;
    const K_I: f32 = 0.;

    let mut pi_controller =
        PIController::new(K_P, K_I, pitch_estimator.pitch().unwrap().to_degrees());

    loop {
        pitch_estimator.update(&DELTA_TIME).unwrap();

        let p_angle = pitch_estimator.pitch().unwrap().to_degrees();
        let output = pi_controller.u(p_angle, &DELTA_TIME);
        roll.write_angle(-(output.clamp(-90., 90.) as i16)).unwrap();

        FreeRtos::delay_ms(UPDATE_TIME_MS.into());
    }
}
