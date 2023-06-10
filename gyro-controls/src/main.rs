use embedded_hal::blocking::delay::DelayMs;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio6, Output, PinDriver};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use mpu6050::Mpu6050; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

fn main() {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    let mut cal_led = PinDriver::output(peripherals.pins.gpio6).unwrap();
    cal_led.set_high().unwrap();

    let scl = peripherals.pins.gpio5;
    let sda = peripherals.pins.gpio4;

    let config = I2cConfig::new().baudrate(115200.Hz());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    let mut mpu = setup_mpu(&mut FreeRtos, i2c);

    loop {
        println!("{}", mpu.get_gyro().unwrap().x);
        FreeRtos::delay_ms(50);
    }
}

#[allow(dead_code)]
fn calibrate_gyro<D: DelayMs<u8>>(delay: &mut D, info_led: &mut PinDriver<Gpio6, Output>) {
    delay.delay_ms(150);
    info_led.set_low().unwrap();
}

fn setup_mpu<D: DelayMs<u8>>(
    delay: &mut D,
    i2c: I2cDriver<'static>,
) -> Mpu6050<I2cDriver<'static>> {
    let mut mpu = Mpu6050::new(i2c);
    mpu.init(delay).unwrap();

    mpu.set_temp_enabled(false).unwrap();
    mpu.set_gyro_range(mpu6050::device::GyroRange::D500)
        .unwrap();

    mpu
}
