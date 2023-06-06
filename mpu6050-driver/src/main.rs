use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use mpu6050_driver::mpu6050::Mpu6050;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;

    let config = I2cConfig::new().baudrate(115200.Hz());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    let mut mpu = Mpu6050::new(i2c).unwrap();
    mpu.init(&mut FreeRtos).unwrap();

    loop {
        let (roll_r, pitch_r, yaw_r) = mpu.gyro().unwrap();
        println!(
            "Roll rate: {:.2}, Pitch rate: {:.2}, Yar rate: {:.2}",
            roll_r, pitch_r, yaw_r
        );
        FreeRtos::delay_ms(50);
    }
}
