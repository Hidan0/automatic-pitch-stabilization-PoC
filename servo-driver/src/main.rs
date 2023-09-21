use anyhow::Result;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::prelude::*;
use esp_idf_sys as _;
use log::info;
use servo_driver::ServoSG90; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

fn main() -> Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut servo = ServoSG90::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio6,
    )?;

    loop {
        for i in 0..180 {
            servo.write_angle(i)?;

            info!("Writing angle {}", i);
            info!("Reading angle {}\n", servo.read_angle());

            FreeRtos::delay_ms(15)
        }

        for i in (0..180).rev() {
            servo.write_angle(i)?;
            FreeRtos::delay_ms(15)
        }

        FreeRtos::delay_ms(30000);
    }
}
