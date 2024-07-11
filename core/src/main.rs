use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use servo::ServoSG90;

type Error = Box<dyn std::error::Error>;
type Result<T> = std::result::Result<T, Error>;

mod servo;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut servo = ServoSG90::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio7,
    )?;

    loop {
        for i in 0..180 {
            servo.write_angle(i)?;

            log::info!("Writing angle {}", i);
            log::info!("Reading angle {}\n", servo.read_angle());

            FreeRtos::delay_ms(15)
        }

        for i in (0..180).rev() {
            servo.write_angle(i)?;
            FreeRtos::delay_ms(15)
        }

        FreeRtos::delay_ms(10000);
    }
}
