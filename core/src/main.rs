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
        peripherals.pins.gpio6,
    )?;

    let trials = 10;
    let mut tot = 0;

    for _ in 0..trials {
        for i in -90..=90 {
            servo.write_angle(i)?;
            tot += i - servo.read_exp_angle();
        }

        for i in (-90..=90).rev() {
            servo.write_angle(i)?;
            tot += i - servo.read_exp_angle();
        }
    }

    println!(
        "Average conversion error: {}",
        tot as f32 / (trials * 360) as f32
    );

    Ok(())
}
