use anyhow::Result;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, Resolution};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::units::FromValueType;

pub const MAX_DUTY_US: f32 = 2500.;
pub const MIN_DUTY_US: f32 = 500.;
pub const MAX_ANGLE: f32 = 180.;
pub const FREQ: f32 = 20_000.;
pub const RESOLUTION: Resolution = Resolution::Bits11;

pub struct ServoSG90<'d> {
    driver: LedcDriver<'d>,
    max_duty: u32,
}

impl<'d> ServoSG90<'d> {
    pub fn new<C: LedcChannel, T: LedcTimer>(
        channel: impl Peripheral<P = C> + 'd,
        timer: impl Peripheral<P = T> + 'd,
        gpio: impl Peripheral<P = impl OutputPin> + 'd,
    ) -> Result<Self> {
        let timer_driver = LedcTimerDriver::<'d>::new(
            timer,
            &TimerConfig::default()
                .frequency(50.Hz())
                .resolution(RESOLUTION),
        )?;

        let driver = LedcDriver::new(channel, timer_driver, gpio)?;
        let max_duty = driver.get_max_duty() - 1;

        Ok(Self { driver, max_duty })
    }

    pub fn write_angle(&mut self, angle: u32) -> Result<()> {
        let angle_us = ((angle as f32 / MAX_ANGLE) * (MAX_DUTY_US - MIN_DUTY_US)) + MIN_DUTY_US;

        let duty: u32 = (angle_us * self.max_duty as f32 / FREQ) as u32;
        self.driver.set_duty(duty)?;

        Ok(())
    }

    pub fn read_angle(&mut self) -> u32 {
        let duty = self.driver.get_duty();
        let angle_us = (duty as f32 * FREQ) / self.max_duty as f32;

        ((angle_us - MIN_DUTY_US) / (MAX_DUTY_US - MIN_DUTY_US) * MAX_ANGLE) as u32
    }
}
