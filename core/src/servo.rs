use crate::Result;
use esp_idf_svc::hal::{
    gpio::OutputPin,
    ledc::{config::TimerConfig, LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, Resolution},
    peripheral::Peripheral,
    units::Hertz,
};

pub const MAX_DUTY_US: f32 = 2500.;
pub const MIN_DUTY_US: f32 = 500.;
pub const MAX_ANGLE: f32 = 180.;
pub const FREQ: f32 = 20_000.;
pub const RESOLUTION: Resolution = Resolution::Bits11;

pub struct ServoSG90<'a> {
    driver: LedcDriver<'a>,
    max_duty: u32,
}

impl<'a> ServoSG90<'a> {
    pub fn new<C: LedcChannel<SpeedMode = <T>::SpeedMode>, T: LedcTimer + 'a>(
        channel: impl Peripheral<P = C> + 'a,
        timer: impl Peripheral<P = T> + 'a,
        gpio: impl Peripheral<P = impl OutputPin> + 'a,
    ) -> Result<Self> {
        let timer_config = TimerConfig {
            frequency: Hertz(50),
            resolution: RESOLUTION,
        };
        let timer_driver = LedcTimerDriver::new(timer, &timer_config).unwrap();
        let driver = LedcDriver::new(channel, timer_driver, gpio).unwrap();

        let max_duty = driver.get_max_duty() - 1;

        Ok(Self { driver, max_duty })
    }
}
