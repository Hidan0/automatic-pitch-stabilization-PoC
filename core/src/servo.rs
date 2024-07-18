use crate::Result;
use esp_idf_svc::hal::{
    gpio::OutputPin,
    ledc::{config::TimerConfig, LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, Resolution},
    peripheral::Peripheral,
    units::Hertz,
};

const MAX_DUTY_US: f32 = 2500.;
const MIN_DUTY_US: f32 = 500.;
const MAX_ANGLE: f32 = 180.;
const FREQ: f32 = 20_000.;
const RESOLUTION: Resolution = Resolution::Bits11;

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

    pub fn write_angle(&mut self, angle: u32) -> Result<()> {
        let angle_us = ((angle as f32 / MAX_ANGLE) * (MAX_DUTY_US - MIN_DUTY_US)) + MIN_DUTY_US;

        let duty: u32 = (angle_us * self.max_duty as f32 / FREQ) as u32;
        self.driver.set_duty(duty).unwrap();

        Ok(())
    }

    pub fn read_exp_angle(&mut self) -> u32 {
        let duty = self.driver.get_duty();
        let angle_us = (duty as f32 * FREQ) / self.max_duty as f32;

        ((angle_us - MIN_DUTY_US) / (MAX_DUTY_US - MIN_DUTY_US) * MAX_ANGLE) as u32
    }
}
