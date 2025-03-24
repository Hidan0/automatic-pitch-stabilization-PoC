use derive_more::Display;
use esp_idf_svc::{
    hal::{
        gpio::OutputPin,
        ledc::{
            config::TimerConfig, LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, Resolution,
        },
        peripheral::Peripheral,
        units::Hertz,
    },
    sys::EspError,
};

use crate::Result;

const MAX_DUTY_US: f32 = 2400.;
const MIN_DUTY_US: f32 = 500.;
const MAX_MINUS_MIN_DUTY: f32 = MAX_DUTY_US - MIN_DUTY_US;

const MAX_ANGLE: f32 = 90.;
const MIN_ANGLE: f32 = -90.;
const MAX_MINUS_MIN_ANGLE: f32 = MAX_ANGLE - MIN_ANGLE;

const FREQ: f32 = 20_000.;
const FREQ_HZ: Hertz = Hertz(50);
const RESOLUTION: Resolution = Resolution::Bits11;

#[derive(Debug, Display)]
pub enum Error {
    #[display("Failed to create LedcTimerDriver. Error: {_0:?}")]
    CreateLedcTimerDriver(EspError),
    #[display("Failed to create LedcDriver. Error: {_0:?}")]
    CreateLedcDriver(EspError),
    #[display("Failed to set duty cycle with value {value}. Error: {error:?}")]
    SetDuty { value: u32, error: EspError },
}

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
            frequency: FREQ_HZ,
            resolution: RESOLUTION,
        };
        let timer_driver =
            LedcTimerDriver::new(timer, &timer_config).map_err(Error::CreateLedcTimerDriver)?;
        let driver =
            LedcDriver::new(channel, timer_driver, gpio).map_err(Error::CreateLedcDriver)?;
        log::info!("LedcDriver created.");

        let max_duty = driver.get_max_duty() - 1;

        Ok(Self { driver, max_duty })
    }

    pub fn write_angle(&mut self, angle: i16) -> Result<()> {
        let angle_us =
            (MAX_MINUS_MIN_DUTY / MAX_MINUS_MIN_ANGLE * (angle as f32 - MIN_ANGLE)) + MIN_DUTY_US;

        let duty = (self.max_duty as f32 * angle_us / FREQ) as u32;
        self.driver.set_duty(duty).map_err(|e| Error::SetDuty {
            value: duty,
            error: e,
        })?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn read_exp_angle(&mut self) -> i16 {
        let duty = self.driver.get_duty();
        let angle_us = (duty as f32 * FREQ) / self.max_duty as f32;

        ((angle_us - MIN_DUTY_US) * (MAX_MINUS_MIN_ANGLE / MAX_MINUS_MIN_DUTY) + MIN_ANGLE) as i16
    }
}
