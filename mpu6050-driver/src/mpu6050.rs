use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c;

/// Mpu6050 device address
const DEVICE_ADDR: u8 = 0x68;
/// Digital Low Pass Filter address
const DLPF_ADDR: u8 = 0x1A;
/// DLPF configuration of 10Hz bandwidth filter for Gyroscope
const DLPF_CFG_GYR_5: u8 = 0x05;
/// Gyro config address
const GYR_ADDR: u8 = 0x1B;
/// Gyro range configuration
const GYR_CONF_500: u8 = 0x8;
/// Gyro sensitivity for LSB
const GYR_500_LSB_SENS: f32 = 65.5;

/// High Byte Register Gyro x orientation
const GYRO_REGX_H: u8 = 0x43;
/// High Byte Register Gyro y orientation
const GYRO_REGY_H: u8 = 0x45;
/// High Byte Register Gyro z orientation
const GYRO_REGZ_H: u8 = 0x47;

/// Power management register
const PWR_MGMT_ADDR: u8 = 0x6B;

/// Maximum number of calibrations
const MAX_CALIBRATIONS: usize = 2000;

#[allow(dead_code)]
#[derive(Debug)]
pub struct Mpu6050<I2C> {
    i2c: I2C,
    rate_cal_roll: f32,
    rate_cal_pitch: f32,
    rate_cal_yaw: f32,
}

impl<I2C, E> Mpu6050<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    /// Create a new instance of the Mpu6050
    pub fn new(i2c: I2C) -> Result<Self, E> {
        Ok(Self {
            i2c,
            rate_cal_roll: 0.,
            rate_cal_pitch: 0.,
            rate_cal_yaw: 0.,
        })
    }

    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
        self.start_up(delay)?;
        self.configure()?;
        self.calibrate(delay)?;
        Ok(())
    }

    fn start_up<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
        delay.delay_ms(250);
        self.write_byte(PWR_MGMT_ADDR, 0x00)?;
        delay.delay_ms(100);
        Ok(())
    }

    fn calibrate<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
        for _ in 0..MAX_CALIBRATIONS {
            let (r, p, y) = self.raw_gyro()?;
            self.rate_cal_roll += r;
            self.rate_cal_pitch += p;
            self.rate_cal_yaw += y;
            delay.delay_ms(1);
        }

        self.rate_cal_roll /= MAX_CALIBRATIONS as f32;
        self.rate_cal_pitch /= MAX_CALIBRATIONS as f32;
        self.rate_cal_yaw /= MAX_CALIBRATIONS as f32;

        Ok(())
    }

    fn configure(&mut self) -> Result<(), E> {
        // switch on the low pass filter
        self.write_byte(DLPF_ADDR, DLPF_CFG_GYR_5)?;

        // set the sensitivity scale factor
        self.write_byte(GYR_ADDR, GYR_CONF_500)?;
        Ok(())
    }

    fn raw_gyro(&mut self) -> Result<(f32, f32, f32), E> {
        let r = self.read_2c_word(GYRO_REGX_H)?;
        let p = self.read_2c_word(GYRO_REGY_H)?;
        let y = self.read_2c_word(GYRO_REGZ_H)?;

        // converts the measurements to °/s
        Ok((
            r as f32 / GYR_500_LSB_SENS,
            p as f32 / GYR_500_LSB_SENS,
            y as f32 / GYR_500_LSB_SENS,
        ))
    }

    pub fn gyro(&mut self) -> Result<(f32, f32, f32), E> {
        let (r, p, y) = self.raw_gyro()?;
        Ok((
            r - self.rate_cal_roll,
            p - self.rate_cal_pitch,
            y - self.rate_cal_yaw,
        ))
    }

    fn write_byte(&mut self, register: u8, byte: u8) -> Result<(), E> {
        self.i2c.write(DEVICE_ADDR, &[register, byte])
    }

    fn read_2c_word(&mut self, register: u8) -> Result<i16, E> {
        let mut buffer = [0, 0];
        self.i2c.write_read(DEVICE_ADDR, &[register], &mut buffer)?;

        let high: u16 = buffer[0] as u16;
        let low: u16 = buffer[1] as u16;

        let word = (high << 8) | low;

        Ok(word as i16)
    }
}
