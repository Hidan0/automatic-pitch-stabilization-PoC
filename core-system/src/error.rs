#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Generic: {0}")]
    Generic(String),

    #[error("Can not initialize MPU6050: {0}")]
    MpuInit(String),

    #[error("Gyro calibration failed")]
    GyroCalibration,

    #[error("Can not read Gyroscope")]
    GyroRead,
    #[error("Can not read Accelerometer")]
    AccelRead,
}
