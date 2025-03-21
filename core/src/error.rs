use derive_more::{Display, From};

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, From, Display)]
pub enum Error {
    // -- Modules
    #[from]
    Servo(crate::servo::Error),
    #[from]
    PitchControl(crate::pitch_control::Error),

    // -- Externals
    #[from]
    #[display("ESP error: {_0}")]
    Esp(esp_idf_svc::sys::EspError),
}
