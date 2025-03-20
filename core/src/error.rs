use derive_more::From;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, From)]
pub enum Error {
    // -- Modules
    #[from]
    Servo(crate::servo::Error),
    #[from]
    PitchControl(crate::pitch_control::Error),

    // -- Externals
    #[from]
    Esp(esp_idf_svc::sys::EspError),
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
