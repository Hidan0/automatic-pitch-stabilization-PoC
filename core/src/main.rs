type Error = Box<dyn std::error::Error>;
type Result<T> = std::result::Result<T, Error>;

mod servo;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");
    Ok(())
}
