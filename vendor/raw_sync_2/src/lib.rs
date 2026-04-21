pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
/// Event implementations
pub mod events;
/// Lock implementations
pub mod locks;

pub enum Timeout {
    Infinite,
    Val(std::time::Duration),
}
