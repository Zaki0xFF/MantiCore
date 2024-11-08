use core::convert::Infallible;
use embedded_hal::digital::OutputPin;

pub(crate) struct NoOutputPin;

impl embedded_hal::digital::ErrorType for NoOutputPin {
    type Error = Infallible;
}

impl OutputPin for NoOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
