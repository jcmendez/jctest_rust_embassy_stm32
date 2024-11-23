use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Speed};
use embassy_time::{Duration, Timer};

pub(crate) enum BlinkSpeed {
    Slow,
    Medium,
    Fast,
}

pub(crate) struct Led<T: Pin> {
    output: Output<'static, T>,
    blink_ms: u16,
}

impl From<AnyPin> for Led<AnyPin> {
    fn from(pin: AnyPin) -> Self {
        Self::new(pin)
    }
}

impl<T: Pin> Led<T> {
    pub(crate) fn new(pin: T) -> Led<T> {
        let mut output = Output::new(pin, Level::High, Speed::Low);
        output.set_low();
        Led {
            output,
            blink_ms: 1000,
        }
    }

    pub(crate) async fn on(&mut self) {
        self.output.set_high();
    }

    pub(crate) async fn off(&mut self) {
        self.output.set_low();
    }

    pub(crate) async fn toggle(&mut self) {
        self.output.toggle();
    }

    pub(crate) fn set_blink_speed(&mut self, speed: BlinkSpeed) {
        self.blink_ms = match speed {
            BlinkSpeed::Slow => 1000,
            BlinkSpeed::Medium => 500,
            BlinkSpeed::Fast => 100,
        };
    }

    pub(crate) fn set_blink_ms(&mut self, ms: u16) {
        self.blink_ms = ms;
    }

    pub(crate) async fn toggle_task(&mut self) {
        loop {
            self.toggle().await;
            Timer::after(Duration::from_millis(self.blink_ms as u64)).await;
        }
    }
}
