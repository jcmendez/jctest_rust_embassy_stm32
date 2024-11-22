use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{DMA1_CH3, DMA1_CH4, I2C1};
use embassy_time::{Duration, Timer};

type I2cType = I2c<'static, I2C1, DMA1_CH3, DMA1_CH4>;
pub(crate) struct RelaySet {
    state: u8,
    i2c: I2cType,
}

impl RelaySet {
    pub(crate) fn new(i2c: I2cType) -> RelaySet {
        RelaySet { state: 0, i2c }
    }

    pub(crate) async fn restart_device(&mut self, relay: u8) {
        self.set(relay).await;
        Timer::after(Duration::from_millis(2000)).await;
        self.clear(relay).await;
    }

    // Private function to set the state of a relay (i.e. engage the relay). Since the
    // connection is physically made on the NC (Normally Closed) side of the relay, setting
    // the relay means opening the circuit, which allows us to restart the device connected
    // The relay parameter is the relay number (0-3)
    async fn set(&mut self, relay: u8) {
        assert!(relay < 4);
        self.state |= 1 << relay;
        self.i2c.write(0x11, &[0x10, self.state]).await.ok();
    }

    // Private function to clear the state of a relay
    // The relay parameter is the relay number (0-3)
    async fn clear(&mut self, relay: u8) {
        assert!(relay < 4);
        self.state &= !(1 << relay);
        self.i2c.write(0x11, &[0x10, self.state]).await.ok();
    }

    fn get(&self, relay: u8) -> bool {
        assert!(relay < 4);
        self.state & (1 << relay) != 0
    }
}
