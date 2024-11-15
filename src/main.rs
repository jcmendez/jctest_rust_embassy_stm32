#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::{rcc::{Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource}, usart::UartTx};
use embassy_stm32::rng::Rng;
use embassy_stm32::usart::Config as UsartConfig;
use embassy_stm32::{bind_interrupts, peripherals, rng, Config};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_time::{Duration, Timer};
use heapless::String;
use fmt::info;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.hsi = true;
    config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL18,
        divp: None,
        divq: Some(PllQDiv::DIV6), // 48Mhz (16 / 1 * 18 / 6)
        divr: Some(PllRDiv::DIV4), // sysclk 72Mhz clock (16 / 1 * 18 / 4)
    });
    let p = embassy_stm32::init(config);
    let usart_config = UsartConfig::default();
    // PB11, PC1 or PA2 could be used as TX pin for LPUART1
    let mut usart = UartTx::new(p.LPUART1, p.PC1, p.DMA1_CH1, usart_config).unwrap();

    info!("Hello, World!");
    let button = Input::new(p.PC13, Pull::Up);
    let mut led = Output::new(p.PC8, Level::High, Speed::Low);
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut buf = [0u8; 16];


    loop {
        if button.is_high() {
            led.set_high();
            info!("Button is pressed");
            let start = embassy_time::Instant::now();
            while embassy_time::Instant::now() - start < Duration::from_secs(15) {
                unwrap!(rng.async_fill_bytes(&mut buf).await);
                info!("random bytes: {:02x}", buf); 
                let mut s: String<128> = String::new();
                core::write!(&mut s, "Random bytes: {:02x?}\r\n", buf).unwrap();
                unwrap!(usart.blocking_write(s.as_bytes()));
                Timer::after(Duration::from_millis(100)).await;
            }
        } else {
            led.set_low();
            info!("Button is not pressed");
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}
