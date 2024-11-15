#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_stm32::{exti::ExtiInput, rcc::{Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource}, usart::UartTx};
use embassy_stm32::rng::Rng;
use embassy_stm32::usart::Config as UsartConfig;
use embassy_stm32::{bind_interrupts, peripherals, rng, Config};
use embassy_stm32::gpio::{Pin, AnyPin, Input, Level, Output, Pull, Speed};
use embassy_time::{Duration, Timer};
use heapless::String;
use fmt::info;

// Global variable to store the delay value for blue LED blinking
static BLINK_MS: AtomicU32 = AtomicU32::new(0);

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::task]
async fn led_task(led: AnyPin) {
    // Configure the LED pin as a push pull ouput and obtain handler.
    let mut led = Output::new(led, Level::Low, Speed::Low);

    loop {
        let del = BLINK_MS.load(Ordering::Relaxed);
        Timer::after(Duration::from_millis(del.into())).await;
        led.toggle();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
    let mut usart = UartTx::new(p.LPUART1, p.PC1, p.DMA1_CH1, usart_config).unwrap();
    let mut usart2 = UartTx::new(p.USART2, p.PA2, p.DMA1_CH2, usart_config).unwrap();

    info!("Hello, World!");
    let mut button = ExtiInput::new(Input::new(p.PC13, Pull::None), p.EXTI13);
    
    let mut green_led = Output::new(p.PF6, Level::High, Speed::Low);
    //let mut blue_led = Output::new(p.PF7, Level::High, Speed::Low);
    let mut red_led = Output::new(p.PF8, Level::High, Speed::Low);
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut buf = [0u8; 16];

    // Create and initialize a delay variable to manage delay loop
    let mut del_var = 1000;

    // Publish blink duration value to global context
    BLINK_MS.store(del_var, Ordering::Relaxed);
 
     // Spawn LED blinking task
    spawner.spawn(led_task(p.PF7.degrade())).unwrap();

    green_led.set_low();
    //blue_led.set_low();
    red_led.set_low();

    loop {
        button.wait_for_rising_edge().await;
        info!("Button is pressed");

        green_led.set_high();
        unwrap!(rng.async_fill_bytes(&mut buf).await);
        info!("random bytes: {:02x}", buf); 
        let mut s: String<128> = String::new();
        core::write!(&mut s, "Random bytes: {:02x?}\r\n", buf).unwrap();
        unwrap!(usart.blocking_write(s.as_bytes()));
        Timer::after(Duration::from_millis(100)).await;
        green_led.set_low();
            
        // If button pressed decrease the delay value
        del_var = del_var - 180;
        info!("Delay value: {}", del_var);
        // If updated delay value drops below 300 then reset it back to starting value
        if del_var <= 100 {
            info!("Resetting delay value to 1000");
            del_var = 1000;
        }
        // Publish updated delay value to global context
        BLINK_MS.store(del_var, Ordering::Relaxed);
        unwrap!(usart2.blocking_write("ATZ\n".as_bytes()));
    }
}
