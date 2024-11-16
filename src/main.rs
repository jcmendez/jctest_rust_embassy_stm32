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
use embassy_stm32::{exti::ExtiInput, gpio::OutputOpenDrain, rcc::{Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource}};
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{usart, bind_interrupts, peripherals, rng, Config};
use embassy_stm32::gpio::{Pin, AnyPin, Input, Level, Output, Pull, Speed};
use embassy_time::{Duration, Timer};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::mutex::Mutex;
use heapless::String;
use fmt::info;

// Global variable to store the delay value for blue LED blinking
static BLINK_MS: AtomicU32 = AtomicU32::new(0);
static BUFFER_RNG: Mutex<ThreadModeRawMutex, [u8; 16]> = Mutex::new([0; 16]);
static BUFFER_UPDATED: Signal<ThreadModeRawMutex, ()> = Signal::new();

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::task]
async fn blue_led_task(led: AnyPin) {
    // Configure the LED pin as a push pull ouput and obtain handler.
    let mut led = Output::new(led, Level::Low, Speed::Low);

    loop {
        let del = BLINK_MS.load(Ordering::Relaxed);
        Timer::after(Duration::from_millis(del.into())).await;
        led.toggle();
    }
}

fn set_fast_blink() {
    BLINK_MS.store(100, Ordering::Relaxed);
}

fn set_slow_blink() {
    BLINK_MS.store(1000, Ordering::Relaxed);
}

#[embassy_executor::task]
async fn red_led_task(led: AnyPin) {
    // Configure the LED pin as a push pull ouput and obtain handler.
    let mut led = Output::new(led, Level::Low, Speed::Low);
    loop {
        BUFFER_UPDATED.wait().await;
        let buf = BUFFER_RNG.lock().await;
        for &duration in buf.iter() {
            led.set_high();
            Timer::after(Duration::from_millis(duration as u64)).await;
            led.set_low();
            Timer::after(Duration::from_millis(duration as u64)).await;
        }
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

    // let usart_config = UsartConfig::default();
    // let mut usart = UartTx::new(p.LPUART1, p.PC1, p.DMA1_CH1, usart_config).unwrap();
    // let mut usart2 = Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH3, p.DMA1_CH4, UsartConfig::default()).unwrap();
    let mut usart3 = Uart::new(p.USART3, p.PD9, p.PD8, Irqs, p.DMA1_CH2, p.DMA1_CH5, UsartConfig::default()).unwrap();
    let mut sara_reset = OutputOpenDrain::new(p.PD10, Level::High, Speed::Low, Pull::Up);
    let mut sara_power = OutputOpenDrain::new(p.PD13, Level::High, Speed::Low, Pull::None);

    let mut button = ExtiInput::new(Input::new(p.PC13, Pull::None), p.EXTI13);
    let mut green_led = Output::new(p.PF6, Level::High, Speed::Low);
    //let mut blue_led = Output::new(p.PF7, Level::High, Speed::Low);
    // let mut red_led = Output::new(p.PF8, Level::High, Speed::Low);    
    info!("Hello, World!");

    // Spawn LED blinking tasks
    spawner.spawn(blue_led_task(p.PF7.degrade())).unwrap();
    spawner.spawn(red_led_task(p.PF8.degrade())).unwrap();

    set_slow_blink();
    green_led.set_low();
    sara_power.set_low();
    Timer::after(Duration::from_millis(2200)).await;
    sara_power.set_high();
    Timer::after(Duration::from_millis(400)).await;
    sara_reset.set_low();
    Timer::after(Duration::from_millis(200)).await;
    sara_reset.set_high();
    info!("SARA is powered on and reset");
    Timer::after(Duration::from_millis(200)).await;

    usart3.write("AT&F0\r\n".as_bytes()).await.ok();
    info!("SARA is reset to factory settings");
    usart3.write("ATE1\r\n".as_bytes()).await.ok();
    info!("Echo is enabled");
    usart3.write("AT+GMM\r\n".as_bytes()).await.ok();
    info!("SARA firmware version is checked");
    // Enable GPS
    // usart3.write("AT+UGPS=1,0,1\r\n".as_bytes()).await.ok();
    // info!("GPS is enabled");
    // usart3.write("AT+CIND?\r\n".as_bytes()).await.ok();
    green_led.set_high();
    set_fast_blink();

    loop {
        button.wait_for_high().await;
    }
}
