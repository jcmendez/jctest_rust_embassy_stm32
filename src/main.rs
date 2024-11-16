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
    let usart_config = UsartConfig::default();
    let mut usart = UartTx::new(p.LPUART1, p.PC1, p.DMA1_CH1, usart_config).unwrap();
    let mut usart2 = Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH3, p.DMA1_CH4, UsartConfig::default()).unwrap();

    info!("Hello, World!");
    let mut button = ExtiInput::new(Input::new(p.PC13, Pull::None), p.EXTI13);
    
    let mut green_led = Output::new(p.PF6, Level::High, Speed::Low);
    //let mut blue_led = Output::new(p.PF7, Level::High, Speed::Low);
    // let mut red_led = Output::new(p.PF8, Level::High, Speed::Low);
    let mut rng = Rng::new(p.RNG, Irqs);

    // Create and initialize a delay variable to manage delay loop
    let mut del_var = 1000;

    // Publish blink duration value to global context
    BLINK_MS.store(del_var, Ordering::Relaxed);
 
     // Spawn LED blinking tasks
    spawner.spawn(blue_led_task(p.PF7.degrade())).unwrap();
    spawner.spawn(red_led_task(p.PF8.degrade())).unwrap();

    green_led.set_low();
    loop {
        button.wait_for_rising_edge().await;
        info!("Button is pressed");

        // When button is pressed, turn on green LED and generate random bytes
        green_led.set_high();
        button.wait_for_falling_edge().await;
        let mut local_buf = [0u8; 16];
        unwrap!(rng.async_fill_bytes(&mut local_buf).await);
        info!("random bytes: {:02x}", local_buf);

        // Copy the random bytes to global buffer and signal the event
        let mut buf = BUFFER_RNG.lock().await;
        buf.copy_from_slice(&local_buf);
        BUFFER_UPDATED.signal(());

        // Print the random bytes to USART, finally turn off the green LED
        let mut s: String<128> = String::new();
        core::write!(&mut s, "Random bytes: {:02x?}\r\n", buf).unwrap();
        unwrap!(usart.blocking_write(s.as_bytes()));
        green_led.set_low();
        
        // Calculate the average of the random values

        let mut sum: u32 = 0;
        for &val in buf.iter() {
            sum += val as u32;
        }
        let avg = sum / 16;

        // If button pressed decrease the delay value
        del_var = avg * 10;
        info!("Delay value: {}", del_var);
        // If updated delay value is below 100 then reset it back to starting value
        if del_var <= 100 {
            info!("Resetting delay value to 1000");
            del_var = 1000;
        }
        // Publish updated delay value to global context
        BLINK_MS.store(del_var, Ordering::Relaxed);
        
        usart2.write("ATZ\n".as_bytes()).await.ok();
    }
}
