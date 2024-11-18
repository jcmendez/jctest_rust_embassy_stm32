#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::prelude::{_embedded_hal_blocking_serial_Write, _embedded_hal_serial_Read};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::peripherals::{
    DMA1_CH1, DMA1_CH2, DMA1_CH5, DMA1_CH6, LPUART1, PF6, PF8, USART3,
};
use embassy_stm32::usart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config as UsartConfig,
    Uart, UartRx, UartTx,
};
use embassy_stm32::{bind_interrupts, peripherals, rng, usart, Config};
use embassy_stm32::{
    exti::ExtiInput,
    gpio::OutputOpenDrain,
    rcc::{Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource},
};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_io::Write;

type LpUart1TxType = Mutex<CriticalSectionRawMutex, Option<UartTx<'static, LPUART1, DMA1_CH1>>>;
static SHARED_LPUART1_TX: LpUart1TxType = Mutex::new(None);
type LpUart1RxType = Mutex<CriticalSectionRawMutex, Option<UartRx<'static, LPUART1, DMA1_CH2>>>;
static SHARED_LPUART1_RX: LpUart1RxType = Mutex::new(None);

type Uart3TxType = Mutex<CriticalSectionRawMutex, Option<BufferedUartTx<'static, USART3>>>;
static SHARED_UART3_TX: Uart3TxType = Mutex::new(None);
type Uart3RxType = Mutex<CriticalSectionRawMutex, Option<BufferedUartRx<'static, USART3>>>;
static SHARED_UART3_RX: Uart3RxType = Mutex::new(None);
type RedLedType = Mutex<ThreadModeRawMutex, Option<Output<'static, PF8>>>;
static SHARED_RED_LED: RedLedType = Mutex::new(None);

type GreenLedType = Mutex<ThreadModeRawMutex, Option<Output<'static, PF6>>>;
static SHARED_GREEN_LED: GreenLedType = Mutex::new(None);

// Global variable to store the delay value for blue LED blinking
static BLINK_MS: AtomicU32 = AtomicU32::new(0);

// It is ok to use these buffers here by themselves and not in a Mutex because they are
// only used in conjunction with UART3, which has the
static mut TX_BUFFER: [u8; 64] = [0; 64];
static mut RX_BUFFER: [u8; 64] = [0; 64];

bind_interrupts!(struct Irqs {
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART3 => BufferedInterruptHandler<peripherals::USART3>;
});

#[embassy_executor::task]
async fn blue_led_task(led: AnyPin) {
    // Configure the LED pin as a push pull output and obtain handler.
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

async fn red_led_on() {
    let mut red_led = SHARED_RED_LED.lock().await;
    red_led.as_mut().unwrap().set_high();
}

async fn red_led_off() {
    let mut red_led = SHARED_RED_LED.lock().await;
    red_led.as_mut().unwrap().set_low();
}

async fn green_led_on() {
    let mut green_led = SHARED_GREEN_LED.lock().await;
    green_led.as_mut().unwrap().set_high();
}

async fn green_led_off() {
    let mut green_led = SHARED_GREEN_LED.lock().await;
    green_led.as_mut().unwrap().set_low();
}

// #[embassy_executor::task]
// async fn lpuart1_reader_task() {
//     let mut buffer = [0; 1];
//     loop {
//         trace!("Reading from LPUART1");
//         let mut lpuart1 = SHARED_LPUART1.lock().await;
//         lpuart1.as_mut().unwrap().read(&mut buffer).await.ok();
//         red_led_on().await;
//         trace!("Writing to USART3");
//         let mut usart3 = SHARED_USART3.lock().await;
//         usart3.as_mut().unwrap().
//         red_led_off().await;
//     }
// }

#[embassy_executor::task]
async fn modem_to_console_task() {
    let mut usart3rx = SHARED_UART3_RX.lock().await;
    let usart3rx = usart3rx.as_mut().unwrap();
    {
        let mut lpuart1 = SHARED_LPUART1_TX.lock().await;
        let lpuart1 = lpuart1.as_mut().unwrap();
        lpuart1
            .write("Starting modem to console task\r\n".as_bytes())
            .await
            .ok();
    }
    loop {
        green_led_off().await;
        trace!("Reading from USART3");
        match usart3rx.read() {
            Ok(c) => {
                green_led_on().await;
                trace!("Read char: {}", c as char);
                let mut lpuart1 = SHARED_LPUART1_TX.lock().await;
                let buf = [c];
                lpuart1.as_mut().unwrap().write(&buf).await.ok();
            }
            Err(_) => {
                info!("Error reading from USART3");
            }
        }
        green_led_off().await;
    }
}

#[embassy_executor::task]
async fn console_to_modem_task() {
    let mut lpuart1rx = SHARED_LPUART1_RX.lock().await;
    let lpuart1rx = lpuart1rx.as_mut().unwrap();
    loop {
        trace!("Reading from LPUART1");
        let mut buf: [u8; 1] = [0; 1];
        match lpuart1rx.read(&mut buf).await {
            Ok(_) => {
                trace!("Read char: {}", buf[0] as char);
                let mut usart3tx = SHARED_UART3_TX.lock().await;
                usart3tx.as_mut().unwrap().bwrite_all(&buf).ok();
            }
            Err(_) => {
                info!("Error reading from LPUART1");
            }
        }
    }
}

async fn send_modem_command(command: &str) {
    trace!("Trying to send command {}", command);
    let mut usart3tx = SHARED_UART3_TX.lock().await;
    let usart3tx = usart3tx.as_mut().unwrap();
    match usart3tx.write_all(command.as_bytes()) {
        Ok(_) => {
            trace!("Command sent");
        }
        Err(_) => {
            info!("Error sending command");
        }
    }
    usart3tx.bflush().unwrap();
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

    let lpuart1 = Uart::new(
        p.LPUART1,
        p.PC0,
        p.PC1,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        UsartConfig::default(),
    )
    .unwrap();
    let (lpuart1tx, lpuart1rx) = lpuart1.split();
    {
        *(SHARED_LPUART1_RX.lock().await) = Some(lpuart1rx);
        *(SHARED_LPUART1_TX.lock().await) = Some(lpuart1tx);
    }
    // let mut usart2 = Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH3, p.DMA1_CH4, UsartConfig::default()).unwrap();

    unsafe {
        let usart3 = BufferedUart::new_with_rtscts(
            p.USART3,
            Irqs,
            p.PD9,
            p.PD8,
            p.PD12,
            p.PD11,
            &mut TX_BUFFER,
            &mut RX_BUFFER,
            UsartConfig::default(),
        )
        .unwrap();
        let (usart3tx, usart3rx) = usart3.split();
        {
            *(SHARED_UART3_RX.lock().await) = Some(usart3rx);
            *(SHARED_UART3_TX.lock().await) = Some(usart3tx);
        }
    };

    let mut button = ExtiInput::new(Input::new(p.PC13, Pull::None), p.EXTI13);
    let green_led = Output::new(p.PF6, Level::High, Speed::Low);
    {
        *(SHARED_GREEN_LED.lock().await) = Some(green_led);
    }
    //let mut blue_led = Output::new(p.PF7, Level::High, Speed::Low);
    let red_led = Output::new(p.PF8, Level::High, Speed::Low);
    {
        *(SHARED_RED_LED.lock().await) = Some(red_led);
    }
    info!("Hello, World!");

    let mut sara_reset = OutputOpenDrain::new(p.PD10, Level::High, Speed::Low, Pull::Up);
    let mut sara_power = OutputOpenDrain::new(p.PD13, Level::High, Speed::Low, Pull::None);

    // Spawn LED blinking tasks
    spawner.spawn(blue_led_task(p.PF7.degrade())).unwrap();

    set_slow_blink();
    sara_power.set_low();
    Timer::after(Duration::from_millis(2200)).await;
    sara_power.set_high();
    Timer::after(Duration::from_millis(400)).await;
    sara_reset.set_low();
    Timer::after(Duration::from_millis(200)).await;
    sara_reset.set_high();
    info!("SARA is powered on and reset");
    Timer::after(Duration::from_millis(200)).await;

    // spawner.spawn(console_to_modem_task()).unwrap();
    // spawner.spawn(modem_to_console_task()).unwrap();

    // Reset to factory settings
    send_modem_command("AT&F0\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Echo commands
    send_modem_command("ATE1\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Print out the modem model
    send_modem_command("AT+GMM\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Turn off the module power savings
    send_modem_command("AT+UPSV=0\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // List operator names
    send_modem_command("AT+COPN\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;


    // Set the SARA UARTS in configuration 4, which is AT instance 1 is
    // UART (5 wire), AT instance 2 not available, diagnostic log is
    // USB-NCM, SPI, SDIO, and GNSS tunneling to AUX UART (5 wire)
    send_modem_command("AT+USIO=4\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Turn on the GNSS receiver, with automatic local aiding (1),
    // and using GPS+SBAS+GLONASS (1+2+64)
    send_modem_command("AT+UGPS=1,1,67\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Return power indicators
    send_modem_command("AT+CIND?\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Return the time based on SARA's clock
    send_modem_command("AT+CCLK?\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Return the SIM card status
    send_modem_command("AT+USIMSTAT?\r\n").await;
    set_fast_blink();
    // Spawn reader tasks

    loop {
        button.wait_for_high().await;
        info!("Button pressed");
        // Set the NMEA messages to RMC (recommended minimum data), at
        // a rate of 1 per message, sending to the GNSS UART (bitmask 4)
        send_modem_command("AT+UGNMEA=\"RMC\"\r\n").await;
        Timer::after(Duration::from_millis(2000)).await;
    }
}
