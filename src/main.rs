#![no_std]
#![no_main]

mod fmt;
mod relay_set;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use core::sync::atomic::{AtomicU32, Ordering};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{
    DMA1_CH1, DMA1_CH2, DMA2_CH1, DMA2_CH2, LPUART1, PF6, PF7, PF8, USART3,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, InterruptHandler, Uart, UartRx, UartTx,
};
use embassy_stm32::{bind_interrupts, i2c, peripherals, Config};
use embassy_stm32::{
    exti::ExtiInput,
    gpio::OutputOpenDrain,
    rcc::{Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource},
};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_io::Write;

type LpUart1TxType = Mutex<CriticalSectionRawMutex, Option<UartTx<'static, LPUART1, DMA2_CH1>>>;
static SHARED_LPUART1_TX: LpUart1TxType = Mutex::new(None);
type LpUart1RxType = Mutex<CriticalSectionRawMutex, Option<UartRx<'static, LPUART1, DMA2_CH2>>>;
static SHARED_LPUART1_RX: LpUart1RxType = Mutex::new(None);
type Uart3TxType = Mutex<CriticalSectionRawMutex, Option<UartTx<'static, USART3, DMA1_CH1>>>;
static SHARED_UART3_TX: Uart3TxType = Mutex::new(None);
type Uart3RxType = Mutex<CriticalSectionRawMutex, Option<UartRx<'static, USART3, DMA1_CH2>>>;
static SHARED_UART3_RX: Uart3RxType = Mutex::new(None);
type RedLedType = Mutex<ThreadModeRawMutex, Option<Output<'static, PF8>>>;
static SHARED_RED_LED: RedLedType = Mutex::new(None);
type BlueLedType = Mutex<ThreadModeRawMutex, Option<Output<'static, PF7>>>;
static SHARED_BLUE_LED: BlueLedType = Mutex::new(None);
type GreenLedType = Mutex<ThreadModeRawMutex, Option<Output<'static, PF6>>>;
static SHARED_GREEN_LED: GreenLedType = Mutex::new(None);

// Global variable to store the delay value for blue LED blinking
static BLINK_MS: AtomicU32 = AtomicU32::new(0);

bind_interrupts!(struct Irqs {
    LPUART1 => InterruptHandler<peripherals::LPUART1>;
    USART2 => BufferedInterruptHandler<peripherals::USART2>;
    USART3 => InterruptHandler<peripherals::USART3>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::task]
async fn blue_led_task() {
    loop {
        let del = BLINK_MS.load(Ordering::Relaxed);
        Timer::after(Duration::from_millis(del.into())).await;
        blue_led_on().await;
        Timer::after(Duration::from_millis(del.into())).await;
        blue_led_off().await;
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

async fn blue_led_on() {
    let mut blue_led = SHARED_BLUE_LED.lock().await;
    blue_led.as_mut().unwrap().set_high();
}

async fn blue_led_off() {
    let mut blue_led = SHARED_BLUE_LED.lock().await;
    blue_led.as_mut().unwrap().set_low();
}

#[embassy_executor::task]
// This task reads from the modem connected in UART3 and writes to the console connected to LPUART1
// For visual feedback, it will blink the green LED when it reads from the modem
async fn modem_to_console_task() {
    {
        let mut lpuart1tx = SHARED_LPUART1_TX.lock().await;
        lpuart1tx
            .as_mut()
            .unwrap()
            .write("Starting modem to console task\r\n".as_bytes())
            .await
            .ok();
    }
    loop {
        green_led_off().await;
        {
            let mut usart3rx = SHARED_UART3_RX.lock().await;
            let usart3rx = usart3rx.as_mut().unwrap();
            match usart3rx.nb_read() {
                Ok(byte) => {
                    green_led_on().await;
                    let mut lpuart1 = SHARED_LPUART1_TX.lock().await;
                    match lpuart1.as_mut().unwrap().write(&[byte]).await {
                        Ok(_) => {}
                        Err(_) => {
                            info!("Error writing to LPUART1");
                        }
                    }
                }
                Err(_) => {}
            }
        }
        Timer::after(Duration::from_millis(1)).await;
        green_led_off().await;
    }
}

#[embassy_executor::task]
async fn consume_console_input_task() {
    {
        let mut lpuart1tx = SHARED_LPUART1_TX.lock().await;
        lpuart1tx
            .as_mut()
            .unwrap()
            .write("Starting console echo task\r\n".as_bytes())
            .await
            .ok();
    }

    loop {
        {
            let mut lpuart1rx = SHARED_LPUART1_RX.lock().await;
            match lpuart1rx.as_mut().unwrap().nb_read() {
                Ok(byte) => {
                    info!("Read byte {}", byte);
                    let mut lpuart1tx = SHARED_LPUART1_TX.lock().await;
                    let lpuart1tx = lpuart1tx.as_mut().unwrap();
                    match lpuart1tx.write(&[byte]).await {
                        Ok(_) => {}
                        Err(_) => {
                            info!("Error writing to LPUART1");
                        }
                    }
                }
                Err(_) => {}
            }
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

async fn send_modem_command(command: &str) {
    info!("Trying to send command {}", command);
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
        p.PC0, // RX
        p.PC1, // TX
        Irqs,
        p.DMA2_CH1, // DMA2_CH1 is the TX channel for LPUART1
        p.DMA2_CH2, // DMA2_CH2 is the RX channel for LPUART1
        UsartConfig::default(),
    )
    .unwrap();
    let (lpuart1tx, lpuart1rx) = lpuart1.split();
    {
        *(SHARED_LPUART1_RX.lock().await) = Some(lpuart1rx);
        *(SHARED_LPUART1_TX.lock().await) = Some(lpuart1tx);
    }
    // let mut usart2 = Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH3, p.DMA1_CH4, UsartConfig::default()).unwrap();

    let usart3 = Uart::new_with_rtscts(
        p.USART3,
        p.PD9, // RX
        p.PD8, // TX
        Irqs,
        p.PD12,     // RTS
        p.PD11,     // CTS
        p.DMA1_CH1, // DMA1_CH1 is the TX channel for USART3
        p.DMA1_CH2, // DMA1_CH2 is the RX channel for USART3
        UsartConfig::default(),
    )
    .unwrap();
    let (usart3tx, usart3rx) = usart3.split();
    {
        *(SHARED_UART3_RX.lock().await) = Some(usart3rx);
        *(SHARED_UART3_TX.lock().await) = Some(usart3tx);
    }

    let mut button = ExtiInput::new(Input::new(p.PC13, Pull::None), p.EXTI13);
    let red_led = Output::new(p.PF8, Level::High, Speed::Low);
    {
        *(SHARED_RED_LED.lock().await) = Some(red_led);
    }
    let blue_led = Output::new(p.PF7, Level::High, Speed::Low);
    {
        *(SHARED_BLUE_LED.lock().await) = Some(blue_led);
    }
    let green_led = Output::new(p.PF6, Level::High, Speed::Low);
    {
        *(SHARED_GREEN_LED.lock().await) = Some(green_led);
    }
    info!("Hello, World!");

    let mut relays = relay_set::RelaySet::new(I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH4,
        Hertz(100_000),
        I2cConfig::default(),
    ));

    red_led_off().await;
    green_led_off().await;
    blue_led_off().await;

    let mut sara_reset = OutputOpenDrain::new(p.PD10, Level::High, Speed::Low, Pull::Up);
    let mut sara_power = OutputOpenDrain::new(p.PD13, Level::High, Speed::Low, Pull::None);

    // Spawn LED blinking tasks
    spawner.spawn(blue_led_task()).unwrap();

    // Visual feedback that we are alive
    set_slow_blink();
    spawner.spawn(consume_console_input_task()).unwrap();
    spawner.spawn(modem_to_console_task()).unwrap();

    sara_power.set_low();
    Timer::after(Duration::from_millis(2200)).await;
    sara_power.set_high();
    Timer::after(Duration::from_millis(400)).await;
    sara_reset.set_low();
    Timer::after(Duration::from_millis(200)).await;
    sara_reset.set_high();
    info!("SARA is powered on and reset");
    Timer::after(Duration::from_millis(200)).await;

    execute_modem_commands().await;
    // Spawn reader tasks
    // spawner.spawn(console_to_modem_task()).unwrap();

    // Visual feedback that we are entering the loop
    set_fast_blink();

    let mut current_relay = 0;

    loop {
        button.wait_for_high().await;
        info!("Button pressed");
        red_led_on().await;
        relays.restart_device(current_relay).await;
        current_relay = (current_relay + 1) % 4;
        red_led_off().await;
        // Set the NMEA messages to RMC (recommended minimum data), at
        // a rate of 1 per message, sending to the GNSS UART (bitmask 4)
        send_modem_command("AT+UGNMEA=\"RMC\"\r\n").await;
        Timer::after(Duration::from_millis(2000)).await;
    }
}

async fn execute_modem_commands() {
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
    send_modem_command("AT+UGPS=1,1,3\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;
    // specify format
    send_modem_command("AT+UGUBX=1\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;
    // enable unsolicited result codes
    send_modem_command("AT+UGIND=1\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;
    // configure NMEA output
    send_modem_command(
        "AT+UGUBX=2,0x06,0x01,0x08,0xF0,0x00,0x01,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01\r\n",
    )
    .await;
    send_modem_command(
        "AT+UGUBX=2,0x06,0x01,0x08,0xF0,0x01,0x01,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01\r\n",
    )
    .await;
    Timer::after(Duration::from_millis(2000)).await;

    // Return power indicators
    send_modem_command("AT+CIND?\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Return the time based on SARA's clock
    send_modem_command("AT+CCLK?\r\n").await;
    Timer::after(Duration::from_millis(2000)).await;

    // Return the SIM card status
    send_modem_command("AT+USIMSTAT?\r\n").await;
}
