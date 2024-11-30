#![no_std]
#![no_main]

mod fmt;
mod leds;
mod lte_modem;
mod relay_set;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use crate::leds::Led;
use atat::heapless::String;
use atat::{
    asynch::{AtatClient, Client},
    AtatIngress, DefaultDigester, Ingress, ResponseSlot, UrcChannel,
};
use atat_derive::{AtatResp, AtatUrc};
use defmt::panic;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Pin, Pull};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{DMA2_CH1, DMA2_CH2, LPUART1, PF6, PF7, PF8};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config as UsartConfig,
    InterruptHandler, Uart, UartRx, UartTx,
};
use embassy_stm32::{bind_interrupts, i2c, peripherals, Config};
use embassy_stm32::{
    exti::ExtiInput,
    rcc::{Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource},
};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

type LpUart1TxType = Mutex<CriticalSectionRawMutex, Option<UartTx<'static, LPUART1, DMA2_CH1>>>;
static SHARED_LPUART1_TX: LpUart1TxType = Mutex::new(None);
type LpUart1RxType = Mutex<CriticalSectionRawMutex, Option<UartRx<'static, LPUART1, DMA2_CH2>>>;
static SHARED_LPUART1_RX: LpUart1RxType = Mutex::new(None);

// static SHARED_LTE_MODEM: Mutex<ThreadModeRawMutex, Option<lte_modem::LteModem>> = Mutex::new(None);
static SHARED_GREEN_LED: Mutex<ThreadModeRawMutex, Option<Led<PF6>>> = Mutex::new(None);
static SHARED_BLUE_LED: Mutex<ThreadModeRawMutex, Option<Led<PF7>>> = Mutex::new(None);
static SHARED_RED_LED: Mutex<ThreadModeRawMutex, Option<Led<PF8>>> = Mutex::new(None);

const INGRESS_BUF_SIZE: usize = 1024;
const URC_CAPACITY: usize = 128;
const URC_SUBSCRIBERS: usize = 3;

bind_interrupts!(struct Irqs {
    LPUART1 => InterruptHandler<peripherals::LPUART1>;
    USART2 => BufferedInterruptHandler<peripherals::USART2>;
    USART3 => BufferedInterruptHandler<peripherals::USART3>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[derive(Clone, AtatResp)]
pub struct MessageWaitingIndication;
#[derive(Clone, AtatUrc)]
pub enum Urc {
    #[at_urc("+UMWI")]
    MessageWaitingIndication(MessageWaitingIndication),
}

#[derive(Clone, Debug, AtatResp)]
pub struct ManufacturerId {
    pub id: String<64>,
}

#[derive(Clone, atat_derive::AtatCmd)]
#[at_cmd("+CGMI", ManufacturerId)]
pub struct GetManufacturerId;

#[embassy_executor::task]
async fn green_led_task() {
    let mut green_led = SHARED_GREEN_LED.lock().await;
    let green_led = green_led.as_mut().unwrap();
    green_led.toggle_task().await;
}

async fn set_fast_blink() {
    let mut green_led = SHARED_GREEN_LED.lock().await;
    let green_led = green_led.as_mut().unwrap();
    green_led.set_blink_speed(leds::BlinkSpeed::Fast);
}

async fn set_slow_blink() {
    let mut green_led = SHARED_GREEN_LED.lock().await;
    let green_led = green_led.as_mut().unwrap();
    green_led.set_blink_speed(leds::BlinkSpeed::Slow);
}

async fn red_led_on() {
    let mut red_led = SHARED_RED_LED.lock().await;
    red_led.as_mut().unwrap().on().await;
}

async fn red_led_off() {
    let mut red_led = SHARED_RED_LED.lock().await;
    red_led.as_mut().unwrap().off().await;
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

    //---------------------------------------------------------------------------------------------
    // Get the LPUART1 an split it in lpuart1tx and lpuart1rx
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

    //---------------------------------------------------------------------------------------------
    // Get the LPUART3 an split it in lpuart3tx and lpuart3rx

    static INGRESS_BUF: StaticCell<[u8; INGRESS_BUF_SIZE]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();

    let usart3 = BufferedUart::new_with_rtscts(
        p.USART3,
        Irqs,
        p.PD9,  // RX
        p.PD8,  // TX
        p.PD12, // RTS
        p.PD11, // CTS
        TX_BUF.init([0; 16]),
        RX_BUF.init([0; 16]),
        UsartConfig::default(),
    )
    .ok()
    .unwrap();
    let (usart3tx, usart3rx) = usart3.split();

    // let mut lte_modem =
    //     lte_modem::LteModem::new(usart3rx, usart3tx, p.PD13.degrade(), p.PD10.degrade());

    let mut button = ExtiInput::new(Input::new(p.PC13, Pull::None), p.EXTI13);
    {
        *(SHARED_GREEN_LED.lock().await) = Some(Led::new(p.PF6));
        *(SHARED_BLUE_LED.lock().await) = Some(Led::new(p.PF7));
        *(SHARED_RED_LED.lock().await) = Some(Led::new(p.PF8));
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

    // Spawn LED blinking tasks
    spawner.spawn(green_led_task()).unwrap();
    // spawner.spawn(consume_console_input_task()).unwrap();
    // spawner.spawn(modem_to_console_task()).unwrap();

    // Visual feedback that we are alive
    set_slow_blink().await;

    static RES_SLOT: ResponseSlot<INGRESS_BUF_SIZE> = ResponseSlot::new();
    static URC_CHANNEL: UrcChannel<Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();
    let ingress = Ingress::new(
        DefaultDigester::<Urc>::default(),
        INGRESS_BUF.init([0; INGRESS_BUF_SIZE]),
        &RES_SLOT,
        &URC_CHANNEL,
    );
    static BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let mut client = Client::new(
        usart3tx,
        &RES_SLOT,
        BUF.init([0; 1024]),
        atat::Config::default(),
    );

    spawner.spawn(ingress_task(ingress, usart3rx)).unwrap();
    // lte_modem.hardware_reset().await;
    // lte_modem.test_commands().await;
    // Spawn reader tasks
    // spawner.spawn(console_to_modem_task()).unwrap();

    // Visual feedback that we are entering the loop
    set_fast_blink().await;

    let mut current_relay = 0;

    loop {
        button.wait_for_high().await;
        info!("Button pressed");
        red_led_on().await;
        relays.restart_device(current_relay).await;
        current_relay = (current_relay + 1) % 4;
        // lte_modem.send_command("AT+UGZDA?", 10000).await.ok();
        // lte_modem.send_command("AT+UGGLL?", 10000).await.ok();

        let modem_id = client.send(&GetManufacturerId).await.ok();
        match modem_id {
            None => {
                info!("No modem ID");
            }
            Some(modem_id) => {
                info!("Modem ID: {:?}", modem_id.id.as_str());
            }
        }
        red_led_off().await;
    }
}

#[embassy_executor::task]
async fn ingress_task(
    mut ingress: Ingress<
        'static,
        DefaultDigester<Urc>,
        Urc,
        INGRESS_BUF_SIZE,
        URC_CAPACITY,
        URC_SUBSCRIBERS,
    >,
    mut reader: BufferedUartRx<'static, embassy_stm32::peripherals::USART3>,
) -> ! {
    ingress.read_from(&mut reader).await
}
