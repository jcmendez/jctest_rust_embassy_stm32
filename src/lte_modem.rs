use atat::asynch::{AtatClient, Client};
use atat::{AtDigester, AtatIngress, DefaultDigester, Ingress, ResponseSlot, UrcChannel};
use atat_derive::{AtatResp, AtatUrc};
use core::fmt::{Debug, Error};
use defmt::panic;
use defmt::*;
use embassy_stm32::gpio::{Level, OutputOpenDrain, Pull, Speed};
use embassy_stm32::peripherals::USART3;
use embassy_stm32::usart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx,
};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Duration, Timer};
use heapless::String;
use static_cell::StaticCell;

const INGRESS_BUF_SIZE: usize = 1024;
const URC_CAPACITY: usize = 128;
const URC_SUBSCRIBERS: usize = 3;

#[derive(Clone, Debug, AtatResp)]
pub struct ResetResponse;
#[derive(Clone, atat_derive::AtatCmd)]
#[at_cmd("Z", ResetResponse)]
pub struct Reset;

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

type ModemIngress =
    Ingress<'static, AtDigester<Urc>, Urc, INGRESS_BUF_SIZE, URC_CAPACITY, URC_SUBSCRIBERS>;

type ModemClient = Client<'static, BufferedUartTx<'static, peripherals::USART3>, INGRESS_BUF_SIZE>;
pub(crate) struct LteModem<'a> {
    reset_pin: peripherals::PD10,
    power_pin: peripherals::PD13,
    usart_rx: BufferedUartRx<'a, USART3>,
    ingress: ModemIngress,
    client: ModemClient,
}

bind_interrupts!(struct Irqs {
    USART3 => BufferedInterruptHandler<peripherals::USART3>;
});

impl LteModem<'_> {
    pub(crate) fn new(
        uart: peripherals::USART3,
        rx_pin: peripherals::PD9,     // RX
        tx_pin: peripherals::PD8,     // TX
        rts_pin: peripherals::PD12,   // RTS
        cts_pin: peripherals::PD11,   // CTS
        reset_pin: peripherals::PD10, // reset gpio
        power_pin: peripherals::PD13, // power gpio
    ) -> LteModem<'static> {
        static RES_SLOT: ResponseSlot<INGRESS_BUF_SIZE> = ResponseSlot::new();
        static URC_CHANNEL: UrcChannel<Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();
        static INGRESS_BUF: StaticCell<[u8; INGRESS_BUF_SIZE]> = StaticCell::new();

        static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
        static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();

        let usart3 = BufferedUart::new_with_rtscts(
            uart,
            Irqs,
            rx_pin,  // RX
            tx_pin,  // TX
            rts_pin, // RTS
            cts_pin, // CTS
            TX_BUF.init([0; 16]),
            RX_BUF.init([0; 16]),
            embassy_stm32::usart::Config::default(),
        )
        .ok()
        .unwrap();
        let (usart_tx, usart_rx) = usart3.split();
        let ingress: ModemIngress = Ingress::new(
            DefaultDigester::<Urc>::default(),
            INGRESS_BUF.init([0; INGRESS_BUF_SIZE]),
            &RES_SLOT,
            &URC_CHANNEL,
        );
        static BUF: StaticCell<[u8; INGRESS_BUF_SIZE]> = StaticCell::new();
        let client = Client::new(
            usart_tx,
            &RES_SLOT,
            BUF.init([0; INGRESS_BUF_SIZE]),
            atat::Config::default(),
        );

        LteModem {
            reset_pin,
            power_pin,
            usart_rx,
            ingress,
            client,
        }
    }

    pub(crate) async fn do_ingress(&mut self) {
        self.ingress.read_from(&mut self.usart_rx).await;
    }

    pub(crate) async fn cmd_reset(&mut self) {
        self.client.send(&Reset).await.ok();
    }
    pub(crate) async fn cmd_get_manufacturer(&mut self) -> Result<ManufacturerId, atat::Error> {
        self.client.send(&GetManufacturerId).await
    }

    pub(crate) async fn hardware_reset(&mut self) {
        info!("Resetting SARA");
        let mut sara_reset =
            OutputOpenDrain::new(&mut self.reset_pin, Level::High, Speed::Low, Pull::Up);
        let mut sara_power =
            OutputOpenDrain::new(&mut self.power_pin, Level::High, Speed::Low, Pull::None);

        sara_power.set_low();
        Timer::after(Duration::from_millis(2200)).await;
        sara_power.set_high();
        Timer::after(Duration::from_millis(400)).await;
        sara_reset.set_low();
        Timer::after(Duration::from_millis(200)).await;
        sara_reset.set_high();
        Timer::after(Duration::from_millis(200)).await;
        info!("SARA is powered on and reset");
    }

    pub(crate) async fn send_command(
        &mut self,
        command: &str,
        timeout_millis: u32,
    ) -> Result<(), Error> {
        info!("Sending command: {:?}", command);
        Ok(())
    }

    pub(crate) async fn get_gnss_clock(&mut self) -> Result<(), Error> {
        self.send_command("AT+UGPSCLCK?", 2000).await.ok();
        Ok(())
    }

    pub(crate) async fn test_commands(&mut self) {
        info!("Executing startup modem commands");
        trace!("Resetting to known state");
        self.cmd_reset().await;

        // Print out the modem model
        trace!("Printing modem model");
        match self.cmd_get_manufacturer().await {
            Ok(manufacturer) => {
                let manufacturer = manufacturer.id.as_str();
                info!("Manufacturer: {:?}", manufacturer);
            }
            Err(e) => {
                info!("Error getting manufacturer id");
            }
        }

        // Module deregistered from the network but RF circuits are not disabled, hence the
        // radio synchronization is retained
        trace!("Turn on radio, but don't try registering with network");
        self.send_command("AT+COPS=2", 180_000).await.ok();

        trace!("Set MT to full functionality");
        self.send_command("AT+CFUN=1", 180_000).await.ok();
        // Turn off the module power savings
        trace!("Turn off power savings");
        self.send_command("AT+UPSV=0", 2000).await.ok();

        trace!("Turning on the SARA led");
        // Set the SARA GPIO 16 to be an output showing the module status
        self.send_command("AT+UGPIOC=16,10", 10_000).await.ok();
        //self.send_command("AT+UGPIOC=23,10", 10_000).await.ok();
        //self.send_command("AT+UGPIOC=24,10", 10_000).await.ok();
        //self.send_command("AT+UGPIOC=25,10", 10_000).await.ok();
        //self.send_command("AT+UGPIOC=33,10", 10_000).await.ok();
        self.send_command("AT+UGPIOC=42,10", 10_000).await.ok();
        self.send_command("AT+UGPIOC=19,10", 10_000).await.ok();
        self.send_command("AT+UGPIOC=46,10", 10_000).await.ok();

        // Set the SARA GPIO 23 to be the time pulse output
        // self.send_command("AT+UGPIOC=23,22", 10_000).await.ok();

        // // List operator names
        // send_command("AT+COPN", 2000).await.ok();
        // Set the SARA UARTS in configuration 4, which is AT instance 1 is
        // UART (5 wire), AT instance 2 not available, diagnostic log is
        // USB-NCM, SPI, SDIO, and GNSS tunneling to AUX UART (5 wire)
        self.send_command("AT+USIO=4", 2000).await.ok();

        // Set the GNSS data go to USB/AUX UART
        self.send_command("AT+UGPRF=1", 2000).await.ok();

        // Check the current GNSS profile configuration
        // self.send_command("AT+UGPRF?", 2000).await.ok();
        // Turn on the GNSS receiver, with automatic local aiding (1),
        // and using GPS+SBAS+GLONASS (1+2+64)
        // For now, just GPS + SBAS which is the default
        self.send_command("AT+UGPS=1,1,3", 2000).await.ok();
        // enable unsolicited result codes
        self.send_command("AT+UGIND=1", 2000).await.ok();
        // enable GNSS time and date output
        self.send_command("AT+UGZDA=1", 2000).await.ok();
        // enable GNSS position output
        self.send_command("AT+UGGLL=1", 2000).await.ok();
        self.send_command("AT+CCLK?", 2000).await.ok();
        // List any files
        self.send_command("AT+ULSTFILE", 2000).await.ok();
        // // Return the SIM card status
        // send_command("AT+USIMSTAT?", 2000).await.ok();
    }
}
