use crate::lte_modem::Error::*;
use defmt::*;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::{AnyPin, Level, OutputOpenDrain, Pull, Speed};
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2, USART3};
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx, Error as UartError};
use embassy_time::{Duration, Timer};

type UartTxType = BufferedUartTx<'static, USART3>;
type UartRxType = BufferedUartRx<'static, USART3>;

pub(crate) struct LteModem {
    uart_rx: UartRxType,
    uart_tx: UartTxType,
    reset_pin: AnyPin,
    power_pin: AnyPin,
}
pub enum Error {
    TimeoutError,
    UartWriteError,
    UartOverrun,
    UartNoise,
    UartFraming,
    UartParity,
    AtCommandError,
    UartReadError,
}
impl LteModem {
    pub(crate) fn new(
        uart_rx: UartRxType,
        uart_tx: UartTxType,
        power_pin: AnyPin,
        reset_pin: AnyPin,
    ) -> LteModem {
        LteModem {
            uart_rx,
            uart_tx,
            power_pin,
            reset_pin,
        }
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
        // self.uart_tx
        //     .write(command.as_bytes())
        //     .await
        //     .map_err(|_| UartWriteError)?;
        // self.uart_tx
        //     .write(b"\r\n")
        //     .await
        //     .map_err(|_| UartWriteError)?;
        //
        // let mut buffer = [0u8; 1024];
        // let timeout = Timer::after(Duration::from_millis(timeout_millis as u64));
        // let read_future = self.uart_rx.read_until_idle(&mut buffer);
        //
        // match select(timeout, read_future).await {
        //     Either::First(_) => {
        //         info!("Timeout");
        //         Err(TimeoutError)
        //     }
        //     Either::Second(result) => match result {
        //         Ok(bytes_read) => {
        //             let response = &buffer[..bytes_read];
        //             let string_response = core::str::from_utf8(response).ok().unwrap();
        //             info!("Response: {}", string_response);
        //             Ok(())
        //         }
        //         Err(UartError::Overrun) => {
        //             info!("Overrun");
        //             Err(UartOverrun)
        //         }
        //         Err(UartError::Framing) => {
        //             info!("Framing");
        //             Err(UartFraming)
        //         }
        //         Err(UartError::Noise) => {
        //             info!("Noise");
        //             Err(UartNoise)
        //         }
        //         Err(UartError::Parity) => {
        //             info!("Parity");
        //             Err(UartParity)
        //         }
        //         Err(_) => {
        //             info!("UartReadError");
        //             Err(UartReadError)
        //         }
        //     },
        Ok(())
    }

    pub(crate) async fn get_gnss_clock(&mut self) -> Result<(), Error> {
        self.send_command("AT+UGPSCLCK?", 2000).await.ok();
        Ok(())
    }

    pub(crate) async fn test_commands(&mut self) {
        info!("Executing startup modem commands");
        trace!("Resetting to known state");
        self.send_command("ATZ", 50).await.ok();

        // Print out the modem model
        trace!("Printing modem model");
        self.send_command("AT+GMM", 50).await.ok();

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
        // // configure NMEA output
        // lte_modem
        //         .send_command(
        //             "AT+UGUBX=2,0x06,0x01,0x08,0xF0,0x00,0x01,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01",
        //             2000,
        //         )
        //         .await
        //         .ok();
        // lte_modem
        //         .send_command(
        //             "AT+UGUBX=2,0x06,0x01,0x08,0xF0,0x01,0x01,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01",
        //             2000,
        //         )
        //         .await
        //         .ok();
        // // Return power indicators
        // send_command("AT+CIND?", 2000).await.ok();
        // Return the time based on SARA's clock
        self.send_command("AT+CCLK?", 2000).await.ok();
        // List any files
        self.send_command("AT+ULSTFILE", 2000).await.ok();
        // // Return the SIM card status
        // send_command("AT+USIMSTAT?", 2000).await.ok();
    }
}
