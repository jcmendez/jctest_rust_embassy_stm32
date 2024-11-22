# Board Info

I don't have a proper board schematic.  This is what I know so far.

## SARA-R510M8S modem and gps
Small pad with labels 1-4.  All voltage converted to 1.8 and to the SARA modem
PA8 to the pad 1
PC7 to the pad 2
PC12 to the pad 3
PD0 to the pad 4
Likely GPIO2,3,4 SDA SCL of the SARA
PD10 to the RESET signal of the SARA
PD13 to the PWR signal of the SARA

## USART2 connector
PD5 to TX
PD6 to RX
PD3 to CTS
PD4 to RTS

## USART3 connector
PD8 to TX
PD9 to RX
PD11 to CTS
PD12 to RTS

## I2C1 connector
PB8 to SCL
PB9 to SDA

## Buttons
Button 1 us PC13
