# SPI Sniffer

Sniffs SPI transactions on a bus connected to the MCU:

  // /CS  -> PB10 and PB13 (to properly detect both edges)
  // SCK  -> PB11
  // MOSI -> PB12

Sends data to USART2 on pin A2 at 115200 8N1

## Receiving using a Serial to USB dongle

```
sudo chmod 777 /dev/ttyUSB1
stty -F /dev/ttyUSB1 115200
cat /dev/ttyUSB1
```
