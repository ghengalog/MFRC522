Arduino RFID Library for MFRC522 (13.56 Mhz)
============================================

Pin order, starting from the bottom left hand pin (in case your
MFRC522 doesn't have pin markings like the B2CQSHOP one):

| Pins | SPI      | UNO  | Mega2560 |
| ---- |:--------:| ----:| --------:|
| 1    | SAD (SS) |  10  |  53      |
| 2    | SCK      |  13  |  52      |
| 3    | MOSI     |  11  |  51      |
| 4    | MISO     |  12  |  50      |
| 5    | IRQ      |  `*` |  `*`     |
| 6    | GND      |  GND |  GND     |
| 7    | RST      |  5   |  ?       |
| 8    | +3.3V    |  3V3 |  3V3     |
`* Not needed`

Using MFRC522 with other SPI components
========================================

If you are planning to use other SPI components you just have to make
sure each have an exclusive SS (Slave Select) line.  MISO, MOSI and
SCK lines may be shared. More reference regarding SPI may be found
[here](http://arduino.cc/en/Reference/SPI).
