# TTN_RA02_Nano


arduino nano, RA02 Lorawan, DHT22 and The Things Network 

DHT pin D2 

use SPI D

![ScreenShot](https://github.com/worrajak/TTN_RA02_Nano/blob/master/nano-2.png?raw=true)


pin mapping arduino nano 

```
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {9, 8, 0},
};
```
