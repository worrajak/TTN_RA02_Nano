# TTN_RA02_Nano


arduino nano, RA02 Lorawan, DHT22 and The Things Network 

DHT pin D2 

use SPI D

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
