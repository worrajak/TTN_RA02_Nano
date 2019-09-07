#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <Wire.h> 
#include <LowPower.h>

#include <DHT.h>

#define DHTTYPE DHT22
#define DHTPIN  2
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

byte buf[10];
byte data_temp[4];
byte data_humid[4];

bool TX_done = false;
bool next = false;

#define MY_DEBUG
static const PROGMEM u1_t NWKSKEY[16] = { 0x6A, 0x3C, 0xA6, 0x47, 0xE0, 0x73, 0x79, 0x80, 0x96, 0x5C, 0x7C, 0x08, 0x32, 0x06, 0x47, 0xAD };
static const u1_t PROGMEM APPSKEY[16] = { 0x32, 0x02, 0x4A, 0x50, 0x04, 0x14, 0x2A, 0x81, 0x16, 0x22, 0x16, 0xA4, 0xAE, 0x87, 0xB0, 0xB3 };
static const u4_t DEVADDR = 0x260410AA ; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

  static osjob_t sendjob;

  const unsigned TX_INTERVAL = 30;
  int TX_COMPLETE = 0;

  float temperature = 0;
  float humidity = 0;
  uint16_t tempC_int = 0;
  uint8_t hum_int = 0;
  int vbat_int;
  int adc_int=0;

  int channel = 0;
  int dr = DR_SF10;  

struct {
  uint8_t Header1[2] = {0x01,0x67};
  char temp[2];
  uint8_t Header2[2] = {0x02,0x68};
  char humid[1];
  uint8_t Header3[2] = {0x03,0x02};
  char adc[2];
  uint8_t Header4[2] = {0x04,0x02};
  char vbat[2];  
}mydata;
  
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {9, 8, 0},
};
void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
            }
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
                Serial.print("RSSI: ");Serial.println(LMIC.rssi);Serial.print("SNR: ");Serial.println(LMIC.snr);         }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
          for (int i=0; i<int(TX_INTERVAL/8); i++) {
            // Use library from https://github.com/rocketscream/Low-Power
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
          }            
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
    next = true;
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        readData();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
    TX_done = false;
}

int Log_flag=0;
long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float diff = 1.0;

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void readData()
{
    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();

    if (checkBound(newTemp, temp, diff)) {
      temperature = newTemp;
    }

    if (checkBound(newHum, hum, diff)) {
      humidity = newHum;
    }

    tempC_int = temperature*10;
    hum_int = humidity*2;
    mydata.temp[0] = tempC_int >> 8;
    mydata.temp[1] = tempC_int;
    mydata.humid[0] =  hum_int ;    
    mydata.adc[0] = adc_int >> 8;
    mydata.adc[1] = adc_int;    
    vbat_int = readVcc()*0.1;
    mydata.vbat[0] = vbat_int >> 8;
    mydata.vbat[1] = vbat_int;    

#ifdef MY_DEBUG  
    Serial.print(temperature);Serial.print(" ");Serial.print(humidity);Serial.print(" ");Serial.print(readVcc());
#endif
}

void setup() {
  byte i;
  uint8_t j,a, result;
  float x;

#ifdef MY_DEBUG     
    Serial.begin(9600);
    Serial.println("Starting...");
#endif    
   
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
    LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
    LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
    LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band          
    LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    //LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)

    //forceTxSingleChannelDr();    
    LMIC_setDrTxpow(dr,14);
    next = true;
    Serial.println("Leave setup...");
}

void loop() {
 if (next == false) {  
    os_runloop_once();
  }else {    
    next = false;
    do_send(&sendjob);
 }
}

