
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <DHT.h>

#define DHTPIN A0
#define DHTTYPE DHT11 
 

CayenneLPP lpp(51);
DHT dht(DHTPIN, DHTTYPE);
int compteur =0;

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x8A, 0x59, 0x27, 0x24, 0x20, 0xC7, 0x63, 0x0A, 0x57, 0xE9, 0x52, 0x9E, 0x33, 0x99, 0xD4, 0x6F };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] ={ 0x82, 0x3C, 0x99, 0xF4, 0x78, 0xB2, 0x2F, 0xBE, 0xF0, 0x26, 0xCF, 0x51, 0x26, 0x0E, 0xF8, 0x6E };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011EAA ; 

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4, 
    .dio = {2, 3, LMIC_UNUSED_PIN }, 
};


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
             if (LMIC.dataLen) 
             {
                // data received in rx slot after tx
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.print(F(" bytes of payload: 0x"));
                for (int i = 0; i < LMIC.dataLen; i++) 
                {
                  if (LMIC.frame[LMIC.dataBeg + i] < 0x10) 
                  {
                      Serial.print(F("0"));
              			  if (LMIC.frame[LMIC.dataBeg + 2] == 0x00)
              			  {
              				  digitalWrite(13,HIGH);
              		  	}
                     if (LMIC.frame[LMIC.dataBeg + 2] == 0x67)
                     {
                        digitalWrite(13,LOW);
                      }
                   }
                   Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                 }
                   Serial.println();
   
          }
     
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      compteur++;
      if (compteur ==1)
      {
     int H = dht.readHumidity();
     Serial.println(H);
     lpp.reset();
     lpp.addRelativeHumidity(1, H);
     LMIC_setTxData2(1,lpp.getBuffer(), lpp.getSize(), 0);
   
      }
      else if (compteur ==2)
      {
     int T = dht.readTemperature();
     Serial.println(T);
     
     lpp.reset();
     lpp.addTemperature(1, T);
     LMIC_setTxData2(1,lpp.getBuffer(), lpp.getSize(), 0);
      compteur = 0;
       }
     
       }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
 
    Serial.begin(9600); 
    Serial.println(F("Starting"));
    while(!Serial);
    dht.begin();
    pinMode(13,OUTPUT);

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
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

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
     LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
//    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band*/
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B

    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
    LMIC_setClockError (MAX_CLOCK_ERROR * 10/100);
    // Start job
    
    do_send(&sendjob);
}



void loop() {
    os_runloop_once();
}
