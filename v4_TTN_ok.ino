//  11marco19
//  placa wifi_lora_32
//  TTN OK
//  Dados enviados com sucesso



#include "heltec.h"
#define WIFI_LoRa_32_V2
#define BAND  915E6 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define Fbattery    3700  //The default battery is 3700mv when the battery is fully charged.
float XS = 0.00225;      //The returned reading is multiplied by this XS to get the battery voltage.
uint16_t MUL = 1000;
uint16_t MMUL = 100;

// Credenciais TTN
static const u1_t PROGMEM APPEUI[8]={ 0x50, 0x13, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
static const u1_t PROGMEM DEVEUI[8]={ 0xEC, 0xEF, 0xEF, 0xEF, 0xEF, 0xEF, 0xEF, 0xEC };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
static const u1_t PROGMEM APPKEY[16] = {0x94, 0xA1, 0x47, 0xF3, 0x03, 0xDA, 0x85, 0x7C, 0xB9, 0xA1, 0x7F, 0xC5, 0x51, 0x30, 0xE7, 0xF1};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// payload para envio dos dados ao gateway TTN 
static uint8_t payload[7];
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;
int  PROXTX_INTERVAL = TX_INTERVAL;


// Pinagem Heltec
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 35, LMIC_UNUSED_PIN},
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

            LMIC_setLinkCheckMode(0);
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
            Serial.println(F("EV_TXCOMPLETE (Janelas RX inclusas)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Recebido ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Recebido "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes de payload"));
            }
            // Proxima transmissao
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // dados recebidos
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
    // Checa se não há rotinas de TX/RX em andamento
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, nao esta enviando"));
    } else {
   
        float temperature = 28.5;
        Serial.print("Temperatura: "); Serial.print(temperature);
        Serial.println(" *C");
        // adjust for the f2sflt16 range (-1 to 1)
        temperature = temperature / 100; 
        
        if (temperature <= 38){PROXTX_INTERVAL = TX_INTERVAL * 20;} 
        if (temperature >42) {PROXTX_INTERVAL = TX_INTERVAL;}
        else {PROXTX_INTERVAL = TX_INTERVAL * 10;} 
     

        float rHumidity = 73.8;
        Serial.print("%RH ");
        Serial.println(rHumidity);
        rHumidity = rHumidity / 100;

        // Voltagem da bateria
          adcAttachPin(13);
          analogSetClockDiv(255); // 1338mS
          uint16_t measuredvbat  =  analogRead(13)*XS*MUL;

        uint16_t payloadTemp = LMIC_f2sflt16(temperature);
        // int -> bytes
        byte tempLow = lowByte(payloadTemp);
        byte tempHigh = highByte(payloadTemp);
        // place the bytes into the payload
        payload[0] = tempLow;
        payload[1] = tempHigh;

        // float -> int
        uint16_t payloadHumid = LMIC_f2sflt16(rHumidity);
        // int -> bytes
        byte humidLow = lowByte(payloadHumid);
        byte humidHigh = highByte(payloadHumid);
        payload[2] = humidLow;
        payload[3] = humidHigh;

        uint16_t payloadMeasuredvbat = LMIC_f2sflt16(measuredvbat);
        // int -> bytes
        byte measuredvbatLow = lowByte(payloadMeasuredvbat);
        byte measuredvbatHigh = highByte(payloadMeasuredvbat);
        // place the bytes into the payload
        payload[4] = measuredvbatLow;
        payload[5] = measuredvbatHigh;

       Serial.println(measuredvbatLow);
       Serial.println(measuredvbatHigh);
   
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
 
      
    }
   
}

void setup() {
    Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
    Heltec.display->init();
    Serial.begin(9600);
    Serial.println(F("Iniciando"));

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setLinkCheckMode(0);
    // alterado de sf7 para sf9
    LMIC_setDrTxpow(DR_SF9,14);
    LMIC_selectSubBand(1);

    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
