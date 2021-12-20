#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "softSerial.h"
#include "TinyGPS++.h"

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15 * 1000;

/* OTAA para*/
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t devEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/* ABP para*/
uint8_t nwkSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr = (uint32_t) 0x00000000;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares and sends the payload of the frame */
static void sendTxFrame(uint8_t port) {
    /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    *for example, if use REGION_CN470,
    *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
    */
    pinMode(GPIO0, OUTPUT);
    pinMode(Vext, OUTPUT);
    digitalWrite(GPIO0, HIGH);
    digitalWrite(Vext, LOW);

    TinyGPSPlus gps;
    softSerial ss(GPIO5 /*TX pin*/, GPIO3 /*RX pin*/);
    ss.begin(9600);

    float lat, lon, alt, hdop;
    uint32_t sats, time, date;

    Serial.println("Waiting for GPS FIX ...");
    uint32_t endTime = millis() + 60 * 1000;
    while ((!gps.location.isValid() || gps.hdop.hdop() > 2.0) && endTime > millis()) {
        while (ss.available()) {
            char c = ss.read();
            Serial.print(c);
            gps.encode(c);
        }
    }
    Serial.println();

    lat = (float) gps.location.lat();
    lon = (float) gps.location.lng();
    alt = (float) gps.altitude.meters();
    hdop = (float) gps.hdop.hdop();
    sats = gps.satellites.value();
    time = gps.time.value();
    date = gps.date.value();

    digitalWrite(Vext, HIGH);

    digitalWrite(GPIO0, LOW);
    pinMode(GPIO0, ANALOG);

    uint16_t batteryVoltage = getBatteryVoltage();

    Serial.print("AGE: ");
    Serial.print(gps.location.age());
    Serial.print(", LAT: ");
    Serial.print(gps.location.lat(), 15);
    Serial.print(", LON: ");
    Serial.print(gps.location.lng(), 15);
    Serial.print(", ALT: ");
    Serial.print(gps.altitude.meters(), 15);
    Serial.print(", SATS: ");
    Serial.print(sats);
    Serial.print(", HDOP: ");
    Serial.print(hdop);
    Serial.print(", TIME: ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.print(gps.time.second());
    Serial.print(".");
    Serial.print(gps.time.centisecond());
    Serial.print(", DATE: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print(", Battery Voltage:");
    Serial.println(batteryVoltage);

    if (!gps.location.isValid() || gps.hdop.hdop() > 2.0) {
        Serial.println("\nGPS FIX not valid");
        return;
    }

    unsigned char *puc;
    appDataSize = 0;
    puc = (unsigned char *) (&lat);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&lon);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&alt);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&hdop);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&sats);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&time);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&date);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];
    appData[appDataSize++] = puc[2];
    appData[appDataSize++] = puc[3];
    puc = (unsigned char *) (&batteryVoltage);
    appData[appDataSize++] = puc[0];
    appData[appDataSize++] = puc[1];

    LoRaWAN.send();
}

void setup() {
    Serial.begin(115200);
    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();
}

void loop() {
    switch (deviceState) {
        case DEVICE_STATE_INIT: {
            printDevParam();
            LoRaWAN.init(loraWanClass, loraWanRegion);
            LoRaWAN.setDataRateForNoADR(5); // Use single datarate in tracking applications
            deviceState = DEVICE_STATE_JOIN;
            break;
        }
        case DEVICE_STATE_JOIN: {
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND: {
            sendTxFrame(appPort);
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_CYCLE: {
            // Schedule next packet transmission
            txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP: {
            LoRaWAN.sleep();
            break;
        }
        default: {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}