#ifndef LORADRIFTER_H
#define LORADRIFTER_H

// A. WiFi & Web Server
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// B. SPI & LoRa
#include <SPI.h>
#include <LoRa.h>

// B1. SPIFFS
#include "SPIFFS.h"

// C. GPS libraries
#include <Wire.h>           // for reseting NMEA output from some T-Beam units
#include <TinyGPS++.h>      // decoding GPS

// D. Power management on the TTGo T-beam
#include "axp20x.h"         // I need it for the new ones

// E. Defines for TTGO T Beam V1.1 with LoRa
#define GPS_RX_PIN                  34
#define GPS_TX_PIN                  12
#define BUTTON_PIN                  38
#define BUTTON_PIN_MASK GPIO_SEL_38
#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define RADIO_SCLK_PIN               5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DI0_PIN               26
#define RADIO_RST_PIN               23

// LoRa definitions
#define LORA_FREQUENCY              915E6
#define LORA_TX_POWER               20     // defaults to 17 dB - between 2 <-> 20
#define LORA_SPREADING_FACTOR       8      // defaults to 7 - between 6 <-> 12
#define LORA_SIGNAL_BANDWIDTH       250E3  // defaults to 125E3
#define LORA_CODING_RATE            6      // defaults to 5 - between 5 <-> 8
#define LORA_PREAMBLE_LENGTH               // defaults to 8 - between 6 <-> 65535
#define LORA_SYNC_WORD                     // defaults to 0x12
#define LORA_GAIN                          // defaults to 0 - between 0 <-> 6

#define GPS_BAND_RATE               9600
#define UNUSE_PIN                   (0)
#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH
#define WEB_SERVER_PIN              BUTTON_PIN
#define ROUTING_TABLE_SIZE          153
#define SAMPLES_BEFORE_WRITE        300    // Number of samples to store in memory before file write
#define NUM_MAX_SERVANTS            8      // Maximum number of servant drifters (just for setting array size)

#define DEBUG_MODE                         // Additional printouts to serial port
// #define IGNORE_GPS_INSIDE                  // For testing indoors, so we dont just send on GPS time

AXP20X_Class PMU;
TinyGPSPlus gps;                      // decoder for GPS stream
AsyncWebServer server(80);            // Create AsyncWebServer object on port 80

// 3 + 4 + 2 + (1 * 5) + (2 * 8) + 4 + 4 = 38 bytes
#pragma pack(1) // Fixes padding issues
struct Packet {
  char name[3];             // D01
  int drifterTimeSlotSec;   // 15
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  double lng;
  double lat;
  uint32_t age;
  int nSamples;
};

class Master {
  public:
    Master() = default;
    double lng = 0.0;
    double lat = 0.0;
    int year = 0;
    uint16_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint32_t age = 0;
    ~Master() = default;
};

#ifdef MASTER_NODE
#include "../../loraDrifterMaster.h"
#endif // MASTER_MODE

#ifdef USING_MESH
#include "loraDrifterMesh.h"
#endif //USING_MESH

bool initPMU() {
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50);
    if(PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        return false;
    }
    PMU.setChgLEDMode(AXP20X_LED_OFF); //The charging indicator can be turned on or off

    /*
     * The default ESP32 power supply has been turned on,
     * no need to set, please do not set it, if it is turned off,
     * it will not be able to program
     **/

    // PMU.setDCDC1Voltage(3300);
    // PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

    /*
     *  Turn off unused power sources to save power
     **/

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
     * Set the power of LoRa and GPS module to 3.3V
     **/
    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD
    PMU.setDCDC1Voltage(3300);  //3.3V Pin next to 21 and 22 is controlled by DCDC1

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
        // pmu_irq = true;
    }, FALLING);

    PMU.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                    AXP202_VBUS_CUR_ADC1 |
                    AXP202_BATT_CUR_ADC1 |
                    AXP202_BATT_VOL_ADC1,
                    AXP202_ON);

    PMU.enableIRQ(AXP202_VBUS_REMOVED_IRQ |
                    AXP202_VBUS_CONNECT_IRQ |
                    AXP202_BATT_REMOVED_IRQ |
                    AXP202_BATT_CONNECT_IRQ,
                    AXP202_ON);
    PMU.clearIRQ();
    return true;
}

void initBoard() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("initBoard");
    initPMU();
    delay(50);
#ifdef BOARD_LED
      /*
      * T-BeamV1.0, V1.1 LED defaults to low level as turn on,
      * so it needs to be forced to pull up
      * * * * */
#if LED_ON == LOW
    gpio_hold_dis(GPIO_NUM_4);
#endif
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif
    delay(50);
    pinMode(WEB_SERVER_PIN, INPUT);
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("init gps");
    delay(50);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    delay(50);
}

// D5. String IP Address
String IpAddress2String(const IPAddress& ipAddress) {
  return String(ipAddress[0]) + String(".") +
    String(ipAddress[1]) + String(".") +
    String(ipAddress[2]) + String(".") +
    String(ipAddress[3]);
}

#endif //LORADRIFTER_H
