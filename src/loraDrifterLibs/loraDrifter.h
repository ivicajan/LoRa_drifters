#ifndef LORADRIFTER_H
#define LORADRIFTER_H

#define USING_MESH        // Comment this defintion to prevent the use of mesh network functionality
// #define DEBUG_MODE        // Additional printouts to serial port
// #define IGNORE_GPS_INSIDE // For testing indoors, so we dont just send on only GPS second

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
#include "axp20x.h"         // PMU library

#include "loraDrifterDataTypes.h"

// Defines for TTGO T Beam V1.1 with LoRa
#define GPS_RX_PIN                  (34)
#define GPS_TX_PIN                  (12)
#define BUTTON_PIN                  (38)
#define BUTTON_PIN_MASK             (GPIO_SEL_38)
#define I2C_SDA                     (21)
#define I2C_SCL                     (22)
#define PMU_IRQ                     (35)
#define UNUSE_PIN                   (0)
#define BOARD_LED                   (4)
#define LED_ON                      (LOW)
#define LED_OFF                     (HIGH)
#define WEB_SERVER_PIN              (BUTTON_PIN)

#define GPS_BAUD_RATE               (9600)
#define SAMPLES_BEFORE_WRITE        (300)    // Number of samples to store in memory before file write
#define SPIFFS_FLASH_SIZE           (3.f)    // SPIFFS formatted size of storage in MB

// LoRa Pins
#define RADIO_SCLK_PIN              (5)
#define RADIO_MISO_PIN              (19)
#define RADIO_MOSI_PIN              (27)
#define RADIO_CS_PIN                (18)
#define RADIO_DI0_PIN               (26)
#define RADIO_RST_PIN               (23)

// LoRa parameters
#define LORA_FREQUENCY              (915E6)
#define LORA_TX_POWER               (20)     // defaults to 17 dB - between 2 <-> 20
#define LORA_SPREADING_FACTOR       (8)      // defaults to 7 - between 6 <-> 12
#define LORA_SIGNAL_BANDWIDTH       (250E3)  // defaults to 125E3
#define LORA_CODING_RATE            (6)      // defaults to 5 - between 5 <-> 8
#define LORA_PREAMBLE_LENGTH                 // defaults to 8 - between 6 <-> 65535
#define LORA_SYNC_WORD                       // defaults to 0x12
#define LORA_GAIN                            // defaults to 0 - between 0 <-> 6


#ifdef USING_MESH
#include "loraDrifterMesh.h"
#endif //USING_MESH

/**
 * @brief Get the battery percentage as determined by the 
 * PMU voltage calculation.
 * 
 * @return current percentage of the battery
 */
float getBatteryPercentage();

/**
 * @brief Initialise the TTGO ESP32 board. Inclusive of PMU, GPS, and GPIO.
 * 
 */
void initBoard();

// D5. String IP Address
String IpAddress2String(const IPAddress& ipAddress);

#endif //LORADRIFTER_H
