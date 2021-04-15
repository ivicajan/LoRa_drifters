
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
#include "axp20x.h"         // Not used on the versions I have but the new ones may need it
AXP20X_Class PMU;

// E. Defines for TTGO T Beam V1.1 with LoRa
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define BUTTON_PIN 38
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
#define RADIO_DIO1_PIN              33
#define RADIO_BUSY_PIN              32

#define GPS_BAND_RATE      9600
#define LoRa_frequency      915E6
#define UNUSE_PIN                   (0)
#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

#define nServantsMax  12           // Maximum number of servant drifters (just for setting array size)
#define nSamplesFileWrite  300      // Number of samples to store in memory before file write

// F. Function definitions
//void resetGPSNMEAOutput(Stream &mySerial);
String processor(const String& var);
//void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS);
void loraProcessRXData(int packetSize);

bool initPMU()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50);
    if (PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        return false;
    }
    /*
     * The charging indicator can be turned on or off
     * * * */
    // PMU.setChgLEDMode(LED_BLINK_4HZ);

    /*
    * The default ESP32 power supply has been turned on,
    * no need to set, please do not set it, if it is turned off,
    * it will not be able to program
    *
    *   PMU.setDCDC1Voltage(3300);
    *   PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    *
    * * * */

    /*
     *   Turn off unused power sources to save power
     * **/
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

SPIClass SDSPI(HSPI);

void initBoard()
{
    initPMU();
    delay(50);

    #ifdef BOARD_LED
    /*
    * T-BeamV1.0, V1.1 LED defaults to low level as trun on,
    * so it needs to be forced to pull up
    * * * * */
#if LED_ON == LOW
    gpio_hold_dis(GPIO_NUM_4);
#endif
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif
    delay(50);
    Serial.begin(115200);
    Serial.println("initBoard");
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(50);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    delay(50);    
}

// G. Classes for Master and Servant Data

class Master
{
  public:
    Master();
    float lon;
    float lat;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int age;
};
Master::Master() {
  lon = 0.0;
  lat = 0.0;
  year = 0;
  month = 0;
  day = 0;
  hour = 0;
  minute = 0;
  second = 0;
  age = 0;
};



class Servant
{
  public:
    Servant();
    void decode(String packet);
    void updateDistBear(float fromLon,float fromLat);
    int ID;
    int loraUpdatePlanSec;
    int lastUpdateMasterTime;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    float lon;
    float lat;
    int age;
    int count;
    float dist;
    float bear;
    int rssi;
};
Servant::Servant() {
  ID = -1;
  loraUpdatePlanSec = 0;
  lastUpdateMasterTime = 0;
  year = 0;
  month = 0;
  day = 0;
  hour = 0;
  minute = 0;
  second = 0;
  lon = 0.0;
  lat = 0.0;
  age = 0;
  count = 0;
  dist = 0.0;
  bear = 0.0;
  rssi = 0;
};

void Servant::updateDistBear(float fromLon,float fromLat) {
   dist=TinyGPSPlus::distanceBetween(fromLat, fromLon, lat, lon);
   bear=TinyGPSPlus::courseTo(fromLat, fromLon, lat, lon);
}

void Servant::decode(String packet) {
  // Example:  D01,15,yyyy-mm-dd,HH:MM:SS,-31.97758433,115.88428733,151,4104
  
  // Update Plan
  int comma1=packet.indexOf(",");
  int comma2=packet.indexOf(",",comma1+1);
  loraUpdatePlanSec=packet.substring(comma1+1,comma2).toInt();
  lastUpdateMasterTime=millis(); 

  // Date
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  String timeFull=packet.substring(comma1+1,comma2);
  
  int colon1=timeFull.indexOf("-");
  int colon2=timeFull.indexOf("-",colon1+1);
  year=timeFull.substring(0,colon1).toInt();
  month=timeFull.substring(colon1+1,colon2).toInt();
  day=timeFull.substring(colon2+1).toInt();

  
  // Time
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  timeFull=packet.substring(comma1+1,comma2);
  
  colon1=timeFull.indexOf(":");
  colon2=timeFull.indexOf(":",colon1+1);
  hour=timeFull.substring(0,colon1).toInt();
  minute=timeFull.substring(colon1+1,colon2).toInt();
  second=timeFull.substring(colon2+1).toInt();
  
  // Latitude
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  lat=packet.substring(comma1+1,comma2).toFloat();
  
  // Longitude
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  lon=packet.substring(comma1+1,comma2).toFloat();

 // Age
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  age=packet.substring(comma1+1,comma2).toInt();

  // Count
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  count=packet.substring(comma1+1,comma2).toInt();
  
}



// H. This is the string literal for the main web page

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>UWA LoRa Drifters</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="1" >
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    table, th, td { border: 1px solid black;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 6px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px}
    input:checked+.slider {background-color: #b30000}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}
  </style>
</head>
<body>

  <h2>LoRa Drifters</h2>
  
  <h4> Master Node </h4>
  <table>
  <tr><td>GPS Time</td>
  <td>Longitude</td>
  <td>Latitude</td>
  <td>GPS Age [milliSec]</td>
  <td>Get Data Link </td>
  <td>Last File Write GPS Time</td>
  <td>Erase Data (NO WARNING)</td>
  </tr>
  %MASTER%
  </table><br><br>
  
  <h4> Servants </h4>
  <table>
  <tr>
  <td>ID</td>
  <td>Lora Update Plan [sec]</td>
  <td>Last Update Master Time [sec] Ago</td>
  <td>Time</td>
  <td>Longitude</td>
  <td>Latitude</td>
  <td>Distance [m]</td>
  <td>Bearing [degN to]</td>
   <td>Count</td>
  <td>RSSI</td>
  </tr>
  %SERVANTS%
  </table>
  

</body>
</html>
)rawliteral";
