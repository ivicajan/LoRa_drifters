
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

// F. Functions
void startWebServer(bool webServerOn);
String processor(const String& var);
String IpAddress2String(const IPAddress& ipAddress);

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
    PMU.setChgLEDMode(AXP20X_LED_OFF);

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
    Serial.begin(115200);
    Serial.println("initBoard");
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
 
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(50);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    delay(50);
}

// =======================================================================================
// A. Global variables
// =======================================================================================
TinyGPSPlus gps;
String drifterName = "D08";   // ID send with packet
int drifterTimeSlotSec = 5; // seconds after start of each GPS minute
int nSamplesFileWrite = 300;      // Number of samples to store in memory before file write
const char* ssid = "DrifterServant";   // Wifi ssid and password
const char* password = "Tracker1";
String hour,minute,second,year,month,day,tTime,tDate;
String csvOutStr = "";                 // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);
bool webServerOn = false;
String csvFileName = "";
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
int webServerPin = BUTTON_PIN;
int gpsLastSecond = -1;

int ledState = LOW;
int ledPin = BOARD_LED;

// H. This is the string literal for the main web page

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <title>UWA LoRa Drifters</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h2 {font-size: 3.0rem;}
      p {font-size: 3.0rem;}
      table, th, td { border: 1px solid black;}
      body {max-width: 700px; margin:0px auto; padding-bottom: 25px;}
    </style>
  </head>
  <body>

    <h2>LoRa Drifters</h2>
    <h4> Servant Node </h4>
    <table>
    <td>Filename </td>
    <td>Get Data Link </td>
    <td>Last File Write GPS Time</td>
    <td>Erase Data (NO WARNING)</td>
    </tr>
    %SERVANT%
    </table><br><br>

    <h4> Configuration </h4>
    <form action="/configure" method="get">
    <table>
    <tr>
    <td> Setting </td>
    <td> Current Values </td>
    <td> New Values </td>
    <td> Guidance </td>
    </tr>
    
    <tr> 
    <td> <label for="fname">Drifter ID:</label> </td>
    <td> %DRIFTERID% </td>
    <td> <input type="text" id="fname" name="drifterID"></td>
    <td> Drifter IDs from D00 to D11 </td>
    </tr>
    <tr>
    <td> <label for="lname">LoRa Sending Second:</label> </td>
    <td> %LORASENDSEC% </td>
    <td> <input type="text" id="lname" name="loraSendSec"></td>
    <td>  Sending second is from 0 to 59 seconds </td>
    </tr>
    </table>
    <input type="submit" value="Configure">
    </form>

  </body>
  </html>
)rawliteral";

// =======================================================================================
// B. Setup
// =======================================================================================

void setup() {
  // A. TTGO Power ups and init radio
  initBoard();
  delay(50);
  
  // B. Setup LEDs for information   
//  Need to setup this part
//  pinMode(BOARD_LED, OUTPUT);
//  digitalWrite(ledPin, ledState);     // will change state when a LoRa packet is received
  pinMode(webServerPin, INPUT);

  // C. Local GPS
  // Started inside initBoard()
  
  //Setup LoRa
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    delay(50);
  
  // G. SPIFFS to write data to onboard Flash
    if (!SPIFFS.begin(true)) {
      Serial.println("An Error has occurred while mounting SPIFFS - need to add retry");
      while (1);
    }
    delay(50);

  //H. Read config file if exists
  file = SPIFFS.open("/config.txt", FILE_READ);
  if (!file){
    Serial.println("Failed to open config.txt configuration file");
  } else {
    String inData=file.readStringUntil('\n');
    int comma = inData.indexOf(",");
    drifterName = inData.substring(0,comma);
    drifterTimeSlotSec = inData.substring(comma+1).toInt();
    Serial.println(inData);
  }
  delay(50);

  csvFileName="/svt"+String(drifterName)+".csv";
  webServerOn = false;
  Serial.println("init ok");
  delay(50);
}



// =======================================================================================
// C. Loop
// =======================================================================================
void loop() {
  
  // A. Check for button press
  if ( digitalRead(BUTTON_PIN) == LOW ) {
    if (webServerOn) {
      Serial.println("Web server already started");
      webServerOn = false;
      startWebServer(webServerOn);
      delay(1000);
    } else {
      webServerOn = true;
      startWebServer(webServerOn);
      Serial.println("Web server started");
      delay(1000);
    }
  }

  if (!webServerOn) {
    // A. Receive and Decode GPS data and send via LoRa if in time slot 
    SerialGPSDecode(Serial1, gps);
    delay(10);
    
    // B. Write data to onboard flash if nSamples is large enough
    Serial.println("nSamples:" + String(nSamples));
    if (nSamples >= nSamplesFileWrite) {  // only write after collecting a good number of samples
        Serial.println("Dump data into the memory");
        writeData2Flash();
    }
  }

  if (webServerOn){
    Serial.println("Web server is ON, not GPS data or saving during the time");
    digitalWrite(BOARD_LED, HIGH);
    delay(40);
    digitalWrite(BOARD_LED, LOW); 
    delay(40);
  }

}


// =======================================================================================
// D. Functions
// =======================================================================================


// D0. Write data to flash
// 
void writeData2Flash (){
  file = SPIFFS.open(csvFileName, FILE_APPEND);
  if (!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
    ESP.restart();
  } else {
    if (file.println(csvOutStr)) {
      Serial.println("Wrote data in file, current size:");
      Serial.println(file.size());
      csvOutStr = ""; nSamples = 0;
      lastFileWrite = tTime;
    } else {
      lastFileWrite = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS) {
  // Read GPS and run decoder
    unsigned long start = millis();
    do
    {
      while (Serial1.available() > 0)
        gps.encode(Serial1.read());     
    } while (millis() - start < 500);
    // C. If this is a new GPS record then save it
    if (gps.time.second() != gpsLastSecond) {
        hour = String(gps.time.hour());
        minute = String(gps.time.minute());
        second = String(gps.time.second());
        year = String(gps.date.year());
        month = String(gps.date.month());
        day = String(gps.date.day());      
        if (hour.length() == 1)
        {
           hour = "0" + hour;     
        }
        if (minute.length() == 1)
        {
           minute = "0" + minute;     
        }
        if (second.length() == 1)
        {
           second = "0" + second;     
        } 
        if (month.length() == 1)
        {
           month = "0" + month;     
        }  
        if (day.length() == 1)
        {
           day = "0" + day;     
        }
         tDate = year + "-" + month + "-" + day;
         tTime = hour + ":" + minute + ":" + second;
         String tLocation = String(gps.location.lat(), 8) + "," + String(gps.location.lng(), 8) + "," + String(gps.location.age());
         String sendPacket = String(drifterName) + "," + String(drifterTimeSlotSec) + "," + tDate + "," + tTime + "," + tLocation + "," + String(nSamples) + "\n";
         gpsLastSecond = gps.time.second();
         if (gps.location.lng() != 0.0){
         csvOutStr += tDate + "," + tTime + "," + tLocation + "\n";
         nSamples += 1;
         // B. Send GPS data on LoRa if it is this units timeslot
          if (gps.time.second() == drifterTimeSlotSec) {
              Serial.println("sending packet via LoRa");
              Serial.println(sendPacket);
              csvOutStr += sendPacket; // Save any packets that are sent (debugging purposes).
              LoRa.beginPacket();
              LoRa.print(sendPacket);
              LoRa.endPacket(true);
              delay(50); // Don't send more than 1 packet
          }
          } else {
            Serial.println(" NO GPS FIX, NOT SENDING OR WRITING ");
          }
          
    }
    delay(50);
}

// D3. Web Server Setup
void startWebServer(bool webServerOn) {

  if (webServerOn) {
    WiFi.softAP(ssid, password);
    Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

    // F. Web Server Callbacks setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/configure", HTTP_GET,
      [](AsyncWebServerRequest * request) {
        int paramsNr = request->params();
        Serial.println(paramsNr);
        for(int i=0;i<paramsNr;i++){
            AsyncWebParameter* p = request->getParam(i);
            if (p->name() == "drifterID"){
              drifterName=p->value();
            }
            if (p->name() == "loraSendSec"){
              drifterTimeSlotSec=String(p->value()).toInt();
            }
        }
        csvFileName="/svt"+String(drifterName)+".csv";
        
        file = SPIFFS.open("/config.txt", FILE_WRITE);
        if (!file){
          Serial.println("Could not open config.txt for writing");
          request->send(200, "text/plain", "Failed writing configuration file config.txt!");
        } else {
          file.print(drifterName+","+String(drifterTimeSlotSec));
          file.close();
          request->send(200, "text/html", "<html><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"\">Success!  BACK </a></html>");
        }
    });
    
    server.on("/getServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      writeData2Flash();
      request->send(SPIFFS, csvFileName, "text/plain", true);
    });
    
    server.on("/deleteServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      SPIFFS.remove(csvFileName);
      file = SPIFFS.open(csvFileName, FILE_WRITE);
      if (!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }
      if (file.println("#FILE ERASED at " + lastFileWrite)) {
        Serial.println("File was created");
      } else {
        Serial.println("File creation failed");
      }
      file.close();
      lastFileWrite = "";
      request->send(200, "text/html", "<html><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"\">Success!  BACK </a></html>");
    });
    server.begin();
  } else {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
  }

}


// D4. Used to update sections of the webpages
//  Replaces placeholder with button section in your web page

String processor(const String& var) {
  if (var == "SERVANT") { 
    
     String servantData = "";
     servantData += "<td>" + csvFileName +"</td>";
     servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getServant\"> GET </a></td>";
     servantData += "<td>" + lastFileWrite + "</td>";
     servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteServant\"> ERASE </a></td>";
     servantData += "</tr>";
     return servantData;  
  }

  if (var == "DRIFTERID") {
    return drifterName;
  }
  if (var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
  return String();
}


// D5. String IP Address
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}
