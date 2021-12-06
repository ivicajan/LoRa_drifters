#include "src/loraDrifterLibs/loraDrifter.h"

// #define USING_IMU
#ifdef USING_IMU
#include "mpu/imu.h"
#endif // USING_IMU

#ifdef USING_MESH
byte routingTable[ROUTING_TABLE_SIZE] = "";
byte payload[24] = "";
int localLinkRssi = 0;
byte localHopCount = 0x00;
byte localNextHopID = 0x00;
byte localAddress = 0x55;
// Diagnostics
int messagesSent = 0;
int messagesReceived = 0;
int node1Rx = 0;
int node2Rx = 0;
int node3Rx = 0;
int node4Rx = 0;
int node5Rx = 0;
int node6Rx = 0;
int node7Rx = 0;
int masterRx = 0;
#endif // USING_MESH

#define SSID     "DrifterServant"   // Wifi ssid and password
#define PASSWORD "Tracker1"

// GLOBAL VARIABLES
String csvOutStr = "";                 // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);
bool webServerOn = false;
String csvFileName = "";
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered

int gpsLastSecond = -1;
String tTime = "";
String drifterName = "D05";       // ID send with packet
int drifterTimeSlotSec = 20;      // seconds after start of each GPS minute
TinyGPSPlus gps;

Packet packet;
SemaphoreHandle_t loraSemaphore = NULL;

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <title>UWA LoRa Drifters</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <link rel="icon" href="data:,">
      <style>
        html {
          font-family: Arial; display: inline-block; text-align: center;
        }
        h2 {
          font-size: 3.0rem;
        }
        p {
          font-size: 3.0rem;
        }
        table {
          width: 100%%;
        }
        table, th, td {
           border: 1px solid black;
        }
        body {
          max-width: 90%%; margin:0px auto; padding-bottom: 25px;
        }
      </style>
    </head>
    <body>
      <h2>LoRa Drifters</h2>
      <h4>Servant Node</h4>
      <table>
        <tr>
          <td><b>Filename</b></td>
          <td><b>Download data</b></td>
          <td><b>Last File Write GPS Time</b></td>
          <td><b>Erase Data (NO WARNING)</b></td>
          <td><b>Calibrate IMU</b></td>
        </tr>
        %SERVANT%
      </table>
      <br><br>
      <h4>Configuration</h4>
      <form action="/configure" method="get">
        <table>
          <tr>
            <td><b>Setting</b></td>
            <td><b>Current Values</b></td>
            <td><b>New Values</b></td>
            <td><b>Guidance</b></td>
          </tr>
          <tr>
            <td><label for="fname"><b>Drifter ID</b></label></td>
            <td>%DRIFTERID%</td>
            <td><input type="text" id="fname" name="drifterID"></td>
            <td>Drifter IDs from D00 to D07</td>
          </tr>
          <tr>
            <td><label for="lname"><b>LoRa Sending Second</b></label></td>
            <td>%LORASENDSEC%</td>
            <td><input type="text" id="lname" name="loraSendSec"></td>
            <td>Sending second is from 0 to 59 seconds</td>
          </tr>
        </table>
        <input type="submit" value="Configure">
      </form>
      <h4>Diagnostics</h4>
      <table>
        <tr>
          <td><b>Sent</b></td>
          <td><b>Recvd</b></td>
          <td><b>node1</b></td>
          <td><b>node2</b></td>
          <td><b>node3</b></td>
          <td><b>node4</b></td>
          <td><b>node5</b></td>
          <td><b>node6</b></td>
          <td><b>node7</b></td>
          <td><b>master</b></td>
        </tr>
        %DIAGNOSTICS%
      </table>
      <br><br>
    </body>
  </html>
)rawliteral";

static void writeData2Flash() {
  file = SPIFFS.open(csvFileName, FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for appending, creating a new one");
    file = SPIFFS.open(csvFileName, FILE_WRITE);
  }
  if(!file) {
      Serial.println("There was an error opening the file for writing");
      lastFileWrite = "FAILED OPEN";
      ESP.restart();
  } else {
    if(file.println(csvOutStr)) {
      Serial.print("Wrote data in file, current size: ");
      Serial.println(file.size());
      csvOutStr = "";
      nSamples = 0;
      lastFileWrite = tTime;
    } else {
      lastFileWrite = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

#ifdef USING_IMU
static void update_imu() {
  if(mpu.update()) {
    static uint32_t prev_ms = millis();
    static uint32_t imu_ms = millis(); //For reset
    if(millis() > prev_ms + SAMPLE_PERIOD_ms) {
      if(millis() > imu_ms + 20000) {
        update_ref_location(); // Reset data after 20 secs
        imu_ms = millis();
      }
      while(Serial1.available() > 0) {
        gps.encode(Serial1.read());
        measure_gps_data();
      }
      //Run Kalman with fusion IMU and GPS
      Update_Kalman();
      if(mpu.update()) {
        measure_imu_data();
      }
      //Ouput current location in lat and lng
      float lat, lng;
      get_current_location(&lat, &lng);
      Serial << "Lat: " << lat << " Lng: " << lng << "\n";

  #ifdef DEBUG_MODE
      Serial << " Acc: " << acc << " Yew[0]: " << float(mpu.getYaw() / 180.f * PI)
              << " pitch: " << float(mpu.getPitch() / 180.f * PI)
              << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
  #endif // DEBUG_MODE
      prev_ms = millis();
    }
  }
}
#endif // USING_IMU

static void listenTask(void * params) {
  (void)params;
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // might not need a delay at all
    const int result = listener(LoRa.parsePacket(), SERVANT_MODE);
  }
}

static void sendTask(void * params) {
  (void)params;
  while(1) {
    if(digitalRead(BUTTON_PIN) == LOW) { //Check for button press
      if(webServerOn) {
        Serial.println("Web server already started");
        webServerOn = false;
      } else {
        Serial.println("Web server started");
        webServerOn = true;
      }
      startWebServer(webServerOn);
      delay(1000);
    }
    if(!webServerOn) {
#ifdef USING_MESH
      int result = 0;
      // if(gps.time.second() == drifterTimeSlotSec) {
#ifdef USING_IMU
      update_imu();
#endif // USING_IMU
      if(runEvery(PL_TX_TIME)) {
        generatePacket();
        result = routePayload(SERVANT_MODE, 0xAA, localAddress, 0x0F, 0);
      }
      if(loop_runEvery(RS_BCAST_TIME)) {
        Serial.println("Route broadcast");
        if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
          result = bcastRoutingStatus(SERVANT_MODE);   // returns 1 or -1
        }
        xSemaphoreGive(loraSemaphore);
      }
#else
      generatePacket();
#endif // USING_MESH
      delay(10);
      // Write data to onboard flash if nSamples is large enough
      if(nSamples >= SAMPLES_BEFORE_WRITE) {  // only write after collecting a good number of samples
        Serial.println("Dump data into the memory");
        writeData2Flash();
      }
    } else {
      Serial.println("Web server is ON, not GPS data or saving during the time");
      delay(40);
      digitalWrite(BOARD_LED, LED_OFF);
      delay(40);
      digitalWrite(BOARD_LED, LED_ON);
    }
  }
}

// Init the LoRa communications with/without default factory settings
static void initLoRa(bool default_params = false) {
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if(!default_params) {
    /* LoRa paramters - https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md */

    // LoRa.setFrequency(LORA_FREQUENCY);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    // LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    // LoRa.setSyncWord(LORA_SYNC_WORD);
    // LoRa.setGain(LORA_GAIN);
    // Enable CRC --> if packet is corrupted, packet gets dropped silently
    LoRa.enableCrc();
  }
  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(true); // if failed, do nothing
  }
}

//H. Read config file, if it exists
static void readConfigFile() {
  file = SPIFFS.open("/config.txt", FILE_READ);
  if(!file) {
    Serial.println("Failed to open config.txt configuration file");
  } else {
    String inData = file.readStringUntil('\n');
    int comma = inData.indexOf(",");
    drifterName = inData.substring(0, comma);
    drifterTimeSlotSec = inData.substring(comma + 1).toInt();
    Serial.println(inData);
    file.close();
  }
  delay(50);
  csvFileName = "/svt" + String(drifterName)+".csv";
}

void setup() {
  initBoard();
  delay(500);
#ifdef USING_IMU
  initIMU();
  delay(500);
#endif // USING_IMU
  if(!SPIFFS.begin(true)) {
    Serial.println("SPIFFS ERROR HAS OCCURED");
    return;
  }
  initLoRa();
  delay(50);
  readConfigFile();
  loraSemaphore = xSemaphoreCreateMutex();
  // Create listen and send RTOS tasks
  xTaskCreatePinnedToCore(listenTask, "listenTask", 10000, NULL, 1, NULL, 0);
  delay(500);
  xTaskCreatePinnedToCore(sendTask, "sendTask", 10000, NULL, 2, NULL, 1);
  delay(500);
  Serial.println("Initialization complete.");
}

static void fill_packet() {
  strcpy(packet.name, drifterName.c_str());
  packet.drifterTimeSlotSec = drifterTimeSlotSec;
  packet.hour = gps.time.hour();
  packet.minute = gps.time.minute();
  packet.second = gps.time.second();
  packet.year = gps.date.year();
  packet.month = gps.date.month();
  packet.day = gps.date.day();
  packet.lng = gps.location.lng();
  packet.lat = gps.location.lat();
  packet.nSamples = nSamples;
  packet.age = gps.location.age();
}

static void generatePacket() {
  const uint32_t start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 400);
  // if(gps.time.second() != gpsLastSecond) {
  if(true) { // TODO: Delete this when in field
    Serial.print("New GPS record from: ");
    Serial.println(drifterName);
    fill_packet();
    gpsLastSecond = gps.time.second();
    if((gps.location.lng() != 0.0) && (gps.location.age() < 1000)) {
      Serial.println("GPS still valid");
      nSamples += 1;
      const String tDate = String(packet.year) + "-" + String(packet.month) + "-" + String(packet.day);
      tTime = String(packet.hour) + ":" + String(packet.minute) + ":" + String(packet.second);
      const String tLocation = String(packet.lng, 6) + "," + String(packet.lat, 6) + "," + String(packet.age);
      csvOutStr += tDate + "," + tTime + "," + tLocation + String(messagesReceived) + String(messagesSent) + "\n";
#ifndef USING_MESH
      // B. Send GPS data on LoRa if it is this units timeslot
      if(gps.time.second() == drifterTimeSlotSec) {
        LoRa.beginPacket();
        LoRa.write((const uint8_t*)&packet, sizeof(packet));
        LoRa.endPacket();
        delay(50); // Don't send more than 1 packet
      }
#endif // USING_MESH
    } else {
      Serial.println("No GPS fix, not sending or writing");
    }
  }
}

static bool calibrateIMU() {
#ifndef USING_IMU
  return false;
#endif // USING_IMU
  // calibrate IMU
  file = SPIFFS.open("imuCalibrations.txt", FILE_READ);
  if(!file) {
    Serial.println("There was an error opening the file for appending, creating a new one");
    file = SPIFFS.open("imuCalibrations.txt", FILE_WRITE);
  }
  if(!file) { // this should be reached unless SPIFFS are not working
      Serial.println("There was an error opening the file for writing");
      lastFileWrite = "FAILED OPEN";
      ESP.restart();
  }
  // use calibrations if we have them
  return true;
}

static void startWebServer(const bool webServerOn) {
  if(!webServerOn) {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
  } else {
    WiFi.softAP(SSID, PASSWORD);
    Serial.println(WiFi.softAPIP());  // Print ESP32 Local IP Address

    // F. Web Server Callbacks setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/configure", HTTP_GET, [](AsyncWebServerRequest * request) {
      int paramsNr = request->params();
      Serial.println(paramsNr);
      for(int ii = 0; ii < paramsNr; ii++) {
        AsyncWebParameter* p = request->getParam(ii);
        if(p->name() == "drifterID") {
          drifterName = p->value();
        }
        if(p->name() == "loraSendSec") {
          drifterTimeSlotSec = String(p->value()).toInt();
        }
      }
      csvFileName = "/svt" + String(drifterName) + ".csv";
      file = SPIFFS.open("/config.txt", FILE_WRITE);
      if(!file) {
        Serial.println("Could not open config.txt for writing");
        request->send(200, "text/plain", "Failed writing configuration file config.txt!");
      } else {
        file.print(drifterName + "," + String(drifterTimeSlotSec));
        file.close();
        request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
      }
    });

    server.on("/getServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      writeData2Flash();
      request->send(SPIFFS, csvFileName, "text/plain", true);
    });

    server.on("/deleteServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      SPIFFS.remove(csvFileName);
      file = SPIFFS.open(csvFileName, FILE_WRITE);
      if(!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }
      if(file.println("#FILE ERASED at " + lastFileWrite)) {
        Serial.println("File was created");
      } else {
        Serial.println("File creation failed");
      }
      file.close();
      lastFileWrite = "";
      request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
    });

    server.on("/calibrateIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
      if(calibrateIMU()) {
        request->send(200, "text/html",
        "<html><span>Successfully calibrated IMU!</span><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Back</a></html>");
      }
      else {
        request->send(200, "text/html",
        "<html><span>Failed to calibrate IMU</span><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Back</a></html>");
      }
    });
    server.begin();
  }
}

// Loop does nothing as the loop functions are split into tasks
void loop() {}

static String processor(const String& var) {
  if(var == "SERVANT") {
    String servantData = "";
    servantData += "<td>" + csvFileName + "</td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getServant\"> GET </a></td>";
    servantData += "<td>" + lastFileWrite + "</td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteServant\"> ERASE </a></td>";
#ifdef USING_IMU
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/calibrateIMU\"> CALIBRATE </a></td>";
#endif // USING_IMU
    servantData += "</tr>";
    return servantData;
  }
  else if(var == "DRIFTERID") {
    return drifterName;
  }
  else if(var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
#ifdef USING_MESH
  else if(var == "DIAGNOSTICS") {
    String diagnosticData = "";
    diagnosticData += "<tr>";
    diagnosticData += "<td>" + String(messagesSent) + "</td>";
    diagnosticData += "<td>" + String(messagesReceived) + "</td>";
    diagnosticData += "<td>" + String(node1Rx) + "</td>";
    diagnosticData += "<td>" + String(node2Rx) + "</td>";
    diagnosticData += "<td>" + String(node3Rx) + "</td>";
    diagnosticData += "<td>" + String(node4Rx) + "</td>";
    diagnosticData += "<td>" + String(node5Rx) + "</td>";
    diagnosticData += "<td>" + String(node6Rx) + "</td>";
    diagnosticData += "<td>" + String(node7Rx) + "</td>";
    diagnosticData += "<td>" + String(masterRx) + "</td>";
    return diagnosticData += "</tr>";
  }
#endif // USING_MESH
  else {
    return String();
  }
}
