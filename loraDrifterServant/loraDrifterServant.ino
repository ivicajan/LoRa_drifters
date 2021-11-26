#include "src/loraDrifterLibs/loraDrifter.h"

// F. Functions
void startWebServer(const bool webServerOn);

// GLOBAL VARIABLES
TinyGPSPlus gps;
String drifterName = "D04";   // ID send with packet
int drifterTimeSlotSec = 35; // seconds after start of each GPS minute
int nSamplesFileWrite = 300;      // Number of samples to store in memory before file write
const char* ssid = "DrifterServant";   // Wifi ssid and password
const char* password = "Tracker1";
String csvOutStr = "";                 // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);
bool webServerOn = false;
String csvFileName = "";
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
int webServerPin = BUTTON_PIN;
int gpsLastSecond = -1;
String tTime = "";

Packet packet;

#ifdef USING_MESH
byte routingTable[153] = "";
byte payload[24] = "";
int localLinkRssi = 0;
byte localHopCount = 0x00;
byte localNextHopID = 0x00;
byte localAddress = 0x44;

// Diagnostics
int messages_sent = 0;
int messages_received = 0;
int node1Rx = 0;
int node2Rx = 0;
int node3Rx = 0;
int node4Rx = 0;
int node5Rx = 0;
int node6Rx = 0;
int node7Rx = 0;
int masterRx = 0;
#endif // USING_MESH

TaskHandle_t ListenTask;
TaskHandle_t SendTask;
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
        table, th, td {
           border: 1px solid black;
        }
        body {
          max-width: 700px; margin:0px auto; padding-bottom: 25px;
        }
      </style>
    </head>
    <body>
      <h2>LoRa Drifters</h2>
      <h4>Servant Node</h4>
      <table>
        <tr>
          <td>Filename</td>
          <td>Get Data Link</td>
          <td>Last File Write GPS Time</td>
          <td>Erase Data (NO WARNING)</td>
        </tr>
        %SERVANT%
      </table>
      <br><br>
      <h4>Routing table</h4>
      <table>
        <tr>
          <td>nodeID</td>
          <td>hopCount</td>
          <td>hopID</td>
          <td>Rssi</td>
          <td>snr</td>
          <td>currentTime</td>
        </tr>
        %ROUTINGTABLE%
      </table>
      <br><br>
      <h4>Configuration</h4>
      <form action="/configure" method="get">
        <table>
          <tr>
            <td>Setting</td>
            <td>Current Values</td>
            <td>New Values</td>
            <td>Guidance</td>
          </tr>
          <tr>
            <td><label for="fname">Drifter ID:</label></td>
            <td>%DRIFTERID%</td>
            <td><input type="text" id="fname" name="drifterID"></td>
            <td>Drifter IDs from D00 to D11</td>
          </tr>
          <tr>
              <td><label for="lname">LoRa Sending Second:</label></td>
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
          <td>Msgs Sent</td>
          <td>Msgs Recvd</td>
          <td>node1Rx</td>
          <td>node2Rx</td>
          <td>node3Rx</td>
          <td>node4Rx</td>
          <td>node5Rx</td>
          <td>node6Rx</td>
          <td>node7Rx</td>
          <td>masterRx</td>
        </tr>
        %DIAGNOSTICS%
      </table>
      <br><br>
    </body>
  </html>
)rawliteral";

void writeData2Flash() {
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

void setup() {
  initBoard();
  delay(500);
  if(!SPIFFS.begin(true)) {
    Serial.println("SPIFFS ERROR HAS OCCURED");
    return;
  }
  pinMode(webServerPin, INPUT);

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  /* LoRa paramters - https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md */

  // LoRa.setFrequency(LORA_FREQUENCY);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  // LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  // LoRa.setSyncWord(LORA_SYNC_WORD);
  // LoRa.setGain(LORA_GAIN);

  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(true); // if failed, do nothing
  }
  delay(50);
  
  // Enable CRC --> if packet is corrupted, packet gets dropped silently
  LoRa.enableCrc();
  //H. Read config file if exists
  file = SPIFFS.open("/config.txt", FILE_READ);
  if(!file) {
    Serial.println("Failed to open config.txt configuration file");
  } else {
    String inData = file.readStringUntil('\n');
    int comma = inData.indexOf(",");
    drifterName = inData.substring(0, comma);
    drifterTimeSlotSec = inData.substring(comma + 1).toInt();
    Serial.println(inData);
  }
  delay(50);

  csvFileName="/svt" + String(drifterName)+".csv";
  webServerOn = false;
  loraSemaphore = xSemaphoreCreateMutex();

  //create a task that will be executed in the listenTask() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    listenTask,      /* Task function. */
                    "listenTask",    /* name of task. */
                    10000,           /* Stack size of task */
                    NULL,            /* parameter of the task */
                    1,               /* priority of the task */
                    &ListenTask,     /* Task handle to keep track of created task */
                    0);              /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the sendTask() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    sendTask,        /* Task function. */
                    "sendTask",      /* name of task. */
                    10000,           /* Stack size of task */
                    NULL,            /* parameter of the task */
                    2,               /* priority of the task */
                    &SendTask,       /* Task handle to keep track of created task */
                    1);              /* pin task to core 1 */
  delay(500);
  Serial.println("Initialization complete.");
}

void generatePacket() {
  const String dName = drifterName;
  unsigned long start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 400);
  if(gps.time.second() != gpsLastSecond) {
    Serial.println("new GPS record");
    strcpy(packet.name, dName.c_str());
    packet.drifterTimeSlotSec = drifterTimeSlotSec;
    Serial.println(dName);
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
//#ifdef DEBUG_MODE
//    Serial.println(packet.hour);
//    Serial.println(packet.minute);
//    Serial.println(packet.second);
//    Serial.println(packet.year);
//    Serial.println(packet.month);
//    Serial.println(packet.day);
//    Serial.println(packet.lng);
//    Serial.println(packet.lat);
//#endif //DEBUG_MODE
    gpsLastSecond = gps.time.second();
    if((gps.location.lng() != 0.0) && (gps.location.age() < 1000)) {
      Serial.println("GPS still valid");
      nSamples += 1;
      const String tDate = String(packet.year) + "-" + String(packet.month) + "-" + String(packet.day);
      tTime = String(packet.hour) + ":" + String(packet.minute) + ":" + String(packet.second);
      const String tLocation = String(packet.lng, 8) + "," + String(packet.lat, 8) + "," + String(packet.age);
      csvOutStr += tDate + "," + tTime + "," + tLocation + String(messages_received) + String(messages_sent) + "\n";
      // B. Send GPS data on LoRa if it is this units timeslot
      if(gps.time.second() == drifterTimeSlotSec) {
        Serial.println("Sending packet via LoRa");
#ifndef USING_MESH
        LoRa.beginPacket();
        LoRa.write((const uint8_t*)&packet, sizeof(packet));
        LoRa.endPacket();
        delay(50); // Don't send more than 1 packet
#endif // USING_MESH
      }
    } else {
      Serial.println("NO GPS FIX, NOT SENDING OR WRITING");
    }
  }
}

void startWebServer(const bool webServerOn) {
  if(!webServerOn) {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
  } else {
    WiFi.softAP(ssid, password);
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
      Serial.print("Before csvFileName");
      csvFileName = "/svt" + String(drifterName) + ".csv";
      Serial.print("Before config file open");
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
    server.begin();
  }
}

void listenTask(void * pvParameters) {
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // might not need a delay at all
    // if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
      const int result = listener(LoRa.parsePacket(), SERVANT_MODE);
    // }
    // xSemaphoreGive(loraSemaphore);
  }
}

void sendTask(void * pvParameters) {
  while(1) {
    // A. Check for button press
    if(digitalRead(BUTTON_PIN) == LOW) {
      if(webServerOn) {
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
    if(!webServerOn) {
#ifdef USING_MESH
      int result = 0;
      // if(gps.time.second() == drifterTimeSlotSec) {
      if(runEvery(PL_TX_TIME)) {
        generatePacket();
        result = routePayload(
          SERVANT_MODE,       // Node Mode
          0xAA,               // recipient: Master
          localAddress,       // sender
          0x0F,               // ttl
          0                   // set resend counter
        );
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
      // B. Write data to onboard flash if nSamples is large enough
      if(nSamples >= nSamplesFileWrite) {  // only write after collecting a good number of samples
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

// Loop does nothing as the loop functions are split into tasks
void loop() {}

String processor(const String& var) {
  if(var == "SERVANT") {
    String servantData = "";
    servantData += "<td>" + csvFileName + "</td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getServant\"> GET </a></td>";
    servantData += "<td>" + lastFileWrite + "</td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteServant\"> ERASE </a></td>";
    servantData += "</tr>";
    return servantData;
  }
  if(var == "ROUTINGTABLE") {
    String routingData = "";
    for(int idx = 0; idx < NUM_NODES; idx++) {
      routingData += "<tr>";
      routingData += "<td>" + String(routingTable[idx * ROUTING_TABLE_ENTRY_SIZE]) + "</td>";
      routingData += "<td>" + String(routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1]) + "</td>";
      routingData += "<td>" + String(routingTable[(idx*ROUTING_TABLE_ENTRY_SIZE) + 2]) + "</td>";
      routingData += "<td>" + String(*(int*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3])) + "</td>";
      routingData += "<td>" + String(*(float*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7])) + "</td>";
      routingData += "<td>" + String(*(unsigned long*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11])) + "</td>";
      if(idx != NUM_NODES - 1) {
        routingData += "</tr>";
      }
    }
    routingData += "</tr>";
    return routingData;
  }
  if(var == "DRIFTERID") {
    return drifterName;
  }
  if(var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
  if(var == "DIAGNOSTICS") {
    String diagnosticData = "";
    diagnosticData += "<tr>";
    diagnosticData += "<td>" + String(messages_sent) + "</td>";
    diagnosticData += "<td>" + String(messages_received) + "</td>";
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
  return String();
}
