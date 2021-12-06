#define USING_MESH

#ifdef USING_MESH
#define MESH_MASTER_MODE
#endif // USING_MESH

#define MASTER_NODE

SemaphoreHandle_t servantSemaphore = NULL;
SemaphoreHandle_t loraSemaphore = NULL;

#include "src/loraDrifterLibs/loraDrifter.h"

// GLOBAL VARIABLES
AsyncWebServer server(80);            // Create AsyncWebServer object on port 80
TinyGPSPlus gps;                      // decoder for GPS stream
#define SSID     "DrifterMaster"      // Wifi ssid and password
#define PASSWORD "Tracker1"
Master m;                             // Master data
Servant s[NUM_MAX_SERVANTS];          // Servants data array
String masterData = "";               // Strings for tabular data output to web page
String servantsData = "";
String diagnosticData = "";
String csvOutStr = "";                // Buffer for output file
String lastFileWrite = "";
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
//int ledState = LOW;
//int ledPin = 14;
int gpsLastSecond = -1;

#ifdef USING_MESH
byte routingTable[ROUTING_TABLE_SIZE] = "";
byte payload[24] = "";
byte localAddress = 0xAA;
byte localNextHopID = 0xAA;
byte localHopCount = 0x00;
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
#endif // USING_MESH

static String processor(const String& var) {
  if(var == "SERVANTS") {    return servantsData; }
  else if(var == "MASTER") { return masterData; }
#ifdef USING_MESH
  else if(var == "DIAGNOSTICS") {
    String diagnosticString =
      R"rawliteral(
      <br><br>
      <h4>Diagnostics</h4>
      <table>
        <tr>
          <td><b>Sent</b></td>
          <td><b>Rcvd</b></td>
        )rawliteral";
    if(xSemaphoreTake(servantSemaphore, portMAX_DELAY) == pdPASS) {
      for(int ii = 0; ii < NUM_MAX_SERVANTS; ii++) {
        if(s[ii].active) {
          diagnosticString += "<td><b>D" + String(ii) + "</b></td>";
        }
      }
    }
    xSemaphoreGive(servantSemaphore);
    return diagnosticString += "</tr>" + diagnosticData;
  }
#endif // USING_MESH
  else {
    return String();
  }
}

// Currently using factory only LoRa parameters
static void initLoRa() {
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(1); // if failed, do nothing
  }
#ifndef USING_MESH
  LoRa.onReceive(onReceive); // register the receive callback
  LoRa.receive(); // put the radio into receive mode
  delay(50);
#endif // USING_MESH
  LoRa.enableCrc(); // if packet is corrupted, packet gets dropped silently
}

static void writeData2Flash() {
  file = SPIFFS.open("/master.csv", FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
    ESP.restart();
  } else {
    if(file.println(csvOutStr)) {
      csvOutStr = "";
      nSamples = 0;
      Serial.print("Wrote data in file, current size: ");
      Serial.println(file.size());
      lastFileWrite = String(m.hour, DEC) + ":" + String(m.minute, DEC) + ":" + String(m.second, DEC);
    } else {
      lastFileWrite = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

#ifndef USING_MESH
static void onReceive(const int packetsize) {
  // received a packet
  uint8_t buffer[sizeof(Packet)];
  for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
    buffer[ii] = LoRa.read();
  }
  Packet packet;
  memcpy(&packet, buffer, sizeof(Packet));
  // Get ID and then send to class for decoding
  const String name = String(packet.name);
  if(!strcmp(name.substring(0, 1).c_str(), "D")) {
    Serial.println("Drifter signal found!");
    // csvOutStr += recv; // Save all packets recevied (debugging purposes)
    const int id = name.substring(1, 3).toInt();
    if(xSemaphoreTake(servantSemaphore, portMAX_DELAY) == pdPASS) {
      s[id].ID = id;
      s[id].decode(packet);
      s[id].rssi = LoRa.packetRssi();
      s[id].updateDistBear(m.lng, m.lat);
      s[id].active = true;
    }
    xSemaphoreGive(servantSemaphore);
    Serial.println("RX from LoRa - decoding completed");
  }
  delay(50);
}
#endif // USING_MESH

static void initWebServer() {
  WiFi.softAP(SSID, PASSWORD);
  Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

  // F. Web Server Callbacks setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/getMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    writeData2Flash();
    request->send(SPIFFS, "/master.csv", "text/html", true);
  });

  server.on("/deleteMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    SPIFFS.remove("/master.csv");
    file = SPIFFS.open("/master.csv", FILE_WRITE);
    if(!file) {
      Serial.println("There was an error opening the file for writing");
      return;
    }
    if(file.println("#FILE ERASED at " + String(m.hour, DEC) + ":" + String(m.minute, DEC) + ":" + String(m.second, DEC))) {
      Serial.println("File was erased / reinit OK");
    } else {
      Serial.println("File reinit failed");
    }
    file.close();
    lastFileWrite="";
    request->send(200, "text/html", "<html><span>Successfully deleted master!</span><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Back</a></html>");
  });
#ifdef USING_MESH
  server.on("/restartDrifter", HTTP_GET, [](AsyncWebServerRequest * request) {
    int paramsNr = request->params();
    for(int ii = 0; ii < paramsNr; ii++) {
      AsyncWebParameter* p = request->getParam(ii);
      if(p->name() == "drifterID") {
        const int drifterID = (p->value()).toInt();
        const byte drifterIDByte = indexToId(drifterID);
        sendFrame(MASTER_MODE, Restart, localAddress, drifterIDByte, localAddress, 0x0F);
      }
    }
    request->send(200, "text/html", "<html><span>Sent restart packet!</span><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Back</a></html>");
  });
#endif // USING_MESH
  server.begin();
}

void setup() {
  initBoard();
  delay(500);
  initLoRa();
  delay(50);
  initWebServer();
  delay(50);
  servantSemaphore = xSemaphoreCreateMutex();
  loraSemaphore = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(listenTask, "listenTask", 10000, NULL, 1, NULL, 1);
  delay(500);
  xTaskCreatePinnedToCore(sendTask, "sendTask", 10000, NULL, 2, NULL, 0);
  delay(500);
  // G. SPIFFS to write data to onboard Flash
  if(!SPIFFS.begin(true)) {
    // TODO: add retry
    Serial.println("An Error has occurred while mounting SPIFFS");
    while(1);
  }
  delay(50);
  Serial.println("Initialization complete.");
}

static void listenTask(void * pvParameters) {
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
    if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
      const int result = listener(LoRa.parsePacket(), MASTER_MODE);
    }
    xSemaphoreGive(loraSemaphore);
  }
}

static void sendTask(void * pvParameters) {
  while(1) {
    generateMaster();
#ifdef USING_MESH
    // if(gps.time.second() == RS_BCAST_TIME) {
    if(loop_runEvery(RS_BCAST_TIME)) { // TODO: delete this for field usage
      if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
        Serial.println("Route broadcast");
        bcastRoutingStatus(MASTER_MODE);
      }
      xSemaphoreGive(loraSemaphore);
#endif // USING_MESH
    }
    servantsData = R"rawliteral(
      <br><br>
      <h4>Servants</h4>
      <table>
        <tr>
          <td><b>ID</b></td>
          <td><b>Lora Update Plan [s]</b></td>
          <td><b>Last Update [s]</b></td>
          <td><b>Time</b></td>
          <td><b>Lon</b></td>
          <td><b>Lat</b></td>
          <td><b>Dist [m]</b></td>
          <td><b>Bearing [degN to]</b></td>
          <td><b>Count</b></td>
          <td><b>RSSI</b></td>
    )rawliteral";
#ifdef USING_MESH
    servantsData += "<td><b>Restart</b></td>";
#endif // USING_MESH
    servantsData += "</tr>";
    if(xSemaphoreTake(servantSemaphore, portMAX_DELAY) == pdPASS) {
      String tempClassColour = "";
      for(int ii = 0; ii < NUM_MAX_SERVANTS; ii++) {
        const uint32_t lastUpdate = (millis() - s[ii].lastUpdateMasterTime) / 1000;
        if(lastUpdate > 120) {
          tempClassColour = R"rawliteral(<td style="background-color:Crimson">)rawliteral";
        }
        else if(lastUpdate > 60 && lastUpdate <= 119) {
          tempClassColour = R"rawliteral(<td style="background-color:DarkOrange">)rawliteral";
        }
        else {
          tempClassColour = R"rawliteral(<td style="background-color:White">)rawliteral";
        }
        if(s[ii].active) {
          servantsData += "<tr>";
          servantsData += "<td><b>" + String(s[ii].ID) + "</b></td>";
          servantsData += "<td>" + String(s[ii].drifterTimeSlotSec) + "</td>";
          servantsData += tempClassColour + String(lastUpdate) + "</td>";
          servantsData += "<td>" + String(s[ii].hour) + ":" + String(s[ii].minute) + ":" + String(s[ii].second) + "</td>";
          servantsData += "<td>" + String(s[ii].lng, 6) + "</td>";
          servantsData += "<td>" + String(s[ii].lat, 6) + "</td>";
          servantsData += "<td>" + String(s[ii].dist) + "</td>";
          servantsData += "<td>" + String(s[ii].bear) + "</td>";
          servantsData += "<td>" + String(s[ii].nSamples) + "</td>";
          servantsData += "<td>" + String(s[ii].rssi) + "</td>";
#ifdef USING_MESH
          servantsData += R"rawliteral(
            <td><form action="/restartDrifter" method="get"><button type="submit" name="drifterID" value=
          )rawliteral";
          servantsData += String(s[ii].ID) + ">Restart</button></form></td>";
#endif // USING_MESH
          servantsData += "</tr></table>";
        }
      }
    }
    xSemaphoreGive(servantSemaphore);
#ifdef USING_MESH
    diagnosticData = "<tr>";
    diagnosticData += "<td>" + String(messagesSent) + "</td>";
    diagnosticData += "<td>" + String(messagesReceived) + "</td>";
    if(xSemaphoreTake(servantSemaphore, portMAX_DELAY) == pdPASS) {
      for(int ii = 0; ii < NUM_MAX_SERVANTS; ii++) {
        if(s[ii].active) {
          diagnosticData += "<td>" + String(getNodeRxCounter(ii)) + "</td>";
        }
      }
    }
    xSemaphoreGive(servantSemaphore);
    diagnosticData += "</tr>";
#endif // USING_MESH
    // D. Write data to onboard flash
    if(nSamples > SAMPLES_BEFORE_WRITE) {  // only write after collecting a good number of samples
      writeData2Flash();
    }
  }
}

static void fill_master() {
  m.lng = gps.location.lng();
  m.lat = gps.location.lat();
  // TODO: Need to add 8 hours onto gps time
  m.year = gps.date.year();
  m.month = gps.date.month();
  m.day = gps.date.day();
  m.hour = gps.time.hour();
  m.minute = gps.time.minute();
  m.second = gps.time.second();
  m.age = gps.location.age();
}

static void generateMaster() {
  // Read GPS and run decoder
  const uint32_t start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 500);

  if(gps.time.second() != gpsLastSecond) {
    fill_master();
    const String tDate = String(m.year) + "-" + String(m.month) + "-" + String(m.day);
    const String tTime = String(m.hour) + ":" + String(m.minute) + ":" + String(m.second);
    masterData =  "<tr><td>" + tDate + " " + tTime + "</td><td>" + String(m.lng, 6) + "</td><td>" + String(m.lat, 6) + "</td><td>" + String(m.age) + "</td>";
    masterData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getMaster\"> GET </a></td>";
    masterData += "<td>" + lastFileWrite + "</td>";
    masterData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteMaster\"> ERASE </a></td>";
    masterData += "</tr>";
    // Update String to be written to file
    if((m.lng != 0.0) && (m.age < 1000)) {
      csvOutStr += tDate + "," + tTime + "," + String(m.lng, 6) + "," + String(m.lat, 6) + "," + String(m.age) + "\n";
      nSamples++;
    } else {
      Serial.println("No GPS fix, not writing local data!");
    }
    gpsLastSecond = gps.time.second();
  }
}

// Loop does nothing as the loop functions are split into RTOS tasks
void loop() {}
