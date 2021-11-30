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
const char* ssid = "DrifterMaster";   // Wifi ssid and password
const char* password = "Tracker1";
Master m;                             // Master data
Servant s[nServantsMax];              // Servants data array
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
const int webServerPin = BUTTON_PIN;
String hour, minute, second, year, month, day, tTime, tDate;

#ifdef USING_MESH
byte routingTable[0x99] = "";
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

TaskHandle_t ListenTask;
TaskHandle_t SendTask;

String processor(const String& var) {
  if(var == "SERVANTS") {    return servantsData;  }
  else if(var == "MASTER") {      return masterData;  }
#ifdef USING_MESH
  else if(var == "DIAGNOSTICS") {
    return R"rawliteral(
    <br><br>
    <h4>Diagnostics</h4>
    <table>
      <tr>
        <td><b>Sent</b></td>
        <td><b>Rcvd</b></td>
        <td><b>D01</b></td>
        <td><b>D02</b></td>
        <td><b>D03</b></td>
        <td><b>D04</b></td>
        <td><b>D05</b></td>
        <td><b>D06</b></td>
        <td><b>D07</b></td>
      </tr>)rawliteral"
      + diagnosticData; }
#endif // USING_MESH
  else {
    return String();
  }
}

void writeData2Flash() {
  file = SPIFFS.open("/master.csv", FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
    ESP.restart();
  } else {
    if(file.println(csvOutStr)) {
      csvOutStr = "";
      nSamples = 0;
      Serial.println("Wrote data in file, current size: ");
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
void onReceive(const int packetsize) {
  // received a packet
  uint8_t buffer[sizeof(Packet)];
  for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
    buffer[ii] = LoRa.read();
  }
  Packet * packet;
  memset(&packet, 0, sizeof(packet));
  packet = (Packet *)buffer;
  // Get ID and then send to class for decoding
  const String name = String(packet->name);
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

// FUNCTION DEFINITIONS
void setup(){
  initBoard();
  delay(500);

  servantSemaphore = xSemaphoreCreateMutex();
  loraSemaphore = xSemaphoreCreateMutex();

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);

  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(1); // if failed, do nothing
  }

#ifndef USING_MESH
   // register the receive callback
  LoRa.onReceive(onReceive);
  // put the radio into receive mode
  LoRa.receive();
  delay(50);
#endif // USING_MESH

  // Enable CRC --> if packet is corrupted, packet gets dropped silently
  LoRa.enableCrc();

  WiFi.softAP(ssid, password);
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
  delay(50);

  //create a task that will be executed in the listenTask() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    listenTask,       /* Task function. */
                    "listenTask",     /* name of task. */
                    10000,            /* Stack size of task */
                    NULL,             /* parameter of the task */
                    1,                /* priority of the task */
                    &ListenTask,      /* Task handle to keep track of created task */
                    1);               /* pin task to core 1 */
  delay(500); 

  //create a task that will be executed in the sendTask() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    sendTask,        /* Task function. */
                    "sendTask",      /* name of task. */
                    10000,           /* Stack size of task */
                    NULL,            /* parameter of the task */
                    2,               /* priority of the task */
                    &SendTask,       /* Task handle to keep track of created task */
                    0);              /* pin task to core 0 */
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

void listenTask(void * pvParameters) {
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
    if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
      const int result = listener(LoRa.parsePacket(), MASTER_MODE);
    }
    xSemaphoreGive(loraSemaphore);
  }
}

void sendTask(void * pvParameters) {
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
      for(int ii = 0; ii < nServantsMax; ii++) {
        if(s[ii].active) {
          servantsData += "<tr>";
          servantsData += "<td>" + String(s[ii].ID) + "</td>";
          servantsData += "<td>" + String(s[ii].drifterTimeSlotSec) + "</td>";
          servantsData += "<td>" + String((millis() - s[ii].lastUpdateMasterTime) / 1000) + "</td>";
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
    diagnosticData = "";
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
    diagnosticData += "</tr>";
#endif // USING_MESH
    // D. Write data to onboard flash
    if(nSamples > SAMPLES_BEFORE_WRITE) {  // only write after collecting a good number of samples
      writeData2Flash();
    }
  }
}

void generateMaster() {
  // Read GPS and run decoder
  const unsigned long start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 500);

  if(gps.time.second() != gpsLastSecond) {
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
      nSamples += 1;
    } else {
      Serial.println(" NO GPS FIX, not WRITING LOCAL DATA !");
    }
    gpsLastSecond = gps.time.second();
  }
}

// Loop does nothing as the loop functions are split into tasks
void loop() {}
