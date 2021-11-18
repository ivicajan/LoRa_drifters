#define MASTER_MODE
#define USING_MESH

#ifdef USING_MESH
#define MESH_MASTER_MODE
#endif // USING_MESH

#include "src/loraDrifterLibs/loraDrifter.h"

// F. Functions
void startWebServer(const bool webServerOn);
String IpAddress2String(const IPAddress& ipAddress);

// GLOBAL VARIABLES
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
AsyncWebServer server(80);            // Create AsyncWebServer object on port 80
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
//int ledState = LOW;
//int ledPin = 14;
int gpsLastSecond = -1;
int webServerPin = BUTTON_PIN;
String hour, minute, second, year, month, day, tTime, tDate;

byte routingTable[0x99] = "";
byte payload[24] = "";
byte localAddress = 0xAA;
byte localNextHopID = 0xAA;
byte localHopCount = 0x00;

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

String processor(const String& var) {
  if(var == "SERVANTS") {    return servantsData;  }
  if(var == "MASTER") {      return masterData;  }
  if(var == "DIAGNOSTICS") { return diagnosticData; }
  return String();
}

void onReceive(const int packetsize) {
  // received a packet
  Serial.println("Received packet:");
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
    s[id].ID = id;
    s[id].decode(packet);
    s[id].rssi = LoRa.packetRssi();
    s[id].updateDistBear(m.lng, m.lat);
    s[id].active = true;
    Serial.println("RX from LoRa - decoding completed");
  }
  delay(50);
}

// FUNCTION DEFINITIONS
void setup(){
  initBoard();
  delay(500);

  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);

  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(1);                       // if failed, do nothing
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
    request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
  });
  server.begin();
  delay(50);

  // G. SPIFFS to write data to onboard Flash
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS - need to add retry");
    while(1);
  }
  delay(50);
  Serial.println("Initialization complete.");
}

void generateMaster() {
  // Read GPS and run decoder
  unsigned long start = millis();
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
    masterData =  "<tr><td>" + tDate + " " + tTime + "</td><td>" + String(m.lng, 8) + "</td><td>" + String(m.lat, 8) + "</td><td>" + String(m.age) + "</td>";
    masterData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getMaster\"> GET </a></td>";
    masterData += "<td>" + lastFileWrite + "</td>";
    masterData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteMaster\"> ERASE </a></td>";
    masterData += "</tr>";
    // Update String to be written to file
    if((m.lng != 0.0) && (m.age < 1000)) {
      csvOutStr += tDate + "," + tTime + "," + String(m.lng, 8) + "," + String(m.lat, 8) + "," + String(m.age) + "\n";
      nSamples += 1;
    } else {
      Serial.println(" NO GPS FIX, not WRITING LOCAL DATA !");
    }
    gpsLastSecond = gps.time.second();
    // Serial.println("nSamples: " + String(nSamples));
  }
}

void loop() {
#ifdef USING_MESH
  const int result = daemon(1); // MESH_MASTER_MODE
#endif // USING_MESH
  generateMaster();

  servantsData = "";
  for(int ii = 0; ii < nServantsMax; ii++) {
    if(s[ii].active) {
      servantsData += "<tr>";
      servantsData += "<td>" + String(s[ii].ID) + "</td>";
      servantsData += "<td>" + String(s[ii].drifterTimeSlotSec) + "</td>";
      servantsData += "<td>" + String((millis() - s[ii].lastUpdateMasterTime) / 1000) + "</td>";
      servantsData += "<td>" + String(s[ii].hour) + ":" + String(s[ii].minute) + ":" + String(s[ii].second) + "</td>";
      servantsData += "<td>" + String(s[ii].lng, 8) + "</td>";
      servantsData += "<td>" + String(s[ii].lat, 8) + "</td>";
      servantsData += "<td>" + String(s[ii].dist) + "</td>";
      servantsData += "<td>" + String(s[ii].bear) + "</td>";
      servantsData += "<td>" + String(s[ii].nSamples) + "</td>";
      servantsData += "<td>" + String(s[ii].rssi) + "</td>";
      servantsData += "</tr>";
    }
  }
  diagnosticData = "";
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
  diagnosticData += "</tr>";
  // D. Write data to onboard flash
  if(nSamples > nSamplesFileWrite) {  // only write after collecting a good number of samples
    writeData2Flash();
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
