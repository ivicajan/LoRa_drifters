/*
All Nodes are filling their Routing Tables and Routing Statuses dynamically. 

Node 1: 
byte localAddress = 0x11;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Node 2: 
byte localAddress = 0x22;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Node 3: 
byte localAddress = 0x33;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Node 4:
byte localAddress = 0x44;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Node 5:
byte localAddress = 0x55;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Node 6:
byte localAddress = 0x66;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Node 7:
byte localAddress = 0x77;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

Master: 
byte localAddress = 0xAA;
byte localNextHopID = 0xAA;
byte localHopCount = 0x00;

*/
#include "loraDrifterMaster.h"

// F. Functions
void startWebServer(const bool webServerOn);
String IpAddress2String(const IPAddress& ipAddress);

// Timing Parameters
#define RS_BCAST_TIME 6000
#define PL_TX_TIME 12000
#define DELETION_TIME 62000
#define ARQ_TIME 2000

//LoRa Frequency
const long frequency = 915E6; 

// GLOBAL VARIABLES
TinyGPSPlus gps;                      // decoder for GPS stream
const char* ssid = "DrifterMaster";   // Wifi ssid and password
const char* password = "Tracker1";
Master m;                             // Master data
Servant s[nServantsMax];              // Servants data array
String masterData = "";               // Strings for tabular data output to web page
String servantsData = "";
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

int masterMode = 1;     
byte routingTable[153] = "";
byte payload[24] = "";
byte localAddress = 0xAA;
byte localNextHopID = 0x00;
byte localHopCount = 0x00;

// DIAGNOSTICS
int node1Rx = 0;
int node2Rx = 0;
int node3Rx = 0;
int node4Rx = 0;
int node5Rx = 0;
int node6Rx = 0;
int node7Rx = 0;

// FUNCTION DEFINITIONS
void setup(){
  initBoard();
  delay(50);
  
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

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
    while (1);
  }
  delay(50);
  Serial.println("Initialization complete.");  
}

void loop(){
  int result = daemon(masterMode);
  //printNodeInfo();

  generate_master(Serial1, gps);

  servantsData = "";
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
      servantsData += "</tr>";
    }
  }

  // D. Write data to onboard flash
  if (nSamples > nSamplesFileWrite) {  // only write after collecting a good number of samples
    writeData2Flash();
  }
  
}

void generate_master(Stream &mySerial, TinyGPSPlus &myGPS) {
  // Read GPS and run decoder
  unsigned long start = millis();
  do {
    while(mySerial.available() > 0) {
      myGPS.encode(mySerial.read());
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
      csvOutStr += tDate + "," + tTime + "," + String(m.lng, 8) + "," + String(m.lat, 8) + "," + String(m.age) + "\n";
      #ifdef DEBUG_MODE
      Serial.print("csvOutStr: ");
      Serial.println(csvOutStr);
      #endif
      nSamples += 1;
    } else {
      Serial.println(" NO GPS FIX, not WRITING LOCAL DATA !");
    }
    gpsLastSecond = gps.time.second();
    // Serial.println("nSamples: " + String(nSamples));
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

String processor(const String& var) {
  if (var == "SERVANTS") {  return servantsData;  }
  if (var == "MASTER") {    return masterData;  }
  return String();
}

int getNodeRxCounter(byte nodeId){
  int res = 0;
  switch(nodeId){
      case 0x11:
        res = node1Rx;
        break;

      case 0x22:
        res = node2Rx;
        break;

      case 0x33:
        res = node3Rx;
        break;

      case 0x44:
        res = node4Rx;
        break;
        
      case 0x55:
        res = node5Rx;
        break;

      case 0x66:
        res = node6Rx;
        break;

      case 0x77:
        res = node7Rx;
        break;
  }
  return res;
}

void incNodeRxCounter(int nodeId){
  switch(nodeId){
    case 17:
      node1Rx++;
      break;

    case 34:
      node2Rx++;
      break;

    case 51:
      node3Rx++;
      break;  

    case 68:
      node4Rx++;
      break; 

    case 85:
      node5Rx++;
      break; 

    case 102:
      node6Rx++;
      break;

    case 119:
      node7Rx++;
      break;
    
  }
  
}
/*
void printNodeInfo(){
  int nodeId = *(int *) (&payload[0]); 
  int hopCount = *(int *) (&payload[4]);
  int nextHop = *(int *) (&payload[8]);
  int linkRssi = *(int *) (&payload[12]);
  int attemptedPayloadTx = *(int *) (&payload[16]);
  incNodeRxCounter(nodeId);
  int nodeRx = getNodeRxCounter(nodeId);
  
  Serial.print("    Received payload from Node ID 0x");
  Serial.print(nodeId, HEX);
  Serial.println(":");

  Serial.print("        hopCount:   ");
  Serial.println(hopCount);

  Serial.print("        nextHop:    0x");
  Serial.println(nextHop, HEX);

  Serial.print("        linkRssi:   ");
  Serial.println(linkRssi);

  Serial.print("        Attempted Payload Transmissions:                   ");
  Serial.println(attemptedPayloadTx);

  
  Serial.print("        Total Payloads Received from this Node:            "); 
  Serial.println(nodeRx);
}
*/
bool validateID(byte nodeID){
  bool valid = false;
  switch(nodeID){
    case 0x11:        // Node 1
      valid = true;
      break;
    
    case 0x22:        // Node 2
      valid = true;
      break;
    
    case 0x33:        // Node 3
      valid = true;
      break;
    
    case 0x44:        // Node 4
      valid = true;
      break;
      
    case 0x55:        // Node 5
      valid = true; 
      break;
      
    case 0x66:        // Node 6
      valid = true; 
      break;
      
    case 0x77:        // Node 7
      valid = true;
      break;
      
    case 0xAA:        // Master Node
      valid = true; 
      break;
    
    case 0xFF:        // BCAST
      valid = true;
      break;
    
    default:        // Invalid ID
      valid = false;
      break;
  }
  return valid;
}

int idToIndex(byte nodeID){
  int index = 0;
  switch(nodeID){
    case 0xAA:
      index = 0;
      break;
      
    case 0x11:
      index = 1;
      break;
      
    case 0x22:
      index = 2;
      break;
      
    case 0x33:
      index = 3;
      break;
      
    case 0x44:
      index = 4;
      break;
      
    case 0x55:
      index = 5;
      break;
      
    case 0x66:
      index = 6;
      break;
      
    case 0x77:
      index = 7;  
      break;
    
  default:
    index = 0;
    break; 
  }
  return index;
}

int insertRoutingTable(
  byte nodeID,
  byte hopCount,
  byte hopID,
  int Rssi,
  float snr,
  unsigned long currentTime
){

  bool validNode = validateID(nodeID);
  bool validHop = validateID(hopID);
  
  if(validNode && validHop){
    int i = idToIndex(nodeID);
  
    memcpy(&routingTable[i*19], &nodeID, sizeof(nodeID));
    memcpy(&routingTable[(i*19)+1], &hopCount, sizeof(hopCount));
    memcpy(&routingTable[(i*19)+2], &hopID, sizeof(hopID));
    memcpy(&routingTable[(i*19)+3], &Rssi, sizeof(Rssi));
    memcpy(&routingTable[(i*19)+7], &snr, sizeof(snr));
    memcpy(&routingTable[(i*19)+11], &currentTime, sizeof(currentTime));  
    
    return 1;       // Success
  }
  return -9;          // E -9: invalid nodeID
}

void printRoutingTable(){
  for(int i=0; i<8; i++){
    byte nodeID = routingTable[i*19];
    byte hopCount = routingTable[(i*19)+1];
    byte hopID = routingTable[(i*19)+2];
    int Rssi = *(int*)(&routingTable[(i*19)+3]);
    float snr = *(float*)(&routingTable[(i*19)+7]);
    unsigned long currentTime = *(unsigned long*)(&routingTable[(i*19)+11]);

    Serial.print("Routing Table Entry ");
    Serial.print(i);
    Serial.println(": ");
    
    Serial.print("nodeID: 0x");
    Serial.println(nodeID, HEX);
    
    Serial.print("hopCount: ");
    Serial.println(hopCount);
    
    Serial.print("hopID: 0x");
    Serial.println(hopID,HEX);
    
    Serial.print("rssi: ");
    Serial.println(Rssi);
    
    Serial.print("snr: ");
    Serial.println(snr);
    
    Serial.print("time: ");
    Serial.println(currentTime);
    Serial.println(" ");
  }
}

boolean runEvery(unsigned long interval){
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void deleteOldEntries(){
  // Reset when entry's time is older than DELETION_TIME
  long int currentTime = millis(); 
  long int lastTime = 0;
  int timeIndex = 0;
  int difference = 0;
  int newIndex = 0;
  
  for(int i=0; i<8; i++){
    timeIndex = (i*19) + 11;
    
    lastTime = *(long int*)(&routingTable[timeIndex]);
    difference = currentTime - lastTime;

    // Sets all 19 fields of an entry to 0x00
    if(difference > DELETION_TIME){
      for (int j=0; j<18; j++){
        newIndex = (i*19) + j;
        routingTable[newIndex] = 0x00;
      }
    }
  }
}

bool checkIfEmpty(){
  
  for(int i=0; i<8; i++){
    byte entry = routingTable[i*19];
    if (entry != 0x00){
      return false; 
    }
  }
  return true;
}

bool searchMaster() {
  //Checks if a nodeID matches the Master ID 0xAA.
  for(int i=0; i<8; i++){
    byte entry = routingTable[i*19];
    if(entry == 0xAA){
      return true;
    }
  }
  return false;
}

int findMinHopCount(){
  int minHopCount = 255; 
  int currentHopCount = 0;
  
  for(int i=0; i<8; i++){
    byte hopCount = routingTable[(i*19)+1]; 
    currentHopCount = (int) hopCount;
    if((currentHopCount != 0) && (currentHopCount < minHopCount)){
      minHopCount = currentHopCount;
    }
  }
  return minHopCount;
}

int findMaxRssi(int minHopCount){
  //To make sure the nextHop of that entry is not local
  
  int currentRssi = 0; 
  int maxRssi = -10000000;
  byte bestRoute = 0x00;
  
  for(int i=0; i<8; i++){
    byte hopCount = routingTable[(i * 19) + 1];
    byte nextHopID = routingTable[(i * 19) +2];
  
    // maintain the currentRssi as maxRssi.
    
    if((hopCount == minHopCount) && (nextHopID != localAddress)){
      currentRssi = *(int *)(&routingTable[(i*19)+3]);
      if (currentRssi > maxRssi){
        maxRssi = currentRssi;
        bestRoute = routingTable[i * 19];         
      } 
    } 
  }
  return (int)bestRoute;
}

int setRoutingStatus(){
  // Update localHopCount and localNextHop

  deleteOldEntries();
  bool isEmpty = checkIfEmpty();
  if(!isEmpty){
    bool masterFound = searchMaster();
    if(!masterFound){
      int minHopCountInt = findMinHopCount();
      int bestRouteInt = findMaxRssi(minHopCountInt);
      
      byte minHopCount = (byte)minHopCountInt;
      byte bestRoute = (byte)bestRouteInt;
      if(minHopCount != 0x00 && bestRoute !=0x00){
        // valid route
        localHopCount = minHopCount + 1;
        localNextHopID = bestRoute; 
        return 1;
      } else {
        // invalid
        return -1;    
      }
    } else {
      // master is inside the routing table
      localNextHopID = 0xAA;
      localHopCount = 0x01;
      return 1;
    } 
  } else {
    // empty table
    localNextHopID = 0x00;
    localHopCount = 0x00;
    return -1;  
  }
}

bool checkFrameHeader(
  int mode,
  byte sizeHeader, 
  byte type, 
  byte router, 
  byte source, 
  byte recipient, 
  byte sender, 
  byte ttl, 
  byte sizePayload
){
  // Check if header values are valid
  if(sizeHeader!= 0x08){
    // Serial.println("checkFrameHeader: invalid sizeHeader");
    return false;
  }
  if(type < 0x41 || type > 0x45){
    // Serial.println("checkFrameHeader: invalid type");
    return false; 
  }
  if(!validateID(router)){
    // Serial.println("checkFrameHeader: invalid router");
    return false;
  }
  if(!validateID(source)){
    // Serial.println("checkFrameHeader: invalid source");
    return false;
  }
  if(!validateID(recipient)){
    // Serial.println("checkFrameHeader: invalid recipient");
    return false;
  }
  if(!validateID(sender)){
    // Serial.println("checkFrameHeader: invalid sender");
    return false;
  }
  if(ttl > 0x0F || ttl == 0x00){
    // Serial.println("checkFrameHeader: invalid ttl");
    return false;
  }
  if(sizePayload!=0x02 && sizePayload!=0x18 && sizePayload!=0x00){
    // Serial.println("checkFrameHeader: invalid sizePayload");
    return false; 
  }

  // type and router ID
  if(mode == 0){              // Node Mode
    if(type==0x43 || type==0x45){   // Type D OR Type E    
      // Serial.println("checkFrameHeader: invalid type for Node Mode");   
      return false;
    }
    if(type==0x41 && sender!=0xAA){
       // Serial.println("checkFrameHeader: Invalid Type && sender ID"); 
       return false;
     }
    if(router!=localAddress && router!=0xFF){     
        // Serial.println("checkFrameHeader: Not addressed to local"); 
        return false;
    }
    return true;
  }
  
  if(mode == 1){              // Master Mode
    if(type != 0x43){         // Type C: Direct Master
      return false;
    }
    if(router != localAddress){     
      return false;
    }
    return true;
  }
  
  if(mode>16 && mode<171){        // ACK Mode
    if(type != 0x45){         // Type E: ACK  
      return false;
    }
    if(router != localAddress){
      return false;
    }
    return true;
  }
  return false;
}

int frameHandler(
  int mode, 
  byte type,
  byte router,
  byte source, 
  byte recipient, 
  byte sender, 
  byte ttl
){
  //Process data frame
  
  int result = 0;

  if(mode == 0){              // Node Mode
    if(type == 0x41){         // Type A: Master BCAST
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();
      unsigned long time = millis();
      result = insertRoutingTable(
        sender,           // nodeID
        0x01,             // hopCount
        0xAA,           // nextHopID
        rssi,             // RSSI
        snr,            // SNR
        time            // time 
      );
      if(result != 1){
        return result;
      }
      result = setRoutingStatus();            
      return result;          // 1 or -1
    }
    
    if(type == 0x42){         // Type B: Neighbor BCAST
      byte hopCount = LoRa.read();  // Parsing Payload
      byte nextHopID = LoRa.read();   
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();
      unsigned long time = millis();
      
      result = insertRoutingTable(
        sender,           // nodeID
        hopCount,           // hopCount
        nextHopID,          // nextHopID
        rssi,             // RSSI
        snr,            // SNR
        time            // time 
      );
      if(result != 1){
        return result;        // invalid nodeID
      }
      result = setRoutingStatus();            
      return result;          // 1 or -1
    }
    
    if(type == 0x44){         // Type D: Route Request
      parsePayload();
      result = routePayload(
        mode,
        recipient,
        sender,
        ttl,
        2    // don't resend when no ack
      );
      
      if(result == 1){
        sendAckBack(mode, source);
        return result;          // Success
      }else{
        return result;          // No ACK
      }
    }
    return -5;                // Node Mode ERR
  } 
  
  if(mode == 1){                // Master Mode
    if(type == 0x43){           // Type C: Direct PL
      result = parsePayload();
      if(result == 2){
        sendAckBack(mode, source);
        return result;          // Payload Processed
      }
      return result;            // Parse ERR
    } 
    return -6;                // Master Mode ERR
  }
  
  // Checking the ackMode that was passed from waitForAck()
  // The address space in integers between 17 and 170. 
  // Node 1 - 0x11 - 17
  // Node 2 - 0x22 - 34
  // Master Node - 0xAA - 170
  
  if(mode>16 && mode<171){          // ACK Mode
    if(type == 0x45){     
      return 1;             // Success
    } 
    return -3;                // ACK Mode ERR
  }
  return -4;                  // frHandler ERR
}

int listener(int frameSize, int mode){
  if(frameSize == 0){
    return 0;             // nothing to receive
  }
  
  // Parse Header
  byte sizeHeader = LoRa.read();
  byte type = LoRa.read();
  byte router = LoRa.read();
  byte source = LoRa.read();
  byte recipient = LoRa.read();
  byte sender = LoRa.read();
  byte ttl = LoRa.read();
  byte sizePayload = LoRa.read();
  Serial.println(sizeHeader);
  Serial.println(type);
  Serial.println(router);
  Serial.println(source);
  Serial.println(recipient);
  Serial.println(sender);
  Serial.println(ttl);
  Serial.println(sizePayload);
  
  bool validHeader = checkFrameHeader(
    mode, 
    sizeHeader,
    type, 
    router, 
    source, 
    recipient, 
    sender,
    ttl,
    sizePayload
  );
  if(validHeader){
    int result = frameHandler(
      mode,
      type, 
      router, 
      source, 
      recipient, 
      sender, 
      ttl
    );
    return result;            // 1 or E (-6 to -1)
  }
  return 0;
}

int parsePayload(){
  // Parses the data
  
  if(LoRa.available() == sizeof(Packet)){
    uint8_t buffer[sizeof(Packet)];
    for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
      buffer[ii] = LoRa.read();
    }
    Packet * packet;
    memset(&packet, 0, sizeof(packet));
    packet = (Packet *)buffer; 
    // Get ID and then send to class for decoding
    String name = String(packet->name);
    //Serial.println(name.substring(0, 1).c_str());
    Serial.println("Packet name:");
    Serial.println(name);
    if(!strcmp(name.substring(0, 1).c_str(), "D")) {
      Serial.println("Drifer signal found!");
      // csvOutStr += recv; // Save all packets recevied (debugging purposes)
      int id = name.substring(1, 3).toInt();
      s[id].ID = id;
      s[id].decode(packet);
      s[id].rssi = LoRa.packetRssi();
      s[id].updateDistBear(m.lng, m.lat);
      s[id].active = true;
      Serial.println("RX from LoRa - decoding completed");
    }
    return 2;
  }
  return -2;                // Payload ERR
}

int routePayload(
  int mode, 
  byte recipient, 
  byte sender,
  byte ttl,
  int resend
){
  // Send the data based on routing status
  
  byte type = 0x00;

  // Check first if the localNextHopID is the Master ID
  if(recipient == localNextHopID){    
    type = 0x43;       // Type C: Direct Master PL
  } else {
    type = 0x44;       // Type D: Route Request
  }
  
  byte router = localNextHopID;
  
  int result = ackHandshake(
    mode,
    type,
    router,
    recipient, 
    sender,
    ttl,
    resend
  );
  
  if(result == 1){    // ACK received
    return result;
  } else {
    return -8;      // No ACK received
  }


}

int ackHandshake(
  int mode,
  byte type, 
  byte router, 
  byte recipient, 
  byte sender,
  byte ttl,
  int resend
){
  // If no ACK received, resent two more times.

  sendFrame(
    mode,
    type, 
    router, 
    recipient, 
    sender, 
    ttl
  );
  bool ack = waitForAck(router);
  
  while(!ack && resend<2){
    sendFrame(
      mode,
      type, 
      router, 
      recipient, 
      sender, 
      ttl
    );
    ack = waitForAck(router);
    resend++;
  }
  
  if (!ack){
    Serial.println("Did not receive ACK from router.");
    return 0;
  } else {
    return 1;
  }
}

bool waitForAck(byte router){
  // Wait for ACK
  
  int maxloops = 0;
  int result = 0;
  int interval = ARQ_TIME * localHopCount;
  int ackMode = (int)router;
  
  while(maxloops<interval && result!=1){
    result = listener(LoRa.parsePacket(), ackMode);
    delay(1);
    maxloops++;
  }
  
  if(result == 1){    // ACK received from router
    return true;    
  }
  return false;
}

void sendAckBack(int mode, byte source){
  
  // To send an ACK back to the source
  
  byte type = 0x45;           // Type E: ACK 
  byte router = source;     
  byte recipient = source;
  byte sender = localAddress;
  byte ttl = 0x0F;

  sendFrame(
    mode,
    type,
    router,
    recipient,
    sender, 
    ttl
  );
}

void sendFrame(
  int mode,
  byte type, 
  byte router, 
  byte recipient, 
  byte sender, 
  byte ttl  
){
  // Send a complete header with a random delay
  
  byte header[8] = ""; 
  header[0] = 0x08;       // sizeHeader
  header[1] = type;     // type
  header[2] = router;     // router 
  header[3] = localAddress;   // source
  header[4] = recipient;    // recipient
  header[5] = sender;     // sender
  header[6] = ttl - 1;    // ttl

  delay(random(20));
  
  if (mode == 0){               // Node Mode
    switch(header[1]){            // check type
    
      case 0x42:              // Type B: RS BCAST
        header[7] = 0x02;       // sizePayload
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write(localHopCount);    // RS payload
        LoRa.write(localNextHopID);   // RS payload 
        LoRa.endPacket(true);
        break;
      
      case 0x43:              // Type C: Direct PL
        header[7] = 0x18;       // sizePayload
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write((const uint8_t*)&m, sizeof(m)); // master packet
        LoRa.endPacket(true);
        break;
        
      case 0x44:              // Type D: RRequest
        header[7] = 0x18;
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write((const uint8_t*)&m, sizeof(m));
        LoRa.endPacket(true);
        break;
      
      case 0x45:              // Type E: ACK
        header[7] = 0x00;       // sizePayload      
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        break;
        
      default: 
        Serial.println("Not valid for this mode");
        break;
      }   
  } else if (mode == 1) {           // Master Mode
    switch(header[1]){            // check type
    
      case 0x41:              // Type A: RS BCAST
        Serial.println("RS BCAST");
        header[7] = 0x00;
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        break;
        
      case 0x45:              // Type E: ACK
        header[7] = 0x00;
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        break; 
      
      default:  
        Serial.println("Not valid for this mode");
        break;    
    }
  } else {
    Serial.println("Not a valid mode");
  }
  
}

int bcastRoutingStatus(int mode){
  // Send a broadcast to the network
  
  int result = 0;
  if(mode == 0){
    result = setRoutingStatus();
    if(result == -1){
      return result;  
    }
    sendFrame(
      mode,       
      0x42,             // type: Type B
      0xFF,             // router: BCAST
      0xFF,             // recipient: BCAST
      localAddress,     // sender
      0x0F               // ttl
    );
  } else if(mode == 1){   // Master Mode
    Serial.println("sendFrame");
    sendFrame(
      mode,       
      0x41,             // type: Type A
      0xFF,             // router: BCAST
      0xFF,             // recipient: BCAST
      localAddress,     // sender
      0x0F               // ttl
    );
  }
  return 1;
}

int daemon(unsigned int mode){
  int result = 0;
  
  if(runEvery(RS_BCAST_TIME)){
    Serial.println("RS_BCAST_TIME");
    result = bcastRoutingStatus(mode);   
    return result;                // returns 1 or -1
  }
  
  result = listener(LoRa.parsePacket(), mode);
  return result;                  // returns 0: Nothing
}                                 // relevant or valid
                                  //
                                  // returns 1: Valid 
                                  // processing
                                  //
                                  // returns -8 to -1:
                                  // Processing Errors
