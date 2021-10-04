//All Nodes fill the routing tables and status dynamically.
//Broadcast local routing status to the network and sent to the master

/*
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
#include "loraDrifter.h"

// F. Functions
void startWebServer(const bool webServerOn);
String IpAddress2String(const IPAddress& ipAddress);

// Timing Parameters
#define RS_BCAST_TIME 6000    //Time intervals, broadcast for every 6000ms
#define PL_TX_TIME 12000      //Receive pay load for every ms
#define DELETION_TIME 62000   //Reset the routing table if entry's time is older than 62000ms
#define ARQ_TIME 2000         //Automatic Repeat Request for every 2000ms

//LoRa Frequency
const long frequency = 915E6; 

// GLOBAL VARIABLES
TinyGPSPlus gps;
const String drifterName = "D01";   // ID send with packet
int drifterTimeSlotSec = 5; // seconds after start of each GPS minute
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

int servantMode = 0;     
byte routingTable[153] = "";
Packet packet;
byte localAddress = 0x11; // Change local address for different nodes.
byte localNextHopID = 0x00;
byte localHopCount = 0x00;
int localLinkRssi = 0;

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
            <td><label for="lname">LoRa Sending Second:</label> </td>
            <td>%LORASENDSEC%</td>
            <td><input type="text" id="lname" name="loraSendSec"></td>
            <td>Sending second is from 0 to 59 seconds</td>
          </tr>
        </table>
        <input type="submit" value="Configure">
      </form>
    </body>
  </html>
)rawliteral";

// FUNCTION DEFINITIONS
void setup(){
  initBoard();
  delay(50);

  pinMode(webServerPin, INPUT);
  
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
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
  
  Serial.println("Initialization complete.");  
}

void loop(){
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
    int result = daemon(servantMode);

    if(loop_runEvery(PL_TX_TIME)){ 
      generate_packet(Serial1, gps);
      result = routePayload(
        servantMode,           // Node Mode
        0xAA,               // recipient: Master
        localAddress,       // sender
        0x0F,               // ttl
        0                   // set resend counter
      );
      if(result != 0){
        // Serial.println(result);
      }
    }
    
    // B. Write data to onboard flash if nSamples is large enough
    if(nSamples >= nSamplesFileWrite) {  // only write after collecting a good number of samples
      Serial.println("Dump data into the memory");
      writeData2Flash();
    }
  } else {
    Serial.println("Web server is ON, not GPS data or saving during the time");
    delay(40);
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
    Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

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
      csvFileName="/svt" + String(drifterName) + ".csv";
      
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
  if(var == "DRIFTERID") {
    return drifterName;
  }
  if(var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
  return String();
}

void writeData2Flash (){
  file = SPIFFS.open(csvFileName, FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
    ESP.restart();
  } else {
    if(file.println(csvOutStr)) {
      Serial.println("Wrote data in file, current size: ");
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

boolean loop_runEvery(unsigned long interval){
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    return true;
  }
  return false;
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
  localLinkRssi = maxRssi;
  return (int)bestRoute;
}

int setRoutingStatus(){
  // Update localHopCount and localNextHop

  deleteOldEntries();
  bool isEmpty = checkIfEmpty();
  Serial.println(isEmpty);
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
      localLinkRssi =  *(int *) (&routingTable[3]);
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
      if(result == 1){
        sendAckBack(mode, source);
        return result;          // Success
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
    return 1;             // Success
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
  Serial.println("ackhandshake");
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

void generate_packet(Stream &mySerial, TinyGPSPlus &myGPS) {
  //String dName = "D01";
  String dName = drifterName;
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
    gpsLastSecond = gps.time.second();
    Serial.print("lng=");
    Serial.println(gps.location.lng());
    Serial.print("age=");
    Serial.println(gps.location.age());
    if(true){//if((gps.location.lng() != 0.0) && (gps.location.age() < 1000)) {
      Serial.println("yes gps");
      // csvOutStr += tDate + "," + tTime + "," + tLocation + "\n";
      nSamples += 1;
      // B. Send GPS data on LoRa if it is this units timeslot
      if(gps.time.second() == drifterTimeSlotSec) {
        Serial.println("Sending packet via LoRa");
      }
    } else {
      Serial.println("NO GPS FIX, NOT SENDING OR WRITING");
    }
  }
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
        LoRa.write((const uint8_t*)&packet, sizeof(packet)); // Data packet
        LoRa.endPacket(true);
        break;
        
      case 0x44:              // Type D: RRequest
        header[7] = 0x18;
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write((const uint8_t*)&packet, sizeof(packet));
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
