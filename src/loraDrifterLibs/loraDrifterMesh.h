#ifndef LORADRIFTERMESH_H
#define LORADRIFTERMESH_H

// Timing Parameters
#define RS_BCAST_TIME             (17000)   // Time intervals, broadcast for every 17s
#define PL_TX_TIME                (12000)   // Receive pay load for every 12s
#define DELETION_TIME             (62000)   // Reset the routing table if entry's time is older than 62s
#define ARQ_TIME                  (2000)    // Automatic Repeat Request for every 2s

#define NUM_NODES                 (11)
#define ROUTING_TABLE_ENTRY_SIZE  (19)
#define MASTER_LOCAL_ID           (0xBB)
#define ROUTING_TABLE_SIZE        (MASTER_LOCAL_ID + 0x11)

#define SERVANT_MODE              (0)
#define MASTER_MODE               (1)

// #define DEBUG_HOP

#ifdef MESH_MASTER_MODE
extern String csvOutStr;
extern String messageLog;
#define MAX_NUM_LOGS              (30)
int numLogs = 0;
extern TinyGPSPlus gps;
class Master;
extern Master m;
class Servant;
extern Servant s[NUM_MAX_SERVANTS];           // Servants data array
extern SemaphoreHandle_t servantSemaphore;
#else
struct Packet;
extern Packet packet;
extern uint32_t last_packet_received_time_ms;
// extern SemaphoreHandle_t loraSemaphore;
#endif //MESH_MASTER_MODE

extern byte routingTable[ROUTING_TABLE_SIZE];
extern byte payload[24];
extern byte localHopCount;
extern byte localNextHopID;
extern byte localAddress;
// DIAGNOSTICS
extern int nodeRx[NUM_NODES];
extern int messagesSent;
extern int messagesReceived;

#ifndef MESH_MASTER_MODE
extern int localLinkRssi;
extern int masterRx;
#endif // MESH_MASTER_MODE

typedef enum {
  RouteBroadcastMaster  = 0x41,
  RouteBroadcastServant = 0x42,
  DirectPayload         = 0x43,
  RouteRequest          = 0x44,
  ACK                   = 0x45,
  Restart               = 0x46
} MessageType;

typedef enum {
  InvalidNodeID   = -9,
  NoACK           = -8,
  MasterModeErr   = -6,
  NodeModeErr     = -5,
  FrameHandlerErr = -4,
  ACKModeErr      = -3,
  PayloadErr      = -2,
  Invalid         = -1,
} ErrorType;

// Note: This is C++ notation, not C
typedef enum {
  Failure = 0,
  Success = 1,
} ResultType;

static int parsePayload() {
#ifdef MESH_MASTER_MODE
  Packet packet;
#endif // MESH_MASTER_NODE
  if(LoRa.available() == sizeof(Packet)) {
    uint8_t buffer[sizeof(Packet)];
    for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
      buffer[ii] = LoRa.read();
    }
    memcpy(&packet, buffer, sizeof(Packet));
#ifdef MESH_MASTER_MODE
// Get ID and then send to class for decoding
    const String name = String(packet.name);
    Serial.println(name);
    if(!strcmp(name.substring(0, 1).c_str(), "D")) {
      Serial.println("Drifter signal found!");
      const int id = name.substring(1, 3).toInt();
      xSemaphoreTake(servantSemaphore, portMAX_DELAY);
      s[id].ID = id;
      s[id].decode(&packet);
      s[id].rssi = LoRa.packetRssi();
      s[id].updateDistBear(m.lng, m.lat);
      s[id].active = true;
      Serial.println("RX from LoRa - decoding completed");
      const String tDate = String(s[id].year) + "-" + String(s[id].month) + "-" + String(s[id].day);
      // Serial.println(tDate);
      const String tTime = String(s[id].hour) + ":" + String(s[id].minute) + ":" + String(s[id].second);
      // Serial.println(tTime);
      const String tLocation = String(s[id].lng, 6) + "," + String(s[id].lat, 6) + "," + String(s[id].age);
      // Serial.println(tLocation);
      csvOutStr += "D" + String(id) + "," + tDate + "," + tTime + "," + tLocation  + '\n';
      // Serial.println("Seg fault after csvOutStr +=");

      xSemaphoreGive(servantSemaphore);
    }
    else {
      Serial.println("Not a complete drifter packet");
      return PayloadErr;
    }
#endif // MESH_MASTER_MODE
    return Success;
  }
  return PayloadErr;
}

// To help compiler
static int setRoutingStatus();
static int frameHandler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl);

static bool validateID(const byte nodeID) {
  switch(nodeID) {
    case 0x11:            // Node 1
    case 0x22:            // Node 2
    case 0x33:            // Node 3
    case 0x44:            // Node 4
    case 0x55:            // Node 5
    case 0x66:            // Node 6
    case 0x77:            // Node 7
    case 0x88:            // Node 8
    case 0x99:            // Node 9
    case 0xAA:            // Node 10
    case MASTER_LOCAL_ID: // Master Node
    case 0xFF:            // BCAST
      return true;
    default:              // Invalid ID
      return false;
  }
  return false;
}

static int idToIndex(const byte nodeID) {
  if(nodeID == MASTER_LOCAL_ID) return 0;
  return (nodeID == 0) ? 0 : nodeID / 0x10;
}

static byte indexToId(const int idx) {
  if(idx == 0) return MASTER_LOCAL_ID;
  return idx * 0x10 + idx;
}

static void incNodeRxCounter(const byte nodeID) {
#ifndef MESH_MASTER_MODE
  if(nodeID == MASTER_LOCAL_ID) {
    masterRx++;
  }
  else 
#endif // MESH_MASTER_MODE
  if(nodeID != MASTER_LOCAL_ID) {
    nodeRx[idToIndex(nodeID)]++;
  }
}

static void printRoutingTable() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte nodeID = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    const byte hopCount = routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const byte hopID = routingTable[(idx*ROUTING_TABLE_ENTRY_SIZE) + 2];
    const int Rssi = *(int*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3]);
    const float snr = *(float*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7]);
    const unsigned long currentTime = *(unsigned long*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11]);
    if(Rssi) {
      Serial.print("Routing Table Entry ");
      Serial.print(idx);
      Serial.println(": ");
      
      Serial.print("nodeID: 0x");
      Serial.println(nodeID, HEX);
      
      Serial.print("hopCount: ");
      Serial.println(hopCount);
      
      Serial.print("hopID: 0x");
      Serial.println(hopID, HEX);
      
      Serial.print("rssi: ");
      Serial.println(Rssi);
      
      Serial.print("snr: ");
      Serial.println(snr);
      
      Serial.print("time: ");
      Serial.println(currentTime);
      Serial.println(" ");
    }
  }
}

static int insertRoutingTable(const byte nodeID, const byte hopCount, const byte hopID, const int Rssi, const float snr, const unsigned long currentTime) {
  if(validateID(nodeID) && validateID(hopID)) { // validate node id and hop id
#ifdef DEBUG_HOP
    if(nodeID == MASTER_LOCAL_ID) {
      return InvalidNodeID;
    }
#endif // DEBUG_HOP
    Serial.print("Added 0x");
    Serial.print((int)nodeID, HEX);
    Serial.println(" to routing table");
    const int idx = idToIndex(nodeID);
    memcpy(&routingTable[idx * ROUTING_TABLE_ENTRY_SIZE], &nodeID, sizeof(nodeID));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1], &hopCount, sizeof(hopCount));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 2], &hopID, sizeof(hopID));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3], &Rssi, sizeof(Rssi));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7], &snr, sizeof(snr));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11], &currentTime, sizeof(currentTime));
#ifndef MESH_MASTER_MODE
    last_packet_received_time_ms = millis();
#endif //MESH_MASTER_MODE
    return Success;
  }
  return InvalidNodeID;
}

boolean runEvery(const unsigned long interval) {
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

boolean loop_runEvery(const unsigned long interval) {
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

static void deleteOldEntries() {
  // Reset when entry's time is older than DELETION_TIME
  const uint32_t currentTime = millis();
  long int lastTime = 0;
  int timeIndex = 0;
  int difference = 0;
  int newIndex = 0;
  
  for(int ii = 0; ii < NUM_NODES; ii++) {
    timeIndex = (ii * ROUTING_TABLE_ENTRY_SIZE) + 11;
    
    lastTime = *(long int*)(&routingTable[timeIndex]);
    difference = currentTime - lastTime;

    // Sets all 19 fields of an entry to 0x00
    if(difference > DELETION_TIME) {
      for(int jj = 0; jj < 18; jj++) {
        newIndex = (ii * ROUTING_TABLE_ENTRY_SIZE) + jj;
        routingTable[newIndex] = 0x00;
      }
    }
  }
}

static bool checkIfEmpty() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte entry = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    if(entry != 0x00) {
      return false;
    }
  }
  return true;
}

static bool searchMaster() {
  //Checks if a nodeID matches the Master ID MASTER_LOCAL_ID.
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte entry = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    if(entry == MASTER_LOCAL_ID) {
      return true;
    }
  }
  return false;
}

static int findMinHopCount() {
  int minHopCount = 255;
  int currentHopCount = 0;
  
  for(int idx = 0; idx < NUM_NODES; idx++) {
    byte hopCount = routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1];
    currentHopCount = (int)hopCount;
    if((currentHopCount != 0) && (currentHopCount < minHopCount)) {
      minHopCount = currentHopCount;
    }
  }
  return minHopCount;
}


static bool checkFrameHeader(const int mode, const byte sizeHeader, const byte type, const byte router, const byte source, const byte recipient, 
                      const byte sender, const byte ttl, const byte sizePayload) {
  // Check if header values are valid
  if(sizeHeader != 0x08) {
    Serial.println("checkFrameHeader: invalid sizeHeader");
    return false;
  }
  if(type < RouteBroadcastMaster || type > Restart) {
    Serial.println("checkFrameHeader: invalid type");
    return false; 
  }
  if(!validateID(router)) {
    Serial.println("checkFrameHeader: invalid router");
    return false;
  }
  if(!validateID(source)) {
    Serial.println("checkFrameHeader: invalid source");
    return false;
  }
  if(!validateID(recipient)) {
    Serial.println("checkFrameHeader: invalid recipient");
    return false;
  }
  if(!validateID(sender)) {
    Serial.println("checkFrameHeader: invalid sender");
    return false;
  }
  if(ttl > 0x0F || ttl == 0x00) {
    Serial.println("checkFrameHeader: invalid ttl");
    return false;
  }
  if(sizePayload != 0x02 && sizePayload != 0x18 && sizePayload != 0x00) {
    Serial.println("checkFrameHeader: invalid sizePayload");
    return false; 
  }

  // type and router ID
  if(mode == SERVANT_MODE) {
    if(type == DirectPayload || type == ACK) {
      Serial.println("checkFrameHeader: invalid type for Node Mode");
      return false;
    }
    if(type == RouteBroadcastMaster && sender != MASTER_LOCAL_ID) {
      Serial.println("checkFrameHeader: Invalid Type && sender ID");
      return false;
    }
    if(type == Restart) {
      return true;
    }
    if(router != localAddress && router != 0xFF) {
        Serial.println("checkFrameHeader: Not addressed to local");
        return false;
    }
    return true;
  }

  if(mode == MASTER_MODE) {
    if(type != DirectPayload) {         // Type C: Direct Master
      return false;
    }
    if(router != localAddress) {
      return false;
    }
    return true;
  }
  if(mode > 0x10 && mode < 0xBC) {
    if(type != ACK) {
      return false;
    }
    if(router != localAddress) {
      return false;
    }
    return true;
  }
  return false;
}

static void typeToPrintout(const byte type, const byte router) {
  switch(type) {
    case RouteBroadcastServant:
      Serial.print("Sending slave broadcast packet: 0x");
      break;
    case RouteBroadcastMaster:
      Serial.print("Sending master broadcast packet: 0x");
      break;
    case DirectPayload:
      Serial.print("Sending direct payload packet to: 0x");
      break;
    case RouteRequest:
      Serial.print("Sending GPS packet to: 0x");
      break;
    case ACK:
      Serial.print("Sending ACK packet to: 0x");
      break;
    case Restart:
      Serial.print("Sending Restart packet to: 0x");
      break;
    default:
      break;
  }
  Serial.println((int)router, HEX);
}

static void sendFrame(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl) {
  // Send a complete header with a random delay
  messagesSent++;
  byte header[8] = "";
  header[0] = 0x08;         // sizeHeader
  header[1] = type;
  header[2] = router;
  header[3] = localAddress; // source
  header[4] = recipient;
  header[5] = sender;
  header[6] = ttl - 1;      // ttl

  delay(random(20));
  if(mode == SERVANT_MODE) {
    switch(type) {
      case RouteBroadcastServant:
        header[7] = 0x02;           // sizePayload
        // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write(localHopCount);  // RS payload
        LoRa.write(localNextHopID); // RS payload
        LoRa.endPacket(true);
        // xSemaphoreGive(loraSemaphore);
        typeToPrintout(type, router);
        break;
      case DirectPayload:
      case RouteRequest:
        header[7] = 0x18;
        // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
#ifndef MESH_MASTER_MODE // for compiling
        LoRa.write((const uint8_t *)&packet, sizeof(Packet));
#endif // MESH_MASTER_MODE
        LoRa.endPacket(true);
        // xSemaphoreGive(loraSemaphore);
        typeToPrintout(type, router);
        break;
      case ACK:
      case Restart:
        header[7] = 0x00;
        // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        // xSemaphoreGive(loraSemaphore);
        typeToPrintout(type, router);
        break;
      default:
        Serial.println("Not valid for this mode");
        break;
      }
  }
  else if(mode == MASTER_MODE) {
    switch(type) {
      case RouteBroadcastMaster:
      case ACK:
      case Restart:
        header[7] = 0x00;
        // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        // xSemaphoreGive(loraSemaphore);
        typeToPrintout(type, router);
        break;
      default:
        Serial.println("Not valid for this mode");
        break;
    }
  }
  else {
    Serial.println("Not a valid mode");
  }
}

// Send an ACK back to the source
static void sendAckBack(const int mode, const byte source) {
  delay(random(5));
  sendFrame(mode, ACK, source, source, localAddress,  0x0F);
}

int listener(const int frameSize, const int mode) {
  if(!frameSize) {
    return Failure;             // nothing to receive
  }
  // Parse Header
  // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
  const byte sizeHeader = LoRa.read();
  const byte type = LoRa.read();
  const byte router = LoRa.read();
  const byte source = LoRa.read();
  const byte recipient = LoRa.read();
  const byte sender = LoRa.read();
  const byte ttl = LoRa.read();
  const byte sizePayload = LoRa.read();
  // xSemaphoreGive(loraSemaphore);
  
  const bool validHeader = checkFrameHeader(mode, sizeHeader,type, router, source, recipient, sender, ttl, sizePayload);
  if(validHeader) {
    messagesReceived++;
    incNodeRxCounter(source);

    if(type == DirectPayload) { // this will go direct to master
      Serial.print("Received DirectPayload packet from: 0x");
      Serial.println((int)sender, HEX);
      Serial.print("Routed from: 0x");
      Serial.println((int)source, HEX);
#ifdef MESH_MASTER_MODE
      if(numLogs >= MAX_NUM_LOGS) {
        messageLog = "";
        numLogs = 0;
      }
      String temp = "";
      if(sender != source) {
        temp = " routed by D" + String(idToIndex(source));
      }
      const String tDate = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day());
      const String tTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      messageLog += "<tr><td>" + tDate + " " + tTime  + "</td><td>Received packet from D" + String(idToIndex(sender)) + temp + "</td></tr>";
      numLogs++;
#endif //MESH_MASTER_MODE
    }
    else if(type == RouteRequest) { // this is a hop
      Serial.print("Received RouteRequest packet from: 0x");
      Serial.println((int)sender, HEX);
      Serial.print("Routed from: 0x");
      Serial.println((int)source, HEX);
    }
    return frameHandler(mode, type, router, source, recipient, sender, ttl); // 1 or E (-6 to -1)
  }
  return Failure;
}

static bool waitForAck(const byte router) {
  int maxloops = 0;
  int result = 0;
  const int interval = ARQ_TIME * localHopCount;
  const int ackMode = (int)router;

  while(maxloops < interval && result != 1) {
    result = listener(LoRa.parsePacket(), ackMode);
    delay(1);
    maxloops++;
  }
  return (result == Success) ? true : false; // ACK received from router or not
}

static int ackHandshake(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl, int resend) {
  // If no ACK received, resend two more times.
  bool ack = false;
  // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
  sendFrame(mode, type, router, recipient, sender, ttl);
  ack = waitForAck(router);
  // xSemaphoreGive(loraSemaphore);

  while(!ack && resend < 2) {
    // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
    sendFrame(mode, type, router, recipient, sender, ttl);
    ack = waitForAck(router);
    // xSemaphoreGive(loraSemaphore);
    resend++;
  }
  return (!ack) ? Failure : Success;
}

static int routePayload(const int mode, const byte recipient, const byte sender, const byte ttl, const int resend) {
  // Send the data based on routing status
  byte type = 0x00;
  // Check first if the localNextHopID is the Master ID
  type = (recipient == localNextHopID) ? DirectPayload : RouteRequest;
  const byte router = localNextHopID;
  int result = ackHandshake(mode, type, router, recipient, sender, ttl, resend);
  Serial.print("Route payload ACK handshake result with router (0x");
  Serial.print((int)router, HEX);
  Serial.print("): ");
  if(result == Failure) {
    Serial.println("Failed.");
    return NoACK;
  }
  Serial.println("Success!");
  return Success;
}

//Process data frame
static int frameHandler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl) {
  int result = 0;
  if(mode == SERVANT_MODE) {
    switch(type) {
      case RouteBroadcastMaster: {
        // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
        const int rssi = LoRa.packetRssi();
        const float snr = LoRa.packetSnr();
        // xSemaphoreGive(loraSemaphore);
        const unsigned long time = millis();
        result = insertRoutingTable(sender, 0x01, MASTER_LOCAL_ID, rssi, snr, time);
        if(result != Success) {
          return result;
        }
        return setRoutingStatus();
      }
      case RouteBroadcastServant: {
        // xSemaphoreTake(loraSemaphore, portMAX_DELAY);
        const byte hopCount = LoRa.read();     // Parsing Payload
        const byte nextHopID = LoRa.read();
        const int rssi = LoRa.packetRssi();
        const float snr = LoRa.packetSnr();
        // xSemaphoreGive(loraSemaphore);
        const unsigned long time = millis();
        result = insertRoutingTable(sender, hopCount, nextHopID, rssi, snr, time);
        if(result != Success) {
          return InvalidNodeID;
        }
        return setRoutingStatus();
      }
      case RouteRequest: {
        parsePayload();
        result = routePayload(mode, recipient, sender, ttl, 0);
        if(result == Success) {
          sendAckBack(mode, source);
          return Success;
        } else {
          return NoACK;
        }
      }
      case Restart: {
        if(recipient == localAddress) {
          sendAckBack(mode, source);
          Serial.println("Restarting device");
          ESP.restart();
        }
      }
      default:
        return NodeModeErr;
    }
  }
  else if(mode == MASTER_MODE) {
    switch(type) {
      case DirectPayload: {
        result = parsePayload();
        if(result == Success) {
          Serial.println("Successfully parsed packet, now sending ack");
          sendAckBack(mode, source);
          return result;          // Success
        }
        return result;            // Parse ERR
      } 
      default:
        return MasterModeErr;
    }
  }
  // Checking the ackMode that was passed from waitForAck()
  // The address space in integers between 17 and 170.
  // Node 1 - 0x11 - 17
  // Node 2 - 0x22 - 34
  // Master Node - MASTER_LOCAL_ID - 184
  
  else if(mode > 0x10 && mode < 0xBC) { // ACK Mode
    if(type == ACK) {
      return Success;
    } 
    return ACKModeErr;
  }
  return FrameHandlerErr;
}

// Send a broadcast to the network
static int bcastRoutingStatus(const int mode) {
  int result = 0;
  if(mode == SERVANT_MODE) {
    result = setRoutingStatus();
    if(result == Invalid) {
      return result;
    }
    sendFrame(mode, RouteBroadcastServant, 0xFF, 0xFF, localAddress, 0x0F);
  }
  else if(mode == MASTER_MODE) {
    sendFrame(mode, RouteBroadcastMaster, 0xFF, 0xFF, localAddress, 0x0F);
  }
  return Success;
}

#ifdef MESH_MASTER_MODE
static int getNodeRxCounter(const byte nodeID) {
  return nodeRx[idToIndex(nodeID)];
}

static void printNodeInfo() {
  const int nodeID = *(int *) (&payload[0]);
  const int hopCount = *(int *) (&payload[4]);
  const int nextHop = *(int *) (&payload[8]);
  const int linkRssi = *(int *) (&payload[12]);
  const int attemptedPayloadTx = *(int *) (&payload[16]);
  incNodeRxCounter(nodeID);
  const int nodeXRx = nodeRx[nodeID];// getNodeRxCounter(nodeID);

  Serial.print("    Received payload from Node ID 0x");
  Serial.print(nodeID, HEX);
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
  Serial.println(nodeXRx);
}
#endif //MESH_MASTER_MODE

static int findMaxRssi(const int minHopCount) {
  //To make sure the nextHop of that entry is not local
  int currentRssi = 0;
  int maxRssi = -10000000;
  byte bestRoute = 0x00;

  for(int ii = 0; ii < NUM_NODES; ii++) {
    const byte hopCount = routingTable[(ii * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const byte nextHopID = routingTable[(ii * ROUTING_TABLE_ENTRY_SIZE) + 2];

    // maintain the currentRssi as maxRssi.
    if((hopCount == minHopCount) && (nextHopID != localAddress)) {
      currentRssi = *(int *)(&routingTable[(ii * ROUTING_TABLE_ENTRY_SIZE) + 3]);
      if (currentRssi > maxRssi) {
        maxRssi = currentRssi;
        bestRoute = routingTable[ii * ROUTING_TABLE_ENTRY_SIZE];
      }
    }
  }
#ifndef MESH_MASTER_MODE
  localLinkRssi = maxRssi;
#endif // MESH_MASTER_MODE
  return (int)bestRoute;
}

 // Returns Success or Invalid
static int setRoutingStatus() {
  // Update localHopCount and localNextHop
  deleteOldEntries();
  if(!checkIfEmpty()) {
    const bool masterFound = searchMaster();
    if(!masterFound) {
      const int minHopCountInt = findMinHopCount();
      const int bestRouteInt = findMaxRssi(minHopCountInt);

      const byte minHopCount = (byte)minHopCountInt;
      const byte bestRoute = (byte)bestRouteInt;
      if(minHopCount != 0x00 && bestRoute != 0x00) {
        // valid route
        localHopCount = minHopCount + 1;
        localNextHopID = bestRoute;
#ifndef MESH_MASTER_MODE
        localLinkRssi =  *(int *) (&routingTable[3]);
#endif // MESH_MASTER_MODE
        return Success;
      } else {
        return Invalid;
      }
    } else {
      // master is inside the routing table
      localNextHopID = MASTER_LOCAL_ID;
      localHopCount = 0x01;
      return Success;
    } 
  } else {
    // empty table
    localNextHopID = 0x00;
    localHopCount = 0x00;
    return Invalid;
  }
}

#endif // LORADRIFTERMESH_H