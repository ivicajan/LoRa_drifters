#ifndef LORADRIFTERMESH_H
#define LORADRIFTERMESH_H

// Timing Parameters
#define RS_BCAST_TIME             17000   // Time intervals, broadcast for every 17s
#define PL_TX_TIME                12000   // Receive pay load for every 12s
#define DELETION_TIME             62000   // Reset the routing table if entry's time is older than 62s
#define ARQ_TIME                  2000    // Automatic Repeat Request for every 2s

#define NUM_NODES                 8
#define ROUTING_TABLE_ENTRY_SIZE  19

#define SERVANT_MODE              0
#define MASTER_MODE               1

class Master;
class Servant;
struct Packet;

#ifdef MESH_MASTER_MODE
extern Master m;
#define nServantsMax             8       // Maximum number of servant drifters (just for setting array size)
extern Servant s[nServantsMax];          // Servants data array
extern SemaphoreHandle_t servantSemaphore;
#else
extern Packet packet;
extern SemaphoreHandle_t loraSemaphore;
#endif //MESH_MASTER_MODE

extern byte routingTable[0x99];
extern byte payload[24];
extern byte localHopCount;
extern byte localNextHopID;
extern byte localAddress;

// DIAGNOSTICS
extern int node1Rx;
extern int node2Rx;
extern int node3Rx;
extern int node4Rx;
extern int node5Rx;
extern int node6Rx;
extern int node7Rx;
extern int messagesSent;
extern int messagesReceived;

#ifndef MESH_MASTER_MODE
extern int localLinkRssi;
extern int masterRx;
#endif // MESH_MASTER_MODE

typedef enum {
  RouteBroadcastMaster = 0x41,
  RouteBroadcastServant = 0x42,
  DirectPayload = 0x43,
  RouteRequest = 0x44,
  ACK      = 0x45,
  Restart  = 0x46
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

// This is backwards
typedef enum {
  Failure = 0,
  Success = 1,
} ResultType;

int parsePayload() {
  Serial.println("Parsing payload!");
#ifdef MESH_MASTER_MODE
  Packet packet;
#endif // MESH_MASTER_NODE
  memset(&packet, 0, sizeof(Packet));
  if(LoRa.available() == sizeof(Packet)) {
    uint8_t buffer[sizeof(Packet)];
    for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
      buffer[ii] = LoRa.read();
    }
    packet = *(Packet *)buffer;
#ifdef MESH_MASTER_MODE
// Get ID and then send to class for decoding
    const String name = String(packet.name);
    Serial.println(name);
    if(!strcmp(name.substring(0, 1).c_str(), "D")) {
      Serial.println("Drifter signal found!");
      // csvOutStr += recv; // Save all packets recevied (debugging purposes)
      const int id = name.substring(1, 3).toInt();
      if(xSemaphoreTake(servantSemaphore, portMAX_DELAY) == pdPASS) {
        s[id].ID = id;
        s[id].decode(&packet);
        s[id].rssi = LoRa.packetRssi();
        s[id].updateDistBear(m.lng, m.lat);
        s[id].active = true;
        Serial.println("RX from LoRa - decoding completed");
      }
      xSemaphoreGive(servantSemaphore);
    }
    else {
      Serial.println("Not a complete drifter packet");
      return PayloadErr;
    }
#endif // MESH_MASTER_MODE
    return Success;
  }
  return PayloadErr;       // Payload ERR
}

// To help compiler
int setRoutingStatus();
int frameHandler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl);

bool validateID(const byte nodeID) {
  switch(nodeID) {
    case 0x11:        // Node 1
    case 0x22:        // Node 2
    case 0x33:        // Node 3
    case 0x44:        // Node 4
    case 0x55:        // Node 5
    case 0x66:        // Node 6
    case 0x77:        // Node 7
    case 0xAA:        // Master Node
    case 0xFF:        // BCAST
      return true;
    default:        // Invalid ID
      return false;
  }
  return false;
}

int idToIndex(const byte nodeID) {
  if(nodeID == 0xAA) return 0;
  return (nodeID == 0) ? 0 : nodeID / 0x10;
}

byte indexToId(const int idx) {
  if(idx == 0) return 0xAA;
  return (idx == 0) ? 0x00 : (idx * 0x10) + idx;
}

void incNodeRxCounter(const int nodeID) {
  switch(nodeID) {
    case 0x11:
      node1Rx++;
      break;
    case 0x22:
      node2Rx++;
      break;
    case 0x33:
      node3Rx++;
      break;
    case 0x44:
      node4Rx++;
      break;
    case 0x55:
      node5Rx++;
      break;
    case 0x66:
      node6Rx++;
      break;
    case 0x77:
      node7Rx++;
      break;
#ifndef MESH_MASTER_MODE
    case 0xAA:
      masterRx++;
      break;
#endif // MESH_MASTER_MODE
    default:
      break;
  }
}

void printRoutingTable() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte nodeID = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    const byte hopCount = routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const byte hopID = routingTable[(idx*ROUTING_TABLE_ENTRY_SIZE) + 2];
    const int Rssi = *(int*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3]);
    const float snr = *(float*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7]);
    const unsigned long currentTime = *(unsigned long*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11]);
    if(Rssi != 0) {
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

int insertRoutingTable(const byte nodeID, const byte hopCount, const byte hopID, const int Rssi, const float snr, const unsigned long currentTime) {
  const bool validNode = validateID(nodeID);
  const bool validHop = validateID(hopID);

  if(validNode && validHop) {
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
    return Success;         // Success
  }
  return InvalidNodeID;   // E -9: invalid nodeID
}

boolean runEvery(const unsigned long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

boolean loop_runEvery(const unsigned long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void deleteOldEntries() {
  // Reset when entry's time is older than DELETION_TIME
  const long int currentTime = millis();
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

bool checkIfEmpty() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte entry = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    if(entry != 0x00) {
      return false;
    }
  }
  return true;
}

bool searchMaster() {
  //Checks if a nodeID matches the Master ID 0xAA.
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte entry = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    if(entry == 0xAA) {
      return true;
    }
  }
  return false;
}

int findMinHopCount() {
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


bool checkFrameHeader(const int mode, const byte sizeHeader, const byte type, const byte router, const byte source, const byte recipient, 
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
    if(type == RouteBroadcastMaster && sender != 0xAA) {
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
  if(mode > 0x10 && mode < 0xAB) {
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

void typeToPrintout(const byte type, const byte router) {
  switch(type) {
    case RouteBroadcastServant:
      Serial.print("Sending broadcast packet to: 0x");
      Serial.println((int)router, HEX);
      break;
    case DirectPayload:
      Serial.print("Sending direct payload packet to: 0x");
      Serial.println((int)router, HEX);
      break;
    default:
      break;
  }
}

void sendFrame(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl) {
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
        header[7] = 0x02;         // sizePayload
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write(localHopCount);    // RS payload
        LoRa.write(localNextHopID);   // RS payload
        LoRa.endPacket(true);
        typeToPrintout(type, router);
        break;
      case DirectPayload:
      case RouteRequest:
        header[7] = 0x18;
        LoRa.beginPacket();
        LoRa.write(header, 8);
#ifndef MESH_MASTER_MODE // for compiling
        LoRa.write((const uint8_t*)&packet, sizeof(Packet));
#endif // MESH_MASTER_MODE
        LoRa.endPacket(true);
        Serial.print("Sending GPS packet to: 0x");
        Serial.println((int)router, HEX);
        typeToPrintout(type, router);
        break;
      case ACK:
      case Restart:
        header[7] = 0x00;       // sizePayload
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        break;
      default:
        Serial.println("Not valid for this mode");
        break;
      }
  } else if(mode == MASTER_MODE) {
    switch(type) {
      case RouteBroadcastMaster:
      case ACK:
      case Restart:
        header[7] = 0x00;
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        typeToPrintout(type, router);
        break;
      default:
        Serial.println("Not valid for this mode");
        break;
    }
  } else {
    Serial.println("Not a valid mode");
  }
}

// Send an ACK back to the source
void sendAckBack(const int mode, const byte source) {
  delay(random(5));
  sendFrame(mode, ACK, source, source, localAddress,  0x0F);
}

int listener(const int frameSize, const int mode) {
  if(!frameSize) {
    return Failure;             // nothing to receive
  }
  // Parse Header
  const byte sizeHeader = LoRa.read();
  const byte type = LoRa.read();
  const byte router = LoRa.read();
  const byte source = LoRa.read();
  const byte recipient = LoRa.read();
  const byte sender = LoRa.read();
  const byte ttl = LoRa.read();
  const byte sizePayload = LoRa.read();

  const bool validHeader = checkFrameHeader(mode, sizeHeader,type, router, source, recipient, sender, ttl, sizePayload);
  if(validHeader) {
    messagesReceived++;
    incNodeRxCounter(source);

    if(type == DirectPayload) { // this will go direct to master
      Serial.print("Received DirectPayload packet from: 0x");
      Serial.println((int)sender, HEX);
      Serial.print("Routed from: 0x");
      Serial.println((int)router, HEX);
    }
    else if(type == RouteRequest) { // this is a hop
      Serial.print("Received RouteRequest packet from: 0x");
      Serial.println((int)sender, HEX);
      Serial.print("Routed from: 0x");
      Serial.println((int)router, HEX);
    }
    return frameHandler(mode, type, router, source, recipient, sender, ttl); // 1 or E (-6 to -1)
  }
  return Failure;
}

bool waitForAck(const byte router) {
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

int ackHandshake(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl, int resend) {
  // If no ACK received, resend two more times.
  bool ack = false;
  if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
    sendFrame(mode, type, router, recipient, sender, ttl);
    ack = waitForAck(router);
  }
  xSemaphoreGive(loraSemaphore);

  while(!ack && resend < 2) {
    if(xSemaphoreTake(loraSemaphore, portMAX_DELAY) == pdTRUE) {
      sendFrame(mode, type, router, recipient, sender, ttl);
      ack = waitForAck(router);
    }
    xSemaphoreGive(loraSemaphore);
    resend++;
  }
  return (!ack) ? Failure : Success;
}

int routePayload(const int mode, const byte recipient, const byte sender, const byte ttl, const int resend) {
  // Send the data based on routing status
  byte type = 0x00;

  // Check first if the localNextHopID is the Master ID
  type = (recipient == localNextHopID) ? DirectPayload : RouteRequest; // Type C: Direct Master PL, Type D: Route Request
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

int frameHandler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl) {
  //Process data frame
  int result = 0;

  if(mode == SERVANT_MODE) {              // Node Mode
    if(type == RouteBroadcastMaster) {         // Type A: Master BCAST
      const int rssi = LoRa.packetRssi();
      const float snr = LoRa.packetSnr();
      const unsigned long time = millis();
      result = insertRoutingTable(sender, 0x01, 0xAA, rssi, snr, time);
      if(result != Success) {
        return result;
      }
      return setRoutingStatus(); // 1 or -1
    }
    else if(type == RouteBroadcastServant) {                // Type B: Neighbor BCAST
      const byte hopCount = LoRa.read();  // Parsing Payload
      const byte nextHopID = LoRa.read();
      const int rssi = LoRa.packetRssi();
      const float snr = LoRa.packetSnr();
      const unsigned long time = millis();
      
      result = insertRoutingTable(sender, hopCount, nextHopID, rssi, snr, time);
      if(result != Success) {
        return result;        // invalid nodeID
      }
      return setRoutingStatus(); // 1 or -1
    }
    else if(type == RouteRequest) {         // Type D: Route Request
      parsePayload();
      result = routePayload(mode, recipient, sender, ttl, 0);
      if(result == 1) { // found a route!
        sendAckBack(mode, source);
        return result;          // Success
      } else {
        return result;          // No ACK
      }
    }
    else if(type == Restart) {
      if(recipient == localAddress) {
        sendAckBack(mode, source);
        Serial.println("Restarting device");
        ESP.restart();
      }
    }
    return NodeModeErr;
  } 
  
  if(mode == MASTER_MODE) {
    if(type == DirectPayload) {
      result = parsePayload();
      if(result == 1) {
        Serial.println("Successfully parsed packet, now sending ack");
        sendAckBack(mode, source);
        return result;          // Success
      }
      return result;            // Parse ERR
    } 
    return MasterModeErr;
  }
  
  // Checking the ackMode that was passed from waitForAck()
  // The address space in integers between 17 and 170.
  // Node 1 - 0x11 - 17
  // Node 2 - 0x22 - 34
  // Master Node - 0xAA - 170
  
  if(mode > 0x10 && mode < 0xAB) {          // ACK Mode
    if(type == ACK) {
      return Success;
    } 
    return ACKModeErr;
  }
  return FrameHandlerErr;
}

int bcastRoutingStatus(const int mode) {
  // Send a broadcast to the network
  int result = 0;
  if(mode == 0) {
    result = setRoutingStatus();
    if(result == -1) {
      return result;
    }
    sendFrame(
      mode,
      RouteBroadcastServant,         // type: Type B
      0xFF,             // router: BCAST
      0xFF,             // recipient: BCAST
      localAddress,     // sender
      0x0F              // ttl
    );
  } else if(mode == 1) {   // Master Mode
    sendFrame(
      mode,
      RouteBroadcastMaster,         // type: Type A
      0xFF,             // router: BCAST
      0xFF,             // recipient: BCAST
      localAddress,     // sender
      0x0F              // ttl
    );
  }
  return Success;
}

#ifdef MESH_MASTER_MODE
int getNodeRxCounter(const byte nodeID){
  switch(nodeID) {
      case 0x11:
        return node1Rx;
      case 0x22:
        return node2Rx;
      case 0x33:
        return node3Rx;
      case 0x44:
        return node4Rx;
      case 0x55:
        return node5Rx;
      case 0x66:
        return node6Rx;
      case 0x77:
        return node7Rx;
  }
  return 0;
}

void printNodeInfo(){
  const int nodeID = *(int *) (&payload[0]);
  const int hopCount = *(int *) (&payload[4]);
  const int nextHop = *(int *) (&payload[8]);
  const int linkRssi = *(int *) (&payload[12]);
  const int attemptedPayloadTx = *(int *) (&payload[16]);
  incNodeRxCounter(nodeID);
  const int nodeRx = getNodeRxCounter(nodeID);

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
  Serial.println(nodeRx);
}
#endif //MESH_MASTER_MODE

int findMaxRssi(const int minHopCount) {
  //To make sure the nextHop of that entry is not local
  int currentRssi = 0;
  int maxRssi = -10000000;
  byte bestRoute = 0x00;

  for(int ii = 0; ii < NUM_NODES; ii++) {
    byte hopCount = routingTable[(ii * ROUTING_TABLE_ENTRY_SIZE) + 1];
    byte nextHopID = routingTable[(ii * ROUTING_TABLE_ENTRY_SIZE) + 2];

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

int setRoutingStatus() {
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
      localNextHopID = 0xAA;
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

int daemon(const unsigned int mode) {
  if(runEvery(RS_BCAST_TIME)) {
    Serial.println("Broadcasting");
    return bcastRoutingStatus(mode);   // returns 1 or -1
  }
  return listener(LoRa.parsePacket(), mode);
}                                 // returns 0: Nothing relevant or valid
                                  // returns 1: Valid processing
                                  // returns -8 to -1: Processing Errors

#endif // LORADRIFTERMESH_H