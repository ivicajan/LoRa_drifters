#ifndef LORADRIFTERMESH_H
#define LORADRIFTERMESH_H

/*
All Nodes are filling their Routing Tables and Routing Statuses dynamically.
Slaves: Broadcast local routing status to the network and sent to the master

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

routing table structure - 19 bytes
  nodeId         1 byte
  hopCount       1 byte
  hopId          1 byte
  Rssi           4 bytes
  snr            4 bytes
  current time   8 bytes

*/

// Timing Parameters
#define RS_BCAST_TIME   4000    //Time intervals, broadcast for every 4000ms
#define PL_TX_TIME      6000   //Receive pay load for every ms              // 6000
#define DELETION_TIME   100000   //Reset the routing table if entry's time is older than 100000ms
#define ARQ_TIME        1000    //Automatic Repeat Request for every 1000ms

#define NUM_NODES                 9
#define ROUTING_TABLE_ENTRY_SIZE 19

// #define MESH_MASTER_MODE
class Master;
class Servant;
struct Packet;

#ifdef MESH_MASTER_MODE
extern Master m;
#define nServantsMax             9       // Maximum number of servant drifters (just for setting array size)
extern Servant s[nServantsMax]; // Servants data array
#else
extern Packet packet;
Master m;
#endif //MESH_MASTER_MODE

extern byte routingTable[0x99];
extern byte payload[24];
extern byte localHopCount;
extern byte localNextHopID;
extern byte localAddress;

#ifdef MESH_MASTER_MODE
// DIAGNOSTICS
extern int node1Rx;
extern int node2Rx;
extern int node3Rx;
extern int node4Rx;
extern int node5Rx;
extern int node6Rx;
extern int node7Rx;
#else
extern int localLinkRssi;
#endif // MESH_MASTER_MODE

typedef enum {
  RSBcastM = 0x41, // master
  RSBcastS = 0x42, // slave
  DirectPl = 0x43,
  RRequest = 0x44,
  ACK      = 0x45,
} MessageType;

typedef enum {
  Invalid_Node_ID   = -9,
  Master_Mode_Err   = -6,
  Node_Mode_Err     = -5,
  Frame_Handler_Err = -4,
  ACK_Mode_Err      = -3,
  Payload_Err       = -2,
  Invalid           = -1,
} ErrorType;

typedef enum {
  Failure = 0,
  Success = 1,
} ResultType;

int parsePayload() {
  // Parses the data
  if(LoRa.available() == sizeof(Packet)) {
    uint8_t buffer[sizeof(Packet)];
    for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
      buffer[ii] = LoRa.read();
    }
    Packet * packet;
    memset(&packet, 0, sizeof(packet));
    packet = (Packet *)buffer;

#ifdef MESH_MASTER_MODE
// Get ID and then send to class for decoding
    String name = String(packet->name);
    Serial.println("Packet name:");
    Serial.println(name);
    if(!strcmp(name.substring(0, 1).c_str(), "D")) {
      Serial.println("Drifter signal found!");
      // csvOutStr += recv; // Save all packets recevied (debugging purposes)
      int id = name.substring(1, 3).toInt();
      s[id].ID = id;
      s[id].decode(packet);
      s[id].rssi = LoRa.packetRssi();
      s[id].updateDistBear(m.lng, m.lat);
      s[id].active = true;
      Serial.println("RX from LoRa - decoding completed");
    }
    return 2;         // Success
#else
    return Success;
#endif // MESH_MASTER_MODE
  }
  return Payload_Err;       // Payload ERR
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

void printRoutingTable() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte nodeID = routingTable[idx * ROUTING_TABLE_ENTRY_SIZE];
    const byte hopCount = routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const byte hopID = routingTable[(idx*ROUTING_TABLE_ENTRY_SIZE) + 2];
    const int Rssi = *(int*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3]);
    const float snr = *(float*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7]);
    const unsigned long currentTime = *(unsigned long*)(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11]);

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

int insertRoutingTable(const byte nodeID, const byte hopCount, const byte hopID, const int Rssi, const float snr, const unsigned long currentTime) {
  const bool validNode = validateID(nodeID);
  const bool validHop = validateID(hopID);

  if(validNode && validHop) {
    const int idx = idToIndex(nodeID);

    memcpy(&routingTable[idx * ROUTING_TABLE_ENTRY_SIZE], &nodeID, sizeof(nodeID));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1], &hopCount, sizeof(hopCount));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 2], &hopID, sizeof(hopID));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3], &Rssi, sizeof(Rssi));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7], &snr, sizeof(snr));
    memcpy(&routingTable[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11], &currentTime, sizeof(currentTime));
    Serial.print("Inserting ");
    Serial.print(nodeID);
    Serial.println(" into routing table");
    printRoutingTable();
    return Success;         // Success
  }
  return Invalid_Node_ID;   // E -9: invalid nodeID
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
  if(sizeHeader!= 0x08) {
    Serial.println("checkFrameHeader: invalid sizeHeader");
    return false;
  }
  if(type < RSBcastM || type > ACK) {
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
  if(mode == 0) {              // Node Mode
    if(type == 0x43 || type == 0x45) {   // Type D OR Type E
      Serial.println("checkFrameHeader: invalid type for Node Mode");
      return false;
    }
    if(type == 0x41 && sender != 0xAA) {
       Serial.println("checkFrameHeader: Invalid Type && sender ID");
       return false;
     }
    if(router != localAddress && router != 0xFF) {
        Serial.println("checkFrameHeader: Not addressed to local");
        return false;
    }
    return true;
  }

  if(mode == 1) {                  // Master Mode
    if(type != DirectPl) {         // Type C: Direct Master
      return false;
    }
    if(router != localAddress) {
      return false;
    }
    return true;
  }

  if(mode > 16 && mode < 171) {        // ACK Mode
    if(type != ACK) {                  // Type E: ACK
      return false;
    }
    if(router != localAddress) {
      return false;
    }
    return true;
  }
  return false;
}

void sendFrame(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl) {
  // Send a complete header with a random delay
  byte header[8] = "";
  header[0] = 0x08;         // sizeHeader
  header[1] = type;         // type
  header[2] = router;       // router
  header[3] = localAddress; // source
  header[4] = recipient;    // recipient
  header[5] = sender;       // sender
  header[6] = ttl - 1;      // ttl

  delay(random(20));
  if(mode == 0) {                 // Node Mode
    switch(header[1]){            // check type
      case RSBcastS:              // Type B: RS BCAST
        header[7] = 0x02;         // sizePayload
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write(localHopCount);    // RS payload
        LoRa.write(localNextHopID);   // RS payload
        LoRa.endPacket(true);
        break;
      case DirectPl:              // Type C: Direct PL
      case RRequest:              // Type D: RRequest
        header[7] = 0x18;
        LoRa.beginPacket();
        LoRa.write(header, 8);
#ifdef MESH_MASTER_MODE
        LoRa.write((const uint8_t*)&m, sizeof(m));
#else
        LoRa.write((const uint8_t*)&packet, sizeof(packet));
#endif // MESH_MASTER_MODE
        LoRa.endPacket(true);
        break;
      case ACK:                 // Type E: ACK
        header[7] = 0x00;       // sizePayload
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        break;
      default:
        Serial.println("Not valid for this mode");
        break;
      }
  } else if(mode == 1) {        // Master Mode
    switch(header[1]){          // check type
      case RSBcastM:            // Type A: RS BCAST
      case ACK:                 // Type E: ACK
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

void sendAckBack(const int mode, const byte source) {
  // To send an ACK back to the source
  sendFrame(mode, ACK, source, source, localAddress,  0x0F);
}

int listener(const int frameSize, const int mode) {
  if(frameSize == 0) {
    return 0;             // nothing to receive
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
    return frameHandler(mode, type, router, source, recipient, sender, ttl); // 1 or E (-6 to -1)
  } 
  return 0;
}

bool waitForAck(const byte router) {
  // Wait for ACK
  int maxloops = 0;
  int result = 0;
  const int interval = ARQ_TIME * localHopCount;
  const int ackMode = (int)router;

  while(maxloops < interval && result != 1) {
    result = listener(LoRa.parsePacket(), ackMode);
    delay(1);
    maxloops++;
  }
  return (result == 1) ? true : false; // ACK received from router or not
}

int ackHandshake(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl, int resend) {
  // If no ACK received, resend two more times.
  sendFrame(mode, type, router, recipient, sender, ttl);
  bool ack = waitForAck(router);

  while(!ack && resend < 2) {
    sendFrame(mode, type, router, recipient, sender, ttl);
    ack = waitForAck(router);
    resend++;
  }
  
  if(!ack) {
    Serial.println("Did not receive ACK from router.");
    return 0;
  } else {
    return Success;
  }
}

int routePayload(const int mode, const byte recipient, const byte sender, const byte ttl, const int resend) {
  // Send the data based on routing status
  byte type = 0x00;

  // Check first if the localNextHopID is the Master ID
  type = (recipient == localNextHopID) ? DirectPl : RRequest; // Type C: Direct Master PL, Type D: Route Request
  if(type == RRequest) {
      Serial.println("RRequest in routepayload");
  }
  const byte router = localNextHopID;
  const int result = ackHandshake(mode, type, router, recipient, sender, ttl, resend);
  return (result == 1) ? result : -8; // Ack received or No ACK received
}

int frameHandler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl) {
  //Process data frame
  int result = 0;

  if(mode == 0) {              // Node Mode
    if(type == RSBcastM) {         // Type A: Master BCAST
      const int rssi = LoRa.packetRssi();
      const float snr = LoRa.packetSnr();
      const unsigned long time = millis();
      result = insertRoutingTable(sender, 0x01, 0xAA, rssi, snr, time);
      if(result != 1) {
        return result;
      }
      return setRoutingStatus(); // 1 or -1
    }
    
    if(type == RSBcastS) {                // Type B: Neighbor BCAST
      const byte hopCount = LoRa.read();  // Parsing Payload
      const byte nextHopID = LoRa.read();
      const int rssi = LoRa.packetRssi();
      const float snr = LoRa.packetSnr();
      const unsigned long time = millis();
      
      result = insertRoutingTable(sender, hopCount, nextHopID, rssi, snr, time);
      if(result != 1) {
        return result;        // invalid nodeID
      }
      return setRoutingStatus(); // 1 or -1
    }
    
    if(type == RRequest) {         // Type D: Route Request
      Serial.println("type == RRequest");
      parsePayload();
      result = routePayload( mode, recipient, sender, ttl, 2);  // don't resend when no ack

      if(result == 1) {
        sendAckBack(mode, source);
        return result;          // Success
      } else {
        return result;          // No ACK
      }
    }
    return Node_Mode_Err;                // Node Mode ERR
  } 
  
  if(mode == 1) {                // Master Mode
    if(type == DirectPl) {           // Type C: Direct PL
      result = parsePayload();
      if(result == 1) {
        sendAckBack(mode, source);
        return result;          // Success
      }
      return result;            // Parse ERR
    } 
    return Master_Mode_Err;                // Master Mode ERR
  }
  
  // Checking the ackMode that was passed from waitForAck()
  // The address space in integers between 17 and 170.
  // Node 1 - 0x11 - 17
  // Node 2 - 0x22 - 34
  // Master Node - 0xAA - 170
  
  if(mode > 16 && mode < 171) {          // ACK Mode
    if(type == ACK) {
      return Success;               // Success
    } 
    return ACK_Mode_Err;                // ACK Mode ERR
  }
  return Frame_Handler_Err;                  // frHandler ERR
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
      RSBcastS,         // type: Type B
      0xFF,             // router: BCAST
      0xFF,             // recipient: BCAST
      localAddress,     // sender
      0x0F              // ttl
    );
  } else if(mode == 1) {   // Master Mode
    sendFrame(
      mode,
      RSBcastM,         // type: Type A
      0xFF,             // router: BCAST
      0xFF,             // recipient: BCAST
      localAddress,     // sender
      0x0F              // ttl
    );
  }
  return Success;
}

#ifdef MESH_MASTER_MODE
int getNodeRxCounter(const byte nodeId){
  switch(nodeId) {
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

void incNodeRxCounter(const int nodeId) {
  switch(nodeId) {
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
  }
}

void printNodeInfo(){
  const int nodeId = *(int *) (&payload[0]);
  const int hopCount = *(int *) (&payload[4]);
  const int nextHop = *(int *) (&payload[8]);
  const int linkRssi = *(int *) (&payload[12]);
  const int attemptedPayloadTx = *(int *) (&payload[16]);
  incNodeRxCounter(nodeId);
  const int nodeRx = getNodeRxCounter(nodeId);

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