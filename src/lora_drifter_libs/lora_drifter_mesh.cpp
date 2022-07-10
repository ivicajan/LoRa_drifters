#include "lora_drifter_mesh.h"

#include "lora_drifter_data_types.h"

#include <LoRa.h>

#include "axp20x.h" //Power management on the TTGo T-beam

// Timing Parameters
#define DELETION_TIME             (62000)   // Reset the routing table if entry's time is older than 62s
#define ARQ_TIME                  (2000)    // Automatic Repeat Request for every 2s

// Comment this out when flashing a servant node.
// TODO: move this to a better place
#define MESH_MASTER_MODE

// Uncommenting this prevents the master from being added to the routing table, this allows for a servant node to be
// forced to hop nodes, to reach master.
// #define DEBUG_HOP

extern AXP20X_Class PMU;
#ifdef MESH_MASTER_MODE
extern String csv_out_str;
extern String message_log;
#define MAX_NUM_LOGS              (30)
static volatile int num_logs = 0;
extern TinyGPSPlus gps;
extern Master master;
extern Servant servants[NUM_MAX_SERVANTS];           // Servants data array
extern SemaphoreHandle_t servant_mutex;
#else
extern Packet packet;
extern volatile uint32_t last_packet_received_time_ms;
#endif //MESH_MASTER_MODE
extern SemaphoreHandle_t lora_mutex;

extern byte routing_table[ROUTING_TABLE_SIZE];
extern byte payload[24];
extern byte local_hop_count;
extern byte local_next_hop_ID;
extern byte local_address;
// DIAGNOSTICS
extern int node_rx[NUM_NODES];
extern volatile int messages_sent;
extern volatile int messages_received;

#ifndef MESH_MASTER_MODE
extern volatile int local_link_rssi;
extern volatile int master_rx;
#endif // MESH_MASTER_MODE

enum class ErrorType : int {
  InvalidNodeID    = -9,
  NoACK            = -8,
  MasterModeErr    = -6,
  NodeModeErr      = -5,
  Frame_handlerErr = -4,
  ACKModeErr       = -3,
  PayloadErr       = -2,
  Invalid          = -1,
};

enum class ResultType : int {
  Failure = 0,
  Success = 1,
};

static int parse_payload() {
#ifdef MESH_MASTER_MODE
  Packet packet;
#endif // MESH_MASTER_NODE
  xSemaphoreTake(lora_mutex, portMAX_DELAY);
  if(LoRa.available() != sizeof(Packet)) {
    xSemaphoreGive(lora_mutex);
    return static_cast<int>(ResultType::Failure);
  }
  uint8_t buffer[sizeof(Packet)];
  for(size_t ii = 0; ii < sizeof(Packet); ii++) {
    buffer[ii] = LoRa.read();
  }
  memcpy(&packet, buffer, sizeof(Packet));
#ifdef MESH_MASTER_MODE
  // Get ID and then send to class for decoding
  const String name = String(packet.name);
  if(!strcmp(name.substring(0, 1).c_str(), "D")) {
    Serial.print("Drifter ");
    Serial.print(name);
    Serial.println(" found!");
    const int id = name.substring(1, 3).toInt();
    xSemaphoreTake(servant_mutex, portMAX_DELAY);
    servants[id].ID = id;
    servants[id].decode(&packet);
    servants[id].rssi = LoRa.packetRssi();
    servants[id].update_dist_bear(master.lng, master.lat);
    servants[id].active = true;
    Serial.println("RX from LoRa - decoding completed");
    const String t_date = String(servants[id].year) + "-" + String(servants[id].month) + "-" + String(servants[id].day);
    const String t_time = String(servants[id].hour) + ":" + String(servants[id].minute) + ":" + String(servants[id].second);
    const String t_location = String(servants[id].lng, 6) + "," + String(servants[id].lat, 6) + "," + String(servants[id].age);
    csv_out_str += "D" + String(id) + "," + t_date + "," + t_time + "," + t_location  + "," + String(servants[id].batt_percent, 2) + '\n';
    xSemaphoreGive(servant_mutex);
  }
  else {
    Serial.println("Not a complete drifter packet");
    xSemaphoreGive(lora_mutex);
    return static_cast<int>(ErrorType::PayloadErr);
  }
#endif // MESH_MASTER_MODE
  xSemaphoreGive(lora_mutex);
  return static_cast<int>(ResultType::Success);
}

// To help compiler
static int set_routing_status();
static int frame_handler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl);

static bool validate_ID(const byte node_ID) {
  switch(node_ID) {
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

static int id_to_index(const byte node_ID) {
  if(node_ID == MASTER_LOCAL_ID) return 0;
  return (node_ID == 0) ? 0 : node_ID / 0x10;
}

byte index_to_id(const int idx) {
  if(idx == 0) return MASTER_LOCAL_ID;
  return idx * 0x10 + idx;
}

static void inc_node_rx_counter(const byte node_ID) {
#ifndef MESH_MASTER_MODE
  if(node_ID == MASTER_LOCAL_ID) {
    master_rx++;
  }
  else 
#endif // MESH_MASTER_MODE
  if(node_ID != MASTER_LOCAL_ID) {
    node_rx[id_to_index(node_ID)]++;
  }
}

static void print_routing_table() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte node_ID = routing_table[idx * ROUTING_TABLE_ENTRY_SIZE];
    const byte hop_count = routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const byte hop_ID = routing_table[(idx*ROUTING_TABLE_ENTRY_SIZE) + 2];
    const int rssi = *(int *)(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3]);
    const float snr = *(float *)(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7]);
    const unsigned long current_time = *(unsigned long*)(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11]);
    if(rssi) {
      Serial.print("Routing Table Entry ");
      Serial.print(idx);
      Serial.println(": ");
      
      Serial.print("node_ID: 0x");
      Serial.println(node_ID, HEX);
      
      Serial.print("hop_count: ");
      Serial.println(hop_count);
      
      Serial.print("hop_ID: 0x");
      Serial.println(hop_ID, HEX);
      
      Serial.print("rssi: ");
      Serial.println(rssi);
      
      Serial.print("snr: ");
      Serial.println(snr);
      
      Serial.print("time: ");
      Serial.println(current_time);
      Serial.println(" ");
    }
  }
}

static int insert_routing_table(const byte node_ID, const byte hop_count, const byte hop_ID, const int rssi, const float snr, const unsigned long current_time) {
  if(validate_ID(node_ID) && validate_ID(hop_ID)) { // validate node id and hop id
#ifdef DEBUG_HOP
    if(node_ID == MASTER_LOCAL_ID) {
      return static_cast<int>(ErrorType::InvalidNodeID);
    }
#endif // DEBUG_HOP
    Serial.print("Added 0x");
    Serial.print((int)node_ID, HEX);
    Serial.println(" to routing table");
    const int idx = id_to_index(node_ID);
    memcpy(&routing_table[idx * ROUTING_TABLE_ENTRY_SIZE], &node_ID, sizeof(node_ID));
    memcpy(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1], &hop_count, sizeof(hop_count));
    memcpy(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 2], &hop_ID, sizeof(hop_ID));
    memcpy(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 3], &rssi, sizeof(rssi));
    memcpy(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 7], &snr, sizeof(snr));
    memcpy(&routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 11], &current_time, sizeof(current_time));
#ifndef MESH_MASTER_MODE
    last_packet_received_time_ms = millis();
    delay(10); // may not need this
#endif //MESH_MASTER_MODE
    return static_cast<int>(ResultType::Success);
  }
  return static_cast<int>(ErrorType::InvalidNodeID);
}

bool run_every(const unsigned long interval) {
  static uint32_t previous_millis = 0;
  const uint32_t current_millis = millis();
  if(current_millis - previous_millis >= interval) {
    previous_millis = current_millis;
    return true;
  }
  return false;
}

bool loop_run_every(const unsigned long interval) {
  static uint32_t previous_millis = 0;
  const uint32_t current_millis = millis();
  if(current_millis - previous_millis >= interval) {
    previous_millis = current_millis;
    return true;
  }
  return false;
}

static void delete_old_entries() {
  // Reset when entry's time is older than DELETION_TIME
  const uint32_t current_time = millis();
  for(int ii = 0; ii < NUM_NODES; ii++) {
    const int time_index = (ii * ROUTING_TABLE_ENTRY_SIZE) + 11;

    const long int last_time = *(long int *)(&routing_table[time_index]);
    const int difference = current_time - last_time;

    // Sets all 19 fields of an entry to 0x00
    if(difference > DELETION_TIME) {
      for(int jj = 0; jj < 18; jj++) {
        const int new_index = (ii * ROUTING_TABLE_ENTRY_SIZE) + jj;
        routing_table[new_index] = 0x00;
      }
    }
  }
}

static bool check_if_empty() {
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte entry = routing_table[idx * ROUTING_TABLE_ENTRY_SIZE];
    if(entry != 0x00) {
      return false;
    }
  }
  return true;
}

static bool is_master_in_table() {
  //Checks if a node_ID matches the Master ID MASTER_LOCAL_ID.
  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte entry = routing_table[idx * ROUTING_TABLE_ENTRY_SIZE];
    if(entry == MASTER_LOCAL_ID) {
      return true;
    }
  }
  return false;
}

static int find_min_hop_count() {
  int min_hop_count = 255;

  for(int idx = 0; idx < NUM_NODES; idx++) {
    const byte hop_count = routing_table[(idx * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const int current_hop_count = static_cast<int>(hop_count);
    if((current_hop_count != 0) && (current_hop_count < min_hop_count)) {
      min_hop_count = current_hop_count;
    }
  }
  return min_hop_count;
}


static bool check_frame_header(const int mode, const byte size_header, const byte type, const byte router, const byte source, const byte recipient, 
                      const byte sender, const byte ttl, const byte size_payload) {
  // Check if header values are valid
  if(size_header != 0x08) {
    Serial.println("check_frame_header: invalid size_header");
    return false;
  }

  if(type < static_cast<byte>(MessageType::RouteBroadcastMaster) || type > static_cast<byte>(MessageType::Restart)) {
    Serial.println("check_frame_header: invalid type");
    return false; 
  }

  if(!validate_ID(router)) {
    Serial.println("check_frame_header: invalid router");
    return false;
  }

  if(!validate_ID(source)) {
    Serial.println("check_frame_header: invalid source");
    return false;
  }

  if(!validate_ID(recipient)) {
    Serial.println("check_frame_header: invalid recipient");
    return false;
  }

  if(!validate_ID(sender)) {
    Serial.println("check_frame_header: invalid sender");
    return false;
  }

  if(ttl > 0x0F || ttl == 0x00) {
    Serial.println("check_frame_header: invalid ttl");
    return false;
  }

  if(size_payload != 0x02 && size_payload != 0x18 && size_payload != 0x00) {
    Serial.println("check_frame_header: invalid size_payload");
    return false; 
  }

  // type and router ID
  if(mode == SERVANT_MODE) {
    if(type == static_cast<byte>(MessageType::DirectPayload) || type == static_cast<byte>(MessageType::ACK)) {
      Serial.println("check_frame_header: invalid type for Node Mode");
      return false;
    }
    if(type == static_cast<byte>(MessageType::RouteBroadcastMaster) && sender != MASTER_LOCAL_ID) {
      Serial.println("check_frame_header: Invalid Type && sender ID");
      return false;
    }
    if(type == static_cast<byte>(MessageType::Restart)) {
      return true;
    }
    if(router != local_address && router != 0xFF) {
        Serial.println("check_frame_header: Not addressed to local");
        return false;
    }
    return true;
  }

  if(mode == MASTER_MODE) {
    if(type != static_cast<byte>(MessageType::DirectPayload)) { // Type C: Direct Master
      return false;
    }
    if(router != local_address) {
      return false;
    }
    return true;
  }
  if(mode > 0x10 && mode < 0xBC) {
    if(type != static_cast<byte>(MessageType::ACK)) {
      return false;
    }
    if(router != local_address) {
      return false;
    }
    return true;
  }
  return false;
}

static void type_to_printout(const byte type, const byte router) {
  switch(static_cast<MessageType>(type)) {
    case MessageType::RouteBroadcastServant:
      Serial.print("Sending slave broadcast packet: 0x");
      break;
    case MessageType::RouteBroadcastMaster:
      Serial.print("Sending master broadcast packet: 0x");
      break;
    case MessageType::DirectPayload:
      Serial.print("Sending direct payload packet to: 0x");
      break;
    case MessageType::RouteRequest:
      Serial.print("Sending GPS packet to: 0x");
      break;
    case MessageType::ACK:
      Serial.print("Sending ACK packet to: 0x");
      break;
    case MessageType::Restart:
      Serial.print("Sending Restart packet to: 0x");
      break;
    default:
      break;
  }
  Serial.println(static_cast<int>(router), HEX);
}

void send_frame(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl) {
  // Send a complete header with a random delay
  messages_sent++;
  byte header[8] = "";
  header[0] = 0x08;         // size_header
  header[1] = type;
  header[2] = router;
  header[3] = local_address; // source
  header[4] = recipient;
  header[5] = sender;
  header[6] = ttl - 1;      // ttl

  delay(random(20));
  if(mode == SERVANT_MODE) {
    switch(static_cast<MessageType>(type)) {
      case MessageType::RouteBroadcastServant:
        header[7] = 0x02;           // size_payload
        xSemaphoreTake(lora_mutex, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.write(local_hop_count);  // RS payload
        LoRa.write(local_next_hop_ID); // RS payload
        LoRa.endPacket(true);
        xSemaphoreGive(lora_mutex);
        type_to_printout(type, router);
        break;
      case MessageType::DirectPayload:
      case MessageType::RouteRequest:
        header[7] = 0x18;
        xSemaphoreTake(lora_mutex, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
#ifndef MESH_MASTER_MODE // for compiling
        LoRa.write((const uint8_t *)&packet, sizeof(Packet));
#endif // MESH_MASTER_MODE
        LoRa.endPacket(true);
        xSemaphoreGive(lora_mutex);
        type_to_printout(type, router);
        break;
      case MessageType::ACK:
      case MessageType::Restart:
        header[7] = 0x00;
        xSemaphoreTake(lora_mutex, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        xSemaphoreGive(lora_mutex);
        type_to_printout(type, router);
        break;
      default:
        Serial.println("Not valid for this mode");
        break;
      }
  }
  else if(mode == MASTER_MODE) {
    switch(static_cast<MessageType>(type)) {
      case MessageType::RouteBroadcastMaster:
      case MessageType::ACK:
      case MessageType::Restart:
        header[7] = 0x00;
        xSemaphoreTake(lora_mutex, portMAX_DELAY);
        LoRa.beginPacket();
        LoRa.write(header, 8);
        LoRa.endPacket(true);
        xSemaphoreGive(lora_mutex);
        type_to_printout(type, router);
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
static void send_ack_back(const int mode, const byte source) {
  delay(random(5));
  send_frame(mode, static_cast<byte>(MessageType::ACK), source, source, local_address,  0x0F);
}

int listener(const int frame_size, const int mode) {
  if(!frame_size) {
    return static_cast<int>(ResultType::Failure);             // nothing to receive
  }
  // Parse Header
  xSemaphoreTake(lora_mutex, portMAX_DELAY);
  const byte size_header = LoRa.read();
  const byte type = LoRa.read();
  const byte router = LoRa.read();
  const byte source = LoRa.read();
  const byte recipient = LoRa.read();
  const byte sender = LoRa.read();
  const byte ttl = LoRa.read();
  const byte size_payload = LoRa.read();
  xSemaphoreGive(lora_mutex);

  const bool valid_header = check_frame_header(mode, size_header, type, router, source, recipient, sender, ttl, size_payload);
  if(valid_header) {
    messages_received++;
    inc_node_rx_counter(source);
    PMU.setChgLEDMode(AXP20X_LED_LOW_LEVEL); // LED full on
    delay(1);
    PMU.setChgLEDMode(AXP20X_LED_OFF); // LED off

    if(type == static_cast<byte>(MessageType::DirectPayload)) { // this will go direct to master
      Serial.print("Received DirectPayload packet from: 0x");
      Serial.println(static_cast<int>(sender), HEX);
      Serial.print("Routed from: 0x");
      Serial.println(static_cast<int>(source), HEX);
#ifdef MESH_MASTER_MODE
      if(num_logs >= MAX_NUM_LOGS) {
        message_log = "";
        num_logs = 0;
      }
      String routed_string = "";
      if(sender != source) {
        routed_string = " routed by D" + String(id_to_index(source));
      }
      const String t_date = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day());
      const String t_time = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      message_log += "<tr><td>" + t_date + " " + t_time  + "</td><td>Received packet from D" + String(id_to_index(sender)) + routed_string + "</td></tr>";
      num_logs++;
#endif //MESH_MASTER_MODE
    }
    else if(type == static_cast<byte>(MessageType::RouteRequest)) { // this is a hop
      Serial.print("Received RouteRequest packet from: 0x");
      Serial.println(static_cast<int>(sender), HEX);
      Serial.print("Routed from: 0x");
      Serial.println(static_cast<int>(source), HEX);
    }
    return frame_handler(mode, type, router, source, recipient, sender, ttl); // 1 or E (-6 to -1)
  }
  return static_cast<int>(ResultType::Failure);
}

static bool wait_for_ack(const byte router) {
  int maxloops = 0;
  int result = 0;
  const int interval = ARQ_TIME * local_hop_count;
  const int ackMode = static_cast<int>(router);

  while(maxloops < interval && result != static_cast<byte>(ResultType::Success)) {
    result = listener(LoRa.parsePacket(), ackMode);
    delay(1);
    maxloops++;
  }
  return result == static_cast<byte>(ResultType::Success); // ACK received from router or not
}

static int ack_handshake(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl, int resend) {
  // If no ACK received, resend two more times.
  send_frame(mode, type, router, recipient, sender, ttl);
  bool ack = wait_for_ack(router);

  while(!ack && resend < 2) {
    send_frame(mode, type, router, recipient, sender, ttl);
    ack = wait_for_ack(router);
    resend++;
  }
  return !ack ? static_cast<byte>(ResultType::Failure) : static_cast<byte>(ResultType::Success);
}

int route_payload(const int mode, const byte recipient, const byte sender, const byte ttl, const int resend) {
  // Send the data based on routing status
  // Check first if the local_next_hop_ID is the Master ID
  const byte type = (recipient == local_next_hop_ID) ? static_cast<byte>(MessageType::DirectPayload) : static_cast<byte>(MessageType::RouteRequest);
  const byte router = local_next_hop_ID;
  const int result = ack_handshake(mode, type, router, recipient, sender, ttl, resend);
  Serial.print("Route payload ACK handshake result with router (0x");
  Serial.print(static_cast<int>(router), HEX);
  Serial.print("): ");
  if(result == static_cast<int>(ResultType::Failure)) {
    Serial.println("Failed.");
    return static_cast<int>(ErrorType::NoACK);
  }
  Serial.println("Success!");
  return static_cast<int>(ResultType::Success);
}

//Process data frame
static int frame_handler(const int mode, const byte type, const byte router, const byte source, const byte recipient, const byte sender, const byte ttl) {
  int result = 0;
  if(mode == SERVANT_MODE) {
    switch(static_cast<MessageType>(type)) {
      case MessageType::RouteBroadcastMaster: {
        xSemaphoreTake(lora_mutex, portMAX_DELAY);
        const int rssi = LoRa.packetRssi();
        const float snr = LoRa.packetSnr();
        xSemaphoreGive(lora_mutex);
        const unsigned long time = millis();
        result = insert_routing_table(sender, 0x01, MASTER_LOCAL_ID, rssi, snr, time);
        if(result != static_cast<int>(ResultType::Success)) {
          return result;
        }
        return set_routing_status();
      }
      case MessageType::RouteBroadcastServant: {
        xSemaphoreTake(lora_mutex, portMAX_DELAY);
        const byte hop_count = LoRa.read();     // Parsing Payload
        const byte next_hop_ID = LoRa.read();
        const int rssi = LoRa.packetRssi();
        const float snr = LoRa.packetSnr();
        xSemaphoreGive(lora_mutex);
        const unsigned long time = millis();
        result = insert_routing_table(sender, hop_count, next_hop_ID, rssi, snr, time);
        if(result != static_cast<int>(ResultType::Success)) {
          return static_cast<int>(ErrorType::InvalidNodeID);
        }
        return set_routing_status();
      }
      case MessageType::RouteRequest: {
        parse_payload();
        result = route_payload(mode, recipient, sender, ttl, 0);
        if(result == static_cast<int>(ResultType::Success)) {
          send_ack_back(mode, source);
          return static_cast<int>(ResultType::Success);
        }
        else {
          return static_cast<int>(ErrorType::NoACK);
        }
      }
      case MessageType::Restart: {
        if(recipient == local_address) {
          send_ack_back(mode, source);
          Serial.println("Restarting device");
          ESP.restart();
        }
      }
      default:
        return static_cast<int>(ErrorType::NodeModeErr);
    }
  }
  else if(mode == MASTER_MODE) {
    switch(static_cast<MessageType>(type)) {
      case MessageType::DirectPayload: {
        result = parse_payload();
        if(result == static_cast<int>(ResultType::Success)) {
          Serial.println("Successfully parsed packet, now sending ack");
          send_ack_back(mode, source);
          return result;          // Success
        }
        return result;            // Parse ERR
      } 
      default:
        return static_cast<int>(ErrorType::MasterModeErr);
    }
  }
  // Checking the ackMode that was passed from wait_for_ack()
  // The address space in integers between 17 and 170.
  // Node 1 - 0x11 - 17
  // Node 2 - 0x22 - 34
  // Master Node - MASTER_LOCAL_ID - 184

  else if(mode > 0x10 && mode < 0xBC) { // ACK Mode
    if(type == static_cast<int>(MessageType::ACK)) {
      return static_cast<int>(ResultType::Success);
    } 
    return static_cast<int>(ErrorType::ACKModeErr);
  }
  return static_cast<int>(ErrorType::Frame_handlerErr);
}

// Send a broadcast to the network
int bcast_routing_status(const int mode) {
  if(mode == SERVANT_MODE) {
    const int result = set_routing_status();
    if(result == static_cast<int>(ErrorType::Invalid)) {
      return result;
    }
    send_frame(mode, static_cast<byte>(MessageType::RouteBroadcastServant), 0xFF, 0xFF, local_address, 0x0F);
  }
  else if(mode == MASTER_MODE) {
    send_frame(mode, static_cast<byte>(MessageType::RouteBroadcastMaster), 0xFF, 0xFF, local_address, 0x0F);
  }
  return static_cast<int>(ResultType::Success);
}

#ifdef MESH_MASTER_MODE
static int get_node_rx_counter(const byte node_ID) {
  return node_rx[id_to_index(node_ID)];
}

static void print_node_info() {
  const int node_ID = *(int *)(&payload[0]);
  const int hop_count = *(int *)(&payload[4]);
  const int next_hop = *(int *)(&payload[8]);
  const int link_rssi = *(int *)(&payload[12]);
  const int attempted_payload_tx = *(int *)(&payload[16]);
  inc_node_rx_counter(node_ID);
  const int node_x_rx = get_node_rx_counter(node_ID);

  Serial.print("    Received payload from Node ID 0x");
  Serial.print(node_ID, HEX);
  Serial.println(":");

  Serial.print("        hop_count:   ");
  Serial.println(hop_count);

  Serial.print("        next_hop:    0x");
  Serial.println(next_hop, HEX);

  Serial.print("        link_rssi:   ");
  Serial.println(link_rssi);

  Serial.print("        Attempted Payload Transmissions:                   ");
  Serial.println(attempted_payload_tx);

  Serial.print("        Total Payloads Received from this Node:            ");
  Serial.println(node_x_rx);
}
#endif //MESH_MASTER_MODE

static int find_max_rssi(const int min_hop_count) {
  //To make sure the next_hop of that entry is not local
  int max_rssi = -10000000;
  byte best_route = 0x00;

  for(int ii = 0; ii < NUM_NODES; ii++) {
    const byte hop_count = routing_table[(ii * ROUTING_TABLE_ENTRY_SIZE) + 1];
    const byte next_hop_ID = routing_table[(ii * ROUTING_TABLE_ENTRY_SIZE) + 2];

    // maintain the current_rssi as max_rssi.
    if((hop_count == min_hop_count) && (next_hop_ID != local_address)) {
      const int current_rssi = *(int *)(&routing_table[(ii * ROUTING_TABLE_ENTRY_SIZE) + 3]);
      if(current_rssi > max_rssi) {
        max_rssi = current_rssi;
        best_route = routing_table[ii * ROUTING_TABLE_ENTRY_SIZE];
      }
    }
  }
#ifndef MESH_MASTER_MODE
  local_link_rssi = max_rssi;
#endif // MESH_MASTER_MODE
  return static_cast<int>(best_route);
}

// Returns Success or Invalid
static int set_routing_status() {
  // Update local_hop_count and local_next_hop
  delete_old_entries();
  if(!check_if_empty()) {
    const bool master_found = is_master_in_table();
    if(!master_found) {
      const int min_hop_count_int = find_min_hop_count();
      const int best_route_int = find_max_rssi(min_hop_count_int);

      const byte min_hop_count = static_cast<byte>(min_hop_count_int);
      const byte best_route = static_cast<byte>(best_route_int);
      if(min_hop_count != 0x00 && best_route != 0x00) {
        // valid route
        local_hop_count = min_hop_count + 0x01;
        local_next_hop_ID = best_route;
#ifndef MESH_MASTER_MODE
        local_link_rssi =  *(int *)(&routing_table[3]);
#endif // MESH_MASTER_MODE
        return static_cast<int>(ResultType::Success);
      }
      else {
        return static_cast<int>(ErrorType::Invalid);
      }
    }
    else {
      // master is inside the routing table
      local_next_hop_ID = MASTER_LOCAL_ID;
      local_hop_count = 0x01;
      return static_cast<int>(ResultType::Success);
    }
  }
  else {
    // empty table
    local_next_hop_ID = 0x00;
    local_hop_count = 0x00;
    return static_cast<int>(ErrorType::Invalid);
  }
}
