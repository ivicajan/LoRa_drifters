#pragma once

#include "Arduino.h" // for byte type

// Timing Parameters
#define RS_BCAST_TIME             (17000)   // Time intervals, broadcast for every 17s
#define PL_TX_TIME                (12000)   // Receive pay load for every 12s

#define NUM_NODES                 (11)
#define ROUTING_TABLE_ENTRY_SIZE  (19)
#define MASTER_LOCAL_ID           (0xBB)
#define ROUTING_TABLE_SIZE        (MASTER_LOCAL_ID + 0x11)

#define SERVANT_MODE              (0)
#define MASTER_MODE               (1)

enum class MessageType: byte {
  RouteBroadcastMaster  = 0x41,
  RouteBroadcastServant = 0x42,
  DirectPayload         = 0x43,
  RouteRequest          = 0x44,
  ACK                   = 0x45,
  Restart               = 0x46
};

/**
 * @brief Send a packet to a master or servant node.
 * 
 * @param mode either servant or master mode who sent this packet
 * @param type is the message type \ref MessageType that we are sending
 * @param router is the device that is forwarding the packet
 * @param recipient is who is intended to receive this packet
 * @param sender is who sent this packet
 * @param ttl time to live until packet is 'lost'
 */
void send_frame(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl);

/**
 * @brief Listens for packets sent over LoRa and decodes them accordingly
 * 
 * @param frame_size size of the packet decoded
 * @param mode either servant or master mode who sent this packet
 * @return the status of the packet handling
 */
int listener(const int frame_size, const int mode);

/**
 * @brief Loop that checks if a certain amount of time has elapsed. Used for PL_TX_TIME definition time slot.
 * 
 * @param interval to check against
 * @return true if past interval threshold else false
 */
bool run_every(const unsigned long interval);

/**
 * @brief Loop that checks if a certain amount of time has elapsed. Used for RS_BCAST_TIME definition time slot.
 * 
 * @param interval to check against
 * @return true if past interval threshold else false
 */
bool loop_run_every(const unsigned long interval);

/**
 * @brief Convert the drifter index (i.e. 8) to the hex representation (i.e. 0x88)
 * 
 * @return the drifter ID
 */
byte index_to_id(const int idx);

/**
 * @brief Send the positional packet payload to either master or to a forwarding node
 * 
 * @param mode either servant or master mode who sent this packet
 * @param recipient is who is intended to receive this packet
 * @param sender is who sent this packet
 * @param ttl time to live until packet is 'lost'
 * @param resend how many times we reattempt sending the packet
 * 
 * @return the status of the payload routing process
 */
int route_payload(const int mode, const byte recipient, const byte sender, const byte ttl, const int resend);

/**
 * @brief Broadcast the routing status of the drifter to all other reachable nodes.
 * 
 * @param mode either servant or master mode who sent this packet
 * @return the status of the broadcast
 */
int bcast_routing_status(const int mode);
