#ifndef LORADRIFTERMESH_H
#define LORADRIFTERMESH_H

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

void sendFrame(const int mode, const byte type, const byte router, const byte recipient, const byte sender, const byte ttl);

int listener(const int frameSize, const int mode);

bool runEvery(const unsigned long interval);

bool loop_runEvery(const unsigned long interval);

byte indexToId(const int idx);

int routePayload(const int mode, const byte recipient, const byte sender, const byte ttl, const int resend);

// Send a broadcast to the network
int bcastRoutingStatus(const int mode);

#endif // LORADRIFTERMESH_H