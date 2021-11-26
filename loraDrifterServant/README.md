# Timing Parameter Details
RS_BCAST_TIME 6000: Time intervals, broadcast for every 6000ms

PL_TX_TIME 10000: Receive pay load for every 10000ms

DELETION_TIME 62000: Reset the routing table if entry's time is older than 62000ms

ARQ_TIME 2000: Automatic Repeat Request for every 2000ms

# Configure for multiple servant nodes
The drifterName is the ID of drifters, need to be changed to a different name.

The localAddress identifies the header of each nodes, need to be changed according to the comments in the head of the code.

drifterName = "D01"; localAddress = 0x11;
drifterName = "D02"; localAddress = 0x22;
etc.

# Mesh info

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
