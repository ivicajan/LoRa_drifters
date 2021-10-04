# Timimg Parameter details
RS_BCAST_TIME 6000: Time intervals, broadcast for every 6000ms
PL_TX_TIME 12000: Receive pay load for every ms
DELETION_TIME 62000: Reset the routing table if entry's time is older than 62000ms
ARQ_TIME 2000: Automatic Repeat Request for every 2000ms

# Configure for multiple servant nodes
The drifterName is the ID of drifters, need to be changed to a different name.
The localAddress identifies the header of each nodes, need to be changed according to the comments in the head of the code.
