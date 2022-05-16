# LoRa T-Beam Ocean Drifters
Ocean drifters using LoRa radio with simple time slicing on a TTGO T-Beam.

TTGO T-Beam modules are used for ocean drifter buoy tracking within the range of
LoRa transmission. The same code can be used onshore as well, for example, for tracking
hikers in a region within LoRa range.

## Master Node
A Master node is used to receive LoRa radio packets from the Servant nodes. The
Master node activates an onboard WiFi Access point to present a `WebServer` to view real-time
data that is recevied by all of the Servant Nodes. The Master node also calculates
distance an bearing to all Servant Nodes in real-time.
The Master node records 1 second GPS positions of the Master node as well as any
packets received from Servant Nodes.

## Servant Node
Each Servant node once turned on, records it's GPS location on the onboard PSRAM
and once per minute sends it's GPS location via LoRa radio. GPS locations at a 1
second interval are saved.

The servant node is configured with an ID `String drifterName = "D06"` and a time slice
`int drifterTimeSlotSec = 28` for to make it's LoRa transmission.
The time slice is a nominated second of the minute at which the transmission will be
started.  GPS time is used for detection of the seconds at which to make the transmission.
In this way, all Servants that are receiving GPS time must be configured to use a
different second in the minute to make the LoRa transmission otherwise collision will occur.

The Servant node can be configured by activating the `WebServer`. The button on the T-Beam is
used to activate and deactivate the `WebServer` on the Servant node.

## Mesh Network (optional)
The LoRa Drifters implementation can work with a mesh network provided the `USING_MESH` is defined
and not commented out (in `loraDrifterServant.ino`). In order to use the mesh network, the user
must name set a servant drifter with a local address `byte localAddress = 0x66` to match their
`String drifterName = "D06"`, for example. No changes need to be made to the masters implementation
provided that `USING_MESH` is defined and not commented out (in `loraDrifterMaster.ino`).

The mesh network works by having the master node and servant nodes using duplex LoRa communications.
The nodes are receiving and transmitting based on time slots and responses to messages. This mesh
network allows for a greater transmission range of messages with the cost of less energy efficiency.

Configurable parameters for mesh network:
<ul>
    <li><code>RS_BCAST_TIME 6000</code>: Time intervals, broadcast for every 6000ms</li>
    <li><code>PL_TX_TIME 10000</code>: Receive pay load for every 10000ms</li>
    <li><code>DELETION_TIME 62000</code>: Reset the routing table if entry's time is older than 62000ms</li>
    <li><code>ARQ_TIME 2000`:</code>Automatic Repeat Request for every 2000ms</li>
</ul>

## IMU (optional)
The Servant drifters may or may not have IMUs installed so using the macro `USING_IMU`, allows for a
user to enable/disable the use of IMU code. When connected to the Master drifter's WiFi network, the
user may calbirate the IMU to allow for easy calibrations in the field.

# SD Storage (optional)
Provided the user has wired the ESP32 device the same way as expected in the Pin Layout Description section, they can uncomment <code>#define USING_IMU</code> to allow the SD card to handle the data storage. Does not currently play with with Async TCP as the download is very slow and times out, but the user should be able to simply remove the SD card to get the data.

# Libraries required:
Flashing ESP32:
https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html#installing-using-boards-manager
- the SD card library used is installed here, and is not the same that can be downloaded from the library manager

Web Server:
https://github.com/me-no-dev/ESPAsyncWebServer
https://github.com/me-no-dev/AsyncTCP

LoRa Communcations:
https://github.com/sandeepmistry/arduino-LoRa

GPS decoding:
https://github.com/mikalhart/TinyGPSPlus

Communicating with the u-blox to reset NMEA Serial output:
https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

Power management on the TTGO T-Beam:
https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries/AXP202X_Library


# Pin Layout Description
| Name | Pin |
|  :----: |  :----: |
| GPS_TX_PIN | 12 |
| GPS_RX_PIN | 34 |
| I2C_SDA | 21 |
| I2C_SCL | 22 |
| PMU_IRQ | 35 |
| RADIO_SCLK_PIN | 5 |
| RADIO_CS_PIN | 18 |
| RADIO_MISO_PIN | 19 |
| RADIO_RST_PIN | 23 |
| RADIO_DI0_PIN | 26 |
| RADIO_MOSI_PIN | 27 |
| HSPI_CS | 2 |
| HSPI_MISO | 4 |
| HSPI_MOSI | 13 |
| HSPI_SCLK | 14 |