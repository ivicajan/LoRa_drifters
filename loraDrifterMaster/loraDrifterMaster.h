#ifndef LORADRIFTERMASTER_H
#define LORADRIFTERMASTER_H

#define nSamplesFileWrite 300      // Number of samples to store in memory before file write

// G. Classes for Master and Servant Data
class Servant {
  public:
    Servant() = default;
    void decode(Packet * packet);
    void updateDistBear(const double fromLon, const double fromLat);
    int ID = 0;
    int drifterTimeSlotSec = 0;
    int lastUpdateMasterTime = 0;
    int year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    double lng = 0.0;
    double lat = 0.0;
    uint32_t age = 0;
    int nSamples = 0;
    int dist = 0;
    float bear = 0.f;
    int rssi = 0;
    bool active = false;
    ~Servant() = default;
};

void Servant::updateDistBear(const double fromLon, const double fromLat) {
   dist = (int)TinyGPSPlus::distanceBetween(fromLat, fromLon, lat, lng);
   bear = TinyGPSPlus::courseTo(fromLat, fromLon, lat, lng);
}

void Servant::decode(Packet * packet) {
    drifterTimeSlotSec = packet->drifterTimeSlotSec;
    lastUpdateMasterTime = millis();
    year = packet->year;
    month = packet->month;
    day = packet->day;
    hour = packet->hour;
    minute = packet->minute;
    second = packet->second;
    lng = packet->lng;
    lat = packet->lat;
    age = packet->age;
    nSamples = packet->nSamples;
// #ifdef DEBUG_MODE
//     Serial.println(drifterTimeSlotSec);
//     Serial.println(lastUpdateMasterTime);
//     Serial.println(year);
//     Serial.println(month);
//     Serial.println(day);
//     Serial.println(hour);
//     Serial.println(minute);
//     Serial.println(second);
//     Serial.println(lng);
//     Serial.println(lat);
//     Serial.println(age);
//     Serial.println(nSamples);
// #endif //DEBUG_MODE
}

// H. This is the string literal for the main web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>UWA LoRa Drifters</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="refresh" content="1" >
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
      table {
        width: 100%%;
      }
      table, th, td {
        border: 1px solid black;
      }
      body {
        max-width: 80%%; margin:0px auto; padding-bottom: 25px;
      }
      .switch {
        position: relative; display: inline-block; width: 120px; height: 68px
      } 
      .switch input {
        display: none
      }
      .slider {
        position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 6px
      }
      .slider:before {
        position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px
      }
      input:checked + .slider {
        background-color: #b30000
      }
      input:checked + .slider:before {
        -webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)
      }
    </style>
  </head>
  <body>
    <h2>LoRa Drifters</h2>
    <h4>Master Node</h4>
    <table>
      <tr>
        <td><b>GPS Time</b></td>
        <td><b>Lon</b></td>
        <td><b>Lat</b></td>
        <td><b>GPS Age [ms]</b></td>
        <td><b>Download Data </b></td>
        <td><b>Last File Write GPS Time</b></td>
        <td><b>Erase Data (NO WARNING)</b></td>
      </tr>
      %MASTER%
    </table>
    <br><br>
    <h4>Servants</h4>
    <table>
      <tr>
        <td><b>ID</b></td>
        <td><b>Lora Update Plan [s]</b></td>
        <td><b>Last Update [s]</b></td>
        <td><b>Time</b></td>
        <td><b>Lon</b></td>
        <td><b>Lat</b></td>
        <td><b>Dist [m]</b></td>
        <td><b>Bearing [degN to]</b></td>
        <td><b>Count</b></td>
        <td><b>RSSI</b></td>
      </tr>
      %SERVANTS%
    </table>
    <br><br>
    <h4>Diagnostics</h4>
    <table>
      <tr>
        <td><b>Sent</b></td>
        <td><b>Recvd</b></td>
        <td><b>node1</b></td>
        <td><b>node2</b></td>
        <td><b>node3</b></td>
        <td><b>node4</b></td>
        <td><b>node5</b></td>
        <td><b>node6</b></td>
        <td><b>node7</b></td>
      </tr>
      %DIAGNOSTICS%
    </table>
    <br><br>
  </body>
</html>
)rawliteral";

#endif //LORADRIFTERMASTER_H
