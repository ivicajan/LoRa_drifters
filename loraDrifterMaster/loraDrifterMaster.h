#ifndef LORADRIFTERMASTER_H
#define LORADRIFTERMASTER_H

// G. Classes for Master and Servant Data
class Servant {
  public:
    Servant() = default;
    void decode(const Packet * const packet) {
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
      storageUsed = packet->storageUsed;
      battPercent = packet->battPercent;
      drifterState = packet->drifterState;
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
    void updateDistBear(const double fromLon, const double fromLat) {
      dist = (int)TinyGPSPlus::distanceBetween(fromLat, fromLon, lat, lng);
      bear = TinyGPSPlus::courseTo(fromLat, fromLon, lat, lng);
    }
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
    float storageUsed = 0.f;
    float battPercent = 0.f;
    drifterStatus_t drifterState;
    int dist = 0;
    float bear = 0.f;
    int rssi = 0;
    bool active = false;
    ~Servant() = default;
};

// H. This is the string literal for the main web page
static const char index_html[] PROGMEM = R"rawliteral(
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
        max-width: 90%%; margin:0px auto; padding-bottom: 25px;
      }
    </style>
  </head>
  <body>
    <h2>LoRa Drifters</h2>
    <h4>Master Node</h4>
    <h5>Battery %BATTERYPERCENT% %%</h5>
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
    %SERVANTS%
    %DIAGNOSTICS%
    </table>
    %STATUSFLAGS%
    </table>
    %MESSAGELOG%
    </table>
  </body>
</html>
)rawliteral";

#endif //LORADRIFTERMASTER_H
