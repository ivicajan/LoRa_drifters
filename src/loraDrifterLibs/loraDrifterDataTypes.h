#ifndef LORADRIFTERDATATYPES_H
#define LORADRIFTERDATATYPES_H

#include <TinyGPS++.h>

#define NUM_MAX_SERVANTS            (11)     // Maximum number of servant drifters (just for setting array size)

class Master {
  public:
    Master() = default;
    double lng = 0.0;
    double lat = 0.0;
    int year = 0;
    uint16_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint32_t age = 0;
    ~Master() = default;
};

typedef union drifterStatus_r {
    uint8_t r;
    struct {
        uint8_t imuUsed : 1;       // 1 == in use
        uint8_t imuError : 1;      // 1 == error
        uint8_t meshUsed : 1;      // 1 == in use
        uint8_t configError : 1;   // 1 == error
        uint8_t lowBattery : 1;    // 1 == error (low battery)
        uint8_t saveError : 1;     // 1 == error
        uint8_t lowStorage : 1;    // 1 == error (low storage)
        uint8_t reserved : 1;
    } b;
} drifterStatus_t;

// 3 + 4 + 2 + (1 * 5) + (2 * 8) + 4 + 4 + 4 + 1 = 43 bytes
#pragma pack(1) // Fixes padding issues
typedef struct {
  char name[3];             // e.g. D01
  int drifterTimeSlotSec;   // e.g. 15
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  double lng;
  double lat;
  uint32_t age;
  float storageUsed;
  float battPercent;
  drifterStatus_t drifterState;
} Packet;

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

#endif //LORADRIFTERDATATYPES_H