#pragma once

#include <TinyGPS++.h>

#define NUM_MAX_SERVANTS (11)     // Maximum number of servant drifters (just for setting array size)

/**
 * @brief Class that holds the state of the master drifter and it's current position.
 * 
 */
class Master {
public:
    Master() = default;
    double lng = 0.0;
    double lat = 0.0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    ~Master() = default;
    void fill_master();
    void generate_master();
private:
    int year = 0;
    uint16_t month = 0;
    uint8_t day = 0;
    uint32_t age = 0;
};

/**
 * @brief Bitfield representation of the drifters current state. Can be expanded upon by 
 * changing the union data type and adding new fields.
 * 
 */
typedef union drifter_status_r {
    uint8_t r;
    struct {
        uint8_t imu_used : 1;       // 1 == in use
        uint8_t imu_error : 1;      // 1 == error
        uint8_t mesh_used : 1;      // 1 == in use
        uint8_t config_error : 1;   // 1 == error
        uint8_t low_battery : 1;    // 1 == error (low battery)
        uint8_t save_error : 1;     // 1 == error
        uint8_t low_storage : 1;    // 1 == error (low storage)
        uint8_t reserved : 1;      // used to maintain shape of union struct
    } b;
} drifter_status_t;

// 4 + 4 + 2 + (1 * 5) + (2 * 8) + 4 + 4 + 4 + 1 = 44 bytes
#pragma pack(1) // Fixes padding
typedef struct {
  char name[4];             // e.g. {'D', '0', '1', '\0'}
  int drifter_time_slot_sec;   // e.g. 15
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  double lng;
  double lat;
  uint32_t age;
  float storage_used;
  float batt_percent;
  drifter_status_t drifter_state;
} Packet;

// G. Classes for Master and Servant Data
class Servant {
  public:
    void decode(const Packet * const packet) {
      drifter_time_slot_sec = packet->drifter_time_slot_sec;
      last_update_master_time = millis();
      year = packet->year;
      month = packet->month;
      day = packet->day;
      hour = packet->hour;
      minute = packet->minute;
      second = packet->second;
      lng = packet->lng;
      lat = packet->lat;
      age = packet->age;
      storage_used = packet->storage_used;
      batt_percent = packet->batt_percent;
      drifter_state = packet->drifter_state;
    }
    void update_dist_bear(const double from_lon, const double from_lat) {
      dist = (int)TinyGPSPlus::distanceBetween(from_lat, from_lon, lat, lng);
      bear = TinyGPSPlus::courseTo(from_lat, from_lon, lat, lng);
    }
    int ID = 0;
    int drifter_time_slot_sec = 0;
    int last_update_master_time = 0;
    int year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    double lng = 0.0;
    double lat = 0.0;
    uint32_t age = 0;
    float storage_used = 0.f;
    float batt_percent = 0.f;
    drifter_status_t drifter_state;
    int dist = 0;
    float bear = 0.f;
    int rssi = 0;
    bool active = false;
};
