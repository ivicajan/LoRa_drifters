#include "src/lora_drifter_libs/lora_drifter.h"

// #define USING_SD_CARD

#ifdef USING_SD_CARD
// Note, the SD card library we are using is from not the library downloaded from 
// the library manager, it is the library from the ESP32 specific files (see README)
#include <SD.h>
// HSPI
#define HSPI_SCLK (14)
#define HSPI_MISO (4)
#define HSPI_MOSI (13)
#define HSPI_CS   (2)
SPIClass * hspi = NULL;
#endif //USING_SD_CARD

#define USING_IMU
// #define WAVE_TANK_TEST

// uncomment OUTPUT_SYSTEM_MONITOR to view heap and stack size
// #define OUTPUT_SYSTEM_MONITOR

#ifdef USING_IMU
#include "mpu/imu.h"
extern BLA::Matrix<3> U_INS;
extern BLA::Matrix<5> X_INS;
extern BLA::Matrix<3> Y_GPS;
extern BLA::Matrix<3> acc_raw;
extern BLA::Matrix<3> Rotation_matrix;
extern BLA::Matrix<8, 8> P;
static BLA::Matrix<2> Updated_GPS; //updated gps after imu kalman modification
extern volatile bool calibrate_imu;
String csv_IMU_file_name = "";
static File imu_file;
String csv_IMU_out_str = "";
static TaskHandle_t imu_task_handle;
#endif // USING_IMU

#define LAST_PACKET_TIMEOUT_ms (600000)
volatile uint32_t last_packet_received_time_ms = 0;

extern AsyncWebServer server;
extern AXP20X_Class PMU;
extern TinyGPSPlus gps;

// GLOBAL VARIABLES
String csv_out_str = "";                 // Buffer for output file
static String last_file_write = "";
static bool web_server_on = false;
static String csv_file_name = "";
static File file;                            // Data file for the SPIFFS output
volatile int n_samples = 0;                         // Counter for the number of samples gathered
static volatile float storage_used = 0.f;                   // MB used in SPIFFS storage

static volatile int gps_last_second = -1;
static String t_time = "";
static String drifter_name = "D09";       // ID send with packet
static volatile int drifter_time_slot_sec = 45;      // seconds after start of each GPS minute

static String ssid_name = "DrifterServant";    // Wifi ssid and password
#define SSID_PASSWORD "Tracker1"

Packet packet;
static drifter_status_t drifter_state; // status flags of the drifter state

SemaphoreHandle_t lora_mutex = NULL;
static SemaphoreHandle_t drifter_state_mutex = NULL;
static TaskHandle_t send_task_handle;
static TaskHandle_t system_monitoring_task_handle;

#ifdef USING_MESH
static TaskHandle_t listen_task_handle;
byte routing_table[ROUTING_TABLE_SIZE] = "";
byte payload[24] = "";
volatile int local_link_rssi = 0;
byte local_hop_count = 0x00;
byte local_next_hop_ID = 0x00;
byte local_address = index_to_id(drifter_name.substring(1, 3).toInt());
// Diagnostics
volatile int messages_sent = 0;
volatile int messages_received = 0;
int node_rx[NUM_NODES] = {0};
volatile int master_rx = 0;
#endif // USING_MESH

static const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <title>UWA LoRa Drifters</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
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
      <h4>Servant Node</h4>
      <h5>Battery %BATTERYPERCENT%</h5>
      %SERVANT%
      </table>
      <br><br>
      <h4>Configuration</h4>
      <form action="/configure" method="get">
        <table>
          <tr>
            <td><b>Setting</b></td>
            <td><b>Current Values</b></td>
            <td><b>New Values</b></td>
            <td><b>Guidance</b></td>
          </tr>
          <tr>
            <td><label for="fname"><b>Drifter ID</b></label></td>
            <td>%DRIFTERID%</td>
            <td><input type="text" id="fname" name="drifterID"></td>
            <td>Drifter IDs from D01 to D10</td>
          </tr>
          <tr>
            <td><label for="lname"><b>LoRa Sending Second</b></label></td>
            <td>%LORASENDSEC%</td>
            <td><input type="text" id="lname" name="loraSendSec"></td>
            <td>Sending second is from 0 to 59 seconds</td>
          </tr>
        </table>
        <input type="submit" value="Configure">
      </form>
      %DIAGNOSTICS%
      </table>
      <br><br>
    </body>
  </html>
)rawliteral";

static void write_data_to_flash() {
#ifdef USING_SD_CARD
  file = SD.open(csv_file_name, FILE_APPEND);
#else
  file = SPIFFS.open(csv_file_name, FILE_APPEND);
#endif //USING_SD_CARD
  if(!file) {
    Serial.println("There was an error opening the file for appending, creating a new one");
#ifdef USING_SD_CARD
    file = SD.open(csv_file_name, FILE_WRITE);
#else
    file = SPIFFS.open(csv_file_name, FILE_WRITE);
#endif //USING_SD_CARD
  }

  if(!file) { // this shouldn't occur
      Serial.println("There was an error opening the file for writing");
      last_file_write = "FAILED OPEN";
      ESP.restart();
  }
  else {
    const uint32_t file_size_before_save = file.size();
    if(file.println(csv_out_str)) {
      Serial.print("Wrote data in file, current size: ");
      const uint32_t file_size_after_save = file.size();
      Serial.print(file_size_after_save);
      Serial.println(" bytes");
      storage_used = file_size_after_save * 0.000001f; // convert to MB
      csv_out_str = "";
      n_samples = 0;
      last_file_write = t_time;
      xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
      drifter_state.b.save_error = (file_size_after_save == file_size_before_save);
      xSemaphoreGive(drifter_state_mutex);
    }
    else {
      last_file_write = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

#ifdef USING_IMU
static void write_IMU_data_to_flash() {
#ifdef USING_SD_CARD
  imu_file = SD.open(csv_IMU_file_name, FILE_APPEND);
#else
  imu_file = SPIFFS.open(csv_IMU_file_name, FILE_APPEND);
#endif //USING_SD_CARD
  if(!imu_file) {
    Serial.println("There was an error opening the imu file for appending, creating a new one");
#ifdef USING_SD_CARD
    imu_file = SD.open(csv_IMU_file_name, FILE_WRITE);
#else
    imu_file = SPIFFS.open(csv_IMU_file_name, FILE_WRITE);
#endif //USING_SD_CARD
  }

  if(!imu_file) { // this shouldn't occur
      Serial.println("There was an error opening the imu file for writing");
      ESP.restart();
  }
  else {
    if(imu_file.println(csv_IMU_out_str)) {
      Serial.print("Wrote data in imu file, current size: ");
      Serial.print(imu_file.size());
      Serial.println(" bytes");
      csv_IMU_out_str = "";
      storage_used += imu_file.size() * 0.000001f; // convert to MB
    }
    else {
      ESP.restart();
    }
  }
  imu_file.close();
  delay(50);
}

static void update_imu() {
  if(mpu.update()) {
    static uint32_t prev_ms = millis();
    static uint32_t imu_ms = millis(); //For reset
    if(millis() > prev_ms + SAMPLE_PERIOD_ms) {
#ifdef WAVE_TANK_TEST
      Y_GPS = {0, 0, Yaw[0]};
#else
      if(Serial1.available() > 0) { // only need 1 measurement here
        gps.encode(Serial1.read());
        measure_gps_data();
      }
      //Ouput current location in lat and lng
      float lat, lng;
      get_current_location(&lat, &lng);
      Updated_GPS = {lat, lng};
#ifdef DEBUG_MODE
      Serial << "Updated Lat: " << lat << " Lng: " << lng << '\n';
#endif //DEBUG_MODE
#endif //WAVE_TASK_TEST
      //Run Kalman with fusion IMU and GPS
      Update_Kalman();
      if(mpu.update()) {
        measure_imu_data();
      }
      rotate_imu_data();
#ifdef DEBUG_MODE
      Serial << " Acc: " << acc << " Yaw[0]: " << float(mpu.getYaw() / 180.f * PI)
             << " pitch: " << float(mpu.getPitch() / 180.f * PI)
             << " Roll: " << float(mpu.getRoll() / 180.f * PI) << " \n ";
#endif // DEBUG_MODE
#ifdef PRINT_DATA_SERIAL
      print_data();
#endif //PRINT_DATA_SERIAL
      prev_ms = millis();
    }
  }
}

static void imu_task(void * params) {
  (void)params;
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // might not need a delay at all
    update_imu();
  }
}
#endif // USING_IMU

#ifdef USING_MESH
static void listen_task(void * params){
  (void)params;
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // might not need a delay at all
    const int result = listener(LoRa.parsePacket(), SERVANT_MODE);
  }
}
#endif // USING_MESH

static void system_monitoring_task(void * params){
  (void)params;
  disableCore1WDT(); // Disable watchdog to keep process alive
  while(1){
#ifdef OUTPUT_SYSTEM_MONITOR
    Serial.println("--------------------System Monitor---------------------");
    Serial.print("Heap remaining: ");
    Serial.println(ESP.getFreeHeap());
#ifdef USING_IMU
    Serial.print("IMU task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(imu_task_handle));
#endif //USING_IMU
#ifdef USING_MESH
    Serial.print("Listen task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(listen_task_handle));
#endif //USING_MESH
    Serial.print("Send task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(send_task_handle));
    Serial.print("System monitoring task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(system_monitoring_task_handle));
    Serial.println("-------------------------------------------------------");
#endif //OUTPUT_SYSTEM_MONITOR
    vTaskDelay(1000); // keep delaying
  }
}

static void send_task(void * params) {
  (void)params;
  disableCore1WDT(); // Disable watchdog to keep process alive
  while(1) {
    if(millis() - last_packet_received_time_ms > LAST_PACKET_TIMEOUT_ms) {
      ESP.restart();
    }

    if(digitalRead(BUTTON_PIN) == LOW) { //Check for button press
      if(web_server_on) {
        web_server_on = false;
#ifdef USING_MESH
        vTaskResume(listen_task_handle);
        Serial.println("Resumed listen_task_handle");
#endif //USING_MESH
        vTaskResume(system_monitoring_task_handle);
        Serial.println("Resumed system_monitoring_task_handle");
#ifdef USING_IMU
        if(imu_task_handle != NULL) {
          vTaskResume(imu_task_handle);
          Serial.println("Resumed imu_task_handle");
        }
#endif //USING_IMU
      } else {
        Serial.println("Turning web server on");
        web_server_on = true;
#ifdef USING_MESH
        vTaskSuspend(listen_task_handle);
        Serial.println("Suspended listen_task_handle");
#endif //USING_MESH
        vTaskSuspend(system_monitoring_task_handle);
        Serial.println("Suspended system_monitoring_task_handle");
#ifdef USING_IMU
        if(imu_task_handle != NULL) {
          vTaskSuspend(imu_task_handle);
          Serial.println("Suspended imu_task_handle");
        }
#endif //USING_IMU
      }
      start_web_server(web_server_on);
      delay(1000);
    }
  if(!web_server_on) {
      generate_packet();
#ifdef USING_MESH
#ifdef IGNORE_GPS_INSIDE
      if(run_every(PL_TX_TIME)) {
#else
      if(gps.time.second() == drifter_time_slot_sec) {
#endif //IGNORE_GPS_INSIDE
        route_payload(SERVANT_MODE, MASTER_LOCAL_ID, local_address, 0x0F, 0);
        PMU.setChgLEDMode(AXP20X_LED_LOW_LEVEL); // LED full on
        delay(50);
        PMU.setChgLEDMode(AXP20X_LED_OFF); // LED off
      }

      if(loop_run_every(RS_BCAST_TIME)) {
        Serial.println("Route broadcast");
        bcast_routing_status(SERVANT_MODE);   // returns 1 or -1
        PMU.setChgLEDMode(AXP20X_LED_LOW_LEVEL); // LED full on
        delay(50);
        PMU.setChgLEDMode(AXP20X_LED_OFF); // LED off
      }
#endif // USING_MESH
      delay(10);
      // Write data to onboard flash if n_samples is large enough
      if(n_samples >= SAMPLES_BEFORE_WRITE) {  // only write after collecting a good number of samples
        Serial.println("Dump data into the memory");
        write_data_to_flash();
#ifdef USING_IMU
        write_IMU_data_to_flash();
#endif // USING_IMU
      }
    }
    else {
      Serial.println("Web server is ON, not GPS data or saving during the time");
      delay(40);
      digitalWrite(BOARD_LED, LED_OFF);
      delay(40);
      digitalWrite(BOARD_LED, LED_ON);
    }
  }
}

// Init the LoRa communications with/without default factory settings
static void init_LoRa(bool default_params = false) {
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if(default_params) {
    /* LoRa paramters - https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md */

    // LoRa.setFrequency(LORA_FREQUENCY);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    // LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    // LoRa.setSyncWord(LORA_SYNC_WORD);
    // LoRa.setGain(LORA_GAIN);
    // Enable CRC --> if packet is corrupted, packet gets dropped silently
    LoRa.enableCrc();
  }

  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(true); // if failed, do nothing
  }
}

//H. Read config file, if it exists
static void read_config_file() {
#ifdef USING_SD_CARD
  file = SD.open("/config.txt", FILE_READ);
#else
  file = SPIFFS.open("/config.txt", FILE_READ);
#endif //USING_SD_CARD
  if(!file) {
    Serial.println("Failed to open config.txt configuration file");
    xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
    drifter_state.b.config_error = 1;
    xSemaphoreGive(drifter_state_mutex);
  }
  else {
    const String inData = file.readStringUntil('\n');
    const int comma = inData.indexOf(",");
    drifter_name = inData.substring(0, comma);
#ifdef USING_MESH
    local_address = index_to_id(drifter_name.substring(1, 3).toInt());
#endif //USING_MESH
    drifter_time_slot_sec = inData.substring(comma + 1).toInt();
    Serial.println(inData);
    file.close();
  }
  delay(50);
  csv_file_name = "/svt" + String(drifter_name) + ".csv";
#ifdef USING_SD_CARD
  file = SD.open(csv_file_name, FILE_APPEND);
#else
  file = SPIFFS.open(csv_file_name, FILE_APPEND);
#endif //USING_SD_CARD
  if(file){
    storage_used = file.size() * 0.000001f; // convert to MB
    file.close();
  }
#ifdef USING_IMU
  csv_IMU_file_name = "/svt" + String(drifter_name) + "_IMU.csv";
#ifdef USING_SD_CARD
  imu_file = SD.open(csv_IMU_file_name, FILE_APPEND);
#else
  imu_file = SPIFFS.open(csv_file_name, FILE_APPEND);
#endif //USING_SD_CARD
  if(imu_file){
    storage_used += imu_file.size() * 0.000001f; // convert to MB
    imu_file.close();
  }
#endif //USING_IMU
}

#ifdef USING_SD_CARD
static void print_volume_size() {
    const uint8_t card_type = SD.cardType();

    if(card_type == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(card_type == CARD_MMC) {
        Serial.println("MMC");
    }
    else if(card_type == CARD_SD) {
        Serial.println("SDSC");
    }
    else if(card_type == CARD_SDHC) {
        Serial.println("SDHC");
    }
    else {
        Serial.println("UNKNOWN");
    }

    const uint64_t card_size = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", card_size);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

static float get_capacity_used() {
  return (float)(SD.usedBytes() / SD.totalBytes());
}
#endif //USING_SD_CARD

void setup() {
  lora_mutex = xSemaphoreCreateMutex();
  drifter_state_mutex = xSemaphoreCreateMutex();
  init_board();
#ifdef USING_SD_CARD
  hspi = new SPIClass(HSPI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS); //SCLK, MISO, MOSI, SS
  pinMode(HSPI_CS, OUTPUT); //HSPI SS

  // see if the card is present and can be initialized:
  if(!SD.begin(HSPI_CS, *hspi)) {
    Serial.println("SD Card failed, or not not present");
  }
  print_volume_size();
#endif //USING_SD_CARD
  delay(500);
  xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
  drifter_state.r = 0;
#ifdef USING_IMU
  const bool init_IMU_ok = init_IMU();
  drifter_state.b.imu_used = 1;
  drifter_state.b.imu_error = !init_IMU_ok;
  delay(500);
#endif // USING_IMU
  xSemaphoreGive(drifter_state_mutex);
  if(!SPIFFS.begin(true)) {
    // TODO: this should attempt to fix SPIFFS
    Serial.println("SPIFFS error has occured");
    return;
  }
  init_LoRa();
  delay(50);
  read_config_file();
#ifdef USING_IMU
  if(init_IMU_ok){
    xTaskCreatePinnedToCore(imu_task, "imu_task", 10000, NULL, 1, &imu_task_handle, 0);
    delay(500);
  }
  else {
    imu_task_handle = NULL;
  }
#endif //USING_IMU
#ifdef USING_MESH
  xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
  // drifter_state.b.mesh_used = 1;
  xSemaphoreGive(drifter_state_mutex);
  xTaskCreatePinnedToCore(listen_task, "listen_task", 5000, NULL, 1, &listen_task_handle, 0);
  delay(500);
#endif //USING_MESH
  xTaskCreatePinnedToCore(send_task, "send_task", 8000, NULL, 1, &send_task_handle, 1);
  xTaskCreatePinnedToCore(system_monitoring_task, "system_monitoring_task", 3000, NULL, 1, &system_monitoring_task_handle, 1);
  delay(500);
  Serial.println("Initialization complete.");
}

static void fill_packet() {
  strcpy(packet.name, drifter_name.c_str());
  packet.drifter_time_slot_sec = drifter_time_slot_sec;
  packet.hour = gps.time.hour();
  packet.minute = gps.time.minute();
  packet.second = gps.time.second();
  packet.year = gps.date.year();
  packet.month = gps.date.month();
  packet.day = gps.date.day();
  packet.lng = gps.location.lng();
  packet.lat = gps.location.lat();
  packet.storage_used = storage_used;
  packet.age = gps.location.age();
  packet.batt_percent = get_battery_percentage();
  xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
#ifdef USING_SD_CARD
  drifter_state.b.low_storage = (get_capacity_used() > 0.75f);
  Serial.printf("%f\n", get_capacity_used());
#else
  drifter_state.b.low_storage = (packet.storage_used > 0.75f * SPIFFS_FLASH_SIZE); // if we are above 75% storage capacity we show the flag 1 (error)
#endif //USING_SD_CARD
  drifter_state.b.low_battery = (packet.batt_percent < 50.f); // if we are below 50% battery we show the flag 1 (error)
  packet.drifter_state = drifter_state;
  xSemaphoreGive(drifter_state_mutex);
#ifdef DEBUG_MODE
  Serial << "Raw Lat: " << static_cast<float>(packet.lat) << " Lng: " << static_cast<float>(packet.lng) << '\n';
#endif // DEBUG_MODE
}

#ifdef USING_MESH
static String node_hops_to_string() {
  String out_str = "";
  for(size_t idx = 1; idx < NUM_NODES; idx++) {
    out_str += String(node_rx[idx]);
    if(idx != NUM_NODES - 1) {
      out_str += ",";
    }
  }
  return out_str;
}
#endif // USING_MESH

static void generate_packet() {
  const uint32_t start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 400);
#ifdef IGNORE_GPS_INSIDE
  if(true) {
#else 
  if(gps.time.second() != gps_last_second) {
#endif // IGNORE_GPS_INSIDE
    // Serial.print("New GPS record from: ");
    // Serial.println(drifter_name);
    fill_packet();
    gps_last_second = gps.time.second();
    // Serial.print("samples: ");
    // Serial.println(n_samples);
#ifdef IGNORE_GPS_INSIDE
    if(true) {
#else 
    if((gps.location.lng() != 0.0) && (gps.location.age() < 1000)) {
#endif // IGNORE_GPS_INSIDE
      n_samples++;
      const String tDate = String(packet.year) + "-" + String(packet.month) + "-" + String(packet.day);
      t_time = String(packet.hour) + ":" + String(packet.minute) + ":" + String(packet.second);
#ifdef USING_IMU
      csv_IMU_out_str += tDate + "," + t_time + ",";
      for(int ii = 0; ii < 5; ii++) {
        csv_IMU_out_str += String(X_INS(ii), 4)+ ",";
      }
      for(int ii = 0; ii < 3; ii++) {
         csv_IMU_out_str += String(Y_GPS(ii), 4)+ ",";
      }
      for(int ii = 0; ii < 3; ii++) {
        csv_IMU_out_str += String(acc_raw(ii), 4)+ ",";
      }
      for(int ii = 0; ii < 3; ii++) {
        csv_IMU_out_str += String(Rotation_matrix(ii), 4)+ ",";
      }
      for(int ii = 0; ii < 2; ii++) {
        csv_IMU_out_str += String(Updated_GPS(ii), 6)+ ",";
      }
      csv_IMU_out_str += "\n";
#endif //USING_IMU
      const String t_location = String(packet.lng, 6) + "," + String(packet.lat, 6) + "," + String(packet.age);
      csv_out_str += tDate + "," + t_time + "," + t_location + "," + String(packet.batt_percent, 2)
#ifdef USING_MESH
      + "," + String(messages_sent) + "," + String(messages_received) + "," + String(master_rx) + "," + node_hops_to_string() // Diagnostics
#endif // USING_MESH
      + "\n";
#ifndef USING_MESH
#ifdef IGNORE_GPS_INSIDE
    if(true) {
#else 
    // Send GPS data on LoRa if it is this units timeslot
    if(gps.time.second() == drifter_time_slot_sec) {
#endif // IGNORE_GPS_INSIDE
        LoRa.beginPacket();
        LoRa.write((const uint8_t *)&packet, sizeof(packet));
        LoRa.endPacket();
        delay(50); // Don't send more than 1 packet
    }
#endif // USING_MESH
    }
  }
#ifdef DEBUG_MODE
  else {
    Serial.println("No GPS fix, not sending or writing");
  }
#endif //DEBUG_MODE
}

#ifdef USING_IMU
// TODO: make this actually begin calibration
static bool calibrate_IMU() {
  if(calibrate_imu == false) {
    calibrate_imu = true;
    return true;
  }
  else {
    calibrate_imu = false;
    return false;
  }
}
#endif //USING_IMU

static void start_web_server(const bool web_server_on) {
  if(!web_server_on) {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.println("Turning web server off");
  }
  else {
    WiFi.softAP(String(ssid_name + drifter_name).c_str(), SSID_PASSWORD);
    Serial.println(WiFi.softAPIP());  // Print ESP32 Local IP Address

    // F. Web Server Callbacks setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/configure", HTTP_GET, [](AsyncWebServerRequest * request) {
      const int params_nr = request->params();
      Serial.println(params_nr);
      for(int ii = 0; ii < params_nr; ii++) {
        AsyncWebParameter* p = request->getParam(ii);
        if(p->name() == "drifterID") {
          drifter_name = p->value();
        }

        if(p->name() == "loraSendSec") {
          drifter_time_slot_sec = String(p->value()).toInt();
        }
      }
      csv_file_name = "/svt" + String(drifter_name) + ".csv";
#ifdef USING_IMU
      csv_IMU_file_name = "/svt" + String(drifter_name) + "_IMU.csv";
#endif //USING_IMU
#ifdef USING_SD_CARD
      file = SD.open("/config.txt", FILE_WRITE);
#else
      file = SPIFFS.open("/config.txt", FILE_WRITE);
#endif //USING_SD_CARD
      if(!file) {
        Serial.println("Could not open config.txt for writing");
        request->send(200, "text/plain", "Failed writing configuration file config.txt!");
        xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
        drifter_state.b.config_error = 1;
        xSemaphoreGive(drifter_state_mutex);
      }
      else {
        file.print(drifter_name + "," + String(drifter_time_slot_sec));
        file.close();
        request->send(200, "text/html", "<html><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
        xSemaphoreTake(drifter_state_mutex, portMAX_DELAY);
        drifter_state.b.config_error = 0;
        xSemaphoreGive(drifter_state_mutex);
      }
    });

    server.on("/getServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      write_data_to_flash();
#ifdef USING_SD_CARD
      request->send(SD, csv_file_name, "text/plain", true);
#else
      request->send(SPIFFS, csv_file_name, "text/plain", true);
#endif //USING_SD_CARD
    });

    server.on("/deleteServant", HTTP_GET, [](AsyncWebServerRequest * request) {
#ifdef USING_SD_CARD
      SD.remove(csv_file_name);
      file = SD.open(csv_file_name, FILE_WRITE);
#else
      SPIFFS.remove(csv_file_name);
      file = SPIFFS.open(csv_file_name, FILE_WRITE);
#endif //USING_SD_CARD
      if(!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }

      if(file.println("#FILE ERASED at " + last_file_write)) {
        Serial.println("File was created");
      }
      else {
        Serial.println("File creation failed");
      }
      file.close();
      last_file_write = "";
      request->send(200, "text/html", "<html><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
    });
#ifdef USING_IMU
    server.on("/getServantIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
      write_IMU_data_to_flash();
#ifdef USING_SD_CARD
      request->send(SD, csv_IMU_file_name, "text/plain", true);
#else
      request->send(SPIFFS, csv_IMU_file_name, "text/plain", true);
#endif //USING_SD_CARD
    });

    server.on("/deleteServantIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
#ifdef USING_SD_CARD
      SD.remove(csv_IMU_file_name);
      file = SD.open(csv_IMU_file_name, FILE_WRITE);
#else
      SPIFFS.remove(csv_IMU_file_name);
      file = SPIFFS.open(csv_IMU_file_name, FILE_WRITE);
#endif //USING_SD_CARD
      if(!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }

      if(file.println("#FILE ERASED at " + last_file_write)) {
        Serial.println("File was created");
      }
      else {
        Serial.println("File creation failed");
      }
      file.close();
      last_file_write = "";
      request->send(200, "text/html", "<html><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
    });

    server.on("/calibrateIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
      if(calibrate_IMU()) {
        request->send(200, "text/html",
        "<html><span>Successfully calibrated IMU!</span><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Back</a></html>");
      }
      else {
        request->send(200, "text/html",
        "<html><span>Failed to calibrate IMU</span><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Back</a></html>");
      }
    });
#endif // USING_IMU
    server.begin();
  }
}

// Loop does nothing as the loop functions are split into tasks
void loop() {}

static String processor(const String & var) {
  if(var == "SERVANT") {
    String servant_data =
      R"rawliteral(
      <table>
        <tr>
          <td><b>Filename</b></td>
          <td><b>Download data</b></td>
          <td><b>Last File Write GPS Time</b></td>
          <td><b>Erase Data (NO WARNING)</b></td>)rawliteral";
#ifdef USING_IMU
    servant_data += "<td><b>Calibrate IMU</b></td>";
    servant_data += "<td><b>Download IMU Data</b></td>";
    servant_data += "<td><b>Erase IMU Data</b></td>";
#endif // USING_IMU
    servant_data += "</tr>";
    servant_data += "<tr><td>" + csv_file_name + "</td>";
    servant_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/getServant\"> GET </a></td>";
    servant_data += "<td>" + last_file_write + "</td>";
    servant_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/deleteServant\"> ERASE </a></td>";
#ifdef USING_IMU
    servant_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/_\"> CALIBRATE </a></td>";
    servant_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/getServantIMU\"> GET IMU </a></td>";
    servant_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/deleteServantIMU\"> ERASE IMU </a></td>";
#endif // USING_IMU
    return servant_data + "</tr>";
  }
  else if(var == "BATTERYPERCENT") {
    return String(get_battery_percentage(), 2);
  }
  else if(var == "DRIFTERID") {
    return drifter_name;
  }
  else if(var == "LORASENDSEC") {
    return String(drifter_time_slot_sec);
  }
#ifdef USING_MESH
  else if(var == "DIAGNOSTICS") {
    String diagnostic_string =
      R"rawliteral(
      <br><br>
      <h4>Diagnostics</h4>
      <table>
        <tr>
          <td><b>Sent</b></td>
          <td><b>Rcvd</b></td>
        )rawliteral";
      if(master_rx > 0) {
        diagnostic_string += "<td><b>Master</b></td>";
      }
      for(int ii = 0; ii < NUM_NODES; ii++) {
        if(node_rx[ii] > 0) {
          diagnostic_string += "<td><b>D" + String(ii) + "</b></td>";
        }
      }
      diagnostic_string += "<tr>";
      diagnostic_string += "<td>" + String(messages_sent) + "</td>";
      diagnostic_string += "<td>" + String(messages_received) + "</td>";
      if(master_rx > 0) {
        diagnostic_string += "<td>" + String(master_rx) + "</td>";
      }
      for(int ii = 0; ii < NUM_NODES; ii++) {
        if(node_rx[ii] > 0) {
          diagnostic_string += "<td>" + String(node_rx[ii]) + "</td>";
        }
      }
      return diagnostic_string;
  }
#endif // USING_MESH
  else {
    return String();
  }
}
