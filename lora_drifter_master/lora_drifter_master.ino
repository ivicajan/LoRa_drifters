#include "lora_drifter_master.h"

#define WIFI_SSID             ("DrifterMaster")      // Wifi ssid and password
#define WIFI_PASSWORD         ("Tracker1")

// #define USING_SEMAPHORES // Enable the use of semaphores, needs further testing on master node, as it seems to get stuck at some point using semaphores.
// Servant nodes seem fine.

#ifdef USING_SEMAPHORES
SemaphoreHandle_t servant_mutex = NULL;
SemaphoreHandle_t lora_mutex = NULL;
#endif //USING_SEMAPHORES

static TaskHandle_t system_monitoring_task_handle;
static TaskHandle_t listen_task_handle;
static TaskHandle_t web_update_task_handle;

#include "src/lora_drifter_libs/lora_drifter.h"

extern TinyGPSPlus gps;
extern AXP20X_Class PMU;
extern AsyncWebServer server;

// GLOBAL VARIABLES
Master master;                        // Master data
Servant servants[NUM_MAX_SERVANTS];   // Servants data array
static String master_data = "";        // Strings for tabular data output to web page
static String servants_data = "";
static String diagnostic_data = "";
String csv_out_str = "";                // Buffer for output file
String last_file_write = "";
static File file;                     // Data file for the SPIFFS output
static volatile int n_samples;         // Counter for the number of samples gathered
static volatile int gps_last_second = -1;

#ifdef USING_MESH
static TaskHandle_t send_task_handle;
byte routing_table[ROUTING_TABLE_SIZE] = "";
byte payload[24] = "";
byte local_address = MASTER_LOCAL_ID;
byte local_next_hop_ID = MASTER_LOCAL_ID;
byte local_hop_count = 0x00;
String message_log = "";
// Diagnostics
volatile int messages_sent = 0;
volatile int messages_received = 0;
int node_rx[NUM_NODES] = {0}; // array of receiving message counts
#endif // USING_MESH

static String processor(const String & var) {
  if(var == "SERVANTS") {
    return servants_data;
  }
  else if(var == "MASTER") {
    return master_data;
  }
  else if(var == "BATTERYPERCENT") {
    return String(get_battery_percentage(), 2);
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
#ifdef USING_SEMAPHORES
    xSemaphoreTake(servant_mutex, SEMAPHORE_MAX_WAIT);
#endif //USING_SEMAPHORES
    for(int ii = 0; ii < NUM_NODES; ii++) {
      if(servants[ii].active) {
        diagnostic_string += "<td><b>D" + String(ii) + "</b></td>";
      }
    }
#ifdef USING_SEMAPHORES
    xSemaphoreGive(servant_mutex);
#endif //USING_SEMAPHORES
    return diagnostic_string += "</tr>" + diagnostic_data;
  }
  else if(var == "STATUSFLAGS") {
    bool status_flags = false;
    for(int ii = 0; ii < NUM_NODES; ii++) {
      if(servants[ii].active) {
        uint8_t status_tmp = 0;
        memcpy(&status_tmp, &servants[ii].drifter_state, sizeof(uint8_t));
        if(status_tmp > 0) { // has status flags - currently ignoring using mesh
          status_flags = true;
          break;
        }
      }
    }
    if(status_flags) {
      String status_string =
        R"rawliteral(
        <br><br>
        <h4>Status Flags</h4>
        <table>
          <tr>
            <td><b>Drifter</b></td>
            <td><b>Statuses</b></td>
          </tr>
          )rawliteral";
#ifdef USING_SEMAPHORES
      xSemaphoreTake(servant_mutex, SEMAPHORE_MAX_WAIT);
#endif //USING_SEMAPHORES
      for(int ii = 0; ii < NUM_NODES; ii++) {
        if(servants[ii].active) {
          uint8_t status_tmp = 0;
          memcpy(&status_tmp, &servants[ii].drifter_state, sizeof(uint8_t));
          if(status_tmp > 0) {
            status_string += "<tr><td><b>D" + String(ii) + "</b></td><td>" + drifter_status_flag_to_string(&servants[ii].drifter_state) + "</td></tr>";
          }
        }
      }
#ifdef USING_SEMAPHORES
      xSemaphoreGive(servant_mutex);
#endif //USING_SEMAPHORES
      return status_string;
    }
    else{
      return String();
    }
  }
  else if(var == "MESSAGELOG") {
    String outputLog =
      R"rawliteral(
      <br><br>
      <h4>Message Log</h4>
      <table>
        <tr>
          <td><b>Timestamp</b></td>
          <td><b>Log</b></td>
        </tr>
        )rawliteral";
    return outputLog += message_log;
  }
#endif // USING_MESH
  else {
    return String();
  }
}

// Currently using factory only LoRa parameters
static void init_LoRa() {
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if(!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(1); // if failed, do nothing
  }
#ifndef USING_MESH
  LoRa.onReceive(on_receive); // register the receive callback
  LoRa.receive(); // put the radio into receive mode
  delay(50);
#endif // USING_MESH
  LoRa.enableCrc(); // if packet is corrupted, packet gets dropped silently
}

static void write_data_to_flash() {
  file = SPIFFS.open("/master.csv", FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for writing, restarting");
    last_file_write = "FAILED OPEN";
    ESP.restart();
  }
  else {
    if(file.println(csv_out_str)) {
      csv_out_str = "";
      n_samples = 0;
      Serial.print("Wrote data in file, current size: ");
      Serial.println(file.size());
      last_file_write = String(master.hour, DEC) + ":" + String(master.minute, DEC) + ":" + String(master.second, DEC);
    }
    else {
      last_file_write = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

#ifndef USING_MESH
static void on_receive(const int packet_size) {
  if(!packet_size) {
    return;
  }
  uint8_t buffer[sizeof(Packet)];
  for(size_t ii = 0; ii < sizeof(Packet); ii++) {
    buffer[ii] = LoRa.read();
  }
  Packet packet;
  memcpy(&packet, buffer, sizeof(Packet));
  // Get ID and then send to class for decoding
  const String name = String(packet.name);
  if(!strcmp(name.substring(0, 1).c_str(), "D")) {
    Serial.print("Drifter ");
    Serial.print(name);
    Serial.println(" signal found!");
    // csv_out_str += recv; // Save all packets recevied (debugging purposes)
    const int id = name.substring(1, 3).toInt();
    if(id < 1 || id > 10) {
      Serial.println("ID not in between 1 and 10");
    }
    else {
#ifdef USING_SEMAPHORES
      xSemaphoreTake(servant_mutex, SEMAPHORE_MAX_WAIT);
#endif //USING_SEMAPHORES
      servants[id].ID = id;
      servants[id].decode(&packet);
      servants[id].rssi = LoRa.packetRssi();
      servants[id].update_dist_bear(master.lng, master.lat);
      servants[id].active = true;
      const String t_date = String(servants[id].year) + "-" + String(servants[id].month) + "-" + String(servants[id].day);
      const String t_time = String(servants[id].hour) + ":" + String(servants[id].minute) + ":" + String(servants[id].second);
      const String t_location = String(servants[id].lng, 6) + "," + String(servants[id].lat, 6) + "," + String(servants[id].age);
      csv_out_str += t_date + "," + t_time + "," + t_location + '\n';

#ifdef USING_SEMAPHORES
      xSemaphoreGive(servant_mutex);
#endif //USING_SEMAPHORES
      Serial.println("RX from LoRa - decoding completed");
    }
  }
  delay(50);
}
#endif // USING_MESH

static void init_web_server() {
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

  // F. Web Server Callbacks setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/getMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    write_data_to_flash();
    request->send(SPIFFS, "/master.csv", "text/html", true);
  });

  server.on("/deleteMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    SPIFFS.remove("/master.csv");
    file = SPIFFS.open("/master.csv", FILE_WRITE);
    if(!file) {
      Serial.println("There was an error opening the file for writing");
      return;
    }
    if(file.println("#FILE ERASED at " + String(master.hour, DEC) + ":" + String(master.minute, DEC) + ":" + String(master.second, DEC))) {
      Serial.println("File was erased / reinit OK");
    } else {
      Serial.println("File reinit failed");
    }
    file.close();
    last_file_write = "";
    request->send(200, "text/html", "<html><span>Successfully deleted master!</span><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Back</a></html>");
  });
#ifdef USING_MESH
  server.on("/restartDrifter", HTTP_GET, [](AsyncWebServerRequest * request) {
    int paramsNr = request->params();
    for(int ii = 0; ii < paramsNr; ii++) {
      AsyncWebParameter* p = request->getParam(ii);
      if(p->name() == "drifterID") {
        const int drifter_ID = (p->value()).toInt();
        const byte drifter_ID_byte = index_to_id(drifter_ID);
        send_frame(MASTER_MODE, static_cast<byte>(MessageType::Restart), local_address, drifter_ID_byte, local_address, 0x0F);
      }
    }
    request->send(200, "text/html", "<html><span>Sent restart packet!</span><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "\">Back</a></html>");
  });
#endif //USING_MESH
  server.begin();
}

void setup() {
  init_board();
  delay(500);
  init_LoRa();
  delay(50);
  init_web_server();
  delay(50);
#ifdef USING_SEMAPHORES
  servant_mutex = xSemaphoreCreateMutex();
  lora_mutex = xSemaphoreCreateMutex();
#endif //USING_SEMAPHORES
  xTaskCreatePinnedToCore(listen_task, "listen_task", 8000, NULL, 1, &listen_task_handle, 0);
  delay(500);
#ifdef USING_MESH
  xTaskCreatePinnedToCore(send_task, "send_task", 8000, NULL, 1, &send_task_handle, 0);
  delay(500);
#endif //USING_MESH
  xTaskCreatePinnedToCore(web_update_task, "web_update_task", 8000, NULL, 1, &web_update_task_handle, 0);
  delay(500);
  xTaskCreatePinnedToCore(system_monitoring_task, "system_monitoring_task", 3000, NULL, 1, &system_monitoring_task_handle, 0);
  delay(500);
  // SPIFFS to write data to onboard Flash
  if(!SPIFFS.begin(true)) {
    // TODO: add retry
    while(1) {
      Serial.println("An Error has occurred while mounting SPIFFS");
    }
  }
  delay(50);
  Serial.println("Initialization complete.");
}

// TODO: Need to add 8 hours onto gps time
void Master::fill_master() {
  this->lng = gps.location.lng();
  this->lat = gps.location.lat();
  this->year = gps.date.year();
  this->month = gps.date.month();
  this->day = gps.date.day();
  this->hour = gps.time.hour();
  this->minute = gps.time.minute();
  this->second = gps.time.second();
  this->age = gps.location.age();
}

void Master::generate_master() {
  // Read GPS and run decoder
  const uint32_t start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 500);

  if(gps.time.second() != gps_last_second) {
    fill_master();
    const String t_date = String(this->year) + "-" + String(this->month) + "-" + String(this->day);
    const String t_time = String(this->hour) + ":" + String(this->minute) + ":" + String(this->second);
    master_data =  "<tr><td>" + t_date + " " + t_time + "</td><td>" + String(this->lng, 6) + "</td><td>" + String(this->lat, 6) + "</td><td>" + String(this->age) + "</td>";
    master_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/getMaster\"> GET </a></td>";
    master_data += "<td>" + last_file_write + "</td>";
    master_data += "<td><a href=\"http://" + ip_address_to_string(WiFi.softAPIP()) + "/deleteMaster\"> ERASE </a></td>";
    master_data += "</tr>";
    // Update String to be written to file
    if((this->lng != 0.0) && (this->age < 1000)) {
      csv_out_str += "Master," + t_date + "," + t_time + "," + String(this->lng, 6) + "," + String(this->lat, 6) + "," + String(this->age) + "," + String(get_battery_percentage(), 2) + '\n';
      n_samples++;
    }
    else {
      Serial.println("No GPS fix, not writing local data!");
    }
    gps_last_second = gps.time.second();
  }
}

static void system_monitoring_task(void * params){
  (void)params;
  disableCore1WDT(); // Disable watchdog to keep process alive
  while(1){
#ifdef OUTPUT_SYSTEM_MONITOR
    Serial.println("--------------------System Monitor---------------------");
    Serial.print("Heap remaining: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Listen task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(listen_task_handle));
    Serial.print("Web update task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(web_update_task_handle));
#ifdef USING_MESH
    Serial.print("Send task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(send_task_handle));
#endif //USING_MESH
    Serial.print("System monitoring task stack remaining: ");
    Serial.println(uxTaskGetStackHighWaterMark(system_monitoring_task_handle));
    Serial.println("-------------------------------------------------------");
#endif //OUTPUT_SYSTEM_MONITOR
    vTaskDelay(1000); // keep delaying
  }
}

static void listen_task(void * params) {
  (void)params;
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
#ifdef USING_MESH
    const int result = listener(LoRa.parsePacket(), MASTER_MODE);
    vTaskDelay(2);
#endif // USING_MESH
  }
}

static String drifter_status_flag_to_string(drifter_status_t * drifter_status_in){
  String string_out = "- ";
  if(drifter_status_in->b.imu_used == 1){
    if(drifter_status_in->b.imu_error == 1){
      string_out += "IMU ERROR -  ";
    }
    else{
      string_out += "IMU OK -  ";
    }
  }
  // unused mesh print
  // if(drifter_status_in->b->mesh_used == 1){
  //   string_out += "USING MESH";
  // }

  if(drifter_status_in->b.config_error == 1){
    string_out += "CONFIG ERROR -  ";
  }

  if(drifter_status_in->b.low_battery == 1){
    string_out += "LOW BATTERY -  ";
  }

  if(drifter_status_in->b.low_battery == 1){
    string_out += "SAVE ERROR -  ";
  }

  if(drifter_status_in->b.save_error == 1){
    string_out += "SPIFFS ERROR -  ";
  }

  if(drifter_status_in->b.low_storage == 1){
    string_out += "LOW STORAGE -  ";
  }
  return string_out;
}

#ifdef USING_MESH
static void send_task(void * params) {
  (void)params;
  bool sent_bcast = false;
  while(1) {
#ifdef IGNORE_GPS_INSIDE
    if(loop_run_every(RS_BCAST_TIME)) {
#else 
    if(gps.time.second() == RS_BCAST_TIME / 1000 && !sent_bcast) { // time is in ms
#endif // IGNORE_GPS_INSIDE
      PMU.setChgLEDMode(AXP20X_LED_LOW_LEVEL); // LED full on
      Serial.println("Route broadcast");
      bcast_routing_status(MASTER_MODE);
      delay(50);
      PMU.setChgLEDMode(AXP20X_LED_OFF); // LED off
      sent_bcast = true;
    }
    else if(gps.time.second() != RS_BCAST_TIME / 1000) {
      sent_bcast = false;
    }
    vTaskDelay(10);
  }
}
#endif // USING_MESH

static void web_update_task(void * params) {
  (void)params;
  while(1) {
    master.generate_master();
    servants_data = R"rawliteral(
      <br><br>
      <h4>Servants</h4>
      <table>
        <tr>
          <td><b>ID</b></td>
          <td><b>Lora Update Plan [s]</b></td>
          <td><b>Last Update [s]</b></td>
          <td><b>Time</b></td>
          <td><b>Batt [%%]</b></td>
          <td><b>Storage Used [MB]</b></td>
          <td><b>Lon</b></td>
          <td><b>Lat</b></td>
          <td><b>Dist [m]</b></td>
          <td><b>Bearing [degN to]</b></td>
          <td><b>RSSI</b></td>
    )rawliteral";
#ifdef USING_MESH
    servants_data += "<td><b>Restart</b></td>";
#endif // USING_MESH
    servants_data += "</tr>";
#ifdef USING_SEMAPHORES
    xSemaphoreTake(servant_mutex, SEMAPHORE_MAX_WAIT);
#endif //USING_SEMAPHORES
    String temp_class_colour = "";
    for(int ii = 0; ii < NUM_MAX_SERVANTS; ii++) {
      const uint32_t last_update = (millis() - servants[ii].last_update_master_time) / 1000;
      if(last_update >= 180) { // 3 minutes and greater
        temp_class_colour = R"rawliteral(<td style="background-color:Crimson">)rawliteral";
      }
      else if(last_update >= 120 && last_update <= 179) { // 2-3 minutes
        temp_class_colour = R"rawliteral(<td style="background-color:DarkOrange">)rawliteral";
      }
      else if(last_update > 60 && last_update <= 119) { // 1-2 minutes
        temp_class_colour = R"rawliteral(<td style="background-color:LightGrey">)rawliteral";
      }
      else {
        temp_class_colour = R"rawliteral(<td style="background-color:White">)rawliteral";
      }
      if(servants[ii].active) {
        servants_data += "<tr>";
        servants_data += "<td><b>" + String(servants[ii].ID) + "</b></td>";
        servants_data += "<td>" + String(servants[ii].drifter_time_slot_sec) + "</td>";
        servants_data += temp_class_colour + String(last_update) + "</td>";
        servants_data += "<td>" + String(servants[ii].hour) + ":" + String(servants[ii].minute) + ":" + String(servants[ii].second) + "</td>";
        servants_data += "<td>" + String(servants[ii].batt_percent, 2) + "</td>";
        servants_data += "<td>" + String(servants[ii].storage_used, 4) + "</td>";
        servants_data += "<td>" + String(servants[ii].lng, 6) + "</td>";
        servants_data += "<td>" + String(servants[ii].lat, 6) + "</td>";
        servants_data += "<td>" + String(servants[ii].dist) + "</td>";
        servants_data += "<td>" + String(servants[ii].bear) + "</td>";
        servants_data += "<td>" + String(servants[ii].rssi) + "</td>";
#ifdef USING_MESH
        servants_data += R"rawliteral(
          <td><form action="/restartDrifter" method="get"><button type="submit" name="drifterID" value=
        )rawliteral";
        servants_data += String(servants[ii].ID) + ">Restart</button></form></td>";
#endif // USING_MESH
        servants_data += "</tr>";
      }
    }
    servants_data += "</table>";
#ifdef USING_SEMAPHORES
    xSemaphoreGive(servant_mutex);
#endif //USING_SEMAPHORES
#ifdef USING_MESH
    diagnostic_data = "<tr>";
    diagnostic_data += "<td>" + String(messages_sent) + "</td>";
    diagnostic_data += "<td>" + String(messages_received) + "</td>";
#ifdef USING_SEMAPHORES
    xSemaphoreTake(servant_mutex, SEMAPHORE_MAX_WAIT);
#endif //USING_SEMAPHORES
    for(int ii = 0; ii < NUM_NODES; ii++) {
      if(servants[ii].active) {
        diagnostic_data += "<td>" + String(node_rx[ii]) + "</td>";
      }
    }
#ifdef USING_SEMAPHORES
    xSemaphoreGive(servant_mutex);
#endif //USING_SEMAPHORES
    diagnostic_data += "</tr>";
#endif // USING_MESH
    // D. Write data to onboard flash
    if(n_samples > SAMPLES_BEFORE_WRITE) {  // only write after collecting a good number of samples
      write_data_to_flash();
    }
    delay(1); // not sure if required
  }
}

// Loop does nothing as the loop functions are split into RTOS tasks
void loop() {}
