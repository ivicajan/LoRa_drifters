#include "src/loraDrifterLibs/loraDrifter.h"

// #define USING_SD_CARD

#ifdef USING_SD_CARD
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

// this shouldnt be a thing but esp32 seems very tempremental in using semaphores (RTOS),
// using semaphores gives us thread safety
// #define USING_SEMAPHORES

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
String csvIMUFileName = "";
static File imu_file;
String csvIMUOutStr = "";
static TaskHandle_t imu_task_handle;
#endif // USING_IMU

#define SSID     "DrifterServant"   // Wifi ssid and password
#define PASSWORD "Tracker1"

#define LAST_PACKET_TIMEOUT_ms (600000)
volatile uint32_t last_packet_received_time_ms = 0;

extern AsyncWebServer server;
extern AXP20X_Class PMU;

// GLOBAL VARIABLES
String csvOutStr = "";                 // Buffer for output file
static String lastFileWrite = "";
static bool webServerOn = false;
static String csvFileName = "";
static File file;                            // Data file for the SPIFFS output
volatile int nSamples = 0;                         // Counter for the number of samples gathered
static volatile float storageUsed = 0.f;                   // MB used in SPIFFS storage

static volatile int gpsLastSecond = -1;
static String tTime = "";
static String drifterName = "D05";       // ID send with packet
static volatile int drifterTimeSlotSec = 25;      // seconds after start of each GPS minute

Packet packet;
static drifterStatus_t drifterState; // status flags of the drifter state

#ifdef USING_SEMAPHORES
static SemaphoreHandle_t loraSemaphore = NULL; // only used here as its commented out elsewhere
static SemaphoreHandle_t drifterStateMutex = NULL;
#endif //USING_SEMAPHORES
static TaskHandle_t send_task_handle;
static TaskHandle_t system_monitoring_task_handle;

#ifdef USING_MESH
static TaskHandle_t listen_task_handle;
byte routingTable[ROUTING_TABLE_SIZE] = "";
byte payload[24] = "";
volatile int localLinkRssi = 0;
byte localHopCount = 0x00;
byte localNextHopID = 0x00;
byte localAddress = indexToId(drifterName.substring(1, 3).toInt());
// Diagnostics
volatile int messagesSent = 0;
volatile int messagesReceived = 0;
int nodeRx[NUM_NODES];
volatile int masterRx = 0;
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

static void writeData2Flash() {
#ifdef USING_SD_CARD
  file = SD.open(csvFileName, FILE_APPEND);
#else
  file = SPIFFS.open(csvFileName, FILE_APPEND);
#endif //USING_SD_CARD
  if(!file) {
    Serial.println("There was an error opening the file for appending, creating a new one");
#ifdef USING_SD_CARD
    file = SD.open(csvFileName, FILE_WRITE);
#else
    file = SPIFFS.open(csvFileName, FILE_WRITE);
#endif //USING_SD_CARD
  }
  if(!file) { // this shouldn't occur
      Serial.println("There was an error opening the file for writing");
      lastFileWrite = "FAILED OPEN";
      ESP.restart();
  }
  else {
    const uint32_t file_size_before_save = file.size();
    if(file.println(csvOutStr)) {
      Serial.print("Wrote data in file, current size: ");
      const uint32_t file_size_after_save = file.size();
      Serial.println(file_size_after_save);
      storageUsed = file_size_after_save * 0.000001f; // convert to MB
      csvOutStr = "";
      nSamples = 0;
      lastFileWrite = tTime;
#ifdef USING_SEMAPHORES
      xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
      drifterState.b.saveError = (file_size_after_save == file_size_before_save);
#ifdef USING_SEMAPHORES
      xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
    }
    else {
      lastFileWrite = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

#ifdef USING_IMU
static void writeIMUData2Flash() {
#ifdef USING_SD_CARD
  imu_file = SD.open(csvIMUFileName, FILE_APPEND);
#else
  imu_file = SPIFFS.open(csvIMUFileName, FILE_APPEND);
#endif //USING_SD_CARD
  if(!imu_file) {
    Serial.println("There was an error opening the imu file for appending, creating a new one");
#ifdef USING_SD_CARD
    imu_file = SD.open(csvIMUFileName, FILE_WRITE);
#else
    imu_file = SPIFFS.open(csvIMUFileName, FILE_WRITE);
#endif //USING_SD_CARD
  }
  if(!imu_file) { // this shouldn't occur
      Serial.println("There was an error opening the imu file for writing");
      ESP.restart();
  }
  else {
    if(imu_file.println(csvIMUOutStr)) {
      Serial.print("Wrote data in imu file, current size: ");
      Serial.println(imu_file.size());
      csvIMUOutStr = "";
      storageUsed += imu_file.size() * 0.000001f; // convert to MB
    }
    else {
      ESP.restart();
    }
  }
  imu_file.close();
  delay(50);
}

static void update_imu() {
  int reset__ = 0;
  if(mpu.update()) {
    static uint32_t prev_ms = millis();
    static uint32_t imu_ms = millis(); //For reset
    if(millis() > prev_ms + SAMPLE_PERIOD_ms) {
#ifdef WAVE_TANK_TEST
      Y_GPS = {0,0,Yaw[0]};
      //if(millis() < 25000) Initial_Kalman();
#else
//      if((millis() > imu_ms + 20000) && (reset__ != 1)){
//        update_ref_location(); // Reset data after 20 secs
//        imu_ms = millis();
//        reset__ = 1;
//      }

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
// #ifdef CALIBRATION_IMU
      // read_Serial_input();
      // check_stable_imu();
// #endif //CALIBRATION_IMU


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

static void imuTask(void * params) {
  (void)params;
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // might not need a delay at all
    update_imu();
  }
}
#endif // USING_IMU

#ifdef USING_MESH
static void listenTask(void * params){
  (void)params;
  disableCore0WDT(); // Disable watchdog to keep process alive
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10)); // might not need a delay at all
    const int result = listener(LoRa.parsePacket(), SERVANT_MODE);
  }
}
#endif // USING_MESH

static void systemMonitoringTask(void * params){
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

static void sendTask(void * params) {
  (void)params;
  disableCore1WDT(); // Disable watchdog to keep process alive
  while(1) {
    if(millis() - last_packet_received_time_ms > LAST_PACKET_TIMEOUT_ms) {
      ESP.restart();
    }
    if(digitalRead(BUTTON_PIN) == LOW) { //Check for button press
      if(webServerOn) {
        webServerOn = false;
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
        webServerOn = true;
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
      startWebServer(webServerOn);
      delay(1000);
    }
  if(!webServerOn) {
#ifdef USING_MESH
      int result = 0;
      generatePacket();
#ifdef IGNORE_GPS_INSIDE
      if(runEvery(PL_TX_TIME)) {
#else
      if(gps.time.second() == drifterTimeSlotSec) {
#endif //IGNORE_GPS_INSIDE
        result = routePayload(SERVANT_MODE, MASTER_LOCAL_ID, localAddress, 0x0F, 0);
        PMU.setChgLEDMode(AXP20X_LED_LOW_LEVEL); // LED full on
        delay(50);
        PMU.setChgLEDMode(AXP20X_LED_OFF); // LED off
      }
      if(loop_runEvery(RS_BCAST_TIME)) {
        Serial.println("Route broadcast");
        result = bcastRoutingStatus(SERVANT_MODE);   // returns 1 or -1
        PMU.setChgLEDMode(AXP20X_LED_LOW_LEVEL); // LED full on
        delay(50);
        PMU.setChgLEDMode(AXP20X_LED_OFF); // LED off
      }
#endif // USING_MESH
      delay(10);
      // Write data to onboard flash if nSamples is large enough
      if(nSamples >= SAMPLES_BEFORE_WRITE) {  // only write after collecting a good number of samples
        Serial.println("Dump data into the memory");
        writeData2Flash();
#ifdef USING_IMU
        writeIMUData2Flash();
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
static void initLoRa(bool default_params = false) {
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
static void readConfigFile() {
#ifdef USING_SD_CARD
  file = SD.open("/config.txt", FILE_READ);
#else
  file = SPIFFS.open("/config.txt", FILE_READ);
#endif //USING_SD_CARD
  if(!file) {
    Serial.println("Failed to open config.txt configuration file");
#ifdef USING_SEMAPHORES
      xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
      drifterState.b.configError = 1;
#ifdef USING_SEMAPHORES
      xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
  }
  else {
    const String inData = file.readStringUntil('\n');
    const int comma = inData.indexOf(",");
    drifterName = inData.substring(0, comma);
#ifdef USING_MESH
    localAddress = indexToId(drifterName.substring(1, 3).toInt());
#endif //USING_MESH
    drifterTimeSlotSec = inData.substring(comma + 1).toInt();
    Serial.println(inData);
    file.close();
  }
  delay(50);
  csvFileName = "/svt" + String(drifterName) + ".csv";
#ifdef USING_SD_CARD
  file = SD.open(csvFileName, FILE_APPEND);
#else
  file = SPIFFS.open(csvFileName, FILE_APPEND);
#endif //USING_SD_CARD
  if(file){
    storageUsed = file.size() * 0.000001f; // convert to MB
    file.close();
  }
#ifdef USING_IMU
  csvIMUFileName = "/svt" + String(drifterName) + "_IMU.csv";
#ifdef USING_SD_CARD
  imu_file = SD.open(csvIMUFileName, FILE_APPEND);
#else
  imu_file = SPIFFS.open(csvFileName, FILE_APPEND);
#endif //USING_SD_CARD
  if(imu_file){
    storageUsed += imu_file.size() * 0.000001f; // convert to MB
    imu_file.close();
  }
#endif //USING_IMU
}

#ifdef USING_SD_CARD
static void printVolumeSize() {
    const uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC) {
        Serial.println("MMC");
    }
    else if(cardType == CARD_SD) {
        Serial.println("SDSC");
    }
    else if(cardType == CARD_SDHC) {
        Serial.println("SDHC");
    }
    else {
        Serial.println("UNKNOWN");
    }

    const uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}
#endif //USING_SD_CARD

void setup() {
#ifdef USING_SEMAPHORES
  xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
  loraSemaphore = xSemaphoreCreateMutex();
  drifterStateMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
  initBoard();
#ifdef USING_SD_CARD
  hspi = new SPIClass(HSPI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS); //SCLK, MISO, MOSI, SS
  pinMode(HSPI_CS, OUTPUT); //HSPI SS

  // see if the card is present and can be initialized:
  if(!SD.begin(HSPI_CS, *hspi)) {
    Serial.println("SD Card failed, or not not present");
  }
  printVolumeSize();
#endif //USING_SD_CARD
  delay(500);
#ifdef USING_SEMAPHORES
  xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
  memset(&drifterState, 0, sizeof(drifterStatus_r));
#ifdef USING_IMU
  const bool initIMUok = initIMU();
  drifterState.b.imuUsed = 1;
  drifterState.b.imuError = !initIMUok;
  delay(500);
#endif // USING_IMU
#ifdef USING_SEMAPHORES
  xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
  if(!SPIFFS.begin(true)) {
    Serial.println("SPIFFS error has occured");
    return;
  }
  initLoRa();
  delay(50);
  readConfigFile();
#ifdef USING_IMU
  if(initIMUok){
    xTaskCreatePinnedToCore(imuTask, "imuTask", 10000, NULL, 1, &imu_task_handle, 0);
    delay(500);
  }
  else {
    imu_task_handle = NULL;
  }
#endif //USING_IMU
#ifdef USING_MESH
#ifdef USING_SEMAPHORES
      xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
      // drifterState.b.meshUsed = 1;
#ifdef USING_SEMAPHORES
      xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
  xTaskCreatePinnedToCore(listenTask, "listenTask", 5000, NULL, 2, &listen_task_handle, 0);
  delay(500);
#endif //USING_MESH
  xTaskCreatePinnedToCore(sendTask, "sendTask", 8000, NULL, 2, &send_task_handle, 1);
  xTaskCreatePinnedToCore(systemMonitoringTask, "systemMonitoringTask", 3000, NULL, 2, &system_monitoring_task_handle, tskIDLE_PRIORITY);
  delay(500);
  Serial.println("Initialization complete.");
}

static void fill_packet() {
  strcpy(packet.name, drifterName.c_str());
  packet.drifterTimeSlotSec = drifterTimeSlotSec;
  packet.hour = gps.time.hour();
  packet.minute = gps.time.minute();
  packet.second = gps.time.second();
  packet.year = gps.date.year();
  packet.month = gps.date.month();
  packet.day = gps.date.day();
  packet.lng = gps.location.lng();
  packet.lat = gps.location.lat();
  packet.storageUsed = storageUsed;
  packet.age = gps.location.age();
  packet.battPercent = getBatteryPercentage();
#ifdef USING_SEMAPHORES
  xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
  drifterState.b.lowStorage = (packet.storageUsed > 0.75f * SPIFFS_FLASH_SIZE); // if we are above 75% storage capacity we show the flag 1 (error)
  drifterState.b.lowBattery = (packet.battPercent < 50.f); // if we are below 50% battery we show the flag 1 (error)
  packet.drifterState = drifterState;
  // uncomment below to display hardcoded errors on master
  // packet.drifterState.b.imuError = 1;
  // packet.drifterState.b.lowBattery = 1;
  // packet.drifterState.b.lowStorage = 1;
#ifdef USING_SEMAPHORES
  xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
#ifdef DEBUG_MODE
  Serial << "Raw Lat: " << float(packet.lat) << " Lng: " << float(packet.lng) << '\n';
#endif
}

#ifdef USING_MESH
static String nodeHopsToString() {
  String outStr = "";
  for(size_t idx = 1; idx < NUM_NODES; idx++) {
    outStr += String(nodeRx[idx]);
    if(idx != NUM_NODES - 1) {
      outStr += ",";
    }
  }
  return outStr;
}
#endif // USING_MESH

static void generatePacket() {
  const uint32_t start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 400);
#ifdef IGNORE_GPS_INSIDE
  if(true) {
#else 
  if(gps.time.second() != gpsLastSecond) {
#endif // IGNORE_GPS_INSIDE
    Serial.print("New GPS record from: ");
    Serial.println(drifterName);
    fill_packet();
    gpsLastSecond = gps.time.second();
    Serial.print("samples: ");
    Serial.println(nSamples);
#ifdef IGNORE_GPS_INSIDE
    if(true) {
#else 
    if((gps.location.lng() != 0.0) && (gps.location.age() < 1000)) {
#endif // IGNORE_GPS_INSIDE
      nSamples++;
      const String tDate = String(packet.year) + "-" + String(packet.month) + "-" + String(packet.day);
      tTime = String(packet.hour) + ":" + String(packet.minute) + ":" + String(packet.second);
#ifdef USING_IMU
      csvIMUOutStr += tDate + "," + tTime + ",";
      for(int ii = 0; ii < 5; ii++) {
        csvIMUOutStr += String(X_INS(ii), 4)+ ",";
      }
       for(int ii = 0; ii < 3; ii++) {
         csvIMUOutStr += String(Y_GPS(ii), 4)+ ",";
       }
       for(int ii = 0; ii < 3; ii++) {
         csvIMUOutStr += String(acc_raw(ii), 4)+ ",";
       }
       for(int ii = 0; ii < 3; ii++) {
         csvIMUOutStr += String(Rotation_matrix(ii), 4)+ ",";
       }
       for(int ii = 0; ii < 2; ii++) {
         csvIMUOutStr += String(Updated_GPS(ii), 6)+ ",";
       }
      csvIMUOutStr += "\n";
#endif //USING_IMU
      const String tLocation = String(packet.lng, 6) + "," + String(packet.lat, 6) + "," + String(packet.age);
      csvOutStr += tDate + "," + tTime + "," + tLocation + "," + String(packet.battPercent, 2)
#ifdef USING_MESH
      + "," + String(messagesSent) + "," + String(messagesReceived) + "," + String(masterRx) + "," + nodeHopsToString() // Diagnostics
#endif // USING_MESH
      + "\n";
#ifndef USING_MESH
#ifdef IGNORE_GPS_INSIDE
    if(true) {
#else 
    // Send GPS data on LoRa if it is this units timeslot
    if(gps.time.second() == drifterTimeSlotSec) {
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
static bool calibrateIMU() {
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

static void startWebServer(const bool webServerOn) {
  if(!webServerOn) {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.println("Turning web server off");
  }
  else {
    WiFi.softAP(SSID, PASSWORD);
    Serial.println(WiFi.softAPIP());  // Print ESP32 Local IP Address

    // F. Web Server Callbacks setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/configure", HTTP_GET, [](AsyncWebServerRequest * request) {
      int paramsNr = request->params();
      Serial.println(paramsNr);
      for(int ii = 0; ii < paramsNr; ii++) {
        AsyncWebParameter* p = request->getParam(ii);
        if(p->name() == "drifterID") {
          drifterName = p->value();
        }
        if(p->name() == "loraSendSec") {
          drifterTimeSlotSec = String(p->value()).toInt();
        }
      }
      csvFileName = "/svt" + String(drifterName) + ".csv";
#ifdef USING_IMU
      csvIMUFileName = "/svt" + String(drifterName) + "_IMU.csv";
#endif //USING_IMU
#ifdef USING_SD_CARD
      file = SD.open("/config.txt", FILE_WRITE);
#else
      file = SPIFFS.open("/config.txt", FILE_WRITE);
#endif //USING_SD_CARD
      if(!file) {
        Serial.println("Could not open config.txt for writing");
        request->send(200, "text/plain", "Failed writing configuration file config.txt!");
#ifdef USING_SEMAPHORES
      xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
      drifterState.b.configError = 1;
#ifdef USING_SEMAPHORES
      xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
      }
      else {
        file.print(drifterName + "," + String(drifterTimeSlotSec));
        file.close();
        request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
#ifdef USING_SEMAPHORES
      xSemaphoreTake(drifterStateMutex, portMAX_DELAY);
#endif //USING_SEMAPHORES
      drifterState.b.configError = 0;
#ifdef USING_SEMAPHORES
      xSemaphoreGive(drifterStateMutex);
#endif //USING_SEMAPHORES
      }
    });

    server.on("/getServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      writeData2Flash();
#ifdef USING_SD_CARD
      request->send(SD, csvFileName, "text/plain", true);
#else
      request->send(SPIFFS, csvFileName, "text/plain", true);
#endif //USING_SD_CARD
    });

    server.on("/deleteServant", HTTP_GET, [](AsyncWebServerRequest * request) {
#ifdef USING_SD_CARD
      SD.remove(csvFileName);
      file = SD.open(csvFileName, FILE_WRITE);
#else
      SPIFFS.remove(csvFileName);
      file = SPIFFS.open(csvFileName, FILE_WRITE);
#endif //USING_SD_CARD
      if(!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }
      if(file.println("#FILE ERASED at " + lastFileWrite)) {
        Serial.println("File was created");
      }
      else {
        Serial.println("File creation failed");
      }
      file.close();
      lastFileWrite = "";
      request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
    });
#ifdef USING_IMU
    server.on("/getServantIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
      writeIMUData2Flash();
#ifdef USING_SD_CARD
      request->send(SD, csvIMUFileName, "text/plain", true);
#else
      request->send(SPIFFS, csvIMUFileName, "text/plain", true);
#endif //USING_SD_CARD
    });

    server.on("/deleteServantIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
#ifdef USING_SD_CARD
      SD.remove(csvIMUFileName);
      file = SD.open(csvIMUFileName, FILE_WRITE);
#else
      SPIFFS.remove(csvIMUFileName);
      file = SPIFFS.open(csvIMUFileName, FILE_WRITE);
#endif //USING_SD_CARD
      if(!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }
      if(file.println("#FILE ERASED at " + lastFileWrite)) {
        Serial.println("File was created");
      }
      else {
        Serial.println("File creation failed");
      }
      file.close();
      lastFileWrite = "";
      request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
    });

    server.on("/calibrateIMU", HTTP_GET, [](AsyncWebServerRequest * request) {
      if(calibrateIMU()) {
        request->send(200, "text/html",
        "<html><span>Successfully calibrated IMU!</span><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Back</a></html>");
      }
      else {
        request->send(200, "text/html",
        "<html><span>Failed to calibrate IMU</span><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Back</a></html>");
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
    String servantData =
      R"rawliteral(
      <table>
        <tr>
          <td><b>Filename</b></td>
          <td><b>Download data</b></td>
          <td><b>Last File Write GPS Time</b></td>
          <td><b>Erase Data (NO WARNING)</b></td>)rawliteral";
#ifdef USING_IMU
    servantData += "<td><b>Calibrate IMU</b></td>";
    servantData += "<td><b>Download IMU Data</b></td>";
    servantData += "<td><b>Erase IMU Data</b></td>";
#endif // USING_IMU
    servantData += "</tr>";
    servantData += "<tr><td>" + csvFileName + "</td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getServant\"> GET </a></td>";
    servantData += "<td>" + lastFileWrite + "</td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteServant\"> ERASE </a></td>";
#ifdef USING_IMU
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/calibrateIMU\"> CALIBRATE </a></td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getServantIMU\"> GET IMU </a></td>";
    servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteServantIMU\"> ERASE IMU </a></td>";
#endif // USING_IMU
    return servantData + "</tr>";
  }
  else if(var == "BATTERYPERCENT") {
    return String(getBatteryPercentage(), 2);
  }
  else if(var == "DRIFTERID") {
    return drifterName;
  }
  else if(var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
#ifdef USING_MESH
  else if(var == "DIAGNOSTICS") {
    String diagnosticString =
      R"rawliteral(
      <br><br>
      <h4>Diagnostics</h4>
      <table>
        <tr>
          <td><b>Sent</b></td>
          <td><b>Rcvd</b></td>
        )rawliteral";
      if(masterRx > 0) {
        diagnosticString += "<td><b>Master</b></td>";
      }
      for(int ii = 0; ii < NUM_NODES; ii++) {
        if(nodeRx[ii] > 0) {
          diagnosticString += "<td><b>D" + String(ii) + "</b></td>";
        }
      }
      diagnosticString += "<tr>";
      diagnosticString += "<td>" + String(messagesSent) + "</td>";
      diagnosticString += "<td>" + String(messagesReceived) + "</td>";
      if(masterRx > 0) {
        diagnosticString += "<td>" + String(masterRx) + "</td>";
      }
      for(int ii = 0; ii < NUM_NODES; ii++) {
        if(nodeRx[ii] > 0) {
          diagnosticString += "<td>" + String(nodeRx[ii]) + "</td>";
        }
      }
      return diagnosticString;
  }
#endif // USING_MESH
  else {
    return String();
  }
}
