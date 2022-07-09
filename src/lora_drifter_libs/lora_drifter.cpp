#include "lora_drifter.h"

AXP20X_Class PMU;
TinyGPSPlus gps;                      // decoder for GPS stream
AsyncWebServer server(80);            // Create AsyncWebServer object on port 80

#define BATT_MAX_BATTERY_VOLTAGE_MV (4200.f)
#define BATT_MIN_BATTERY_VOLTAGE_MV (3200.f)
#define BATT_VOLTAGE_RANGE_MV       (BATT_MAX_BATTERY_VOLTAGE_MV - BATT_MIN_BATTERY_VOLTAGE_MV)

float get_battery_percentage() {
    return ((PMU.getBattVoltage() - BATT_MIN_BATTERY_VOLTAGE_MV) / BATT_VOLTAGE_RANGE_MV) * 100.f;
}

/**
 * @brief Initialise the power managmenet unit on the TTGO ESP32.
 * 
 * @return true if the initialisation is successful, else false
 */
static bool init_PMU() {
    Serial.println("init_PMU");
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50);
    if(PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        Serial.println("PMU failed!");
        return false;
    }
    PMU.setChgLEDMode(AXP20X_LED_OFF); //The charging indicator can be turned on or off

    /*
     * The default ESP32 power supply has been turned on,
     * no need to set, please do not set it, if it is turned off,
     * it will not be able to program
     **/

    // PMU.setDCDC1Voltage(3300);
    // PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

    /*
     *  Turn off unused power sources to save power
     **/

    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
     * Set the power of LoRa and GPS module to 3.3V
     **/
    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD
    PMU.setDCDC1Voltage(3300);  //3.3V Pin next to 21 and 22 is controlled by DCDC1

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
        // pmu_irq = true;
    }, FALLING);

    PMU.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                    AXP202_VBUS_CUR_ADC1 |
                    AXP202_BATT_CUR_ADC1 |
                    AXP202_BATT_VOL_ADC1,
                    AXP202_ON);

    PMU.enableIRQ(AXP202_VBUS_REMOVED_IRQ |
                    AXP202_VBUS_CONNECT_IRQ |
                    AXP202_BATT_REMOVED_IRQ |
                    AXP202_BATT_CONNECT_IRQ,
                    AXP202_ON);
    PMU.clearIRQ();
    return true;
}

static void init_GPS() {
    Serial.println("init gps");
    Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(50);
}

static void init_lights_and_pins() {
    Serial.println("init_lights_and_pins");
#ifdef BOARD_LED
      /*
      * T-BeamV1.0, V1.1 LED defaults to low level as turn on,
      * so it needs to be forced to pull up
      * * * * */
#if LED_ON == LOW
    gpio_hold_dis(GPIO_NUM_4);
#endif //LED_ON == LOW
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif //BOARD_LED
    delay(50);
    pinMode(WEB_SERVER_PIN, INPUT);
    delay(50);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    delay(50);
}

void init_board() {
    Serial.begin(115200);
    while(!Serial) {
        Serial.println("Could not init 115200 serial");
    };
    Serial.println("init_board");
    init_PMU();
    init_GPS();
    init_lights_and_pins();
}

// Convert an IP address to a string
String ip_address_to_string(const IPAddress& ip_address) {
  return String(ip_address[0]) + String(".") +
    String(ip_address[1]) + String(".") +
    String(ip_address[2]) + String(".") +
    String(ip_address[3]);
}
