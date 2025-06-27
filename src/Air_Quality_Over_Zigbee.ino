/*Mohammed AL Kharusi
2024/2025 FYP
Air Quality Monitoring System
This code is used to read data from SEN54, DHT22 and MQ135 sensors using accurate calibration metods 
then the data is desplayed on an LCD screen and published via Zigbee.
*/

/*
DHT library: is used to read temperature and humidity from the DHT22 sensor.
MQUnifiedsensor library: is used to read data from the MQ135 sensor.
SensirionI2CSen5x library: is used to read data from the SEN54 sensor.
LiquidCrystal_I2C library: is used to control the LCD display.
*/
#include <Wire.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include "SensirionI2CSen5x.h"
#include <LiquidCrystal_I2C.h>

/*
esp zigbee core library: is used to create a custom Zigbee cluster for the sensor data and managing the Zigbee stack.
esp zigbee ha standard library: is used to create a Zigbee endpoint for the sensor data.
esp zigbee attribute library: is used to define Zigbee attributes for the sensor data.
freertos library: is used to create a task for the Zigbee stack.
*/
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_attribute.h"



// Define data types for attributes, based on the Zigbee Cluster Library (ZCL) data types
#define ZB_DATATYPE_UNSIGNED_16BIT  0x21  // Unsigned 16-bit integer
#define ZB_DATATYPE_SIGNED_16BIT    0x29  // Signed 16-bit integer

// Define attribute access permissions that only allow reading (no writing)
#define ATTR_ACCESS_READ_ONLY 1

// vendor allows to create custom clusters
#define ZB_CLUSTER_SENSOR_DATA      0xFC00

// IDs to save the attributs to be used in the Zigbee cluster
#define ZB_ATTR_DHT_TEMP            0x0000
#define ZB_ATTR_DHT_HUMIDITY        0x0001
#define ZB_ATTR_MQ_CO2              0x0002
#define ZB_ATTR_MQ_CO               0x0003
#define ZB_ATTR_MQ_NH4              0x0004
#define ZB_ATTR_SEN54_TEMP          0x0005
#define ZB_ATTR_SEN54_HUMIDITY      0x0006
#define ZB_ATTR_SEN54_VOC           0x0007
#define ZB_ATTR_SEN54_NOX           0x0008
#define ZB_ATTR_SEN54_PM1           0x0009
#define ZB_ATTR_SEN54_PM2_5         0x000A
#define ZB_ATTR_SEN54_PM4           0x000B
#define ZB_ATTR_SEN54_PM10          0x000C


// specify the device type as an end device
#define ESP_ZB_DEVICE_TYPE_ED ((esp_zb_nwk_device_type_t)1)

/* Macro to configure the Zigbee End Device this allows to set the device type and connnect to the network 
and set the timeout for the end device and keep alive */
#define INSTALLCODE_POLICY_ENABLE   false
#ifndef ESP_ZB_ED_AGING_TIMEOUT_64MIN
  #define ESP_ZB_ED_AGING_TIMEOUT_64MIN 64
#endif
#define ED_AGING_TIMEOUT            ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE               3000 

/* Macro to configure the Zigbee End Device 
 ensure the device maintains a connection to the network */

#define ESP_ZB_ZED_CONFIG()  \
  {                         \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,  \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,  \
    .nwk_cfg = {            \
      .zed_cfg = {          \
         .ed_timeout = ED_AGING_TIMEOUT,  \
         .keep_alive = ED_KEEP_ALIVE,     \
      }                    \
    }                      \
  }

// define the primary channel mask allowing the device to connect to any channel in the 2.4 GHz band
#ifndef ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
  #define ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK 0x07FFF800UL
#endif
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

// define the endpoint for the sensor data to be published in private channel
#define SENSOR_ENDPOINT 20


// sensor data struct to hold allows dynamic data access
struct SensorData {
  float dhtTemp = 0;
  float dhtHumidity = 0;
  float CO2 = 0;
  float CO = 0;
  float NH4 = 0;
  float temperature = 0;
  float humidity = 0;
  float voc = 0;
  float nox = 0;
  float pm1p0 = 0;
  float pm2p5 = 0;
  float pm4p0 = 0;
  float pm10p0 = 0;
};

SensorData sensorData;
SensorData* sensorDataPtr = &sensorData;

// ------------------------------
// LCD Display & Button Handling to toggle between sensor data

#define BUTTON_PIN 2
volatile int screenIndex = 0;
volatile bool screenNeedsUpdate = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ------------------------------
// I2C pins for sensors and LCD
#define I2C_SDA 21
#define I2C_SCL 22

// ------------------------------
// DHT22 Sensor parameters 

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ------------------------------
// MQ135 Sensor parameters

#define MQ135_PIN 1
#define Voltage_Resolution 5.0
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6
MQUnifiedsensor MQ135("ESP-32", Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, "MQ-135");

// ------------------------------
// SEN54 Sensor

SensirionI2CSen5x sen5x;

// ------------------------------
// create cutomised Zigbee cluster for sensor data using List and Attribute libraries

esp_zb_cluster_list_t* custom_sensor_clusters_create(void) {

  // Create a new cluster list and attribute list for cluster.
  esp_zb_cluster_list_t* cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_attribute_list_t* sensor_attr_list = esp_zb_zcl_attr_list_create(ZB_CLUSTER_SENSOR_DATA);

  // static variable for sensor initial values suitable for the Zigbee cluster
  static int16_t dht_temp_init       = 0;
  static int16_t dht_humidity_init   = 0;
  static int16_t mq_co2_init         = 0;
  static int16_t mq_co_init          = 0;
  static int16_t mq_nh4_init         = 0;
  static int16_t sen54_temp_init     = 0;
  static int16_t sen54_humidity_init = 0;
  static int16_t sen54_voc_init      = 0;
  static int16_t sen54_nox_init      = 0;
  static int16_t sen54_pm1_init      = 0;
  static int16_t sen54_pm2_5_init    = 0;
  static int16_t sen54_pm4_init      = 0;
  static int16_t sen54_pm10_init     = 0;

  /* Add attributes to the list with the required the following parameters:
   (attribute list, cluster id, attribute id, attribute type, attribute access, pointer to initial sensor value) */
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_DHT_TEMP,       ZB_DATATYPE_SIGNED_16BIT,   ATTR_ACCESS_READ_ONLY, (void *)&dht_temp_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_DHT_HUMIDITY,   ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&dht_humidity_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_MQ_CO2,         ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&mq_co2_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_MQ_CO,          ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&mq_co_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_MQ_NH4,         ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&mq_nh4_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_TEMP,     ZB_DATATYPE_SIGNED_16BIT,   ATTR_ACCESS_READ_ONLY, (void *)&sen54_temp_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_HUMIDITY, ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_humidity_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_VOC,      ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_voc_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_NOX,      ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_nox_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_PM1,      ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_pm1_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_PM2_5,    ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_pm2_5_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_PM4,      ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_pm4_init);
  esp_zb_cluster_add_attr(sensor_attr_list, ZB_CLUSTER_SENSOR_DATA, ZB_ATTR_SEN54_PM10,     ZB_DATATYPE_UNSIGNED_16BIT, ATTR_ACCESS_READ_ONLY, (void *)&sen54_pm10_init);

  // Add the attribute list to the cluster list
  ESP_ERROR_CHECK( esp_zb_cluster_list_add_custom_cluster(cluster_list, sensor_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE) );
  return cluster_list;
}

// ------------------------------
// Create a custom sensor endpoint with the Zigbee cluster

static esp_zb_ep_list_t* custom_sensor_ep_create(uint8_t endpoint_id) {

  esp_zb_ep_list_t* ep_list = esp_zb_ep_list_create(); // Create a new endpoint list and endpoint configuration
  esp_zb_endpoint_config_t endpoint_config = { 
    .endpoint = endpoint_id, // Set the endpoint ID
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, // use the Home Automation profile ID
    .app_device_id = 0xFFFF,  // Set the device ID
    .app_device_version = 0 // Set the device version
  };
  // Create a custom cluster list for the sensor data
  esp_zb_cluster_list_t* cluster_list = custom_sensor_clusters_create();
  esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
  return ep_list;
}
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    Serial.println("Zigbee signal received.");
}

// ------------------------------
// zigbee task to create a Zigbee network and register the sensor endpoint

static void zigbee_task(void* pvParameters) {
  // Initialize the Zigbee stack 
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  esp_zb_init(&zb_nwk_cfg);
  
  // register the sensor endpoint
  esp_zb_ep_list_t* sensor_ep = custom_sensor_ep_create(SENSOR_ENDPOINT);
  esp_zb_device_register(sensor_ep);
  
  // set the network channel mask.
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  
  // Start Zigbee stack.
  ESP_ERROR_CHECK( esp_zb_start(false) );
  
  // Run the Zigbee loop to handle Zigbee events.
  while (true) {
    esp_zb_main_loop_iteration();
  }
}

// ------------------------------
/*
Function to update the Zigbee attribute with the sensor data 
it convertss the float value to a 16 bit integer and updates the Zigbee attribute 
*/
static void update_zb_attribute(uint16_t attr_id, float value) {
  int16_t measured_value = (int16_t)(value * 100);
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ZB_CLUSTER_SENSOR_DATA,
                                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, &measured_value, false);
  esp_zb_lock_release();
}

// ------------------------------
// LCD Display functions

// Function to repeat spaces for centering text on the LCD
String repeatSpaces(int count) {
  String result = "";
  for (int i = 0; i < count; i++) {
    result += " ";
  }
  return result;
}

// Function to display text on the LCD centered
void displayLCDCentered(String line1, String line2) {
  lcd.clear();
  int pad1 = (16 - line1.length()) / 2;
  String paddedLine1 = repeatSpaces(pad1) + line1;
  lcd.setCursor(0, 0);
  lcd.print(paddedLine1);
  
  int pad2 = (16 - line2.length()) / 2;
  String paddedLine2 = repeatSpaces(pad2) + line2;
  lcd.setCursor(0, 1);
  lcd.print(paddedLine2);
}

// Function to update the LCD with sensor data based on the button press
void updateLCD(SensorData* data) {
  switch (screenIndex) {
    case 0:
      displayLCDCentered("DHT Temp: " + String(data->dhtTemp, 1) + " C",
                           "DHT Hum: " + String(data->dhtHumidity, 1) + "%");
      break;
    case 1:
      displayLCDCentered("PM2.5: " + String(data->pm2p5, 1),
                         "PM10: " + String(data->pm10p0, 1));
      break;
    case 2:
      displayLCDCentered("Sen Temp: " + String(data->temperature, 1) + " C",
                         "Sen Hum: " + String(data->humidity, 1) + "%");
      break;
    case 3:
      displayLCDCentered("CO2: " + String(data->CO2, 1) + " ppm",
                         "VOC: " + String(data->voc, 1));
      break;
    default:
      displayLCDCentered("Sensor Data", "");
      break;
  }
}

// ------------------------------
// Button Interrupt Handler 
// to toggle between sensor data screens
void IRAM_ATTR buttonPressed() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    screenIndex = (screenIndex + 1) % 4;
    screenNeedsUpdate = true;
    lastDebounceTime = currentTime;
  }
}


void setup() {
  Serial.begin(115200); // serial monitor
  
  // Initialize I2C for sensors and LCD.
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  
  // Initialize DHT22 Sensor.
  dht.begin();
  
  // MQ135 Sensor Initialization 
  MQ135.init();
  MQ135.setRegressionMethod(1);
  Serial.print("Calibrating MQ135 sensor...");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    delay(200);
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println(" done!");
  
  // SEN54 Sensor Initialization
  sen5x.begin(Wire);
  if (sen5x.startMeasurement()) {
    Serial.println("SEN54 measurement started successfully");
  } else {
    Serial.println("Failed to start SEN54 measurement");
  }
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Air :)");
  
  // initialize button with interrupt handler
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);
  
  // Create a task for the Zigbee stack to run in parallel.
  xTaskCreate(zigbee_task, "ZigbeeTask", 4096, NULL, 5, NULL);
}


void loop() {
  static unsigned long lastSensorUpdate = 0;
  unsigned long currentMillis = millis();
  
  // Update sensor readings every 50 seconds 
  if (currentMillis - lastSensorUpdate >= 50000) {
    lastSensorUpdate = currentMillis;
    
    // read DHT22 Sensor data
    sensorData.dhtTemp = dht.readTemperature();
    sensorData.dhtHumidity = dht.readHumidity();
    if (isnan(sensorData.dhtTemp) || isnan(sensorData.dhtHumidity)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    
    // Read MQ135 Sensor for CO2, CO, and NH4 
    MQ135.update();
    MQ135.setA(110.47);
    MQ135.setB(-2.862);
    sensorData.CO2 = MQ135.readSensor() + 400;
    
    MQ135.setA(605.18);
    MQ135.setB(-3.937);
    sensorData.CO = MQ135.readSensor();
    
    MQ135.setA(102.2);
    MQ135.setB(-2.473);
    sensorData.NH4 = MQ135.readSensor();
    
    // Read SEN54 Sensor data
    uint16_t error = sen5x.readMeasuredValues(sensorData.pm1p0, sensorData.pm2p5,
                                               sensorData.pm4p0, sensorData.pm10p0,
                                               sensorData.humidity, sensorData.temperature,
                                               sensorData.voc, sensorData.nox);
    if (error != 0) {
      Serial.println("Failed to read from SEN54 sensor!");
    } else {
      Serial.println("Sensor data updated successfully");
    }
    
    // Update the LCD with the new data.
    updateLCD(sensorDataPtr);
    
    // Publish sensor data via Zigbee by updating the attributes
    update_zb_attribute(ZB_ATTR_DHT_TEMP, sensorData.dhtTemp);
    update_zb_attribute(ZB_ATTR_DHT_HUMIDITY, sensorData.dhtHumidity);
    update_zb_attribute(ZB_ATTR_MQ_CO2, sensorData.CO2);
    update_zb_attribute(ZB_ATTR_MQ_CO, sensorData.CO);
    update_zb_attribute(ZB_ATTR_MQ_NH4, sensorData.NH4);
    update_zb_attribute(ZB_ATTR_SEN54_TEMP, sensorData.temperature);
    update_zb_attribute(ZB_ATTR_SEN54_HUMIDITY, sensorData.humidity);
    update_zb_attribute(ZB_ATTR_SEN54_VOC, sensorData.voc);
    update_zb_attribute(ZB_ATTR_SEN54_NOX, sensorData.nox);
    update_zb_attribute(ZB_ATTR_SEN54_PM1, sensorData.pm1p0);
    update_zb_attribute(ZB_ATTR_SEN54_PM2_5, sensorData.pm2p5);
    update_zb_attribute(ZB_ATTR_SEN54_PM4, sensorData.pm4p0);
    update_zb_attribute(ZB_ATTR_SEN54_PM10, sensorData.pm10p0);
  }
  
  // switch between sensor data screens immediately if the button was pressed (LCD)
  if (screenNeedsUpdate) {
    screenNeedsUpdate = false;
    updateLCD(sensorDataPtr);
    Serial.print("Screen updated to index: ");
    Serial.println(screenIndex);
  }
  
  delay(10);
}
