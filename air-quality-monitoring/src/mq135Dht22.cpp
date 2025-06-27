/*Mohammed AL Kharusi
2024/2025 FYP
Air Quality Monitoring System
This code is used to read data from DHT22 and MQ135 sensors and publish it to the HiveMQ Cloud MQTT broker.
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>  // For SSL connection
#include <DHT.h>               // Include the DHT library for DHT22 sensor
#include <NTPClient.h>         // Library for network time protocol
#include <WiFiUdp.h>           // Required for network time protocol
#include "MQUnifiedsensor.h"   // Include the MQUnifiedsensor library

// Network connection parameters
const char* ssid = "VODAFONE-7C80";
const char* password = "";

// HiveMQ Cloud broker connection
const char* mqtt_server = ".s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "makrs";
const char* mqtt_password = "";

// Define DHT22 sensor parameters (GPIO4)
#define DHTPIN 4           
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Initialize WiFi and MQTT client
WiFiClientSecure espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // Sync every 60 seconds

// Define MQ135 sensor parameters
#define placa "ESP-32"
#define Voltage_Resolution 5.0
#define MQ135_PIN 34            // Analog input pin for MQ-135
#define ADC_Bit_Resolution 12   // ADC resolution for ESP32
#define RatioMQ135CleanAir 3.6  // RS/R0 ratio in clean air
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, "MQ-135");

// WiFi connection setup
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT callback function
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

// Reconnect to MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(WiFi.macAddress());
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("test/topic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/*
the setup function initializes the serial communication, WiFi connection, MQTT client, DHT22 sensor, NTP client, and MQ135 sensor.
*/
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  dht.begin();

  // Initialize NTP client for time synchronization
  timeClient.begin();

  // Skip certificate validation to connect to HiveMQ Cloud
  espClient.setInsecure();

  // Initialize MQ135 sensor
  MQ135.init();
  MQ135.setRegressionMethod(1); 
  Serial.print("Calibrating please wait...");
  
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();  // Update data, read voltage on analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    delay(200);  // Adding a delay to stabilize readings
  }

  // Average of 10 readings for calibration value
  MQ135.setR0(calcR0 / 10);  
  Serial.println(" done!");

  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue found, R0 is infinite");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero");
    while (1);
  }

  MQ135.serialDebug(false);  // Disable serial debug output from the sensor
}

void loop() {
  // Reconnect to WiFi and MQTT broker if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    WiFi.reconnect();
    delay(5000);
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read temperature and humidity from DHT22 sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Update and read gas concentrations from MQ135 sensor
  MQ135.update(); 

  // Set the regression method to calculate the gas concentration
  MQ135.setA(110.47); MQ135.setB(-2.862); //CO2
  float CO2 = MQ135.readSensor() + 400;

  MQ135.setA(605.18); MQ135.setB(-3.937); //CO
  float CO = MQ135.readSensor();

  MQ135.setA(102.2); MQ135.setB(-2.473);  //NH4
  float NH4 = MQ135.readSensor();

  // Get the current time
  timeClient.update();
  String iTime = timeClient.getFormattedTime();

  // JSON payload with gas and environmental data
  String payload = "{";
  payload += "\"temperature\":" + String(temperature, 2) + ",";
  payload += "\"humidity\":" + String(humidity, 2) + ",";
  payload += "\"CO2\":" + String(CO2, 2) + ",";
  payload += "\"CO\":" + String(CO, 2) + ",";
  payload += "\"NH4\":" + String(NH4, 2) + ",";
  payload += "\"timestamp\":\"" + iTime + "\"";
  payload += "}";

  // convert payload to char array for MQTT
  char msg[300];
  payload.toCharArray(msg, 300);

  // Publish the payload to the MQTT broker
  client.publish("sensors", msg);

  // Debug output to Serial
  Serial.print("Published data: ");
  Serial.println(msg);

  delay(15000);  // Wait for 15 seconds before sending the next data
}
