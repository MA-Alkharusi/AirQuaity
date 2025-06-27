/*Mohammed AL Kharusi
2024/2025 FYP
Air Quality Monitoring System
This code is used to read data from, SEN54, DHT22 and MQ135 sensors and publish it to the HiveMQ Cloud MQTT broker.
*/

/*
To add the credentials
1- Connect to Air_Quality_Over_Wifi
2- use 12345678 as the password
3- Open a browser and go to
        http://192.168.4.1
4- Add the credentials
5- Save and restart the ESP32

To reset the credentials
1- press the reset button on the device
2- Connect to AirQualityWIFI
3- delete the old credentials by going to
        http://192.168.4.1/reset
4- add the new credentials by going to
        http://192.168.4.1
*/

#include <Wire.h>             // Include the Wire library for I2C communication
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>  // For SSL connection
#include <DHT.h>               // Include the DHT library for DHT22 sensor
#include <NTPClient.h>         // Library for network time protocol
#include <WiFiUdp.h>           // Required for network time protocol
#include "MQUnifiedsensor.h"   // Include the MQUnifiedsensor library
#include "SensirionI2CSen5x.h" // Include the SensirionI2CSen5x library for SEN54 sensor
#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library for LCD display
#include <Preferences.h>       // Include the Preferences library for storing credentials
#include <WebServer.h>         // Include the WebServer library allowing to create a web server using ESP32
#include <ArduinoJson.h>

// Define a struct to hold sensor data
struct SensorData {
  float dhtTemp = 0;
  float dhtHumidity = 0;
  float CO2 = 0;
  float CO = 0;
  float temperature = 0;
  float humidity = 0;
  float voc = 0;
  float nox = 0;
  float pm1p0 = 0;
  float pm2p5 = 0;
  float pm4p0 = 0;
  float pm10p0 = 0;
};

// Access point Button pin
#define AP_BUTTON_PIN 15

// Access point mode variables allowing the ESP32 to act as an access point
volatile bool apModeButtonPressed = false; // Set by ISR when button pressed
bool apModeActive = false;                 // True when AP mode is active
unsigned long apModeStartTime = 0;        // Time when AP mode started (for timeout)
volatile unsigned long lastAPModeTime = 0; 
const unsigned long apDebounceDelay = 50; // Debounce delay for the AP button
const unsigned long apTimeout = 600000;   // 10 minutes timeout for AP mode

//WiFi AP Mode Configuration allows the ESP32 to act as an access point
#define AP_SSID "AirQualityWIFI"
#define AP_PASSWORD "12345678"

//define static IP address for AP mode
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Create an instance of the Preferences library to store credentials
Preferences preferences;
// Create a web server on port 80
WebServer server(80); 

//WiFi & MQTT Clients and NTP Client for time synchronization
WiFiClientSecure espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
// Create an instance of the NTPClient library to sync time every 60 seconds
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 60000); 

// WIFI and MQTT credentials vrariables
String storedSSID, storedPassword, storedMQTTServer, storedMQTTPort, storedMQTTUser, storedMQTTPassword;

// Sensor warm-up period
const unsigned long SENSOR_WARMUP_TIME = 60000;

// Load credentials from the Preferences library
void loadCredentials() {
  preferences.begin("credentials", true);
  storedSSID = preferences.getString("ssid", "");
  storedPassword = preferences.getString("password", "");
  storedMQTTServer = preferences.getString("mqtt_server", "");
  storedMQTTPort   = preferences.getString("mqtt_port", "");
  storedMQTTUser = preferences.getString("mqtt_user", "");
  storedMQTTPassword = preferences.getString("mqtt_pass", "");
  preferences.end();
}

// Save credentials to the Preferences library
void saveCredentials(String ssid, String password, String mqttServer, String mqttPort, String mqttUser, String mqttPass) {
  preferences.begin("credentials", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putString("mqtt_server", mqttServer);
  preferences.putString("mqtt_port", mqttPort);
  preferences.putString("mqtt_user", mqttUser);
  preferences.putString("mqtt_pass", mqttPass);
  preferences.end();
}

// Create a custome Web Configuration page
void handleRoot() {
    Serial.println(" Serving Config Page...");
    
    String page = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Air Quality Wi-Fi & HiveMQ Configuration</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: linear-gradient(to right, #121212, #1e1e1e);
      color: white;
      margin: 0;
      padding: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      min-height: 100vh;
    }
    .container {
      background: rgba(255, 255, 255, 0.15);
      padding: 25px;
      border-radius: 12px;
      box-shadow: 0px 5px 20px rgba(0, 0, 0, 0.4);
      width: 90%;
      max-width: 400px;
      text-align: center;
      box-sizing: border-box;
    }
    .input-group {
      position: relative;
      margin-bottom: 16px;
      text-align: left;
    }
    label {
      display: block;
      margin-bottom: 4px;
      font-weight: bold;
      font-size: 0.95rem;
    }
    .input-group input {
      width: 100%;
      padding: 12px 50px 12px 12px;
      border: none;
      border-radius: 8px;
      background: rgba(255, 255, 255, 0.2);
      color: white;
      font-size: 16px;
      outline: none;
      box-sizing: border-box;
    }
    .input-group input:focus {
      background: rgba(255, 255, 255, 0.3);
      border: 1px solid #32a5b9;
    }
    .toggle-btn {
      position: absolute;
      right: 10px;
      top: 50%;
      transform: translateY(-50%);
      background: transparent;
      border: none;
      color: #32a5b9;
      cursor: pointer;
      font-size: 14px;
      outline: none;
    }
    .btn {
      background: #004196;
      color: white;
      padding: 12px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      font-size: 16px;
      transition: 0.3s ease-in-out;
      width: 100%;
      box-sizing: border-box;
    }
    .btn:hover {
      background: #32a5b9;
      transform: scale(1.05);
    }
    h2 {
      margin-bottom: 15px;
      font-size: 1.2rem;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>Air Quality Monitor</h2>
    <form action="/save" method="post">
      <div class="input-group">
        <label for="ssid">Wi-Fi SSID:</label>
        <input type="text" id="ssid" name="ssid" value="%SSID%" required>
      </div>
      <div class="input-group">
        <label for="password">Wi-Fi Password:</label>
        <input type="password" id="password" name="password" value="%PASSWORD%" required>
        <button type="button" class="toggle-btn" onclick="togglePassword('password', this)">Show</button>
      </div>
      <div class="input-group">
        <label for="mqtt_server">MQTT Server:</label>
        <input type="text" id="mqtt_server" name="mqtt_server" value="%MQTT%" required>
      </div>
      <div class="input-group">
        <label for="mqtt_port">MQTT Port:</label>
        <input type="text" id="mqtt_port" name="mqtt_port" value="%MQTTPORT%" required>
      </div>
      <div class="input-group">
        <label for="mqtt_user">MQTT User:</label>
        <input type="text" id="mqtt_user" name="mqtt_user" value="%MQTTUSER%">
      </div>
      <div class="input-group">
        <label for="mqtt_pass">MQTT Password:</label>
        <input type="password" id="mqtt_pass" name="mqtt_pass" value="%MQTTPASS%">
        <button type="button" class="toggle-btn" onclick="togglePassword('mqtt_pass', this)">Show</button>
      </div>
      <button class="btn" type="submit">Save Configuration</button>
    </form>
  </div>
  <script>
    function togglePassword(fieldId, btn) {
      var input = document.getElementById(fieldId);
      if (input.type === "password") {
        input.type = "text";
        btn.textContent = "Hide";
      } else {
        input.type = "password";
        btn.textContent = "Show";
      }
    }
  </script>
</body>
</html>
)rawliteral";

page.replace("%SSID%", storedSSID);
page.replace("%PASSWORD%", storedPassword);
page.replace("%MQTT%", storedMQTTServer);
page.replace("%MQTTPORT%", storedMQTTPort);
page.replace("%MQTTUSER%", storedMQTTUser);
page.replace("%MQTTPASS%", storedMQTTPassword);

    server.send(200, "text/html", page);
}


// Handle the form submission and save the credentials
void handleSave() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  String mqttServer = server.arg("mqtt_server");
  String mqttPort   = server.arg("mqtt_port");
  String mqttUser = server.arg("mqtt_user");
  String mqttPass = server.arg("mqtt_pass");

  if (!ssid.isEmpty() && !mqttServer.isEmpty() && !mqttPort.isEmpty()) {
      saveCredentials(ssid, password, mqttServer, mqttPort, mqttUser, mqttPass);
      server.send(200, "text/plain", " Credentials saved! Restarting ESP...");
      delay(5000);  // Allow time for response to reach client before restarting
      ESP.restart();
  } else {
      server.send(400, "text/plain", " Invalid Input! All fields are required.");
  }
}

// Connect to Wi-Fi
bool connectToWiFi() {
  loadCredentials();
  if (storedSSID.isEmpty()) return false;

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(storedSSID);

  int maxRetries = 5;
  for (int i = 0; i < maxRetries; i++) {
    WiFi.begin(storedSSID.c_str(), storedPassword.c_str());
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWi-Fi connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    Serial.println("\nFailed to connect to Wi-Fi. Retrying...");
    delay(5000); // Wait 5 seconds before retrying
  }
  Serial.println("Wi-Fi connection failed after retries.");
  return false;
}

// Connect to MQTT
bool connectToMQTT() {
  loadCredentials();
  if (storedMQTTServer.isEmpty()) {
    Serial.println("No MQTT Server Found in Preferences!");
    return false;
  }

  Serial.print("Connecting to MQTT Broker: ");
  Serial.println(storedMQTTServer);

  int maxRetries = 5;
  for (int i = 0; i < maxRetries; i++) {
    client.setServer(storedMQTTServer.c_str(), storedMQTTPort.toInt());
    espClient.setInsecure();
    if (client.connect("ESP32_Client", storedMQTTUser.c_str(), storedMQTTPassword.c_str())) {
      Serial.println("Connected to MQTT Broker!");
      return true;
    }
    Serial.print("MQTT Connection Failed! State: ");
    Serial.println(client.state());
    delay(5000); // Wait 5 seconds before retrying
  }
  Serial.println("MQTT connection failed after retries.");
  return false;
}

// handle a reset request
void handleReset() {
  Serial.println("Reset requested. Clearing credentials and restarting...");
  
  // Clear all stored credentials
  preferences.begin("credentials", false);
  preferences.clear();
  preferences.end();
  
  // Send a response to the client
  server.send(200, "text/plain", "Resetting to factory defaults. Restarting ESP32...");
  // Give the client time to receive the response before restarting
  delay(2000);
  ESP.restart();
}

// access point button force to use IRAM_ATTR to run in IRAM memory to avoid stack overflow
void IRAM_ATTR apModeButtonISR() {
  unsigned long currentTime = millis();
   if (currentTime - lastAPModeTime >= apDebounceDelay) {
   apModeButtonPressed = true;
   lastAPModeTime = currentTime;
 }
}

// Activate the Access Point mode
void activateAPMode() {
  Serial.println("Activating AP Mode for credential reset (10 minutes active)");
  
  // If currently in station mode, switch to dual mode (AP and STA) to keep normal operations running
  if (WiFi.getMode() == WIFI_STA) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  
  // Configure the soft Access Point IP and start the AP.
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  // Start the web server and handle the root, save and reset routes
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", HTTP_GET, handleReset);
  server.begin();
  
  Serial.print("AP Mode IP Address: ");
  Serial.println(WiFi.softAPIP());
}

// Stop the Access Point mode
void deactivateAPMode() {
  Serial.println("Deactivating AP Mode");
  server.stop();                     // Stop the web server
  WiFi.softAPdisconnect(true);       // Turn off the access point
  
    WiFi.mode(WIFI_STA); 
    connectToWiFi();
  apModeActive = false;
}


// Create a global SensorData instance and a pointer to it
SensorData sensorData;
SensorData* sensorDataPtr = &sensorData;

//global flag to indicate the screen needs to be updated
volatile bool screenNeedsUpdate = false;

//SDA and SCL pins for I2C communication with the SEN54 sensor
#define I2C_SDA 21  // GPIO21 for SDA
#define I2C_SCL 22  // GPIO22 for SCL

// LCD Button pin and variables
#define BUTTON_PIN 2 // GPIO2 pin
volatile int screenIndex = 0; // To track screen 
unsigned long lastDebounceTime = 0; // Debounce timer avoiding multiple presses
const unsigned long debounceDelay = 50; // Debounce delay

// Initialize the LCD (address 0x27 and 16 columns x 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define DHT22 sensor parameters (GPIO4)
#define DHTPIN 4           
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define MQ135 sensor parameters
#define MQ135_PIN 1
#define Voltage_Resolution 5.0 // Voltage used
#define ADC_Bit_Resolution 12 // Analog to Digital Converter Resolution
#define RatioMQ135CleanAir 3.6 // Ratio of Rs/Ro in clean air
MQUnifiedsensor MQ135("ESP-32", Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, "MQ-135");

// SEN54 Sensor
SensirionI2CSen5x sen5x;

// function to repeat spaces for centering text on LCD
String repeatSpaces(int count) {
  String result = "";
  for (int i = 0; i < count; i++) {
    result += " ";
  }
  return result;
}

// Function to display text on LCD centered
void displayLCDCentered(String line1, String line2) {
  lcd.clear();
  // Center align line 1
  int pad1 = (16 - line1.length()) / 2;
  String paddedLine1 = repeatSpaces(pad1) + line1;
  lcd.setCursor(0, 0);
  lcd.print(paddedLine1);

  // center align line 2
  int pad2 = (16 - line2.length()) / 2;
  String paddedLine2 = repeatSpaces(pad2) + line2;
  lcd.setCursor(0, 1);
  lcd.print(paddedLine2);
}

// Function to display data on the LCD
void updateLCD(SensorData* data) {
  switch (screenIndex) {
    case 0:
      // DHT22 readings
      displayLCDCentered("DHT Temp: " + String(data->dhtTemp, 1) + " C", "DHT Hum: " + String(data->dhtHumidity, 1) + " %");
      break;
    case 1:
      // SEN54 temperature & humidity
      displayLCDCentered("SEN Temp: " + String(data->temperature, 1) + " C", "SEN Hum: " + String(data->humidity, 1) + " %");
      break;
    case 2:
      // SEN54 particulate matter (PM1.0 & PM2.5)
      displayLCDCentered("PM1.0: " + String(data->pm1p0, 1), "PM2.5: " + String(data->pm2p5, 1));
      break;
    case 3:
      // SEN54 particulate matter (PM4.0 & PM10)
      displayLCDCentered("PM4.0: " + String(data->pm4p0, 1), "PM10: " + String(data->pm10p0, 1));
      break;
    case 4:
      // SEN54 & MQ135 gas readings (VOC & CO2)
      displayLCDCentered("VOC: " + String(data->voc, 1)+ " ppm", "CO2: " + String(data->CO2, 1)+ " ppm");
      break;
    case 5:
      // MQ135 gas readings ( CO)
      displayLCDCentered("CO: " + String(data->CO, 2)," ppm");
      break;
    default:
      // Fallback to screen 0 if something goes wrong
      displayLCDCentered("DHT Temp: " + String(data->dhtTemp, 1) + " C","DHT Hum: " + String(data->dhtHumidity, 1) + " %");
      break;
  }
}

// update screenIndex
void buttonPressed() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) { // Debounce logic
    screenIndex = (screenIndex + 1) % 6; // Cycle through 6 screens so if it reaches 6 it goes back to 0
    screenNeedsUpdate = true;           // Set the flag to indicate LCD update
    lastDebounceTime = currentTime;
  }
}

/*
the setup function initializes the serial communication, WiFi connection, MQTT client, SEN54, DHT22 sensor, NTP client, and MQ135 sensor.
*/
void setup() {
  Serial.begin(115200);
  Serial.println("Air Quality Monitoring System");

 // Start the Access Point mode if no Wi-Fi credentials are found or no MQTT server is found
 if (!connectToWiFi()) {
  Serial.println("Failed to connect to Wi-Fi! Starting AP Mode...");
  activateAPMode();              
  apModeActive = true;           // Set the flag
  apModeStartTime = millis();    // Record the start time
} else {
  Serial.println("Connected to Wi-Fi!");
  connectToMQTT();
}

  // Initialize I2C communication for SEN54 sensor
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  // Initialize DHT22 sensor
  dht.begin();

  // Initialize Network Time Protocol (NTP) client
  timeClient.begin();
  espClient.setInsecure();

  // Initialize MQ135 sensor
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

  // Initialize SEN54 sensor
sen5x.begin(Wire);  // Initialize the SEN54 sensor
if (sen5x.startMeasurement()) {
  Serial.println("SEN54 measurement started successfully");
} else {
  Serial.println("Failed to start SEN54 measurement");
}

 // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Initialize switch button
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Configure button pin with internal pull up resistor
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);

    // Initialize the AP mode button
    pinMode(AP_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(AP_BUTTON_PIN), apModeButtonISR, FALLING);

  if (!connectToWiFi()) {
    activateAPMode();
  } else {
    connectToMQTT();
  }
  // Update LCD
  updateLCD(sensorDataPtr);
}


void loop() {

  server.handleClient();
  client.loop();

    // Check for AP mode button presses and activate/deactivate AP mode
    if (apModeButtonPressed) {
      apModeButtonPressed = false;  // clear the flag
      
      if (apModeActive) {
        // Second click: deactivate AP mode
        deactivateAPMode();
        apModeActive = false;
      } else {
        // First click: activate AP mode and record the start time
        activateAPMode();
        apModeActive = true;
        apModeStartTime = millis();
      }
    }

  // Check if AP mode has been active for 10 minutes
  if (apModeActive && (millis() - apModeStartTime >= apTimeout)) {
    Serial.println("AP Mode timeout reached, deactivating AP Mode");
    deactivateAPMode();
    apModeActive = false;
  }

  // Check Wi-Fi connection and attempt to reconnect
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting reconnection...");
    connectToWiFi();
  }

    // Check MQTT connection and attempt to reconnect
    if (!client.connected()) {
      Serial.println("MQTT disconnected, attempting reconnection...");
      connectToMQTT();
    }

  unsigned long currentMillis = millis();  // current time in milliseconds

    // During the sensor warm-up period, display a message and skip sensor reading/publishing
    if (currentMillis < SENSOR_WARMUP_TIME) {
      static bool warmupShown = false;
      if (!warmupShown) {
        displayLCDCentered("Heating Sensor", "Please Wait...");
        Serial.println("Sensor warming up...");
        warmupShown = true;
      }
      return;
    }

// Sensor data and MQTT publish timers
static unsigned long lastSensorUpdate = 0;   
static unsigned long lastMqttPublish = 0;   
   
//logic update
if (currentMillis - lastSensorUpdate >= 50000) { // Check if 50 seconds have passed
    lastSensorUpdate = currentMillis;

  // Read DHT22 data
  sensorDataPtr->dhtTemp = dht.readTemperature();
    sensorDataPtr->dhtHumidity = dht.readHumidity();
    if (isnan(sensorDataPtr->dhtTemp) || isnan(sensorDataPtr->dhtHumidity)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

  // Read MQ135 data
  MQ135.update();
  MQ135.setA(110.47); 
  MQ135.setB(-2.862);
  sensorDataPtr->CO2 = MQ135.readSensor() + 400;

  MQ135.setA(605.18);
  MQ135.setB(-3.937);
  sensorDataPtr->CO  = MQ135.readSensor();

uint16_t error = sen5x.readMeasuredValues(sensorDataPtr->pm1p0, sensorDataPtr->pm2p5, sensorDataPtr->pm4p0, sensorDataPtr->pm10p0,
                                            sensorDataPtr->humidity, sensorDataPtr->temperature, sensorDataPtr->voc, sensorDataPtr->nox);
if (error != 0) {
    Serial.println("Failed to read from SEN54 sensor! Reinitializing...");
    // Attempt to reinitialize SEN54
    sen5x.begin(Wire);
    if (sen5x.startMeasurement()) {
        Serial.println("SEN54 reinitialized successfully");
    } else {
        Serial.println("SEN54 reinitialization failed");
    }
} else {
    Serial.println("Sensor data updated successfully");
}
    // Update LCD after sensor data is updated
    updateLCD(sensorDataPtr);
    }
 // Update the LCD immediately if the button was pressed
  if (screenNeedsUpdate) {
    screenNeedsUpdate = false; // Reset the flag
    updateLCD(sensorDataPtr);  // Update the LCD with the current screen
    Serial.print("Screen updated to index: ");
    Serial.println(screenIndex);
  }

  // Get current time
  timeClient.update();
  String timestamp = timeClient.getFormattedTime();

 // MQTT Publish Logic
  if (currentMillis - lastMqttPublish >= 60000) { // Publish every 60 seconds
    lastMqttPublish = currentMillis;

    StaticJsonDocument<512> jsonDoc;
    jsonDoc["DHT22_Temperature"] = sensorDataPtr->dhtTemp;
    jsonDoc["DHT22_Humidity"] = sensorDataPtr->dhtHumidity;
    jsonDoc["SEN54_Temperature"] = sensorDataPtr->temperature;
    jsonDoc["SEN54_Humidity"] = sensorDataPtr->humidity;
    jsonDoc["CO2"] = sensorDataPtr->CO2;
    jsonDoc["CO"] = sensorDataPtr->CO;
    jsonDoc["VOC"] = sensorDataPtr->voc;
    jsonDoc["PM1_0"] = sensorDataPtr->pm1p0;
    jsonDoc["PM2_5"] = sensorDataPtr->pm2p5;
    jsonDoc["PM4_0"] = sensorDataPtr->pm4p0;
    jsonDoc["PM10"] = sensorDataPtr->pm10p0;
    jsonDoc["timestamp"] = timeClient.getFormattedTime();
    
    char payload[256];
    serializeJson(jsonDoc, payload, sizeof(payload));
    client.publish("sensors", payload);
    Serial.print("Published: ");
    Serial.println(payload);
}

}