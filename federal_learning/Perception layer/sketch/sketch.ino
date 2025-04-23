#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Network credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE 150
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected to MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_MPU6050")) {
      Serial.println("connected");
      // Publish connection status
      client.publish("status", "ESP32_MPU6050 connected");
      // Subscribe to input topic
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050. Please check wiring.");
    while (1);
  }
  Serial.println("MPU6050 found and initialized");

  // Connect to WiFi and MQTT broker
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish sensor data every 200ms
  if (millis() - lastMsg > 200) {
    lastMsg = millis();

    // Read accelerometer data
    sensors_event_t a;
    mpu.getAccelerometerSensor()->getEvent(&a);

    // Read gyroscope data
    sensors_event_t g;
    mpu.getGyroSensor()->getEvent(&g);

    // Format sensor data as JSON for MQTT
  snprintf(msg, MSG_BUFFER_SIZE, "(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
         a.acceleration.x, a.acceleration.y, a.acceleration.z,
         g.gyro.x, g.gyro.y, g.gyro.z);

    // Publish sensor data
    client.publish("IMUSensor", msg);

    // Print sensor data to Serial for debugging
    Serial.print("Acceleration X:");
    Serial.print(a.acceleration.x);
    Serial.print(" Y:");
    Serial.print(a.acceleration.y);
    Serial.print(" Z:");
    Serial.print(a.acceleration.z);
    Serial.print(" | Gyroscope X:");
    Serial.print(g.gyro.x);
    Serial.print(" Y:");
    Serial.print(g.gyro.y);
    Serial.print(" Z:");
    Serial.println(g.gyro.z);
  }
}