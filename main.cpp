#include <Arduino.h> 

#include <WiFi.h>
#include <PubSubClient.h>

// ---- Wi-Fi ----
const char* ssid     = "*********";
const char* password = "**********";

// ---- MQTT ----
const char* mqtt_server = "broker.hivemq.com"; // IP de ton broker
WiFiClient espClient;
PubSubClient client(espClient);
const char* topic = "home/temperature";

// ---- Connexion MQTT ----
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    if (client.connect("ESP32_TEMP_stm32")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2s");
      delay(2000);
    }
  }
}

#define RXD2 16  // RX ESP32 → TX STM32
#define TXD2 17  // TX ESP32 → RX STM32
String temperatureData = "";

void setup() {
  Serial.begin(115200);       // Moniteur USB
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // UART2 pour STM32
  Serial.println("UART2 Ready...");

// ---- Connexion Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");

  // ---- Connexion MQTT ----
  client.setServer(mqtt_server, 1883);
  connectMQTT();
}

void loop() {
    if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

 // ---- Lecture UART ----
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c >= 32 || c == '\n') { // ignore caractères non imprimables sauf '\n'
      temperatureData += c;
    }
  }

  // ---- Publication si on a une ligne complète ----
  if (temperatureData.indexOf("\n") != -1) {
    temperatureData.trim(); // supprime \r ou espaces
    Serial.println("Temp received: " + temperatureData);

    // Publie sur MQTT
  
    client.publish(topic, temperatureData.c_str());

    temperatureData = ""; // réinitialise pour la prochaine lecture
  }
  delay(100);
}
