#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---- Wi-Fi ----
const char* ssid     = "Galaxy A04s 3535";
const char* password = "crar5370";

// ---- MQTT (Mosquitto public) ----
const char* mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);

// ---- Topics ----
const char* topicV1 = "stm32/pot/V1";
const char* topicV2 = "stm32/pot/V2";
const char* topicV3 = "stm32/pot/V3";

// ---- UART ----
#define RXD2  25 //16  // vert RX ESP32 ← TX STM32
#define TXD2 26 //17  // TX ESP32 → RX STM32

String uartBuffer = "";

// ---- Connexion MQTT ----
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_POT_NODE")) {
      Serial.println(" connected");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("UART2 Ready");

  // ---- Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  // ---- MQTT ----
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
    if (c == '\n') {
      // Ligne complète reçue
      uartBuffer.trim();
      Serial.println("UART received: " + uartBuffer);

      // ---- Extraction des 3 valeurs ----
      int firstComma  = uartBuffer.indexOf(',');
      int secondComma = uartBuffer.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        String V1 = uartBuffer.substring(0, firstComma);
        String V2 = uartBuffer.substring(firstComma + 1, secondComma);
        String V3 = uartBuffer.substring(secondComma + 1);

        // ---- Publication MQTT ----
        client.publish(topicV1, V1.c_str());
        client.publish(topicV2, V2.c_str());
        client.publish(topicV3, V3.c_str());

        Serial.println("Published:");
        Serial.println(" V1 = " + V1);
        Serial.println(" V2 = " + V2);
        Serial.println(" V3 = " + V3);
      }

      uartBuffer = ""; // reset buffer
    } else if (c >= 32) {
      uartBuffer += c;
    }
  }

  delay(50);
}
