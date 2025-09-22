# 📡 Multi-Node STM32 CAN Network with ESP32 MQTT Bridge & Node-RED Dashboard

## 🚀 Présentation

Ce projet met en œuvre un système embarqué distribué basé sur plusieurs **STM32F4** et **ESP32**.
L’objectif est de créer un **réseau CAN basse consommation** entre différents nœuds STM32, avec un **nœud central** qui collecte, sauvegarde et transfère les données vers l’IoT via MQTT.
Les données sont ensuite affichées sur **Node-RED** et sur **smartphone via Bluetooth**.

---

## 🖥️ Architecture du système

```
 [STM32 #1] Potentiomètres  ┐
                            │
                            │   CAN Bus
                            │
 [STM32 #2] Température     ┘
         ↓ (Sleep mode)
 ─────────────────────────────────────
 [STM32 #3] Nœud Central
   - Reçoit les données
   - Sauvegarde en Backup SRAM
   - Envoie via USART → ESP32
   - Passe en mode Standby
 ─────────────────────────────────────
 [ESP32 #1] MQTT Publisher → Broker Mosquitto
 ─────────────────────────────────────
                🌐 Internet
 ─────────────────────────────────────
 [ESP32 #2] MQTT Subscriber → UART → STM32 #4 → Bluetooth → Smartphone
 ─────────────────────────────────────
                       📊 Node-RED Dashboard
```

---

## 🔧 Fonctionnalités

* 📡 **Communication CAN** entre 3 cartes STM32F4.
* 💤 **Low-power mode (Sleep & Standby)** pour optimiser la consommation.
* 💾 **Sauvegarde des états en Backup SRAM** du nœud central.
* 🔄 **Communication série (USART)** entre STM32 et ESP32.
* 🌍 **Transmission MQTT via ESP32** vers un broker Mosquitto.
* 📊 **Interface Node-RED** pour visualisation en temps réel.
* 📱 **Affichage mobile via Bluetooth** (STM32 + ESP32 bridge).

---

## 🛠️ Composants utilisés

* 4x STM32F407 discovery
* 3x MCP transceiver 2551
* 2x ESP32 Wi-Fi/Bluetooth
* 1x Broker MQTT (Mosquitto)
* Potentiomètres (x3)
* Capteur de température  DS1621 
* Smartphone avec application Bluetooth
* Interface **Node-RED**

---

## ⚙️ Fonctionnement détaillé

1. **Acquisition des données** :

   * STM32 #1 lit les **3 potentiomètres**.
   * STM32 #2 lit la **température**.
   * Les deux passent ensuite en **mode Sleep**.

2. **Collecte par le nœud central** :

   * STM32 #3 envoie des **commandes CAN** pour demander les données.
   * Sauvegarde des mesures dans la **Backup SRAM**.
   * Transmission vers l’ESP32 via **USART**.
   * Passage en **mode Standby**.

3. **Envoi vers le Cloud** :

   * L’ESP32 publie les données sur le **broker MQTT Mosquitto**.

4. **Diffusion des données** :

   * ESP32 #2 récupère les données du serveur MQTT.
   * Transmission vers STM32 #4, qui les diffuse via **Bluetooth** au smartphone.
   * Affichage parallèle sur **Node-RED Dashboard**.

---

## 🖥️ Installation & Utilisation

1. **STM32**

   * Compiler et flasher le firmware (CAN + USART + Low Power).
   * Configurer le nœud central avec **Backup SRAM** et **Standby mode**.

2. **ESP32**

   * Flasher le code avec **Arduino IDE** ou **PlatformIO**.
   * Configurer les identifiants Wi-Fi et l’adresse du broker Mosquitto.

3. **MQTT Broker (Mosquitto)**

   * Installer Mosquitto sur PC ou serveur cloud :

     ```bash
     sudo apt install mosquitto mosquitto-clients
     ```
   * Lancer le broker sur le port `1883`.

4. **Node-RED**

   * Installer Node-RED :

     ```bash
     npm install -g --unsafe-perm node-red
     ```
   * Ajouter un **node MQTT subscriber** pour afficher les données.

5. **Smartphone**

   * Connecter via Bluetooth au STM32 #4.
   * Afficher les données avec une application type **Serial Bluetooth Terminal**.

---


