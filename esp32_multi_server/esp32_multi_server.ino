#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <BluetoothSerial.h>
#include <SPIFFS.h>
#include "mbedtls/gcm.h"
#include "secret_config.h"

#define AP_SSID "ESP32-GPS-Config"
#define AP_PASSWORD "12345678"
#define WIFI_TIMEOUT 20000
#define RESET_BUTTON_PIN 0
#define BUTTON_HOLD_TIME 3000
#define BT_DEVICE_NAME "ESP32_GPS_Tracker"
#define NUM_SERVERS 4

struct ServerConfig {
  const char* host;
  int port;
  bool enabled;
};

ServerConfig servers[NUM_SERVERS] = {
  { "uesteban.ddnsking.com", 5051, true },
  { "jesucaracu.ddns.net", 5051, true },
  { "chidrobo.ddnsking.com", 5051, true },
  { "ivbarrios.ddns.net", 5051, false }
};

WiFiUDP udp;
WebServer server(80);
Preferences preferences;
mbedtls_gcm_context aes;
BluetoothSerial SerialBT;

unsigned long lastSend = 0;
const unsigned long sendInterval = 10000;
unsigned long lastBTCheck = 0;
const unsigned long btCheckInterval = 1000;
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;
bool gpsValid = false;
bool configMode = false;
bool bluetoothConnected = false;
double lastLat = 0.0;
double lastLng = 0.0;
String deviceID = "";
String btBuffer = "";
int sendSuccess[NUM_SERVERS] = { 0, 0, 0, 0 };
int sendFails[NUM_SERVERS] = { 0, 0, 0, 0 };
String savedSSID = "";
String savedPassword = "";

void handleRoot() {
  File file = SPIFFS.open("/config.html", "r");
  if (!file) {
    server.send(500, "text/plain", "Error: config.html not found");
    Serial.println("Error: config.html not found in SPIFFS");
    return;
  }

  String html = file.readString();
  file.close();

  preferences.begin("wifi", true);
  String currentSSID = preferences.getString("ssid", "Ninguna");
  preferences.end();

  html.replace("CURRENT_SSID_PLACEHOLDER", currentSSID);

  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String ssid = server.arg("ssid");
    String password = server.arg("password");

    preferences.begin("wifi", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();

    Serial.println("\n✓ Credenciales guardadas:");
    Serial.println("  SSID: " + ssid);

    server.send(200, "text/plain", "OK");

    delay(1000);
    Serial.println("\n🔄 Reiniciando ESP32...");
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void checkResetButton() {
  bool buttonPressed = (digitalRead(RESET_BUTTON_PIN) == LOW);

  if (buttonPressed && !buttonWasPressed) {
    buttonPressStart = millis();
    buttonWasPressed = true;
    Serial.println("\n🔘 Botón presionado - mantén 3 segundos para modo config...");
  }

  if (buttonPressed && buttonWasPressed) {
    unsigned long pressDuration = millis() - buttonPressStart;

    if (pressDuration >= BUTTON_HOLD_TIME && !configMode) {
      Serial.println("\n✓ Botón mantenido 3 segundos!");
      Serial.println("🔧 Entrando en modo configuración...");

      WiFi.disconnect();
      delay(100);
      startConfigMode();
    }
  }

  if (!buttonPressed && buttonWasPressed) {
    buttonWasPressed = false;
  }
}

void startConfigMode() {
  configMode = true;

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║         🔧 MODO CONFIGURACIÓN ACTIVADO         ║");
  Serial.println("╚════════════════════════════════════════════════╝");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress IP = WiFi.softAPIP();

  Serial.println("\n📱 ACCEDE A LA CONFIGURACIÓN:");
  Serial.println("   ────────────────────────────────────────");
  Serial.println("   1. Conecta tu teléfono/PC al WiFi:");
  Serial.printf("      Red: %s\n", AP_SSID);
  Serial.printf("      Contraseña: %s\n", AP_PASSWORD);
  Serial.println("\n   2. Abre tu navegador y ve a:");
  Serial.print("      http://");
  Serial.println(IP);
  Serial.println("   ────────────────────────────────────────");

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  Serial.println("✓ Servidor web iniciado");
  Serial.println("⏳ Esperando configuración...\n");
}

bool connectToWiFi() {
  preferences.begin("wifi", true);
  savedSSID = preferences.getString("ssid", "");
  savedPassword = preferences.getString("password", "");
  preferences.end();

  if (savedSSID.length() == 0) {
    Serial.println("⚠️  No hay credenciales WiFi guardadas");
    return false;
  }

  Serial.println("\n🔗 Intentando conectar a WiFi...");
  Serial.println("   SSID: " + savedSSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi conectado exitosamente");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("   RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm\n");
    return true;
  } else {
    Serial.println("✗ No se pudo conectar a WiFi");
    return false;
  }
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("\n🔵 Cliente Bluetooth conectado!");
    bluetoothConnected = true;
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("\n⚪ Cliente Bluetooth desconectado");
    bluetoothConnected = false;
    deviceID = "";
    gpsValid = false;
  }
}

bool parseGPSData(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1) {
    return false;
  }

  String receivedDeviceID = data.substring(0, firstComma);
  String latStr = data.substring(firstComma + 1, secondComma);
  String lngStr = data.substring(secondComma + 1);

  double lat = latStr.toDouble();
  double lng = lngStr.toDouble();

  if (lat >= -90 && lat <= 90 && lng >= -180 && lng <= 180) {
    deviceID = receivedDeviceID;
    lastLat = lat;
    lastLng = lng;
    gpsValid = true;

    Serial.printf("📍 GPS recibido: %.6f, %.6f (Device: %s)\n", lat, lng, deviceID.c_str());
    return true;
  }

  return false;
}

void readBluetoothData() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        if (parseGPSData(btBuffer)) {
          // Successfully parsed GPS data
        } else {
          Serial.println("⚠️  Datos GPS inválidos: " + btBuffer);
        }
        btBuffer = "";
      }
    } else {
      btBuffer += c;

      if (btBuffer.length() > 200) {
        Serial.println("⚠️  Buffer overflow, limpiando...");
        btBuffer = "";
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  ESP32 Bluetooth GPS Tracker with Multi-Server║");
  Serial.println("║          Encrypted Data Transmission          ║");
  Serial.println("╚════════════════════════════════════════════════╝");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("✗ Error montando SPIFFS");
    Serial.println("⚠️  Continuando sin portal web...");
  } else {
    Serial.println("✓ SPIFFS montado correctamente");
  }

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  Serial.printf("✓ Botón de reset configurado en GPIO %d\n", RESET_BUTTON_PIN);

  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("✗ Error inicializando Bluetooth!");
    while (1) delay(1000);
  }
  SerialBT.register_callback(btCallback);
  Serial.printf("✓ Bluetooth inicializado: %s\n", BT_DEVICE_NAME);
  Serial.println("  Esperando conexión desde Android...");

  mbedtls_gcm_init(&aes);
  int ret = mbedtls_gcm_setkey(&aes, MBEDTLS_CIPHER_ID_AES, AES_KEY, 128);
  if (ret != 0) {
    Serial.printf("✗ Error inicializando AES: -0x%04x\n", -ret);
    while (1) delay(1000);
  }
  Serial.println("✓ AES-128-GCM inicializado");

  Serial.println("\n📡 Servidores configurados:");
  for (int i = 0; i < NUM_SERVERS; i++) {
    Serial.printf("   [%d] %s:%d - %s\n",
                  i + 1,
                  servers[i].host,
                  servers[i].port,
                  servers[i].enabled ? "✓" : "✗");
  }

  if (!connectToWiFi()) {
    startConfigMode();
  } else {
    Serial.println("\n⏳ Esperando datos GPS vía Bluetooth...");
    Serial.println("💡 Conecta desde la app Android para comenzar\n");
  }
}

void loop() {
  checkResetButton();

  if (configMode) {
    server.handleClient();
    return;
  }

  if (bluetoothConnected) {
    readBluetoothData();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi desconectado. Intentando reconectar...");
    if (!connectToWiFi()) {
      Serial.println("✗ Reconexión fallida. Iniciando modo configuración...");
      startConfigMode();
    }
    return;
  }

  if (millis() - lastBTCheck >= btCheckInterval) {
    static bool lastBTState = false;
    if (bluetoothConnected != lastBTState) {
      if (bluetoothConnected) {
        Serial.println("\n🔵 Estado: Bluetooth conectado, esperando GPS...");
      } else {
        Serial.println("\n⚪ Estado: Esperando conexión Bluetooth...");
      }
      lastBTState = bluetoothConnected;
    }
    lastBTCheck = millis();
  }

  if (millis() - lastSend >= sendInterval) {
    if (gpsValid && bluetoothConnected) {
      sendToAllServers(lastLat, lastLng);
    } else if (!bluetoothConnected) {
      Serial.println("⚠️  Esperando conexión Bluetooth...");
    } else if (!gpsValid) {
      Serial.println("⚠️  Esperando datos GPS del teléfono...");
    }
    lastSend = millis();
  }
}

void sendToAllServers(double lat, double lng) {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║          📤 ENVIANDO A SERVIDORES              ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.printf("📍 Ubicación: %.6f, %.6f\n", lat, lng);
  Serial.printf("🆔 Device ID: %s\n\n", deviceID.c_str());

  char plaintext[100];
  snprintf(plaintext, sizeof(plaintext), "%s,%.6f,%.6f", deviceID.c_str(), lat, lng);
  size_t plaintext_len = strlen(plaintext);

  int successCount = 0;
  int failCount = 0;

  for (int i = 0; i < NUM_SERVERS; i++) {
    if (!servers[i].enabled) continue;

    Serial.printf("→ Servidor %d (%s:%d)... ",
                  i + 1, servers[i].host, servers[i].port);

    bool success = sendEncryptedToServer(
      servers[i].host,
      servers[i].port,
      plaintext,
      plaintext_len);

    if (success) {
      Serial.println("✓ OK");
      sendSuccess[i]++;
      successCount++;
    } else {
      Serial.println("✗ FAIL");
      sendFails[i]++;
      failCount++;
    }

    delay(10);
  }

  Serial.println("\n────────────────────────────────────────────────");
  Serial.printf("✓ Exitosos: %d | ✗ Fallidos: %d\n", successCount, failCount);
  Serial.println("════════════════════════════════════════════════\n");
}

bool sendEncryptedToServer(const char* host, int port, const char* plaintext, size_t plaintext_len) {
  uint8_t iv[12];
  for (int i = 0; i < 12; i++) {
    iv[i] = random(0, 256);
  }

  uint8_t ciphertext[128];
  uint8_t tag[16];

  int ret = mbedtls_gcm_crypt_and_tag(
    &aes,
    MBEDTLS_GCM_ENCRYPT,
    plaintext_len,
    iv, 12,
    NULL, 0,
    (const uint8_t*)plaintext,
    ciphertext,
    16,
    tag);

  if (ret != 0) return false;

  uint8_t packet[200];
  size_t packet_len = 0;

  memcpy(packet, iv, 12);
  packet_len += 12;
  memcpy(packet + packet_len, ciphertext, plaintext_len);
  packet_len += plaintext_len;
  memcpy(packet + packet_len, tag, 16);
  packet_len += 16;

  if (udp.beginPacket(host, port) != 1) return false;
  udp.write(packet, packet_len);
  return (udp.endPacket() == 1);
}