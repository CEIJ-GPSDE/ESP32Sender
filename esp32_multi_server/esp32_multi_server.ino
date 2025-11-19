#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <BluetoothSerial.h>
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
  { "chidrobo.ddns.net.com", 5051, true },
  { "null", 5051, false }
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

// Embedded HTML configuration page
const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 GPS - Configuración WiFi</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 20px;
    }
    .container {
      background: white;
      border-radius: 20px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
      padding: 40px;
      max-width: 450px;
      width: 100%;
    }
    h1 {
      color: #667eea;
      font-size: 28px;
      margin-bottom: 10px;
      text-align: center;
    }
    .subtitle {
      color: #666;
      text-align: center;
      margin-bottom: 30px;
      font-size: 14px;
    }
    .icon {
      text-align: center;
      font-size: 60px;
      margin-bottom: 20px;
    }
    .current-network {
      background: #e3f2fd;
      border-left: 4px solid #2196f3;
      padding: 12px 15px;
      margin-bottom: 20px;
      border-radius: 5px;
      font-size: 13px;
    }
    .current-network strong {
      color: #1976d2;
      display: block;
      margin-bottom: 5px;
    }
    .current-network code {
      background: white;
      padding: 2px 6px;
      border-radius: 3px;
      font-size: 12px;
    }
    label {
      display: block;
      margin-bottom: 8px;
      color: #333;
      font-weight: 600;
      font-size: 14px;
    }
    input[type="text"],
    input[type="password"] {
      width: 100%;
      padding: 12px 15px;
      border: 2px solid #e0e0e0;
      border-radius: 10px;
      font-size: 16px;
      transition: all 0.3s;
      margin-bottom: 20px;
    }
    input[type="text"]:focus,
    input[type="password"]:focus {
      outline: none;
      border-color: #667eea;
      box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
    }
    button {
      width: 100%;
      padding: 14px;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
      border: none;
      border-radius: 10px;
      font-size: 16px;
      font-weight: 600;
      cursor: pointer;
      transition: transform 0.2s;
    }
    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
    }
    button:active {
      transform: translateY(0);
    }
    .info-box {
      background: #f5f7ff;
      border-left: 4px solid #667eea;
      padding: 15px;
      margin-top: 20px;
      border-radius: 5px;
      font-size: 13px;
      color: #555;
    }
    .info-box strong {
      color: #667eea;
    }
    #status {
      margin-top: 20px;
      padding: 15px;
      border-radius: 10px;
      text-align: center;
      font-weight: 600;
      display: none;
    }
    .success {
      background: #d4edda;
      color: #155724;
      border: 2px solid #c3e6cb;
    }
    .error {
      background: #f8d7da;
      color: #721c24;
      border: 2px solid #f5c6cb;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="icon">📡🔵</div>
    <h1>ESP32 BT GPS Tracker</h1>
    <p class="subtitle">Configuración de Red WiFi</p>
    
    <div class="current-network">
      <strong>🔌 Red actual:</strong>
      <code>CURRENT_SSID_PLACEHOLDER</code>
    </div>
    
    <form id="wifiForm" onsubmit="submitForm(event)">
      <label for="ssid">🌐 Nueva Red (SSID)</label>
      <input type="text" id="ssid" name="ssid" placeholder="Mi_Red_WiFi" required>
      
      <label for="password">🔐 Contraseña</label>
      <input type="password" id="password" name="password" placeholder="••••••••" required>
      
      <button type="submit">💾 Guardar y Conectar</button>
    </form>
    
    <div id="status"></div>
    
    <div class="info-box">
      <strong>ℹ️ Información:</strong><br>
      • El ESP32 recibe GPS vía Bluetooth<br>
      • Conecta desde la app Android<br>
      • Mantén presionado BOOT 3 seg para volver aquí
    </div>
  </div>

  <script>
    function submitForm(e) {
      e.preventDefault();
      const ssid = document.getElementById('ssid').value;
      const password = document.getElementById('password').value;
      const status = document.getElementById('status');
      
      status.style.display = 'block';
      status.className = '';
      status.innerHTML = '⏳ Guardando configuración...';
      
      fetch('/save', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: `ssid=${encodeURIComponent(ssid)}&password=${encodeURIComponent(password)}`
      })
      .then(response => response.text())
      .then(data => {
        status.className = 'success';
        status.innerHTML = '✅ Guardado! Reiniciando ESP32...<br>Espera 30 segundos';
        document.getElementById('wifiForm').style.display = 'none';
      })
      .catch(error => {
        status.className = 'error';
        status.innerHTML = '❌ Error al guardar. Intenta nuevamente.';
      });
    }
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
    Serial.println("\n=== Serving configuration page ===");
    
    // Get current SSID from preferences
    preferences.begin("wifi", true);
    String currentSSID = preferences.getString("ssid", "Ninguna");
    preferences.end();
    
    // Load HTML from PROGMEM and replace placeholder
    String html = FPSTR(CONFIG_HTML);
    html.replace("CURRENT_SSID_PLACEHOLDER", currentSSID);
    
    Serial.println("✓ Sending HTML to browser...");
    server.send(200, "text/html", html);
    Serial.println("✓ HTML sent successfully\n");
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