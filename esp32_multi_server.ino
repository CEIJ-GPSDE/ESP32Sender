#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include "mbedtls/aes.h"
#include "mbedtls/md.h"
#include "mbedtls/gcm.h"
#include "secret_config.h"



// ════════════════════════════════════════════════════════════════
// CONFIGURACIÓN WIFI
// ════════════════════════════════════════════════════════════════
const char* ssid = "oppo";
const char* password = "euribe2006";


// ════════════════════════════════════════════════════════════════
// CONFIGURACIÓN DE MÚLTIPLES SERVIDORES
// ════════════════════════════════════════════════════════════════
#define NUM_SERVERS 4

struct ServerConfig {
  const char* host;
  int port;
  bool enabled;  // Permite habilitar/deshabilitar servidores
};

// Configurar tus 4 servidores aquí:
ServerConfig servers[NUM_SERVERS] = {
  {"uesteban.ddnsking.com", 5051, true},   // Servidor 1
  {"jesucaracu.ddns.net", 5051, true},           // Servidor 2 (ejemplo: servidor local)
  {"chidrobo.ddnsking.com", 5051, true},    // Servidor 3 (ejemplo: backup)
  {"ivbarrios.ddns.net", 5051, true}     // Servidor 4 (deshabilitado por defecto)
};

const char* deviceID = "ESP32_001";


// ════════════════════════════════════════════════════════════════
// CONFIGURACIÓN GPS
// ════════════════════════════════════════════════════════════════
#define GPS_RX_PIN 16  // RX2 del ESP32 -> TX del GPS
#define GPS_TX_PIN 17  // TX2 del ESP32 -> RX del GPS
#define GPS_BAUD 9600  // Velocidad estándar del NEO-6M/7M


// ════════════════════════════════════════════════════════════════
// CLIENTES GLOBALES
// ════════════════════════════════════════════════════════════════
WiFiUDP udp;                      // Cliente UDP (se reutiliza para todos los servidores)
mbedtls_gcm_context aes;          // Cliente de encriptación
TinyGPSPlus gps;                  // Parser GPS
HardwareSerial gpsSerial(2);      // UART2 del ESP32


// ════════════════════════════════════════════════════════════════
// VARIABLES DE TEMPORIZACIÓN
// ════════════════════════════════════════════════════════════════
unsigned long lastSend = 0;
const unsigned long sendInterval = 10000;  // 10 segundos
unsigned long lastGPSCheck = 0;
const unsigned long gpsCheckInterval = 1000;  // Verificar GPS cada 1 segundo


// ════════════════════════════════════════════════════════════════
// VARIABLES DE ESTADO
// ════════════════════════════════════════════════════════════════
bool gpsValid = false;
double lastLat = 0.0;
double lastLng = 0.0;
int satellitesCount = 0;

// Estadísticas de envío por servidor
int sendSuccess[NUM_SERVERS] = {0, 0, 0, 0};
int sendFails[NUM_SERVERS] = {0, 0, 0, 0};


// ════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║   ESP32 Multi-Server Encrypted GPS Tracker    ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  
  // ────────────────────────────────────────────────
  // Inicializar GPS Serial (UART2)
  // ────────────────────────────────────────────────
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("✓ GPS Serial inicializado en pines RX:%d TX:%d @ %d baud\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
  
  // ────────────────────────────────────────────────
  // Inicializar AES-GCM
  // ────────────────────────────────────────────────
  mbedtls_gcm_init(&aes);
  int ret = mbedtls_gcm_setkey(&aes, MBEDTLS_CIPHER_ID_AES, AES_KEY, 128);
  if (ret != 0) {
    Serial.printf("✗ Error inicializando AES: -0x%04x\n", -ret);
    while(1) delay(1000);
  }
  Serial.println("✓ AES-128-GCM inicializado");
  
  // ────────────────────────────────────────────────
  // Mostrar configuración de servidores
  // ────────────────────────────────────────────────
  Serial.println("\n📡 Servidores configurados:");
  for (int i = 0; i < NUM_SERVERS; i++) {
    Serial.printf("   [%d] %s:%d - %s\n", 
                  i + 1,
                  servers[i].host, 
                  servers[i].port,
                  servers[i].enabled ? "✓ Habilitado" : "✗ Deshabilitado");
  }
  
  // ────────────────────────────────────────────────
  // Conectar WiFi
  // ────────────────────────────────────────────────
  Serial.print("\nConectando a WiFi");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ Error: No se pudo conectar a WiFi");
  }
  
  Serial.println("\n⏳ Esperando señal GPS...");
  Serial.println("   (Esto puede tardar 30-60 segundos en exterior)");
}


// ════════════════════════════════════════════════════════════════
// LOOP PRINCIPAL
// ════════════════════════════════════════════════════════════════
void loop() {
  // Leer datos del GPS continuamente
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  
  // Verificar WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reconectando...");
    WiFi.reconnect();
    delay(5000);
    return;
  }
  
  // Mostrar estado del GPS periódicamente
  if (millis() - lastGPSCheck >= gpsCheckInterval) {
    checkGPSStatus();
    lastGPSCheck = millis();
  }
  
  // Enviar ubicación cada 10 segundos a TODOS los servidores
  if (millis() - lastSend >= sendInterval) {
    if (gpsValid && gps.location.isValid()) {
      double lat = gps.location.lat();
      double lng = gps.location.lng();
      
      sendToAllServers(lat, lng);
      
      lastLat = lat;
      lastLng = lng;
      lastSend = millis();
    } else {
      Serial.println("⚠️  No se puede enviar: GPS sin señal válida");
      Serial.println("   Asegúrate de estar en exterior con vista al cielo");
      lastSend = millis();
    }
  }
}


// ════════════════════════════════════════════════════════════════
// FUNCIÓN: Enviar a todos los servidores
// ════════════════════════════════════════════════════════════════
void sendToAllServers(float lat, float lng) {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║          📤 ENVIANDO A SERVIDORES              ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.printf("📍 Ubicación: %.6f, %.6f\n", lat, lng);
  Serial.printf("🛰️  Satélites: %d | HDOP: %.2f\n\n", satellitesCount, gps.hdop.hdop());
  
  // Crear mensaje una sola vez (se encriptará con IV diferente para cada servidor)
  char plaintext[100];
  snprintf(plaintext, sizeof(plaintext), "%s,%.6f,%.6f", deviceID, lat, lng);
  size_t plaintext_len = strlen(plaintext);
  
  int successCount = 0;
  int failCount = 0;
  
  // Enviar a cada servidor habilitado
  for (int i = 0; i < NUM_SERVERS; i++) {
    if (!servers[i].enabled) {
      Serial.printf("⊘ Servidor %d: Deshabilitado\n", i + 1);
      continue;
    }
    
    Serial.printf("→ Servidor %d (%s:%d)... ", 
                  i + 1, servers[i].host, servers[i].port);
    
    bool success = sendEncryptedToServer(
      servers[i].host, 
      servers[i].port, 
      plaintext, 
      plaintext_len
    );
    
    if (success) {
      Serial.println("✓ OK");
      sendSuccess[i]++;
      successCount++;
    } else {
      Serial.println("✗ FAIL");
      sendFails[i]++;
      failCount++;
    }
    
    // Pequeña pausa entre envíos (opcional, evita congestión)
    delay(10);
  }
  
  // Resumen del envío
  Serial.println("\n────────────────────────────────────────────────");
  Serial.printf("✓ Exitosos: %d | ✗ Fallidos: %d\n", successCount, failCount);
  Serial.println("════════════════════════════════════════════════\n");
}


// ════════════════════════════════════════════════════════════════
// FUNCIÓN: Enviar a un servidor específico
// ════════════════════════════════════════════════════════════════
bool sendEncryptedToServer(const char* host, int port, const char* plaintext, size_t plaintext_len) {
  // ──────────────────────────────────────────────────
  // 1. Generar IV aleatorio (12 bytes para GCM)
  // ──────────────────────────────────────────────────
  uint8_t iv[12];
  for (int i = 0; i < 12; i++) {
    iv[i] = random(0, 256);
  }
  
  // ──────────────────────────────────────────────────
  // 2. Buffer para texto cifrado + tag
  // ──────────────────────────────────────────────────
  uint8_t ciphertext[128];
  uint8_t tag[16];
  
  // ──────────────────────────────────────────────────
  // 3. Encriptar con AES-GCM
  // ──────────────────────────────────────────────────
  int ret = mbedtls_gcm_crypt_and_tag(
    &aes,
    MBEDTLS_GCM_ENCRYPT,
    plaintext_len,
    iv, 12,
    NULL, 0,
    (const uint8_t*)plaintext,
    ciphertext,
    16,
    tag
  );
  
  if (ret != 0) {
    Serial.printf("Error encriptando: -0x%04x\n", -ret);
    return false;
  }
  
  // ──────────────────────────────────────────────────
  // 4. Construir paquete: [IV][Ciphertext][Tag]
  // ──────────────────────────────────────────────────
  uint8_t packet[200];
  size_t packet_len = 0;
  
  memcpy(packet, iv, 12);
  packet_len += 12;
  
  memcpy(packet + packet_len, ciphertext, plaintext_len);
  packet_len += plaintext_len;
  
  memcpy(packet + packet_len, tag, 16);
  packet_len += 16;
  
  // ──────────────────────────────────────────────────
  // 5. Enviar por UDP
  // ──────────────────────────────────────────────────
  int result = udp.beginPacket(host, port);
  if (result != 1) {
    return false;
  }
  
  udp.write(packet, packet_len);
  int sent = udp.endPacket();
  
  return (sent == 1);
}


// ════════════════════════════════════════════════════════════════
// FUNCIÓN: Verificar estado del GPS
// ════════════════════════════════════════════════════════════════
void checkGPSStatus() {
  if (gps.location.isValid()) {
    gpsValid = true;
    satellitesCount = gps.satellites.value();
    
    // Mostrar info solo cada 10 segundos
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint >= 10000) {
      Serial.println("\n📡 Estado GPS:");
      Serial.printf("   Lat: %.6f°\n", gps.location.lat());
      Serial.printf("   Lng: %.6f°\n", gps.location.lng());
      Serial.printf("   Altitud: %.1f m\n", gps.altitude.meters());
      Serial.printf("   Satélites: %d\n", satellitesCount);
      Serial.printf("   HDOP: %.2f\n", gps.hdop.hdop());
      Serial.printf("   Velocidad: %.1f km/h\n", gps.speed.kmph());
      
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.printf("   Fecha/Hora: %02d/%02d/%04d %02d:%02d:%02d UTC\n",
                     gps.date.day(), gps.date.month(), gps.date.year(),
                     gps.time.hour(), gps.time.minute(), gps.time.second());
      }
      
      // Mostrar estadísticas de envío
      Serial.println("\n📊 Estadísticas de envío:");
      for (int i = 0; i < NUM_SERVERS; i++) {
        if (servers[i].enabled) {
          int total = sendSuccess[i] + sendFails[i];
          float successRate = (total > 0) ? (sendSuccess[i] * 100.0 / total) : 0;
          Serial.printf("   Servidor %d: %d✓ / %d✗ (%.1f%% éxito)\n", 
                       i + 1, sendSuccess[i], sendFails[i], successRate);
        }
      }
      
      lastStatusPrint = millis();
    }
  } else {
    gpsValid = false;
    
    // Mostrar diagnóstico si no hay señal
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning >= 5000) {
      Serial.print("⏳ Buscando señal GPS");
      
      if (gps.charsProcessed() < 10) {
        Serial.println(" - ⚠️  No se reciben datos del módulo GPS");
        Serial.println("   Verifica las conexiones:");
        Serial.printf("   - GPS TX -> ESP32 RX (Pin %d)\n", GPS_RX_PIN);
        Serial.printf("   - GPS RX -> ESP32 TX (Pin %d)\n", GPS_TX_PIN);
        Serial.println("   - GPS VCC -> 3.3V o 5V");
        Serial.println("   - GPS GND -> GND");
      } else {
        Serial.printf(" - Caracteres procesados: %d\n", gps.charsProcessed());
        Serial.printf("   Sentencias válidas: %d\n", gps.sentencesWithFix());
        Serial.println("   Esperando fix GPS (mueve el módulo al exterior)");
      }
      
      lastWarning = millis();
    }
  }
}


// ════════════════════════════════════════════════════════════════
// FUNCIÓN: Imprimir bytes en hexadecimal
// ════════════════════════════════════════════════════════════════
void printHex(uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02x", data[i]);
  }
  Serial.println();
}
