// Copyright 2023-2025 kalman.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Prueba de estabilidad del WiFi.
//
// Unica pregunta que responde: el ESP32 mantiene la asociacion con el AP de la
// Raspberry durante horas, o la pierde cada tanto?
//
// No hay lidar, motores ni micro-ROS a proposito: si el WiFi falla aca, el
// problema es de red y no hay otra variable que pueda explicarlo.
//
// NUNCA reinicia. Un reinicio borraria los contadores, que son justamente el
// resultado de la prueba.
//
// Salida por UDP broadcast (el robot corre con fuente externa, sin serial):
//   puerto 8890, JSON, un datagrama por heartbeat
//
// Comandos entrantes en el mismo puerto:
//   LOUD    heartbeat cada 30 s (default): mide con keep-alive
//   QUIET   sin heartbeat, solo acumula: mide sin trafico de salida
//   STATUS  responde el estado actual una vez, sin cambiar de modo
//   RESET   pone los contadores a cero

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>

// ---------------------------------------------------------------------------
// Configuracion. Se lee de /config.yaml para no duplicar credenciales; si no
// se puede leer, se usan estos valores.
// ---------------------------------------------------------------------------
static String ssid = "kalman-robot";
static String pass = "kalman2024";

static const uint16_t REPORT_PORT = 8890;
static const char * BROADCAST_ADDR = "192.168.4.255";

static const uint32_t HEARTBEAT_MS = 30000;
static const uint32_t POLL_MS = 250;
static const uint32_t RECONNECT_RETRY_MS = 5000;
static const uint8_t LED_GPIO = 11;

// ---------------------------------------------------------------------------
// Estadisticas acumuladas. Sobreviven todo el dia porque nunca se reinicia.
// ---------------------------------------------------------------------------
struct Stats {
  uint32_t drops = 0;             // veces que se perdio la asociacion
  uint32_t total_down_s = 0;      // segundos acumulados sin red
  uint32_t max_down_s = 0;        // la caida mas larga
  uint32_t last_down_s = 0;       // duracion de la ultima caida
  int32_t  rssi_min = 0;          // peor senal vista
  int32_t  rssi_now = 0;
  uint32_t reconnect_attempts = 0;
  uint32_t heartbeats_sent = 0;
  uint32_t ip_changes = 0;        // el DHCP dio una IP distinta
};

static Stats stats;
static WiFiUDP udp;
static bool udp_ready = false;
static bool quiet_mode = false;
static bool wifi_up_prev = false;
static unsigned long down_since_ms = 0;
static unsigned long last_heartbeat_ms = 0;
static unsigned long last_retry_ms = 0;
static unsigned long last_poll_ms = 0;
static String last_ip = "";

// ---------------------------------------------------------------------------

static void logLine(const String & s) {
  Serial.println(s); // sirve si alguien conecta el USB durante la prueba
}

// Lee ssid/password de /config.yaml. Parseo minimo: solo busca las dos claves
// para no arrastrar todo el CONFIG del firmware normal.
static void loadCredentials() {
  if (!SPIFFS.begin(true)) {
    logLine("SPIFFS no disponible, usando credenciales por defecto");
    return;
  }
  File f = SPIFFS.open("/config.yaml");
  if (!f || f.isDirectory()) {
    logLine("/config.yaml no encontrado, usando credenciales por defecto");
    return;
  }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("#"))
      continue;
    int colon = line.indexOf(':');
    if (colon < 0)
      continue;
    String key = line.substring(0, colon);
    String val = line.substring(colon + 1);
    key.trim();
    val.trim();
    if (key == "ssid" && val.length() > 0)
      ssid = val;
    else if (key == "password" && val.length() > 0)
      pass = val;
  }
  f.close();
}

static String buildReport(const char * event) {
  uint32_t up_s = millis() / 1000;
  // Uptime de red en centesimas, para no arrastrar float en el calculo
  uint32_t pct_x100 = up_s > 0 ?
    (uint32_t)(((uint64_t)(up_s - stats.total_down_s) * 10000) / up_s) : 10000;

  String s = "{";
  s += "\"ev\":\""; s += event; s += "\",";
  s += "\"up_s\":";        s += String(up_s); s += ",";
  s += "\"drops\":";       s += String(stats.drops); s += ",";
  s += "\"down_s\":";      s += String(stats.total_down_s); s += ",";
  s += "\"max_down_s\":";  s += String(stats.max_down_s); s += ",";
  s += "\"last_down_s\":"; s += String(stats.last_down_s); s += ",";
  s += "\"uptime_pct\":";  s += String(pct_x100 / 100.0, 2); s += ",";
  s += "\"rssi\":";        s += String(stats.rssi_now); s += ",";
  s += "\"rssi_min\":";    s += String(stats.rssi_min); s += ",";
  s += "\"retries\":";     s += String(stats.reconnect_attempts); s += ",";
  s += "\"ip_changes\":";  s += String(stats.ip_changes); s += ",";
  s += "\"hb\":";          s += String(stats.heartbeats_sent); s += ",";
  s += "\"mode\":\"";      s += (quiet_mode ? "QUIET" : "LOUD"); s += "\",";
  s += "\"ip\":\"";        s += last_ip; s += "\"";
  s += "}";
  return s;
}

static void sendReport(const char * event, IPAddress to, bool broadcast) {
  if (!udp_ready || WiFi.status() != WL_CONNECTED)
    return;

  String payload = buildReport(event);
  IPAddress dest = to;
  if (broadcast) {
    // WiFi.broadcastIP() calcula la broadcast de la subred asociada. Calcularla
    // a mano fallaba: subnetMask() puede devolver ceros y el resultado terminaba
    // siendo 255.255.255.255, que el stack rechaza con ENOTCONN.
    dest = WiFi.broadcastIP();
  }

  // Socket dedicado para enviar: reusar el de escucha tras un parsePacket()
  // deja el destino pegado al ultimo remitente y da ENOTCONN en broadcast
  WiFiUDP tx;
  if (tx.beginPacket(dest, REPORT_PORT) == 1) {
    tx.print(payload);
    if (tx.endPacket() != 1) {
      Serial.print("envio fallido a ");
      Serial.println(dest);
    }
  }
  tx.stop();
  logLine(payload);
}

// Comandos entrantes: permiten cambiar de modo sin reflashear
static void pollCommands() {
  if (!udp_ready)
    return;
  int len = udp.parsePacket();
  if (len <= 0)
    return;

  char buf[64];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0)
    return;
  buf[n] = '\0';

  String cmd(buf);
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.startsWith("{")) // es el eco de nuestro propio broadcast
    return;

  IPAddress from = udp.remoteIP();

  if (cmd == "QUIET") {
    quiet_mode = true;
    logLine("modo QUIET: sin heartbeat");
    sendReport("mode_quiet", from, false);
  } else if (cmd == "LOUD") {
    quiet_mode = false;
    logLine("modo LOUD: heartbeat cada 30 s");
    sendReport("mode_loud", from, false);
  } else if (cmd == "STATUS") {
    sendReport("status", from, false);
  } else if (cmd == "RESET") {
    stats = Stats();
    stats.rssi_min = WiFi.RSSI();
    logLine("contadores reiniciados");
    sendReport("reset", from, false);
  }
}

static void connectWiFi() {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), pass.c_str());
  stats.reconnect_attempts++;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW);

  Serial.println();
  Serial.println("=== Prueba de estabilidad de WiFi ===");
  Serial.println("Sin lidar, motores ni micro-ROS. Nunca reinicia.");

  loadCredentials();
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Reporte UDP: ");
  Serial.print(BROADCAST_ADDR);
  Serial.print(":");
  Serial.println(REPORT_PORT);

  connectWiFi();
}

void loop() {
  unsigned long now = millis();

  if (now - last_poll_ms < POLL_MS)
    return;
  last_poll_ms = now;

  bool wifi_up = WiFi.status() == WL_CONNECTED;

  // --- transiciones ---
  if (wifi_up && !wifi_up_prev) {
    uint32_t down_s = down_since_ms > 0 ? (now - down_since_ms) / 1000 : 0;
    stats.last_down_s = down_s;
    stats.total_down_s += down_s;
    if (down_s > stats.max_down_s)
      stats.max_down_s = down_s;

    String ip = WiFi.localIP().toString();
    if (last_ip.length() > 0 && ip != last_ip)
      stats.ip_changes++;
    last_ip = ip;

    // El socket queda atado a la IP anterior tras una reconexion
    if (udp_ready)
      udp.stop();
    udp_ready = udp.begin(REPORT_PORT);

    stats.rssi_now = WiFi.RSSI();
    if (stats.rssi_min == 0 || stats.rssi_now < stats.rssi_min)
      stats.rssi_min = stats.rssi_now;

    logLine("WiFi UP tras " + String(down_s) + "s, IP " + ip);
    sendReport("up", IPAddress(), true);
    down_since_ms = 0;

  } else if (!wifi_up && wifi_up_prev) {
    stats.drops++;
    down_since_ms = now;
    last_retry_ms = now;
    logLine("WiFi DOWN (caida #" + String(stats.drops) + ")");
  }
  wifi_up_prev = wifi_up;

  // --- estado conectado ---
  if (wifi_up) {
    digitalWrite(LED_GPIO, HIGH); // fijo = red OK

    int32_t rssi = WiFi.RSSI();
    if (rssi != 0) {  // 0 = lectura no valida, conservar la anterior
      stats.rssi_now = rssi;
      if (stats.rssi_min == 0 || rssi < stats.rssi_min)
        stats.rssi_min = rssi;
    }

    pollCommands();

    if (!quiet_mode && now - last_heartbeat_ms >= HEARTBEAT_MS) {
      last_heartbeat_ms = now;
      stats.heartbeats_sent++;
      sendReport("hb", IPAddress(), true);
    }
    return;
  }

  // --- estado caido: parpadeo rapido y reintento ---
  digitalWrite(LED_GPIO, (now / 200) % 2 ? HIGH : LOW);

  if (now - last_retry_ms >= RECONNECT_RETRY_MS) {
    last_retry_ms = now;
    logLine("reconectando (intento " + String(stats.reconnect_attempts + 1) + ")");
    connectWiFi();
  }
}
