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

// Prueba aislada del enlace micro-ROS.
//
// Unica pregunta: el ESP32 sigue recibiendo /cmd_vel indefinidamente, o el
// enlace se degrada pasado un tiempo?
//
// Deliberadamente NO hay lidar, motores, IMU, PID ni telemetria a 20 Hz. Solo
// un suscriptor de cmd_vel y un contador. Si esto falla, la causa esta en
// micro-ROS, en el agente o en la red; no en el codigo de la aplicacion.
//
// Se compara contra el firmware completo (env esp32-s3-diag):
//   aislado falla + completo falla  -> el problema no es el codigo del robot
//   aislado OK    + completo falla  -> el problema es de la aplicacion
//   ambos OK                        -> el fix del desborde resolvio todo
//
// NUNCA reinicia: un reinicio borraria los contadores.
//
// Reporte por UDP broadcast al puerto 8892, cada 30 s.
// Comandos entrantes en el mismo puerto: STATUS, RESET.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>

#include <micro_ros_kaia.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

// --------------------------------------------------------------------------
// Configuracion; se lee de /config.yaml para no duplicar credenciales
// --------------------------------------------------------------------------
static String ssid = "kalman-robot";
static String pass = "kalman2024";
static String agent_ip = "192.168.4.1";
static uint16_t agent_port = 8888;
static uint8_t domain_id = 20;

static const uint16_t REPORT_PORT = 8892;
static const uint32_t HEARTBEAT_MS = 30000;
static const uint32_t PING_PERIOD_MS = 5000;
static const uint8_t LED_GPIO = 11;

// La marca donde desbordaba el contador de 32 bits: 2^32 us = 71.58 min
static const uint32_t WRAP_MARK_S = 4295;

// --------------------------------------------------------------------------
static rcl_subscription_t twist_sub;
static geometry_msgs__msg__Twist twist_msg;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rclc_executor_t executor;
static rcl_node_t node;

static WiFiUDP udp;
static bool udp_ready = false;

struct Stats {
  uint32_t cmd_vel_rx = 0;        // comandos recibidos en total
  int64_t  last_cmd_vel_us = -1;  // marca de tiempo del ultimo
  float    last_linear_x = 0;
  uint32_t ping_fails = 0;
  uint32_t ping_ok = 0;
  uint32_t wifi_drops = 0;
  uint32_t gap_max_s = 0;         // mayor silencio entre comandos
};
static Stats stats;

// --------------------------------------------------------------------------

static void twist_cb(const void * msgin) {
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msgin;

  int64_t now_us = esp_timer_get_time();
  if (stats.last_cmd_vel_us > 0) {
    uint32_t gap_s = (uint32_t)((now_us - stats.last_cmd_vel_us) / 1000000);
    if (gap_s > stats.gap_max_s)
      stats.gap_max_s = gap_s;
  }

  stats.cmd_vel_rx++;
  stats.last_cmd_vel_us = now_us;
  stats.last_linear_x = msg->linear.x;

  Serial.print("[cmd_vel #");
  Serial.print(stats.cmd_vel_rx);
  Serial.print("] linear.x=");
  Serial.print(msg->linear.x, 3);
  Serial.print(" angular.z=");
  Serial.print(msg->angular.z, 3);
  Serial.print("  up=");
  Serial.print((uint32_t)(now_us / 1000000));
  Serial.println("s");
}

static void loadConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS no disponible, usando valores por defecto");
    return;
  }
  File f = SPIFFS.open("/config.yaml");
  if (!f || f.isDirectory()) {
    Serial.println("/config.yaml no encontrado, usando valores por defecto");
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
    if (val.length() == 0)
      continue;
    if (key == "ssid")            ssid = val;
    else if (key == "password")   pass = val;
    else if (key == "ip")         agent_ip = val;
    else if (key == "port")       agent_port = (uint16_t)val.toInt();
    else if (key == "domain_id")  domain_id = (uint8_t)val.toInt();
  }
  f.close();
}

static String buildReport(const char * event) {
  uint32_t up_s = (uint32_t)(esp_timer_get_time() / 1000000);
  int64_t since_cmd = stats.last_cmd_vel_us > 0 ?
    (esp_timer_get_time() - stats.last_cmd_vel_us) / 1000000 : -1;

  String s = "{";
  s += "\"ev\":\"";  s += event; s += "\",";
  s += "\"up_s\":";           s += String(up_s); s += ",";
  s += "\"past_wrap\":";      s += (up_s > WRAP_MARK_S ? "true" : "false"); s += ",";
  s += "\"cmd_vel_rx\":";     s += String(stats.cmd_vel_rx); s += ",";
  s += "\"since_cmd_s\":";    s += String((long)since_cmd); s += ",";
  s += "\"gap_max_s\":";      s += String(stats.gap_max_s); s += ",";
  s += "\"last_linear_x\":";  s += String(stats.last_linear_x, 3); s += ",";
  s += "\"ping_ok\":";        s += String(stats.ping_ok); s += ",";
  s += "\"ping_fails\":";     s += String(stats.ping_fails); s += ",";
  s += "\"wifi_drops\":";     s += String(stats.wifi_drops); s += ",";
  s += "\"rssi\":";           s += String(WiFi.RSSI()); s += ",";
  s += "\"ip\":\"";           s += WiFi.localIP().toString(); s += "\"";
  s += "}";
  return s;
}

static void sendReport(const char * event, IPAddress to, bool broadcast) {
  if (WiFi.status() != WL_CONNECTED)
    return;

  IPAddress dest = broadcast ? WiFi.broadcastIP() : to;
  String payload = buildReport(event);

  // Socket de envio dedicado: reusar el de escucha tras un parsePacket() deja
  // el destino pegado al ultimo remitente y da ENOTCONN en broadcast
  WiFiUDP tx;
  if (tx.beginPacket(dest, REPORT_PORT) == 1) {
    tx.print(payload);
    tx.endPacket();
  }
  tx.stop();
  Serial.println(payload);
}

static void spinReport() {
  static unsigned long last_hb_ms = 0;

  if (udp_ready) {
    int len = udp.parsePacket();
    if (len > 0) {
      char buf[32];
      int n = udp.read(buf, sizeof(buf) - 1);
      if (n > 0) {
        buf[n] = '\0';
        String cmd(buf);
        cmd.trim();
        cmd.toUpperCase();
        if (cmd == "STATUS") {
          sendReport("status", udp.remoteIP(), false);
        } else if (cmd == "RESET") {
          stats = Stats();
          Serial.println("contadores reiniciados");
          sendReport("reset", udp.remoteIP(), false);
        }
      }
    }
  }

  if (millis() - last_hb_ms >= HEARTBEAT_MS) {
    last_hb_ms = millis();
    sendReport("hb", IPAddress(), true);
  }
}

// Vigila el agente sin reiniciar nunca: solo cuenta
static void spinPing() {
  static unsigned long last_ping_ms = 0;
  if (millis() - last_ping_ms < PING_PERIOD_MS)
    return;
  last_ping_ms = millis();

  if (WiFi.status() != WL_CONNECTED)
    return;

  if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) {
    stats.ping_ok++;
    digitalWrite(LED_GPIO, HIGH);
  } else {
    stats.ping_fails++;
    Serial.print("ping fallido (");
    Serial.print(stats.ping_fails);
    Serial.println(" acumulados) — NO se reinicia");
    digitalWrite(LED_GPIO, LOW);
  }
}

static void connectWiFi() {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), pass.c_str());

  Serial.print("Conectando a ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
  }
  Serial.print(" OK, IP ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW);

  Serial.println();
  Serial.println("=== Prueba aislada de micro-ROS + cmd_vel ===");
  Serial.println("Sin lidar, motores, IMU ni telemetria. Nunca reinicia.");

  loadConfig();
  Serial.print("Agente: ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.print(agent_port);
  Serial.print("  domain_id ");
  Serial.println(domain_id);

  connectWiFi();
  udp_ready = udp.begin(REPORT_PORT);
  Serial.print("Reporte UDP puerto ");
  Serial.println(REPORT_PORT);

  set_microros_wifi_transports(agent_ip.c_str(), agent_port);
  delay(500);

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);

  Serial.print("Conectando al agente ");
  while (rclc_support_init_with_options(&support, 0, NULL,
                                        &init_options, &allocator)
         != RCL_RET_OK) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(" OK");

  rclc_node_init_default(&node, "microros_cmdvel_test", "", &support);

  // Reliable por defecto, igual que el firmware normal (compatible con Nav2)
  rclc_subscription_init_default(
    &twist_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg,
                                 &twist_cb, ON_NEW_DATA);

  Serial.println("Listo. Esperando /cmd_vel ...");
  digitalWrite(LED_GPIO, HIGH);
  sendReport("boot", IPAddress(), true);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    stats.wifi_drops++;
    Serial.println("WiFi caido, reconectando ...");
    connectWiFi();
    udp.stop();
    udp_ready = udp.begin(REPORT_PORT);
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  spinPing();
  spinReport();
}
