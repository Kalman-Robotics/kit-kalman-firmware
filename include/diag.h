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

#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include "robot_config.h"

// Canal de estado y control del robot, por UDP 8891.
//
// Todo lo que hay aqui es pasivo: se calcula con datos que el firmware ya
// tiene y no anade trabajo al bucle principal. La instrumentacion pesada de
// las fases 2-4 --traza circular, historial en RTC RAM, watchdog de
// recepcion, estadisticas de memoria DMA-- se quito: no encontro la causa del
// fallo y su propio coste degradaba el bucle y provocaba reinicios que no
// habrian ocurrido de otro modo.
//
// Comandos entrantes (texto plano, respuesta JSON de una linea):
//   STATUS          reporte completo
//   PING            sonda
//   STOP_MOTORS     frena los motores
//   RESET_ODOM      odometria a cero
//   RESET_COUNTERS  contadores a cero
//   REBOOT <tok>    reinicio remoto; tok = ultimos 4 hex de la MAC
//   REBOOT_SAFE     idem, frenando motores antes
//
// Se responde SIEMPRE, tambien a comandos desconocidos: sin eso no se
// distingue "no llego" de "llego y no se entiende" de "el chip esta colgado".

// ---------------------------------------------------------------------------
// Puentes hacia el resto del firmware. diag.h no puede incluir motors.h ni
// session.h sin crear un ciclo de inclusiones, asi que se definen en main.cpp.
// ---------------------------------------------------------------------------
const char * diagSessionState();
void diagStopMotors();
void diagResetOdom();
String diagRebootToken();
void diagNoteCmd(const String & cmd);
float diagOdomX();
float diagOdomYaw();

extern char     g_last_cmd[24];
extern int64_t  g_last_cmd_us;
extern uint32_t g_cmd_count;
extern bool     g_ota_active;

// Motivo del ultimo reset y salud del bucle. Es lo unico que sobrevivio de la
// instrumentacion: sin rst no se sabe por que se reinicio el robot, y cuesta
// una sola llamada en setup().
extern esp_reset_reason_t g_rst_reason;
extern uint32_t g_loop_max_ms;
extern uint32_t g_loop_max_ms_since_report;

// Contadores del enlace. Se incrementan donde ocurre el evento; ninguno
// dispara acciones por su cuenta.
extern uint32_t g_ping_fails;
extern uint32_t g_agent_lost;
extern uint32_t g_wifi_drops;
extern uint32_t g_cmd_vel_rx;

extern WiFiUDP diag_udp;

static const uint16_t DIAG_PORT = 8891;
static const uint32_t DIAG_HEARTBEAT_MS = 30000;

inline const char * resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "ext_pin";
    case ESP_RST_SW:        return "sw";        // ESP.restart() del propio codigo
    case ESP_RST_PANIC:     return "panic";     // crash: excepcion o abort
    case ESP_RST_INT_WDT:   return "int_wdt";   // interrupcion bloqueada
    case ESP_RST_TASK_WDT:  return "task_wdt";  // loop bloqueado
    case ESP_RST_WDT:       return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "brownout";  // caida de tension
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

// Se llama al final de loop(). Mide cuanto tardo la iteracion anterior: un
// pico sostenido indica que algo esta bloqueando el bucle.
inline void diagLoopTick() {
  static int64_t loop_start_us = 0;
  if (loop_start_us != 0) {
    uint32_t loop_ms = (uint32_t)((esp_timer_get_time() - loop_start_us) / 1000);
    if (loop_ms > g_loop_max_ms)
      g_loop_max_ms = loop_ms;
    if (loop_ms > g_loop_max_ms_since_report)
      g_loop_max_ms_since_report = loop_ms;
  }
  loop_start_us = esp_timer_get_time();
}

inline void diagBegin() {
  Serial.print("[DIAG] motivo del ultimo reset: ");
  Serial.print(resetReasonName(g_rst_reason));
  Serial.print(", heap libre ");
  Serial.println(ESP.getFreeHeap());

  diag_udp.begin(DIAG_PORT);
  Serial.print("[DIAG] estado y control por UDP ");
  Serial.println(DIAG_PORT);
}

inline String diagReport() {
  uint32_t up_s = (uint32_t)(esp_timer_get_time() / 1000000);

  String s = "{";
  s += "\"ev\":\"diag\",";
  s += "\"fw\":\"";           s += CONFIG::FW_VERSION; s += "\",";
  s += "\"up_s\":";           s += String(up_s); s += ",";
  s += "\"rst\":\"";          s += resetReasonName(g_rst_reason); s += "\",";
  s += "\"state\":\"";        s += diagSessionState(); s += "\",";
  s += "\"heap\":";           s += String(ESP.getFreeHeap()); s += ",";
  s += "\"heap_min\":";       s += String(ESP.getMinFreeHeap()); s += ",";
  s += "\"loop_max_ms\":";    s += String(g_loop_max_ms); s += ",";
  s += "\"loop_max_now\":";   s += String(g_loop_max_ms_since_report); s += ",";
  s += "\"ping_fails\":";     s += String(g_ping_fails); s += ",";
  s += "\"agent_lost\":";     s += String(g_agent_lost); s += ",";
  s += "\"wifi_drops\":";     s += String(g_wifi_drops); s += ",";
  s += "\"cmd_vel_rx\":";     s += String(g_cmd_vel_rx); s += ",";
  s += "\"rssi\":";           s += String(WiFi.RSSI()); s += ",";
  s += "\"wifi_status\":";    s += String((int)WiFi.status()); s += ",";
  s += "\"odom_x\":";         s += String(diagOdomX(), 3); s += ",";
  s += "\"odom_yaw\":";       s += String(diagOdomYaw(), 3); s += ",";
  s += "\"last_cmd\":\"";     s += g_last_cmd; s += "\",";
  s += "\"cmd_count\":";      s += String(g_cmd_count); s += ",";
  s += "\"ota\":";            s += (g_ota_active ? "true" : "false"); s += ",";
  s += "\"ip\":\"";           s += WiFi.localIP().toString(); s += "\"";
  s += "}";
  return s;
}

// port = 0 usa DIAG_PORT; para responder a un comando hay que pasar el puerto
// de origen del datagrama. Se envia por el propio socket de escucha para que
// el origen sea DIAG_PORT: un socket nuevo tomaria un puerto efimero y quien
// filtre las respuestas por su origen las descartaria.
inline void diagSend(IPAddress to, bool broadcast, uint16_t port = 0) {
  if (WiFi.status() != WL_CONNECTED)
    return;

  IPAddress dest = broadcast ? WiFi.broadcastIP() : to;
  uint16_t dport = (port == 0 || broadcast) ? DIAG_PORT : port;

  if (diag_udp.beginPacket(dest, dport) == 1) {
    diag_udp.print(diagReport());
    diag_udp.endPacket();
  }
  g_loop_max_ms_since_report = 0;
}

inline void diagReply(IPAddress to, uint16_t port,
                      const char * ack, const char * result) {
  static uint32_t seq = 0;
  String s = "{\"v\":1,\"ack\":\"";
  s += ack;
  s += "\",\"result\":\"";
  s += result;
  s += "\",\"state\":\"";
  s += diagSessionState();
  s += "\",\"up_s\":";
  s += String((uint32_t)(esp_timer_get_time() / 1000000));
  s += ",\"seq\":";
  s += String(++seq);
  s += "}";

  if (diag_udp.beginPacket(to, port ? port : DIAG_PORT) == 1) {
    diag_udp.print(s);
    diag_udp.endPacket();
  }
  Serial.println(s);
}

inline void diagSpin() {
  static unsigned long last_hb_ms = 0;
  static unsigned long last_poll_ms = 0;

  // Sondear el socket en cada iteracion competia con la lectura serial del
  // LiDAR. Los comandos son esporadicos, asi que 50 ms de resolucion sobra.
  if (millis() - last_poll_ms < 50)
    return;
  last_poll_ms = millis();

  int len = diag_udp.parsePacket();
  if (len > 0) {
    // 128 bytes: los comandos con argumento (REBOOT <token>) no entran en 32
    char buf[128];
    int n = diag_udp.read(buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = 0;
      String cmd(buf);
      cmd.trim();
      cmd.toUpperCase();

      IPAddress from = diag_udp.remoteIP();
      uint16_t fport = diag_udp.remotePort();
      diagNoteCmd(cmd);

      String arg = "";
      int sp = cmd.indexOf(' ');
      if (sp > 0) {
        arg = cmd.substring(sp + 1);
        arg.trim();
        cmd = cmd.substring(0, sp);
      }

      if (cmd == "STATUS") {
        diagSend(from, false, fport);

      } else if (cmd == "PING") {
        diagReply(from, fport, "PING", "OK");

      } else if (cmd == "STOP_MOTORS") {
        diagStopMotors();
        diagReply(from, fport, "STOP_MOTORS", "OK");

      } else if (cmd == "RESET_ODOM") {
        diagResetOdom();
        diagReply(from, fport, "RESET_ODOM", "OK");

      } else if (cmd == "RESET_COUNTERS") {
        g_ping_fails = 0;
        g_agent_lost = 0;
        g_wifi_drops = 0;
        g_cmd_vel_rx = 0;
        g_loop_max_ms = 0;
        g_loop_max_ms_since_report = 0;
        diagReply(from, fport, "RESET_COUNTERS", "OK");

      } else if (cmd == "REBOOT" || cmd == "REBOOT_SAFE") {
        // Token: ultimos 4 hex de la MAC. No es seguridad (la red esta
        // aislada) sino proteccion contra un reinicio accidental por un bug
        // en la Pi durante una clase.
        if (arg != diagRebootToken()) {
          diagReply(from, fport, cmd.c_str(), "ERR bad_token");
        } else {
          bool safe = (cmd == "REBOOT_SAFE");
          diagReply(from, fport, cmd.c_str(), "OK");
          if (safe) {
            diagStopMotors();
            delay(100);
          }
          Serial.println("[DIAG] reinicio remoto solicitado");
          Serial.flush();
          delay(200);
          ESP.restart();
        }

      } else {
        diagReply(from, fport, cmd.c_str(), "UNKNOWN");
      }
    }
  }

  if (millis() - last_hb_ms >= DIAG_HEARTBEAT_MS) {
    last_hb_ms = millis();
    diagSend(IPAddress(), true);
  }
}
