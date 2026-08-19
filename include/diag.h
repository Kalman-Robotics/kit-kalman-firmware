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

// Modo diagnostico para pruebas de larga duracion.
//
// Con DIAG_NO_RESTART el firmware NO reinicia ante ningun fallo: cuenta el
// evento, se recupera y sigue. Un reinicio borraria los contadores, que son
// justamente el resultado de la prueba.
//
// Sirve para dos cosas:
//   - medir cuantas veces se pierde el agente micro-ROS y por cuanto tiempo
//   - validar el fix del desborde de 71.58 min: hay que pasar esa marca sin
//     reiniciar, con comandos de velocidad espaciados
//
// Se activa con -DDIAG_NO_RESTART=1 en platformio.ini (env esp32-s3-diag).
// En el env normal no esta definido y el firmware reinicia como siempre.

#ifndef DIAG_NO_RESTART
#define DIAG_NO_RESTART 0
#endif

// ---------------------------------------------------------------------------
// Forense de reinicios. Disponible en los dos modos: un reinicio de hardware
// (watchdog, brownout, panic) tampoco lo decide el firmware en produccion, y
// esta es la unica forma de saber por que ocurrio sin mirar el puerto serie.
// ---------------------------------------------------------------------------

extern esp_reset_reason_t g_rst_reason;
extern uint32_t g_loop_max_ms;
extern uint32_t g_loop_max_ms_since_report;

// Texto del motivo, para que el log se lea sin tabla de conversion
inline const char * resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "poweron";
    case ESP_RST_EXT:      return "ext_pin";
    case ESP_RST_SW:       return "sw";        // ESP.restart() del propio codigo
    case ESP_RST_PANIC:    return "panic";     // crash: excepcion o abort
    case ESP_RST_INT_WDT:  return "int_wdt";   // interrupcion bloqueada
    case ESP_RST_TASK_WDT: return "task_wdt";  // loop bloqueado
    case ESP_RST_WDT:      return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";  // caida de tension
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
  }
}

// Se llama al final de loop(). Mide cuanto tardo la iteracion anterior: si algo
// bloquea el bucle mas alla del timeout del task watchdog, aparece aca antes de
// provocar un reset.
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

inline void diagLogResetReason() {
  Serial.print("[DIAG] motivo del ultimo reset: ");
  Serial.print(resetReasonName(g_rst_reason));
  Serial.print(" (");
  Serial.print((int)g_rst_reason);
  Serial.print("), heap libre ");
  Serial.println(ESP.getFreeHeap());
}

#if DIAG_NO_RESTART

// Reemplaza a ESP.restart(): registra el motivo y sigue.
#define DIAG_RESTART(reason) diagSkipRestart(reason)

struct DiagStats {
  uint32_t restarts_skipped = 0;   // reinicios que se habrian producido
  uint32_t agent_lost = 0;         // veces que se perdio el agente
  uint32_t agent_recovered = 0;    // veces que volvio sin reiniciar
  uint32_t ping_fails = 0;         // pings fallidos acumulados
  uint32_t wifi_drops = 0;
  uint32_t cmd_vel_rx = 0;         // comandos de velocidad recibidos
  int64_t  last_cmd_vel_s = -1;    // hace cuanto llego el ultimo (s)
  uint32_t total_agent_down_s = 0;
  uint32_t max_agent_down_s = 0;
  String   last_skip_reason = "";
};

extern DiagStats diag;
extern WiFiUDP diag_udp;

static const uint16_t DIAG_PORT = 8891;
static const uint32_t DIAG_HEARTBEAT_MS = 30000;

inline void diagSkipRestart(const char * reason) {
  diag.restarts_skipped++;
  diag.last_skip_reason = reason;
  Serial.print("[DIAG] reinicio omitido (");
  Serial.print(reason);
  Serial.print("), total omitidos: ");
  Serial.println(diag.restarts_skipped);
}

inline void diagBegin() {
  diagLogResetReason();
  diag_udp.begin(DIAG_PORT);
  Serial.print("[DIAG] modo sin reinicios, reporte UDP ");
  Serial.println(DIAG_PORT);
}

inline String diagReport() {
  uint32_t up_s = (uint32_t)(esp_timer_get_time() / 1000000);
  int64_t now_us = esp_timer_get_time();

  String s = "{";
  s += "\"ev\":\"diag\",";
  s += "\"up_s\":";            s += String(up_s); s += ",";
  // La marca de las 71.58 min: si el firmware pasa de aca aceptando cmd_vel,
  // el desborde de 32 bits esta resuelto
  s += "\"past_wrap\":";       s += (up_s > 4295 ? "true" : "false"); s += ",";
  s += "\"restarts_skipped\":"; s += String(diag.restarts_skipped); s += ",";
  s += "\"agent_lost\":";      s += String(diag.agent_lost); s += ",";
  s += "\"agent_recovered\":"; s += String(diag.agent_recovered); s += ",";
  s += "\"ping_fails\":";      s += String(diag.ping_fails); s += ",";
  s += "\"wifi_drops\":";      s += String(diag.wifi_drops); s += ",";
  s += "\"cmd_vel_rx\":";      s += String(diag.cmd_vel_rx); s += ",";
  s += "\"last_cmd_vel_s\":";  s += String(diag.last_cmd_vel_s); s += ",";
  s += "\"agent_down_s\":";    s += String(diag.total_agent_down_s); s += ",";
  s += "\"max_agent_down_s\":"; s += String(diag.max_agent_down_s); s += ",";
  s += "\"rssi\":";            s += String(WiFi.RSSI()); s += ",";
  // Forense: motivo del ultimo reset y salud del heap y del bucle
  s += "\"rst\":";             s += String((int)g_rst_reason); s += ",";
  s += "\"rst_name\":\"";       s += resetReasonName(g_rst_reason); s += "\",";
  s += "\"heap\":";            s += String(ESP.getFreeHeap()); s += ",";
  s += "\"heap_min\":";        s += String(ESP.getMinFreeHeap()); s += ",";
  s += "\"loop_max_ms\":";     s += String(g_loop_max_ms); s += ",";
  // Pico del bucle solo en el intervalo desde el reporte anterior: un pico
  // aislado al arrancar no enmascara el comportamiento actual
  s += "\"loop_max_now\":";    s += String(g_loop_max_ms_since_report); s += ",";
  s += "\"reason\":\"";        s += diag.last_skip_reason; s += "\",";
  s += "\"ip\":\"";            s += WiFi.localIP().toString(); s += "\"";
  s += "}";
  return s;
}

inline void diagSend(IPAddress to, bool broadcast) {
  if (WiFi.status() != WL_CONNECTED)
    return;

  IPAddress dest = broadcast ? WiFi.broadcastIP() : to;
  String payload = diagReport();

  // Socket de envio dedicado: reusar el de escucha tras un parsePacket() deja
  // el destino pegado al ultimo remitente
  WiFiUDP tx;
  if (tx.beginPacket(dest, DIAG_PORT) == 1) {
    tx.print(payload);
    tx.endPacket();
  }
  tx.stop();
  Serial.println(payload);
  g_loop_max_ms_since_report = 0;
}

// Heartbeat periodico y atencion de comandos (STATUS / RESET)
inline void diagSpin() {
  static unsigned long last_hb_ms = 0;

  int len = diag_udp.parsePacket();
  if (len > 0) {
    char buf[32];
    int n = diag_udp.read(buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      String cmd(buf);
      cmd.trim();
      cmd.toUpperCase();
      if (cmd == "STATUS") {
        diagSend(diag_udp.remoteIP(), false);
      } else if (cmd == "RESET") {
        diag = DiagStats();
        g_loop_max_ms = 0;
        g_loop_max_ms_since_report = 0;
        Serial.println("[DIAG] contadores reiniciados");
        diagSend(diag_udp.remoteIP(), false);
      }
    }
  }

  if (millis() - last_hb_ms >= DIAG_HEARTBEAT_MS) {
    last_hb_ms = millis();
    diagSend(IPAddress(), true);
  }
}

#else  // modo normal: el firmware reinicia como siempre

#define DIAG_RESTART(reason) ESP.restart()
// En modo normal no hay reporte UDP, pero el motivo del reset se sigue
// registrando por serie: es el dato mas valioso tras un reinicio inesperado
inline void diagBegin() { diagLogResetReason(); }
inline void diagSpin() {}

#endif
