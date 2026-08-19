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
inline void diagBegin() {}
inline void diagSpin() {}

#endif
