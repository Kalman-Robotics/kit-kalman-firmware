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
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_core_dump.h>

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

// ---------------------------------------------------------------------------
// Deteccion de fallo UNIDIRECCIONAL de recepcion.
//
// Se observo al ESP32 transmitiendo con normalidad (reportes UDP puntuales
// cada 30 s) mientras dejaba de recibir: ping ICMP al 100 % de perdida, ARP
// FAILED, cmd_vel congelado, y aun asi authorized=yes y cero desasociaciones
// en el AP. TX vivo, RX muerto.
//
// Desde la Raspberry no se puede medir la duracion real de esos episodios: el
// script de vigilancia sondea cada 15 s y los "22 s" observados son un
// artefacto suyo. Estos contadores la miden desde adentro.
//
// Se registra todo trafico entrante, no solo el UDP de comandos: la Raspberry
// manda comandos de forma esporadica, asi que un contador que solo cuente esos
// casi nunca subiria. La respuesta del ping al agente micro-ROS es trafico
// entrante continuo (1 Hz) y sirve como testigo fiable de que el RX funciona.
// ---------------------------------------------------------------------------

extern volatile uint32_t g_rx_count;      // paquetes entrantes de cualquier tipo
extern volatile int64_t  g_last_rx_us;    // instante del ultimo
extern uint32_t g_rx_stalls;              // episodios de RX detenido
extern uint32_t g_rx_stall_max_s;         // el mas largo: duracion real del fallo
extern bool     g_rx_stalled;             // dentro de un episodio ahora mismo

// Uptime al comenzar cada corte. Si los cortes se agrupan cerca de 3600 s o de
// un multiplo, hay un contador que desborda o un temporizador que vence, lo que
// acota el bug enormemente; si salen dispersos, es un evento aleatorio.
// Observado hasta ahora: panic a ~1677 s, RX stall a 3587 s (59.8 min).
static const uint8_t RX_STALL_LOG_LEN = 8;
extern uint32_t g_rx_stall_at_s[RX_STALL_LOG_LEN];
extern uint8_t  g_rx_stall_log_n;

// Umbral para declarar RX detenido. El ping corre a 1 Hz, asi que 10 s sin un
// solo paquete entrante no puede ser normal.
static const int64_t RX_STALL_US = 10LL * 1000 * 1000;

// Llamar cada vez que llega algo desde la red
inline void diagNoteRx() {
  g_rx_count++;
  g_last_rx_us = esp_timer_get_time();
  if (g_rx_stalled) {
    uint32_t stall_s = 0;
    g_rx_stalled = false;
    Serial.print("[DIAG] RX restablecido tras ");
    Serial.print(stall_s);
    Serial.println("s");
  }
}

// Si el RX lleva demasiado tiempo muerto, provocar un panic a proposito: eso
// dispara el core dump y deja la fotografia del estado. Sin esto, el episodio
// del 22-ago dejo el chip inerte 10 h sin panic y por tanto sin volcado.
// Solo actua muy pasado el umbral de deteccion, para no interferir con cortes
// que se recuperan solos.
static const uint32_t RX_STALL_PANIC_S = 120;

// Vigila el silencio de entrada. Se llama desde loop().
inline void diagRxWatchdog() {
  if (WiFi.status() != WL_CONNECTED || g_last_rx_us == 0)
    return;

  int64_t silence_us = esp_timer_get_time() - g_last_rx_us;
  if (silence_us < RX_STALL_US) {
    g_rx_stalled = false;
    return;
  }

  uint32_t silence_s = (uint32_t)(silence_us / 1000000);
  if (silence_s > g_rx_stall_max_s)
    g_rx_stall_max_s = silence_s;

  if (!g_rx_stalled) {
    g_rx_stalled = true;
    g_rx_stalls++;

    // Momento en que empezo el corte, no en que se detecto
    uint32_t began_s = (uint32_t)((g_last_rx_us) / 1000000);
    if (g_rx_stall_log_n < RX_STALL_LOG_LEN)
      g_rx_stall_at_s[g_rx_stall_log_n++] = began_s;

    Serial.print("[DIAG] corte comenzo en up_s=");
    Serial.println(began_s);
    Serial.print("[DIAG] RX STALL #");
    Serial.print(g_rx_stalls);
    Serial.print(": ");
    Serial.print(silence_s);
    Serial.print("s sin recibir nada, WiFi.status()=");
    Serial.print(WiFi.status());
    Serial.print(" rssi=");
    Serial.println(WiFi.RSSI());
  }

  // Forzar el volcado cuando el corte ya no se va a recuperar
  if (silence_s >= RX_STALL_PANIC_S) {
    Serial.println("[DIAG] RX muerto demasiado tiempo: provocando core dump");
    Serial.flush();
    assert(false && "rx_stall: volcado forzado para capturar el estado");
  }
}

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

// ---------------------------------------------------------------------------
// Resumen del ultimo core dump.
//
// El framework de Arduino ya trae CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y con
// formato ELF, asi que los cuelgues anteriores YA escribieron su volcado en la
// particion coredump. Publicarlo por UDP evita tener que conectar el USB para
// saber donde murio.
//
// panic_pc se traduce a linea de codigo con:
//   xtensa-esp32s3-elf-addr2line -pfiaC -e .pio/build/<env>/firmware.elf 0x<PC>
// ---------------------------------------------------------------------------

extern uint32_t g_panic_pc;
extern char     g_panic_task[16];
extern bool     g_have_coredump;

// ---------------------------------------------------------------------------
// Traza circular en RTC RAM.
//
// El reporte cada 30 s es demasiado grueso: el cuelgue ocurre entre dos
// muestras y no se ve la transicion. Esta traza guarda los ultimos 128 eventos
// del loop en memoria RTC no inicializada, que SOBREVIVE a un reset (panic,
// watchdog o brownout), y se vuelca al arrancar. Dice en que punto del loop se
// quedo, incluso si el core dump fallara.
// ---------------------------------------------------------------------------

enum trace_ev_t {
  TR_LOOP = 1,
  TR_LIDAR = 2,
  TR_EXECUTOR = 3,
  TR_PING = 4,
  TR_TELEM = 5,
  TR_CMD_VEL = 6,
  TR_MOTORS = 7,
  TR_SESSION = 8,
};

inline const char * traceEvName(uint8_t e) {
  switch (e) {
    case TR_LOOP:     return "loop";
    case TR_LIDAR:    return "lidar";
    case TR_EXECUTOR: return "executor";
    case TR_PING:     return "ping";
    case TR_TELEM:    return "telem";
    case TR_CMD_VEL:  return "cmd_vel";
    case TR_MOTORS:   return "motors";
    case TR_SESSION:  return "session";
    default:          return "?";
  }
}

#define TRACE_LEN   128
#define TRACE_MAGIC 0x54524331  // "TRC1"

struct TraceBuf {
  uint32_t magic;
  uint16_t idx;
  struct {
    uint32_t t_ms;
    uint8_t  ev;
    uint16_t data;
  } e[TRACE_LEN];
};

extern RTC_NOINIT_ATTR TraceBuf g_trace;

// Muy barata a proposito: la llama el loop miles de veces por segundo.
// Sin IRAM_ATTR: solo la usa el loop, nunca un ISR, y marcarla como inline en
// IRAM provoca errores de relocacion al enlazar.
inline void traceMark(uint8_t ev, uint16_t data = 0) {
  uint16_t i = g_trace.idx;
  g_trace.e[i].t_ms = (uint32_t)(esp_timer_get_time() / 1000);
  g_trace.e[i].ev = ev;
  g_trace.e[i].data = data;
  g_trace.idx = (i + 1) % TRACE_LEN;
}

// Vuelca por serie la traza del arranque anterior, si es valida
inline void diagDumpTrace() {
  if (g_trace.magic != TRACE_MAGIC) {
    // Primer arranque o RAM perdida: inicializar
    g_trace.magic = TRACE_MAGIC;
    g_trace.idx = 0;
    memset(g_trace.e, 0, sizeof(g_trace.e));
    return;
  }

  Serial.println("[DIAG] traza del arranque anterior (mas reciente al final):");
  uint16_t start = g_trace.idx;
  for (uint16_t k = 0; k < TRACE_LEN; k++) {
    uint16_t i = (start + k) % TRACE_LEN;
    if (g_trace.e[i].t_ms == 0)
      continue;
    Serial.print("  t=");
    Serial.print(g_trace.e[i].t_ms);
    Serial.print("ms ");
    Serial.print(traceEvName(g_trace.e[i].ev));
    if (g_trace.e[i].data) {
      Serial.print(" d=");
      Serial.print(g_trace.e[i].data);
    }
    Serial.println();
  }
  g_trace.idx = 0;
  memset(g_trace.e, 0, sizeof(g_trace.e));
}

inline void diagReadCoreDump() {
  esp_core_dump_summary_t * sum =
    (esp_core_dump_summary_t *) malloc(sizeof(esp_core_dump_summary_t));
  if (sum == NULL)
    return;

  if (esp_core_dump_get_summary(sum) == ESP_OK) {
    g_have_coredump = true;
    g_panic_pc = sum->exc_pc;
    strncpy(g_panic_task, sum->exc_task, sizeof(g_panic_task) - 1);
    g_panic_task[sizeof(g_panic_task) - 1] = 0;

    Serial.print("[DIAG] core dump del cuelgue anterior: PC=0x");
    Serial.print(g_panic_pc, HEX);
    Serial.print(" tarea=");
    Serial.println(g_panic_task);
    Serial.println("[DIAG] traducir con: xtensa-esp32s3-elf-addr2line -pfiaC "
                   "-e .pio/build/<env>/firmware.elf 0x<PC>");
  }
  free(sum);
}

inline void diagLogResetReason() {
  Serial.print("[DIAG] motivo del ultimo reset: ");
  Serial.print(resetReasonName(g_rst_reason));
  Serial.print(" (");
  Serial.print((int)g_rst_reason);
  Serial.print("), heap libre ");
  Serial.println(ESP.getFreeHeap());
  diagReadCoreDump();
  diagDumpTrace();
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

  // Recepcion: si last_rx_s crece mientras este reporte sigue saliendo, el
  // fallo es unidireccional y aca queda medida su duracion real
  int64_t since_rx = g_last_rx_us > 0 ?
    (esp_timer_get_time() - g_last_rx_us) / 1000000 : -1;
  s += "\"rx\":";              s += String(g_rx_count); s += ",";
  s += "\"last_rx_s\":";       s += String((long)since_rx); s += ",";
  s += "\"rx_stalls\":";       s += String(g_rx_stalls); s += ",";
  s += "\"rx_stall_max_s\":";  s += String(g_rx_stall_max_s); s += ",";
  s += "\"stall_at\":[";
  for (uint8_t i = 0; i < g_rx_stall_log_n; i++) {
    if (i) s += ",";
    s += String(g_rx_stall_at_s[i]);
  }
  s += "],";

  // Los buffers de recepcion del driver WiFi salen de memoria DMA. Si esta
  // region se agota, el driver no puede reservar buffers de RX y deja de
  // recibir sin dejar de transmitir: encaja exactamente con lo observado.
  s += "\"heap_dma\":";        s += String(heap_caps_get_free_size(MALLOC_CAP_DMA)); s += ",";
  s += "\"heap_dma_min\":";    s += String(heap_caps_get_minimum_free_size(MALLOC_CAP_DMA)); s += ",";

  // Si WiFi.status() sigue en WL_CONNECTED (3) durante el fallo, el problema
  // esta por debajo de la capa Arduino
  s += "\"wifi_status\":";     s += String((int)WiFi.status()); s += ",";
  if (g_have_coredump) {
    s += "\"panic_pc\":\"0x";     s += String(g_panic_pc, HEX); s += "\",";
    s += "\"panic_task\":\"";     s += g_panic_task; s += "\",";
  }
  {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
      s += "\"wifi_ch\":";     s += String(ap.primary); s += ",";
    }
  }
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

  diagRxWatchdog();

  int len = diag_udp.parsePacket();
  if (len > 0) {
    diagNoteRx();
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
        g_rx_stalls = 0;
        g_rx_stall_max_s = 0;
        g_rx_stall_log_n = 0;
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
