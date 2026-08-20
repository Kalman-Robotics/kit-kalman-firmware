# Petición para el firmware: instrumentar causa de reinicios y cuelgues

Repo: `kit-kalman-firmware`, rama `nexus-lab-telem`
Contexto: pruebas de estabilidad micro-ROS del 19-ago-2026 desde la Raspberry.

---

## Qué pasó

Durante una corrida de 2 h con el firmware `esp32-s3-diag` (modo diagnóstico,
`DIAG_NO_RESTART`), el ESP32 entró en un estado anómalo y **se reinició solo**.

Cronología del incidente (hora local de la Pi):

| Hora | Evento |
|---|---|
| 18:36 | Robot movido de sitio; RSSI baja de −30 a −44 dBm |
| ~18:38 | Telemetría micro-ROS deja de publicar |
| 18:38:30 | El ESP32 **seguía emitiendo** su reporte UDP de diag (8891), `up=01:59:17` |
| 18:39:08 | Pulso `cmd_vel` #23 → **SIN RESPUESTA** |
| ~18:39 | **El ESP32 se reinicia** (`up_s` vuelve a 0) |
| 18:42 | Recuperado, `up_s=278`, contadores a cero |

### Estado observado desde la Pi durante el fallo

```
ARP:          FAILED            (no responde a nivel IP)
tx bitrate:   1.0 MBit/s        (colapsó desde 72.2 — el AP no recibe ACKs)
authorized:   yes               (seguía asociado a nivel radio 802.11)
desasociaciones del AP: 0       (el WiFi nunca se cayó)
telemetría ROS: muerta
reportes UDP 8891: SEGUÍAN LLEGANDO hasta ~1.5 min antes del reinicio
```

**Lo importante:** el firmware estaba vivo y transmitiendo UDP mientras el
stack micro-ROS ya no respondía. Un fallo puro de radio habría cortado también
esos reportes. Y `restarts_skipped=0` con `up_s` reiniciado ⇒ el reinicio
**no lo decidió el firmware**: fue de hardware (watchdog, brownout o panic).

---

## Lo que NO es (verificado con datos)

- **No es el bug del wraparound.** Se cruzó la marca de 71.58 min y hubo
  9 pulsos `cmd_vel` correctos después. El síntoma además es distinto: aquí
  `cmd_vel_rx` se quedó congelado (el comando nunca llegó), mientras que el bug
  del `unsigned long` dejaba llegar el comando y frenaba los motores.
- **No es el WiFi del AP.** 0 desasociaciones, 0 errores de interfaz,
  `tx failed` 124 de ~3.9 M paquetes (0.003 %).
- **No es el agente micro-ROS.** CPU al 5 %, carga del sistema 0.81, sin
  errores en su log.
- **No son los `ping_fails`.** Ver análisis abajo: el código está bien.

## Lo que NO se puede determinar desde la Raspberry

La causa del reinicio. Esa información solo existe dentro del ESP32.

---

## Análisis del código actual (correcto, no hace falta cambiarlo)

`spinPing()` en `src/main.cpp:502`:

```cpp
rmw_ret_t rc = rmw_uros_ping_agent(cfg.UROS_PING_TIMEOUT_MS, 1);
if (rc != RMW_RET_OK) {
  diag.ping_fails++;
  ...
} else {
  ping_fail_count = 0;   // se resetea al primer exito — correcto
}
```

Config: ping cada 1 s, timeout 500 ms, `UROS_PING_MAX_FAILS = 5` **consecutivos**.

En 2 h ⇒ ~7.200 pings, 33 fallidos = **0,46 %**. Pérdida UDP normal sobre WiFi.
`agent_lost` se mantuvo en 0 en toda la corrida: la lógica funciona bien.
**No tocar esta parte.**

---

## Lo que se pide: instrumentar el diagnóstico

Objetivo: que el próximo incidente diga **por sí mismo** su causa, por UDP, sin
depender del puerto serie (el robot está en operación y vigilar el serie no es
práctico).

### 1. Razón del último reset — lo más importante

En `setup()`, capturar una vez:

```cpp
#include <esp_system.h>
esp_reset_reason_t g_rst_reason = esp_reset_reason();
```

Valores esperados y qué significaría cada uno:

| Valor | Enum | Interpretación |
|---|---|---|
| 3 | `ESP_RST_SW` | `ESP.restart()` — decisión del propio código |
| 4 | `ESP_RST_PANIC` | Crash del firmware (excepción / abort) |
| 7 | `ESP_RST_TASK_WDT` | **Loop bloqueado** — alguna tarea no cedió |
| 8 | `ESP_RST_INT_WDT` | Interrupción bloqueada |
| 11 | `ESP_RST_BROWNOUT` | **Caída de tensión** — problema de alimentación |

### 2. Memoria — para descartar fuga

```cpp
ESP.getFreeHeap()      // heap libre actual
ESP.getMinFreeHeap()   // mínimo histórico desde el arranque
```

Si `heap_min` baja de forma sostenida a lo largo de las horas, hay fuga y ese
sería el mecanismo del cuelgue.

### 3. Publicarlo en el JSON de diag

En `include/diag.h`, junto a los campos que ya existen:

```cpp
s += "\"rst\":";      s += String((int)g_rst_reason); s += ",";
s += "\"heap\":";     s += String(ESP.getFreeHeap()); s += ",";
s += "\"heap_min\":"; s += String(ESP.getMinFreeHeap()); s += ",";
s += "\"loop_max_ms\":"; s += String(g_loop_max_ms); s += ",";
```

### 4. Tiempo máximo del loop — opcional pero muy útil

Mediría directamente si algo bloquea el bucle principal (candidato número uno
para un `TASK_WDT`):

```cpp
// al final de loop()
static int64_t loop_start_us = 0;
uint32_t loop_ms = (esp_timer_get_time() - loop_start_us) / 1000;
if (loop_ms > g_loop_max_ms) g_loop_max_ms = loop_ms;
loop_start_us = esp_timer_get_time();
```

Se resetea con el comando `RESET` que ya existe.

### 5. Nota sobre el watchdog

`grep esp_task_wdt|WDT|watchdog|yield()` sobre el firmware ⇒ **sin resultados**.
No hay alimentación explícita de watchdog. Si el reset resulta ser
`ESP_RST_TASK_WDT`, conviene revisar si algún `lidar->loop()` o el executor de
micro-ROS puede bloquear el bucle más allá del timeout del TWDT.

---

## Hipótesis a verificar (por orden de sospecha)

1. **Brownout** (`rst=11`). El incidente ocurrió al mover el robot, con el
   lidar girando y motores activos. Un pico de corriente encaja.
2. **Task watchdog** (`rst=7`). Algo bloqueó el loop; el estado zombi previo
   (firmware emitiendo UDP pero micro-ROS muerto) es compatible con una tarea
   colgada mientras otra seguía viva.
3. **Fuga de memoria** (`heap_min` decreciente). Explicaría que ocurra tras
   ~2 h y no al principio.

Con los campos de arriba, el siguiente episodio distingue las tres sin tocar
el puerto serie.

---

## Cómo se validará

La Raspberry ya tiene corriendo `scripts/wifi_monitor.py --port 8891`, que
loguea cada reporte JSON. Al añadir los campos aparecerán automáticamente en
la captura y en `logs/forense.log`, que registra el momento exacto de cada
reinicio y estado zombi.

## Estado de la validación del wraparound (contexto, no es un problema)

- **23 de 24 pulsos OK.** El único fallo es este incidente.
- Marca de 71.58 min cruzada con **9 pulsos correctos después**.
- `gaps=0` en 149.960 mensajes de telemetría.
- El fix del `int64_t` **se sostiene**; falta repetir una corrida limpia de 2 h
  sin incidentes de red para cerrarlo formalmente.
