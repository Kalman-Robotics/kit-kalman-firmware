# Protocolo de control ESP32 ↔ Raspberry Pi — v1

**Estado:** propuesta para implementar y validar
**Fecha:** 22-ago-2026
**Ámbito:** red aislada del AP `kalman-robot` (192.168.4.0/24)

---

## 1. Canales

Tres canales separados por responsabilidad. Un fallo o saturación en uno no
debe arrastrar a los otros.

| Canal | Puerto | Dirección | Responsabilidad | Frecuencia |
|---|---|---|---|---|
| Datos | 8888 | bidireccional | micro-ROS / XRCE-DDS | continuo |
| **Control** | **8889** | bidireccional | comandos y respuestas | bajo demanda |
| Telemetría | 8891 | ESP32 → Pi | estado del dispositivo | cada 30 s |

**Cambio respecto al estado actual:** hoy `RESET` y `STATUS` viven en el 8891
(canal de telemetría). Se mueven **todos los comandos al 8889**, dejando el 8891
solo para reportes.

### Direccionamiento

- **Comandos:** unicast a la IP del ESP32 (`192.168.4.48`, fija por reserva DHCP)
- **Excepción:** `PING` acepta broadcast a `192.168.4.255` para descubrimiento
- **Respuestas:** unicast al emisor, mismo puerto

---

## 2. Formato

### Comando (Pi → ESP32)

```
<COMANDO> [arg1] [arg2]
```

Texto plano ASCII, un comando por datagrama, mayúsculas, máximo 128 bytes.

### Respuesta (ESP32 → Pi)

JSON de una línea, máximo 256 bytes:

```json
{"v":1,"ack":"RESET_ODOM","result":"OK","state":"ACTIVE","up_s":4521,"seq":18}
```

| Campo | Tipo | Descripción |
|---|---|---|
| `v` | int | versión del protocolo (1) |
| `ack` | string | comando al que responde |
| `result` | string | `OK` · `ERR <motivo>` · `UNKNOWN` |
| `state` | string | `IDLE` · `ACTIVE` · `GRACE` |
| `up_s` | int | uptime en segundos |
| `seq` | int | secuencia de respuesta, incrementa siempre |

**Se responde SIEMPRE**, incluso a comandos desconocidos. Eso distingue
"no llegó" de "llegó y no se entendió", que es lo que hace observable dónde se
rompe el flujo.

`seq` permite detectar respuestas duplicadas cuando la Pi reintenta.

---

## 3. Comandos

### 3.1 Sesión

| Comando | Efecto | Estado resultante |
|---|---|---|
| `SESSION_START` | El agente micro-ROS está listo, conectarse | `ACTIVE` |
| `SESSION_END` | La sesión terminó, volver a esperar | `IDLE` |
| `PING` | Sonda, no cambia estado | sin cambio |

Ya implementados en el firmware (`include/session.h`). **Falta integrarlos**:
`session_notify.py` existe pero no se llama desde ningún sitio.

### 3.2 Reinicio parcial

Permiten dejar el robot en estado conocido **sin** los 60–90 s de un arranque
completo.

| Comando | Reinicia | NO toca |
|---|---|---|
| `RESET_ODOM` | `odom_pos_x/y/yaw` a 0 | sesión, WiFi, contadores |
| `RESET_COUNTERS` | contadores de diagnóstico | odometría, sesión |
| `RESET_MICROROS` | cierra y reabre la sesión XRCE-DDS | WiFi, odometría |
| `RESET_WIFI` | `esp_wifi_stop()` + `esp_wifi_start()` | odometría, contadores |
| `STOP_MOTORS` | `setMotorSpeeds(0,0)` inmediato | todo lo demás |

`RESET_WIFI` corta la conexión varios segundos: no es un comando de uso
rutinario.

### 3.3 Reinicio total

```
REBOOT <token>
REBOOT_SAFE <token>
```

`REBOOT_SAFE` frena motores y cierra la sesión micro-ROS antes de reiniciar.

**Token:** los últimos 4 dígitos hexadecimales de la MAC del ESP32
(`dc:b4:d9:04:72:fc` → `72fc`). No es seguridad —la red está aislada— sino
protección contra un reinicio accidental por un error de software en la Pi
durante una clase.

Sin token o con token incorrecto: `{"result":"ERR bad_token"}`.

**Nota de implementación:** en el build de diagnóstico (`DIAG_NO_RESTART`) los
reinicios están desactivados a propósito para preservar contadores. `REBOOT`
debe ser una excepción explícita a esa regla, o no funcionará en ese build.

### 3.4 Consulta

| Comando | Devuelve |
|---|---|
| `STATUS` | el JSON de diagnóstico completo (30+ campos) |
| `STATUS_NET` | subconjunto de red, más ligero para sondeos frecuentes |

---

## 4. Campos a añadir al JSON de telemetría

El reporte periódico actual no incluye el estado de sesión, que existe en el
firmware pero solo viaja en el ACK del 8889.

```json
"v": 1,
"session_state": "IDLE" | "ACTIVE" | "GRACE",
"session_since_s": 1234,
"odom_x": 4.69, "odom_y": 0.0, "odom_yaw": 0.0,
"motors_enabled": true,
"lidar_on": true,
"ping_timeout_ms": 100,
"last_cmd": "SESSION_START",
"last_cmd_s": 45,
"cmd_count": 47
```

**`last_cmd` / `last_cmd_s` / `cmd_count`** responden a una pregunta concreta:
cuando se revise un incidente, saber si alguien envió un comando justo antes.

---

## 5. Fiabilidad

UDP no garantiza entrega. La Raspberry debe:

1. Enviar cada comando **3 veces** con 200 ms de separación
2. Esperar respuesta hasta **2 s** desde el primer envío
3. Sin respuesta tras los 3 intentos → comando fallido, registrar y actuar

**Todos los comandos deben ser idempotentes.** Con reintentos triples, el ESP32
recibirá cada comando varias veces:

| Comando | Repetido |
|---|---|
| `RESET_ODOM` | sigue en 0 — correcto |
| `SESSION_START` estando `ACTIVE` | sin efecto — correcto |
| `REBOOT` | el primero reinicia, los demás se pierden — correcto |
| `STOP_MOTORS` | siguen parados — correcto |

---

## 6. Reacción ante desconexión

Esta es la lógica que ejecuta la Raspberry cuando el ESP32 deja de responder.

### 6.1 El fallo observado

Medido en 5 episodios entre el 21 y 22 de agosto. Firma constante:

```
tx bitrate (AP → ESP32):  1.0 Mbps    el AP no recibe ACKs
rx bitrate (ESP32 → AP):  65–72 Mbps  el AP SÍ recibe del ESP32
ping ICMP:                100 % pérdida
asociación:               authorized=yes (aparentemente sana)
```

**Diagnóstico:** la entrada de estación del AP se desincroniza. El camino
ESP32 → Pi funciona; el camino Pi → ESP32 está roto. El chip sigue vivo.

### 6.2 Escalado de recuperación

```
    ESP32 deja de responder
              │
              ▼
    ┌─────────────────────┐
    │ NIVEL 0 — esperar   │  4 ciclos x 15 s = 60 s de gracia
    │ ¿se recupera solo?  │  (evita reaccionar a cortes puntuales)
    └─────────┬───────────┘
              │ no
              ▼
    ┌─────────────────────┐
    │ NIVEL 1 — AP        │  iw dev wlan0 station del <MAC>
    │ expulsar estación   │  fuerza reasociación
    └─────────┬───────────┘  ✓ resolvió los 5 episodios medidos
              │ no responde en 30 s
              ▼
    ┌─────────────────────┐
    │ NIVEL 2 — ESP32     │  REBOOT <token> por UDP 8889
    │ reinicio remoto     │  (requiere que el 8889 responda)
    └─────────┬───────────┘
              │ sin ACK
              ▼
    ┌─────────────────────┐
    │ NIVEL 3 — humano    │  robot.failed al CMS
    │ intervención física │  "el robot necesita reinicio manual"
    └─────────────────────┘
```

### 6.3 Por qué ese orden

**Nivel 0 antes que nada.** El `hostapd-cleaner` original expulsaba la estación
tras **un solo** ping fallido de 1 s, y causaba ~90 reasociaciones por hora. Los
60 s de gracia evitan reaccionar ante pérdidas transitorias.

**Nivel 1 antes que nivel 2.** El `station del` es no destructivo: no reinicia
el ESP32, no pierde la odometría, no corta la sesión más de lo que ya está.
Resolvió los 5 episodios medidos.

**Nivel 2 solo si el 1 falla.** Un `REBOOT` cuesta 60–90 s de arranque completo.
Y tiene una limitación: si el ESP32 no recibe nada (que es el fallo observado),
el comando no llegará. Por eso va **después** del `station del`, que restaura
justamente ese camino.

**Nivel 3 cuando nada funciona.** Registrar y avisar, no reintentar en bucle.

### 6.4 Qué se registra en cada nivel

```csv
inicio,fin,nivel_resuelto,duracion_s,tx_bitrate,rx_bitrate,
up_s,rst_name,ping_fails,agent_lost,heap_min,rssi,session_id
```

`session_id` permite responder: **¿en qué sesión de qué alumno ocurrió?**

### 6.5 Durante una sesión activa

Si el fallo ocurre con un alumno conectado:

1. Notificar al CMS: `robot.degraded` con el nivel en curso
2. El CMS muestra "reconectando…" en vez de un robot que no responde
3. Al recuperar: `robot.ready` de nuevo
4. Si se llega al nivel 3: `robot.failed` y el CMS decide (cancelar, reprogramar)

---

## 7. Criterios de aceptación

A validar tras la implementación. **Los umbrales se fijan tras medir el
baseline**, no antes.

| Criterio | Cómo se mide | Umbral |
|---|---|---|
| El procesamiento de comandos no bloquea el bucle | `loop_max_ms` durante ráfaga de comandos | ≤ baseline + 20 % |
| La telemetría no se degrada | `ros2 topic hz /telemetry` | ≥ 19 Hz |
| Sin consumo acumulativo de memoria | `heap_min` tras 1000 comandos | sin tendencia decreciente |
| Latencia comando → ACK | timestamp en la Pi | p95 < 200 ms |
| Fiabilidad con reintentos triples | ACKs recibidos / comandos enviados | ≥ 99 % |

### Experimento de validación

1. **Baseline:** 30 min sin comandos → registrar `loop_max_ms`, hz, `heap_min`
2. **Carga progresiva:** un comando cada 10 s → cada 1 s → 10/s
3. **Medir lo mismo en cada tramo**
4. **Ajustar la spec** con los resultados

Puede que algo se caiga en el paso 4: quizá el ACK en JSON resulte costoso y
haya que volver al formato posicional, o `RESET_WIFI` bloquee demasiado.

---

## 8. Fuera de alcance (decisiones explícitas)

| Descartado | Motivo |
|---|---|
| Cifrado | Red aislada, sin salida a internet, solo el ESP32 conectado |
| Confirmación en dos fases | Innecesaria para comandos idempotentes |
| Cola de comandos en el ESP32 | Si llegan dos a la vez, el último gana: más simple |
| JSON en los comandos | El parsing en el ESP32 no aporta sobre texto plano |

---

## 9. Estado de implementación

| Elemento | Firmware | Raspberry |
|---|---|---|
| JSON de telemetría (8891) | ✓ implementado | ✓ `wifi_monitor.py` |
| `STATUS` / `RESET` | ✓ implementado (en 8891) | ✓ |
| `SESSION_START/END/PING` | ✓ implementado (8889) | ✗ `session_notify.py` sin integrar |
| ACK en JSON con `v`/`seq` | ✗ | ✗ |
| Reinicio parcial | ✗ | ✗ |
| `REBOOT` con token | ✗ | ✗ |
| Campos nuevos en el JSON | ✗ | — |
| Escalado de recuperación | — | parcial (`kalman-recuperador`, nivel 0–1) |
