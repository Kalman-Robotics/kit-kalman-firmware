# Petición al firmware: comandos de control

Repo: `kit-kalman-firmware`, rama `nexus-lab-telem`
Fecha: 22-ago-2026

---

## Lo que ya funciona (validado con medición)

El firmware cargado hoy responde bien:

| Comando | Latencia | Resultado |
|---|---|---|
| `STATUS` | 5.9 ms media, **p95 10.8 ms** | JSON de 33 campos |
| `HISTORY` | 600 ms | historial en RTC RAM |

**Fiabilidad medida: 20/20 comandos respondidos, 0 pérdidas, sin reintentos.**

Campos nuevos confirmados: `boots`, `panic_pc`, `panic_task`.

### Hallazgo importante del último panic

```
panic_pc:   0x40377c4a
panic_task: tiT
```

**`tiT` es la tarea TCP/IP de lwIP.** El crash ocurrió dentro del stack de red,
no en el loop de aplicación. Es la pista más concreta que hemos tenido: encaja
con que el fallo sea de recepción y con que el `int_wdt` salte por una tarea de
red bloqueada.

Para traducir la dirección a línea de código:
```bash
xtensa-esp32s3-elf-addr2line -pfiaC -e .pio/build/<env>/firmware.elf 0x40377c4a
```

---

## Lo que falta — probado y sin respuesta

Ejecutado el 22-ago 16:29, cada comando con 3 s de espera:

```
RESET            SIN RESPUESTA
PING             SIN RESPUESTA
RESET_ODOM       SIN RESPUESTA
STOP_MOTORS      SIN RESPUESTA
RESET_MICROROS   SIN RESPUESTA
COMANDO_FALSO    SIN RESPUESTA   <- deberia responder UNKNOWN
```

---

## Prioridad 1 — responder SIEMPRE

**El más importante, aunque parezca menor.**

Un comando desconocido debe devolver:

```json
{"v":1,"ack":"COMANDO_FALSO","result":"UNKNOWN","state":"IDLE","up_s":507,"seq":12}
```

Sin esto, no se puede distinguir:
- el comando no llegó (problema de red)
- el comando llegó pero no se entiende (problema de versión)
- el ESP32 está colgado (problema del chip)

Los tres se ven igual desde la Raspberry: silencio. Responder siempre es lo que
hace observable dónde se rompe el flujo.

---

## Prioridad 2 — comandos de estado seguro

Permiten dejar el robot listo entre alumnos **sin** los 60–90 s de un reinicio
completo.

| Comando | Efecto | No toca |
|---|---|---|
| `STOP_MOTORS` | `setMotorSpeeds(0,0)` inmediato | sesión, WiFi, odometría |
| `RESET_ODOM` | `odom_pos_x/y/yaw` a 0 | sesión, WiFi, contadores |

Ambos idempotentes: repetirlos no cambia nada.

Respuesta esperada:
```json
{"v":1,"ack":"RESET_ODOM","result":"OK","state":"ACTIVE","up_s":4521,"seq":18}
```

---

## Prioridad 3 — recuperación parcial

| Comando | Efecto | Nota |
|---|---|---|
| `RESET_MICROROS` | cierra y reabre la sesión XRCE-DDS | sin tocar WiFi |
| `RESET_WIFI` | `esp_wifi_stop()` + `esp_wifi_start()` | corta la conexión unos segundos |

`RESET_WIFI` sería la versión desde el ESP32 de lo que hoy hacemos desde la Pi
con `iw station del`.

**Advertencia:** el fallo que investigamos es que el ESP32 **deja de recibir**.
Si eso ocurre, ninguno de estos comandos le llegará. Por eso el escalado de
recuperación pone `iw station del` (lado Pi) **antes** que cualquier comando
remoto.

---

## Prioridad 4 — reinicio remoto

```
REBOOT <token>
REBOOT_SAFE <token>
```

`REBOOT_SAFE` frena motores y cierra la sesión micro-ROS antes de reiniciar.

**Token:** últimos 4 dígitos hex de la MAC (`dc:b4:d9:04:72:fc` → `72fc`).
No es seguridad —la red está aislada— sino protección contra un reinicio
accidental por un bug en la Pi durante una clase.

Sin token o incorrecto: `{"result":"ERR bad_token"}`

**Nota:** en el build de diagnóstico (`DIAG_NO_RESTART`) los reinicios están
desactivados para preservar contadores. `REBOOT` debe ser excepción explícita.

---

## Prioridad 5 — unificar el canal

Hoy los comandos viven en el 8891 (canal de telemetría). La especificación
propone moverlos al 8889 (canal de control), dejando el 8891 solo para reportes.

**Ventaja:** si la telemetría se satura, el control sigue respondiendo.

**Coste:** hay que actualizar los scripts de la Raspberry. No urgente — se puede
hacer cuando el resto esté estable, o mantener ambos puertos durante la
transición.

---

## Formato de respuesta acordado

```json
{"v":1,"ack":"<COMANDO>","result":"OK|ERR <motivo>|UNKNOWN",
 "state":"IDLE|ACTIVE|GRACE","up_s":4521,"seq":18}
```

`seq` incrementa siempre y permite detectar respuestas duplicadas cuando la Pi
reintenta.

---

## Campos que faltarían en el JSON periódico

Ya están `boots`, `panic_pc`, `panic_task`. Añadiría:

```json
"session_state": "IDLE" | "ACTIVE" | "GRACE",
"odom_x": 4.69, "odom_y": 0.0, "odom_yaw": 0.0,
"last_cmd": "STATUS", "last_cmd_s": 45, "cmd_count": 47
```

`session_state` evita tener que sondear el 8889 para saber la fase del robot.

`last_cmd` / `cmd_count` responden a una pregunta concreta al revisar un
incidente: **¿alguien mandó un comando justo antes?**

---

## Criterios de aceptación

Medidos con `scripts/probar_eventos.py` en la Raspberry.

| Criterio | Umbral | Medido hoy |
|---|---|---|
| Latencia p95 comando → ACK | < 200 ms | **10.8 ms** ✓ |
| Fiabilidad sin reintentos | ≥ 99 % | **100 %** ✓ |
| `loop_max_ms` durante ráfaga | ≤ baseline + 20 % | pendiente |
| `/telemetry` durante ráfaga | ≥ 19 Hz | pendiente |
| `heap_min` tras 1000 comandos | sin tendencia | pendiente |

Los tres pendientes se miden cuando existan los comandos de control: la prueba
consiste en enviar comandos a ritmo creciente (1 cada 10 s → 1/s → 10/s) y
comparar contra un baseline de 30 min sin comandos.
