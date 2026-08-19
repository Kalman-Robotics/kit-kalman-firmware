# Prueba de estabilidad de micro-ROS — instrucciones para la Raspberry

Objetivo: verificar que el enlace micro-ROS con el ESP32 no se pierde con el
tiempo, en particular pasada la marca de **71.58 minutos** donde aparecía el bug
histórico que obligaba a reiniciar cada 55 min.

---

## Contexto: el bug que se está validando

El ESP32 dejaba de aceptar comandos de velocidad pasada ~1 hora. Se le atribuía a
Fast DDS o a micro-ROS, y el parche fue reiniciar el ESP32 cada 55 minutos, lo
que limita cada sesión de laboratorio a esa duración.

**La causa raíz era otra**: `esp_timer_get_time()` devuelve `int64_t`
microsegundos, pero el firmware lo guardaba en `unsigned long`, que en ESP32 es
de 32 bits. Eso desborda a los **4.294.967.295 µs = 71.58 minutos**.

Tras el desbordamiento, el cálculo del timeout de `cmd_vel` daba un número
enorme, el firmware concluía que hacía 71 minutos que no llegaba un comando, y
**frenaba los motores en cada iteración**. El robot seguía recibiendo `/cmd_vel`
perfectamente; el timeout lo anulaba de inmediato.

Detalle que explica por qué se veía como fallo permanente: la recuperación
dependía de que llegara un comando nuevo que actualizara la variable. Con
comandos continuos (20 Hz) la ventana rota dura ~50 ms y es casi invisible. Con
comandos espaciados cada 5 minutos, dura **hasta 5 minutos** — y cada intento de
prueba caía dentro de ella.

**No era un bug de micro-ROS ni de Fast DDS.** Era aritmética de 32 bits en el
código de aplicación. El firmware ahora usa `int64_t` en todas las variables de
tiempo; el desbordamiento pasa a estar a ~292.000 años.

---

## Qué se va a correr

Firmware cargado en el ESP32: **`esp32-s3-diag`** — el sistema completo (lidar,
motores, IMU, telemetría, micro-ROS) pero en **modo diagnóstico: nunca
reinicia**. Cuenta cada evento y sigue, porque un reinicio borraría los
contadores que son el resultado de la prueba.

### Reinicios desactivados

| Causa | Cuándo dispararía normalmente |
|---|---|
| Agente perdido | 5 pings fallidos (~5 s) |
| WiFi no recuperado | 30 s sin reconectar |
| Fin de sesión | `SESSION_END` o grace de 2 min |
| `SESSION_START` en IDLE | aviso de la Raspberry |
| Agente no encontrado al arrancar | 60 s buscando |
| WiFi perdido durante el arranque | — |

**Importante:** como el ESP32 no reinicia si no encuentra al agente, hay que
tener el **agente micro-ROS corriendo** antes o durante el arranque del robot.

Los reinicios de hardware (watchdog, brownout, panic) **no** se pueden
desactivar. Si ocurren, el contador `up_s` del reporte se reinicia — así se
detectan.

---

## Cómo correrla

Dos terminales en la Raspberry.

### Terminal 1 — enviar comandos y verificar respuesta

```bash
python3 scripts/cmd_vel_test.py --interval 300 -o completa.log
```

Manda un pulso de `cmd_vel` cada 5 minutos (reproduce el escenario del bug) y
confirma la respuesta cruzando contra la telemetría que vuelve.

Opciones: `--interval <s>`, `--speed <m/s>` (default 0.08), `--duration <s>`.

Requiere `source /opt/ros/<distro>/setup.bash` y que `kalman_interfaces` esté en
el entorno.

### Terminal 2 — capturar los contadores del ESP32

```bash
python3 scripts/wifi_monitor.py --port 8891 -o completa_esp32.log
```

El firmware emite un reporte JSON por UDP broadcast cada 30 s.

Consulta puntual en cualquier momento:
```bash
python3 scripts/wifi_monitor.py --port 8891 --cmd STATUS
python3 scripts/wifi_monitor.py --port 8891 --cmd RESET   # contadores a cero
```

---

## Qué se está verificando

| Señal | Cómo se valida |
|---|---|
| `cmd_vel` | El robot recibe el comando |
| **Odometría** | `odom_vel_x` confirma que las ruedas giraron de verdad |
| **Telemetría** | Contador de mensajes + campo `seq` para detectar mensajes **perdidos**, no solo ausentes |
| **Lidar** | Bytes del campo `lds` que llegan en cada intervalo |
| **Sectores** | `dist_front_mm` (valida también el cálculo de mediana ±5°) |

Cada línea del log del terminal 1:

```
[15:32:10] up=01:12:30 #15 v=0.08 -> OK  odom=0.079 m/s
    (ok=15 fail=0 telem=8420 gaps=0 lidar=OK front=847mm)  [pasada la marca de 71.58 min]
```

`gaps=0` importa: detecta telemetría perdida por huecos en `seq`. Sin eso, un
enlace que pierde el 30% de los mensajes se vería igual que uno sano.

Reporte del ESP32 (terminal 2):

```json
{"ev":"diag","up_s":4350,"past_wrap":true,"restarts_skipped":0,
 "agent_lost":0,"agent_recovered":0,"ping_fails":0,"wifi_drops":0,
 "cmd_vel_rx":47,"last_cmd_vel_s":4200,"agent_down_s":0,"rssi":-38}
```

---

## El hito: 71.58 minutos (4295 s)

Ambos lados lo marcan:
- El script: `[pasada la marca de 71.58 min]`
- El firmware: `"past_wrap": true`

**Criterio de éxito:** llegar a ~2 horas con `fail=0` y `restarts_skipped=0`.

Si se cumple, el bug está cerrado y **se puede quitar el reinicio programado de
55 min**, con lo que las sesiones dejan de tener límite de duración.

---

## Si algo falla

Hay un segundo firmware compilado para aislar la causa: **`microros-test`** —
solo micro-ROS y un suscriptor de `cmd_vel`, sin lidar, motores, IMU ni
telemetría. Reporta por UDP **8892**.

```bash
pio run -e microros-test -t upload      # del lado del PC con el ESP32
python3 scripts/wifi_monitor.py --port 8892 -o aislado_esp32.log
```

La comparación da el diagnóstico:

| Aislado | Completo | Conclusión |
|---|---|---|
| falla | falla | El problema **no** es el código del robot: es micro-ROS, el agente o la red |
| OK | falla | El problema **sí** es de la aplicación (lidar, PID, memoria) |
| OK | OK | El fix del desborde resolvió todo |

---

## Estado de las otras pruebas

- **WiFi (completada):** el ESP32 mantiene la asociación con el AP. Firmware
  `wifi-test`, reportes en UDP 8890.
- **Canal de sesión (implementado, sin probar):** `scripts/session_notify.py`,
  UDP 8889. Falta integrarlo en el arranque/parada del contenedor Docker.
- **XML de Fast DDS (pendiente):** ver `docs/TAREAS_RASPBERRY.md`.

## Puertos UDP en uso

| Puerto | Uso |
|---|---|
| 8888 | agente micro-ROS (XRCE-DDS) |
| 8889 | canal de control de sesión |
| 8890 | prueba de WiFi |
| 8891 | diagnóstico del firmware completo |
| 8892 | prueba aislada de micro-ROS |
