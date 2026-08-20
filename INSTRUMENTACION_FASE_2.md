# Fase 2 de instrumentación: fallo UNIDIRECCIONAL de recepción

Repo: `kit-kalman-firmware`, rama `nexus-lab-telem`
Fecha: 19-ago-2026, tras la primera instrumentación (rst/heap/loop_max ya funcionando).

---

## Hallazgo nuevo — esto cambia el diagnóstico

Durante los episodios de "estado zombi", el ESP32 **sigue transmitiendo
perfectamente** pero **deja de recibir**.

Evidencia (episodio de las 19:19):

```
19:18:45  reporte UDP 8891  up=00:17:27     <- antes
19:19:15  reporte UDP 8891  up=00:17:57     <- DURANTE el fallo
19:19:45  reporte UDP 8891  up=00:18:27     <- DURANTE el fallo
19:20:15  reporte UDP 8891  up=00:18:57     <- despues
```

Cero huecos: los reportes salen cada 30 s exactos.

Simultáneamente, desde la Raspberry:
```
ping ICMP:    100 % de pérdida
ARP:          FAILED / INCOMPLETE
tx bitrate:   1.0 MBit/s   (el AP baja la tasa porque no recibe ACKs)
micro-ROS:    telemetría muerta, cmd_vel_rx congelado
asociación:   authorized=yes, 0 desasociaciones
```

**Conclusión: el camino ESP32 → Pi funciona; el camino Pi → ESP32 está roto.**

Esto descarta:
- Cuelgue del firmware (seguiría sin emitir)
- Caída de radio / desasociación (el AP no registra ninguna)
- Bloqueo del `loop()` (un bloqueo de 727 ms no explica minutos sin recibir,
  y los reportes de 30 s salieron puntuales)

Queda como sospecha principal: **el receptor WiFi del ESP32 o su stack lwIP de
entrada**, que deja de aceptar tramas mientras la transmisión sigue viva.

### Frecuencia observada
8 episodios entre 18:48 y 19:29 (uno cada 5-7 min). Empezaron tras mover el
robot a otra posición (RSSI de −30 a −41/−44 dBm). Antes de moverlo, 2 h sin
un solo episodio.

### Nota metodológica
La duración de "~22 s" que se observó en los primeros episodios es un artefacto
del script de vigilancia de la Raspberry (chequea cada 15 s + timeouts). **No es
la duración real del fallo.** Por eso hace falta medirla desde el firmware.

---

## Qué se pide instrumentar

### 1. Contadores de RECEPCIÓN — lo más importante

El objetivo es que el ESP32 diga si está recibiendo algo:

```cpp
// Cada vez que llega CUALQUIER paquete UDP al socket de comandos (8889/8891)
volatile uint32_t g_udp_rx_count = 0;
volatile int64_t  g_last_udp_rx_us = 0;   // esp_timer_get_time() del ultimo rx

// En el JSON:
s += "\"udp_rx\":";        s += String(g_udp_rx_count); s += ",";
s += "\"last_rx_s\":";     s += String((int)((esp_timer_get_time() - g_last_udp_rx_us)/1000000)); s += ",";
```

Con esto, durante un episodio se vería `last_rx_s` creciendo mientras el
reporte sigue saliendo ⇒ confirma el fallo unidireccional y **mide su duración
real**.

### 2. Estado del stack WiFi de ESP-IDF

```cpp
#include <esp_wifi.h>
wifi_ap_record_t ap;
esp_wifi_sta_get_ap_info(&ap);       // rssi, canal, estado de la asociacion

// Estadisticas del driver (si estan disponibles en la version de IDF):
s += "\"wifi_status\":"; s += String((int)WiFi.status()); s += ",";
s += "\"wifi_rssi\":";   s += String(WiFi.RSSI()); s += ",";
s += "\"wifi_ch\":";     s += String(ap.primary); s += ",";
```

Si `WiFi.status()` sigue diciendo `WL_CONNECTED` durante el fallo, el problema
está por debajo (driver/lwIP) y no en la capa Arduino.

### 3. Watchdog de recepción — la métrica que cierra el caso

```cpp
// En el loop, junto a spinPing():
if ((esp_timer_get_time() - g_last_udp_rx_us) > 10*1000*1000) {   // 10 s sin recibir NADA
    g_rx_stall_count++;
    Serial.println("RX STALL: 10s sin recibir un solo paquete");
    // Opcional: intentar recuperacion suave
    // WiFi.disconnect(); WiFi.reconnect();
}

s += "\"rx_stalls\":";     s += String(g_rx_stall_count); s += ",";
s += "\"rx_stall_max_s\":"; s += String(g_rx_stall_max_s); s += ",";
```

`rx_stall_max_s` daría **la duración real** de cada episodio, que hoy no se
puede medir desde la Raspberry.

### 4. Heap por partes (opcional)

El heap total bajó poco (172.436 → 170.056 en 28 min), pero conviene ver si
se agota una región concreta:

```cpp
s += "\"heap_dma\":"; s += String(heap_caps_get_free_size(MALLOC_CAP_DMA)); s += ",";
```

Los buffers de recepción WiFi usan memoria DMA. **Si `heap_dma` se agota, el
driver no puede reservar buffers de RX y deja de recibir — sin dejar de
transmitir.** Esa es la hipótesis que mejor encaja con lo observado.

---

## Hipótesis por orden de probabilidad

1. **Agotamiento de buffers DMA de recepción.** Explica exactamente el síntoma:
   TX sigue, RX muere, sin desasociación ni cuelgue. Se verifica con `heap_dma`.
2. **Bug del driver WiFi de ESP-IDF con RSSI bajo.** Empezó al mover el robot.
   Se verifica cruzando `wifi_rssi` con los episodios.
3. **Saturación de la cola de RX de lwIP.** Si algo no drena los sockets, se
   llenan y se descartan tramas. Se verifica con `udp_rx` estancado.

---

## Qué probar del lado de la Raspberry mientras tanto

- **Volver el robot a la posición original** (RSSI −30) y ver si los episodios
  desaparecen. Es la prueba más directa de si el RSSI es el disparador, y no
  cuesta nada.
- Cambiar el canal del AP (hoy canal 1) por si hay interferencia en 2.4 GHz.

## Lo que ya funciona y no hay que tocar

`rst_name`, `heap`, `heap_min`, `loop_max_ms`, `loop_max_now` — llegan bien y
ya dieron información útil (`loop_max_ms` llegó a 727 ms, aunque no explica
los episodios).
