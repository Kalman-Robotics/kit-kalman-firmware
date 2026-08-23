# Fase 4: core dump y trazas — llegar a la causa raíz

Repo: `kit-kalman-firmware`, rama `nexus-lab-telem`
Fecha: 22-ago-2026. Todas las horas en hora local (America/Lima, UTC−5).

---

## Resumen: 4 días de pruebas, causa aún no identificada

El ESP32 se bloquea por completo en momentos impredecibles. Se han descartado
seis hipótesis con datos, pero **la instrumentación externa ya no puede avanzar
más**: todo lo que se mide desde la Raspberry son síntomas posteriores al fallo.

### Los cuatro episodios medidos

| Fecha | Tiempo hasta el fallo | Carga durante la prueba |
|---|---|---|
| 19-ago | ~1 h 40 min | baja (1 pulso/20 min) |
| 20-ago | 12 h 35 min | baja |
| 21-ago | 22 min | media |
| **22-ago** | **6 h 38 min** | **MÁXIMA** (ver abajo) |

**No hay correlación con la carga.** El episodio con más estrés aguantó más que
el de carga media. Un fallo eléctrico o de agotamiento de recursos habría
mostrado la tendencia contraria.

### La prueba de estrés máximo (21-ago 20:53 → 22-ago 03:01)

Durante 6 h 38 min, de forma simultánea y sostenida:

```
cmd_vel     0.15 m/s (velocidad máxima) + 1.0 rad/s, publicado a 10 Hz
motores     forzando contra una pared (corriente alta continua)
lidar       encendido, girando a 10 Hz
vibración   giro continuo con impactos
total       235.684 comandos cmd_vel recibidos
```

Resultado: **ningún indicador se degradó** hasta el instante del fallo.

### El instante del fallo — cero preaviso

Los cuatro últimos reportes antes del silencio, cada 30 s:

```
up_s=23813  pf=5  lost=0  stalls=0  loop=537  heap_min=156872  cmd_rx=234784
up_s=23843  pf=5  lost=0  stalls=0  loop=537  heap_min=156872  cmd_rx=235084
up_s=23873  pf=5  lost=0  stalls=0  loop=537  heap_min=156872  cmd_rx=235384
up_s=23903  pf=5  lost=0  stalls=0  loop=537  heap_min=156872  cmd_rx=235684
--- 03:01:57 último reporte, silencio total desde entonces ---
```

Idénticos. Sin fuga de memoria, sin `ping_fails` creciendo, sin `rx_stalls`,
sin aumento del tiempo de loop. **Estaba perfecto y desapareció.**

### Tras el fallo: el chip queda completamente inerte

Verificado con medición directa (22-ago 13:44, 10 h 42 min después):

```
paquetes recibidos del ESP32 en 30 s:  0        <- NO transmite
ping:                                  100 % pérdida
```

Nota metodológica: `iw station dump` sigue mostrando `inactive time: 0 ms`,
`authorized: yes` y `connected time: 17 h`, pero **son valores residuales
congelados**. El contador `rx packets` del AP no avanza: el chip no ejecuta
código.

---

## Hipótesis descartadas con datos

| Hipótesis | Cómo se descartó |
|---|---|
| Fuga de memoria | `heap_min` inmóvil en 156872 durante 6 h de estrés |
| Buffers DMA agotados | `heap_dma_min` inmóvil en 149088 |
| Volumen de tráfico `cmd_vel` | 235.684 comandos sin degradación |
| Consumo eléctrico de motores | 6 h 38 min forzando contra pared, sin efecto |
| Pico de corriente del lidar | ciclos ON/OFF sin ningún cambio medible |
| Vibración / contacto intermitente | giro con impactos continuos, sin efecto |
| RSSI bajo | falló a −28 dBm (40 cm) y aguantó 10 h a −49 dBm |
| Brownout | `rst_name=brownout` solo aparece al cortar la fuente a mano |

## Hipótesis principal: condición de carrera

Lo que encaja con **todas** las observaciones:

- **Tiempos aleatorios** (22 min a 12 h 35 min): probabilidad baja y constante
  por iteración produce una distribución exponencial. El loop corre miles de
  veces por segundo.
- **Cae de golpe**: no hay recurso agotándose progresivamente.
- **`rst_name = int_wdt`** (visto 2 veces): el Interrupt Watchdog salta cuando
  las interrupciones quedan bloqueadas >300 ms. Es la firma típica de un
  interbloqueo, no de un fallo eléctrico.
- **El estrés máximo no lo aceleró**: descarta agotamiento de recursos.

Candidatos concretos: un ISR (WiFi, encoders, lidar) que interrumpe mientras
una tarea tiene tomado un mutex, o acceso concurrente sin protección a una
estructura compartida.

---

## LO QUE SE PIDE

### 1. Core dump a flash — PRIORITARIO

Es la única vía que queda para ver el interior del ESP32 en el instante del
fallo.

En `sdkconfig` / `platformio.ini`:
```
CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y
CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF=y
CONFIG_ESP_COREDUMP_CHECKSUM_CRC32=y
CONFIG_ESP_COREDUMP_MAX_TASKS_NUM=64
CONFIG_ESP_COREDUMP_STACK_SIZE=1024
```

Requiere una partición `coredump` en la tabla de particiones (~64 KB):
```csv
# Name,   Type, SubType, Offset,  Size
coredump, data, coredump,,        64K
```

**Tras el próximo cuelgue**, se lee por USB cuando convenga (no hace falta
vigilar el serie en el momento):
```bash
idf.py coredump-info      # resumen legible: tareas, backtrace
idf.py coredump-debug     # sesión gdb sobre el volcado
```

Qué distinguiría:

| Resultado | Conclusión |
|---|---|
| Backtrace en una función concreta | **Software**: se tiene la línea exacta |
| Tarea esperando un mutex/semáforo | **Condición de carrera confirmada** |
| Punteros corruptos / stack desbordado | Corrupción de memoria |
| **No hay dump pese a estar activado** | El chip murió sin poder escribir → apunta a hardware |

### 2. Publicar el resumen del dump por UDP

Para no depender de conectar el USB, publicar en el JSON de diag lo esencial
del último core dump:

```cpp
#include <esp_core_dump.h>

esp_core_dump_summary_t s;
if (esp_core_dump_get_summary(&s) == ESP_OK) {
    // direccion donde crasheo y tarea implicada
    "panic_pc":   0x...
    "panic_task": "nombre de la tarea"
}
```

`panic_pc` se traduce a línea de código con:
```bash
xtensa-esp32s3-elf-addr2line -pfiaC -e .pio/build/<env>/firmware.elf 0x<PC>
```

### 3. Bajar el timeout del Interrupt Watchdog

Para que el `int_wdt` salte antes y capture el bloqueo más cerca de su origen:

```
CONFIG_ESP_INT_WDT_TIMEOUT_MS=300     # valor por defecto, confirmar
CONFIG_ESP_TASK_WDT_EN=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=y
```

Nota: el firmware **no alimenta ningún watchdog explícitamente**
(`grep esp_task_wdt` no devuelve resultados). Activar el Task WDT daría una
segunda red de seguridad y otra firma distinta cuando el bloqueo esté en una
tarea y no en un ISR.

### 4. Traza circular en RAM — para ver los últimos milisegundos

El reporte cada 30 s es demasiado grueso: el fallo ocurre entre dos muestras.
Un buffer circular en RAM no inicializada (`RTC_NOINIT_ATTR` sobrevive al
reset) permitiría ver qué hacía el firmware justo antes:

```cpp
RTC_NOINIT_ATTR struct {
    uint32_t magic;
    uint16_t idx;
    struct { uint32_t t_ms; uint8_t evento; uint16_t dato; } ev[256];
} traza;

// marcar eventos clave en el loop:
//   1=inicio_loop  2=lidar_loop  3=executor_spin  4=ping  5=telem
//   6=cmd_vel_rx   7=motor_update  8=wifi_event
TRAZA(3, ret);   // macro ligera, solo escribe timestamp + id

// en setup(), si magic coincide, volcar la traza por UDP:
// muestra los ultimos ~256 eventos ANTES del bloqueo
```

Esto diría **en qué punto del loop se quedó**, incluso si el core dump fallara.

### 5. Estadísticas de lwIP (pendiente de la fase 3)

Sigue sin aplicarse y ayudaría a descartar la capa de red:

```
CONFIG_LWIP_STATS=y
```
```cpp
lwip_stats.udp.recv / .drop / .memerr
lwip_stats.ip.recv / .drop
lwip_stats.pbuf.err
```

### 6. Marcar las secciones críticas sospechosas

Si hay `portENTER_CRITICAL`, `taskENTER_CRITICAL`, `noInterrupts()` o mutex
tomados en el loop o en ISRs, conviene instrumentar su duración:

```cpp
// registrar cuanto se mantiene cada seccion critica
// si alguna supera ~50 ms, es candidata directa al int_wdt
```

---

## Prueba de control (lado Raspberry, sin coste)

Alimentar el ESP32 **por USB desde un ordenador**, sin la fuente del robot:

- Si con USB no falla nunca → el problema es la alimentación
- Si falla igual → confirma que es software

Se puede hacer en paralelo al core dump.

---

## Estado del resto de la investigación

**Cerrado y validado:**
- Bug del wraparound `int64_t`: ~41 pulsos `cmd_vel` correctos pasada la marca
  de 71.58 min. **El reinicio de 55 min ya no hace falta** (cron desactivado).
- Bug del AP: `ap_max_inactivity=5` → 300 en `hostapd.conf`. Causaba ~90
  reasociaciones/hora.
- `UROS_PING_TIMEOUT_MS` 500 → 100: eliminó el bucle de realimentación que
  convertía un corte breve en permanente. `loop_max_ms` pasó de 1995 a ~50 ms.

**Validado hoy — Discovery Server:**
- Elimina el XML de peers y con él el delay al conectarse el estudiante.
- 0 errores de XMLPARSER, 16 tópicos, telemetría 19.7 Hz, joystick funcionando.
- Los nodos **no se relanzan** cuando aparece `lab-maze` en `/etc/hosts`.

**Abierto:** este bloqueo aleatorio del ESP32.
