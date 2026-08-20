# Fase 3 — Plan de trabajo

## Qué hace el cambio que vas a aplicar

```
UROS_PING_TIMEOUT_MS: 500 → 100
```

### Lo que SÍ ataca

Cuando el agente no responde, `rmw_uros_ping_agent()` **bloquea el loop durante
todo el timeout**. Con 500 ms y un ping por segundo, el firmware pasa el **50 %
del tiempo congelado**. Bajándolo a 100 ms, pasa al **10 %**.

Eso rompe el bucle de realimentación que convierte un corte breve en permanente:

```
corte de 1-2 s
  → el ping falla → loop bloqueado 500 ms
  → durante ese tiempo NO se drena el socket UDP
  → llegan paquetes que nadie recoge
  → mas pings fallan → mas bloqueo
  → el corte no se cierra nunca
```

Evidencia que lo respalda (medida en este robot):
```
loop_max_now:  36 ms   (funcionamiento normal)
               508 ms  (primer ping fallido)
               1995 ms (cuelgue establecido)
```

Y en la comunidad: [micro_ros_espidf_component#165](https://github.com/micro-ROS/micro_ros_espidf_component/issues/165)
reporta un ESP32-S3 donde un timer de 100 Hz **cayó a 1 Hz** al perderse el agente.

### Lo que NO ataca — importante

**Este cambio no explica por qué empieza el corte.** Solo evita que se extienda.

Es una mitigación válida y probablemente muy efectiva, pero la causa raíz sigue
sin identificarse.

---

## Qué haremos después de aplicarlo

### Objetivo: averiguar QUÉ rompe la recepción

El dato que tenemos del último corte:

```
up_s=3584   rx=4416   last_rx=0    pf=0    loop=36ms     <- todo normal
up_s=3614   rx=4419   last_rx=27   pf=26   loop=508ms    <- ya cortado
```

El corte empezó en **up_s=3587 (59 min 47 s)**, de forma **súbita**: sin
degradación previa de heap, sin pings fallidos antes, con RSSI −30 dBm y a
40 cm de la Pi. Entre los dos reportes el ESP32 recibió 3 paquetes y se detuvo.

También: el último `cmd_vel` fue en `up_s=3430`, **157 s antes** del corte. Con
pulsos cada 300 s, el corte cayó a mitad del intervalo de espera ⇒ **el
disparador no es recibir un comando**.

### Paso 1 — Registrar el uptime de cada corte (sin tocar firmware)

Lo que más interesa de tu observación ("siempre pasa después de un tiempo X"):

| Episodio | up_s al fallar |
|---|---|
| Panic | ~1677 s (28 min) |
| RX stall | **3587 s (59.8 min)** |

Dos puntos no son un patrón, pero **59.8 min está muy cerca de una hora**.

- Si los próximos cortes se agrupan cerca de 3600 s o de un múltiplo ⇒ hay un
  **contador que desborda o un temporizador que vence**. Eso acota el bug
  enormemente.
- Si salen dispersos ⇒ es un evento aleatorio y hay que ir a lwIP.

**Coste: cero.** Solo hay que dejar correr y anotar. Ya está automatizado en
`vigila-recuperacion.sh`.

### Paso 2 — Estadísticas de lwIP (si el paso 1 no da patrón)

En `sdkconfig`:
```
CONFIG_LWIP_STATS=y
```

Y publicar en el JSON de diag:
```cpp
#include "lwip/stats.h"
s += "\"lwip_udp_recv\":";   s += String(lwip_stats.udp.recv);   s += ",";
s += "\"lwip_udp_drop\":";   s += String(lwip_stats.udp.drop);   s += ",";
s += "\"lwip_udp_memerr\":"; s += String(lwip_stats.udp.memerr); s += ",";
s += "\"lwip_ip_recv\":";    s += String(lwip_stats.ip.recv);    s += ",";
s += "\"lwip_ip_drop\":";    s += String(lwip_stats.ip.drop);    s += ",";
s += "\"lwip_pbuf_err\":";   s += String(lwip_stats.pbuf.err);   s += ",";
```

Esto responde directamente dónde se pierden los paquetes:

| Observación | Conclusión |
|---|---|
| `ip_recv` sube, `udp_recv` no | se pierden entre IP y UDP |
| `udp_drop` sube | lwIP los descarta (ver `memerr`) |
| `ip_recv` tampoco sube | el problema está bajo lwIP (driver) |
| `pbuf_err` > 0 | se agotaron los buffers de red |

### Paso 3 — Buffer de alta resolución (si hace falta el instante exacto)

El reporte cada 30 s es demasiado grueso: el corte ocurre entre dos muestras.

```cpp
// Buffer circular en RAM: ultimos 60 s a 100 ms de resolucion
struct Sample { uint32_t rx; uint16_t loop_ms; int8_t rssi; };
Sample hist[600];
// Al detectar rx_stall, volcarlo por UDP
```

Mostraría la transición real, no el resultado 30 s después.

---

## Cómo mediremos si el cambio funcionó

Con `UROS_PING_TIMEOUT_MS=100`, comparar contra esta corrida:

| Métrica | Antes (500 ms) | Qué esperamos |
|---|---|---|
| `loop_max_ms` durante fallo | 1995 ms | ≤ 400 ms |
| Duración del stall | 3330 s (no se recupera) | segundos, o recuperación |
| `rx_stalls` en 2 h | 1 (permanente) | pueden ser más pero **cortos** |
| `agent_recovered` | 0 | **> 0** ⇒ se recupera solo |

**El indicador clave es `agent_recovered`.** Si pasa de 0 a un número positivo,
significa que el ESP32 recupera la sesión sin reiniciar — que es exactamente lo
que hoy no ocurre.

Un matiz honesto: si los stalls siguen apareciendo pero se recuperan solos, el
robot sería usable, pero la causa raíz seguiría ahí. Por eso el paso 1 (registrar
el uptime de cada corte) hay que hacerlo igual.

---

## Segundo cambio a considerar

```
RMW_UXRCE_ALLOW_DYNAMIC_ALLOCATIONS=ON
```

Workaround de [rmw_microxrcedds#241](https://github.com/micro-ROS/rmw_microxrcedds/issues/241):
tras varias desconexiones el cliente **no puede reconectar** porque `get_memory()`
en `memory.c` devuelve NULL — se agota el **pool interno de micro-ROS**, que es
distinto del heap del sistema.

Esto explica por qué `heap` y `heap_dma` se veían perfectamente estables mientras
el firmware no lograba reconectar: **estábamos midiendo la memoria equivocada**.

Recomendación: aplicarlo **por separado** del cambio del ping, para poder
atribuir la mejora a uno u otro.
