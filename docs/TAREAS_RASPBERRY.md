# Tareas del lado de la Raspberry Pi

Contexto para trabajar en la Raspberry. El firmware del ESP32 ya está resuelto
en el repo `kit-kalman-firmware`; lo que sigue es lo que falta hacer del lado
del host.

---

## 1. Arquitectura (para entender el resto)

```
┌──────────────────────────────────────────────────────────────┐
│ AWS                                                          │
│   Backend de sesiones                                        │
└───────────────────────┬──────────────────────────────────────┘
                        │ Husarnet (VPN overlay)
                        │ SOLO Raspberry ↔ AWS ↔ alumno
┌───────────────────────┴──────────────────────────────────────┐
│ Robot                                                        │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ Raspberry Pi                                           │  │
│  │  - AP WiFi "kalman-robot", IP 192.168.4.1              │  │
│  │  - Docker + micro-ROS agent (UDP 8888)                 │  │
│  │  - Husarnet (hacia AWS y el alumno)                    │  │
│  └────────────────────────┬───────────────────────────────┘  │
│                           │ WiFi local 192.168.4.x           │
│                           │ SIN Husarnet                     │
│  ┌────────────────────────┴───────────────────────────────┐  │
│  │ ESP32-S3   IP por DHCP (ej. 192.168.4.48)              │  │
│  │  - cliente micro-ROS (XRCE-DDS sobre UDP)              │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

**Clave: el ESP32 nunca ve Husarnet ni DDS.** Habla XRCE-DDS por UDP contra
`192.168.4.1:8888`. No participa del descubrimiento DDS. Todo el problema de
peers y XML es exclusivamente del lado de la Raspberry.

### Flujo de una sesión

```
1. Estudiante agenda
2. Estudiante inicia sesión
3. Se crea el grupo Husarnet (Raspberry + AWS)
4. Se añaden robot y AWS al grupo
5. Arranca el contenedor Docker con micro-ROS   ← recién acá existe el agente
```

El robot está encendido 24/7 con fuente externa. Entre el encendido y el paso 5
pueden pasar horas.

---

## 2. Problema abierto: el XML de Fast DDS se rompe con peers no resueltos

### Síntoma

Al agregar un alumno nuevo al grupo Husarnet hay que regenerar el XML de peers y
**reiniciar todo el stack**, lo que corta la sesión del ESP32.

Peor: si se lanza el XML con un peer cuyo hostname todavía no resuelve (el alumno
aún no se conectó), Fast DDS **falla el parseo del XML completo** y cae al perfil
por defecto — sin peers, intentando multicast, que sobre Husarnet no funciona.

Cyclone DDS no tiene este problema (ignora los peers no resueltos y carga el
resto), pero no se logró hacerlo funcionar en este setup.

### Opciones, de mejor a peor

**A) Discovery Server (recomendado)** — elimina la lista de peers por completo.
No hay XML que pueda romperse; el alumno se conecta al servidor y este lo
anuncia. Es el único enfoque donde el problema desaparece en vez de mitigarse.

```bash
# En la Raspberry, una sola vez:
fastdds discovery -i 0 -l <hostname-husarnet-raspberry> -p 11811

# El agente micro-ROS:
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<hostname-husarnet-raspberry>:11811
export ROS_DOMAIN_ID=20
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# El alumno, las mismas variables:
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<hostname-husarnet-raspberry>:11811
export ROS_SUPER_CLIENT=TRUE     # para que ros2 topic list / rqt vean todo
export ROS_DOMAIN_ID=20
```

micro-ROS es compatible: el agente usa Fast DDS internamente.

**B) IPv6 literal en vez de hostname (arreglo rápido)** — Husarnet asigna a cada
peer una IPv6 estable. Una IPv6 literal no necesita resolución DNS, así que no
puede romper el parseo:

```xml
<address>fc94:a67f:2b47:756c:6e1c:7c05:7361:7378</address>
<!-- en vez de: <address>alumno-host</address> -->
```

El peer queda simplemente inalcanzable hasta que el alumno se conecta — que es el
comportamiento de Cyclone que se busca.

**C) Pre-poblar `/etc/hosts`** del contenedor para que los hostnames siempre
resuelvan. Frágil y hay que mantenerlo.

### Importante

`ROS_DOMAIN_ID=20` — está en `data/config.yaml` del firmware. El alumno y el
agente tienen que usar el mismo dominio.

---

## 3. Pendiente: quitar el reinicio de 55 minutos

Hay un reinicio programado cada 55 min del lado de la Raspberry, puesto como
parche porque el ESP32 dejaba de aceptar `cmd_vel` pasada ~1 hora.

**La causa raíz ya fue encontrada y corregida en el firmware.** No era Fast DDS
ni micro-ROS: `esp_timer_get_time()` devuelve `int64_t` y el firmware lo guardaba
en `unsigned long` (32 bits en ESP32), que desborda a los **71.58 minutos**. Tras
el desbordamiento el cálculo del timeout de `cmd_vel` daba un número enorme y los
motores quedaban frenados hasta el siguiente comando.

El firmware ahora usa `int64_t` en todas las variables de tiempo. Desborde nuevo:
~292.000 años.

**Antes de quitar el reinicio de 55 min**, validar: dejar el robot recibiendo
comandos espaciados (cada 5 min) durante más de 72 minutos y confirmar que a los
~71.6 min sigue respondiendo. Si pasa esa marca, el bug está cerrado y las
sesiones pueden durar lo que se necesite.

---

## 4. Scripts que ya existen en el repo del firmware

### `scripts/wifi_monitor.py` — prueba de estabilidad de WiFi (en curso)

Captura los reportes UDP del firmware de diagnóstico `wifi-test` que corre en el
ESP32. Puerto **8890**.

```bash
python3 wifi_monitor.py -o fase_a.log      # escucha y loguea
python3 wifi_monitor.py --cmd STATUS       # consulta puntual
python3 wifi_monitor.py --cmd QUIET        # modo sin heartbeat
python3 wifi_monitor.py --cmd LOUD         # heartbeat cada 30 s
python3 wifi_monitor.py --cmd RESET        # contadores a cero
```

Prueba de dos fases (para descartar que el heartbeat mantenga viva la conexión
artificialmente):
- Mañana en `LOUD` → mide con keep-alive
- Tarde en `QUIET` → mide sin tráfico de salida
- Comparar `drops` y `uptime_pct` de cada fase

### `scripts/session_notify.py` — control de sesión (implementado, sin probar)

Avisa al ESP32 el estado de la sesión. Puerto **8889**.

```bash
python3 session_notify.py start   # tras levantar el agente micro-ROS
python3 session_notify.py end     # al terminar la sesión
python3 session_notify.py ping    # diagnóstico
```

Repite el broadcast 3 veces (UDP no garantiza entrega) y espera los ACK del
robot. Sale con código 1 si ningún robot responde — sirve directo en un script de
arranque para detectar que el robot no está.

Respuesta del ESP32:
```
ACK SESSION_START 192.168.4.48 IDLE 3421
    └ evento     └ IP robot   └ estado └ uptime_s
```

**Integración pendiente en el contenedor Docker:**
- `session_notify.py start` — después de que el agente esté escuchando
- `session_notify.py end` — en el shutdown del contenedor

---

## 5. Máquina de estados del ESP32 (para entender los ACK)

| Estado | Significado | LED sistema (GPIO 11) |
|---|---|---|
| `ACTIVE` | Sesión en curso, agente conectado | apagado |
| `GRACE` | Agente perdido, reintenta 2 min | parpadeo rápido |
| `IDLE` | Sin sesión, esperando aviso | destello cada 3 s |

Lógica: al perder el agente el ESP32 entra en `GRACE` y espera 2 minutos por si
fue un corte de red o un reinicio del contenedor. Si la Raspberry manda
`SESSION_END`, corta la espera de inmediato y vuelve a `IDLE`.

Esto resuelve la ambigüedad de fondo: por red, "se cayó la red" y "terminó la
sesión" se ven idénticos. Solo la Raspberry sabe la diferencia.

---

## 6. Orden sugerido de trabajo

1. **En curso:** prueba de estabilidad de WiFi (todo el día, con `wifi_monitor.py`)
2. Validar el fix del wraparound: >72 min con comandos espaciados
3. Resolver el XML de Fast DDS — opción B (IPv6 literal) como arreglo rápido,
   opción A (Discovery Server) como solución definitiva
4. Integrar `session_notify.py` en el arranque/parada del contenedor
5. Quitar el reinicio de 55 min una vez validado el punto 2

---

## Referencias

- [ROS_DISCOVERY_SERVER con Husarnet](https://husarnet.com/docs/ros2/ros-discovery-server-env)
- [Fast DDS Discovery Server — Husarnet](https://husarnet.com/blog/ros2-dds-discovery-server)
- [XML custom de Fast DDS — Husarnet](https://husarnet.com/docs/ros2/custom-fastdds-xml)
- [micro-ROS y Discovery Server — Vulcanexus](https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/discovery_server/discovery_server.html)
