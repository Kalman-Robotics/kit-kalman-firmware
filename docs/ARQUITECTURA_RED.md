# Arquitectura de red y ciclo de vida de la sesión

Documento de referencia para entender dónde vive cada componente y por qué el
firmware del ESP32 se comporta como se comporta. Escrito a partir del flujo real
de los laboratorios remotos.

## Topología

```
┌─────────────────────────────────────────────────────────────────┐
│                                                    AWS (nube)    │
│                                        ┌──────────────────────┐  │
│                                        │  Backend / sesiones  │  │
│                                        └──────────┬───────────┘  │
└───────────────────────────────────────────────────┼──────────────┘
                                                    │
                                        Husarnet (VPN overlay)
                                     ── SOLO Raspberry ↔ AWS ──
                                                    │
┌───────────────────────────────────────────────────┼──────────────┐
│ Robot                                             │              │
│                                     ┌─────────────┴───────────┐  │
│                                     │  Raspberry Pi           │  │
│                                     │  - AP WiFi "kalman-robot"│ │
│                                     │  - 192.168.4.1          │  │
│                                     │  - Docker + micro-ROS   │  │
│                                     │    agent (UDP 8888)     │  │
│                                     └─────────────┬───────────┘  │
│                                                   │              │
│                                        WiFi local (192.168.4.x)  │
│                                        SIN Husarnet              │
│                                                   │              │
│                                     ┌─────────────┴───────────┐  │
│                                     │  ESP32-S3               │  │
│                                     │  - cliente WiFi (STA)   │  │
│                                     │  - IP por DHCP (.48)    │  │
│                                     │  - micro-ROS client     │  │
│                                     └─────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### Puntos clave

- **El ESP32 nunca ve Husarnet.** Solo habla con `192.168.4.1` por la LAN que
  abre la Raspberry. La VPN es un asunto exclusivo de Raspberry ↔ AWS.
- **La IP del agente es fija y conocida**: `192.168.4.1:8888`, la Raspberry como
  AP. No cambia entre sesiones.
- **No hay pines entre Raspberry y ESP32.** Están unidos solo por WiFi, así que
  cualquier señal de control tiene que viajar por la red.
- **El ESP32 toma IP por DHCP** del AP de la Raspberry (se probó `.48`). La IP
  estática está implementada pero desactivada: dio problemas del lado de la
  Raspberry (ver `data/config.yaml`).

## Ciclo de vida de una sesión de laboratorio

```
1. Estudiante agenda
2. Estudiante inicia sesión
3. Se crea el grupo Husarnet (Raspberry + AWS)
4. Se añaden robot y AWS al grupo
5. Arranca el contenedor Docker en la Raspberry con micro-ROS
   ▲
   └── RECIÉN ACÁ existe el agente que el ESP32 busca
```

### La consecuencia para el firmware

Entre que el robot se enciende y el paso 5 puede pasar **mucho tiempo**: el robot
puede estar encendido a la mañana y la sesión agendarse para la tarde.

El firmware fue escrito asumiendo que **el agente ya existe al arrancar**. Ese
supuesto no se cumple en este flujo, y de ahí salen los problemas observados.

## Estado actual del firmware frente a este flujo

| Comportamiento | Constante | Efecto en el flujo real |
|---|---|---|
| Busca el agente al arrancar, en bucle | — | Correcto solo si el agente ya está |
| Timeout de 60 s y reinicio | `UROS_AGENT_CONN_TIMEOUT_MS` | **Ciclo de reinicios cada ~68 s mientras espera la sesión** |
| Ping cada 1 s, 5 fallos → reinicio | `UROS_PING_MAX_FAILS` | Correcto: detecta caída del agente en operación |
| Reconexión WiFi, 30 s → reinicio | `WIFI_RECONNECT_TIMEOUT_MS` | Correcto: la Raspberry puede reiniciar su AP |

El punto conflictivo es el segundo. Esperando al estudiante, el robot entra en:

```
buscar 60 s → reiniciar → bootear ~8 s → buscar 60 s → reiniciar → ...
```

Funciona (cuando el agente aparece, el ciclo siguiente lo toma), pero desgasta
flash, hace ruido en la red y no distingue "no hay sesión todavía" de "algo está
roto".

## Camino propuesto: aviso por WiFi

Como no hay pines, la Raspberry tiene que avisarle al ESP32 por la red. Opciones,
de menor a mayor complejidad:

1. **Espera pasiva con backoff.** El ESP32 sigue sondeando al agente pero cada
   vez más espaciado (1 s → 2 s → 5 s → 30 s) y **sin reiniciar**. No requiere
   ningún cambio del lado de la Raspberry. Es el punto de partida más seguro.
2. **Broadcast UDP desde la Raspberry.** Al levantar el contenedor, la Raspberry
   emite un datagrama a `192.168.4.255:<puerto>`; el ESP32 escucha y conecta al
   recibirlo. Requiere un script en el arranque del contenedor.
3. **Endpoint HTTP en el ESP32.** La Raspberry hace un POST cuando el agente está
   listo. Más control, pero más código y más superficie de fallo.

La 1 y la 2 se combinan bien: backoff como base, broadcast para que la conexión
sea inmediata cuando la sesión arranca.

## Qué hay que decidir todavía

- ¿El robot queda encendido esperando, o se enciende como parte de la sesión?
  Si es lo segundo, el problema casi desaparece y alcanza con subir el timeout.
- ¿Se puede tocar el arranque del contenedor en la Raspberry para emitir el
  aviso? Eso habilita la opción 2.
- ¿Qué debe hacer el robot si la sesión termina? Hoy el ping detecta la caída del
  agente y reinicia, lo cual lo deja listo para la sesión siguiente.
