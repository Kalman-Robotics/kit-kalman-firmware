# Kit-Kalman-Firmware

Este repositorio contiene el firmware del robot Kit-Kalman, basado en ESP32 y compatible con micro-ROS.

## Características principales

- **Plataforma**: ESP32 (compatible con ESP32-DevKitC)
- **Framework**: Arduino + micro-ROS
- **Conectividad**: WiFi (2.4 GHz) y Serial
- **Interfaz web**: Configuración y monitoreo vía navegador
- **Integración ROS2**: Comunicación mediante micro-ROS agent
- **Sistema de archivos**: SPIFFS para almacenar configuraciones y archivos web

## Requisitos previos (Windows)

1. **Instalar PlatformIO**
   - Descarga e instala [Visual Studio Code](https://code.visualstudio.com/)
   - Instala la extensión PlatformIO desde el marketplace de VS Code

2. **Drivers USB**
   - [CP210x USB to UART Bridge](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
   - [CH340 Driver](https://sparks.gogo.co.nz/ch340.html)

3. **Identificar puerto COM**
   - Abre el Administrador de dispositivos
   - Busca en "Puertos (COM y LPT)"
   - Anota el número de puerto (ej: COM3)

## Instalación y uso

### 1. Descargar el firmware
```cmd
cd C:\kalman
git clone https://github.com/kalman-robotics/kit-kalman-firmware.git firmware
cd C:\kalman\firmware
code ./
```

### 2. Configurar archivo config.yaml (ANTES de subir)

**IMPORTANTE**: ESP32 y computadora deben estar en la misma red WiFi.

```
┌─────────────┐          ┌─────────────┐
│   ESP32     │          │ Computadora │
│  (Robot)    │  ◄────►  │  (Docker)   │
└─────────────┘          └─────────────┘
       │                        │
       └────────┬───────────────┘
                │
         ┌──────▼──────┐
         │   Router    │
         │    WiFi     │
         └─────────────┘
```

**Paso 2.1: Obtener la IP de tu computadora**
```cmd
ipconfig
```
Busca la IP en "Adaptador de LAN inalámbrica Wi-Fi" → "Dirección IPv4"
Ejemplo: `192.168.1.100`

**Paso 2.2: Editar `data/config.yaml`**

Cada robot tiene un número de serie. Si es K-01, usar `kalmanbot_01`

```yaml
robot:
  name: kalmanbot_01        # Número de serie del robot
  web: KALMANBOT_01.AI
  use_web: false
  wifi:
    ssid: MiRedWiFi         # Nombre de tu red WiFi
    password: 123456789     # Contraseña de tu WiFi
  computer:
    ip: 192.168.1.100       # IP de tu computadora (del paso 2.1)
    port: 8888              # Puerto micro-ROS (no cambiar)
```

⚠️ **IMPORTANTE**: Cada vez que modifiques `data/config.yaml`, debes repetir los pasos 5 y 6.

### 3. Configurar puerto COM
Edita `platformio.ini` y cambia:
```ini
upload_port = COM3  ; Cambia COM3 por tu puerto
```

### 4. Compilar
```cmd
platformio run
```

### 5. Subir firmware
```cmd
platformio run --target upload
```

### 6. Subir archivos de configuración (SPIFFS)
```cmd
platformio run --target uploadfs
```

### 7. Ver salida serial
```cmd
platformio device monitor
```


