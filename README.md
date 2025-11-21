# Kit-Kalman Firmware

Firmware para ESP32-S3 con micro-ROS y WiFi.

## Requisitos

### Software
1. **Python 3.x** - [Descargar](https://www.python.org/downloads/)
2. **VS Code** - [Descargar](https://code.visualstudio.com/)

### Drivers USB (instalar según tu chip)
- [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (Silicon Labs)
- [CH340](https://sparks.gogo.co.nz/ch340.html)

---

## Instalación

### 1. Clonar repositorio
```cmd
cd C:\kalman
git clone -b web-name https://github.com/kalman-robotics/kit-kalman-firmware.git firmware
cd firmware\lib
git clone https://github.com/Kalman-Robotics/micro_ros_kalman
cd ..
```

### 2. Instalar PlatformIO
```cmd
pip install platformio
```

Reinicia tu terminal después de instalar.

### 3. Configurar WiFi

Edita `data/config.yaml`:

```yaml
robot:
  name: kalmanbot_01        # Cambiar según número de serie (K-01 → kalmanbot_01)
  web: KALMANBOT_01.AI
  use_web: false
  wifi:
    ssid: TU_WIFI           # Nombre de tu red WiFi
    password: TU_PASSWORD   # Contraseña WiFi
  computer:
    ip: 192.168.1.100       # Tu IP (ver con: ipconfig)
    port: 8888
```

**Obtener tu IP:**
```cmd
ipconfig
```
Busca "Dirección IPv4" en "Adaptador de LAN inalámbrica Wi-Fi"

⚠️ **ESP32 y computadora deben estar en la misma red WiFi**

---

## Uso

### Compilar y subir firmware
```cmd
platformio run --target upload
```

### Subir configuración WiFi
```cmd
platformio run --target uploadfs
```

### Monitor serial
```cmd
platformio device monitor
```

**Nota:** El puerto COM se detecta automáticamente. Si falla, verifica que el ESP32 esté conectado y los drivers instalados.

---

## Troubleshooting

**Error: Puerto no detectado**
- Verifica drivers USB instalados
- Revisa Administrador de dispositivos → "Puertos (COM y LPT)"
- Desconecta y reconecta el ESP32

**Error: platformio no reconocido**
- Reinicia el terminal
- Verifica instalación: `pip show platformio`

**Cambios en config.yaml no aplican**
- Debes ejecutar `uploadfs` cada vez que modifiques `data/config.yaml`


