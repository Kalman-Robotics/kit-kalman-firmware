# Kit-Kalman Firmware

Firmware para ESP32-S3 con micro-ROS y WiFi.

<video src="images/test.mp4" controls width="256">
  Tu navegador no soporta el elemento de video.
</video>


https://github.com/user-attachments/assets/7aa0fbdb-eac2-4d8c-a0b1-aa40485893ad


- [Kit-Kalman Firmware](#kit-kalman-firmware)
  - [Requisitos](#requisitos)
    - [Software](#software)
    - [Drivers USB (instalar según tu chip)](#drivers-usb-instalar-según-tu-chip)
  - [Instalación](#instalación)
    - [1. Clonar repositorio y agregar dependencias](#1-clonar-repositorio-y-agregar-dependencias)
    - [2. Instalar PlatformIO](#2-instalar-platformio)
    - [3. Configurar WiFi](#3-configurar-wifi)
      - [Obtner tu IP local](#obtner-tu-ip-local)
    - [4. Configurar platformio.ini](#4-configurar-platformioini)
      - [Obtener el puerto](#obtener-el-puerto)
  - [Uso de PlatformIO para compilar, subir y monitorear](#uso-de-platformio-para-compilar-subir-y-monitorear)
    - [Compilar y subir firmware](#compilar-y-subir-firmware)
    - [Subir configuración config.yaml](#subir-configuración-configyaml)
    - [Monitor serial](#monitor-serial)
  - [Siguientes pasos](#siguientes-pasos)
  - [Troubleshooting](#troubleshooting)
    - [Error: Puerto no detectado](#error-puerto-no-detectado)
    - [Error: platformio no reconocido](#error-platformio-no-reconocido)
    - [Cambios en config.yaml no aplican](#cambios-en-configyaml-no-aplican)


## Requisitos

### Software
1. **Python 3.x** - [Descargar](https://www.python.org/downloads/)
2. **VS Code** - [Descargar](https://code.visualstudio.com/)
3.  **GIT** - [Descargar](https://git-scm.com/downloads)

### Drivers USB (instalar según tu chip)
- [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (Silicon Labs)
- [CH340](https://sparks.gogo.co.nz/ch340.html): utilizada por el Kit-Kalman


## Instalación

### 1. Clonar repositorio y agregar dependencias

<details>
<summary>Windows (PowerShell / CMD)</summary>

```cmd
cd C:\
winget install --id Git.Git -e --source winget
mkdir kalman
cd C:\kalman
git clone https://github.com/kalman-robotics/kit-kalman-firmware.git
cd cd kit-kalman-firmware\lib
```
Incluir la última versión de `micro_ros_kalman` en el folder `lib`:
```
pip install vcstool
```
- Si no está en el proyecto, agregar:
```cmd
vcs import lib < kalman_dependencies.repos
```
- Si ya está en el proyecto, actualizar:
```cmd
rm -rf lib\micro_ros_kalman
vcs import lib < kalman_dependencies.repos
```

</details>

<details>
<summary>Linux (Debian/Ubuntu)</summary>

Abre una terminal y ejecuta:
```bash
git clone https://github.com/tu-usuario/kit-kalman-firmware.git
cd kit-kalman-firmware
```

Incluir la última versión de `micro_ros_kalman` en el folder `lib`:
- Si no está en el proyecto, agregar:
```bash
cd <folder_del_repositorio>
vcs import lib < kalman_dependencies.repos
```
- Si ya está en el proyecto, actualizar:
```bash
cd <folder_del_repositorio>
rm -rf lib/micro_ros_kalman
vcs import lib < kalman_dependencies.repos
```

</details>

### 2. Instalar PlatformIO

1. Abre Visual Studio Code
2. Ve a Extensions (Ctrl+Shift+X)
3. Busca "PlatformIO IDE"
4. Haz click en "Install"
5. Reinicia VSCode si es necesario

<details>
<summary>Windows (PowerShell / CMD)</summary>

```powershell
# PowerShell (recomendado)
python -m pip install --user platformio

# En CMD (si pip está en PATH)
python -m pip install --user platformio
```

</details>

<details>
<summary>Linux</summary>

```bash
# Instalar con Python 3
python3 -m pip install --user platformio

# Si es necesario, añadir ~/.local/bin al PATH
export PATH="$PATH:$HOME/.local/bin"
```
</details>

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

#### Obtner tu IP local

<details>
<summary>Windows (PowerShell / CMD)</summary>

```cmd
ipconfig
```

Busca "Dirección IPv4" en el apartado "Adaptador de LAN inalámbrica Wi-Fi".
</details>

<details>
<summary>Linux</summary>

```bash
ip addr show
# o, para una salida más breve:
hostname -I
```

Busca la dirección "inet" en la interfaz de Wi‑Fi (p. ej. wlan0, wlp2s0) — esa es tu IP local.
</details>

⚠️ ESP32 y computadora deben estar en la misma red WiFi y no debe ser 5G, porque el ESP32-S3 solo soporta 2.4G.


### 4. Configurar platformio.ini
Se debe configurar `upload_port` y `monitor_port` en `platformio.ini` según el puerto asignado al ESP32-S3.

#### Obtener el puerto
<details>
<summary>Windows (PowerShell / CMD)</summary>

- **Alternativa 1**: Abre Administrador de dispositivos → "Puertos (COM y LPT)" y busca CH340/CP210x u otro dispositivo USB serial.
- **Alternativa 2**: Ejecuta `mode` para listar puertos COM disponibles.

Busca el puerto asociado al ESP32 (p. ej. COM3, COM4). Puedes conectar/desconectar para observar cambios.
</details>

<details>
<summary>Linux</summary>

```bash
ls /dev/serial/by-id
# o
ls /dev/ttyUSB* /dev/ttyACM*
```
Busca el dispositivo asociado al ESP32 (p. ej. /dev/ttyUSB0, /dev/ttyACM0). Puedes conectar/desconectar para observar cambios.
</details>

## Uso de PlatformIO para compilar, subir y monitorear

### Compilar y subir firmware
```cmd
platformio run --target upload
```

### Subir configuración config.yaml
```cmd
platformio run --target uploadfs
```

### Monitor serial
Conectar a la entrada derecha vista desde la parte trasera del robot.
```cmd
platformio device monitor
```

**Nota:** El puerto se detecta automáticamente. Si falla, verifica que el ESP32 esté conectado y los drivers instalados.


<img src="images/isometrica.jpeg" alt="isométrica" width="256" />

## Siguientes pasos
Del lado de la computadora:
- Instalar micro-ROS
- Ejecutar el agente de micro-ROS
- Lanzar nodos de ROS2 del Kit-Kalman

## Troubleshooting

### Error: Puerto no detectado

<details>
<summary>Windows (PowerShell / CMD)</summary>

- Abre Administrador de dispositivos → "Puertos (COM y LPT)" y busca CH340/CP210x u otro dispositivo USB serial.
- Ejecuta `mode` para listar puertos COM disponibles.
- Si no aparece: reinstala el driver correspondiente (CH340/CP210x), prueba otro cable/puerto USB y desconecta/reconecta el ESP32.
- Si aparece pero no funciona: desinstala el dispositivo en el Administrador y selecciona "Buscar cambios de hardware".
</details>

<details>
<summary>Linux</summary>

- Conecta el ESP32 y comprueba con `dmesg | tail -n 20` o `dmesg | grep -i usb` para ver la asignación del dispositivo.
- Lista dispositivos: `ls /dev/ttyUSB* /dev/ttyACM*`
- Si el dispositivo no tiene permisos: añade tu usuario al grupo dialout `sudo usermod -a -G dialout $USER` y vuelve a iniciar sesión (o `newgrp dialout`).
- Si no aparece: prueba otro cable/puerto o revisa `sudo dmesg -w` al conectar para detectar errores.
</details>

### Error: platformio no reconocido

<details>
<summary>Windows (PowerShell / CMD)</summary>

- Reinicia el terminal.
- Verifica: `pip show platformio`
- Si no está instalado: `python -m pip install --user platformio`
- Asegura que la carpeta de scripts de usuario esté en PATH (p. ej. `%USERPROFILE%\AppData\Roaming\Python\PythonXX\Scripts`); reinicia el terminal después de ajustar el PATH.
- Alternativa: ejecutar con Python: `python -m platformio --version`.
</details>

<details>
<summary>Linux</summary>

- Reinicia el terminal.
- Instala (si falta): `python3 -m pip install --user platformio`
- Asegura que `~/.local/bin` esté en tu PATH (`export PATH=$PATH:~/.local/bin` o añadir al .bashrc/.zshrc).
- Comprueba con `platformio --version` o `python3 -m platformio --version`.
</details>

### Cambios en config.yaml no aplican

<details>
<summary>Windows (PowerShell / CMD)</summary>

- Después de editar `data/config.yaml` ejecuta: `platformio run --target uploadfs`
- Si usas otros entornos, especifica: `platformio run -e <env> --target uploadfs`
- Verifica que estés editando el `data/config.yaml` correcto en la raíz del proyecto y revisa la salida del comando por errores.
- Si continúa sin aplicarse, reinicia el ESP32 tras el uploadfs.
</details>

<details>
<summary>Linux</summary>

- Ejecuta: `platformio run --target uploadfs` (o `platformio run -e <env> --target uploadfs` si usas un entorno concreto).
- Asegura permisos de acceso al puerto serie (`/dev/ttyUSB0`/`/dev/ttyACM0`) y que tu usuario esté en el grupo dialout.
- Confirma que el `data/config.yaml` modificado es el del proyecto y revisa la salida del comando para errores. Reinicia el dispositivo si hace falta.
</details>
