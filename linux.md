# Kit-Kalman-Firmware - Guía para Linux

Esta guía describe los pasos para configurar, compilar y subir el firmware del robot Kit-Kalman en Linux (probado en Ubuntu 22.04).

## Requisitos previos
- Linux (Ubuntu 22.04 o superior recomendado)
- Visual Studio Code
- Python 3.7 o superior
- Git
- Permisos de acceso al puerto serie (USB)

## Instalación

### 1. Clonar el repositorio
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

### 2. Instalar la extensión de PlatformIO en VSCode
1. Abre Visual Studio Code
2. Ve a Extensions (Ctrl+Shift+X)
3. Busca "PlatformIO IDE"
4. Haz click en "Install"
5. Reinicia VSCode si es necesario

Alternativamente, puedes instalar PlatformIO desde la terminal:
```bash
pip3 install platformio
```

### 3. Configurar permisos para el puerto serie
Para poder acceder al puerto USB sin necesidad de usar `sudo`, añade tu usuario al grupo `dialout`:
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

> [!IMPORTANT]
> Después de ejecutar estos comandos, debes cerrar sesión y volver a iniciarla para que los cambios surtan efecto.

### 4. Abrir el proyecto con PlatformIO
1. En VSCode, ve a File → Open Folder
2. Selecciona la carpeta `kit-kalman-firmware`
3. PlatformIO detectará automáticamente el proyecto

## Compilación y Carga

### 5. Compilar el proyecto
Haz click en el icono "check" (✓) de PlatformIO en la barra inferior de VSCode o ejecuta en la terminal:
```bash
platformio run
```

### 6. Identificar el puerto serie
Conecta el robot por USB y ejecuta uno de los siguientes comandos para identificar el puerto:

```bash
# Listar todos los dispositivos USB conectados
ls /dev/ttyUSB*
ls /dev/ttyACM*

# O usar dmesg para ver el último dispositivo conectado
dmesg | grep tty
```

Normalmente el puerto será `/dev/ttyUSB0` o `/dev/ttyACM0`.

### 7. Configurar el puerto serie en platformio.ini
Edita el archivo `platformio.ini` y configura el puerto encontrado:
```ini
upload_port = /dev/ttyUSB0    ; Reemplaza con tu puerto
monitor_port = /dev/ttyUSB0   ; Usa el mismo puerto
```

### 8. Subir el Firmware al robot
Haz click en el icono "flecha derecha" (→) de PlatformIO en la barra inferior de VSCode o ejecuta:
```bash
platformio run --target upload
```

> [!TIP]
> Si obtienes un error de permisos, verifica que hayas añadido tu usuario al grupo `dialout` y reiniciado la sesión.

### 9. Subir el sistema de archivos SPIFFS
Para subir el contenido de la carpeta `data/` (archivos de configuración y web), ejecuta:
```bash
platformio run --target uploadfs
```

> [!NOTE]
> Este paso es necesario para que la interfaz web y los archivos de configuración funcionen correctamente.

## Configuración y Uso

### 10. Monitorizar la salida serial
Haz click en el icono "enchufe" (🔌) de PlatformIO en la barra inferior de VSCode o ejecuta:
```bash
platformio device monitor
```

Si no aparece nada, presiona el botón de reset en el robot y sigue los pasos que se indican en la salida serial.

> [!TIP]
> Para salir del monitor serial, presiona Ctrl+C

Alternativamente, puedes usar `minicom` o `screen`:
```bash
# Usando screen
screen /dev/ttyUSB0 115200

# Usando minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### 11. Configurar la conexión WiFi del robot
1. Cambia a la red WiFi del robot (debería aparecer como "Kit-Kalman-XXXX")
2. Abre el navegador web y accede a:
   ```
   http://192.168.4.1/
   ```
3. Introduce los siguientes datos en la interfaz web:
   - **SSID**: Nombre de tu red WiFi
   - **Password**: Contraseña de tu red WiFi
   - **IP de la computadora**: Tu dirección IP local
4. El robot se reiniciará y se conectará a la red WiFi configurada

#### Obtener tu dirección IP en Linux
Puedes obtener tu dirección IP ejecutando:
```bash
# Para redes WiFi
ip addr show wlan0 | grep "inet " | awk '{print $2}' | cut -d/ -f1

# Para redes Ethernet
ip addr show eth0 | grep "inet " | awk '{print $2}' | cut -d/ -f1

# O de forma más general
hostname -I
```

> [!IMPORTANT]
> No conectes a redes WiFi de 5 GHz. El módulo WiFi del ESP32 solo soporta 2.4 GHz.

### 12. Instalar ROS2 (si aún no lo tienes)
Si no tienes ROS2 instalado, sigue la guía oficial para Ubuntu:
```bash
# Configurar UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Añadir el repositorio de ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2 Humble (recomendado para Ubuntu 22.04)
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 13. Instalar micro-ROS agent
```bash
# Cargar el entorno de ROS2
source /opt/ros/humble/setup.bash

# Instalar micro-ROS agent
sudo apt install ros-humble-micro-ros-agent
```

### 14. Ejecutar el agente micro-ROS
```bash
# Cargar el entorno de ROS2
source /opt/ros/humble/setup.bash

# Ejecutar el agente (reemplaza con tu IP)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

> [!TIP]
> Puedes añadir `source /opt/ros/humble/setup.bash` a tu archivo `~/.bashrc` para que se cargue automáticamente:
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> ```

### 15. Verificar la conexión
Posteriormente puedes listar los tópicos disponibles del robot:
```bash
ros2 topic list
```

Para ver los datos de un tópico específico:
```bash
ros2 topic echo /nombre_del_topico
```

## Solución de problemas comunes

### Error de permisos al acceder al puerto serie
```bash
# Verificar que estás en el grupo dialout
groups | grep dialout

# Si no aparece, añadirlo de nuevo
sudo usermod -a -G dialout $USER

# Reiniciar la sesión o ejecutar
newgrp dialout
```

### El puerto /dev/ttyUSB0 no aparece
- Verifica que el cable USB transmita datos (no solo carga)
- Comprueba con `dmesg | tail` qué dispositivo se detecta al conectar
- Puede que aparezca como `/dev/ttyACM0` en lugar de `/dev/ttyUSB0`
- Verifica que el driver CH340 o CP210x esté instalado:
  ```bash
  lsmod | grep ch341
  lsmod | grep cp210x
  ```

### El robot no se conecta al WiFi
- Verifica que tu red sea de 2.4 GHz (no 5 GHz)
- Asegúrate de haber ingresado correctamente el SSID y la contraseña
- Comprueba que la IP de tu computadora sea correcta
- Verifica que el firewall no bloquee el puerto 8888

### Error al compilar
- Asegúrate de tener Python 3.7 o superior instalado
- Elimina la carpeta `.pio` y vuelve a compilar:
  ```bash
  rm -rf .pio
  platformio run
  ```
- Verifica que tengas conexión a internet para que PlatformIO descargue las dependencias

### Problemas de red con micro-ROS
```bash
# Verificar que el puerto 8888 esté abierto
sudo ufw allow 8888/udp

# Verificar conectividad
ping <IP_DEL_ROBOT>

# Ver logs del agente con más detalle
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## Comandos útiles

### Limpiar y recompilar el proyecto
```bash
platformio run --target clean
platformio run
```

### Ver información del dispositivo conectado
```bash
platformio device list
```

### Borrar completamente la flash del ESP32
```bash
platformio run --target erase
```

### Actualizar PlatformIO
```bash
platformio upgrade
platformio pkg update
```

## Enlaces útiles
- [Documentación de PlatformIO](https://docs.platformio.org/)
- [micro-ROS para ESP32](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Guía general del proyecto](README.md)
