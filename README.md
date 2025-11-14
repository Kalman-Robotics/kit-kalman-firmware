# NOMBRE
Este repositorio contiene el Firmware del robot

- [NOMBRE](#nombre)
  - [Pasos probados en Ubuntu 22.04:](#pasos-probados-en-ubuntu-2204)
    - [1. Clonar el repositorio](#1-clonar-el-repositorio)
    - [2. Instalar la extensión de PlatformIO en VSCode](#2-instalar-la-extensión-de-platformio-en-vscode)
    - [3. Abrir el proyecto con PlatformIO](#3-abrir-el-proyecto-con-platformio)
    - [4. Compilar el proyecto](#4-compilar-el-proyecto)
    - [5. Subir el Firmware al robot](#5-subir-el-firmware-al-robot)
    - [6. Subir el sistema de archivos SPIFFS](#6-subir-el-sistema-de-archivos-spiffs)
    - [7. Monitorizar la salida serial](#7-monitorizar-la-salida-serial)
    - [8. Configurar la conexión Wifi del robot](#8-configurar-la-conexión-wifi-del-robot)
    - [9. Ejecutar el agente micro-ROS en la computadora](#9-ejecutar-el-agente-micro-ros-en-la-computadora)

## Pasos probados en Ubuntu 22.04:
### 1. Clonar el repositorio
### 2. Instalar la extensión de PlatformIO en VSCode
### 3. Abrir el proyecto con PlatformIO
### 4. Compilar el proyecto
Hacer click en el icono "check" de PlatformIO en la barra inferior de VSCode o ejecutar:
```
platformio run
```
### 5. Subir el Firmware al robot
Es necesario modificar el archivo `platformio.ini` y setear correctamente:
- `upload_port` con el puerto serie del robot conectado por USB (ejemplo: `/dev/ttyUSB0` en Linux).
- `monitor_port` con el mismo puerto serie del robot.

A continuación, hacer click en el icono "flecha derecha" de PlatformIO en la barra inferior de VSCode o ejecutar:
```
platformio run --target upload
```
### 6. Subir el sistema de archivos SPIFFS
Para subir el contenido de la carptes `data/`, ejecutar el comando:
```
platformio run --target uploadfs
```
### 7. Monitorizar la salida serial
Hacer click en el icono "enchufe" de PlatformIO en la barra inferior de VSCode o ejecutar:
```
platformio device monitor
```
Si no aparece nada, presionar el botón de reset en el robot y seguir los pasos que se indican en la salida serial.
### 8. Configurar la conexión Wifi del robot
- Cambiarse a la red Wi‑Fi del robot 
- Abrir el navegador web y acceder a la dirección que indica la terminal serial:
```
https://192.168.4.1/
```
- Introducir los siguientes datos:
  - Nombre de red Wi-Fi (SSID)
  - Contraseña de red Wi-Fi (Password)
  - IP de la computadora
- Automáticamente el robot y la computadora se conectarán a la red Wi‑Fi configurada.

>> [!Note] Importante: No conectar a redes Wi‑Fi de 5 GHz. El módulo Wi‑Fi del robot solo soporta 2.4 GHz.

### 9. Ejecutar el agente micro-ROS en la computadora
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i <IP_DE_LA_COMPUTADORA>
```
Posteriormente se puede listar los tópicos disponibles del robot.