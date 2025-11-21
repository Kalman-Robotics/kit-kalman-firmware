#!/usr/bin/env python3
"""
PlatformIO pre-script para detectar automáticamente el puerto serial del ESP32.
"""

Import("env")
import serial.tools.list_ports

def find_esp32_port():
    """
    Busca el puerto serial del ESP32 basándose en el VID:PID común.

    VID:PID comunes para ESP32:
    - 10C4:EA60 - CP2102 (Silicon Labs)
    - 1A86:7523 - CH340
    - 0403:6001 - FTDI
    - 303A:1001 - ESP32-S3 native USB
    """

    # VID:PID conocidos para ESP32
    esp32_ids = [
        (0x303A, 0x1001),  # ESP32-S3 native USB
        (0x10C4, 0xEA60),  # CP2102
        (0x1A86, 0x7523),  # CH340
        (0x0403, 0x6001),  # FTDI
    ]

    ports = serial.tools.list_ports.comports()

    # Buscar por VID:PID
    for port in ports:
        if port.vid is not None and port.pid is not None:
            for vid, pid in esp32_ids:
                if port.vid == vid and port.pid == pid:
                    print(f"✓ ESP32 detectado en {port.device} (VID:PID {vid:04X}:{pid:04X})")
                    return port.device

    # Buscar por descripción si no se encontró por VID:PID
    for port in ports:
        description = port.description.lower()
        if any(keyword in description for keyword in ['cp210', 'ch340', 'usb-serial', 'uart', 'esp32']):
            print(f"✓ Dispositivo serial detectado en {port.device}: {port.description}")
            return port.device

    # Si no se encuentra nada específico, mostrar advertencia
    if ports:
        print(f"⚠ No se detectó ESP32, usando primer puerto disponible: {ports[0].device}")
        return ports[0].device

    print("✗ ERROR: No se encontró ningún puerto serial")
    return None

# Auto-detectar puerto si no está definido
if not env.get("UPLOAD_PORT"):
    detected_port = find_esp32_port()
    if detected_port:
        env.Replace(UPLOAD_PORT=detected_port)
        print(f"→ Puerto de carga configurado: {detected_port}")
    else:
        print("⚠ ADVERTENCIA: No se pudo detectar el puerto automáticamente")
        print("  Por favor, define upload_port en platformio.ini")

# Configurar monitor_port si no está definido
if not env.get("MONITOR_PORT"):
    detected_port = find_esp32_port()
    if detected_port:
        env.Replace(MONITOR_PORT=detected_port)
