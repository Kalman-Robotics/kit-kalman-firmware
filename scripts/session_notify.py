#!/usr/bin/env python3
"""Avisa al ESP32 el estado de la sesion de laboratorio.

Se ejecuta en la Raspberry, que es el AP de la red del robot. El ESP32 escucha
en UDP 8889 y responde un ACK, lo que permite verificar que el aviso llego y en
que estado quedo el robot.

Uso:
    ./session_notify.py start      # el agente micro-ROS ya esta listo
    ./session_notify.py end        # la sesion termino, el robot deja de buscar
    ./session_notify.py ping       # sonda de diagnostico, no cambia el estado

Integracion tipica con el contenedor:
    ./session_notify.py start   # despues de que el agente este escuchando
    ./session_notify.py end     # en el shutdown del contenedor

Salida: imprime cada ACK recibido y termina con codigo 0 si hubo al menos uno,
1 si ningun robot respondio.
"""

import argparse
import socket
import sys
import time

DEFAULT_BROADCAST = "192.168.4.255"
DEFAULT_PORT = 8889
# UDP no garantiza entrega: repetir es mas barato que perder un aviso
REPEAT = 3
REPEAT_GAP_S = 0.2
ACK_WAIT_S = 1.5

EVENTS = {
    "start": "SESSION_START",
    "end": "SESSION_END",
    "ping": "PING",
}


def notify(event: str, addr: str, port: int) -> dict:
    """Emite el evento y junta los ACK. Devuelve {ip: texto_del_ack}."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Enlazar al mismo puerto: el ESP32 responde al puerto de origen
    sock.bind(("", port))
    sock.settimeout(0.2)

    acks = {}
    payload = event.encode()

    for i in range(REPEAT):
        sock.sendto(payload, (addr, port))
        print(f"-> {event} a {addr}:{port} ({i + 1}/{REPEAT})")
        if i < REPEAT - 1:
            time.sleep(REPEAT_GAP_S)

    deadline = time.time() + ACK_WAIT_S
    while time.time() < deadline:
        try:
            data, src = sock.recvfrom(256)
        except socket.timeout:
            continue
        text = data.decode(errors="replace").strip()
        if not text.startswith("ACK"):
            continue  # es el eco de nuestro propio broadcast
        if src[0] not in acks:
            acks[src[0]] = text
            print(f"<- {src[0]}: {text}")

    sock.close()
    return acks


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("event", choices=sorted(EVENTS))
    ap.add_argument("--addr", default=DEFAULT_BROADCAST,
                    help=f"destino broadcast (default {DEFAULT_BROADCAST})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"puerto UDP (default {DEFAULT_PORT})")
    args = ap.parse_args()

    acks = notify(EVENTS[args.event], args.addr, args.port)

    if not acks:
        print("ERROR: ningun robot respondio", file=sys.stderr)
        return 1

    print(f"\n{len(acks)} robot(s) confirmaron el aviso")
    return 0


if __name__ == "__main__":
    sys.exit(main())
