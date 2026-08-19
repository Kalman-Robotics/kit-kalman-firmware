#!/usr/bin/env python3
"""Captura los reportes de la prueba de estabilidad de WiFi del ESP32.

Se ejecuta en la Raspberry (o en cualquier maquina de la red del robot) durante
toda la prueba. Escucha el puerto UDP 8890, muestra cada reporte y lo guarda en
un archivo para poder revisarlo despues.

Uso:
    ./wifi_monitor.py                      # escucha y loguea a wifi_test.log
    ./wifi_monitor.py -o corrida1.log      # otro archivo de salida
    ./wifi_monitor.py --cmd STATUS         # pide el estado y termina
    ./wifi_monitor.py --cmd QUIET          # pasa a modo sin heartbeat
    ./wifi_monitor.py --cmd LOUD           # vuelve al heartbeat cada 30 s
    ./wifi_monitor.py --cmd RESET          # pone los contadores a cero

Para la prueba de dos fases:
    por la manana   ./wifi_monitor.py -o fase_a.log        (modo LOUD)
    al mediodia     ./wifi_monitor.py --cmd QUIET
    por la tarde    ./wifi_monitor.py -o fase_b.log
    comparar drops y uptime_pct de cada fase
"""

import argparse
import json
import socket
import sys
import time
from datetime import datetime

DEFAULT_PORT = 8890
DEFAULT_BROADCAST = "192.168.4.255"
DEFAULT_LOG = "wifi_test.log"
CMD_REPEAT = 3
CMD_GAP_S = 0.2
CMD_WAIT_S = 2.0


def stamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def fmt_hms(seconds: int) -> str:
    h, rem = divmod(int(seconds), 3600)
    m, s = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


def describe(rec: dict) -> str:
    """Una linea legible por reporte."""
    ev = rec.get("ev", "?")
    up = fmt_hms(rec.get("up_s", 0))
    drops = rec.get("drops", 0)
    pct = rec.get("uptime_pct", 0)
    rssi = rec.get("rssi", 0)
    rssi_min = rec.get("rssi_min", 0)
    max_down = rec.get("max_down_s", 0)

    line = (f"[{stamp()}] {ev:<10} up={up} drops={drops} "
            f"uptime={pct}% rssi={rssi}dBm(min {rssi_min})")
    if max_down:
        line += f" peor_caida={max_down}s"
    if rec.get("ip_changes"):
        line += f" ip_changes={rec['ip_changes']}"
    if ev == "up" and rec.get("last_down_s"):
        line += f"  <-- se recupero tras {rec['last_down_s']}s"
    return line


def send_command(cmd: str, addr: str, port: int) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.settimeout(0.2)

    for i in range(CMD_REPEAT):
        sock.sendto(cmd.encode(), (addr, port))
        print(f"-> {cmd} ({i + 1}/{CMD_REPEAT})")
        if i < CMD_REPEAT - 1:
            time.sleep(CMD_GAP_S)

    got = 0
    deadline = time.time() + CMD_WAIT_S
    while time.time() < deadline:
        try:
            data, src = sock.recvfrom(512)
        except socket.timeout:
            continue
        text = data.decode(errors="replace").strip()
        if not text.startswith("{"):
            continue
        try:
            rec = json.loads(text)
        except json.JSONDecodeError:
            continue
        print(f"<- {src[0]}: {describe(rec)}")
        got += 1

    sock.close()
    if not got:
        print("ERROR: el robot no respondio", file=sys.stderr)
        return 1
    return 0


def listen(port: int, log_path: str) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))

    print(f"Escuchando UDP {port}. Log: {log_path}")
    print("Ctrl-C para terminar.\n")

    last_seen = None
    count = 0

    with open(log_path, "a", encoding="utf-8") as log:
        log.write(f"\n=== inicio de captura {stamp()} ===\n")
        log.flush()
        try:
            while True:
                data, src = sock.recvfrom(512)
                text = data.decode(errors="replace").strip()
                if not text.startswith("{"):
                    continue
                try:
                    rec = json.loads(text)
                except json.JSONDecodeError:
                    continue

                count += 1
                last_seen = time.time()
                line = describe(rec)
                print(line)
                log.write(line + "\n")
                log.write("    " + text + "\n")
                log.flush()   # el robot puede quedarse todo el dia: no bufferear

        except KeyboardInterrupt:
            gap = f"{time.time() - last_seen:.0f}s" if last_seen else "nunca"
            summary = (f"\n=== fin de captura {stamp()} — "
                       f"{count} reportes, ultimo hace {gap} ===")
            print(summary)
            log.write(summary + "\n")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cmd", choices=["STATUS", "QUIET", "LOUD", "RESET"],
                    help="envia un comando al robot y termina")
    ap.add_argument("-o", "--out", default=DEFAULT_LOG,
                    help=f"archivo de log (default {DEFAULT_LOG})")
    ap.add_argument("--addr", default=DEFAULT_BROADCAST,
                    help=f"broadcast para comandos (default {DEFAULT_BROADCAST})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"puerto UDP (default {DEFAULT_PORT})")
    args = ap.parse_args()

    if args.cmd:
        return send_command(args.cmd, args.addr, args.port)
    return listen(args.port, args.out)


if __name__ == "__main__":
    sys.exit(main())
