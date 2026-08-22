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


# Motivos de reset que indican un fallo, no un reinicio ordenado
RESET_ALARMING = {"panic", "task_wdt", "int_wdt", "other_wdt", "brownout"}


def describe(rec: dict) -> str:
    """Una linea legible por reporte."""
    ev = rec.get("ev", "?")
    up = fmt_hms(rec.get("up_s", 0))
    rssi = rec.get("rssi", 0)

    line = f"[{stamp()}] {ev:<10} up={up} rssi={rssi}dBm"

    # --- prueba de WiFi (puerto 8890) ---
    if "uptime_pct" in rec:
        line += (f" drops={rec.get('drops', 0)}"
                 f" uptime={rec['uptime_pct']}%"
                 f"(min {rec.get('rssi_min', 0)})")
        if rec.get("max_down_s"):
            line += f" peor_caida={rec['max_down_s']}s"
        if rec.get("ip_changes"):
            line += f" ip_changes={rec['ip_changes']}"
        if ev == "up" and rec.get("last_down_s"):
            line += f"  <-- se recupero tras {rec['last_down_s']}s"

    # --- diagnostico del firmware (8891) / prueba aislada (8892) ---
    if "cmd_vel_rx" in rec:
        line += f" cmd_vel={rec['cmd_vel_rx']}"
    if "agent_lost" in rec:
        line += f" agent_lost={rec['agent_lost']}"
    if rec.get("ping_fails"):
        line += f" ping_fails={rec['ping_fails']}"
    if rec.get("restarts_skipped"):
        line += f" restarts_skipped={rec['restarts_skipped']}"

    # --- forense: lo que dice por que se reinicio ---
    rst = rec.get("rst_name")
    if rst:
        mark = "  <== REINICIO ANOMALO" if rst in RESET_ALARMING else ""
        line += f" rst={rst}{mark}"

    heap = rec.get("heap")
    heap_min = rec.get("heap_min")
    if heap is not None:
        line += f" heap={heap // 1024}k"
        if heap_min is not None:
            line += f"(min {heap_min // 1024}k)"
            # Menos de 20k libres historicos es zona de riesgo
            if heap_min < 20000:
                line += " <== HEAP BAJO"

    loop_now = rec.get("loop_max_now")
    if loop_now is not None:
        line += f" loop={loop_now}ms"
        # El task watchdog del ESP32 salta a los 5 s
        if loop_now > 1000:
            line += " <== BUCLE LENTO"

    # --- recepcion: el fallo unidireccional se ve aca ---
    last_rx = rec.get("last_rx_s")
    if last_rx is not None and last_rx >= 0:
        line += f" last_rx={last_rx}s"
        # El ping corre a 1 Hz: mas de 10 s sin recibir nada no es normal
        if last_rx >= 10:
            line += " <== RX DETENIDO"
    if rec.get("rx_stalls"):
        line += (f" rx_stalls={rec['rx_stalls']}"
                 f"(max {rec.get('rx_stall_max_s', 0)}s)")

    # Uptime de cada corte: si se agrupan cerca de 3600 s o de un multiplo,
    # hay un temporizador venciendo y no un evento aleatorio
    at = rec.get("stall_at")
    if at:
        marks = []
        for t in at:
            near_hour = abs(t % 3600 - 3600) < 120 or t % 3600 < 120
            marks.append(f"{t}s{'*' if near_hour else ''}")
        line += f" cortes_en=[{','.join(marks)}]"
        if any(abs(t % 3600 - 3600) < 120 or t % 3600 < 120 for t in at):
            line += " <== CERCA DE UN MULTIPLO DE 1 H"

    # heap DMA: si se agota, el driver no puede reservar buffers de RX
    dma = rec.get("heap_dma")
    if dma is not None:
        line += f" dma={dma // 1024}k"
        dma_min = rec.get("heap_dma_min")
        if dma_min is not None:
            line += f"(min {dma_min // 1024}k)"
            if dma_min < 8000:
                line += " <== DMA AGOTANDOSE"

    # Core dump del cuelgue anterior: la direccion donde murio
    pc = rec.get("panic_pc")
    if pc:
        line += (f"  <== CORE DUMP pc={pc}"
                 f" tarea={rec.get('panic_task', '?')}")

    # Arranques desde el ultimo corte de alimentacion: si crece, el robot
    # se esta reiniciando solo
    boots = rec.get("boots")
    if boots is not None and boots > 1:
        line += f" boots={boots}"

    # Respuesta al comando HISTORY: el historial completo
    hist = rec.get("hist")
    if hist is not None:
        n_boots = rec.get("boots", "?")
        line += (f"\n  historial ({len(hist)} eventos, "
                 f"{n_boots} arranques):")
        for h in hist:
            d = f" d={h['d']}" if h.get("d") else ""
            line += f"\n    boot#{h['b']} up={h['t']}s {h['ev']}{d}"

    if rec.get("past_wrap"):
        line += "  [pasada la marca de 71.58 min]"

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
    ap.add_argument("--cmd", choices=["STATUS", "QUIET", "LOUD", "RESET",
                                      "HISTORY", "CLEAR_HISTORY"],
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
