#!/usr/bin/env python3
"""Prueba de larga duracion del enlace micro-ROS.

Envia comandos de velocidad cada N minutos y verifica que el robot responda,
comparando contra la odometria que vuelve por el topico de telemetria.

Reproduce el escenario donde aparecia el bug historico: con comandos espaciados,
tras el desborde de 32 bits a los 71.58 min el ESP32 dejaba de mover los motores
hasta el siguiente comando. El fix (int64_t en el firmware) deberia eliminarlo.

Uso:
    ros2 run ... o directamente:
    ./cmd_vel_test.py                      # cada 5 min, indefinido
    ./cmd_vel_test.py --interval 300       # intervalo en segundos
    ./cmd_vel_test.py --speed 0.08         # velocidad lineal (m/s)
    ./cmd_vel_test.py --duration 1.5       # cuanto dura cada pulso (s)
    ./cmd_vel_test.py -o microros_test.log

Que mirar: la marca de 71.58 min (4295 s). Si los comandos posteriores siguen
confirmando movimiento, el bug esta cerrado y se puede quitar el reinicio
programado de 55 min de la Raspberry.
"""

import argparse
import sys
import time
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
except ImportError:
    print("ERROR: falta ROS 2. Hacer 'source /opt/ros/<distro>/setup.bash'",
          file=sys.stderr)
    sys.exit(1)

# El topico de telemetria del robot; se usa para confirmar que se movio
TELEM_TOPIC = "telemetry"
CMD_VEL_TOPIC = "cmd_vel"
WRAP_MARK_S = 4295  # 71.58 min, donde desbordaba el contador de 32 bits


def stamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def fmt_hms(seconds: float) -> str:
    h, rem = divmod(int(seconds), 3600)
    m, s = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


class CmdVelTester(Node):
    def __init__(self, speed, duration, log):
        super().__init__("cmd_vel_tester")
        self.speed = speed
        self.duration = duration
        self.log = log

        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # La telemetria del robot es best-effort; hay que igualar el QoS o no
        # llega nada
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.last_odom_vel = 0.0
        self.last_odom_x = 0.0
        self.last_seq = -1
        self.seq_gaps = 0          # mensajes de telemetria perdidos
        self.telem_count = 0
        self.lidar_bytes = 0       # datos crudos del lidar que llegan
        self.dist_front = 0        # sectores de distancia
        self.telem_sub = None
        try:
            from kalman_interfaces.msg import NexusTelemetry
            self.telem_sub = self.create_subscription(
                NexusTelemetry, TELEM_TOPIC, self._on_telem, qos)
        except ImportError:
            self.write("AVISO: kalman_interfaces no disponible; "
                       "no se podra confirmar el movimiento por odometria")

        self.sent = 0
        self.confirmed = 0
        self.failed = 0
        self.t0 = time.time()

    def _on_telem(self, msg):
        self.telem_count += 1
        self.last_odom_vel = abs(getattr(msg, "odom_vel_x", 0.0))
        self.last_odom_x = getattr(msg, "odom_pos_x", 0.0)
        self.dist_front = getattr(msg, "dist_front_mm", 0)

        # El lidar viaja crudo en el campo lds; si deja de llegar, se nota
        lds = getattr(msg, "lds", None)
        if lds is not None:
            self.lidar_bytes += len(lds)

        # seq permite detectar telemetria perdida, no solo ausente
        seq = getattr(msg, "seq", None)
        if seq is not None:
            if self.last_seq >= 0 and seq > self.last_seq + 1:
                self.seq_gaps += seq - self.last_seq - 1
            self.last_seq = seq

    def write(self, line: str):
        print(line)
        if self.log:
            self.log.write(line + "\n")
            self.log.flush()   # la prueba dura horas: no bufferear

    def spin_for(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_pulse(self) -> bool:
        """Manda un pulso de velocidad y confirma por odometria."""
        self.sent += 1
        elapsed = time.time() - self.t0

        tw = Twist()
        tw.linear.x = self.speed

        peak = 0.0
        end = time.time() + self.duration
        while time.time() < end and rclpy.ok():
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
            peak = max(peak, self.last_odom_vel)

        # Parar
        self.pub.publish(Twist())
        self.spin_for(0.5)

        # Umbral holgado: basta con que se haya movido
        moved = peak > (self.speed * 0.15)
        if moved:
            self.confirmed += 1
        else:
            self.failed += 1

        mark = ""
        if elapsed > WRAP_MARK_S:
            mark = "  [pasada la marca de 71.58 min]"

        status = "OK  odom=%.3f m/s" % peak if moved else "SIN RESPUESTA"
        lidar = "lidar=OK" if self.lidar_bytes > 0 else "lidar=SIN DATOS"
        self.write(f"[{stamp()}] up={fmt_hms(elapsed)} #{self.sent} "
                   f"v={self.speed} -> {status}"
                   f"  (ok={self.confirmed} fail={self.failed}"
                   f" telem={self.telem_count} gaps={self.seq_gaps}"
                   f" {lidar} front={self.dist_front}mm){mark}")

        # Reiniciar el contador de bytes: mide el intervalo, no el acumulado
        self.lidar_bytes = 0
        return moved


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interval", type=float, default=300,
                    help="segundos entre comandos (default 300 = 5 min)")
    ap.add_argument("--speed", type=float, default=0.08,
                    help="velocidad lineal en m/s (default 0.08)")
    ap.add_argument("--duration", type=float, default=1.5,
                    help="duracion de cada pulso en s (default 1.5)")
    ap.add_argument("-o", "--out", default="microros_test.log")
    args = ap.parse_args()

    log = open(args.out, "a", encoding="utf-8")
    log.write(f"\n=== inicio {stamp()} "
              f"(intervalo {args.interval}s, v={args.speed}) ===\n")
    log.flush()

    rclpy.init()
    node = CmdVelTester(args.speed, args.duration, log)

    node.write(f"Enviando cmd_vel cada {args.interval}s. Ctrl-C para terminar.")
    node.write("Esperando telemetria del robot ...")
    node.spin_for(3.0)
    if node.telem_count == 0:
        node.write("AVISO: no llega telemetria. Verificar que el agente "
                   "micro-ROS este corriendo y el ROS_DOMAIN_ID sea correcto.")

    try:
        while rclpy.ok():
            node.send_pulse()
            node.spin_for(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - node.t0
        node.write(f"\n=== fin {stamp()} — duracion {fmt_hms(elapsed)}, "
                   f"enviados {node.sent}, ok {node.confirmed}, "
                   f"fallidos {node.failed} ===")
        if elapsed > WRAP_MARK_S and node.failed == 0:
            node.write("RESULTADO: se paso la marca de 71.58 min sin fallos. "
                       "El fix del desborde funciona.")
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
