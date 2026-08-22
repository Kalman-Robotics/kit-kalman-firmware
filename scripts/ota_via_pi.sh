#!/usr/bin/env bash
# Carga el firmware al ESP32 por WiFi, usando la Raspberry como puente.
#
# La PC de desarrollo esta en 192.168.18.x y el ESP32 en 192.168.4.x, que es la
# red que abre la Raspberry. Como no hay ruta directa entre ambas, espota.py
# falla con "Listen Failed": necesita que el ESP32 conecte de vuelta.
#
# La Raspberry si esta en las dos redes, asi que este script compila en local,
# copia el binario alla y lanza la carga desde ese lado.
#
# Uso:
#   ./scripts/ota_via_pi.sh                      # env y destino por defecto
#   ./scripts/ota_via_pi.sh -e esp32-s3-diag-ota
#   PI_HOST=pi@192.168.18.50 ./scripts/ota_via_pi.sh
#
# Requiere acceso SSH a la Raspberry (idealmente con clave, para no teclear la
# contrasena tres veces).

set -euo pipefail

ENV_NAME="${ENV_NAME:-esp32-s3-diag-ota}"
PI_HOST="${PI_HOST:-pi@raspberrypi.local}"
ESP_IP="${ESP_IP:-192.168.4.48}"
ESP_PORT="${ESP_PORT:-3232}"
OTA_PASS="${OTA_PASS:-kalman2024}"
REMOTE_DIR="${REMOTE_DIR:-/tmp}"

while [ $# -gt 0 ]; do
  case "$1" in
    -e|--env)   ENV_NAME="$2"; shift 2 ;;
    -H|--host)  PI_HOST="$2";  shift 2 ;;
    -i|--ip)    ESP_IP="$2";   shift 2 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "opcion desconocida: $1" >&2; exit 1 ;;
  esac
done

BIN=".pio/build/${ENV_NAME}/firmware.bin"

echo "==> Compilando ${ENV_NAME}"
pio run -e "${ENV_NAME}"

if [ ! -f "${BIN}" ]; then
  echo "ERROR: no se genero ${BIN}" >&2
  exit 1
fi

SIZE=$(wc -c < "${BIN}")
echo "==> Binario: ${BIN} ($((SIZE / 1024)) KB)"

# espota.py viene con el framework de Arduino; se copia junto al binario para
# no depender de que la Raspberry lo tenga instalado
ESPOTA=$(find "${HOME}/.platformio/packages" -name espota.py 2>/dev/null | head -1)
if [ -z "${ESPOTA}" ]; then
  echo "ERROR: no se encontro espota.py en ~/.platformio" >&2
  exit 1
fi

echo "==> Copiando a ${PI_HOST}:${REMOTE_DIR}"
scp -q "${BIN}" "${ESPOTA}" "${PI_HOST}:${REMOTE_DIR}/"

echo "==> Cargando a ${ESP_IP}:${ESP_PORT} desde la Raspberry"
ssh "${PI_HOST}" "python3 ${REMOTE_DIR}/espota.py \
  -i ${ESP_IP} -p ${ESP_PORT} -a ${OTA_PASS} \
  -f ${REMOTE_DIR}/firmware.bin -r -d"

echo
echo "==> Carga terminada. Verificar la version con:"
echo "    python3 scripts/wifi_monitor.py --port 8891 --cmd STATUS"
echo "    (buscar el campo \"fw\")"
