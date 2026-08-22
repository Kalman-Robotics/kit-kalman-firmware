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
#   .\scripts\ota_via_pi.ps1
#   .\scripts\ota_via_pi.ps1 -PiHost pi@192.168.18.50
#   .\scripts\ota_via_pi.ps1 -EnvName esp32-s3-diag-ota -EspIp 192.168.4.48

param(
    [string]$EnvName = "esp32-s3-diag-ota",
    [string]$PiHost  = "pi5_robot",   # alias de ~/.ssh/config
    [string]$EspIp   = "192.168.4.48",
    [int]   $EspPort = 3232,
    [string]$OtaPass = "kalman2024",
    [string]$RemoteDir = "/tmp"
)

$ErrorActionPreference = "Stop"

Write-Host "==> Compilando $EnvName"
pio run -e $EnvName
if ($LASTEXITCODE -ne 0) { throw "fallo la compilacion" }

$bin = ".pio\build\$EnvName\firmware.bin"
if (-not (Test-Path $bin)) { throw "no se genero $bin" }

$kb = [math]::Round((Get-Item $bin).Length / 1KB)
Write-Host "==> Binario: $bin ($kb KB)"

# espota.py viene con el framework de Arduino; se copia junto al binario para
# no depender de que la Raspberry lo tenga instalado
$espota = Get-ChildItem "$env:USERPROFILE\.platformio\packages" -Recurse `
    -Filter "espota.py" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $espota) { throw "no se encontro espota.py en ~/.platformio" }

Write-Host "==> Copiando a ${PiHost}:$RemoteDir"
scp -q $bin $espota.FullName "${PiHost}:$RemoteDir/"
if ($LASTEXITCODE -ne 0) { throw "fallo el scp" }

Write-Host "==> Cargando a ${EspIp}:$EspPort desde la Raspberry"
$cmd = "python3 $RemoteDir/espota.py -i $EspIp -p $EspPort -a $OtaPass " +
       "-f $RemoteDir/firmware.bin -r -d"
ssh $PiHost $cmd
if ($LASTEXITCODE -ne 0) { throw "fallo la carga OTA" }

Write-Host ""
Write-Host "==> Carga terminada. Verificar la version con:"
Write-Host "    python3 scripts/wifi_monitor.py --port 8891 --cmd STATUS"
Write-Host '    (buscar el campo "fw")'
