# Prueba minima de micro-ROS por serial

Un publisher y nada mas: `/min_counter` (std_msgs/Int32) a 1 Hz. Sin logs,
sin subscriber, sin LED. Si algo falla, reinicia y reintenta.

Existe para aislar el transporte: si esto publica, el enlace serial funciona
y cualquier fallo del firmware completo esta en el firmware. Si esto no
publica, el problema es el transporte, el cable o el agente.

## Cargar

    pio run -t upload                          # desde Windows, COM8
    pio run -t upload --upload-port /dev/ttyACM1   # desde la Raspberry

## Agente

micro-ROS va por `Serial` = UART0 = chip CH343. En la Raspberry ese es el
dispositivo `1a86`, NO el `Espressif_USB_JTAG` (ese es el USB nativo del S3,
otro puerto distinto):

    ls -l /dev/serial/by-id/
    # usb-1a86_USB_Single_Serial_*-if00        -> este
    # usb-Espressif_USB_JTAG_serial_debug_*    -> este NO

    ros2 run micro_ros_agent micro_ros_agent serial \
      --dev /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AF7090991-if00 -b 115200

    ros2 topic echo /min_counter

Los numeros ttyACM0/ttyACM1 se intercambian entre arranques; el enlace
`by-id` es estable.

## Baudrate

115200, que es el defecto de micro-ROS y el mas tolerante. El valor lo fija
`Serial.begin()` en `lib/micro_ros_kalman/src/default_transport.cpp`, que NO
esta versionado (.gitignore linea 6): si clonas el repo en otra maquina,
comprueba que ahi diga 115200 y que coincida con el `-b` del agente.
