#include <Arduino.h>

extern "C"
{
  #include <stdio.h>
  #include <stdbool.h>
  #include <sys/time.h>

  int clock_gettime(clockid_t unused, struct timespec *tp) __attribute__ ((weak));
  bool arduino_transport_open(struct uxrCustomTransport * transport) __attribute__ ((weak));
  bool arduino_transport_close(struct uxrCustomTransport * transport) __attribute__ ((weak));
  size_t arduino_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode) __attribute__ ((weak));
  size_t arduino_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) __attribute__ ((weak));

  #define micro_rollover_useconds 4294967295

  int clock_gettime(clockid_t unused, struct timespec *tp)
  {
    (void)unused;
    static uint32_t rollover = 0;
    static uint32_t last_measure = 0;

    uint32_t m = micros();
    rollover += (m < last_measure) ? 1 : 0;

    uint64_t real_us = (uint64_t) (m + rollover * micro_rollover_useconds);
    tp->tv_sec = real_us / 1000000;
    tp->tv_nsec = (real_us % 1000000) * 1000;
    last_measure = m;

    return 0;
  }
  bool arduino_transport_open(struct uxrCustomTransport * transport)
  {
    // Solo la primera vez. micro-ROS llama a open() en cada reintento de
    // conexion con el agente, y en USB nativo cada begin()/end() desconecta y
    // reenumera el dispositivo: repetido, el host acaba dejando de reconocerlo
    // --el puerto desaparece a los pocos segundos--. Con el UART esto no
    // pasaba porque el chip CH343 mantiene el puerto vivo por su cuenta.
    static bool inicializado = false;
    if (inicializado)
      return true;

    // Los buffers por defecto de HWCDC son de 256 B, menos que el MTU de 512
    // de micro-XRCE-DDS (uxr/client/config.h): un frame completo no cabe y el
    // ring buffer se llena, que dispara el camino lento de write().
    USBSerial.setTxBufferSize(1024);
    USBSerial.setRxBufferSize(1024);
    USBSerial.begin();

    // Sin esto write() espera hasta tx_timeout_ms (100 ms) cuando el cable
    // esta puesto pero nadie lee. Repetido en cada spin_some, lleva el bucle
    // de control de milisegundos a decimas de segundo.
    USBSerial.setTxTimeoutMs(0);

    inicializado = true;
    return true;
  }

  bool arduino_transport_close(struct uxrCustomTransport * transport)
  {
    // A proposito NO se llama a USBSerial.end(): apagar el periferico USB
    // desconecta el dispositivo del host. micro-ROS cierra el transporte en
    // cada fallo de conexion, asi que hacerlo aqui es lo que hacia desaparecer
    // el puerto a los ~15 s. El enlace se deja abierto; no hay nada que
    // liberar y el host mantiene la enumeracion.
    return true;
  }

  size_t arduino_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;
    return USBSerial.write(buf, len);
  }

  size_t arduino_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void)errcode;
    USBSerial.setTimeout(timeout);
    return USBSerial.readBytes((char *)buf, len);
  }
}
