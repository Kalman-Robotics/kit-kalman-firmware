// Copyright 2023-2025 KAIA.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include "robot_config.h"
#include "diag.h"

// Canal de control de sesion entre la Raspberry y el ESP32.
//
// La Raspberry es el AP (192.168.4.1) y corre el agente micro-ROS dentro de un
// contenedor Docker que solo existe mientras hay una sesion de laboratorio. El
// ESP32 esta encendido de forma permanente, asi que necesita distinguir dos
// situaciones que por red se ven identicas:
//
//   - el agente no responde por un corte de red o un reinicio del contenedor
//     => hay que reintentar
//   - el agente no responde porque la sesion termino
//     => hay que dejar de buscar y volver a esperar
//
// Ningun heuristico de tiempo separa esos casos de forma confiable, por eso la
// Raspberry lo dice explicitamente por UDP.
//
// Protocolo (texto plano, un mensaje por datagrama):
//
//   Raspberry -> ESP32   broadcast a 192.168.4.255:8889
//     SESSION_START      el agente esta listo, conectarse ya
//     SESSION_END        la sesion termino, volver a esperar
//     PING               sonda de diagnostico
//
//   ESP32 -> Raspberry   unicast al emisor, mismo puerto
//     ACK <evento> <ip> <estado> <uptime_s>
//
// El ACK permite a la Raspberry saber que el robot recibio el aviso y en que
// estado quedo, que es lo que hace observable donde se rompe el flujo.

enum session_state_t {
  SESSION_IDLE,       // sin sesion: esperando aviso de la Raspberry
  SESSION_ACTIVE,     // agente conectado, laboratorio en curso
  SESSION_GRACE,      // agente perdido: reintentando por si fue un corte
};

inline const char * sessionStateName(session_state_t s) {
  switch (s) {
    case SESSION_IDLE:   return "IDLE";
    case SESSION_ACTIVE: return "ACTIVE";
    case SESSION_GRACE:  return "GRACE";
  }
  return "?";
}

class SessionLink {
  public:
    void begin(uint16_t port) {
      port_ = port;
      listening_ = udp_.begin(port);
      if (!listening_) {
        Serial.print("SessionLink: UDP begin() failed on port ");
        Serial.println(port);
      } else {
        Serial.print("SessionLink listening on UDP ");
        Serial.println(port);
      }
    }

    // Reabre el socket tras una reconexion de WiFi: al bajar la interfaz el
    // socket queda atado a una IP que ya no existe
    void restart() {
      udp_.stop();
      listening_ = udp_.begin(port_);
    }

    bool listening() const { return listening_; }

    // Devuelve true si llego un evento. Hay que llamarla seguido desde loop().
    // El ACK se manda siempre que el datagrama sea valido, incluso si el evento
    // no cambia nada, asi la Raspberry puede sondear sin efectos secundarios.
    bool poll(String & event_out, session_state_t state) {
      if (!listening_)
        return false;

      // No sondear en cada iteracion: compite con la lectura serial del LiDAR
      static unsigned long last_poll_ms = 0;
      if (millis() - last_poll_ms < 50)
        return false;
      last_poll_ms = millis();

      int len = udp_.parsePacket();
      if (len <= 0)
        return false;

      char buf[64];
      int n = udp_.read(buf, sizeof(buf) - 1);
      if (n <= 0)
        return false;
      buf[n] = '\0';

      String event(buf);
      event.trim();
      event.toUpperCase();

      IPAddress from = udp_.remoteIP();
      Serial.print("SessionLink rx '");
      Serial.print(event);
      Serial.print("' from ");
      Serial.println(from);

      sendAck(from, event, state);

      event_out = event;
      last_rx_ms_ = millis();
      return true;
    }

    unsigned long lastRxMs() const { return last_rx_ms_; }

  private:
    void sendAck(IPAddress to, const String & event, session_state_t state) {
      String ack = "ACK ";
      ack += event;
      ack += " ";
      ack += WiFi.localIP().toString();
      ack += " ";
      ack += sessionStateName(state);
      ack += " ";
      ack += String(millis() / 1000);

      if (udp_.beginPacket(to, port_) != 1) {
        Serial.println("SessionLink: beginPacket() failed");
        return;
      }
      udp_.print(ack);
      if (udp_.endPacket() != 1)
        Serial.println("SessionLink: endPacket() failed");
    }

    WiFiUDP udp_;
    uint16_t port_ = 8889;
    bool listening_ = false;
    unsigned long last_rx_ms_ = 0;
};
