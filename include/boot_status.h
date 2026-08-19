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

#include "led_rgb.h"

// Codigo de colores del LED RGB durante el arranque. Cada etapa tiene su propio
// color para poder ver, sin monitor serial, en cual se quedo trabado el robot.
//
// OJO: el LED RGB comparte el GPIO 48 con el bus del IMU, asi que solo se puede
// usar antes de imu.begin(). Despues del arranque el LED se apaga y el estado se
// sigue por el LED de sistema.
//
//   BOOT_WIFI_CONNECTING   azul parpadeante   asociandose al AP
//   BOOT_WIFI_RETRY        violeta parpadeante  reintento tras timeout de WiFi
//   BOOT_WIFI_NO_IP        cian parpadeante   asociado pero sin IP valida
//   BOOT_AGENT_SEARCHING   ambar fijo         buscando el agente micro-ROS
//   BOOT_AGENT_RETRY       ambar parpadeante  el agente no responde, reintentando
//   BOOT_AGENT_TIMEOUT     rojo parpadeante   se agoto la espera, reiniciando
//   BOOT_ROS_INIT          turquesa fijo      creando nodo, pubs y subs
//   BOOT_READY             verde fijo         todo arriba
//   BOOT_SESSION_IDLE      blanco parpadeo    sin sesion, esperando a la Raspberry
//   BOOT_SESSION_GRACE     naranja parpadeo   agente perdido, reintentando
enum boot_state_t {
  BOOT_WIFI_CONNECTING,
  BOOT_WIFI_RETRY,
  BOOT_WIFI_NO_IP,
  BOOT_AGENT_SEARCHING,
  BOOT_AGENT_RETRY,
  BOOT_AGENT_TIMEOUT,
  BOOT_ROS_INIT,
  BOOT_READY,
  BOOT_SESSION_IDLE,
  BOOT_SESSION_GRACE,
};

extern RGBLedControl rgb_led;

static const uint8_t BOOT_LED_BRIGHTNESS = 30;

// Pinta el estado. Para los estados parpadeantes alterna en cada llamada, asi
// que hay que llamarla periodicamente desde el bucle de espera correspondiente.
inline void setBootState(boot_state_t state) {
  static bool blink = false;
  blink = !blink;

  switch (state) {
    case BOOT_WIFI_CONNECTING:
      rgb_led.setColor(0, 0, 255, BOOT_LED_BRIGHTNESS, blink);
      break;
    case BOOT_WIFI_RETRY:
      rgb_led.setColor(160, 0, 255, BOOT_LED_BRIGHTNESS, blink);
      break;
    case BOOT_WIFI_NO_IP:
      rgb_led.setColor(0, 255, 255, BOOT_LED_BRIGHTNESS, blink);
      break;
    case BOOT_AGENT_SEARCHING:
      rgb_led.setColor(255, 150, 0, BOOT_LED_BRIGHTNESS, true);
      break;
    case BOOT_AGENT_RETRY:
      rgb_led.setColor(255, 150, 0, BOOT_LED_BRIGHTNESS, blink);
      break;
    case BOOT_AGENT_TIMEOUT:
      rgb_led.setColor(255, 0, 0, BOOT_LED_BRIGHTNESS, blink);
      break;
    case BOOT_ROS_INIT:
      rgb_led.setColor(0, 255, 180, BOOT_LED_BRIGHTNESS, true);
      break;
    case BOOT_READY:
      rgb_led.setColor(0, 255, 0, BOOT_LED_BRIGHTNESS, true);
      break;
    // Estos dos ocurren despues del arranque, cuando el RGB ya esta apagado
    // por el IMU. Se sigue el estado por el LED de sistema (ver spinSessionLed)
    case BOOT_SESSION_IDLE:
    case BOOT_SESSION_GRACE:
      break;
  }
}
