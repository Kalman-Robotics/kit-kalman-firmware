// Copyright 2023-2025 kalman.AI
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

#include <Arduino.h>

// Con el transporte serial de micro-ROS, el UART transporta tramas XRCE-DDS:
// cualquier Serial.print suelto se intercala entre ellas y corrompe la sesion.
// Por eso los logs se compilan a nada por defecto.
//
// Para depurar sin micro-ROS (por ejemplo, probando motores o LiDAR contra el
// monitor serie) basta con compilar con -DKALMAN_DEBUG_SERIAL. NO uses ese
// build con el agente conectado.
//
// Lo que hay que observar en operacion normal va al topico /link_health y a
// rosout, no aqui.

#ifdef KALMAN_DEBUG_SERIAL
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  // El do/while(0) mantiene la macro utilizable como sentencia dentro de un
  // if sin llaves, que es como aparece en varios sitios del firmware.
  #define DEBUG_PRINT(...)    do {} while (0)
  #define DEBUG_PRINTLN(...)  do {} while (0)
  #define DEBUG_PRINTF(...)   do {} while (0)
#endif
