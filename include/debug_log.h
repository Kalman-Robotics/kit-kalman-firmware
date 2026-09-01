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

// La placa expone dos puertos serie USB independientes, uno por conector:
//
//   conector UART (chip CH343)  -> Serial     -> tramas XRCE-DDS de micro-ROS
//   conector USB  (nativo S3)   -> USBSerial  -> estos logs
//
// Son caminos fisicos distintos, asi que imprimir aqui NO corrompe la sesion
// micro-ROS: se puede depurar con el agente conectado, que es justo lo que
// hace falta para las pruebas de estabilidad del enlace.
//
// Los logs salen por USBSerial solo si se compila con -DKALMAN_DEBUG_SERIAL
// (env esp32-s3-debug). Por defecto se compilan a nada: escribir en un puerto
// que nadie lee cuesta tiempo en el bucle, y el bucle es lo que se esta
// midiendo. Lo que hay que observar en operacion normal va al topico
// /link_health y a rosout.
//
// Requiere el segundo cable USB al conector rotulado USB de la placa. Sin el,
// USBSerial acepta las escrituras y las descarta: no bloquea ni falla.

#ifdef KALMAN_DEBUG_SERIAL
  #define DEBUG_PRINT(...)    USBSerial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  USBSerial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...)   USBSerial.printf(__VA_ARGS__)
  // Abre el puerto nativo. En USB el baudrate es indiferente --no hay reloj
  // fisico que acordar-- asi que begin() va sin argumentos.
  #define DEBUG_BEGIN()       do { USBSerial.begin(); delay(500); } while (0)
#else
  // El do/while(0) mantiene la macro utilizable como sentencia dentro de un
  // if sin llaves, que es como aparece en varios sitios del firmware.
  #define DEBUG_PRINT(...)    do {} while (0)
  #define DEBUG_PRINTLN(...)  do {} while (0)
  #define DEBUG_PRINTF(...)   do {} while (0)
  #define DEBUG_BEGIN()       do {} while (0)
#endif
