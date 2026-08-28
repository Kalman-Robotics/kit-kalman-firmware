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
#include "robot_config.h"
#include "debug_log.h"

extern CONFIG cfg;

float getBatteryMilliVolts() {
  float voltage_mv = analogReadMilliVolts(cfg.adc_bat_gpio);
  //if (voltage_mv < cfg.adc_bat_voltage_empty)
  //  voltage_mv = 0;
  return voltage_mv * cfg.adc_bat_atten;
}

void setupADC() {
//  if (!adcAttachPin(cfg.adc_bat_gpio))
//    DEBUG_PRINTLN("adcAttachPin() FAILED");

  DEBUG_PRINT("Battery ADC attenuation ");
  DEBUG_PRINT(cfg.adc_bat_atten);

  float batt_mv = getBatteryMilliVolts();
  if (batt_mv < cfg.adc_bat_voltage_empty) {
    DEBUG_PRINTLN();
    DEBUG_PRINTLN("Battery NOT detected. Check battery switch, "
      "connection or replace battery");
  } else {
    DEBUG_PRINT(", voltage ");
    DEBUG_PRINT(batt_mv*0.001f);
    DEBUG_PRINTLN("V");
  }
}
