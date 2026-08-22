// Copyright 2023-2024 REMAKE.AI, kalman.AI, MAKERSPET.COM
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

#include <PID_Timed.h>
#include <Arduino.h>

class MotorController {
  public:
    typedef void (*SetPWMCallback)(MotorController*, float);
    enum encoder_type_t {
      ENCODER_UNSIGNED,
      ENCODER_SIGNED,
    };

    void init(encoder_type_t encoder_type, uint8_t ticks_per_pulse);
    void setPWMCallback(SetPWMCallback callback);
    bool setTargetRPM(float rpm);
    void resetEncoders();
    void update();
    float getShaftAngle();
    void reverseMotor(bool reversed);
    void reverseEncoder(bool reversed);
    void setMaxRPM(float rpm);
    void setEncoderPPR(float ppr);
    void setPIDConfig(float kp, float ki, float kd, float period, float kpm);
    void setPIDKp(float kp);
    void setPIDKpm(float kpm);
    void setPIDKi(float ki);
    void setPIDKd(float kd);
    void setPIDPeriod(float period);
    float getCurrentPWM();
    float getCurrentRPM();
    float getTargetRPM();
    float getPIDError() { return targetRPM - measuredRPM; }
    float getMaxRPM();
    float getEncoderTPR();
    float getEncoderPPR();
    float getPIDKp();
    float getPIDKpm();
    float getPIDKi();
    float getPIDKd();
    float getPIDPeriod();
    void enablePID(bool en);
    // Lectura protegida: el contador lo escriben los ISR de encoder desde el
    // otro nucleo. Sin la seccion critica se puede leer un valor a medio
    // actualizar.
    int32_t getEncoderValue() {
      int32_t v;
      portENTER_CRITICAL(&encoderMux);
      v = encoder;
      portEXIT_CRITICAL(&encoderMux);
      return encoderReversed ? -v : v;
    }

  protected:
    // int32_t y no long: en ESP32 un incremento de 32 bits es una sola
    // instruccion, asi que el ISR no puede dejarlo a medias. A 1050 PPR y
    // 200 rpm tarda ~2 dias en desbordar, y la resta de deltas es correcta
    // ante el desborde por ser aritmetica en complemento a dos.
    volatile int32_t encoder;
    portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
    bool encoderReversed;

    void setPWM(float value);
    SetPWMCallback set_pwm_callback;
    PID_FLOAT pid;
    float pidPWM;
    float targetRPM;
    float measuredRPM;
    float pwm;
    float maxRPM;
    bool cw;

    uint8_t ticksPerPulse;
    float encoderPPR;
    float encoderTPR;
    float encoderTPR_reciprocal;
    float ticksPerMicroSecToRPM;

    unsigned int pidUpdatePeriodUs;
    encoder_type_t encoderType;
    int32_t encPrev;
    bool setPointHasChanged;
    bool motorReversed;
    unsigned long tickSampleTimePrev;
    bool switchingCw;

  public:
    // IRAM_ATTR obligatorio: las llama un ISR. Si quedaran en flash, una
    // interrupcion de encoder mientras el cacheo esta deshabilitado (escritura
    // a NVS/SPIFFS, operaciones internas del driver WiFi) provoca un cache miss
    // con las interrupciones cortadas => int_wdt o panic. Con 4 ISR en CHANGE y
    // encoders de 1050 PPR la ventana es minuscula pero se abre miles de veces
    // por segundo.
    void IRAM_ATTR tickSignedEncoder(bool increment) {
//      if (increment ^ encoderReversed)
      if (increment)
        encoder = encoder + 1;
      else
        encoder = encoder - 1;
    }
    void IRAM_ATTR tickUnsignedEncoder() {
//      if (cw ^ encoderReversed)
      if (cw)
        encoder = encoder + 1;
      else
        encoder = encoder - 1;
    }
};
