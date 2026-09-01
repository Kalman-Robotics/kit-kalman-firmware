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

#ifndef ESP32
  #error This code runs on ESP32
#endif

#include "robot_config.h"
#include "boot_status.h"
#include "util.h"
#include <stdio.h>
#include "motors.h"
#include "lidar.h"
#include "ros.h"
#include "adc.h"
#include "IMU6500.h"
#include "buzzer.h"
#include "led_rgb.h"
#include <SPIFFS.h>
#include "debug_log.h"

#define PIN_BUZZER 10
CONFIG cfg;
RGBLedControl rgb_led(48);
IMU6500 imu;
BuzzerController buzzer(PIN_BUZZER);
kalman_interfaces__msg__JointPosVel joint[MOTOR_COUNT];
float joint_prev_pos[MOTOR_COUNT] = {0};
uint8_t nexus_lidar_buf[CONFIG::LIDAR_BUF_LEN] = {0};

// esp_timer_get_time() devuelve int64_t microsegundos desde el arranque.
// Guardarlo en unsigned long (32 bits en ESP32) lo trunca y desborda a los
// 71.58 min, lo que dejaba a cmd_vel calculando una antiguedad enorme y
// frenando los motores hasta el siguiente comando. Con int64_t el desborde
// pasa a estar a ~292.000 anios.
int64_t telem_prev_pub_time_us = 0;
int64_t ping_prev_pub_time_us = 0;
int64_t ros_params_update_prev_time_us = 0;
int64_t imu_last_pub_us = 0;

int64_t last_cmd_vel_us = 0;

int64_t ramp_duration_us = 0;
int64_t ramp_start_time_us = 0;
float ramp_start_rpm_right = 0;
float ramp_start_rpm_left = 0;
float ramp_target_rpm_right = 0;
float ramp_target_rpm_left = 0;
bool ramp_enabled = true;

unsigned long stat_sum_spin_telem_period_us = 0;
unsigned long stat_max_spin_telem_period_us = 0;

#if ESP_IDF_VERSION_MAJOR >= 5
  #error Espressif IDF v5 is not yet supported
#endif

// -------- FUNCTION PROTOYPES --------
void updateSpeedRamp();
void publishTelem(int64_t step_time_us);
void calcOdometry(int64_t step_time_us, float joint_pos_delta_right, float joint_pos_delta_left);
void spinTelem(bool force_pub);
void spinControlStatus();
void spinPing();
void spinSession();
void spinSessionLed();
// Motivo del ultimo reset y salud del bucle: lo unico que quedo de la
// instrumentacion de las fases 2-4. El resto se quito por no aportar al
// diagnostico y degradar el bucle.
esp_reset_reason_t g_rst_reason = ESP_RST_UNKNOWN;
uint32_t g_loop_max_ms = 0;
uint32_t g_loop_max_ms_since_report = 0;

// Contadores del enlace; ninguno dispara acciones por su cuenta. Se publican
// en /link_health a 1 Hz: ese topico es la unica forma de observarlos ahora
// que el UART lo ocupa el transporte micro-ROS.
uint32_t g_ping_fails = 0;
uint32_t g_agent_lost = 0;
uint32_t g_cmd_vel_rx = 0;

// Estado de la sesion con el agente. Lo gobierna spinPing().
session_state_t session_state = SESSION_IDLE;  // hasta que setup() decida
unsigned long session_grace_start_ms = 0;
void sessionToIdle(const char * reason);
void spinIMU(int64_t time_now_us);
void updateROSParams();
void setMotorSpeeds(float rpm_left, float rpm_right);
bool isBootButtonPressed(uint8_t sec);
void resetTelemMsg();
void resetNexusMsg();
void setupLIDAR();
void setupADC();
void setupMotors();
rcl_ret_t addROSParams();
rcl_ret_t updateROSConfigParams();
rcl_ret_t updateROSRealTimeParams();
String set_param_callback(const char * param_name, const char * param_value);
void twist_sub_callback(const void *msgin);
// -------- FUNCTION PROTOYPES --------

void twist_sub_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_vel_us = esp_timer_get_time();
    g_cmd_vel_rx++;

  float target_speed_lin_x = constrain(msg->linear.x, -0.15f, 0.15f);
  float target_speed_ang_z = constrain(msg->angular.z, -1.0f, 1.0f);
  //DEBUG_PRINT("linear.x ");
  //DEBUG_PRINT(msg->linear.x);
  //DEBUG_PRINT(", angular.z ");
  //DEBUG_PRINTLN(msg->angular.z);

  if (msg->linear.y != 0) {
    DEBUG_PRINT("Warning: /cmd_vel linear.y = ");
    DEBUG_PRINT(msg->linear.y);
    DEBUG_PRINTLN(" not zero");
  }

  // Twist to target wheel speeds
  float twist_target_speed_right = 0;
  float twist_target_speed_left = 0;

  cfg.twistToWheelSpeeds(target_speed_lin_x, target_speed_ang_z,
    &twist_target_speed_right, &twist_target_speed_left);

  twist_target_speed_left = -twist_target_speed_left;

  // Wheel speeds to RPM
  float twist_target_rpm_right = cfg.speed_to_rpm(twist_target_speed_right);
  float twist_target_rpm_left = cfg.speed_to_rpm(twist_target_speed_left);

  //DEBUG_PRINT(", twist_target_rpm_right ");
  //DEBUG_PRINT(twist_target_rpm_right);
  //DEBUG_PRINT(", twist_target_rpm_left ");
  //DEBUG_PRINT(twist_target_rpm_left);

  // Limit target RPM
  float limited_target_rpm_right =
    absMin(twist_target_rpm_right, motorRight.getMaxRPM());
  float limited_target_rpm_left =
    absMin(twist_target_rpm_left, motorLeft.getMaxRPM());

  //DEBUG_PRINT(", limited_target_rpm_right ");
  //DEBUG_PRINT(limited_target_rpm_right);
  //DEBUG_PRINT(", limited_target_rpm_left ");
  //DEBUG_PRINT(limited_target_rpm_left);

  // Scale down both target RPMs to within limits
  if (twist_target_rpm_right != limited_target_rpm_right ||
    twist_target_rpm_left != limited_target_rpm_left) {

    float rpm_scale_down_factor_right = 1;
    float rpm_scale_down_factor_left = 1;

    if (twist_target_rpm_right != 0) {
      rpm_scale_down_factor_right = limited_target_rpm_right /
        twist_target_rpm_right;
    }
    if (twist_target_rpm_left != 0) {
      rpm_scale_down_factor_left = limited_target_rpm_left /
        twist_target_rpm_left;
    }

    float rpm_scale_down_factor = min(rpm_scale_down_factor_right,
      rpm_scale_down_factor_left);   

    ramp_target_rpm_right = twist_target_rpm_right * rpm_scale_down_factor;
    ramp_target_rpm_left = twist_target_rpm_left * rpm_scale_down_factor;
  } else {
    ramp_target_rpm_right = twist_target_rpm_right;
    ramp_target_rpm_left = twist_target_rpm_left;
  }

  //DEBUG_PRINT(", ramp_target_rpm_ri  ght ");
  //DEBUG_PRINT(ramp_target_rpm_right);
  //DEBUG_PRINT(", ramp_target_rpm_left ");
  //DEBUG_PRINTLN(ramp_target_rpm_left);

  if (!ramp_enabled) {
    setMotorSpeeds(ramp_target_rpm_left, ramp_target_rpm_right);
    return;
  }

  // Calculate change in speeds
  ramp_start_rpm_right = motorRight.getTargetRPM();
  ramp_start_rpm_left = motorLeft.getTargetRPM();
  
  float ramp_start_speed_right = cfg.rpm_to_speed(ramp_start_rpm_right);
  float ramp_start_speed_left = cfg.rpm_to_speed(ramp_start_rpm_left);

  float ramp_target_speed_right = cfg.rpm_to_speed(ramp_target_rpm_right);
  float ramp_target_speed_left = cfg.rpm_to_speed(ramp_target_rpm_left);

  float ramp_speed_diff_right = ramp_target_speed_right - ramp_start_speed_right;
  float ramp_speed_diff_left = ramp_target_speed_left - ramp_start_speed_left;

  // Calculate time to accelerate
  float abs_speed_diff_right = abs(ramp_speed_diff_right);
  float abs_speed_diff_left = abs(ramp_speed_diff_left);
  float max_abs_speed_diff = max(abs_speed_diff_right, abs_speed_diff_left);

  ramp_duration_us = max_abs_speed_diff * cfg.speed_diff_to_us;
  ramp_start_time_us = esp_timer_get_time(); // Start speed ramp

  updateSpeedRamp();
}

void updateSpeedRamp() {
  if (ramp_target_rpm_right == motorRight.getTargetRPM() &&
    ramp_target_rpm_left == motorLeft.getTargetRPM()) {
    return;
  }

  int64_t time_now_us = esp_timer_get_time();
  int64_t ramp_elapsed_time_us = time_now_us - ramp_start_time_us;

  float rpm_right;
  float rpm_left;

  if (ramp_elapsed_time_us < ramp_duration_us) {
    float ratio = (float)ramp_elapsed_time_us / (float)ramp_duration_us; // 0..1
    float rpm_change_right = (ramp_target_rpm_right - ramp_start_rpm_right) * ratio;
    float rpm_change_left = (ramp_target_rpm_left - ramp_start_rpm_left) * ratio;

    rpm_right = ramp_start_rpm_right + rpm_change_right;
    rpm_left = ramp_start_rpm_left + rpm_change_left;
  } else {
    rpm_right = ramp_target_rpm_right;
    rpm_left = ramp_target_rpm_left;
  }

  setMotorSpeeds(rpm_left, rpm_right);
}

String set_param_callback(const char * param_name, const char * param_value) {

  static String text;

  if (param_name == NULL) {
    write_file(cfg.NETWORK_YAML_PATH, text.c_str());
    DEBUG_PRINTLN(", restarting...");
    delay(100);
    ESP.restart();
    return "";
  } else {
    text = text + String(param_name) + ": " + String(param_value) + '\n';
    return strcmp(param_name, "pass") == 0 ? "****" : String(param_value);
  }
}

void spinTelem(bool force_pub) {
  static int telem_pub_count = 0;
  int64_t time_now_us = esp_timer_get_time();
  int64_t step_time_us = time_now_us - telem_prev_pub_time_us;

  if (!force_pub && (step_time_us < cfg.UROS_TELEM_PUB_PERIOD_US))
    return;

  publishTelem(step_time_us);
  telem_prev_pub_time_us = time_now_us;

  //if (++telem_pub_count % 5 == 0) {
  //  DEBUG_PRINT("RPM L ");
  //  DEBUG_PRINT(motorLeft.getCurrentRPM());
  //  DEBUG_PRINT(" R ");
  //  DEBUG_PRINTLN(motorRight.getCurrentRPM());
  //}

  stat_sum_spin_telem_period_us += step_time_us;
  stat_max_spin_telem_period_us = stat_max_spin_telem_period_us <= step_time_us ?
    step_time_us : stat_max_spin_telem_period_us;
  
  // How often telemetry gets published
  if (++telem_pub_count % cfg.SPIN_TELEM_STATS == 0) {
    String s = "Telem avg ";
    s = s + String(stat_sum_spin_telem_period_us / (1000*cfg.SPIN_TELEM_STATS));
    s = s + " max ";
    s = s + String(stat_max_spin_telem_period_us / 1000);
    s = s + "ms";

    float rpm = lidar->getCurrentScanFreqHz();
    if (rpm >= 0) {
      s = s + ", LiDAR RPM ";
      s = s + String(rpm);
    }

    s = s + ", wheels RPM ";
    s = s + String(motorLeft.getCurrentRPM()) + " ";
    s = s + String(motorRight.getCurrentRPM());

    s = s + ", RSSI " + String(nexus_msg.wifi_rssi_dbm) + "dBm";
    printlnNB(s);

    stat_sum_spin_telem_period_us = 0;
    stat_max_spin_telem_period_us = 0;
  }
}

void spinControlStatus() {
  static int64_t prev_time_us = 0;
  int64_t time_now_us = esp_timer_get_time();
  if (time_now_us - prev_time_us < cfg.UROS_TELEM_PUB_PERIOD_US)
    return;
  prev_time_us = time_now_us;

  control_status_msg.r_current_speed   = motorRight.getCurrentRPM();
  control_status_msg.r_current_control = motorRight.getCurrentPWM();
  control_status_msg.r_current_error   = motorRight.getPIDError();
  control_status_msg.r_setpoint        = motorRight.getTargetRPM();
  control_status_msg.l_current_speed   = motorLeft.getCurrentRPM();
  control_status_msg.l_current_control = motorLeft.getCurrentPWM();
  control_status_msg.l_current_error   = motorLeft.getPIDError();
  control_status_msg.l_setpoint        = motorLeft.getTargetRPM();

  rcl_ret_t rc = rcl_publish(&control_status_pub, &control_status_msg, NULL);
  if (rc != RCL_RET_OK) {
    DEBUG_PRINT("rcl_publish(control_status) error ");
    DEBUG_PRINTLN(rc);
  }
}

void publishTelem(int64_t step_time_us) {
  struct timespec tv = {0, 0};
  clock_gettime(CLOCK_REALTIME, &tv);
  nexus_msg.stamp.sec = tv.tv_sec;
  nexus_msg.stamp.nanosec = tv.tv_nsec;

  float joint_pos_delta[MOTOR_COUNT];
  float step_time = 1e-6 * (float)step_time_us;

  // El campo sigue en el .msg de NexusTelemetry y quitarlo obligaria a
  // regenerar libmicroros.a. Sobre serial no hay radio que medir: 0 = sin dato.
  nexus_msg.wifi_rssi_dbm = 0;

  for (unsigned char i = 0; i < MOTOR_COUNT; i++) {
    joint[i].pos = i == 0 ? motorLeft.getShaftAngle() : motorRight.getShaftAngle();
    joint_pos_delta[i] = joint[i].pos - joint_prev_pos[i];
    joint[i].vel = joint_pos_delta[i] / step_time;
    joint_prev_pos[i] = joint[i].pos;
  }

  calcOdometry(step_time_us, joint_pos_delta[0], joint_pos_delta[1]);

  // Copiar el ultimo scan completo; el driver llama postPacket() antes de
  // postScanPoint(), asi que nexus_msg no se puede actualizar desde el callback
  lidar_sectors_publish();

  rcl_ret_t rc = rcl_publish(&nexus_telem_pub, &nexus_msg, NULL);
  if (rc != RCL_RET_OK) {
    DEBUG_PRINT("rcl_publish(nexus_msg) error ");
    DEBUG_PRINTLN(rc);
  }

  nexus_msg.lds.size = 0;
  nexus_msg.seq++;
}

void calcOdometry(int64_t step_time_us, float joint_pos_delta_right,
  float joint_pos_delta_left) {

  if (step_time_us == 0)
    return;

  float distance_right = -joint_pos_delta_right * cfg.wheel_radius;
  float distance_left = joint_pos_delta_left * cfg.wheel_radius;

  // TODO use Runge-Kutta integration for better accuracy
  float average_distance = (distance_right + distance_left) * 0.5;
  float d_yaw = (distance_left - distance_right)*cfg.base_wheel_track_recip;
//  if (abs(d_yaw) > 1) {
//    printNB("WARNING: odometry asin() domain out of bounds. Check motor encoders.");
//    d_yaw = d_yaw > 0 ? 1 : -1;
//  }
//  d_yaw = asin(d_yaw);

  // Average angle during the motion
  float average_angle = d_yaw*0.5 + nexus_msg.odom_pos_yaw;

  if (average_angle > PI)
    average_angle -= TWO_PI;
  else if (average_angle < -PI)
    average_angle += TWO_PI;

  float d_x = cos(average_angle) * average_distance;
  float d_y = sin(average_angle) * average_distance;

  nexus_msg.odom_pos_x += d_x;
  nexus_msg.odom_pos_y += d_y;
  nexus_msg.odom_pos_yaw += d_yaw;

  if (nexus_msg.odom_pos_yaw > PI)
    nexus_msg.odom_pos_yaw -= TWO_PI;
  else if (nexus_msg.odom_pos_yaw < -PI)
    nexus_msg.odom_pos_yaw += TWO_PI;

  float d_time = 1e-6 * (float)step_time_us;
  nexus_msg.odom_vel_x = average_distance / d_time;
  nexus_msg.odom_vel_yaw = d_yaw / d_time;
}

// Salud del enlace, publicada en /link_health a 1 Hz.
//
// Con el transporte serial ocupando el UART los Serial.print ya no salen a
// ningun lado, asi que este topico es la unica forma de observar como se
// comporta la conexion durante las pruebas de estabilidad. Se publica tanto en
// exito como en fallo: una serie continua permite ver la degradacion, no solo
// el momento de la caida.
void publishLinkHealth(bool ping_ok, uint32_t rtt_us, uint8_t fail_streak) {
  if (session_state == SESSION_IDLE)
    return;

  const char * state = sessionStateName(session_state);

  int n = snprintf(link_health_buf, sizeof(link_health_buf),
    "{\"ok\":%s,\"rtt_us\":%lu,\"fail_streak\":%u,"
    "\"ping_fails\":%lu,\"agent_lost\":%lu,"
    "\"state\":\"%s\",\"uptime_s\":%lu,\"heap\":%lu}",
    ping_ok ? "true" : "false",
    (unsigned long)rtt_us,
    (unsigned)fail_streak,
    (unsigned long)g_ping_fails,
    (unsigned long)g_agent_lost,
    state,
    (unsigned long)(millis() / 1000),
    (unsigned long)ESP.getFreeHeap());

  if (n <= 0)
    return;
  // snprintf trunca en vez de desbordar, pero devuelve lo que habria escrito
  link_health_msg.data.size = (n < (int)sizeof(link_health_buf)) ?
    (size_t)n : sizeof(link_health_buf) - 1;

  rcl_publish(&link_health_pub, &link_health_msg, NULL);
}

void spinPing() {
  static uint8_t ping_fail_count = 0;
  int64_t time_now_us = esp_timer_get_time();
  int64_t step_time_us = time_now_us - ping_prev_pub_time_us;

  if (step_time_us < cfg.UROS_PING_PUB_PERIOD_US)
    return;

  ping_prev_pub_time_us = time_now_us;

  // Sobre serial el ping es el unico juez del enlace: ya no hay un spinWiFi()
  // que se ocupe de la reconexion por su cuenta.
  int64_t ping_start_us = esp_timer_get_time();
  rmw_ret_t rc = rmw_uros_ping_agent(cfg.UROS_PING_TIMEOUT_MS, 1);
  uint32_t ping_rtt_us = (uint32_t)(esp_timer_get_time() - ping_start_us);

  if (rc != RMW_RET_OK) {
    g_ping_fails++;
    ping_fail_count++;
    DEBUG_PRINT("Ping failed (");
    DEBUG_PRINT(ping_fail_count);
    DEBUG_PRINT("/");
    DEBUG_PRINT(cfg.UROS_PING_MAX_FAILS);
    DEBUG_PRINTLN(")");
    if (ping_fail_count >= cfg.UROS_PING_MAX_FAILS &&
        session_state == SESSION_ACTIVE) {
      // No se reinicia aca: el agente puede haberse reiniciado del lado del
      // host y volver en segundos. Se entra en GRACE y se espera; si el cable
      // se desconecto, el ping seguira fallando y de eso se encarga el
      // temporizador de gracia.
      DEBUG_PRINTLN("micro-ROS agent lost, entering GRACE");
      setMotorSpeeds(0, 0);
      lidar->stop();
      session_state = SESSION_GRACE;
      session_grace_start_ms = millis();
      g_agent_lost++;
    }
  } else {
    // El agente volvio dentro del periodo de gracia: es el unico caso que se
    // resuelve sin reiniciar
    if (session_state == SESSION_GRACE) {
      DEBUG_PRINTLN("micro-ROS agent recovered, back to ACTIVE");
      session_state = SESSION_ACTIVE;
      lidar->start();
    }
    ping_fail_count = 0;
  }

  publishLinkHealth(rc == RMW_RET_OK, ping_rtt_us, ping_fail_count);
}

void updateROSParams() {
  if (ros_config_params_changed) {
    ros_config_params_changed = false;
    rcl_ret_t ret = updateROSConfigParams();
    if (ret != RCL_RET_OK) {
      DEBUG_PRINT("updateROSConfigParams() error ");
      DEBUG_PRINTLN(ret);
    }
  }

  int64_t time_now_us = esp_timer_get_time();
  int64_t step_time_us = time_now_us - ros_params_update_prev_time_us;
  if (step_time_us >= cfg.UROS_PARAMS_UPDATE_PERIOD_US) {

    rcl_ret_t ret = updateROSRealTimeParams();
    if (ret != RCL_RET_OK) {
      DEBUG_PRINT("updateROSRealTimeParams() error ");
      DEBUG_PRINTLN(ret);
    }

    ros_params_update_prev_time_us = time_now_us;
  }
}

static unsigned long session_idle_poll_ms = 0;

// Vuelve a esperar una sesion. micro-ROS no libera limpiamente sus recursos,
// asi que la unica forma segura de dejar el cliente listo para la proxima
// sesion es reiniciar. Se hace aca, cuando ya no hay alumno esperando, y no
// durante GRACE, donde el reinicio costaria segundos de reconexion.
void sessionToIdle(const char * reason) {
  DEBUG_PRINT("Session -> IDLE (");
  DEBUG_PRINT(reason);
  DEBUG_PRINTLN("), restarting to await next session");
  setMotorSpeeds(0, 0);
  lidar->stop();
  // Sin Serial.flush(): el UART lo ocupa el transporte micro-ROS y lo que
  // quede en el buffer son tramas XRCE-DDS de una sesion que ya termino.
  delay(400);
  ESP.restart();
}

// Procesa los avisos de la Raspberry y administra el periodo de gracia.
// Gestion de la sesion sobre el enlace serial.
//
// Con WiFi la sesion la anunciaba la Raspberry por UDP, porque el agente vivia
// en un contenedor que aparecia y desaparecia y por red no habia forma de
// distinguir "aun no empezo" de "se cayo". Sobre serial esa ambiguedad no
// existe: si el agente responde al ping, hay sesion. spinPing() ya mueve
// ACTIVE <-> GRACE, asi que aqui solo queda el plazo maximo de gracia.
void spinSession() {
  // Una sola vez: sin esta guarda sessionToIdle() se reintentaba en cada
  // iteracion del loop --365 veces en una prueba--, con su setMotorSpeeds +
  // lidar->stop + delay(200) cada vez. Eso degradaba el bucle de 56 a 265 ms y
  // convertia un corte de 16 s en un estado roto permanente.
  static bool idle_attempted = false;
  if (session_state != SESSION_GRACE)
    idle_attempted = false;

  if (session_state == SESSION_GRACE && !idle_attempted &&
      millis() - session_grace_start_ms >= cfg.SESSION_GRACE_MS) {
    idle_attempted = true;
    // El agente no volvio dentro del plazo: se asume fin de sesion
    sessionToIdle("grace timeout");
  }
}

// Patron del LED de sistema segun el estado. El RGB no se puede usar despues
// del arranque: comparte el GPIO 48 con el IMU.
//   ACTIVE  apagado (sin parpadeo, todo normal)
//   GRACE   parpadeo rapido, 200 ms
//   IDLE    destello corto cada 3 s
void spinSessionLed() {
  static unsigned long last_ms = 0;
  static bool on = false;

  if (session_state == SESSION_ACTIVE) {
    if (on) {
      on = false;
      digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
    }
    return;
  }

  unsigned long period = (session_state == SESSION_GRACE) ? 200 :
    (on ? 100 : 3000);

  if (millis() - last_ms >= period) {
    last_ms = millis();
    on = !on;
    digiWrite(cfg.led_sys_gpio, on ? HIGH : LOW, cfg.led_sys_invert);
  }
}

void loop() {
  lidar->loop();

  // En IDLE no hay agente ni entidades micro-ROS creadas: llamar al executor
  // o al ping ahi solo gastaria tiempo y falsearia los contadores.
  if (session_state != SESSION_IDLE) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    if (ret != RCL_RET_OK) {
      DEBUG_PRINT("rclc_executor_spin_some() error ");
      DEBUG_PRINTLN(ret);
    }

    updateROSParams();
    int64_t time_now_us = esp_timer_get_time();
    spinIMU(time_now_us);
    spinTelem(false);
    spinControlStatus();
    spinPing();
  }
  spinSession();
  spinSessionLed();

  // Antes este freno lo disparaba la caida del WiFi. Sobre serial el
  // equivalente es haber perdido al agente: en GRACE e IDLE no hay quien
  // mande cmd_vel, asi que dejar los motores girando seria peligroso.
  if (session_state != SESSION_ACTIVE) {
    setMotorSpeeds(0, 0);
  } else if (last_cmd_vel_us > 0 &&
             (esp_timer_get_time() - last_cmd_vel_us) > (int64_t)cfg.cmd_vel_timeout_us) {
    ramp_target_rpm_right = 0;
    ramp_target_rpm_left = 0;
    ramp_start_rpm_right = motorRight.getTargetRPM();
    ramp_start_rpm_left = motorLeft.getTargetRPM();
    ramp_start_time_us = esp_timer_get_time();
    ramp_duration_us = cfg.speed_diff_to_us *
      max(abs(cfg.rpm_to_speed(ramp_start_rpm_right)),
          abs(cfg.rpm_to_speed(ramp_start_rpm_left)));
    last_cmd_vel_us = 0;
    updateSpeedRamp();
  } else {
    updateSpeedRamp();
  }
  motorLeft.update();
  motorRight.update();
}

bool isBootButtonPressed(uint8_t sec) {
  if (digiRead(cfg.button_boot_gpio, cfg.button_boot_invert))
    DEBUG_PRINTLN("BOOT button pressed. Keep pressing for web config.");
  else
    return false;

  uint32_t msec = sec * 1000;
  unsigned long start_time_ms = millis();
  while (digiRead(cfg.button_boot_gpio, cfg.button_boot_invert)) {
    delay(50);
    digiWrite(cfg.led_sys_gpio, !digiRead(cfg.led_sys_gpio, cfg.led_sys_invert),
      cfg.led_sys_invert);
    if (millis() - start_time_ms > msec)
      return true;
  }
  return false;
}

void resetTelemMsg() {
  for (int i = 0; i < MOTOR_COUNT; i++)
    joint_prev_pos[i] = 0;
}

void resetNexusMsg() {
  nexus_msg.seq = 0;
  nexus_msg.odom_pos_x = 0;
  nexus_msg.odom_pos_y = 0;
  nexus_msg.odom_pos_yaw = 0;
  nexus_msg.odom_vel_x = 0;
  nexus_msg.odom_vel_yaw = 0;
  nexus_msg.wifi_rssi_dbm = 0;
  nexus_msg.dist_front_mm = 0;
  nexus_msg.dist_left_mm = 0;
  nexus_msg.dist_back_mm = 0;
  nexus_msg.dist_right_mm = 0;
  nexus_msg.joint.data = joint;
  nexus_msg.joint.capacity = MOTOR_COUNT;
  nexus_msg.joint.size = MOTOR_COUNT;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    joint[i].pos = 0;
    joint[i].vel = 0;
    joint_prev_pos[i] = 0;
  }
  nexus_msg.lds.data = nexus_lidar_buf;
  nexus_msg.lds.capacity = cfg.LIDAR_BUF_LEN;
  nexus_msg.lds.size = 0;
}

void spinIMU(int64_t time_now_us) {
  if ((time_now_us - imu_last_pub_us) < 10000)  // 10ms = 100Hz
    return;

  imu.read();
  imu_msg.accel_x = imu.getAccelX();
  imu_msg.accel_y = imu.getAccelY();
  imu_msg.accel_z = imu.getAccelZ();
  imu_msg.gyro_x = imu.getGyroX();
  imu_msg.gyro_y = imu.getGyroY();
  imu_msg.gyro_z = imu.getGyroZ();

  rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
  if (rc != RCL_RET_OK) {
    DEBUG_PRINT("rcl_publish(imu_msg");
    DEBUG_PRINT(") error ");
    DEBUG_PRINTLN(rc);
  }

  imu_last_pub_us = time_now_us;
}
/*
void blink_error_code(int n_blinks) {
  unsigned int i = 0;
  while(i++ < cfg.ERR_REBOOT_BLINK_CYCLES) {
    blink(cfg.LONG_BLINK_MS, 1);
    digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
    delay(cfg.SHORT_BLINK_PAUSE_MS);
    blink(cfg.SHORT_BLINK_MS, n_blinks);
    delay(cfg.LONG_BLINK_PAUSE_MS);
  }
}

void error_loop(int n_blinks){
  lidar->stop();

  //char buffer[40];
  //sprintf(buffer, "Blinking error %d", n_blinks);  
  //logMsg(buffer, rcl_interfaces__msg__Log__FATAL);
  DEBUG_PRINT("Blinking LED ");
  DEBUG_PRINT(n_blinks);
  DEBUG_PRINTLN(" times...");

  blink_error_code(n_blinks);

  DEBUG_PRINTLN("Rebooting...");
  Serial.flush();

  ESP.restart();
}
*/

void setup() {

  // Silence buzzer — active-low: INPUT = high impedance = silent
  pinMode(PIN_BUZZER, INPUT);

  bool spiffs_ok = SPIFFS.begin(true);
//  blink_error_code(cfg.ERR_SPIFFS_INIT);
  bool wifi_yaml_exists = SPIFFS.exists(cfg.NETWORK_YAML_PATH);
  String wifi_yaml_err;
  if (wifi_yaml_exists)
    wifi_yaml_err = cfg.load(cfg.NETWORK_YAML_PATH);

  bool config_yaml_exists = SPIFFS.exists(cfg.CONFIG_YAML_PATH);
  String config_yaml_err;
  if (config_yaml_exists)
    config_yaml_err = cfg.load(cfg.CONFIG_YAML_PATH);

  // El UART lo abre el transporte micro-ROS en set_microros_transports(), a
  // 921600. Abrirlo aqui a 115200 dejaba al agente hablando a otra velocidad.
  // Los logs van por el otro puerto USB (ver include/debug_log.h); sin
  // -DKALMAN_DEBUG_SERIAL esto no compila a nada.
  DEBUG_BEGIN();
  setPinDrive(cfg.monitor_gpio_tx);

  // Antes que nada: el motivo del reset anterior. Distingue un ESP.restart()
  // del propio codigo de un watchdog, un panic o un brownout, que es lo unico
  // que no se puede deducir desde la Raspberry.
  g_rst_reason = esp_reset_reason();

  DEBUG_PRINTLN();
  DEBUG_PRINT("kalman.ai firmware version ");
  DEBUG_PRINTLN(cfg.FW_VERSION);

  DEBUG_PRINT("ESP IDF version ");
  DEBUG_PRINTLN(esp_get_idf_version());

  if (spiffs_ok) {
    DEBUG_PRINTLN("SPIFFS mounted successfully");
    // Antes se exigia el HTML del portal de configuracion. Ese portal ya no
    // existe: SPIFFS solo hace falta para los .yaml.
  } else {
    DEBUG_PRINTLN("Error mounting SPIFFS");
    idle();
  }

  if (wifi_yaml_exists) {
    DEBUG_PRINT(cfg.NETWORK_YAML_PATH);
    DEBUG_PRINT(" found; ");
    if (wifi_yaml_err.length() != 0) {
      DEBUG_PRINT("error parsing: ");
      DEBUG_PRINTLN(wifi_yaml_err);
    } else
      DEBUG_PRINTLN("loaded OK");
  }

  if (config_yaml_exists) {
    DEBUG_PRINT(cfg.CONFIG_YAML_PATH);
    DEBUG_PRINT(" found; ");
    if (config_yaml_err.length() != 0) {
      DEBUG_PRINT("error parsing: ");
      DEBUG_PRINTLN(config_yaml_err);
    } else
      DEBUG_PRINTLN("loaded OK");
  }

  setPinMode(cfg.led_sys_gpio, OUTPUT);
  digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);

  setPinMode(cfg.button_boot_gpio, INPUT);

  // El portal de configuracion web solo existia para introducir credenciales
  // WiFi y la IP del agente. Sobre serial no hay nada que configurar por red:
  // el agente esta al otro lado del cable.

  DEBUG_PRINT("Board model ");
  DEBUG_PRINT(cfg.board_model);
  DEBUG_PRINT(", version ");
  DEBUG_PRINT(cfg.board_version);
  DEBUG_PRINT(", manufacturer ");
  DEBUG_PRINTLN(cfg.board_manufacturer);
  DEBUG_PRINT("Robot name ");
  DEBUG_PRINT(cfg.robot_name);
  DEBUG_PRINT(", web ");
  DEBUG_PRINTLN(cfg.robot_web);  
  cfg.board_manufacturer = ""; // free up a little memory
  cfg.board_model = "";
  cfg.board_version = "";

  // LiDAR power pin — config ya cargado, apagado por defecto
  if (cfg.lidar_gpio_power != 255) {
    pinMode(cfg.lidar_gpio_power, OUTPUT);
    digitalWrite(cfg.lidar_gpio_power, LOW);
  }

  setupLIDAR();
  setupADC();
  setupMotors();

  // El LED RGB solo esta disponible hasta imu.begin(): comparten el GPIO 48
  rgb_led.begin();

  // Transporte serial: el agente esta al otro lado del mismo cable por el que
  // se programa la placa. No hay red que levantar ni IP que resolver, asi que
  // el arranque ya no depende de nada externo salvo del propio agente.
  setBootState(BOOT_AGENT_SEARCHING);
  set_microros_transports();

  // Con WiFi el robot arrancaba en IDLE y esperaba un SESSION_START por UDP,
  // porque el agente vivia en un contenedor que podia aparecer horas despues.
  // Sobre serial se espera al agente aqui mismo: sondear es barato y no hay
  // ambiguidad posible --si responde, hay sesion--. Se espera indefinidamente
  // en vez de reiniciar: un reinicio no acerca la aparicion del agente y
  // ademas pierde el motivo del reset anterior, que es lo que se diagnostica.
  uint32_t agent_wait_attempts = 0;
  while (rmw_uros_ping_agent(cfg.UROS_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
    agent_wait_attempts++;
    setBootState((agent_wait_attempts & 1) ?
      BOOT_AGENT_SEARCHING : BOOT_WIFI_RETRY);
    delay(500);
  }

  setupMicroROS(&twist_sub_callback);
  session_state = SESSION_ACTIVE;

  // Verde fijo: agente conectado
  setBootState(BOOT_READY);
  delay(200);

  // Apagar RGB — liberar GPIO 48 para el IMU
  rgb_led.turnOff();

  // Iniciar IMU ahora que el RGB está apagado
  if (!imu.begin(48, 47, 400000)) {
    DEBUG_PRINTLN("Error initializing IMU6500");
  } else {
    DEBUG_PRINTLN("IMU6500 initialized successfully");
  }

  //pubDiagnostics();

  rcl_ret_t rc = addROSParams();
  if (rc != RCL_RET_OK) {
    DEBUG_PRINT("addROSParams(");
    DEBUG_PRINT(") error ");
    DEBUG_PRINTLN(rc);
  }

  ros_config_params_changed = true;
  updateROSParams();
  DEBUG_PRINTLN("Micro-ROS initialized");
  cfg.robot_name = ""; // free up a little memory
  cfg.robot_web = ""; 
  //DEBUG_PRINT("Diagnostics pub ");
  //DEBUG_PRINTLN(pubDiagnostics() ? "OK" : "FAILED");
  //pubDiagnostics();
  
  resetTelemMsg();
  resetNexusMsg();

  // El LiDAR solo gira durante una sesion: esta encendido 24/7 y su motor se
  // desgasta. Arranca en ~2 s cuando llega SESSION_START.
  if (session_state == SESSION_ACTIVE)
    startLIDAR();
  else
    DEBUG_PRINTLN("LiDAR en espera (sin sesion activa)");
    //blink_error_code(cfg.ERR_LIDAR_START);
    //error_loop(cfg.ERR_LIDAR_START);
}