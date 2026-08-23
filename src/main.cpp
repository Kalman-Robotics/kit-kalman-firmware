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
#include "util.h"
#include "session.h"
#include "diag.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <stdio.h>
#include "motors.h"
#include "ap.h"
#include "lidar.h"
#include "ros.h"
#include "adc.h"
#include "IMU6500.h"
#include "buzzer.h"
#include "led_rgb.h"
#include <SPIFFS.h>

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
bool spinWiFi();
void spinSession();
void spinSessionLed();
// Motivo del ultimo reset y salud del bucle: lo unico que quedo de la
// instrumentacion de las fases 2-4. El resto se quito por no aportar al
// diagnostico y degradar el bucle.
esp_reset_reason_t g_rst_reason = ESP_RST_UNKNOWN;
uint32_t g_loop_max_ms = 0;
uint32_t g_loop_max_ms_since_report = 0;

// Contadores del enlace; ninguno dispara acciones por su cuenta
uint32_t g_ping_fails = 0;
uint32_t g_agent_lost = 0;
uint32_t g_wifi_drops = 0;
uint32_t g_cmd_vel_rx = 0;

WiFiUDP diag_udp;

char     g_last_cmd[24] = {0};
int64_t  g_last_cmd_us = 0;
uint32_t g_cmd_count = 0;
bool     g_ota_active = false;

// Estado de la sesion de laboratorio. Ver include/session.h para el protocolo.
SessionLink session_link;
session_state_t session_state = SESSION_ACTIVE;
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
  //Serial.print("linear.x ");
  //Serial.print(msg->linear.x);
  //Serial.print(", angular.z ");
  //Serial.println(msg->angular.z);

  if (msg->linear.y != 0) {
    Serial.print("Warning: /cmd_vel linear.y = ");
    Serial.print(msg->linear.y);
    Serial.println(" not zero");
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

  //Serial.print(", twist_target_rpm_right ");
  //Serial.print(twist_target_rpm_right);
  //Serial.print(", twist_target_rpm_left ");
  //Serial.print(twist_target_rpm_left);

  // Limit target RPM
  float limited_target_rpm_right =
    absMin(twist_target_rpm_right, motorRight.getMaxRPM());
  float limited_target_rpm_left =
    absMin(twist_target_rpm_left, motorLeft.getMaxRPM());

  //Serial.print(", limited_target_rpm_right ");
  //Serial.print(limited_target_rpm_right);
  //Serial.print(", limited_target_rpm_left ");
  //Serial.print(limited_target_rpm_left);

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

  //Serial.print(", ramp_target_rpm_ri  ght ");
  //Serial.print(ramp_target_rpm_right);
  //Serial.print(", ramp_target_rpm_left ");
  //Serial.println(ramp_target_rpm_left);

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
    Serial.println(", restarting...");
    delay(100);
    ESP.restart();
    return "";
  } else {
    text = text + String(param_name) + ": " + String(param_value) + '\n';
    return strcmp(param_name, "pass") == 0 ? "****" : String(param_value);
  }
}

static inline bool initWiFi(const String & ssid, const String & passw,
  boot_state_t led_state = BOOT_WIFI_CONNECTING) {

  // Tras un reinicio por software el modulo conserva estado del arranque
  // anterior; reasociar encima deja el stack WiFi inconsistente. Bajar la
  // sesion antes de cada intento evita quedarse a medio asociar.
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  // El modo de ahorro de energia hace dormir la radio entre beacons, lo que
  // agrega latencia a cada paquete UDP de micro-ROS
  WiFi.setSleep(false);

  // IP fija: se saltea el DHCP, que agrega entre 200 ms y 2 s al arranque y es
  // una fuente de fallos intermitentes. Vacio en config.yaml = DHCP.
  if (cfg.static_ip.length() > 0) {
    IPAddress ip, gw, mask;
    if (!ip.fromString(cfg.static_ip)) {
      Serial.print("Invalid robot.wifi.static_ip: ");
      Serial.println(cfg.static_ip);
      return false;
    }
    // Sin gateway explicito se asume el AP; sin mascara, /24
    if (!gw.fromString(cfg.gateway_ip))
      gw = IPAddress(ip[0], ip[1], ip[2], 1);
    if (!mask.fromString(cfg.subnet_mask))
      mask = IPAddress(255, 255, 255, 0);

    // DNS = gateway: el firmware solo habla con el agente por IP, pero sin DNS
    // configurado algunas versiones del stack retrasan el DHCP igual
    if (!WiFi.config(ip, gw, mask, gw)) {
      Serial.println("WiFi.config() failed, falling back to DHCP");
    } else {
      Serial.print("Static IP ");
      Serial.print(cfg.static_ip);
      Serial.print(" gw ");
      Serial.println(gw);
    }
  }

  WiFi.begin(ssid, passw);

  // Sondear rapido y parpadear lento: con un delay() largo por vuelta el
  // WiFi puede quedar asociado hasta 1 s antes de que el bucle lo note.
  const uint32_t poll_delay_ms = 50;
  const uint32_t blink_period_ms = 500;
  unsigned long startMillis = millis();
  unsigned long last_blink_ms = 0;
  bool blink_on = false;

  Serial.println();
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  Serial.print(" ...");

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startMillis >= cfg.WIFI_CONN_TIMEOUT_MS) {
      Serial.println(" timed out");
      return false;
    }

    if (millis() - last_blink_ms >= blink_period_ms) {
      last_blink_ms = millis();
      blink_on = !blink_on;
      setBootState(led_state);
      digiWrite(cfg.led_sys_gpio, blink_on ? HIGH : LOW, cfg.led_sys_invert);
    }
    delay(poll_delay_ms);
  }

  // WL_CONNECTED solo dice que la asociacion cerro; el DHCP puede seguir en
  // curso. Arrancar micro-ROS sin IP valida es una de las causas de quedarse
  // colgado buscando al agente.
  unsigned long ip_wait_start_ms = millis();
  while (WiFi.localIP() == INADDR_NONE) {
    if (millis() - ip_wait_start_ms >= cfg.WIFI_DHCP_TIMEOUT_MS) {
      Serial.println(" connected but no IP assigned");
      return false;
    }
    setBootState(BOOT_WIFI_NO_IP);
    delay(20);
  }

  digiWrite(cfg.led_sys_gpio, LOW, cfg.led_sys_invert);
  Serial.print(" connected, ");
  Serial.print("IP ");
  Serial.println(WiFi.localIP());
  //printWiFiChannel();
  return true;
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
  //  Serial.print("RPM L ");
  //  Serial.print(motorLeft.getCurrentRPM());
  //  Serial.print(" R ");
  //  Serial.println(motorRight.getCurrentRPM());
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
    Serial.print("rcl_publish(control_status) error ");
    Serial.println(rc);
  }
}

void publishTelem(int64_t step_time_us) {
  struct timespec tv = {0, 0};
  clock_gettime(CLOCK_REALTIME, &tv);
  nexus_msg.stamp.sec = tv.tv_sec;
  nexus_msg.stamp.nanosec = tv.tv_nsec;

  float joint_pos_delta[MOTOR_COUNT];
  float step_time = 1e-6 * (float)step_time_us;

  long rssi_dbm = WiFi.RSSI();
  rssi_dbm = rssi_dbm > 127 ? 127 : rssi_dbm;
  rssi_dbm = rssi_dbm < -128 ? -128 : rssi_dbm;
  nexus_msg.wifi_rssi_dbm = (int8_t) rssi_dbm;

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
    Serial.print("rcl_publish(nexus_msg) error ");
    Serial.println(rc);
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

void spinPing() {
  static uint8_t ping_fail_count = 0;
  int64_t time_now_us = esp_timer_get_time();
  int64_t step_time_us = time_now_us - ping_prev_pub_time_us;

  if (step_time_us < cfg.UROS_PING_PUB_PERIOD_US)
    return;

  ping_prev_pub_time_us = time_now_us;

  // Sin WiFi el ping no puede salir: no gastar el presupuesto de fallos aqui,
  // de eso se encarga la reconexion de spinWiFi()
  if (WiFi.status() != WL_CONNECTED)
    return;

  rmw_ret_t rc = rmw_uros_ping_agent(cfg.UROS_PING_TIMEOUT_MS, 1);
  if (rc != RMW_RET_OK) {
    g_ping_fails++;
    Serial.print("Ping failed (");
    Serial.print(++ping_fail_count);
    Serial.print("/");
    Serial.print(cfg.UROS_PING_MAX_FAILS);
    Serial.println(")");
    if (ping_fail_count >= cfg.UROS_PING_MAX_FAILS &&
        session_state == SESSION_ACTIVE) {
      // No se reinicia aca: puede ser un corte de red o un reinicio del
      // contenedor, y en ese caso el agente vuelve en segundos. Se entra en
      // GRACE y se espera; si fue fin de sesion, la Raspberry avisa
      // SESSION_END y se corta la espera de inmediato.
      Serial.println("micro-ROS agent lost, entering GRACE");
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
      Serial.println("micro-ROS agent recovered, back to ACTIVE");
      session_state = SESSION_ACTIVE;
      lidar->start();
    }
    ping_fail_count = 0;
  }
}

void updateROSParams() {
  if (ros_config_params_changed) {
    ros_config_params_changed = false;
    rcl_ret_t ret = updateROSConfigParams();
    if (ret != RCL_RET_OK) {
      Serial.print("updateROSConfigParams() error ");
      Serial.println(ret);
    }
  }

  int64_t time_now_us = esp_timer_get_time();
  int64_t step_time_us = time_now_us - ros_params_update_prev_time_us;
  if (step_time_us >= cfg.UROS_PARAMS_UPDATE_PERIOD_US) {

    rcl_ret_t ret = updateROSRealTimeParams();
    if (ret != RCL_RET_OK) {
      Serial.print("updateROSRealTimeParams() error ");
      Serial.println(ret);
    }

    ros_params_update_prev_time_us = time_now_us;
  }
}

// Vigila el WiFi durante la operacion y lo reconecta activamente. Antes esto
// quedaba librado a la reconexion automatica del stack del ESP32; si esa falla
// el robot se queda con motores y LiDAR parados de forma indefinida.
// No bloquea: cada llamada hace un solo paso y vuelve al loop().
bool spinWiFi() {
  static bool wifi_ok_prev = true;
  static unsigned long wifi_lost_ms = 0;
  static unsigned long last_retry_ms = 0;

  bool wifi_ok = WiFi.status() == WL_CONNECTED;

  if (wifi_ok && !wifi_ok_prev) {
    Serial.print("WiFi connection restored, IP ");
    Serial.println(WiFi.localIP());
    // El socket quedo atado a la IP anterior
    session_link.restart();
    if (session_state == SESSION_ACTIVE)
      lidar->start();
    wifi_lost_ms = 0;
  } else if (!wifi_ok && wifi_ok_prev) {
    Serial.println("WiFi connection lost: pausing motors, LiDAR");
    g_wifi_drops++;
    setMotorSpeeds(0, 0);
    lidar->stop();
    wifi_lost_ms = millis();
    last_retry_ms = millis();
  }
  wifi_ok_prev = wifi_ok;

  if (!wifi_ok) {
    // Tras el plazo de gracia se reinicia: el socket de micro-ROS quedo atado a
    // la sesion anterior, asi que reconectar el WiFi solo no alcanza para
    // recuperar la sesion con el agente.
    if (millis() - wifi_lost_ms >= cfg.WIFI_RECONNECT_TIMEOUT_MS) {
      Serial.println("WiFi not recovered, restarting...");
      Serial.flush();
      ESP.restart();
    }

    if (millis() - last_retry_ms >= cfg.WIFI_RECONNECT_RETRY_MS) {
      last_retry_ms = millis();
      Serial.println("Reconnecting to WiFi ...");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      WiFi.begin(cfg.ssid, cfg.pass);
    }
  }

  return wifi_ok;
}

static unsigned long session_idle_poll_ms = 0;

// Vuelve a esperar una sesion. micro-ROS no libera limpiamente sus recursos,
// asi que la unica forma segura de dejar el cliente listo para la proxima
// sesion es reiniciar. Se hace aca, cuando ya no hay alumno esperando, y no
// durante GRACE, donde el reinicio costaria segundos de reconexion.
void sessionToIdle(const char * reason) {
  Serial.print("Session -> IDLE (");
  Serial.print(reason);
  Serial.println("), restarting to await next session");
  setMotorSpeeds(0, 0);
  lidar->stop();
  Serial.flush();
  delay(200);
  ESP.restart();
}

// Procesa los avisos de la Raspberry y administra el periodo de gracia.
// Puentes usados por diag.h para atender los comandos de control
const char * diagSessionState() { return sessionStateName(session_state); }

void diagStopMotors() {
  ramp_target_rpm_right = 0;
  ramp_target_rpm_left = 0;
  setMotorSpeeds(0, 0);
}

void diagResetOdom() {
  nexus_msg.odom_pos_x = 0;
  nexus_msg.odom_pos_y = 0;
  nexus_msg.odom_pos_yaw = 0;
  nexus_msg.odom_vel_x = 0;
  nexus_msg.odom_vel_yaw = 0;
}

// Ultimos 4 hex de la MAC
String diagRebootToken() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char t[5];
  snprintf(t, sizeof(t), "%02X%02X", mac[4], mac[5]);
  return String(t);
}

float diagOdomX()   { return nexus_msg.odom_pos_x; }
float diagOdomYaw() { return nexus_msg.odom_pos_yaw; }

void diagNoteCmd(const String & cmd) {
  strncpy(g_last_cmd, cmd.c_str(), sizeof(g_last_cmd) - 1);
  g_last_cmd[sizeof(g_last_cmd) - 1] = 0;
  g_last_cmd_us = esp_timer_get_time();
  g_cmd_count++;
}

// Actualizacion por WiFi. La tabla default_8MB ya reserva dos particiones de
// aplicacion de 3.34 MB cada una y la imagen ocupa 1.13 MB, asi que no hay que
// reorganizar el flash: OTA escribe en la particion inactiva y, si la carga se
// corta a mitad, el bootloader sigue arrancando la anterior.
void setupOTA() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char host[32];
  snprintf(host, sizeof(host), "esp32s3-%02X%02X%02X",
           mac[3], mac[4], mac[5]);

  ArduinoOTA.setHostname(host);
  ArduinoOTA.setPassword(cfg.OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    // Parar todo antes de flashear: la escritura bloquea el loop varios
    // segundos y un robot en movimiento no tendria quien lo frene
    Serial.println("[OTA] inicio: parando motores y LiDAR");
    ramp_target_rpm_right = 0;
    ramp_target_rpm_left = 0;
    setMotorSpeeds(0, 0);
    lidar->stop();
    // Sin esto el watchdog de RX podria provocar un panic a mitad de la carga
    g_ota_active = true;
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] completado, reiniciando");
    Serial.flush();
  });

  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static uint8_t last_pct = 255;
    uint8_t pct = total ? (done * 100) / total : 0;
    if (pct != last_pct && pct % 10 == 0) {
      last_pct = pct;
      Serial.print("[OTA] ");
      Serial.print(pct);
      Serial.println("%");
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print("[OTA] error ");
    Serial.println(error);
    // La carga fallo: el bootloader seguira arrancando la particion anterior
    g_ota_active = false;
    lidar->start();
  });

  ArduinoOTA.begin();
  Serial.print("OTA activo, hostname ");
  Serial.println(host);
}

void spinSession() {
  String event;
  if (session_link.poll(event, session_state)) {
    if (event == "SESSION_START") {
      if (session_state == SESSION_GRACE) {
        // El agente volvio dentro del periodo de gracia: el ping lo detecta y
        // vuelve a ACTIVE por su cuenta, aca solo se registra
        Serial.println("Session: START recibido durante GRACE");
      } else if (session_state == SESSION_IDLE) {
        Serial.println("Session: START recibido, reiniciando para conectar");
        Serial.flush();
        delay(200);
        ESP.restart();
      }
    } else if (event == "SESSION_END") {
      // Corta el periodo de gracia: no fue un corte de red, la sesion termino
      if (session_state != SESSION_IDLE)
        sessionToIdle("SESSION_END");
    }
    // PING y cualquier otro evento ya recibieron su ACK en poll()
  }

  // Una sola vez: en modo diagnostico DIAG_RESTART no reinicia, y sin esta
  // guarda sessionToIdle() se reintentaba en cada iteracion del loop --365
  // veces en una prueba--, con su setMotorSpeeds + lidar->stop + delay(200)
  // cada vez. Eso degradaba el bucle de 56 a 265 ms y convertia un corte de
  // 16 s en un estado roto permanente.
  static bool idle_attempted = false;
  if (session_state != SESSION_GRACE)
    idle_attempted = false;

  if (session_state == SESSION_GRACE && !idle_attempted &&
      millis() - session_grace_start_ms >= cfg.SESSION_GRACE_MS) {
    idle_attempted = true;
    // Se agoto la espera sin que la Raspberry avisara nada: se asume fin de
    // sesion no anunciado
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
  bool wifi_ok = spinWiFi();

  // Coste de OTA en el bucle: se mide aparte para poder compararlo contra el
  // baseline sin OTA y decidir si vale la pena dejarlo siempre activo
  // Escuchar el socket de OTA. Medido en 91 us por llamada, pero como los
  // otros sondeos UDP no hace falta en cada iteracion: una carga reintenta la
  // invitacion, asi que 50 ms no la impiden.
  {
    static unsigned long last_ota_ms = 0;
    if (g_ota_active || millis() - last_ota_ms >= 50) {
      last_ota_ms = millis();
      ArduinoOTA.handle();
    }
  }
  lidar->loop();

  // Process micro-ROS callbacks
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  if (ret != RCL_RET_OK) {
    Serial.print("rclc_executor_spin_some() error ");
    Serial.println(ret);
  }

  updateROSParams();
  int64_t time_now_us = esp_timer_get_time();
  spinIMU(time_now_us);
  spinTelem(false);
  spinControlStatus();
  spinPing();
  // Estos tres sondean sockets UDP; hacerlo en cada iteracion le quitaba
  // tiempo al drenado del UART del LiDAR. Ver el limitador dentro de cada uno.
  spinSession();
  spinSessionLed();
  // diagSpin() incluye el watchdog de RX, que daba falsos positivos: su
  // testigo se alimenta sobre todo de la respuesta del ping (1 Hz) y
  // envejecia con la red sana. Se conserva el reporte por UDP, que es pasivo.
  diagSpin();

  if (!wifi_ok) {
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

  diagLoopTick();
}

bool isBootButtonPressed(uint8_t sec) {
  if (digiRead(cfg.button_boot_gpio, cfg.button_boot_invert))
    Serial.println("BOOT button pressed. Keep pressing for web config.");
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
    Serial.print("rcl_publish(imu_msg");
    Serial.print(") error ");
    Serial.println(rc);
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
  Serial.print("Blinking LED ");
  Serial.print(n_blinks);
  Serial.println(" times...");

  blink_error_code(n_blinks);

  Serial.println("Rebooting...");
  Serial.flush();

  ESP.restart();
}
*/

void setup() {

  // Silence buzzer — active-low: INPUT = high impedance = silent
  pinMode(PIN_BUZZER, INPUT);

  bool spiffs_ok = SPIFFS.begin(true);
//  blink_error_code(cfg.ERR_SPIFFS_INIT);
  bool html_exists = false;
  if (spiffs_ok)
    html_exists = SPIFFS.exists(cfg.INDEX_HTML_PATH);

  bool wifi_yaml_exists = SPIFFS.exists(cfg.NETWORK_YAML_PATH);
  String wifi_yaml_err;
  if (wifi_yaml_exists)
    wifi_yaml_err = cfg.load(cfg.NETWORK_YAML_PATH);

  bool config_yaml_exists = SPIFFS.exists(cfg.CONFIG_YAML_PATH);
  String config_yaml_err;
  if (config_yaml_exists)
    config_yaml_err = cfg.load(cfg.CONFIG_YAML_PATH);

  Serial.begin(cfg.MONITOR_BAUD);
  setPinDrive(cfg.monitor_gpio_tx);
  while(!Serial)
    delay(0);

  // Antes que nada: el motivo del reset anterior. Distingue un ESP.restart()
  // del propio codigo de un watchdog, un panic o un brownout, que es lo unico
  // que no se puede deducir desde la Raspberry.
  g_rst_reason = esp_reset_reason();

  Serial.println();
  Serial.print("kalman.ai firmware version ");
  Serial.println(cfg.FW_VERSION);

  Serial.print("ESP IDF version ");
  Serial.println(esp_get_idf_version());

  if (spiffs_ok) {
    Serial.println("SPIFFS mounted successfully");
    if (!html_exists) {
      Serial.println("Sketch data not found. Please upload sketch data.");
      idle();
    }
  } else {
    Serial.println("Error mounting SPIFFS");
    idle();
  }

  if (wifi_yaml_exists) {
    Serial.print(cfg.NETWORK_YAML_PATH);
    Serial.print(" found; ");
    if (wifi_yaml_err.length() != 0) {
      Serial.print("error parsing: ");
      Serial.println(wifi_yaml_err);
    } else
      Serial.println("loaded OK");
  }

  if (config_yaml_exists) {
    Serial.print(cfg.CONFIG_YAML_PATH);
    Serial.print(" found; ");
    if (config_yaml_err.length() != 0) {
      Serial.print("error parsing: ");
      Serial.println(config_yaml_err);
    } else
      Serial.println("loaded OK");
  }

  setPinMode(cfg.led_sys_gpio, OUTPUT);
  digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);

  setPinMode(cfg.button_boot_gpio, INPUT);

  bool launch_web_config = false;

  if (cfg.use_web) {
    Serial.println("Web configuration mode enabled (robot.use_web: true)");    
    // Load network.yaml if it exists (for backward compatibility)
    if (wifi_yaml_exists && wifi_yaml_err.length() == 0) {
      Serial.println("Using WiFi credentials from network.yaml");
    } else {
      if (cfg.ssid.length() == 0) {
        Serial.println("WiFi SSID unknown");
        launch_web_config = true;
      }
      if (cfg.dest_ip.length() == 0) {
        Serial.println("dest_ip unknown");
        launch_web_config = true;
      }
    }
    Serial.println("To enter web config push-and-release RST, "
      "then push-and-hold BOOT within 1 sec");
    delay(1000);
    launch_web_config |= isBootButtonPressed(cfg.RESET_SETTINGS_HOLD_SEC);
  } 
  else {
    // NEW: Direct configuration mode - bypass web config
    Serial.println("Web configuration disabled (robot.use_web: false)");
    Serial.println("Using WiFi and micro-ROS settings from config.yaml");    
    if (cfg.ssid.length() == 0) {
      Serial.println("ERROR: WiFi SSID not configured in config.yaml");
      Serial.println("Please set robot.wifi.ssid in config.yaml");
      idle();
    }
    if (cfg.dest_ip.length() == 0) {
      Serial.println("ERROR: micro-ROS agent IP not configured in config.yaml");
      Serial.println("Please set robot.computer.ip in config.yaml");
      idle();
    }
    Serial.print("WiFi SSID: ");
    Serial.println(cfg.ssid);
    Serial.print("micro-ROS agent: ");
    Serial.print(cfg.dest_ip);
    Serial.print(":");
    Serial.println(cfg.dest_port);    
    launch_web_config = false; // Force bypass web config
  }

  if (launch_web_config) {
    digiWrite(cfg.led_sys_gpio, HIGH, cfg.led_sys_invert);

    AP ap;
    ap.obtainConfig(cfg.robot_web.c_str(), set_param_callback);
    return;
  }

  Serial.print("Board model ");
  Serial.print(cfg.board_model);
  Serial.print(", version ");
  Serial.print(cfg.board_version);
  Serial.print(", manufacturer ");
  Serial.println(cfg.board_manufacturer);
  Serial.print("Robot name ");
  Serial.print(cfg.robot_name);
  Serial.print(", web ");
  Serial.println(cfg.robot_web);  
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

  // Azul parpadeante en el primer intento, violeta a partir del segundo, para
  // distinguir "conectando" de "reintentando tras un timeout"
  uint8_t wifi_attempt = 0;
  while(!initWiFi(cfg.ssid, cfg.pass,
    wifi_attempt == 0 ? BOOT_WIFI_CONNECTING : BOOT_WIFI_RETRY)) {
    wifi_attempt++;
    setBootState(BOOT_WIFI_RETRY);
    delay(500);
  }

  // Ambar: buscando agente micro-ROS. Fijo en el primer intento, parpadeante
  // en los siguientes; setupMicroROS() se encarga del timeout y del reinicio.
  setBootState(BOOT_AGENT_SEARCHING);

  // Canal de sesion con la Raspberry: se abre antes de buscar el agente para
  // poder recibir avisos incluso mientras se espera
  session_link.begin(cfg.SESSION_UDP_PORT);
  diagBegin();
  setupOTA();

  set_microros_wifi_transports(cfg.dest_ip.c_str(), cfg.dest_port);
  delay(100); // asentar el socket UDP; si no basta, setupMicroROS() reintenta

  setupMicroROS(&twist_sub_callback);

  // Verde fijo: agente conectado
  setBootState(BOOT_READY);
  delay(200);

  // Apagar RGB — liberar GPIO 48 para el IMU
  rgb_led.turnOff();

  // Iniciar IMU ahora que el RGB está apagado
  if (!imu.begin(48, 47, 400000)) {
    Serial.println("Error initializing IMU6500");
  } else {
    Serial.println("IMU6500 initialized successfully");
  }

  //pubDiagnostics();

  rcl_ret_t rc = addROSParams();
  if (rc != RCL_RET_OK) {
    Serial.print("addROSParams(");
    Serial.print(") error ");
    Serial.println(rc);
  }

  ros_config_params_changed = true;
  updateROSParams();
  Serial.println("Micro-ROS initialized");
  cfg.robot_name = ""; // free up a little memory
  cfg.robot_web = ""; 
  //Serial.print("Diagnostics pub ");
  //Serial.println(pubDiagnostics() ? "OK" : "FAILED");
  //pubDiagnostics();
  
  resetTelemMsg();
  resetNexusMsg();

  startLIDAR();
    //blink_error_code(cfg.ERR_LIDAR_START);
    //error_loop(cfg.ERR_LIDAR_START);
}