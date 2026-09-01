// Prueba minima del transporte serial de micro-ROS.
//
// No toca motores, LiDAR, IMU ni SPIFFS: solo abre el enlace y publica. Si
// esto funciona y el firmware completo no, el problema esta en el firmware;
// si esto tampoco funciona, el problema es el transporte, el cable o el
// agente, y no hay que buscar mas lejos.
//
// Usa los DOS puertos USB de la placa a la vez:
//   conector UART (chip CH343) -> Serial     -> tramas micro-ROS
//   conector USB  (nativo S3)  -> USBSerial  -> logs legibles
// Son caminos independientes, asi que los logs ya no corrompen las tramas.
//
// Publica:
//   /test_counter  (std_msgs/Int32)   contador a 1 Hz
//   /test_status   (std_msgs/String)  uptime y heap libre, a 1 Hz
// Escucha:
//   /test_echo     (std_msgs/Int32)   lo recibido se refleja en /test_status
//
// Agente:
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600

#include <Arduino.h>
#include <micro_ros_kaia.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

// El LED de sistema es la unica salida visible: con el UART ocupado por las
// tramas XRCE-DDS no hay Serial.print que valga.
#define LED_SYS_GPIO 11

// Logs por el USB nativo del S3. Independiente del UART0 que lleva micro-ROS,
// asi que se puede imprimir libremente con el agente conectado.
#define LOG(...)    Serial.print(__VA_ARGS__)
#define LOGLN(...)  Serial.println(__VA_ARGS__)

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t counter_pub;
rcl_publisher_t status_pub;
rcl_subscription_t echo_sub;

std_msgs__msg__Int32  counter_msg;
std_msgs__msg__Int32  echo_msg;
std_msgs__msg__String status_msg;

static char status_buf[128];
static int32_t last_echo = 0;
static uint32_t echo_count = 0;

// Parpadeo de diagnostico: n destellos, para distinguir en que paso fallo
void blink(uint8_t n, uint16_t on_ms) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(LED_SYS_GPIO, HIGH);
    delay(on_ms);
    digitalWrite(LED_SYS_GPIO, LOW);
    delay(on_ms);
  }
}

// Sin agente no hay a quien reportar el fallo: se deja el patron parpadeando
// para poder leer el paso que fallo desde fuera.
void fail_loop(uint8_t code) {
  static const char * const kStep[] = {
    "", "rclc_support_init (no encuentra al agente)", "node_init",
    "publisher_init", "subscription_init", "executor_init" };
  while (true) {
    LOG("FALLO paso ");
    LOG(code);
    LOG(": ");
    LOGLN(code <= 5 ? kStep[code] : "desconocido");
    blink(code, 150);
    delay(1200);
  }
}

void echo_callback(const void * msgin) {
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
  last_echo = m->data;
  echo_count++;
}

void setup() {
  pinMode(LED_SYS_GPIO, OUTPUT);
  digitalWrite(LED_SYS_GPIO, LOW);

  // Puerto nativo para logs. El baudrate es indiferente: en USB no hay reloj
  // que acordar, cualquier velocidad del monitor sirve.
  Serial.begin(115200);
  delay(500);   // margen para que el host enumere el puerto
  LOGLN();
  LOGLN("=== prueba de transporte serial micro-ROS ===");
  LOGLN("micro-ROS: conector USB nativo (COM9) | estos logs: UART CH343 (COM8)");

  // Abre el UART a 921600. A partir de aqui el puerto lleva tramas XRCE-DDS:
  // cualquier Serial.print lo corrompe.
  set_microros_transports();
  LOGLN("transporte USB-CDC abierto, esperando al agente...");

  // Margen para que el agente detecte el puerto tras el reset de la placa
  delay(2000);

  allocator = rcl_get_default_allocator();

  // 1 destello: no encuentra al agente
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    fail_loop(1);

  // 2: el agente respondio pero rechazo el nodo
  LOGLN("agente encontrado, creando nodo");

  if (rclc_node_init_default(&node, "kalman_serial_test", "", &support) != RCL_RET_OK)
    fail_loop(2);

  // 3: fallo la creacion de publishers
  if (rclc_publisher_init_default(&counter_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/test_counter") != RCL_RET_OK)
    fail_loop(3);

  if (rclc_publisher_init_default(&status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/test_status") != RCL_RET_OK)
    fail_loop(3);

  // 4: fallo el subscriber. Se prueba tambien la direccion agente -> placa:
  // publicar funciona con el enlace medio roto, recibir no.
  if (rclc_subscription_init_default(&echo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/test_echo") != RCL_RET_OK)
    fail_loop(4);

  // 5: fallo el executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    fail_loop(5);

  if (rclc_executor_add_subscription(&executor, &echo_sub, &echo_msg,
        &echo_callback, ON_NEW_DATA) != RCL_RET_OK)
    fail_loop(5);

  // El String de micro-ROS no reserva memoria: apunta al buffer estatico
  status_msg.data.data = status_buf;
  status_msg.data.capacity = sizeof(status_buf);
  status_msg.data.size = 0;

  counter_msg.data = 0;

  LOGLN("nodo y entidades creados; publicando a 1 Hz");
  LOGLN("comprueba con:  ros2 topic list");

  // 3 destellos rapidos: todo creado, entrando al bucle
  blink(3, 80);
}

void loop() {
  static unsigned long last_pub_ms = 0;

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (millis() - last_pub_ms >= 1000) {
    last_pub_ms = millis();

    counter_msg.data++;
    rcl_publish(&counter_pub, &counter_msg, NULL);

    int n = snprintf(status_buf, sizeof(status_buf),
      "{\"uptime_s\":%lu,\"heap\":%lu,\"echo_rx\":%lu,\"last_echo\":%ld}",
      (unsigned long)(millis() / 1000),
      (unsigned long)ESP.getFreeHeap(),
      (unsigned long)echo_count,
      (long)last_echo);
    if (n > 0) {
      status_msg.data.size = (n < (int)sizeof(status_buf)) ?
        (size_t)n : sizeof(status_buf) - 1;
      rcl_publish(&status_pub, &status_msg, NULL);
      // El mismo dato que va al topico, tambien legible aqui: permite ver si
      // la placa publica aunque el agente no muestre nada del otro lado.
      LOG("pub #");
      LOG(counter_msg.data);
      LOG("  ");
      LOGLN(status_buf);
    }

    // Latido: un destello corto por publicacion
    digitalWrite(LED_SYS_GPIO, HIGH);
    delay(30);
    digitalWrite(LED_SYS_GPIO, LOW);
  }
}
