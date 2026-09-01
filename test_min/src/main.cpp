// Lo minimo que puede publicar por micro-ROS serial. Nada mas.
// Sin logs, sin subscriber, sin LED, sin fail_loop.
//
// micro-ROS va por Serial = UART0 = chip CH343.
// En la Raspberry:  /dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00
//
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 115200
//   ros2 topic echo /min_counter

#include <Arduino.h>
#include <micro_ros_kaia.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t pub;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void setup() {
  set_microros_transports();

  // El agente necesita ver el puerto estable antes del primer intento
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Si algo falla se reinicia y vuelve a intentarlo. Sin logs no hay a quien
  // reportar, y reintentar es lo unico util que se puede hacer.
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    delay(1000); ESP.restart();
  }
  if (rclc_node_init_default(&node, "min_node", "", &support) != RCL_RET_OK) {
    delay(1000); ESP.restart();
  }
  if (rclc_publisher_init_default(&pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/min_counter") != RCL_RET_OK) {
    delay(1000); ESP.restart();
  }

  msg.data = 0;
}

void loop() {
  msg.data++;
  rcl_publish(&pub, &msg, NULL);
  delay(1000);
}
