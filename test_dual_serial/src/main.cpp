// Prueba de los dos puertos serie USB del ESP32-S3. Sin micro-ROS.
//
// Solo comprueba una cosa: que los dos conectores USB de la placa son caminos
// independientes y se puede escribir por ambos a la vez.
//
//   conector UART (chip CH343)  -> Serial     -> COM8
//   conector USB  (nativo S3)   -> USBSerial  -> COM9
//
// Abre los dos monitores serie (uno por COM) y deberias ver a cada puerto
// anunciarse por su nombre y contar. Si solo uno cuenta, ese conector es el
// unico vivo; si ambos cuentan, se pueden usar en paralelo --que es lo que
// hace falta para depurar con micro-ROS ocupando el UART--.

#include <Arduino.h>

#define LED_SYS_GPIO 11

void setup() {
  pinMode(LED_SYS_GPIO, OUTPUT);
  digitalWrite(LED_SYS_GPIO, LOW);

  // UART0, por el chip CH343
  Serial.begin(115200);

  // USB nativo del S3. El baudrate no importa: no hay reloj fisico que
  // acordar, el monitor puede abrirse a cualquier velocidad.
  USBSerial.begin();

  // Margen para que el host enumere ambos puertos antes de imprimir: sin esto
  // se pierden las primeras lineas y parece que el puerto no responde.
  delay(2000);

  Serial.println();
  Serial.println("=== Estoy en el conector UART (chip CH343) ===");
  Serial.println("Si lees esto, este puerto funciona.");

  USBSerial.println();
  USBSerial.println("=== Estoy en el conector USB nativo del S3 ===");
  USBSerial.println("Si lees esto, este puerto funciona.");
}

void loop() {
  static uint32_t n = 0;
  n++;

  // Cada puerto se identifica en cada linea: si por error ambos monitores
  // apuntan al mismo COM, se nota de inmediato.
  Serial.print("[UART CH343] contador ");
  Serial.println(n);

  USBSerial.print("[USB nativo] contador ");
  USBSerial.println(n);

  digitalWrite(LED_SYS_GPIO, HIGH);
  delay(100);
  digitalWrite(LED_SYS_GPIO, LOW);
  delay(900);
}
