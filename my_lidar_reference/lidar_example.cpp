#include <Arduino.h>
#include "lidar_reader.hpp"
#include <vector>

// #define SerialLidar Serial2  // Use Serial2 for LD19 (change if needed)
HardwareSerial& SerialLidar = Serial2;  // Use Serial2 for LD19
void setup() {
  Serial.begin(115200);      // For logging
  SerialLidar.begin(230400, SERIAL_8N1, 37, -1); // LD19 baud rate

  Serial.println("LD19 LiDAR test starting...");
}

void loop() {
  // Try to get points from the LiDAR
  std::vector<LidarPoint> points = getPoints();

  if (points.size() == 12) { // LD19 always sends 12 points per packet
    Serial.println("Received 12 points:");
    for (const auto& pt : points) {
      Serial.println(pt.toString());
    }
  } else {
    Serial.println("No valid LiDAR packet received.");
  }

  delay(200); // Wait before next read
}