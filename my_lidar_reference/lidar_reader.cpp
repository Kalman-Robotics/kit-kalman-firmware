#include "lidar_reader.hpp"

//////LIDARPOINT

LidarPoint::LidarPoint(uint16_t distance, uint8_t intensity, float angle)
    : _distance(distance), _intensity(intensity), _angle(angle) {}

String LidarPoint::toString() const {
  String result = "(distance=";
  result += String(_distance);
  result += ", intensity=";
  result += String(_intensity);
  result += ", angle=";
  result += String(_angle);
  result += ")";
  return result;
}

//////FUNCTIONS

std::vector<LidarPoint> getPoints() {
  std::vector<LidarPoint> points;
  if (!SerialLidar.find("T,")) {  // equivalent en char de 84 44 (decimal)
    Serial.println("lidar_reader.getPoints : error, no header-verlen found in RX for the LiDAR LD19");
  } else {
    // The previous instruction (find) jumped to the beginning of the information
    // Now the stream is aligned
    byte buffer[45];
    size_t nbrBytesReceived = SerialLidar.readBytes(buffer, 45);
    if (nbrBytesReceived != 45) {
      Serial.println("lidar_reader.getPoints : error, wrong number of bytes received (" + String((uint32_t) nbrBytesReceived) + ")");
    } else {
      uint16_t speed = _get2BytesLsbMsb(buffer, 0);
      uint16_t startAngle = _get2BytesLsbMsb(buffer, 2);

      LidarPoint data[] = {// no for loop possible due to 'const' in LidarPoint class
                            LidarPoint(_get2BytesLsbMsb(buffer, 4), buffer[6], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 7), buffer[9], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 10), buffer[12], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 13), buffer[15], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 16), buffer[18], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 19), buffer[21], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 22), buffer[24], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 25), buffer[27], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 28), buffer[30], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 31), buffer[33], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 34), buffer[36], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 37), buffer[39], 0)};

      uint16_t endAngle = _get2BytesLsbMsb(buffer, 40);
      uint16_t timestamp = _get2BytesLsbMsb(buffer, 42);
      uint8_t crcCheck = buffer[44];

      if (_calCRC8FromBuffer(buffer, 44) == crcCheck) {
        uint16_t step = angleStep(startAngle, endAngle);
        for (unsigned int i = 0; i < 12; i++) {
          points.push_back(
              LidarPoint(
                  data[i].distance(),
                  data[i].intensity(),
                  angleFromStep(startAngle, step, i)));
        }
      }
    }
  }
  return points;
}

uint8_t _calCRC8FromBuffer(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
  uint8_t crc = 0xD8;                                       // pre-calculated header and verlen values (crc = crcTable[(crc ^ 0x54) & 0xff];crc = crcTable[(crc ^ 0x2C) & 0xff];)
  for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {  // ignores the last value of the p array (which contains the crc check value)
    crc = crcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
  return (buffer[index + 1] << 8) | buffer[index];
}

uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
  if (startAngle <= endAngle) {
    return (endAngle - startAngle) / lenMinusOne;
  } else {
    return (36000 + endAngle - startAngle) / lenMinusOne;
  }
}

uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
  return (startAngle + (step * indice)) % 36000;
}