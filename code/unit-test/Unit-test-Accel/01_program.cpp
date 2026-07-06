#include "01_program.h"
#include "Adafruit_BNO08x.h"
#include "define.h"
#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <math.h>

/*Private Marco*/
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t imuValue;
/*Private Variables*/
void set_up(void) {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Unit test Sensor Accel");
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 not detected");
    while (1);
  }else {
    Serial.println("BNO085 Init Success");
  }
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

}
void main_loop(void) {
  if (bno08x.getSensorEvent(&imuValue)) {
    if (imuValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = imuValue.un.gameRotationVector.real;
      float qx = imuValue.un.gameRotationVector.i;
      float qy = imuValue.un.gameRotationVector.j;
      float qz = imuValue.un.gameRotationVector.k;

      float imuYaw = atan2( 2.0 * (qw * qz + qx * qy),1.0 - 2.0 * (qy * qy + qz * qz));
    }
  }
}
