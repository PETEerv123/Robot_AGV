#include "01_program.h"
#include "02_Encoder.h"
#include <Arduino.h>


/*Private Marco*/

Encoder_Handle Encoder_FL;


/*Private Variables*/
void set_up(void) {
  Serial.begin(115200);
  Serial.println("Unit test speed");
  Encoder_FL.pinA = PIN_ENCODER_FL_A;
  Encoder_FL.pinB = PIN_ENCODER_FL_B;
  Encoder_FL.id = 0;  // bật ngắt cho id 0
  Encoder_Init(&Encoder_FL);
}
void main_loop(void) {
  Serial.println(Encoder_GetPosition(&Encoder_FL));
  // unsigned long now = millis();
  // float dt = (now - lastTime) / 1000.0;
  // lastTime = now;
  // long ticks = Encoder_FL.read();6311
  // long delta = ticks - prevTicks;
  // prevTicks = ticks;

  // float rev = delta / TICKS_PER_REV;
  // float vCurrent = (rev * WHEEL_CIRC) / dt;
}
// void Event_SerialPrint(void){
//     char
// }
