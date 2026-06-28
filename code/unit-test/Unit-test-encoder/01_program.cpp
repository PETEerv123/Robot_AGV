#include "01_program.h"
#include "02_Encoder.h"
#include "define.h"
#include <Arduino.h>


/*Private Marco*/

Encoder_Handle Encoder_FL; //fornt left
Encoder_Handle Encoder_FR; //fornt right
Encoder_Handle Encoder_BL; //back  left
Encoder_Handle Encoder_BR; //back  right

/*Private Variables*/
void set_up(void) {
  Serial.begin(115200);
  Serial.println("Unit test speed");
  Encoder_FL.pinA = PIN_ENCODER_FL_A;
  Encoder_FL.pinB = PIN_ENCODER_FL_B;
  Encoder_FL.id = 0;  // bật ngắt cho id 0
  Encoder_Init(&Encoder_FL);
  
  Encoder_FR.pinA = PIN_ENCODER_FR_A;
  Encoder_FR.pinB = PIN_ENCODER_FR_B;
  Encoder_FR.id = 1;  // bật ngắt cho id 1
  Encoder_Init(&Encoder_FR);
  
  Encoder_BL.pinA = PIN_ENCODER_BL_A;
  Encoder_BL.pinB = PIN_ENCODER_BL_B;
  Encoder_BL.id = 2;  // bật ngắt cho id 2
  Encoder_Init(&Encoder_BL);
  
  Encoder_BR.pinA = PIN_ENCODER_BR_A;
  Encoder_BR.pinB = PIN_ENCODER_BR_B;
  Encoder_BR.id = 3;  // bật ngắt cho id 3
  Encoder_Init(&Encoder_BR);
}
void main_loop(void) {
  char buffer[250];
  sprintf(buffer, "Encoder FL: %ld, Encoder FR: %ld, Encoder BL: %ld, Encoder BR: %ld", 
          Encoder_GetPosition(&Encoder_FL), 
          Encoder_GetPosition(&Encoder_FR), 
          Encoder_GetPosition(&Encoder_BL), 
          Encoder_GetPosition(&Encoder_BR));
  Serial.println(buffer);
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
