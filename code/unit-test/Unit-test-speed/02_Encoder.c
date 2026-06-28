#include "02_Encoder.h"

#define Encoder_Global(type) static type
Encoder_Global(Encoder_Handle) *encoders[MAX_ENCODERS] = {0};

Encoder_PRIVATE(void) updateEncoder(Encoder_Handle* pHandle){
  
    uint8_t b = digitalRead(pHandle->pinB);
    if (b == HIGH) {
        pHandle->position++;
    } else {
        pHandle->position--;
    }
}
void isr_enc0() { if (encoders[0]) updateEncoder(encoders[0]); }
void isr_enc1() { if (encoders[1]) updateEncoder(encoders[1]); }
void isr_enc2() { if (encoders[2]) updateEncoder(encoders[2]); }
void isr_enc3() { if (encoders[3]) updateEncoder(encoders[3]); }

static void (*isr_table[MAX_ENCODERS])() = {
    isr_enc0, isr_enc1, isr_enc2, isr_enc3
};

Encoder_PUBLIC(void) Encoder_Init(Encoder_Handle *pHandle){
    if (pHandle == NULL) {
        // Serial.println("Error: Encoder handle is NULL");
        return;
    }

    pinMode(pHandle->pinA, INPUT_PULLUP);
    pinMode(pHandle->pinB, INPUT_PULLUP);

    pHandle->position = 0;
    pHandle->state = 0;
    
    encoders[pHandle->id] = pHandle;
    
    attachInterrupt(digitalPinToInterrupt(pHandle->pinA), isr_table[pHandle->id], FALLING);
    
}
Encoder_PUBLIC(int32_t) Encoder_GetPosition(Encoder_Handle *pHandle) {
    if (pHandle == NULL) return 0;

    noInterrupts();
    int32_t pos= pHandle->position;
    interrupts();
    return pos; 
}
