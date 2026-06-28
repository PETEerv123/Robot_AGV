#include "02_Encoder.h"

#define Encoder_Global(type) static type
Encoder_Global(Encoder_Handle) *encoders[MAX_ENCODERS] = {0};

Encoder_PRIVATE(void) updateEncoder(Encoder_Handle* pHandle){
    
    pHandle->state = (pHandle->state << 2) | (digitalRead(pHandle->pinA) << 1) | digitalRead(pHandle->pinB);
    switch (pHandle->state & 0x0F)
    {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            pHandle->position++;
            break;
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            pHandle->position--;
            break;
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
    encoders[pHandle->id] = pHandle;
    attachInterrupt(digitalPinToInterrupt(pHandle->pinA), isr_table[pHandle->id], FALLING);
}
Encoder_PUBLIC(uint32_t) Encoder_GetPosition(Encoder_Handle *pHandle) {
    if (pHandle == NULL) return 0;

    noInterrupts();
    uint32_t pos= pHandle->position;
    interrupts();
    return pos; 
}