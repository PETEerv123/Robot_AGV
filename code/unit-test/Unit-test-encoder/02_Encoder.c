#include "02_Encoder.h"

#define Encoder_Global(type) static type
Encoder_Global(Encoder_Handle) *encoders[MAX_ENCODERS] = {0};

Encoder_PRIVATE(void) updateEncoder(Encoder_Handle* pHandle){
    uint8_t a = (uint8_t)digitalRead(pHandle->pinA);
    uint8_t b = (uint8_t)digitalRead(pHandle->pinB);

    uint8_t current = (uint8_t)((a << 1) | b);
    
    if (current == pHandle->state) return; 

    uint8_t combined = (uint8_t)(pHandle->state << 2) | current;
    pHandle->state = current;
    switch (combined)
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
    
        case 0b0011:
        case 0b1100:
            pHandle->position += 2;
            break;
    
        case 0b0110:
        case 0b1001:
            pHandle->position -= 2;
            break;
    
        default:
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
    uint8_t a = digitalRead(pHandle->pinA);
    uint8_t b = digitalRead(pHandle->pinB);
    pHandle->state = (a << 1) | b;

    pHandle->position = 0;

    
    encoders[pHandle->id] = pHandle;
    
    attachInterrupt(digitalPinToInterrupt(pHandle->pinA), isr_table[pHandle->id], CHANGE);
    attachInterrupt(digitalPinToInterrupt(pHandle->pinB), isr_table[pHandle->id], CHANGE);

    
}
Encoder_PUBLIC(int32_t) Encoder_GetPosition(Encoder_Handle *pHandle) {
    if (pHandle == NULL) return 0;

    noInterrupts();
    int32_t pos= pHandle->position;
    interrupts();
    return pos; 
}
