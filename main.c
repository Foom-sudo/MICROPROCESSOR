#include "stm32f4xx.h"

#define TRIG_PIN    (1 << 0)
#define ECHO_PIN    (1 << 1)
#define BUZZER_PIN  (1 << 2)
#define SERVO_MIN   1500
#define SERVO_MID   500
#define SERVO_MAX   2500

volatile uint16_t adc_value = 0;
volatile uint16_t current_pos = SERVO_MIN;
volatile uint8_t emergency_mode = 0;

void delay_us(uint32_t us) {
    uint32_t count = us * 20; 
    while(count--) __NOP();
}

void delay_ms(uint32_t ms) {
    while(ms--) delay_us(1000);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        if (!emergency_mode) current_pos = SERVO_MAX;
        EXTI->PR |= (1 << 0);
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1 << 1)) {
        emergency_mode = 1;
        EXTI->PR |= (1 << 1);
    }
}

void EXTI2_IRQHandler(void) {
    if (EXTI->PR & (1 << 2)) {
        emergency_mode = 0;
        current_pos = SERVO_MIN;
        EXTI->PR |= (1 << 2);
    }
}

void EXTI3_IRQHandler(void) {
    if (EXTI->PR & (1 << 3)) {
        if (!emergency_mode) current_pos = SERVO_MID;
        EXTI->PR |= (1 << 3);
    }
}


