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

void Init_All_Hardware(void) {
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | 
                     RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN);
    
    RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN);

    GPIOA->MODER |= (1 << (0 * 2));
    GPIOD->MODER |= (1 << (2 * 2));

    GPIOA->MODER |= (2U << (9 * 2));      
    GPIOA->AFR[1] |= (1U << ((9 - 8) * 4)); 

    GPIOB->MODER |= (0x55000000); 

    GPIOA->PUPDR |= ((2U << (2 * 2)) | (2U << (3 * 2)));
    GPIOB->PUPDR |= ((1U << (0 * 2)) | (1U << (1 * 2)));

    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; 
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; 
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA; 
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA; 

    EXTI->IMR |= 0x0F;  
    EXTI->FTSR |= (1 << 0) | (1 << 1);
    EXTI->RTSR |= (1 << 2) | (1 << 3);
    
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

    GPIOC->MODER |= (3 << (0 * 2));
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SQR3 = 10;

    TIM1->PSC = 16 - 1;    
    TIM1->ARR = 20000 - 1; 
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER |= TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE; 
    TIM1->CR1 |= TIM_CR1_CEN;
}

uint32_t Get_Distance_CM(void) {
    uint32_t count = 0;
    uint32_t timeout = 100000; 

    GPIOA->BSRR = TRIG_PIN;
    delay_us(12);
    GPIOA->BSRR = (TRIG_PIN << 16);

    while(!(GPIOA->IDR & ECHO_PIN) && timeout--);
    if(timeout == 0) return 999; 

    while((GPIOA->IDR & ECHO_PIN)) {
        count++;
        delay_us(1);
        if(count > 20000) break; 
    }
    return (count / 58);
}
