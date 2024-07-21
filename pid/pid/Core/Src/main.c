#include "stm32f4xx.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
} PIDController;

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
}

float PID_Compute(PIDController *pid, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->previous_error = error;
    return output;
}


void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
    GPIOA->AFR[0] |= 0x0011;

    GPIOA->MODER |= GPIO_MODER_MODER8_1;
    GPIOA->AFR[1] |= 0x0001;
}

void TIM2_Encoder_Init(void) {

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM2->CR1 |= TIM_CR1_CEN;
}

int32_t ReadEncoderSpeed(void) {
    return (int32_t)(TIM2->CNT);
}

void TIM1_PWM_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM1->PSC = 84 - 1;
    TIM1->ARR = 1000 - 1;

    TIM1->CCR1 = 0;

    TIM1->CR1 |= TIM_CR1_CEN;
}

void SetMotorSpeed(float speed) {
    if (speed > 1000.0f) speed = 1000.0f;
    if (speed < 0.0f) speed = 0.0f;
    TIM1->CCR1 = (uint32_t)speed;
}

int main(void) {
    GPIO_Init();
    TIM2_Encoder_Init();
    TIM1_PWM_Init();

    PIDController pid;
    PID_Init(&pid, 1.0f, 0.1f, 0.01f);

    float setpoint = 100.0f;
    float measured_speed = 0.0f;
    float dt = 0.01f;

    while (1) {
        measured_speed = (float)ReadEncoderSpeed();

        float control_signal = PID_Compute(&pid, setpoint, measured_speed, dt);

        SetMotorSpeed(control_signal);

        HAL_Delay(10);
    }
}
