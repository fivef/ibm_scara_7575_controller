#ifndef __STM32F4_IO_CUSTOMCODE_H
#define __STM32F4_IO_CUSTOMCODE_H

//encoder
#include "waijung_hwdrvlib.h" 
#include "stm32f4xx_gpio.h"

//interrupt
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

/*
 * definitions for the quadrature encoder pins
 */
// Left Motor Channels
#define ENCLA_PIN               GPIO_Pin_15
#define ENCLA_GPIO_PORT         GPIOA
#define ENCLA_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLA_SOURCE            GPIO_PinSource15
#define ENCLA_AF                GPIO_AF_TIM2

#define ENCLB_PIN               GPIO_Pin_3
#define ENCLB_GPIO_PORT         GPIOB
#define ENCLB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCLB_SOURCE            GPIO_PinSource3
#define ENCLB_AF                GPIO_AF_TIM2

#define ENCLZ_PIN               GPIO_Pin_0
#define ENCLZ_GPIO_PORT         GPIOD
#define ENCLZ_GPIO_CLK          RCC_AHB1Periph_GPIOD
#define ENCLZ_PIN_SOURCE        EXTI_PinSource0
#define ENCLZ_PORT_SOURCE       EXTI_PortSourceGPIOD
#define ENCLZ_EXTI_LINE         EXTI_Line0
#define ENCLZ_EXTI_VECTOR       EXTI0_IRQn 
#define ENCLZ_EXTI_HANDLER      EXTI0_IRQHandler

// Right Motor Channels
#define ENCRA_PIN               GPIO_Pin_6
#define ENCRA_GPIO_PORT         GPIOB
#define ENCRA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRA_SOURCE            GPIO_PinSource6
#define ENCRA_AF                GPIO_AF_TIM4

#define ENCRB_PIN               GPIO_Pin_7
#define ENCRB_GPIO_PORT         GPIOB
#define ENCRB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRB_SOURCE            GPIO_PinSource7
#define ENCRB_AF                GPIO_AF_TIM4

#define ENCRZ_PIN               GPIO_Pin_12
#define ENCRZ_GPIO_PORT         GPIOB
#define ENCRZ_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRZ_PIN_SOURCE        EXTI_PinSource12
#define ENCRZ_PORT_SOURCE       EXTI_PortSourceGPIOB
#define ENCRZ_EXTI_LINE         EXTI_Line12
#define ENCRZ_EXTI_VECTOR       EXTI15_10_IRQn  
#define ENCRZ_EXTI_HANDLER      EXTI15_10_IRQHandler

// determine the timers to use
#define ENCL_TIMER              TIM2
#define ENCL_TIMER_CLK          RCC_APB1Periph_TIM2
#define ENCR_TIMER              TIM4
#define ENCR_TIMER_CLK          RCC_APB1Periph_TIM4

//TimerConfig
#define TIM_EncoderMode_TI12	((uint16_t)0x0003)
#define TIM_ICPolarity_Rising	((uint16_t)0x0000)

//encoder settings
 /*after how many encoder steps will there be a Z index signal*/
#define STEPS_PER_REVOLUTION	2000

/*The current counter value must be in the range counts per rev -  
valid index threshold and counts per rev + valid index threshold to 
be valid and to trigger a counter reset*/
#define VALID_INDEX_THRESHOLD	20

//vars

int32_t encoder1_sum_old;
int32_t encoder2_sum_old;

int32_t encoder1_sum_old_for_speed;
int32_t encoder2_sum_old_for_speed;

int32_t encoder1_index_counter;
int32_t encoder2_index_counter;

//functions

void enable_customio(void);
void disable_customio(void);

void output_customio(boolean_T resetEncoder1, 
        boolean_T resetEncoder2, 
        int32_T * encoder1, 
        int32_T * encoder2, 
        int32_T * encoder1_speed, 
        int32_T * encoder2_speed, 
        int32_T * encoder1_index, 
        int32_T * encoder2_index); 

void ENCLZ_EXTI_HANDLER(void);
void ENCRZ_EXTI_HANDLER(void);

void encoder1Reset (void);
void encoder2Reset (void);

void Configure_Interrupt_Pin(uint32_t pin, 
                              GPIO_TypeDef* port, 
                              uint32_t gpio_clock, 
                              uint8_t pin_source, 
                              uint8_t port_source, 
                              uint32_t line, 
                              uint8_t vector);

//timer functions
void TIM_EncoderInterfaceConfig2(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SetAutoreload2(TIM_TypeDef* TIMx, uint32_t Autoreload);
void TIM_Cmd2(TIM_TypeDef* TIMx, FunctionalState NewState);
uint32_t TIM_GetCounter2(TIM_TypeDef* TIMx);
void TIM_SetCounter2(TIM_TypeDef* TIMx, uint32_t Counter);

#endif
