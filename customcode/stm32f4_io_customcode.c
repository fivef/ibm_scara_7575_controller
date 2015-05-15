#include "stm32f4_io_customcode.h"


void enable_customio(void)
{
 /*
 * Configure two timers as quadrature encoder counters. 
 * Details of which timers should be used are
 * in the project hardware header file.
 * Most timers can be used if channels 1 and 2 are available on pins.
 * The timers are mostly 16 bit. Timers can be set to 32 bit but they are
 * not very convenient for IO pins so the counters are simply set to to
 * 16 bit counting regardless.
 * A mouse needs 32 bits of positional data and, since it also needs the
 * current speed, distance is not maintained by the encoder code but will
 * be looked after by the motion control code.
 * The counters are set to X4 mode. The only alternative is X2 counting.

 z index interrupt counting on PD0 and PB12

 */
  GPIO_InitTypeDef GPIO_InitStructure;
  // turn on the clocks for each of the ports needed
  RCC_AHB1PeriphClockCmd (ENCLA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCLB_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRB_GPIO_CLK, ENABLE);

  // now configure the pins themselves
  // they are all going to be inputs with pullups
  GPIO_StructInit (&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = ENCLA_PIN;
  GPIO_Init (ENCLA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCLB_PIN;
  GPIO_Init (ENCLB_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRA_PIN;
  GPIO_Init (ENCRA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRB_PIN;
  GPIO_Init (ENCRB_GPIO_PORT, &GPIO_InitStructure);

  // Connect the pins to their Alternate Functions
  GPIO_PinAFConfig (ENCLA_GPIO_PORT, ENCLA_SOURCE, ENCLA_AF);
  GPIO_PinAFConfig (ENCLB_GPIO_PORT, ENCLB_SOURCE, ENCLB_AF);
  GPIO_PinAFConfig (ENCRA_GPIO_PORT, ENCRA_SOURCE, ENCRA_AF);
  GPIO_PinAFConfig (ENCRB_GPIO_PORT, ENCRB_SOURCE, ENCRB_AF);

  // Timer peripheral clock enable
  RCC_APB1PeriphClockCmd (ENCL_TIMER_CLK, ENABLE);
  RCC_APB1PeriphClockCmd (ENCR_TIMER_CLK, ENABLE);

  // set them up as encoder inputs
  // set both inputs to rising polarity to let it use both edges
  TIM_EncoderInterfaceConfig2 (ENCL_TIMER, TIM_EncoderMode_TI12, 
                              TIM_ICPolarity_Rising, 
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload2 (ENCL_TIMER, 0xffff);
  TIM_EncoderInterfaceConfig2 (ENCR_TIMER, TIM_EncoderMode_TI12, 
                              TIM_ICPolarity_Rising, 
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload2 (ENCR_TIMER, 0xffff);

  // turn on the timer/counters
  TIM_Cmd2 (ENCL_TIMER, ENABLE);
  TIM_Cmd2 (ENCR_TIMER, ENABLE);
  encodersReset();



  //Z index interrupt init
  Configure_Interrupt_Pin(ENCLZ_PIN, 
                          ENCLZ_GPIO_PORT, 
                          ENCLZ_GPIO_CLK, 
                          ENCLZ_PIN_SOURCE, 
                          ENCLZ_PORT_SOURCE, 
                          ENCLZ_EXTI_LINE, 
                          ENCLZ_EXTI_VECTOR);

  Configure_Interrupt_Pin(ENCRZ_PIN, 
                          ENCRZ_GPIO_PORT, 
                          ENCRZ_GPIO_CLK, 
                          ENCRZ_PIN_SOURCE, 
                          ENCRZ_PORT_SOURCE, 
                          ENCRZ_EXTI_LINE, 
                          ENCRZ_EXTI_VECTOR);
}

void disable_customio(void)
{
    // do nothing
}

void output_customio(boolean_T in1, boolean_T in2, int16_T * out1, int16_T * out2){
    

    encoder1 = TIM_GetCounter2 (ENCL_TIMER);
    encoder2 = TIM_GetCounter2 (ENCR_TIMER);



    encoder1_sum = encoder1 + encoder1_sum;
    encoder2_sum = encoder2 + encoder2_sum;

    
    *out1 = encoder1_sum; 
	  *out2 = encoder2_sum; 

    TIM_SetCounter2 (ENCL_TIMER, 0);

    TIM_SetCounter2 (ENCR_TIMER, 0);
/*
    oldEncoder1 = encoder1;
    encoder1 = TIM_GetCounter2 (ENCR_TIMER);
    encoder1Count = encoder1 - oldEncoder1;
    encoder1Total =  encoder1Total + encoder1Count;

    TIM_SetCounter2 (ENCL_TIMER, 0);

    TIM_SetCounter2 (ENCR_TIMER, 0);

    
    *out1 = encoder1Total; 
	*out2 = encoder1; 
 */
}


void TIM_EncoderInterfaceConfig2(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity)
{
  uint16_t tmpsmcr = 0;
  uint16_t tmpccmr1 = 0;
  uint16_t tmpccer = 0;
    
  /* Check the parameters */
  assert_param(IS_TIM_LIST2_PERIPH(TIMx));
  assert_param(IS_TIM_ENCODER_MODE(TIM_EncoderMode));
  assert_param(IS_TIM_IC_POLARITY(TIM_IC1Polarity));
  assert_param(IS_TIM_IC_POLARITY(TIM_IC2Polarity));

  /* Get the TIMx SMCR register value */
  tmpsmcr = TIMx->SMCR;

  /* Get the TIMx CCMR1 register value */
  tmpccmr1 = TIMx->CCMR1;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Set the encoder Mode */
  tmpsmcr &= (uint16_t)~TIM_SMCR_SMS;
  tmpsmcr |= TIM_EncoderMode;

  /* Select the Capture Compare 1 and the Capture Compare 2 as input */
  tmpccmr1 &= ((uint16_t)~TIM_CCMR1_CC1S) & ((uint16_t)~TIM_CCMR1_CC2S);
  tmpccmr1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

  /* Set the TI1 and the TI2 Polarities */
  tmpccer &= ((uint16_t)~TIM_CCER_CC1P) & ((uint16_t)~TIM_CCER_CC2P);
  tmpccer |= (uint16_t)(TIM_IC1Polarity | (uint16_t)(TIM_IC2Polarity << (uint16_t)4));

  /* Write to TIMx SMCR */
  TIMx->SMCR = tmpsmcr;

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmr1;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

void TIM_SetAutoreload2(TIM_TypeDef* TIMx, uint32_t Autoreload)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  
  /* Set the Autoreload Register value */
  TIMx->ARR = Autoreload;
}

void TIM_Cmd2(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the TIM Counter */
    TIMx->CR1 |= TIM_CR1_CEN;
  }
  else
  {
    /* Disable the TIM Counter */
    TIMx->CR1 &= (uint16_t)~TIM_CR1_CEN;
  }
}

void encodersReset (void)
{
  __disable_irq();
    
    encoder1 = 0;
    encoder2 = 0;
    /*
    oldEncoder1 = 0;
    encoder1Count = 0;
    encoder1Total = 0;
     */
  TIM_SetCounter2 (ENCL_TIMER, 0);
  TIM_SetCounter2 (ENCR_TIMER, 0);
    __enable_irq();
}



uint32_t TIM_GetCounter2(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Get the Counter Register value */
  return TIMx->CNT;
}

void TIM_SetCounter2(TIM_TypeDef* TIMx, uint32_t Counter)
{
  /* Check the parameters */
   assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Set the Counter Register value */
  TIMx->CNT = Counter;
}


/* Configure pins to be interrupts */
void Configure_Interrupt_Pin(uint32_t pin, 
                              GPIO_TypeDef* port, 
                              uint32_t gpio_clock, 
                              uint8_t pin_source, 
                              uint8_t port_source, 
                              uint32_t line, 
                              uint8_t vector) {
  /* Set variables used */
  GPIO_InitTypeDef GPIO_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  /* Enable clock for GPIOD */
  RCC_AHB1PeriphClockCmd(gpio_clock, ENABLE);
  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Set pin as input */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = pin;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(port, &GPIO_InitStruct);
  
  /* Tell system that you will use PD0 for EXTI_Line0 */
  SYSCFG_EXTILineConfig(port_source, pin_source);
  
  /* PD0 is connected to EXTI_Line0 */
  EXTI_InitStruct.EXTI_Line = line;
  /* Enable interrupt */
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  /* Interrupt mode */
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  /* Triggers on rising and falling edge */
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  /* Add to EXTI */
  EXTI_Init(&EXTI_InitStruct);

  /* Add IRQ vector to NVIC */
  /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
  NVIC_InitStruct.NVIC_IRQChannel = vector;
  /* Set priority */
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
  /* Set sub priority */
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
  /* Enable interrupt */
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  /* Add to NVIC */
  NVIC_Init(&NVIC_InitStruct);
}

/* Set interrupt handlers */
void ENCLZ_EXTI_HANDLER(void) {
  /* Make sure that interrupt flag is set */
  if (EXTI_GetITStatus(ENCLZ_EXTI_LINE) != RESET) {
    /* Do your stuff */

   
    
    
    /* Clear interrupt flag */
    EXTI_ClearITPendingBit(ENCLZ_EXTI_LINE);
  }
}

void ENCRZ_EXTI_HANDLER(void) {
  /* Make sure that interrupt flag is set */
  if (EXTI_GetITStatus(ENCRZ_EXTI_LINE) != RESET) {
    /* Do your stuff */


    
    
    /* Clear interrupt flag */
    EXTI_ClearITPendingBit(ENCRZ_EXTI_LINE);
  }
}