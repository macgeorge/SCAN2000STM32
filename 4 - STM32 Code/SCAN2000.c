#include <stdio.h>
#include <string.h>
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"

#define MAXALLOWEDONCHANNELS 6

#define CH21STATEOFF		0
#define CH21STATEOUT		1
#define CH21STATESENSE	2
#define CH21REQOFF			0
#define CH21REQOUT			1
#define CH21REQSENSE		2
#define CH21NOREQ				3

void SystemClock_Config(void);
void GPIO_Init(void);
void USART4_Init(void);
void UARTSendDMA(void);

uint64_t receivedSequence = 0;
uint8_t receivedCounter = 0;
int8_t numberOfChannelsActive = 0;
uint32_t channelState = 0;
uint8_t channel21State = CH21STATEOFF;
uint8_t channel21Req = CH21NOREQ;
uint8_t channel21DMMCommand = CH21REQOFF;
uint8_t channelSequence[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 1, 2, 3, 4, 5, 6, 7 , 8, 9, 10, 21};
GPIO_TypeDef* GPIOsequence[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
uint32_t PinSequence[] = {LL_GPIO_PIN_10, LL_GPIO_PIN_11, LL_GPIO_PIN_12, LL_GPIO_PIN_15, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7, LL_GPIO_PIN_8, LL_GPIO_PIN_8, LL_GPIO_PIN_2, LL_GPIO_PIN_1, LL_GPIO_PIN_0, LL_GPIO_PIN_7, LL_GPIO_PIN_6, LL_GPIO_PIN_5, LL_GPIO_PIN_4, LL_GPIO_PIN_3, LL_GPIO_PIN_2};
uint8_t uartsinglemessage[40], uartbuffer[2000], uarttransmitbuffer[2000];	
int main(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  SystemClock_Config();

  GPIO_Init();
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1); /* turn on led */
  USART4_Init();
	printf("\n===============\nBoot\n");
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1); /* turn off led */
  
	while (1)
  {

	}
}

void EXTI4_15_IRQHandler(void)
{
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {	/* Clock interrupt */
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_14);
		receivedSequence = receivedSequence << 1;
		receivedSequence |= LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9);
		receivedCounter++;
  }
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_15) != RESET)
  {
		/* Strobe Interrupt */
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_15);
		LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_1); /* toggle led */
		uartbuffer[0] = '\0';
		
		if (receivedCounter != 48)	{
			sprintf((char *)uartsinglemessage, "Warning: 48 clocks not received, %d\n", receivedCounter);
			strcat((char *)uartbuffer, (char *)uartsinglemessage);
		}
		
		if (receivedSequence != 0x0) {
			/* if no command, do nothing */
			uint8_t counter;
			for (counter = 0 ; counter<20 ; counter++) {
				if ((receivedSequence & ((uint64_t)1<<(2*counter))) !=0) {
					/* turn off channel */
					LL_GPIO_ResetOutputPin(GPIOsequence[channelSequence[counter]-1], PinSequence[channelSequence[counter]-1]);
					numberOfChannelsActive--;
					channelState &= ~((long)1<<(channelSequence[counter]-1));
					sprintf((char *)uartsinglemessage, "CH%d OFF\n", channelSequence[counter]);
					strcat((char *)uartbuffer, (char *)uartsinglemessage);
					if (numberOfChannelsActive<0) 
					{
						sprintf((char *)uartsinglemessage, "Warning too many ssr off requests\n");
						strcat((char *)uartbuffer, (char *)uartsinglemessage);
						numberOfChannelsActive = 0;
					}
				}
				
				if ((receivedSequence & ((uint64_t)1<<(2*counter+1))) !=0) {
					/* turn on channel */
					if (numberOfChannelsActive < MAXALLOWEDONCHANNELS) {
						LL_GPIO_SetOutputPin(GPIOsequence[channelSequence[counter]-1], PinSequence[channelSequence[counter]-1]);
						numberOfChannelsActive++;
						channelState |= (1<<(channelSequence[counter]-1));
						sprintf((char *)uartsinglemessage, "CH%d ON\n", channelSequence[counter]);
						strcat((char *)uartbuffer, (char *)uartsinglemessage);
					}
					else
					{
						sprintf((char *)uartsinglemessage, "Error, maximum allowed of on ssrs reached. CH%d did not switch on\n", channelSequence[counter]);
						strcat((char *)uartbuffer, (char *)uartsinglemessage);;
					}
				}
			}
			
			/* channel 21 */
			if ((channelState & 0xFFC00)==0) {
				/* If channels 11 to 20 not used, turn off both bus2 switches */
					channel21Req = CH21REQOFF;
			}
			else {
				/* But turn the bus on again, on the next command for CH11 to CH20 use */
				channel21Req = channel21DMMCommand;
			}
			if ((receivedSequence & ((uint64_t)1<<(2*20))) !=0) {
					/* switch bus2 to out */
				channel21Req = CH21REQOUT;
				channel21DMMCommand = CH21REQOUT;
			}
			if ((receivedSequence & ((uint64_t)1<<(2*20+1))) !=0) {
					/* switch bus2 to sense */
				channel21Req = CH21REQSENSE;
				channel21DMMCommand = CH21REQSENSE;
			}
			
			if ((channel21Req == CH21REQOFF) && channel21State != CH21STATEOFF) {
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
				channel21State = CH21STATEOFF;
				numberOfChannelsActive--;
				sprintf((char *)uartsinglemessage, "CH21 OFF\n");
				strcat((char *)uartbuffer, (char *)uartsinglemessage);
			}
			
			if ((channel21Req == CH21REQOUT) && (channel21State != CH21STATEOUT)) {
				if ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS - 1) || ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS) && (channel21State != CH21STATEOFF))) {
					LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
					LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
					if (channel21State == CH21STATEOFF) numberOfChannelsActive++;
					channel21State = CH21STATEOUT;
					sprintf((char *)uartsinglemessage, "CH21 OUT\n");
					strcat((char *)uartbuffer, (char *)uartsinglemessage);
				}
				else
				{
					sprintf((char *) uartsinglemessage, "Error, maximum allowed of on ssrs reached. Bus2 Mux did not switch on\n");
					strcat((char *)uartbuffer, (char *)uartsinglemessage);
				}
			}
			
			if ((channel21Req == CH21REQSENSE) && channel21State != CH21STATESENSE ) {
				if ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS - 1) || ((numberOfChannelsActive <= MAXALLOWEDONCHANNELS) && (channel21State != CH21STATEOFF))) {
					LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
					LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
					if (channel21State == CH21STATEOFF) numberOfChannelsActive++;
					channel21State = CH21STATESENSE;
					sprintf((char *)uartsinglemessage, "CH21 SENSE\n");
					strcat((char *)uartbuffer, (char *)uartsinglemessage);
				}
				else
				{
					sprintf((char *)uartsinglemessage, "Error, maximum allowed of on ssrs reached. Bus2 Mux did not switch on\n");
					strcat((char *)uartbuffer, (char *)uartsinglemessage);
				}
			}
		}
		channel21Req = CH21NOREQ;
		receivedSequence = 0x0;
		receivedCounter = 0;
		if (uartbuffer[0] != '\0') UARTSendDMA();
  }
	
}

void DMA1_Channel1_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC1(DMA1))
	{
		LL_DMA_ClearFlag_TC1(DMA1);
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	}
	
}

int fputc(int ch, FILE* fil) {
	uint8_t charac = ch;
	/* Send over debug USART */

	while(LL_USART_IsActiveFlag_TC(USART4) != 1);
	LL_USART_TransmitData8 (USART4, charac);
	return ch;
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  };

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(64000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(64000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}


void USART4_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_DMA_InitTypeDef DMA_InitStruct;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART4);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  
  /**USART4 GPIO Configuration  
  PA0   ------> USART4_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART4, &USART_InitStruct);
  LL_USART_ConfigHalfDuplexMode(USART4);

	/* DMA Setup */
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
	DMA_InitStruct.NbData = 0;
	DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)uarttransmitbuffer;
	DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART4, LL_USART_DMA_REG_DATA_TRANSMIT);
	DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_MEMORY_NOINCREMENT;
	DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART4_TX;
	DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;
	LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
	
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 4);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
  LL_USART_Enable(USART4);
}

void UARTSendDMA(void) {
	uint8_t uartstringlength;
	while(LL_USART_IsActiveFlag_TC(USART4) != 1); // wait for previous transfer to finish
	strcpy((char *)uarttransmitbuffer, (char *)uartbuffer);
	uartstringlength = strlen((char *)uarttransmitbuffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, uartstringlength);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);

	/* PB9 (Data) */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* PA1 (LED) */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA2 (CH20) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA3 (CH19) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA4 (CH18) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA5 (CH17) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA6 (CH16) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA7 (CH15) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /* PB0 (CH14) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB1 (CH13) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB2 (CH12) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA8 (CH11) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA9 (Bus Sense) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PC6 (Bus In) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA10 (CH1) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA11 (CH2) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA12 (CH3) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA15 (CH4) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB3 (CH5) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB4 (CH6) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB5 (CH7) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB6 (CH8) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB7 (CH9) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB8 (CH10) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* PC14 (Clock) */
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTC, LL_EXTI_CONFIG_LINE14);
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_14;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_EXTI_Init(&EXTI_InitStruct);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_14, LL_GPIO_MODE_INPUT);

  /* PC15 (Strobe) */
	LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTC, LL_EXTI_CONFIG_LINE15);
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_15;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_EXTI_Init(&EXTI_InitStruct);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_15, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_15, LL_GPIO_MODE_INPUT);

  NVIC_SetPriority(EXTI4_15_IRQn, 1);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void Error_Handler(void)
{

}
