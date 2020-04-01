#include "main.h"

#include "dmx_transmitter.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

volatile static uint16_t slots_sent;
volatile static uint16_t slots_count;
volatile static const uint8_t *slots_ptr;

void dmx_send(const uint8_t *slots, uint16_t size)
{
	/* Setup global variables, they will be accessed from interrupts  */
	slots_ptr = slots;
	slots_sent = 0;
	slots_count = size;

	/*
	 * DMX512 packet starts with Reset Sequence. Reset Sequence starts with
	 * Break (low level), so set pin to output low.
	 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

	/*
	 * Waiting for Break time, and everything further, is done through TIM3
	 * interrupt. At first interrupt (CC1) this timer changes pin to high
	 * for MarkAfterBreak. Then, in update interrupt, it configures pin into
	 * floating mode. Right after this, it sends first DMX byte 0x00, and
	 * turn on second timer for sending slots after MarkBetween slots.
	 */

	/* So, enable this timer */
	__HAL_TIM_ENABLE(&htim3);

	/* Wait until transmission of whole DMX packet is finished */
	while (slots_ptr != NULL) {};
}

/* Interrupt handler for sending next slot (from TIM2 interrupt) */
void dmx_slot(void)
{
	/*
	 * Each slot is sent in Update Interrupt of TIM2. So, period of TIM2 is
	 * time, needed for 11bits of slot (44us) and Mark Between Slots (0..1s)
	 */
	if (slots_sent < slots_count)
		USART2->DR = slots_ptr[slots_sent++];
	else {
		/* Stop timer when all slots are sent */
		TIM2->CR1 &= ~(TIM_CR1_CEN);
		TIM2->CNT = 0;

		/* Indicate that transmission finished */
		slots_ptr = NULL;
	}
}

/* Interrupt handler for sending reset sequence (from TIM3 interrupt) */
void dmx_reset_sequence(void)
{
	if (TIM3->SR & TIM_IT_UPDATE)
	{
		/* Reset sequence finished, stop timer */
		TIM3->CR1 &= ~(TIM_CR1_CEN);
		TIM3->CNT = 0;

		/* Send start code 0x00 */
		USART2->DR = 0x00;

		/* Start timer for sending slots */
		__HAL_TIM_ENABLE(&htim2);
	}
	else if (TIM3->SR & TIM_IT_CC1)
	{
		/* Break end, set floating ping mode for Mark After Break */
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}
}
