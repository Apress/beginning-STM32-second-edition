/* Interrupt driven UART Receiver Demo
 * For the Apress book:
 *   Beginning STM32: Developing with FreeRTOS, libopencm3 and GGC
 *   Second Edition
 *   Warren Gay
 *
 *      ____STM32____   _RS232_
 *	TX:	A9        RX
 *	RX:	A10       TX
 *	Baud:	115200
 *      Mode:	8N1
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

static QueueHandle_t uart_txq,	// TX queue
		     uart_rxq;	// RX queue

#define INVERT_CASE	0

extern void
vApplicationStackOverflowHook(
  xTaskHandle *pxTask,
  signed portCHAR *pcTaskName);

/***************************************
 * Handle Stack Overflow
 ***************************************/

void
vApplicationStackOverflowHook(
  xTaskHandle *pxTask __attribute((unused)),
  signed portCHAR *pcTaskName __attribute((unused))) {

	for (;;)
		; // Loop forever
}

/***************************************
 * Initialize clocks
 ***************************************/

static void
init_clock(void) {

	// Clock for GPIO port A: GPIO_USART1_TX
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART1);
}

/***************************************
 * ISR For Received UART Data
 ***************************************/

void
usart1_isr(void) {
	char ch;
	BaseType_t hptask=pdFALSE;


	while ( ((USART_SR(USART1) & USART_SR_RXNE) != 0) ) {
		// Have received data:
		ch = usart_recv(USART1);

		// Use best effort to send byte to queue
		xQueueSendFromISR(uart_rxq,&ch,&hptask);
	}
}

/***************************************
 * Initialize UART Device
 ***************************************/

static void
init_usart(void) {

	// Create queues
	uart_txq = xQueueCreate(256,sizeof(char));
	uart_rxq = xQueueCreate(256,sizeof(char));

	// Enable Interrupt controller
	nvic_enable_irq(NVIC_USART1_IRQ);

	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);
	gpio_set_mode(GPIOA,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		GPIO_USART1_RX);

	usart_set_baudrate(USART1,115200);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX_RX);
	usart_set_parity(USART1,USART_PARITY_NONE);

	usart_enable_rx_interrupt(USART1);
	usart_enable(USART1);
}

/***************************************
 * Enable GPIO for LED
 ***************************************/

static void
init_gpio(void) {

	// gpio for LED
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);
	gpio_set(GPIOC,GPIO13); // LED off
}

/***************************************
 * Queue a character to be transmitted
 ***************************************/

static inline void
uart_putc(char ch) {

	// Queue the byte to send
	while ( xQueueSend(uart_txq,&ch,0) != pdPASS )
		taskYIELD();
	if ( ch == '\n' ) {
		// When we see LF, also send CR
		ch = '\r';
		while ( xQueueSend(uart_txq,&ch,0) != pdPASS )
			taskYIELD();
	}
}

/***************************************
 * Return a received charactor (blocks)
 **************************************/

static char
uart_getc(void) {
	char ch;

	while ( xQueueReceive(uart_rxq,&ch,0) != pdPASS )
		taskYIELD();
	gpio_toggle(GPIOC,GPIO13);
	return ch;
}

/***************************************
 * UART Sending Task: 
 ***************************************/

static void
tx_task(void *args __attribute((unused))) {
	char ch;

	for (;;) {
		while ( xQueueReceive(uart_txq,&ch,portMAX_DELAY) != pdPASS )
			taskYIELD();

		// Received a char to transmit:
		while ( !usart_get_flag(USART1,USART_SR_TXE) )
			taskYIELD(); // Not ready to send
		usart_send(USART1,ch);
	}
}

/***************************************
 * Main Task
 ***************************************/

static void
main_task(void *args __attribute((unused))) {

	for (;;) {
		// Block until char received
		char ch = uart_getc();	
#if INVERT_CASE
		if ( islower(ch) )
			ch = toupper(ch);
		else if ( isupper(ch) )
			ch = tolower(ch);
#endif
		uart_putc(ch);
	}
}

/***************************************
 * Main program
 ***************************************/

int
main(void) {

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	init_clock();
	init_gpio();
	init_usart();

	xTaskCreate(main_task,"MAIN",100,NULL,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(tx_task,"UARTTX",100,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();

	for (;;); // Don't go past here
	return 0;
}

/* End main.c */
