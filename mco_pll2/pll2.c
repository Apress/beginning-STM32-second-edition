/* hsi.c - Put PLL / 2 clock on MCO
 * Warren W. Gay VE3WWG
 *
 * PA8 = MCO
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int
main(void) {

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	// LED Configuration:
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);
	gpio_clear(GPIOC,GPIO13);	// LED Off

	// MCO Configuration:
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO8);		// PA8=MCO

	rcc_set_mco(RCC_CFGR_MCO_PLL_DIV2);

	gpio_set(GPIOC,GPIO13); // LED On
	for (;;);
	return 0;
}

// End pll2.c
