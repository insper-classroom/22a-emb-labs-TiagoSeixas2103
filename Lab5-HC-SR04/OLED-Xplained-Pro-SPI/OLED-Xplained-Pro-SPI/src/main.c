#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define BUT1_PIO	PIOD
#define BUT1_PIO_ID		ID_PIOD
#define BUT1_PIO_IDX	28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO	PIOC
#define BUT2_PIO_ID		ID_PIOC
#define BUT2_PIO_IDX	31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO	PIOA
#define BUT3_PIO_ID		ID_PIOA
#define BUT3_PIO_IDX	19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

#define TRIG_PIO	PIOA
#define TRIG_PIO_ID		ID_PIOA
#define TRIG_PIO_IDX	2
#define TRIG_PIO_IDX_MASK (1u << TRIG_PIO_IDX)

#define ECHO_PIO	PIOB
#define ECHO_PIO_ID		ID_PIOB
#define ECHO_PIO_IDX	4
#define ECHO_PIO_IDX_MASK (1u << ECHO_PIO_IDX)

#define _PIO_DEFAULT             (0u << 0)
#define _PIO_PULLUP              (1u << 0)
#define _PIO_DEGLITCH            (1u << 1)
#define _PIO_DEBOUNCE            (1u << 3)

volatile char echo_flag;
float tempo = 0.000058;
float freq;
float tempo2;
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

void callback_echo(void){
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)) {
		freq = 1/(2*tempo);
		RTT_init(freq, 0, 0);
		echo_flag = 0;
	} else {
		tempo2 = rtt_read_timer_value(RTT)/(freq);	
	}
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		pin_toggle(TRIG_PIO, TRIG_PIO_IDX_MASK);    // BLINK Led
	}

}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

void init(void){
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_set_output(TRIG_PIO, TRIG_PIO_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_set_input(ECHO_PIO, ECHO_PIO_IDX_MASK, PIO_DEFAULT);
	pio_handler_set(ECHO_PIO,
					ECHO_PIO_ID,
					ECHO_PIO_IDX_MASK,
					PIO_IT_EDGE,
					callback_echo);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);
	
	//BUTTON 1
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	//BUTTON 2
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	

	//BUTTON 3
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	
	
}

void ativa_ultrassonico(){
	pio_clear(TRIG_PIO, TRIG_PIO_IDX_MASK);
	delay_us(10);
	pio_set(TRIG_PIO, TRIG_PIO_IDX_MASK);
}

int main (void)
{
	board_init();
	sysclk_init();
	init();
	delay_init();
	
	gfx_mono_ssd1306_init();
  

	while(1) {
			ativa_ultrassonico();
			
			float distancia = (tempo2 * 340 * 100.0)/2.0;
			char str[128];
			sprintf(str, "%.6f", distancia);
			gfx_mono_draw_string(str, 0, 0, &sysfont);		
	}
}
