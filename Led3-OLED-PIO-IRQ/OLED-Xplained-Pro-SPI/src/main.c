#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

#define BUT1_PIO	PIOD
#define BUT1_PIO_ID		ID_PIOD
#define BUT1_PIO_IDX	28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)


#define _PIO_DEFAULT             (0u << 0)
#define _PIO_PULLUP              (1u << 0)
#define _PIO_DEGLITCH            (1u << 1)
#define _PIO_DEBOUNCE            (1u << 3)

void pisca_led(int n, int t);
volatile char but1_flag;

void pisca_led1(int t){
	for (int i=0;i<30;i++){
		pio_clear(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
	}
}

void but_callback(void){
	if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		but1_flag = 1;
	} else {
		but1_flag = 0;
	}
}




void io_init(void)
{

	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	//WDT->WDT_MR = WDT_MR_WDDIS;

	io_init();

  // Init OLED
	gfx_mono_ssd1306_init();
  
  // Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	int delay = 400;

	while(1) {
		
		if (but1_flag) {
			delay_ms(300);
			if (but1_flag) {
				delay += 100;
			} else {
				delay -= 100;	
			}
			int frequencia = 1000000/(2*delay);
			
			char str[128];
			gfx_mono_draw_string("              ", 10, 0, &sysfont);
			sprintf(str, "%d mHz", frequencia);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			
			pisca_led1(delay);
			but1_flag = 0;
		}
		
		
		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}

