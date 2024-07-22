/*
	Proyecto: Implementacion de controlador FOPID para el control de velocidad de un motor dc con ATSAM3X8E (DUE).
	Autor: RogerS, V: 1.0
	Notas:	
			- Funcion SPI para pantalla ili9341. Visualizacion de datos.
			- Modificar variables en archivo cof_board.h para Arduino DUE para el Timer.
			- Driver TC (Timer Counter) para realizar la funcion de medicion de frecuencia, modo de operacion de captura de onda

*/
#include <asf.h>
#include <math.h>
#include <stdio.h>	
#include <stdlib.h>	
#include <inttypes.h>
#include <conf_board.h>
#include <conf_clock.h>

/* *************** VARIABLES GLOBALES *************** */

#define PWM_FREQUENCY		1000000		// Frecuencia del reloj A en HZ (1MHz)
#define PERIOD_VALUE		100	  	// Duracion del ciclo pwm
#define INIT_DUTY_VALUE		5		// Duracion del pulso activo (ALTO)

#define BG_COLOR	ILI9341_COLOR(0, 0, 0)   // Color de fondo (negro) cuando se limpia la pantalla.
#define TOTAL_PIXELS ((uint32_t)ILI9341_DEFAULT_WIDTH * ILI9341_DEFAULT_HEIGHT)   // Total de pixeles de la pantalla 320x240

#define Canal_A IOPORT_CREATE_PIN(PIOB, 25)		// Canal A del encoder TIOA0, pin 25.
#define LED IOPORT_CREATE_PIN(PIOB, 27)		// Define salida led indicador.
#define C_PWM IOPORT_CREATE_PIN(PIOC, 3)	// Salida para driver MC33926, pin 35

pwm_channel_t pwm_channel_instance;

#define TC_CAPTURE_TIMER_SELECTION TC_CMR_TCCLKS_TIMER_CLOCK3	// Seleccion de captura del Timer
static const uint32_t divisors[5] = {2, 8, 32, 128, 0};
static uint32_t gs_ul_captured_pulses;	// Estado de captura
static uint32_t gs_ul_captured_ra;
static uint32_t gs_ul_captured_rb;

// Variable tipo char para almacenar el dato entero
char cad0[10];	// para ciclo de trabajo senal encoder
char cad1[10];	// fecuencia de la senal de encoder
char cad2[10];	// velocidad angular del motor dc
char cad3[10];	// valor de punto de ajuste
char cad4[10];	// velocidad angular con reductor del motor dc
char cad5[10];
char cad6[50];

char cad7[10];
char cad8[50];
char cad9[10];

/* *********************************** FUNCIONES *********************************** */
static void SPI_9341(void){		// Configuracion SPI-ILI9341
	pmc_enable_periph_clk(ID_PIOA);
	//pmc_enable_periph_clk(ID_PIOB);

	pio_set_output(PIOA, PIO_PA29, LOW, DISABLE, ENABLE);	// Reset
	pio_set_output(PIOB, PIO_PB13, LOW, DISABLE, ENABLE);	// D/C

	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA25A_SPI0_MISO);	// MISO
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA26A_SPI0_MOSI);	// MOSI
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA27A_SPI0_SPCK);	// SCK
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA28A_SPI0_NPCS0);	// CS

	pmc_enable_periph_clk(ID_SPI0);

	spi_enable(SPI0);
	spi_master_init(SPI0);
	ili9341_init();		// Inicializar el servicio ILI9341, no cambiar de ubicacion
	
	ili9341_backlight_on();
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(ILI9341_DEFAULT_WIDTH, ILI9341_DEFAULT_HEIGHT);
	//ili9341_duplicate_pixel(ILI9341_COLOR(250, 0, 0), 240UL * 320UL);
}
static void tc_capture_initialize(void)
{
	sysclk_enable_peripheral_clock(ID_TC_CAPTURE);	// Habilitar el reloj del periferico, configurar PMC para habilitar el modulo TC
	tc_init(TC, TC_CHANNEL_CAPTURE,		// Iniciar TC para modo captura
			TC_CAPTURE_TIMER_SELECTION    // Seleccion de reloj
			| TC_CMR_LDRA_RISING	// Carga RA: Flanco ascendente de TIOA 
			| TC_CMR_LDRB_FALLING	// Carga RB: Flanco de caida de TIOA */
			| TC_CMR_ABETRG		// Disparo (trigger) externo: TIOA 
			| TC_CMR_ETRGEDG_FALLING	// Borde de disparo (trigger) externo: borde de caída 
	);
}
void TC_Handler(void)	// Breve controlador de interrupciones para el TC TC_CHANNEL_CAPTURE
{
	if ((tc_get_status(TC, TC_CHANNEL_CAPTURE) & TC_SR_LDRBS) == TC_SR_LDRBS) {		//! [tc_capture_irq_handler_status]
		gs_ul_captured_pulses++;		// Captura de Pulsos totales
		gs_ul_captured_ra = tc_read_ra(TC, TC_CHANNEL_CAPTURE);		// Captura de flanco ascendente
		gs_ul_captured_rb = tc_read_rb(TC, TC_CHANNEL_CAPTURE);		// Captura de flanco descendente
	}
}
static uint32_t motor(uint32_t trab){	// Funcion que permite actualizar el valor de ul_duty
	pwm_channel_instance.ul_period = PERIOD_VALUE;
	pwm_channel_instance.ul_duty = trab;
	pwm_channel_init(PWM, &pwm_channel_instance);
	pwm_channel_enable(PWM, PWM_CHANNEL_0);
}
/* *************** FUNCION PRINCIPAL *************** */
int main (void)
{
/* *************** VARIABLES LOCALES *************** */	
	uint16_t frequence, dutycycle;
	uint32_t vel;
	uint32_t vel_red;
	uint32_t setp = 10;		// Valor de Punto de Ajuste1, -4.38, 7.595, -6.501, 2.74, -0.4532
	
	int order = 7;		// filtro IIR de orden 4 0.4656, -2.223, 4.25, -4.068, 1.95, -0.3743
	uint32_t a[] = {0.416, -3.034, 8.973, -13.88, 11.89, -5.369, 1};//den{-0.760672, 2.51369, -2.75301, 1}; // funciona 0.416, -3.034, 8.973, -13.88, 11.89, -5.369, 1
	uint32_t b[] = {-0.2948, 1.674, -3.828, 4.465, -2.741, 0.8, -0.07469};//num{-0.949776, 3.25646, -3.71102, 1.4051}; // funciona -0.2948, 1.674, -3.828, 4.465, -2.741, 0.8, -0.07469
	uint32_t s[0];
	
	//uint32_t ek_1=0;uint32_t ek_2=0;uint32_t ek_3=0;uint32_t ek_4=0;uint32_t ek_5=0;uint32_t ek_6=0;uint32_t ek_7=0;
	//uint32_t uk_1=0;uint32_t uk_2=0;uint32_t uk_3=0;uint32_t uk_4=0;uint32_t uk_5=0;uint32_t uk_6=0;uint32_t uk_7=0;
/* *************** Codigo de inicializacion *************** */
	board_init();
	sysclk_init();
	
	const usart_serial_options_t usart_options = {
		.baudrate =   CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits =   CONF_UART_STOP_BITS
	};
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &usart_options);
	usart_enable_tx(CONF_UART);
	usart_enable_rx(CONF_UART);	
	
	pio_configure_pin(C_PWM, PIO_TYPE_PIO_PERIPH_B);
	ioport_set_pin_dir(LED, IOPORT_DIR_OUTPUT);		// Establecer el gpio led como salida.
	ioport_set_pin_mode(Canal_A, PIN_TC_CAPTURE_MUX);	// Configurar Pines PIO para TC
	ioport_disable_pin(Canal_A);	// Deshabilitar el pin IOPORT, basado en un pin creado con IOPORT_CREATE_PIN () para habilitar el modo periférico
	
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, PWM_CHANNEL_0);
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM,&clock_setting);
	pwm_channel_instance.channel = PWM_CHANNEL_0;
	pwm_channel_instance.ul_prescaler = PWM_CMR_CPRE_CLKA;
	pwm_channel_instance.polarity = PWM_HIGH;
	pwm_channel_instance.alignment = PWM_ALIGN_LEFT;
	//pwm_channel_instance.ul_period = PERIOD_VALUE;
	//pwm_channel_instance.ul_duty = INIT_DUTY_VALUE;
	//pwm_channel_init(PWM, &pwm_channel_instance);
	//pwm_channel_enable(PWM, PWM_CHANNEL_0);
	
	//ioport_init();		//	Inicializar el servicio del puerto.
	SPI_9341();		//	Configuracion SPI ili9341
	gfx_init();		// SERVICIO
	gfx_set_orientation(11);	// Orientacion de la pantalla
	tc_capture_initialize();	// Inicializar la funcion de captura
	NVIC_DisableIRQ(TC_IRQn);
	NVIC_ClearPendingIRQ(TC_IRQn);
	NVIC_SetPriority(TC_IRQn, 0);
	NVIC_EnableIRQ(TC_IRQn);
	
	ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_LOW );
	gfx_draw_string("CONTROL FOPID-VELOCIDAD MOTOR DC ", 10, 10, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(255, 124, 0));
	
	for (int i=0; i<=order; i++){
		s[i] = 0;
	}
/* *************** Codigo de aplicacion *************** */
	while (1)
	{		
		tc_enable_interrupt(TC, TC_CHANNEL_CAPTURE, TC_IER_LDRBS);		// Empezar a capturar
		tc_start(TC, TC_CHANNEL_CAPTURE);	//  Iniciar el contador del temporizador en TC TC_CHANNEL_CAPTURE
				
		tc_disable_interrupt(TC, TC_CHANNEL_CAPTURE, TC_IDR_LDRBS);
		
		frequence = (sysclk_get_peripheral_bus_hz(TC)/divisors[TC_CAPTURE_TIMER_SELECTION])/gs_ul_captured_rb;		// frecuencia de la señal de encoder
		dutycycle = (gs_ul_captured_rb - gs_ul_captured_ra) * 100 /	gs_ul_captured_rb;		// ciclo de trabajo de la señal de encoder
		vel = (frequence*60/1000);		// Velocidad angular SIN reductor [rpm]
		vel_red = vel/200;	// Velocidad angular CON reductor (vel/cte_reductor)
					
		gfx_draw_string("Ciclo de trabajo de senal encoder: ", 10, 25, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));	// coordenadas x y
		gfx_draw_string("Frecuencia de senal del encoder: ", 10, 35, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		gfx_draw_string("Velocidad nominal(RPM): ", 10, 50, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		gfx_draw_string("Punto de Ajuste(RPM): ", 10, 60, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		gfx_draw_string("Velocidad angular con reductor(RPM): ", 10, 70, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));

		gfx_draw_string(itoa(dutycycle, cad0, 10), 215, 25, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		gfx_draw_string(itoa(frequence, cad1, 10), 205, 35, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		gfx_draw_string(itoa(vel,cad2, 10), 150, 50, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		gfx_draw_string(itoa(setp,cad3, 10), 140, 60, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		gfx_draw_string(itoa(vel_red, cad4, 10), 230, 70, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));		
		
		//gfx_draw_string("Pulsos Capturados: ", 10, 65, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		//gfx_draw_string(itoa((unsigned)gs_ul_captured_pulses,cad4, 10), 200, 65, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		//gfx_draw_string("Flancos ascendentes: ", 10, 75, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		//gfx_draw_string(itoa((unsigned)gs_ul_captured_ra,cad5, 10), 200, 75, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		//gfx_draw_string("Flancos descendentes: ", 10, 85, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		//gfx_draw_string(itoa((unsigned)gs_ul_captured_rb,cad6, 10), 200, 85, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		
		gfx_draw_string("Senal de Error: ", 10, 85, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		gfx_draw_string("Senal salida FOPID: ", 10, 95, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		
		
		ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_HIGH );		// inicio de ciclo
		uint32_t yk = vel_red;		// lectura de la senal de velocidad del sensor(encoder)
		uint32_t ek = setp - yk;	// senal de error---setp - yk
		
		//float sal = 31.36*uk_1-0.0276*uk_2-41.61*uk_3-0.02741*uk_4+12.68*uk_5-0.004478*uk_6+25.57*ek-38.67*ek_1-2.021*ek_2+22.1*ek_3-3.982*ek_4-3.012*ek_5+0.7924*ek_6;
		//double sal = a[0]*uk_1+a[1]*uk_2+a[2]*uk_3+a[3]*uk_4+a[4]*uk_5+a[5]*uk_6+a[6]*uk_7+b[0]*ek_1+b[1]*ek_2+b[2]*ek_3+b[3]*ek_4+b[4]*ek_5+b[5]*ek_6+b[6]*ek_7;
		
		uint32_t feedback = 0; uint32_t feedforwad = 0;

		for (int i = 1; i <= order; i++){
			feedback = feedback - a[i] * s[i];
			feedforwad = feedforwad + b[i] * s[i];
		}
		s[0] = ek + a[0] * feedback;
		uint32_t sal = b[0] * s[1] + feedforwad;
		for (int i = 1; i <= order; i++){
			s[i] = s[i-1];
		}
		if (sal >= 100){
			sal = 100;
		}
		if (sal <= 0){
			sal = 0;
		}		
		//uk_6=uk_5;uk_5=uk_4;uk_4=uk_3;uk_3=uk_2;uk_2=uk_1;uk_1=sal;
		//ek_6=ek_5;ek_5=ek_4;ek_4=ek_3;ek_3=ek_2;ek_2=ek_1;ek_1=ek;		

		gfx_draw_string(itoa(ek, cad5, 10), 105, 85, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		gfx_draw_string(itoa(sal, cad6, 10), 130, 95, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 255, 0));
		motor(sal);		//enviar senal pwm al puerto de salida
		printf("%" PRIu32 "\n", vel_red);
		/*
		if (!gs_ul_captured_pulses)		// si no recibe alguna señal, se reestablecen a cero las variables de
		{
			gs_ul_captured_pulses = 0;
			gs_ul_captured_ra = 0;
			gs_ul_captured_rb = 0;
			
			gfx_draw_string("Sin senal de encoder! ", 10, 95, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		}else{
			gfx_draw_string("Senal recibida de encoder!", 10, 95, &sysfont, GFX_COLOR(0, 0, 0), GFX_COLOR(0, 200, 255));
		}
		
		
		//gs_ul_captured_pulses = 0;
		//gs_ul_captured_ra = 0;
		//gs_ul_captured_rb = 0;
		*/
	}
}
