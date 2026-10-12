//============================================================
// Laboratorio 3 - Tecnologias de Microprocesamiento
// Grupo 7
// Problema D - Piano electronico
//============================================================

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>


//------------------------------------------------------------
// Definiciones y constantes
//------------------------------------------------------------

#define LCD_RS PC0
#define LCD_E  PC1

#define TECLAS 8
#define ANTIRREBOTE 3
#define INTERVALO_TECLAS_MS 10
#define DURACION_PRUEBA_MS 1000

#define VOZ_MELODIA PB1    // OC1A / D9
#define VOZ_BAJO    PD3    // OC2B / D3

typedef enum
{
	MODO_PIANO,
	MODO_CANCION,
	MODO_PRUEBA
} modo_t;

typedef struct
{
	uint8_t melodia;
	uint8_t bajo;
	uint16_t duracion_ms;
} paso_t;


//------------------------------------------------------------
// Canciones y notas
//------------------------------------------------------------

// Cada paso contiene melodia, bajo y duracion en milisegundos.
// Nota 0: silencio. Duracion 0: fin de cancion.

// Who Can It Be Now.
const paso_t cancion_1[] PROGMEM =
{
	// Compases 1-2
	{13,  9, 234},
	{10,  9, 234},
	{ 6,  9, 234},
	{14,  9, 469},
	{ 6,  9, 234},
	{10,  9, 234},
	{ 0,  9, 236},
	// Respuesta: dos acentos de Mi4; bajo Do#3 repetido en corcheas
	{ 3,  9, 180},
	{ 3,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 3,  9, 180},
	{ 3,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  57},

	// Compases 3-4
	{13, 12, 234},
	{10, 12, 234},
	{ 6, 12, 234},
	{14, 12, 469},
	{ 6, 12, 234},
	{10, 12, 234},
	{ 0, 12, 236},
	// Respuesta: dos acentos de Do#4; bajo Si2 repetido en corcheas
	{13, 12, 180},
	{13,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{13, 12, 180},
	{13,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  57},

	// Compases 5-6
	{13,  9, 234},
	{10,  9, 234},
	{ 6,  9, 234},
	{14,  9, 469},
	{ 6,  9, 234},
	{10,  9, 234},
	{ 0,  9, 236},
	// Respuesta: dos acentos de Mi4; bajo Do#3 repetido en corcheas
	{ 3,  9, 180},
	{ 3,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 3,  9, 180},
	{ 3,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  54},
	{ 0,  9, 180},
	{ 0,  0,  57},

	// Compases 7-8
	{13, 12, 234},
	{10, 12, 234},
	{ 6, 12, 234},
	{14, 12, 469},
	{ 6, 12, 234},
	{10, 12, 234},
	{ 0, 12, 236},
	// Respuesta: dos acentos de Do#4; bajo Si2 repetido en corcheas
	{13, 12, 180},
	{13,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{13, 12, 180},
	{13,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  54},
	{ 0, 12, 180},
	{ 0,  0,  57},
	{ 0,  0,   0}
};
// C2: Pideme la luna - Leo Dan.
const paso_t cancion_2[] PROGMEM =
{
	// Compas 1
	{ 0,  0,  562},
	{ 3,  0,  188},
	{11,  0,  188},
	{ 7,  0,  187},
	{ 6,  0,  187},
	{ 7,  0,  188},
	{ 7, 15,  375},
	{11, 15,  315},
	{11,  0,   60},
	{ 0, 15,  690},
	{ 0,  0,   60},

	// Compas 2
	{ 0, 15,  562},
	{ 3, 15,  128},
	{ 3,  0,   60},
	{11, 15,  188},
	{ 7, 15,  187},
	{ 6, 15,  187},
	{ 7, 15,  128},
	{ 7,  0,   60},
	{ 7, 15,  375},
	{ 6, 15,  165},
	{ 0, 15,   22},
	{ 6, 15,  128},
	{ 6,  0,   60},
	{ 6, 15,  188},
	{ 0, 15,  502},
	{ 0,  0,   60},

	// Compas 3
	{ 0, 15,  690},
	{ 0,  0,   60},
	{ 0, 15,  562},
	{ 3, 15,  128},
	{ 3,  0,   60},
	{ 7, 16,  188},
	{ 6, 16,  187},
	{ 7, 16,  187},
	{ 6, 16,  128},
	{ 6,  0,   60},
	{ 7, 16,  188},
	{ 6, 16,  187},
	{ 7, 16,  187},
	{ 6, 16,  128},
	{ 6,  0,   60},

	// Compas 4
	{ 7, 16,  375},
	{ 6, 16,  187},
	{ 7, 16,  128},
	{ 7,  0,   60},
	{ 7, 16,  375},
	{ 0, 16,  187},
	{ 3, 16,  128},
	{ 3,  0,   60},
	{ 7, 16,  188},
	{ 6, 16,  187},
	{ 7, 16,  187},
	{ 6, 16,  128},
	{ 6,  0,   60},
	{11, 16,  375},
	{ 7, 16,  187},
	{ 6, 16,  128},
	{ 6,  0,   60},
	{ 0,  0,    0}
};

// C3: All Star - Smash Mouth.
const paso_t cancion_3[] PROGMEM =
{
	// Compas 1
	{ 0,  0, 1800},
	{ 5,  0,  600},
	// Compas 2
	{15, 17,  300},
	{ 7, 17,  278},
	{ 0, 17,   22},
	{ 7,  0,  300},
	{ 7, 12,  300},
	{ 6,  1,  300},
	{ 5,  1,  278},
	{ 0,  1,   22},
	{ 5,  0,  300},
	{ 7,  2,  300},
	// Compas 3
	{ 7, 18,  278},
	{ 0, 18,   22},
	{ 7, 18,  278},
	{ 0, 18,   22},
	{ 7,  0,  300},
	{ 5, 19,  278},
	{ 0, 19,   22},
	{ 5, 15,  300},
	{ 9, 15,  300},
	{ 0,  0,  300},
	{ 9, 12,  300},
	// Compas 4
	{15, 19,  300},
	{ 7, 19,  278},
	{ 0, 19,   22},
	{ 7,  0,  300},
	{ 6,  9,  278},
	{ 0,  9,   22},
	{ 6,  2,  300},
	{ 5,  2,  278},
	{ 0,  2,   22},
	{ 5,  0,  300},
	{ 9,  3,  300},
	// Compas 5
	{ 9, 15,  600},
	{ 9,  0,  300},
	{ 9, 12,  300},
	{ 0,  1,  300},
	{ 6,  1,  278},
	{ 0,  1,   22},
	{ 6,  0,  300},
	{ 7,  2,  300},
	// Compas 6
	{15, 19,  300},
	{ 7, 19,  278},
	{ 0, 19,   22},
	{ 7,  0,  300},
	{ 6,  9,  278},
	{ 0,  9,   22},
	{ 6,  2,  300},
	{ 5,  2,  278},
	{ 0,  2,   22},
	{ 5,  0,  300},
	{ 8,  3,  300},
	// Compas 7
	{ 8, 18,  300},
	{ 7, 18,  278},
	{ 0, 18,   22},
	{ 7,  0,  300},
	{ 6, 19,  278},
	{ 0, 19,   22},
	{ 6, 15,  300},
	{ 5, 15,  278},
	{ 0, 15,   22},
	{ 5,  0,  300},
	{15, 12,  300},
	// Compas 8
	{15, 17,  300},
	{ 7, 17,  278},
	{ 0, 17,   22},
	{ 7,  0,  300},
	{ 6, 12,  278},
	{ 0, 12,   22},
	{ 6,  1,  300},
	{ 5,  1,  278},
	{ 0,  1,   22},
	{ 5,  0,  300},
	{ 6,  2,  300},
	// Compas 9
	{ 6, 18,  300},
	{ 3, 18,  300},
	{ 3,  0,  300},
	{ 3, 19,  300},
	{ 0, 15,  300},
	{ 5, 15,  278},
	{ 0, 15,   22},
	{ 5,  0,  300},
	{ 5, 12,  150},
	{ 2, 12,  150},
	{ 0,  0,    0}
};

const paso_t *const canciones[3] = {cancion_1, cancion_2, cancion_3};

// TOP calculado para Fast PWM con F_CPU = 16 MHz y duty cercano al 50 %.
// Notas 1-8: teclado. Notas adicionales solo para canciones.
const uint16_t top_melodia[16] PROGMEM =
{0, 7644, 6809, 6066, 5726, 5101, 4544, 4049, 3821,
5404, 4815, 3607, 3213, 7214, 3033, 3404};
const uint8_t top_bajo[20] PROGMEM =
{0, 118, 105, 94, 88, 79, 70, 62, 59,
112, 83, 74, 126, 70, 55, 141, 168, 212, 189, 158};

const char *const nombres_melodia[16] =
{"--", "Do4", "Re4", "Mi4", "Fa4", "Sol4", "La4", "Si4", "Do5",
"Fa#4", "Sol#4", "Do#5", "Re#5", "Do#4", "Mi5", "Re5"};
const char *const nombres_bajo[20] =
{"--", "Do3", "Re3", "Mi3", "Fa3", "Sol3", "La3", "Si3", "Do4",
	"Do#3", "Fa#3", "Sol#3", "Si2", "La3", "Do#4", "La2", "Fa#2",
"Re2", "Mi2", "Sol2"};


//------------------------------------------------------------
// Variables globales
//------------------------------------------------------------

volatile uint16_t tiempo_ms = 0;

uint8_t muestra_previa[TECLAS] = {0};
uint8_t tecla_estable[TECLAS] = {0};
uint8_t contador_rebote[TECLAS] = {0};

modo_t modo_actual = MODO_PIANO;
uint8_t nota_piano = 0;
uint8_t nota_melodia_activa = 0;
uint8_t nota_bajo_activa = 0;
uint8_t comando_cancion_pendiente = 0;
uint8_t cancion_actual = 0;
uint16_t indice_paso = 0;
uint16_t inicio_paso_ms = 0;
uint16_t duracion_paso_ms = 0;
uint16_t inicio_prueba_ms = 0;


//------------------------------------------------------------
// Prototipos de funciones
//------------------------------------------------------------

void configurar_gpio(void);
void configurar_usart(void);
void configurar_timer0(void);

uint16_t obtener_tiempo_ms(void);
void uart_enviar_caracter(char dato);
void uart_enviar_cadena(const char *texto);
void uart_revisar(void);
void mostrar_menu(void);

void lcd_inicializar(void);
void lcd_enviar_nibble(uint8_t nibble);
void lcd_comando(uint8_t comando);
void lcd_dato(uint8_t dato);
void lcd_linea(uint8_t fila, const char *texto);
void lcd_mostrar(const char *primera, const char *segunda);

uint8_t leer_tecla(uint8_t indice);
void actualizar_teclas(void);
uint8_t nota_presionada(void);
void actualizar_piano(void);

void reproducir_melodia(uint8_t nota);
void reproducir_bajo(uint8_t nota);
void silenciar_voces(void);

void mostrar_paso(uint8_t melodia, uint8_t bajo);
void reproducir_siguiente_paso(uint16_t ahora);
void iniciar_cancion(uint8_t numero);
void iniciar_prueba(void);
void volver_a_piano(void);


//------------------------------------------------------------
// Programa principal
//------------------------------------------------------------

int main(void)
{
	uint16_t ultimo_teclado_ms = 0;

	configurar_gpio();
	configurar_usart();
	configurar_timer0();
	lcd_inicializar();

	lcd_mostrar("Modo piano", "Pulse una nota");
	mostrar_menu();
	sei();

	while (1)
	{
		uart_revisar();
		uint16_t ahora = obtener_tiempo_ms();

		if ((uint16_t)(ahora - ultimo_teclado_ms) >= INTERVALO_TECLAS_MS)
		{
			ultimo_teclado_ms = ahora;
			actualizar_teclas();

			if (modo_actual == MODO_PIANO)
			actualizar_piano();
		}

		if (modo_actual == MODO_CANCION &&
		(uint16_t)(ahora - inicio_paso_ms) >= duracion_paso_ms)
		{
			reproducir_siguiente_paso(ahora);
		}

		if (modo_actual == MODO_PRUEBA &&
		(uint16_t)(ahora - inicio_prueba_ms) >= DURACION_PRUEBA_MS)
		{
			volver_a_piano();
			uart_enviar_cadena("Prueba finalizada\r\n");
		}
	}
}


//------------------------------------------------------------
// CONFIGURACION GPIO
//------------------------------------------------------------

void configurar_gpio(void)
{
	// Ocho pulsadores conectados a GND con pull-up interno.
	DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5) |
	(1 << PD6) | (1 << PD7));
	PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5) |
	(1 << PD6) | (1 << PD7);

	DDRB &= ~((1 << PB0) | (1 << PB2) | (1 << PB3));
	PORTB |= (1 << PB0) | (1 << PB2) | (1 << PB3);

	DDRB |= (1 << VOZ_MELODIA);
	DDRD |= (1 << VOZ_BAJO);
	PORTB &= ~(1 << VOZ_MELODIA);
	PORTD &= ~(1 << VOZ_BAJO);

	DDRC |= 0x3F;
	PORTC &= ~0x3F;
}


//------------------------------------------------------------
// CONFIGURACION USART
//------------------------------------------------------------

void configurar_usart(void)
{
	// 9600 baud, 8 bits, sin paridad, 1 bit de parada.
	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}


//------------------------------------------------------------
// CONFIGURACION TIMER0
//------------------------------------------------------------

void configurar_timer0(void)
{
	// CTC, prescaler 64, OCR0A = 249: base de 1 ms.
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 249;
	TIMSK0 = (1 << OCIE0A);
}


//------------------------------------------------------------
// USART
//------------------------------------------------------------

void uart_enviar_caracter(char dato)
{
	while (!(UCSR0A & (1 << UDRE0)))
	{
	}

	UDR0 = dato;
}

void uart_enviar_cadena(const char *texto)
{
	while (*texto)
	uart_enviar_caracter(*texto++);
}

void mostrar_menu(void)
{
	uart_enviar_cadena("\r\nPROBLEMA D - PIANO ELECTRONICO\r\n");
	uart_enviar_cadena("C1, C2, C3: seleccionar cancion\r\n");
	uart_enviar_cadena("S: detener y volver a piano\r\n");
	uart_enviar_cadena("T: probar las dos voces (1 s)\r\n");
	uart_enviar_cadena("M: mostrar menu\r\n");
	uart_enviar_cadena("C1: Men At Work. C2: Pideme la luna. C3: All Star.\r\n");
}

// Recibe y confirma comandos sin bloquear la reproduccion.
void uart_revisar(void)
{
	char dato;

	if (!(UCSR0A & (1 << RXC0)))
	return;

	dato = UDR0;

	if (dato == '\r' || dato == '\n')
	return;

	uart_enviar_caracter(dato);

	if (dato == 's' || dato == 'S')
	{
		comando_cancion_pendiente = 0;
		volver_a_piano();
		uart_enviar_cadena("\r\nModo piano\r\n");
		return;
	}

	if (comando_cancion_pendiente)
	{
		comando_cancion_pendiente = 0;
		uart_enviar_cadena("\r\n");

		if (dato >= '1' && dato <= '3')
		iniciar_cancion((uint8_t)(dato - '1'));
		else
		uart_enviar_cadena("Cancion invalida. Use C1, C2 o C3.\r\n");

		return;
	}

	if (dato == 'c' || dato == 'C')
	{
		comando_cancion_pendiente = 1;
		return;
	}

	uart_enviar_cadena("\r\n");

	if (dato == 't' || dato == 'T')
	iniciar_prueba();
	else if (dato == 'm' || dato == 'M')
	{
		if (modo_actual == MODO_PIANO)
		mostrar_menu();
		else
		uart_enviar_cadena("S para volver a modo piano.\r\n");
	}
	else
	uart_enviar_cadena("Comando invalido. Use M para ver el menu.\r\n");
}


//------------------------------------------------------------
// LCD 16x2
//------------------------------------------------------------

void lcd_enviar_nibble(uint8_t nibble)
{
	PORTC = (PORTC & 0xC3) | ((nibble & 0x0F) << 2);
	PORTC |= (1 << LCD_E);
	_delay_us(1);
	PORTC &= ~(1 << LCD_E);
	_delay_us(50);
}

void lcd_comando(uint8_t comando)
{
	PORTC &= ~(1 << LCD_RS);
	lcd_enviar_nibble(comando >> 4);
	lcd_enviar_nibble(comando & 0x0F);

	if (comando == 0x01 || comando == 0x02)
	_delay_ms(2);
}

void lcd_dato(uint8_t dato)
{
	PORTC |= (1 << LCD_RS);
	lcd_enviar_nibble(dato >> 4);
	lcd_enviar_nibble(dato & 0x0F);
}

void lcd_inicializar(void)
{
	_delay_ms(40);
	PORTC &= ~((1 << LCD_RS) | (1 << LCD_E));

	lcd_enviar_nibble(0x03);
	_delay_ms(5);
	lcd_enviar_nibble(0x03);
	_delay_us(150);
	lcd_enviar_nibble(0x03);
	lcd_enviar_nibble(0x02);

	lcd_comando(0x28);
	lcd_comando(0x0C);
	lcd_comando(0x06);
	lcd_comando(0x01);
}

// Muestra dos textos de hasta 16 caracteres y limpia el resto.
void lcd_linea(uint8_t fila, const char *texto)
{
	uint8_t i = 0;

	lcd_comando((fila == 0) ? 0x80 : 0xC0);

	while (i < 16 && *texto)
	{
		lcd_dato(*texto++);
		i++;
	}

	while (i++ < 16)
	lcd_dato(' ');
}

void lcd_mostrar(const char *primera, const char *segunda)
{
	lcd_linea(0, primera);
	lcd_linea(1, segunda);
}


//------------------------------------------------------------
// TECLADO
//------------------------------------------------------------

// Devuelve 1 si el pulsador indicado esta presionado.
uint8_t leer_tecla(uint8_t indice)
{
	static const uint8_t bits_d[5] = {PD2, PD4, PD5, PD6, PD7};
	static const uint8_t bits_b[3] = {PB0, PB2, PB3};

	if (indice < 5)
	return (PIND & (1 << bits_d[indice])) == 0;

	return (PINB & (1 << bits_b[indice - 5])) == 0;
}

// Actualiza los ocho pulsadores con antirrebote de 30 ms.
void actualizar_teclas(void)
{
	for (uint8_t i = 0; i < TECLAS; i++)
	{
		uint8_t lectura = leer_tecla(i);

		if (lectura != muestra_previa[i])
		{
			muestra_previa[i] = lectura;
			contador_rebote[i] = 1;
		}
		else if (contador_rebote[i] < ANTIRREBOTE)
		{
			contador_rebote[i]++;
		}

		if (contador_rebote[i] >= ANTIRREBOTE)
		tecla_estable[i] = lectura;
	}
}

// Retorna la primera nota activa (1-8), o 0 si ninguna esta pulsada.
uint8_t nota_presionada(void)
{
	for (uint8_t i = 0; i < TECLAS; i++)
	{
		if (tecla_estable[i])
		return i + 1;
	}

	return 0;
}

void actualizar_piano(void)
{
	uint8_t nota = nota_presionada();

	if (nota == nota_piano)
	return;

	nota_piano = nota;
	reproducir_melodia(nota);

	if (nota == 0)
	lcd_mostrar("Modo piano", "Pulse una nota");
	else
	{
		char linea[17] = "Nota: ";
		uint8_t i = 6;
		const char *nombre = nombres_melodia[nota];

		while (*nombre && i < 16)
		linea[i++] = *nombre++;

		linea[i] = '\0';
		lcd_mostrar("Modo piano", linea);
	}
}


//------------------------------------------------------------
// GENERACION DE TONOS
//------------------------------------------------------------

// Genera melodia mediante Timer1 Fast PWM, OC1A y prescaler 8.
// Entrada: nota de 1 a 15; 0 detiene el buzzer.
void reproducir_melodia(uint8_t nota)
{
	if (nota == nota_melodia_activa)
	return;

	nota_melodia_activa = nota;
	TCCR1A = 0;
	TCCR1B = 0;
	PORTB &= ~(1 << VOZ_MELODIA);

	if (nota == 0 || nota > 15)
	return;

	uint16_t top = pgm_read_word(&top_melodia[nota]);
	TCNT1 = 0;
	ICR1 = top;
	OCR1A = (top + 1) / 2;

	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
}

// Genera bajo mediante Timer2 Fast PWM, OC2B y prescaler 1024.
// Entrada: nota de 1 a 19; 0 detiene el buzzer.
void reproducir_bajo(uint8_t nota)
{
	if (nota == nota_bajo_activa)
	return;

	nota_bajo_activa = nota;
	TCCR2A = 0;
	TCCR2B = 0;
	PORTD &= ~(1 << VOZ_BAJO);

	if (nota == 0 || nota > 19)
	return;

	uint8_t top = pgm_read_byte(&top_bajo[nota]);
	TCNT2 = 0;
	OCR2A = top;
	OCR2B = (top + 1) / 2;

	TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
}

void silenciar_voces(void)
{
	reproducir_melodia(0);
	reproducir_bajo(0);
}


//------------------------------------------------------------
// REPRODUCCION DE CANCIONES
//------------------------------------------------------------

void mostrar_paso(uint8_t melodia, uint8_t bajo)
{
	char texto[17] = "M:      B:      ";
	const char *nombre;
	uint8_t i;


	nombre = nombres_melodia[melodia];
	i = 2;
	while (*nombre && i < 7)
	texto[i++] = *nombre++;

	nombre = nombres_bajo[bajo];
	i = 10;
	while (*nombre && i < 15)
	texto[i++] = *nombre++;

	if (cancion_actual == 0)
	lcd_mostrar("Who Can It Be?", texto);
	else if (cancion_actual == 1)
	lcd_mostrar("Pideme la luna", texto);
	else
	lcd_mostrar("All Star", texto);
}

// Lee el siguiente paso desde Flash y actualiza ambas voces.
void reproducir_siguiente_paso(uint16_t ahora)
{
	const paso_t *paso = &canciones[cancion_actual][indice_paso];
	uint16_t duracion = pgm_read_word(&paso->duracion_ms);

	if (duracion == 0)
	{
		volver_a_piano();
		uart_enviar_cadena("Cancion finalizada. Modo piano.\r\n");
		return;
	}

	uint8_t melodia = pgm_read_byte(&paso->melodia);
	uint8_t bajo = pgm_read_byte(&paso->bajo);

	if (melodia > 15 || bajo > 19)
	{
		volver_a_piano();
		uart_enviar_cadena("Error en datos de la cancion.\r\n");
		return;
	}

	uint8_t actualizar_lcd = (melodia != nota_melodia_activa || indice_paso == 0);
	reproducir_melodia(melodia);
	reproducir_bajo(bajo);

	if (actualizar_lcd)
	mostrar_paso(melodia, bajo);

	inicio_paso_ms = ahora;
	duracion_paso_ms = duracion;
	indice_paso++;
}

// Selecciona C1, C2 o C3 si existen datos cargados.
void iniciar_cancion(uint8_t numero)
{
	silenciar_voces();
	nota_piano = 0;
	modo_actual = MODO_CANCION;
	cancion_actual = numero;
	indice_paso = 0;

	if (numero == 0)
	uart_enviar_cadena("C1: Who Can It Be Now.\r\n");
	else if (numero == 1)
	uart_enviar_cadena("C2: Pideme la luna.\r\n");
	else
	uart_enviar_cadena("C3: All Star.\r\n");

	reproducir_siguiente_paso(obtener_tiempo_ms());
}

// Comprueba ambas salidas simultaneamente durante un segundo.
void iniciar_prueba(void)
{
	silenciar_voces();
	nota_piano = 0;
	modo_actual = MODO_PRUEBA;
	inicio_prueba_ms = obtener_tiempo_ms();

	reproducir_melodia(1);
	reproducir_bajo(1);
	lcd_mostrar("Prueba 2 voces", "Do4 + Do3");
	uart_enviar_cadena("Prueba de dos voces: Do4 y Do3.\r\n");
}

void volver_a_piano(void)
{
	silenciar_voces();
	modo_actual = MODO_PIANO;
	nota_piano = 0;
	lcd_mostrar("Modo piano", "Pulse una nota");
}


//------------------------------------------------------------
// INTERRUPCIONES
//------------------------------------------------------------

// Base de tiempo de 1 ms para teclado y duracion de notas.
ISR(TIMER0_COMPA_vect)
{
	tiempo_ms++;
}

// Lectura atomica del contador de 16 bits actualizado por Timer0.
uint16_t obtener_tiempo_ms(void)
{
	uint16_t valor;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		valor = tiempo_ms;
	}

	return valor;
}
