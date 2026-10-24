//============================================================
// Laboratorio 4 - Tecnologias de Microprocesamiento
// Grupo 7
// Problema D - Juego de memoria Simon
//============================================================

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>

//------------------------------------------------------------
// Definiciones y constantes
//------------------------------------------------------------

#define LEDS_TOTAL              64u
#define SECUENCIA_MAX           32u
#define COLORES_TOTAL           4u
#define LED_BRILLO              32u

#define JOYSTICK_SW             PD2
#define MATRIZ_DIN              PD4
#define BUZZER_OC1A             PB1

#define TIEMPO_ELEMENTO_MS      500UL
#define TIEMPO_PAUSA_MS         200UL
#define TIEMPO_INICIO_MS       1800UL
#define TIEMPO_LIBRE_MS        3000UL
#define TIEMPO_EXITO_MS        1000UL
#define TIEMPO_ERROR_MS        1200UL
#define TIEMPO_REBOTE_MS         30UL
#define TIEMPO_JOYSTICK_MS      170UL
#define TIEMPO_MUESTREO_MS       25UL
#define ADC_ZONA_MUERTA        170u

#define EEPROM_MARCA           0x53u
#define EEPROM_VERSION         0x01u
#define EEPROM_ESPERA          0u
#define EEPROM_JUGANDO         1u
#define EEPROM_TERMINADA       2u

#define COLOR_ROJO              0u
#define COLOR_VERDE             1u
#define COLOR_AZUL              2u
#define COLOR_AMARILLO          3u

typedef struct {
	uint8_t posicion;          // 0..63 = fila*8 + columna
	uint8_t color;             // 0..3
} Elemento;

typedef enum {
	ESTADO_INICIO,
	ESTADO_MODO_LIBRE,
	ESTADO_PREPARAR_NIVEL,
	ESTADO_MOSTRAR_ELEMENTO,
	ESTADO_PAUSA_SECUENCIA,
	ESTADO_TURNO_JUGADOR,
	ESTADO_NIVEL_SUPERADO,
	ESTADO_ERROR
} Estado;

//------------------------------------------------------------
// Variables globales
//------------------------------------------------------------

volatile uint32_t reloj_ms = 0;

static Estado estado = ESTADO_INICIO;
static uint32_t inicio_estado_ms = 0;
static uint32_t ultimo_joystick_ms = 0;
static uint32_t ultima_muestra_ms = 0;

static Elemento secuencia[SECUENCIA_MAX];
static uint8_t longitud_secuencia = 0;
static uint8_t nivel = 1;
static uint8_t indice_reproduccion = 0;
static uint8_t indice_respuesta = 0;
static uint8_t fila_cursor = 3;
static uint8_t columna_cursor = 3;
static uint8_t color_cursor = COLOR_AZUL;

// Datos persistentes: ultimo nivel alcanzado, record y estado resumido.
static uint8_t ultimo_nivel = 1;
static uint8_t record = 0;
static uint8_t estado_persistente = EEPROM_ESPERA;

// Matriz en formato GRB: 64 * 3 = 192 bytes.
static uint8_t imagen_grb[LEDS_TOTAL * 3u];
static uint8_t imagen_pendiente = 1;
static uint8_t fotograma_anterior = 0xFFu;
static uint8_t segundo_tono_exito = 0;

static uint16_t centro_x = 512;
static uint16_t centro_y = 512;
static uint8_t joystick_direccion = 0;
static uint16_t aleatorio = 0xA53Cu;

static uint8_t boton_estable = 1;
static uint8_t boton_anterior = 1;
static uint8_t boton_leido = 1;
static uint32_t boton_cambio_ms = 0;
static uint8_t evento_boton = 0;

// Valores GRB atenuados; blanco completo no se utiliza.
static const uint8_t colores_grb[COLORES_TOTAL][3] = {
	{0, LED_BRILLO, 0},                  // rojo
	{LED_BRILLO, 0, 0},                  // verde
	{0, 0, LED_BRILLO},                  // azul
	{LED_BRILLO / 2, LED_BRILLO / 2, 0} // amarillo
};
static const uint16_t frecuencias[COLORES_TOTAL] = {262u, 330u, 392u, 523u};

// Fuente de digitos 3x5, una fila por byte; bit 2 = columna izquierda.
static const uint8_t digitos_3x5[10][5] PROGMEM = {
	{7,5,5,5,7}, {2,6,2,2,7}, {7,1,7,4,7}, {7,1,7,1,7}, {5,5,7,1,1},
	{7,4,7,1,7}, {7,4,7,5,7}, {7,1,1,1,1}, {7,5,7,5,7}, {7,5,7,1,7}
};

//------------------------------------------------------------
// Prototipos de funciones
//------------------------------------------------------------

void configurar_gpio(void);
static void configurar_timers(void);
static void configurar_adc(void);
static uint16_t leer_adc(uint8_t canal);
static uint32_t tiempo_actual(void);
static uint16_t nuevo_aleatorio(void);
static void leer_boton(uint32_t ahora);
static void actualizar_cursor(uint32_t ahora);
static void calibrar_joystick(void);
static void activar_tono(uint16_t hz);
static void apagar_tono(void);
static void borrar_imagen(void);
static void poner_led(uint8_t fila, uint8_t columna, uint8_t color);
static void dibujar_numero(uint8_t n, uint8_t color);
static void mostrar_imagen(void);
static void actualizar_imagen(uint32_t ahora);
static void eeprom_cargar(void);
static void eeprom_guardar(void);
static void entrar_estado(Estado nuevo, uint32_t ahora);
static void ejecutar_fsm(uint32_t ahora);

//------------------------------------------------------------
// Programa principal
//------------------------------------------------------------

int main(void)
{
	// Configuracion inicial
	configurar_gpio();
	configurar_adc();
	configurar_timers();
	calibrar_joystick();    // Joystick quieto y centrado al encender.
	sei();

	entrar_estado(ESTADO_INICIO, tiempo_actual());

	while (1)
	{
		uint32_t ahora = tiempo_actual();
		leer_boton(ahora);
		ejecutar_fsm(ahora);
		actualizar_imagen(ahora);
	}

	return 0;
}

//------------------------------------------------------------
// CONFIGURACION GPIO
//------------------------------------------------------------

void configurar_gpio(void)
{
	DDRC &= (uint8_t)~(_BV(PC0) | _BV(PC1));  // ADC0 / ADC1
	PORTC &= (uint8_t)~(_BV(PC0) | _BV(PC1));

	DDRD &= (uint8_t)~_BV(JOYSTICK_SW);
	PORTD |= _BV(JOYSTICK_SW);               // Pulsador activo en 0

	DDRD |= _BV(MATRIZ_DIN);
	PORTD &= (uint8_t)~_BV(MATRIZ_DIN);

	DDRB |= _BV(BUZZER_OC1A);
	PORTB &= (uint8_t)~_BV(BUZZER_OC1A);
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: base de tiempo, ADC, pulsador y PRNG
//------------------------------------------------------------

static void configurar_timers(void)
{
	// Timer0: CTC, 16 MHz / 64 / (249 + 1) = 1000 Hz.
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS01) | _BV(CS00);
	OCR0A = 249;
	TCNT0 = 0;
	TIMSK0 = _BV(OCIE0A);

	// Timer1: CTC y salida OC1A; se habilita al sonar cada nota.
	TCCR1A = 0;
	TCCR1B = _BV(WGM12);
	TIMSK1 = 0;
	apagar_tono();
}

static uint32_t tiempo_actual(void)
{
	uint32_t resultado;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { resultado = reloj_ms; }
	return resultado;
}

static void configurar_adc(void)
{
	ADMUX = _BV(REFS0);                       // Vref = AVcc
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
	DIDR0 |= _BV(ADC0D) | _BV(ADC1D);
}

static uint16_t leer_adc(uint8_t canal)
{
	ADMUX = (ADMUX & 0xF0u) | (canal & 0x0Fu);
	ADCSRA |= _BV(ADSC);
	while (ADCSRA & _BV(ADSC)) { }
	return ADC;
}

static void calibrar_joystick(void)
{
	uint32_t x = 0, y = 0;
	for (uint8_t i = 0; i < 16; ++i)
	{
		x += leer_adc(0);
		y += leer_adc(1);
		_delay_ms(2);
	}
	centro_x = (uint16_t)(x / 16u);
	centro_y = (uint16_t)(y / 16u);
}

static uint16_t nuevo_aleatorio(void)
{
	// xorshift16: pseudoaleatorio, no criptografico
	uint16_t x = aleatorio;
	x ^= (uint16_t)(x << 7);
	x ^= (uint16_t)(x >> 9);
	x ^= (uint16_t)(x << 8);
	aleatorio = x ? x : 0xA53Cu;
	return aleatorio;
}

static void leer_boton(uint32_t ahora)
{
	uint8_t muestra = (PIND & _BV(JOYSTICK_SW)) ? 1u : 0u;
	evento_boton = 0;
	if (muestra != boton_leido)
	{
		boton_leido = muestra;
		boton_cambio_ms = ahora;
	}
	if ((ahora - boton_cambio_ms >= TIEMPO_REBOTE_MS) &&
	boton_estable != boton_leido)
	{
		boton_anterior = boton_estable;
		boton_estable = boton_leido;
		if (boton_anterior && !boton_estable)
		{
			evento_boton = 1;                 // Solo flanco de bajada validado.
			aleatorio ^= (uint16_t)ahora;
			if (!aleatorio) aleatorio = 0xA53Cu;
		}
	}
}

static void actualizar_cursor(uint32_t ahora)
{
	if (ahora - ultima_muestra_ms < TIEMPO_MUESTREO_MS) return;
	ultima_muestra_ms = ahora;

	uint16_t x_adc = leer_adc(0);
	uint16_t y_adc = leer_adc(1);
	// Mezclar ruido residual del ADC, sin exigir aleatoriedad criptografica.
	aleatorio ^= (uint16_t)((x_adc & 3u) << 5);
	aleatorio ^= (uint16_t)((y_adc & 3u) << 11);
	if (!aleatorio) aleatorio = 0xA53Cu;

	int16_t dx = (int16_t)x_adc - (int16_t)centro_x;
	int16_t dy = (int16_t)y_adc - (int16_t)centro_y;
	uint16_t ax = (dx < 0) ? (uint16_t)-dx : (uint16_t)dx;
	uint16_t ay = (dy < 0) ? (uint16_t)-dy : (uint16_t)dy;
	uint8_t direccion = 0;

	// En diagonal se prioriza el eje con mayor desviacion.
	if (ax >= ay && ax > ADC_ZONA_MUERTA) direccion = (dx > 0) ? 1u : 2u;
	else if (ay > ADC_ZONA_MUERTA) direccion = (dy > 0) ? 3u : 4u;

	if (direccion == 0)
	{
		joystick_direccion = 0;
		return;
	}
	if (direccion == joystick_direccion &&
	ahora - ultimo_joystick_ms < TIEMPO_JOYSTICK_MS) return;

	joystick_direccion = direccion;
	ultimo_joystick_ms = ahora;
	uint8_t fila_vieja = fila_cursor;
	uint8_t columna_vieja = columna_cursor;

	// Orientacion provisional; invertir signos si el modulo esta girado.
	if (direccion == 1 && columna_cursor < 7) columna_cursor++;
	if (direccion == 2 && columna_cursor > 0) columna_cursor--;
	if (direccion == 3 && fila_cursor < 7) fila_cursor++;
	if (direccion == 4 && fila_cursor > 0) fila_cursor--;

	if (fila_cursor != fila_vieja || columna_cursor != columna_vieja)
	imagen_pendiente = 1;
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: buzzer por hardware en OC1A (D9)
//------------------------------------------------------------

static void activar_tono(uint16_t hz)
{
	if (hz == 0) { apagar_tono(); return; }
	// CTC con prescaler 8: f = F_CPU / (2 * 8 * (OCR1A + 1)).
	uint16_t cuenta = (uint16_t)(1000000UL / hz - 1UL);
	TCCR1B = _BV(WGM12);                  // Detener antes de cargar TOP.
	TCNT1 = 0;
	OCR1A = cuenta;
	TCCR1A = _BV(COM1A0);                  // Toggle OC1A on compare match
	TCCR1B = _BV(WGM12) | _BV(CS11);
}

static void apagar_tono(void)
{
	TCCR1A = 0;
	TCCR1B = _BV(WGM12);                  // Parar reloj del timer.
	PORTB &= (uint8_t)~_BV(BUZZER_OC1A);
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: framebuffer y WS2812B
//------------------------------------------------------------

static void borrar_imagen(void)
{
	for (uint16_t i = 0; i < LEDS_TOTAL * 3u; ++i) imagen_grb[i] = 0;
}

static uint8_t indice_fisico(uint8_t fila, uint8_t columna)
{
	return (uint8_t)(fila * 8u + columna);
}

static void poner_led(uint8_t fila, uint8_t columna, uint8_t color)
{
	if (fila > 7u || columna > 7u || color >= COLORES_TOTAL) return;
	uint16_t i = (uint16_t)indice_fisico(fila, columna) * 3u;
	imagen_grb[i] = colores_grb[color][0];
	imagen_grb[i + 1u] = colores_grb[color][1];
	imagen_grb[i + 2u] = colores_grb[color][2];
}

static void dibujar_digito(uint8_t n, uint8_t columna, uint8_t color)
{
	if (n > 9u) return;
	for (uint8_t fila = 0; fila < 5; fila++)
	{
		uint8_t bits = pgm_read_byte(&digitos_3x5[n][fila]);
		for (uint8_t c = 0; c < 3; c++)
		if (bits & (1u << (2u-c))) poner_led(fila+1u, columna+c, color);
	}
}

static void dibujar_numero(uint8_t n, uint8_t color)
{
	if (n > 99u) n = 99u;
	if (n < 10u) dibujar_digito(n, 2u, color);
	else
	{
		dibujar_digito((uint8_t)(n / 10u), 0u, color);
		dibujar_digito((uint8_t)(n % 10u), 4u, color);
	}
}

// Transmisor AVR 16 MHz.
// Nucleo de 20 ciclos/bit basado en la implementacion AVR de Adafruit NeoPixel.
// https://github.com/adafruit/Adafruit_NeoPixel

static void __attribute__((noinline)) ws2812_mostrar(void)
{
	volatile uint8_t *puerto = &PORTD;
	const uint8_t *ptr = imagen_grb;
	uint8_t byte = *ptr++;
	uint8_t alto = (uint8_t)(PORTD | _BV(PD4));
	uint8_t bajo = (uint8_t)(PORTD & (uint8_t)~_BV(PD4));
	uint8_t siguiente = bajo;
	uint8_t bits = 8u;
	uint16_t cuenta = LEDS_TOTAL * 3u;
	uint8_t sreg = SREG;

	cli();
	__asm__ volatile (
	"inicio%=:                    \n\t"
	"st   %a[puerto], %[alto]    \n\t"
	"sbrc %[byte], 7             \n\t"
	"mov  %[siguiente], %[alto]  \n\t"
	"dec  %[bits]                \n\t"
	"st   %a[puerto], %[siguiente]\n\t"
	"mov  %[siguiente], %[bajo]  \n\t"
	"breq byte_nuevo%=           \n\t"
	"rol  %[byte]                \n\t"
	"rjmp .+0                    \n\t"
	"nop                         \n\t"
	"st   %a[puerto], %[bajo]    \n\t"
	"nop                         \n\t"
	"rjmp .+0                    \n\t"
	"rjmp inicio%=               \n\t"
	"byte_nuevo%=:               \n\t"
	"ldi  %[bits], 8             \n\t"
	"ld   %[byte], %a[ptr]+      \n\t"
	"st   %a[puerto], %[bajo]    \n\t"
	"nop                         \n\t"
	"sbiw %[cuenta], 1           \n\t"
	"brne inicio%=               \n\t"
	: [puerto] "+&e" (puerto), [byte] "+&r" (byte),
	[bits] "+&r" (bits), [siguiente] "+&r" (siguiente),
	[cuenta] "+&w" (cuenta)
	: [ptr] "e" (ptr), [alto] "r" (alto), [bajo] "r" (bajo)
	: "cc", "memory"
	);

	PORTD &= (uint8_t)~_BV(PD4);
	SREG = sreg;
	_delay_us(300); // Reset/latch compatible con el modelo TEP.
}

static void mostrar_imagen(void)
{
	ws2812_mostrar();
	
	// Timer0 no cuenta todos los ticks mientras las IRQ estan apagadas.
	// Compensacion aproximada: un tick perdido por cada envio de ~1,92 ms.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { reloj_ms += 1u; }
}

static void actualizar_imagen(uint32_t ahora)
{
	// Marcas de fotograma independientes de los tiempos del joystick.
	uint32_t t = ahora - inicio_estado_ms;
	uint8_t fotograma = 0;
	if (estado == ESTADO_INICIO)
	fotograma = (t >= (TIEMPO_INICIO_MS / 2u)) ? 1u : 0u;
	else if (estado == ESTADO_NIVEL_SUPERADO || estado == ESTADO_ERROR)
	fotograma = (uint8_t)((t / 150u) & 1u);

	if (fotograma != fotograma_anterior)
	{
		fotograma_anterior = fotograma;
		imagen_pendiente = 1;
	}
	// Los WS2812 retienen su ultimo color: no reenviar 50 veces por segundo.
	// Actualizar solo ante movimientos, transiciones o fotogramas de animacion.
	if (!imagen_pendiente) return;
	imagen_pendiente = 0;
	borrar_imagen();
	switch (estado)
	{
		case ESTADO_INICIO:
		if (fotograma == 0u) dibujar_numero(ultimo_nivel, COLOR_VERDE);
		else dibujar_numero(record, COLOR_AZUL);
		break;
		case ESTADO_MODO_LIBRE:
		case ESTADO_TURNO_JUGADOR:
		poner_led(fila_cursor, columna_cursor, color_cursor);
		break;
		case ESTADO_MOSTRAR_ELEMENTO:
		{
			Elemento elemento = secuencia[indice_reproduccion];
			poner_led(elemento.posicion / 8u, elemento.posicion % 8u,
			elemento.color);
			break;
		}
		case ESTADO_NIVEL_SUPERADO:
		case ESTADO_ERROR:
		if (fotograma == 0u)
		for (uint8_t f = 0; f < 8; f++)
		for (uint8_t c = 0; c < 8; c++)
		poner_led(f, c, estado == ESTADO_ERROR ? COLOR_ROJO : COLOR_VERDE);
		break;
		default:
		// PREPARAR_NIVEL / PAUSA_SECUENCIA: matriz apagada.
		break;
	}
	mostrar_imagen();
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: EEPROM (direcciones 0..5 del anexo)
//------------------------------------------------------------

static void eeprom_cargar(void)
{
	uint8_t datos[6];
	eeprom_read_block(datos, (const void*)0, 6);
	uint8_t check = 0;
	for (uint8_t i = 0; i < 5; ++i) check ^= datos[i];

	if (datos[0] != EEPROM_MARCA || datos[1] != EEPROM_VERSION ||
	datos[5] != check || datos[2] < 1u ||
	datos[2] > SECUENCIA_MAX || datos[3] > SECUENCIA_MAX ||
	datos[4] > EEPROM_TERMINADA)
	{
		ultimo_nivel = 1;
		record = 0;
		estado_persistente = EEPROM_ESPERA;
		eeprom_guardar();
	}
	else
	{
		ultimo_nivel = datos[2];
		record = datos[3];
		estado_persistente = datos[4];
	}
}

static void eeprom_guardar(void)
{
	uint8_t datos[6] = {
		EEPROM_MARCA, EEPROM_VERSION, ultimo_nivel,
		record, estado_persistente, 0
	};
	for (uint8_t i = 0; i < 5; ++i) datos[5] ^= datos[i];
	eeprom_update_block(datos, (void*)0, 6);
}

//------------------------------------------------------------
// MAQUINA DE ESTADOS MOORE: entradas y salidas por estado
//------------------------------------------------------------

static void entrar_estado(Estado nuevo, uint32_t ahora)
{
	estado = nuevo;
	inicio_estado_ms = ahora;
	imagen_pendiente = 1;
	fotograma_anterior = 0xFFu;
	apagar_tono();

	switch (nuevo)
	{
		case ESTADO_INICIO:
		eeprom_cargar();
		nivel = 1;
		longitud_secuencia = 0;
		indice_reproduccion = 0;
		indice_respuesta = 0;
		fila_cursor = 3;
		columna_cursor = 3;
		break;
		case ESTADO_MODO_LIBRE:
		joystick_direccion = 0;
		break;
		case ESTADO_MOSTRAR_ELEMENTO:
		activar_tono(frecuencias[secuencia[indice_reproduccion].color]);
		break;
		case ESTADO_TURNO_JUGADOR:
		joystick_direccion = 0;
		indice_respuesta = 0;
		break;
		case ESTADO_NIVEL_SUPERADO:
		segundo_tono_exito = 0;
		ultimo_nivel = nivel;
		if (nivel > record) record = nivel;
		estado_persistente = EEPROM_ESPERA;
		eeprom_guardar();
		activar_tono(660u);
		break;
		case ESTADO_ERROR:
		ultimo_nivel = nivel;
		estado_persistente = EEPROM_TERMINADA;
		eeprom_guardar();
		activar_tono(160u);
		break;
		default: break;
	}
}

static void ejecutar_fsm(uint32_t ahora)
{
	uint32_t transcurrido = ahora - inicio_estado_ms;

	switch (estado)
	{
		case ESTADO_INICIO:
		if (transcurrido >= TIEMPO_INICIO_MS)
		entrar_estado(ESTADO_MODO_LIBRE, ahora);
		break;

		case ESTADO_MODO_LIBRE:
		actualizar_cursor(ahora);
		if (evento_boton)
		{
			uint8_t nuevo = (uint8_t)(nuevo_aleatorio() % (COLORES_TOTAL - 1u));
			if (nuevo >= color_cursor) nuevo++;
			color_cursor = nuevo;
			imagen_pendiente = 1;
		}
		if (transcurrido >= TIEMPO_LIBRE_MS)
		entrar_estado(ESTADO_PREPARAR_NIVEL, ahora);
		break;

		case ESTADO_PREPARAR_NIVEL:
		if (nivel <= SECUENCIA_MAX && longitud_secuencia < nivel)
		{
			// Semilla variable por ADC + reloj (no criptografica).
			aleatorio ^= (uint16_t)(leer_adc(0) & 0x03u) << 8;
			aleatorio ^= (uint16_t)ahora;
			if (!aleatorio) aleatorio = 0xA53Cu;
			secuencia[longitud_secuencia].posicion = (uint8_t)(nuevo_aleatorio() % 64u);
			secuencia[longitud_secuencia].color = (uint8_t)(nuevo_aleatorio() % COLORES_TOTAL);
			longitud_secuencia++;
		}
		indice_reproduccion = 0;
		ultimo_nivel = nivel;
		estado_persistente = EEPROM_JUGANDO;
		eeprom_guardar();
		entrar_estado(ESTADO_MOSTRAR_ELEMENTO, ahora);
		break;

		case ESTADO_MOSTRAR_ELEMENTO:
		if (transcurrido >= TIEMPO_ELEMENTO_MS)
		entrar_estado(ESTADO_PAUSA_SECUENCIA, ahora);
		break;

		case ESTADO_PAUSA_SECUENCIA:
		if (transcurrido >= TIEMPO_PAUSA_MS)
		{
			if (indice_reproduccion + 1u < longitud_secuencia)
			{
				indice_reproduccion++;
				entrar_estado(ESTADO_MOSTRAR_ELEMENTO, ahora);
			}
			else entrar_estado(ESTADO_TURNO_JUGADOR, ahora);
		}
		break;

		case ESTADO_TURNO_JUGADOR:
		actualizar_cursor(ahora);
		if (evento_boton)
		{
			uint8_t posicion = (uint8_t)(fila_cursor * 8u + columna_cursor);
			if (posicion != secuencia[indice_respuesta].posicion)
			entrar_estado(ESTADO_ERROR, ahora);
			else if (indice_respuesta + 1u == longitud_secuencia)
			entrar_estado(ESTADO_NIVEL_SUPERADO, ahora);
			else indice_respuesta++;
		}
		break;

		case ESTADO_NIVEL_SUPERADO:
		if (transcurrido >= TIEMPO_EXITO_MS)
		{
			if (nivel >= SECUENCIA_MAX) entrar_estado(ESTADO_INICIO, ahora);
			else
			{
				nivel++;
				entrar_estado(ESTADO_MODO_LIBRE, ahora);
			}
		}
		else if (transcurrido >= 500u && !segundo_tono_exito)
		{
			segundo_tono_exito = 1;
			activar_tono(880u);
		}
		break;

		case ESTADO_ERROR:
		if (transcurrido >= TIEMPO_ERROR_MS)
		entrar_estado(ESTADO_INICIO, ahora);
		break;
	}
}

//------------------------------------------------------------
// INTERRUPCIONES
//------------------------------------------------------------

ISR(TIMER0_COMPA_vect)
{
	reloj_ms++;
}
