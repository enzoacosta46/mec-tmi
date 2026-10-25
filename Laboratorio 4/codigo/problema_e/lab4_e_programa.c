//============================================================
// Laboratorio 4 - Tecnologias de Microprocesamiento
// Grupo 7
// Problema E - Burbuja de inclinacion
//============================================================

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

//------------------------------------------------------------
// Definiciones y constantes
//------------------------------------------------------------

#define LEDS_TOTAL 64U
#define LED_BRILLO 32U
#define MATRIZ_DIN PD4
#define BOTON_HOLD PD2
#define BOTON_CALIBRAR PD5
#define BUZZER_OC2B PD3

#define TIEMPO_MUESTREO_MS 50UL
#define TIEMPO_SERIAL_MS 250UL
#define TIEMPO_REBOTE_MS 30UL
#define TIEMPO_BUZZER_MS 100UL
#define TIEMPO_PARPADEO_MS 100UL
#define TIEMPO_ALERTA_MS 400UL
#define TIEMPO_REINTENTO_MS 1000UL
#define TIEMPO_CALIBRACION_MS 1500UL
#define FILTRO_MUESTRAS 5U
#define ZONA_MUERTA_GRADOS 3.0F
#define ANGULO_LIMITE_GRADOS 30.0F
#define CALIBRACION_MAX_RANGO 2.0F

#define MPU_DIRECCION 0x68U
#define MPU_REG_ACEL_X_H 0x3Bu
#define MPU_REG_CONFIG 0x1AU
#define MPU_REG_SMPLRT_DIV 0x19U
#define MPU_REG_GIRO_CONFIG 0x1BU
#define MPU_REG_ACEL_CONFIG 0x1CU
#define MPU_REG_PWR_MGMT_1 0x6BU
#define MPU_REG_WHO_AM_I 0x75U
#define TWI_ESPERA_MAX 50000U

#define EEPROM_MARCA 0xE5U
#define EEPROM_VERSION 0x01U
#define TX_TAMANO 128U
#define RAD_A_GRADOS 57.2957795F

#define COLOR_ROJO 0U
#define COLOR_VERDE 1U
#define COLOR_AZUL 2U
#define COLOR_AMARILLO 3U

typedef enum {
	ESTADO_NORMAL,
	ESTADO_HOLD,
	ESTADO_CALIBRANDO
}
Estado;

typedef struct {
	uint8_t lectura_anterior;
	uint8_t estable;
	uint32_t cambio_ms;
	uint8_t evento;
}
Pulsador;

//------------------------------------------------------------
// Variables globales
//------------------------------------------------------------

static volatile uint16_t timer1_desbordes = 0;
static Estado estado = ESTADO_NORMAL;
static uint32_t ultima_muestra_ms = 0;
static uint32_t ultimo_serial_ms = 0;
static uint32_t ultimo_reintento_ms = 0;

static Pulsador pulsador_hold = {
	1U,
	1U,
	0U,
	0U
};
static Pulsador pulsador_calibrar = {
	1U,
	1U,
	0U,
	0U
};

static uint8_t sensor_valido = 0;
static uint8_t sensor_iniciado = 0;
static float angulo_x_crudo = 0.0F;
static float angulo_y_crudo = 0.0F;
static float angulo_x_filtrado = 0.0F;
static float angulo_y_filtrado = 0.0F;
static float offset_x = 0.0F;
static float offset_y = 0.0F;
static int16_t giro_x = 0, giro_y = 0, giro_z = 0;

static float muestras_x[FILTRO_MUESTRAS];
static float muestras_y[FILTRO_MUESTRAS];
static float suma_filtro_x = 0.0F, suma_filtro_y = 0.0F;
static uint8_t filtro_indice = 0, filtro_cantidad = 0;

static uint8_t cal_cantidad = 0;
static float cal_suma_x = 0.0F, cal_suma_y = 0.0F;
static float cal_min_x = 0.0F, cal_max_x = 0.0F;
static float cal_min_y = 0.0F, cal_max_y = 0.0F;
static uint32_t inicio_calibracion_ms = 0;

static uint8_t fila_burbuja = 3U;
static uint8_t columna_burbuja = 4U;
static uint8_t color_burbuja = COLOR_AZUL;
static uint8_t borde_anterior = 0;
static uint8_t alerta_activa = 0;
static uint8_t alerta_fotograma = 0;
static uint32_t inicio_alerta_ms = 0;

static uint8_t imagen_grb[LEDS_TOTAL * 3U];
static uint8_t imagen_pendiente = 1U;

static volatile uint8_t tx_datos[TX_TAMANO];
static volatile uint8_t tx_escritura = 0, tx_lectura = 0;

static
const uint8_t colores_grb[4][3] = {
	{
		0,
		LED_BRILLO,
		0
	}, // Rojo
	{
		LED_BRILLO,
		0,
		0
	}, // Verde
	{
		0,
		0,
		LED_BRILLO
	}, // Azul
	{
		LED_BRILLO / 2,
		LED_BRILLO / 2,
		0
	} // Amarillo
};

//------------------------------------------------------------
// Prototipos de funciones
//------------------------------------------------------------

void configurar_gpio(void);
static void configurar_timers(void);
static uint32_t tiempo_actual(void);
static void configurar_uart(void);
static void uart_linea(const char * texto);
static void informar_estado(void);
static
const char * texto_estado(Estado valor);
static void leer_pulsador(Pulsador * boton, uint8_t pin, uint32_t ahora);
static void activar_tono(void);
static void apagar_tono(void);
static void actualizar_alerta(uint32_t ahora);
static void iniciar_alerta(uint32_t ahora);
static void borrar_imagen(void);
static void poner_led(uint8_t fila, uint8_t columna, uint8_t color);
static void ws2812_mostrar(void);
static void actualizar_imagen(void);
static void cargar_calibracion(void);
static void guardar_calibracion(void);
static void reiniciar_filtro(void);
static void filtrar_angulos(float x, float y);
static uint8_t mapear_angulo(float angulo, uint8_t invertido);
static uint8_t calcular_color(float x, float y);
static void procesar_inclinacion(uint32_t ahora);
static void entrar_estado(Estado nuevo, uint32_t ahora);
static void ejecutar_fsm(uint32_t ahora);
static void procesar_mediciones(uint32_t ahora);
static uint8_t obtener_angulos(float * x, float * y);
static void configurar_twi(void);
static uint8_t twi_esperar(void);
static void twi_parar(void);
static uint8_t twi_iniciar(uint8_t direccion_lectura);
static uint8_t twi_enviar(uint8_t dato);
static uint8_t twi_recibir(uint8_t * dato, uint8_t confirmar);
static uint8_t mpu_leer_registro(uint8_t registro, uint8_t * dato);
static uint8_t mpu_escribir_registro(uint8_t registro, uint8_t dato);
static uint8_t mpu_iniciar(void);
static uint8_t mpu_leer(float * x, float * y);

//------------------------------------------------------------
// Programa principal
//------------------------------------------------------------

int main(void) {
	// Configuracion inicial
	configurar_gpio();
	configurar_timers();
	configurar_uart();
	cargar_calibracion();
	configurar_twi();
	sei();
	// Primer intento inmediato; posteriormente reintenta cada segundo.
	sensor_iniciado = mpu_iniciar();
	ultimo_reintento_ms = tiempo_actual();

	uart_linea("PROBLEMA E:BURBUJA_INCLINACION");

	entrar_estado(ESTADO_NORMAL, tiempo_actual());

	while (1) {
		uint32_t ahora = tiempo_actual();
		leer_pulsador( & pulsador_hold, BOTON_HOLD, ahora);
		leer_pulsador( & pulsador_calibrar, BOTON_CALIBRAR, ahora);
		procesar_mediciones(ahora);
		ejecutar_fsm(ahora);
		actualizar_alerta(ahora);
		actualizar_imagen();

		if ((uint32_t)(ahora - ultimo_serial_ms) >= TIEMPO_SERIAL_MS) {
			ultimo_serial_ms = ahora;
			informar_estado();
		}
	}
	return 0;
}

//------------------------------------------------------------
// CONFIGURACION GPIO
//------------------------------------------------------------

void configurar_gpio(void) {
	// Pulsadores activos en bajo con resistencias pull-up internas.
	DDRD &= (uint8_t) ~(_BV(BOTON_HOLD) | _BV(BOTON_CALIBRAR));
	PORTD |= _BV(BOTON_HOLD) | _BV(BOTON_CALIBRAR);

	// Matriz WS2812 y buzzer pasivo en salida OC2B.
	DDRD |= _BV(MATRIZ_DIN) | _BV(BUZZER_OC2B);
	PORTD &= (uint8_t) ~(_BV(MATRIZ_DIN) | _BV(BUZZER_OC2B));
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: reloj, UART y pulsadores
//------------------------------------------------------------

static void configurar_timers(void) {
	// Timer1 libre a /64 = 250000 cuentas/s. Desborde cada 262,144 ms.
	// Sigue contando durante los ~2 ms con IRQ deshabilitadas por WS2812.
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCNT1 = 0;
	TIFR1 = _BV(TOV1);
	TIMSK1 = _BV(TOIE1);

	// Timer2: buzzer PASIVO en OC2B/PD3; PWM de 2 kHz, ~50 %.
	TCCR2A = _BV(WGM21) | _BV(WGM20);
	TCCR2B = 0;
	TCNT2 = 0;
	OCR2A = 124U;
	OCR2B = 62U;
	TCCR2B = _BV(WGM22) | _BV(CS22); // Prescaler 64.
	apagar_tono();
}

static uint32_t tiempo_actual(void) {
	uint16_t desbordes, cuenta;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		desbordes = timer1_desbordes;
		cuenta = TCNT1;
		// Desborde pendiente antes de ejecutarse la ISR.
		if ((TIFR1 & _BV(TOV1)) && cuenta < 0x8000U) desbordes++;
	}
	// 250 ticks de Timer1 equivalen a 1 ms.
	return ((((uint32_t) desbordes << 16) | cuenta) / 250UL);
}

static void configurar_uart(void) {
	UCSR0A = 0;
	UBRR0H = 0;
	UBRR0L = 103U; // 9600 baudios, 8N1, reloj 16 MHz.
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	UCSR0B = _BV(TXEN0);
}

static void uart_linea(const char * texto) {
	// Cola no bloqueante.
	uint8_t longitud = 0;
	while (texto[longitud] && longitud < 120U) longitud++;
	if (texto[longitud]) return;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		uint8_t libres = (uint8_t)((tx_lectura - tx_escritura - 1U) & (TX_TAMANO - 1U));
		if ((uint8_t)(longitud + 2U) <= libres) {
			for (uint8_t i = 0; i < longitud; i++) {
				tx_datos[tx_escritura] = (uint8_t) texto[i];
				tx_escritura = (uint8_t)((tx_escritura + 1U) & (TX_TAMANO - 1U));
			}
			tx_datos[tx_escritura] = '\r';
			tx_escritura = (uint8_t)((tx_escritura + 1U) & (TX_TAMANO - 1U));
			tx_datos[tx_escritura] = '\n';
			tx_escritura = (uint8_t)((tx_escritura + 1U) & (TX_TAMANO - 1U));
			UCSR0B |= _BV(UDRIE0);
		}
	}
}

static const char * texto_estado(Estado valor) {
	switch (valor) {
		case ESTADO_NORMAL:
		return "NORMAL";
		case ESTADO_HOLD:
		return "HOLD";
		case ESTADO_CALIBRANDO:
		return "CALIBRANDO";
		default:
		return "DESCONOCIDO";
	}
}

static void formato_angulo(float grados, char * texto, uint8_t tamano) {
	if (tamano < 8U) return;

	int16_t valor = (int16_t) (grados * 100.0F + (grados >= 0.0F ? 0.5F : -0.5F));

	uint16_t absoluto = (valor < 0) ? (uint16_t)(-valor) : (uint16_t) valor;

	uint16_t entero = absoluto / 100U;
	uint8_t decimales = absoluto % 100U;

	uint8_t i = 0;

	texto[i++] = (valor < 0) ? '-' : '+';

	if (entero >= 100U)
	    texto[i++] = '0' + (entero / 100U);

	if (entero >= 10U)
	    texto[i++] = '0' + ((entero / 10U) % 10U);

	texto[i++] = '0' + (entero % 10U);
	texto[i++] = '.';
	texto[i++] = '0' + (decimales / 10U);
	texto[i++] = '0' + (decimales % 10U);
	texto[i] = '\0';
}

static uint8_t agregar_texto(char * destino, uint8_t pos, const char * origen) {
	while ( * origen && pos < 63U) {
		destino[pos++] = * origen++;
	}

	destino[pos] = '\0';
	return pos;
}

static void informar_estado(void)
{
	if (!sensor_valido)
	{
		uart_linea("MPU:SIN_LECTURA");
		return;
	}

	char pitch[12];
	char roll[12];
	char linea[80];

	formato_angulo(-angulo_x_filtrado, pitch, sizeof(pitch));
	formato_angulo(angulo_y_filtrado, roll, sizeof(roll));

	uint8_t i = 0;

	i = agregar_texto(linea, i, "ESTADO:");
	i = agregar_texto(linea, i, texto_estado(estado));
	i = agregar_texto(linea, i, ";PITCH:");
	i = agregar_texto(linea, i, pitch);
	i = agregar_texto(linea, i, ";ROLL:");
	i = agregar_texto(linea, i, roll);
	i = agregar_texto(linea, i, ";F:");

	linea[i++] = '0' + fila_burbuja;

	i = agregar_texto(linea, i, ";C:");

	linea[i++] = '0' + columna_burbuja;
	linea[i] = '\0';

	uart_linea(linea);
}

static void leer_pulsador(Pulsador * boton, uint8_t pin, uint32_t ahora) {
	uint8_t muestra = (PIND & _BV(pin)) ? 1U : 0U;
	boton -> evento = 0U;
	if (muestra != boton -> lectura_anterior) {
		boton -> lectura_anterior = muestra;
		boton -> cambio_ms = ahora;
	}
	if ((uint32_t)(ahora - boton -> cambio_ms) >= TIEMPO_REBOTE_MS && muestra != boton -> estable) {
		boton -> estable = muestra;
		if (!muestra) boton -> evento = 1U; // Flanco de pulsacion.
	}
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: buzzer pasivo y WS2812B
//------------------------------------------------------------

static void activar_tono(void) {
	TCCR2A |= _BV(COM2B1); // Habilita PWM sobre el pin OC2B.
}

static void apagar_tono(void) {
	TCCR2A &= (uint8_t) ~_BV(COM2B1);
	PORTD &= (uint8_t) ~_BV(BUZZER_OC2B);
}

static void iniciar_alerta(uint32_t ahora) {
	alerta_activa = 1U;
	inicio_alerta_ms = ahora;
	alerta_fotograma = 0U;
	activar_tono();
	imagen_pendiente = 1U;
}

static void actualizar_alerta(uint32_t ahora) {
	if (!alerta_activa) return;
	uint32_t transcurrido = ahora - inicio_alerta_ms;
	if (transcurrido >= TIEMPO_BUZZER_MS) apagar_tono();
	if (transcurrido >= TIEMPO_ALERTA_MS) {
		alerta_activa = 0U;
		alerta_fotograma = 0U;
		imagen_pendiente = 1U;
		return;
	}
	uint8_t nuevo = (uint8_t)((transcurrido / TIEMPO_PARPADEO_MS) & 1U);
	if (nuevo != alerta_fotograma) {
		alerta_fotograma = nuevo;
		imagen_pendiente = 1U;
	}
}

static void borrar_imagen(void) {
	for (uint16_t i = 0; i < LEDS_TOTAL * 3U; ++i) imagen_grb[i] = 0U;
}

static void poner_led(uint8_t fila, uint8_t columna, uint8_t color) {
	if (fila > 7U || columna > 7U || color > COLOR_AMARILLO) return;
	uint16_t indice = (uint16_t)(fila * 8U + columna) * 3U;
	imagen_grb[indice] = colores_grb[color][0];
	imagen_grb[indice + 1U] = colores_grb[color][1];
	imagen_grb[indice + 2U] = colores_grb[color][2];
}

// Emisor de 20 ciclos/bit para ATmega328P a 16 MHz.
static void __attribute__((noinline)) ws2812_mostrar(void) {
	volatile uint8_t * puerto = & PORTD;
	const uint8_t * ptr = imagen_grb;
	uint8_t byte = * ptr++;
	uint8_t alto = (uint8_t)(PORTD | _BV(PD4));
	uint8_t bajo = (uint8_t)(PORTD & (uint8_t) ~_BV(PD4));
	uint8_t siguiente = bajo;
	uint8_t bits = 8U;
	uint16_t cuenta = LEDS_TOTAL * 3U;
	uint8_t sreg = SREG;

	cli();
	__asm__ volatile(
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
	"brne inicio%=               \n\t": [puerto]
	"+&e"(puerto), [byte]
	"+&r"(byte),
	[bits]
	"+&r"(bits), [siguiente]
	"+&r"(siguiente),
	[cuenta]
	"+&w"(cuenta): [ptr]
	"e"(ptr), [alto]
	"r"(alto), [bajo]
	"r"(bajo): "cc", "memory"
	);

	PORTD &= (uint8_t) ~_BV(PD4);
	SREG = sreg;
	_delay_us(300); // Reset/latch del modelo TEP.
}

static void actualizar_imagen(void) {
	if (!imagen_pendiente) return;
	imagen_pendiente = 0U;
	borrar_imagen();

	// En NORMAL, sin sensor valido, la matriz permanece apagada.
	// En HOLD conserva la posicion capturada incluso si falla una lectura.
	if ((sensor_valido || estado == ESTADO_HOLD || estado == ESTADO_CALIBRANDO) &&
	    !(alerta_activa && alerta_fotograma == 0U)) {

		poner_led(fila_burbuja, columna_burbuja, color_burbuja);
	}
	ws2812_mostrar();
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: EEPROM y procesamiento angular
//------------------------------------------------------------

static void cargar_calibracion(void) {
	uint8_t datos[7];
	eeprom_read_block(datos, (const void * ) 0, sizeof(datos));
	uint8_t comprobacion = 0U;
	for (uint8_t i = 0; i < 6U; ++i) comprobacion ^= datos[i];
	
    int16_t ox = (int16_t)((uint16_t) datos[2] | ((uint16_t) datos[3] << 8));
	int16_t oy = (int16_t)((uint16_t) datos[4] | ((uint16_t) datos[5] << 8));
	
    if (datos[0] == EEPROM_MARCA && datos[1] == EEPROM_VERSION &&
	    datos[6] == comprobacion && ox >= -9000 && ox <= 9000 &&
	    oy >= -9000 && oy <= 9000) {

		offset_x = (float) ox / 100.0F;
		offset_y = (float) oy / 100.0F;
    } else {
		offset_x = 0.0F;
		offset_y = 0.0F;
	}
}

static void guardar_calibracion(void) {
	int16_t ox = (int16_t)(offset_x * 100.0F);
	int16_t oy = (int16_t)(offset_y * 100.0F);
	uint8_t datos[7] = {
		EEPROM_MARCA,
		EEPROM_VERSION,
		(uint8_t) ox,
		(uint8_t)((uint16_t) ox >> 8),
		(uint8_t) oy,
		(uint8_t)((uint16_t) oy >> 8),
		0U
	};
	for (uint8_t i = 0; i < 6U; ++i) datos[6] ^= datos[i];
	// Reduce escrituras innecesarias y desgaste.
	eeprom_update_block(datos, (void * ) 0, sizeof(datos));
}

static void reiniciar_filtro(void) {
	filtro_indice = 0U;
	filtro_cantidad = 0U;
	suma_filtro_x = 0.0F;
	suma_filtro_y = 0.0F;
	angulo_x_filtrado = 0.0F;
	angulo_y_filtrado = 0.0F;
}

static void filtrar_angulos(float x, float y) {
	if (filtro_cantidad >= FILTRO_MUESTRAS) {
		suma_filtro_x -= muestras_x[filtro_indice];
		suma_filtro_y -= muestras_y[filtro_indice];
	} else filtro_cantidad++;

	muestras_x[filtro_indice] = x;
	muestras_y[filtro_indice] = y;
	suma_filtro_x += x;
	suma_filtro_y += y;
	filtro_indice = (uint8_t)((filtro_indice + 1U) % FILTRO_MUESTRAS);
	angulo_x_filtrado = suma_filtro_x / (float) filtro_cantidad;
	angulo_y_filtrado = suma_filtro_y / (float) filtro_cantidad;
}

static uint8_t mapear_angulo(float angulo, uint8_t invertido) {
	if (angulo < -ANGULO_LIMITE_GRADOS) angulo = -ANGULO_LIMITE_GRADOS;
	if (angulo > ANGULO_LIMITE_GRADOS) angulo = ANGULO_LIMITE_GRADOS;
	// Redondeo al entero mas cercano; empate hacia el entero superior.
	uint8_t coordenada = (uint8_t)(((angulo + 30.0F) * 7.0F / 60.0F) + 0.5F);
	if (coordenada > 7U) coordenada = 7U;
	return invertido ? (uint8_t)(7U - coordenada) : coordenada;
}

static uint8_t calcular_color(float x, float y) {
	float ax = fabsf(x), ay = fabsf(y);
	if (ax <= ZONA_MUERTA_GRADOS) ax = 0.0F;
	if (ay <= ZONA_MUERTA_GRADOS) ay = 0.0F;
	if (ax == 0.0F && ay == 0.0F) return COLOR_AZUL;
	if (ax >= 2.0F * ay) return COLOR_ROJO;
	if (ay >= 2.0F * ax) return COLOR_VERDE;
	return COLOR_AMARILLO;
}

static void procesar_inclinacion(uint32_t ahora) {
	float x = angulo_x_filtrado, y = angulo_y_filtrado;
	uint8_t fila, columna, color;

	if (fabsf(x) <= ZONA_MUERTA_GRADOS && fabsf(y) <= ZONA_MUERTA_GRADOS) {
		fila = 3U;
		columna = 4U;
		color = COLOR_AZUL;
    } else {
		fila = mapear_angulo(y, 1U);
		columna = mapear_angulo(x, 0U);
		color = calcular_color(x, y);
	}

	if (fila != fila_burbuja || columna != columna_burbuja || color != color_burbuja) {
		fila_burbuja = fila;
		columna_burbuja = columna;
		color_burbuja = color;
		imagen_pendiente = 1U;
	}

	uint8_t borde = (fila == 0U || fila == 7U || columna == 0U || columna == 7U);
	if (borde && !borde_anterior) iniciar_alerta(ahora);
	borde_anterior = borde;
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: FSM con tres estados
//------------------------------------------------------------

static void entrar_estado(Estado nuevo, uint32_t ahora) {
	estado = nuevo;
	// Finalizar alertas anteriores antes de cualquier cambio de modo.
	alerta_activa = 0U;
	alerta_fotograma = 0U;
	apagar_tono();
	imagen_pendiente = 1U;

	if (nuevo == ESTADO_CALIBRANDO) {
		inicio_calibracion_ms = ahora;
		cal_cantidad = 0U;
		cal_suma_x = 0.0F;
		cal_suma_y = 0.0F;
	}
	// HOLD conserva fila/color y no resetea el historial del filtro.
	// El estado CALIBRANDO mantiene el ultimo LED visible.
}

static void ejecutar_fsm(uint32_t ahora) {
	switch (estado) {
		case ESTADO_NORMAL:
		if (pulsador_hold.evento) {
			entrar_estado(ESTADO_HOLD, ahora);
			uart_linea("EVENTO:HOLD_ON");
        } else if (pulsador_calibrar.evento && sensor_valido) {
			entrar_estado(ESTADO_CALIBRANDO, ahora);
			uart_linea("EVENTO:CALIBRANDO");
		}
		break;

		case ESTADO_HOLD:
		if (pulsador_hold.evento) {
			entrar_estado(ESTADO_NORMAL, ahora);
			uart_linea("EVENTO:HOLD_OFF");
			if (sensor_valido) procesar_inclinacion(ahora);
		}
		break;

		case ESTADO_CALIBRANDO:
		if ((uint32_t)(ahora - inicio_calibracion_ms) >= TIEMPO_CALIBRACION_MS) {
			uart_linea("ERROR:CALIBRACION_TIMEOUT");
			entrar_estado(ESTADO_NORMAL, ahora);
			if (sensor_valido) procesar_inclinacion(ahora);
		}
		break;
	}
}

static void procesar_mediciones(uint32_t ahora) {
	if ((uint32_t)(ahora - ultima_muestra_ms) < TIEMPO_MUESTREO_MS) return;
	ultima_muestra_ms = ahora;

	if (!sensor_iniciado) {
		if ((uint32_t)(ahora - ultimo_reintento_ms) < TIEMPO_REINTENTO_MS) return;
		ultimo_reintento_ms = ahora;
		sensor_iniciado = mpu_iniciar();
		if (!sensor_iniciado) {
			sensor_valido = 0U;
			imagen_pendiente = 1U;
			return;
		}
		uart_linea("MPU:DETECTADO");
	}

	float x, y;

	if (!obtener_angulos( & x, & y)) {
		if (sensor_valido) uart_linea("ERROR:LECTURA_MPU");

		sensor_valido = 0U;
		alerta_activa = 0U;
		alerta_fotograma = 0U;
		apagar_tono();
		sensor_iniciado = 0U;
		ultimo_reintento_ms = ahora;
		if (estado == ESTADO_CALIBRANDO) {
			uart_linea("ERROR:CALIBRACION_SIN_SENSOR");
			entrar_estado(ESTADO_NORMAL, ahora);
		}
		imagen_pendiente = 1U;

		reiniciar_filtro();

		return;
	}

	if (!sensor_valido) {
		imagen_pendiente = 1U;
	}

	sensor_valido = 1U;
	angulo_x_crudo = x;
	angulo_y_crudo = y;

	if (estado == ESTADO_CALIBRANDO) {
		if (cal_cantidad == 0U) {
			cal_min_x = cal_max_x = x;
			cal_min_y = cal_max_y = y;
		}

		if (x < cal_min_x) cal_min_x = x;
		if (x > cal_max_x) cal_max_x = x;
		if (y < cal_min_y) cal_min_y = y;
		if (y > cal_max_y) cal_max_y = y;

		cal_suma_x += x;
		cal_suma_y += y;
		cal_cantidad++;

		if (cal_cantidad == FILTRO_MUESTRAS) {
			if ((cal_max_x - cal_min_x) <= CALIBRACION_MAX_RANGO &&
			    (cal_max_y - cal_min_y) <= CALIBRACION_MAX_RANGO) {
				
                offset_x = cal_suma_x / (float) FILTRO_MUESTRAS;
				offset_y = cal_suma_y / (float) FILTRO_MUESTRAS;

				guardar_calibracion();
				reiniciar_filtro();

				fila_burbuja = 3U;
				columna_burbuja = 4U;
				color_burbuja = COLOR_AZUL;
				borde_anterior = 0U;
				imagen_pendiente = 1U;
				uart_linea("OK:CALIBRACION_EEPROM");
			} else uart_linea("ERROR:CALIBRACION_INESTABLE");

			entrar_estado(ESTADO_NORMAL, ahora);
			// Tras calibrar esperamos la proxima medicion antes de dibujar.
		}
		return;
	}

	filtrar_angulos(x - offset_x, y - offset_y);
	if (estado == ESTADO_NORMAL) procesar_inclinacion(ahora);
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES: comunicacion TWI / MPU6050
//------------------------------------------------------------

static void configurar_twi(void) {
	// SDA=PC4, SCL=PC5. Pull-ups deben ser externas y compatibles
	DDRC &= (uint8_t) ~(_BV(PC4) | _BV(PC5));
	PORTC &= (uint8_t) ~(_BV(PC4) | _BV(PC5));
	TWSR = 0U; // Prescaler 1.
	TWBR = 72U; // 100 kHz con F_CPU = 16 MHz.
	TWCR = _BV(TWEN);
}

static uint8_t twi_esperar(void) {
	uint16_t espera = TWI_ESPERA_MAX;
	while (!(TWCR & _BV(TWINT)) && --espera) {}
	return (TWCR & _BV(TWINT)) != 0U;
}

static void twi_parar(void) {
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

static uint8_t twi_iniciar(uint8_t direccion_lectura) {
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	if (!twi_esperar()) return 0U;
	uint8_t estado_twi = (TWSR & 0xF8U);
	if (estado_twi != 0x08U && estado_twi != 0x10U) return 0U;
	TWDR = direccion_lectura;
	TWCR = _BV(TWINT) | _BV(TWEN);
	if (!twi_esperar()) return 0U;
	return (TWSR & 0xF8U) == ((direccion_lectura & 1U) ? 0x40U : 0x18U);
}

static uint8_t twi_enviar(uint8_t dato) {
	TWDR = dato;
	TWCR = _BV(TWINT) | _BV(TWEN);
	return twi_esperar() && ((TWSR & 0xF8U) == 0x28U);
}

static uint8_t twi_recibir(uint8_t * dato, uint8_t confirmar) {
	TWCR = _BV(TWINT) | _BV(TWEN) | (confirmar ? _BV(TWEA) : 0U);
	if (!twi_esperar()) return 0U;
	if ((TWSR & 0xF8U) != (confirmar ? 0x50U : 0x58U)) return 0U;
	* dato = TWDR;
	return 1U;
}

static uint8_t mpu_leer_bloque(uint8_t registro, uint8_t * buffer, uint8_t cantidad) {
	uint8_t correcto = 0U;
	if (!cantidad) return 0U;
	if (!twi_iniciar((uint8_t)(MPU_DIRECCION << 1))) goto fin;
	if (!twi_enviar(registro)) goto fin;
	if (!twi_iniciar((uint8_t)((MPU_DIRECCION << 1) | 1U))) goto fin;
	for (uint8_t i = 0; i < cantidad; ++i) {
		if (!twi_recibir( & buffer[i], (uint8_t)(i + 1U < cantidad))) goto fin;
	}
	correcto = 1U;
	fin:
	twi_parar();
	return correcto;
}

static uint8_t mpu_leer_registro(uint8_t registro, uint8_t * dato) {
	return mpu_leer_bloque(registro, dato, 1U);
}

static uint8_t mpu_escribir_registro(uint8_t registro, uint8_t dato) {
	uint8_t correcto = 0U;
	if (!twi_iniciar((uint8_t)(MPU_DIRECCION << 1))) goto fin;
	if (!twi_enviar(registro)) goto fin;
	if (!twi_enviar(dato)) goto fin;
	correcto = 1U;
	fin:
	twi_parar();
	return correcto;
}

static uint8_t mpu_iniciar(void) {
	uint8_t identificacion = 0U;
	if (!mpu_leer_registro(MPU_REG_WHO_AM_I, & identificacion)) return 0U;
	if (identificacion != 0x68U) return 0U;
	// Despertar; acelerometro +/-2g, giroscopio +/-250 grados/s.
	if (!mpu_escribir_registro(MPU_REG_PWR_MGMT_1, 0x01U)) return 0U;
	if (!mpu_escribir_registro(MPU_REG_CONFIG, 0x03U)) return 0U;
	// DLPF activo: reloj interno 1 kHz; 1 kHz / (1 + 9) = 100 Hz.
	// El ATmega mantiene su lectura a 20 Hz (cada 50 ms).
	if (!mpu_escribir_registro(MPU_REG_SMPLRT_DIV, 9U)) return 0U;
	if (!mpu_escribir_registro(MPU_REG_ACEL_CONFIG, 0x00U)) return 0U;
	if (!mpu_escribir_registro(MPU_REG_GIRO_CONFIG, 0x00U)) return 0U;
	return 1U;
}

static int16_t leer_i16(const uint8_t * buffer) {
	return (int16_t)((uint16_t)((uint16_t) buffer[0] << 8) | buffer[1]);
}

static uint8_t mpu_leer(float * x, float * y) {
	uint8_t datos[14];
	if (!mpu_leer_bloque(MPU_REG_ACEL_X_H, datos, sizeof(datos))) return 0U;
	int16_t ax = leer_i16( & datos[0]);
	int16_t ay = leer_i16( & datos[2]);
	int16_t az = leer_i16( & datos[4]);
	// datos[6..7] temperatura, datos[8..13] velocidades angulares.
	giro_x = leer_i16( & datos[8]);
	giro_y = leer_i16( & datos[10]);
	giro_z = leer_i16( & datos[12]);
	float fx = (float) ax, fy = (float) ay, fz = (float) az;
	if (ax == 0 && ay == 0 && az == 0) return 0U; // Dato improbable.
	// Convencion: X+ eleva borde derecho; Y+ eleva borde superior.
	// Pitch = atan2(-ax, sqrt(ay^2+az^2)); X = -Pitch.
	*x = atan2f(fx, sqrtf(fy * fy + fz * fz)) * RAD_A_GRADOS;
	*y = atan2f(fy, sqrtf(fx * fx + fz * fz)) * RAD_A_GRADOS;
	return 1U;
}

static uint8_t obtener_angulos(float * x, float * y) {
	return mpu_leer(x, y);
}

//------------------------------------------------------------
// INTERRUPCIONES
//------------------------------------------------------------

ISR(TIMER1_OVF_vect) {
	timer1_desbordes++;
}

ISR(USART_UDRE_vect) {
	if (tx_lectura == tx_escritura) {
		UCSR0B &= (uint8_t) ~_BV(UDRIE0);
		} else {
		UDR0 = tx_datos[tx_lectura];
		tx_lectura = (uint8_t)((tx_lectura + 1U) & (TX_TAMANO - 1U));
	}
}