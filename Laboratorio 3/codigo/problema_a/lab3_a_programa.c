//============================================================
// Laboratorio 3 - Tecnologías de Microprocesamiento
// Grupo 7
// Problema A - Control de temperatura
//============================================================

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>


//------------------------------------------------------------
// Definiciones y constantes
//------------------------------------------------------------

#define DHT_DDR     DDRD
#define DHT_PORT    PORTD
#define DHT_PINREG  PIND
#define DHT_PIN     PD2

#define HEATER_DDR  DDRB
#define HEATER_PORT PORTB
#define HEATER_PIN  PB0

#define FAN_DDR     DDRD
#define FAN_PIN     PD3

#define LCD_DDR     DDRC
#define LCD_PORT    PORTC
#define LCD_RS      PC0
#define LCD_E       PC1
#define LCD_D4      PC2
#define LCD_D5      PC3
#define LCD_D6      PC4
#define LCD_D7      PC5

#define PUNTO_MEDIO_INICIAL 20
#define PUNTO_MEDIO_MIN     5
#define PUNTO_MEDIO_MAX     24

#define PWM_BAJO  85
#define PWM_MEDIO 170
#define PWM_ALTO  255


typedef enum
{
	ESTADO_CALENTANDO = 0,
	ESTADO_ESTABLE,
	ESTADO_VENT_BAJO,
	ESTADO_VENT_MEDIO,
	ESTADO_VENT_ALTO
} estado_termico_t;


//------------------------------------------------------------
// Variables globales
//------------------------------------------------------------

volatile uint16_t contador_ms = 0;
volatile uint8_t lectura_pendiente = 0;

uint8_t punto_medio = PUNTO_MEDIO_INICIAL;
uint8_t ultima_temperatura = 0;
uint8_t lectura_valida = 0;
uint32_t tiempo_segundos = 0;
estado_termico_t estado_actual = ESTADO_ESTABLE;


//------------------------------------------------------------
// Prototipos de funciones
//------------------------------------------------------------

void configurar_gpio(void);
void configurar_usart(void);
void configurar_timer0(void);
void configurar_pwm(void);
void numero_a_cadena(uint32_t valor, char *buffer);

void uart_enviar_caracter(char c);
void uart_enviar_cadena(const char *texto);
void uart_enviar_uint(uint16_t valor);
void uart_enviar_ulong(uint32_t valor);
void revisar_uart(void);
void mostrar_menu(void);
void mostrar_configuracion(void);

void lcd_inicializar(void);
void lcd_comando(uint8_t comando);
void lcd_dato(uint8_t dato);
void lcd_enviar_nibble(uint8_t nibble);
void lcd_posicion(uint8_t fila, uint8_t columna);
void lcd_enviar_cadena(const char *texto);
void lcd_enviar_uint(uint8_t valor);
void lcd_limpiar_linea(uint8_t fila);
void lcd_actualizar(void);

uint8_t dht11_leer(uint8_t *temperatura, uint8_t *humedad);
uint8_t esperar_nivel_dht(uint8_t nivel, uint8_t timeout_us);

void actualizar_control(uint8_t temperatura);
void aplicar_salidas(estado_termico_t estado);
void enviar_telemetria(uint8_t temperatura);
void recalcular_control(void);

uint8_t limite_calefaccion(void);
uint8_t limite_ideal_superior(void);
uint8_t limite_bajo_superior(void);
uint8_t limite_medio_superior(void);
const char *nombre_estado(estado_termico_t estado);


//------------------------------------------------------------
// Programa principal
//------------------------------------------------------------

int main(void)
{
	// Configuración inicial de periféricos
	configurar_gpio();
	configurar_usart();
	configurar_timer0();
	configurar_pwm();
	lcd_inicializar();

	mostrar_menu();
	lcd_actualizar();

	sei();

	while (1)
	{
		revisar_uart();

		if (lectura_pendiente)
		{
			uint8_t temperatura;
			uint8_t humedad;

			cli();
			lectura_pendiente = 0;
			sei();

			tiempo_segundos += 5;

			if (dht11_leer(&temperatura, &humedad))
			{
				ultima_temperatura = temperatura;
				lectura_valida = 1;

				actualizar_control(temperatura);
				lcd_actualizar();
				enviar_telemetria(temperatura);
			}
			else
			{
				lectura_valida = 0;
				aplicar_salidas(ESTADO_ESTABLE);
				lcd_actualizar();
				uart_enviar_cadena("ERROR,DHT11\r\n");
			}
		}
	}
}


//------------------------------------------------------------
// CONFIGURACIÓN GPIO
//------------------------------------------------------------

void configurar_gpio(void)
{
	// DHT11 como entrada con pull-up
	DHT_DDR &= ~(1 << DHT_PIN);
	DHT_PORT |= (1 << DHT_PIN);

	// Calefactor y ventilador como salidas
	HEATER_DDR |= (1 << HEATER_PIN);
	HEATER_PORT &= ~(1 << HEATER_PIN);
	FAN_DDR |= (1 << FAN_PIN);

	// LCD en modo de 4 bits
	LCD_DDR |= (1 << LCD_RS) | (1 << LCD_E) |
	(1 << LCD_D4) | (1 << LCD_D5) |
	(1 << LCD_D6) | (1 << LCD_D7);
}


//------------------------------------------------------------
// CONFIGURACIÓN USART
//------------------------------------------------------------

void configurar_usart(void)
{
	// 9600 baud, 8 bits, sin paridad, 1 bit de parada
	UBRR0H = 0;
	UBRR0L = 103;

	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}


//------------------------------------------------------------
// CONFIGURACIÓN TIMER0
//------------------------------------------------------------

void configurar_timer0(void)
{
	// CTC, prescaler 64, interrupción cada 1 ms
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 249;
	TIMSK0 = (1 << OCIE0A);
}


//------------------------------------------------------------
// CONFIGURACIÓN PWM DEL VENTILADOR
//------------------------------------------------------------

void configurar_pwm(void)
{
	// Timer2 Fast PWM sobre OC2B (PD3), prescaler 64
	TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (1 << CS22);
	OCR2B = 0;
}


//------------------------------------------------------------
// FUNCIONES AUXILIARES
//------------------------------------------------------------

// Convierte un entero sin signo a una cadena decimal.
// Entrada: valor a convertir. Salida: texto terminado en '\0'.
void numero_a_cadena(uint32_t valor, char *buffer)
{
	char invertido[10];
	uint8_t cantidad = 0;
	uint8_t i = 0;

	if (valor == 0)
	{
		buffer[0] = '0';
		buffer[1] = '\0';
		return;
	}

	while (valor > 0)
	{
		invertido[cantidad++] = (char)('0' + (valor % 10));
		valor /= 10;
	}

	while (cantidad > 0)
	buffer[i++] = invertido[--cantidad];

	buffer[i] = '\0';
}


//------------------------------------------------------------
// USART
//------------------------------------------------------------

// Envía un carácter por USART.
// Entrada: c = carácter a transmitir.
void uart_enviar_caracter(char c)
{
	while (!(UCSR0A & (1 << UDRE0)))
	{
	}

	UDR0 = c;
}

// Envía una cadena terminada en '\0'.
// Entrada: texto = puntero a la cadena.
void uart_enviar_cadena(const char *texto)
{
	while (*texto)
	{
		uart_enviar_caracter(*texto++);
	}
}

void uart_enviar_uint(uint16_t valor)
{
	char buffer[6];
	numero_a_cadena(valor, buffer);
	uart_enviar_cadena(buffer);
}

void uart_enviar_ulong(uint32_t valor)
{
	char buffer[11];
	numero_a_cadena(valor, buffer);
	uart_enviar_cadena(buffer);
}

// Revisa la recepción USART sin bloquear el programa y procesa el menú.
void revisar_uart(void)
{
	if (!(UCSR0A & (1 << RXC0)))
	return;

	char dato = UDR0;

	if (dato == '+')
	{
		if (punto_medio < PUNTO_MEDIO_MAX)
		{
			punto_medio++;
			recalcular_control();
		}
		else
		{
			uart_enviar_cadena("No se puede aumentar: limite de seguridad del DHT11.\r\n");
		}
	}
	else if (dato == '-')
	{
		if (punto_medio > PUNTO_MEDIO_MIN)
		{
			punto_medio--;
			recalcular_control();
		}
		else
		{
			uart_enviar_cadena("No se puede disminuir: limite de seguridad del DHT11.\r\n");
		}
	}
	else if (dato == 'm' || dato == 'M')
	{
		mostrar_menu();
	}
	else if (dato == 'c' || dato == 'C')
	{
		mostrar_configuracion();
	}
}

void mostrar_menu(void)
{
	uart_enviar_cadena("\r\nPROBLEMA A - CONTROL DE TEMPERATURA\r\n");
	uart_enviar_cadena("+  Aumentar punto medio 1 C\r\n");
	uart_enviar_cadena("-  Disminuir punto medio 1 C\r\n");
	uart_enviar_cadena("C  Mostrar configuracion\r\n");
	uart_enviar_cadena("M  Mostrar menu\r\n\r\n");
	uart_enviar_cadena("CSV: tiempo_s,temp_C,calefactor,pwm,ideal_min,ideal_max,estado\r\n");
	mostrar_configuracion();
}

void mostrar_configuracion(void)
{
	uart_enviar_cadena("Punto medio: ");
	uart_enviar_uint(punto_medio);
	uart_enviar_cadena(" C | Rango ideal: ");
	uart_enviar_uint(limite_calefaccion() + 1);
	uart_enviar_caracter('-');
	uart_enviar_uint(limite_ideal_superior());
	uart_enviar_cadena(" C\r\n");
}


//------------------------------------------------------------
// LCD 16x2
//------------------------------------------------------------

void lcd_inicializar(void)
{
	_delay_ms(40);

	LCD_PORT &= ~((1 << LCD_RS) | (1 << LCD_E));

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
	_delay_ms(2);
}

// Envía 4 bits al LCD por PC2-PC5.
// Entrada: nibble = valor de 4 bits a transmitir.
void lcd_enviar_nibble(uint8_t nibble)
{
	LCD_PORT = (LCD_PORT & 0xC3) | ((nibble & 0x0F) << 2);

	LCD_PORT |= (1 << LCD_E);
	_delay_us(1);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_us(50);
}

void lcd_comando(uint8_t comando)
{
	LCD_PORT &= ~(1 << LCD_RS);
	lcd_enviar_nibble(comando >> 4);
	lcd_enviar_nibble(comando & 0x0F);

	if (comando == 0x01 || comando == 0x02)
	_delay_ms(2);
}

void lcd_dato(uint8_t dato)
{
	LCD_PORT |= (1 << LCD_RS);
	lcd_enviar_nibble(dato >> 4);
	lcd_enviar_nibble(dato & 0x0F);
}

void lcd_posicion(uint8_t fila, uint8_t columna)
{
	uint8_t direccion = (fila == 0 ? 0x00 : 0x40) + columna;
	lcd_comando(0x80 | direccion);
}

void lcd_enviar_cadena(const char *texto)
{
	while (*texto)
	{
		lcd_dato(*texto++);
	}
}

void lcd_enviar_uint(uint8_t valor)
{
	char buffer[4];
	numero_a_cadena(valor, buffer);
	lcd_enviar_cadena(buffer);
}

void lcd_limpiar_linea(uint8_t fila)
{
	lcd_posicion(fila, 0);

	for (uint8_t i = 0; i < 16; i++)
	lcd_dato(' ');
}

// Actualiza temperatura y rango ideal mostrados en pantalla.
void lcd_actualizar(void)
{
	lcd_limpiar_linea(0);
	lcd_posicion(0, 0);

	if (lectura_valida)
	{
		lcd_enviar_cadena("Temp: ");
		lcd_enviar_uint(ultima_temperatura);
		lcd_enviar_cadena(" C");
	}
	else
	{
		lcd_enviar_cadena("Esperando DHT11");
	}

	lcd_limpiar_linea(1);
	lcd_posicion(1, 0);
	lcd_enviar_cadena("Ideal:");
	lcd_enviar_uint(limite_calefaccion() + 1);
	lcd_dato('-');
	lcd_enviar_uint(limite_ideal_superior());
	lcd_enviar_cadena(" C");
}


//------------------------------------------------------------
// DHT11
//------------------------------------------------------------

// Espera hasta que la línea del DHT cambie de nivel.
// Retorno: 1 si cambió a tiempo, 0 si venció el timeout.
uint8_t esperar_nivel_dht(uint8_t nivel, uint8_t timeout_us)
{
	while (((DHT_PINREG & (1 << DHT_PIN)) != 0) == nivel)
	{
		if (timeout_us == 0)
		return 0;

		_delay_us(1);
		timeout_us--;
	}

	return 1;
}

// Lee la trama de 40 bits del DHT11 y valida el checksum.
// Salidas: temperatura y humedad.
// Retorno: 1 si la lectura es válida, 0 si ocurre un error.
uint8_t dht11_leer(uint8_t *temperatura, uint8_t *humedad)
{
	uint8_t datos[5] = {0, 0, 0, 0, 0};
	uint8_t sreg;

	DHT_DDR |= (1 << DHT_PIN);
	DHT_PORT &= ~(1 << DHT_PIN);
	_delay_ms(18);

	sreg = SREG;
	cli();

	DHT_PORT |= (1 << DHT_PIN);
	_delay_us(30);

	DHT_DDR &= ~(1 << DHT_PIN);
	DHT_PORT |= (1 << DHT_PIN);

	if (!esperar_nivel_dht(1, 100) ||
	!esperar_nivel_dht(0, 100) ||
	!esperar_nivel_dht(1, 100))
	{
		SREG = sreg;
		return 0;
	}

	for (uint8_t i = 0; i < 40; i++)
	{
		if (!esperar_nivel_dht(0, 80))
		{
			SREG = sreg;
			return 0;
		}

		_delay_us(40);

		datos[i / 8] <<= 1;

		if (DHT_PINREG & (1 << DHT_PIN))
		datos[i / 8] |= 1;

		if (i < 39 && !esperar_nivel_dht(1, 100))
		{
			SREG = sreg;
			return 0;
		}
	}

	SREG = sreg;

	if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) != datos[4])
	return 0;

	*humedad = datos[0];
	*temperatura = datos[2];

	return 1;
}


//------------------------------------------------------------
// CONTROL DE TEMPERATURA
//------------------------------------------------------------

// Selecciona el estado térmico según la temperatura medida.
// Entrada: temperatura en grados Celsius.
void actualizar_control(uint8_t temperatura)
{
	if (temperatura <= limite_calefaccion())
	estado_actual = ESTADO_CALENTANDO;
	else if (temperatura <= limite_ideal_superior())
	estado_actual = ESTADO_ESTABLE;
	else if (temperatura <= limite_bajo_superior())
	estado_actual = ESTADO_VENT_BAJO;
	else if (temperatura <= limite_medio_superior())
	estado_actual = ESTADO_VENT_MEDIO;
	else
	estado_actual = ESTADO_VENT_ALTO;

	aplicar_salidas(estado_actual);
}

// Aplica calefactor y PWM del ventilador según el estado recibido.
void aplicar_salidas(estado_termico_t estado)
{
	HEATER_PORT &= ~(1 << HEATER_PIN);
	OCR2B = 0;

	switch (estado)
	{
		case ESTADO_CALENTANDO:
		HEATER_PORT |= (1 << HEATER_PIN);
		break;

		case ESTADO_VENT_BAJO:
		OCR2B = PWM_BAJO;
		break;

		case ESTADO_VENT_MEDIO:
		OCR2B = PWM_MEDIO;
		break;

		case ESTADO_VENT_ALTO:
		OCR2B = PWM_ALTO;
		break;

		case ESTADO_ESTABLE:
		default:
		break;
	}
}

// Reaplica el control después de modificar el punto medio.
void recalcular_control(void)
{
	if (lectura_valida)
	actualizar_control(ultima_temperatura);

	lcd_actualizar();
	mostrar_configuracion();
}

uint8_t limite_calefaccion(void)
{
	return punto_medio - 5;
}

uint8_t limite_ideal_superior(void)
{
	return punto_medio + 5;
}

uint8_t limite_bajo_superior(void)
{
	return punto_medio + 15;
}

uint8_t limite_medio_superior(void)
{
	return punto_medio + 25;
}

const char *nombre_estado(estado_termico_t estado)
{
	switch (estado)
	{
		case ESTADO_CALENTANDO:
		return "CALEFACTOR";
		case ESTADO_ESTABLE:
		return "ESTABLE";
		case ESTADO_VENT_BAJO:
		return "VENT_BAJO";
		case ESTADO_VENT_MEDIO:
		return "VENT_MEDIO";
		case ESTADO_VENT_ALTO:
		return "VENT_ALTO";
		default:
		return "DESCONOCIDO";
	}
}


//------------------------------------------------------------
// TELEMETRÍA
//------------------------------------------------------------

// Envía una muestra en formato CSV para registrar y graficar los datos.
void enviar_telemetria(uint8_t temperatura)
{
	uart_enviar_cadena("DATA,");
	uart_enviar_ulong(tiempo_segundos);
	uart_enviar_caracter(',');
	uart_enviar_uint(temperatura);
	uart_enviar_caracter(',');
	uart_enviar_uint(estado_actual == ESTADO_CALENTANDO ? 1 : 0);
	uart_enviar_caracter(',');
	uart_enviar_uint(OCR2B);
	uart_enviar_caracter(',');
	uart_enviar_uint(limite_calefaccion() + 1);
	uart_enviar_caracter(',');
	uart_enviar_uint(limite_ideal_superior());
	uart_enviar_caracter(',');
	uart_enviar_cadena(nombre_estado(estado_actual));
	uart_enviar_cadena("\r\n");
}


//------------------------------------------------------------
// INTERRUPCIONES
//------------------------------------------------------------

// Base temporal de 1 ms. Cada 5 s solicita una nueva lectura.
ISR(TIMER0_COMPA_vect)
{
	contador_ms++;

	if (contador_ms >= 5000)
	{
		contador_ms = 0;
		lectura_pendiente = 1;
	}
}
