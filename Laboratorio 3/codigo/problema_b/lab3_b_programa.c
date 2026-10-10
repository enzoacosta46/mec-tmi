//============================================================
// Laboratorio 3 - Tecnologías de Microprocesamiento
// Grupo 7
// Problema B - Control de iluminación
//============================================================

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>


//------------------------------------------------------------
// Definiciones y constantes
//------------------------------------------------------------

#define LDR_CANAL        0
#define POSICION_CANAL   1

#define MOTOR_DDR        DDRB
#define MOTOR_PORT       PORTB
#define MOTOR_IN1        PB0
#define MOTOR_IN2        PB1

#define PWM_DDR          DDRD
#define PWM_PIN          PD3

#define SETPOINT_INICIAL 512

#define ERROR_PARADA     15
#define ERROR_ARRANQUE   30
#define PWM_MINIMO       80
#define PWM_MAXIMO       255

#define POSICION_MIN     20
#define POSICION_MAX     1000

#define MUESTRAS_ADC     8


typedef enum
{
    MOTOR_DETENIDO = 0,
    MOTOR_HORARIO,
    MOTOR_ANTIHORARIO,
    MOTOR_LIMITE_MAX,
    MOTOR_LIMITE_MIN
} estado_motor_t;


//------------------------------------------------------------
// Variables globales
//------------------------------------------------------------

volatile uint16_t contador_control_ms = 0;
volatile uint16_t contador_telemetria_ms = 0;
volatile uint8_t control_pendiente = 0;
volatile uint8_t telemetria_pendiente = 0;

uint16_t setpoint = SETPOINT_INICIAL;
uint16_t ldr_actual = 0;
uint16_t posicion_actual = 0;
uint8_t pwm_actual = 0;
uint32_t tiempo_ms = 0;
estado_motor_t estado_motor = MOTOR_DETENIDO;

uint16_t valor_uart = 0;
uint8_t digitos_uart = 0;


//------------------------------------------------------------
// Prototipos de funciones
//------------------------------------------------------------

void configurar_gpio(void);
void configurar_usart(void);
void configurar_adc(void);
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

uint16_t adc_leer_muestra(uint8_t canal);
uint16_t adc_leer_promedio(uint8_t canal);

void actualizar_control(void);
uint8_t calcular_pwm(uint16_t error_abs);
void motor_horario(uint8_t pwm);
void motor_antihorario(uint8_t pwm);
void motor_detener(void);
const char *nombre_estado_motor(estado_motor_t estado);

void enviar_telemetria(void);


//------------------------------------------------------------
// Programa principal
//------------------------------------------------------------

int main(void)
{
    // Configuración inicial de periféricos
    configurar_gpio();
    configurar_usart();
    configurar_adc();
    configurar_timer0();
    configurar_pwm();

    actualizar_control();
    mostrar_menu();

    sei();

    while (1)
    {
        revisar_uart();

        if (control_pendiente)
        {
            control_pendiente = 0;
            actualizar_control();
        }

        if (telemetria_pendiente)
        {
            telemetria_pendiente = 0;
            tiempo_ms += 500;
            enviar_telemetria();
        }
    }
}


//------------------------------------------------------------
// CONFIGURACIÓN GPIO
//------------------------------------------------------------

void configurar_gpio(void)
{
    // PB0 y PB1 controlan el sentido del puente H
    MOTOR_DDR |= (1 << MOTOR_IN1) | (1 << MOTOR_IN2);
    MOTOR_PORT &= ~((1 << MOTOR_IN1) | (1 << MOTOR_IN2));

    // PD3 / OC2B entrega el PWM al enable del puente H
    PWM_DDR |= (1 << PWM_PIN);

    // ADC0 y ADC1 como entradas sin pull-up
    DDRC &= ~((1 << PC0) | (1 << PC1));
    PORTC &= ~((1 << PC0) | (1 << PC1));
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
// CONFIGURACIÓN ADC
//------------------------------------------------------------

void configurar_adc(void)
{
    // Referencia AVCC, resultado de 10 bits alineado a la derecha
    ADMUX = (1 << REFS0);

    // ADC habilitado, prescaler 128: 16 MHz / 128 = 125 kHz
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    DIDR0 = (1 << ADC0D) | (1 << ADC1D);
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
// CONFIGURACIÓN PWM DEL MOTOR
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

void uart_enviar_caracter(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
    {
    }

    UDR0 = c;
}

void uart_enviar_cadena(const char *texto)
{
    while (*texto)
        uart_enviar_caracter(*texto++);
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

// Recibe un setpoint decimal entre 0 y 1023 terminado con Enter.
// También acepta C para mostrar la configuración y M para el menú.
void revisar_uart(void)
{
    char dato;

    if (!(UCSR0A & (1 << RXC0)))
        return;

    dato = UDR0;

    if (dato >= '0' && dato <= '9')
    {
        if (digitos_uart < 4)
        {
            valor_uart = (uint16_t)(valor_uart * 10 + (dato - '0'));
            digitos_uart++;
            uart_enviar_caracter(dato);
        }
        return;
    }

    if (dato == '\r' || dato == '\n')
    {
        if (digitos_uart == 0)
            return;

        uart_enviar_cadena("\r\n");

        if (valor_uart <= 1023)
        {
            setpoint = valor_uart;
            uart_enviar_cadena("Setpoint actualizado: ");
            uart_enviar_uint(setpoint);
            uart_enviar_cadena("\r\n");
            actualizar_control();
        }
        else
        {
            uart_enviar_cadena("Setpoint invalido. Rango permitido: 0-1023.\r\n");
        }

        valor_uart = 0;
        digitos_uart = 0;
        return;
    }

    if (digitos_uart == 0 && (dato == 'c' || dato == 'C'))
    {
        uart_enviar_caracter(dato);
        uart_enviar_cadena("\r\n");
        mostrar_configuracion();
        return;
    }

    if (digitos_uart == 0 && (dato == 'm' || dato == 'M'))
    {
        uart_enviar_caracter(dato);
        uart_enviar_cadena("\r\n");
        mostrar_menu();
        return;
    }

    valor_uart = 0;
    digitos_uart = 0;
    uart_enviar_cadena("\r\nEntrada invalida. Ingrese un valor entre 0 y 1023.\r\n");
}

void mostrar_menu(void)
{
    uart_enviar_cadena("\r\nPROBLEMA B - CONTROL DE ILUMINACION\r\n");
    uart_enviar_cadena("Ingrese setpoint 0-1023 y presione Enter\r\n");
    uart_enviar_cadena("C  Mostrar configuracion\r\n");
    uart_enviar_cadena("M  Mostrar menu\r\n\r\n");
    uart_enviar_cadena("CSV: tiempo_ms,setpoint,ldr,posicion,pwm,sentido\r\n");
    mostrar_configuracion();
}

void mostrar_configuracion(void)
{
    uart_enviar_cadena("Setpoint: ");
    uart_enviar_uint(setpoint);
    uart_enviar_cadena(" | Histeresis: ");
    uart_enviar_uint(ERROR_PARADA);
    uart_enviar_caracter('/');
    uart_enviar_uint(ERROR_ARRANQUE);
    uart_enviar_cadena(" | Posicion: ");
    uart_enviar_uint(POSICION_MIN);
    uart_enviar_caracter('-');
    uart_enviar_uint(POSICION_MAX);
    uart_enviar_cadena("\r\n");
}


//------------------------------------------------------------
// ADC
//------------------------------------------------------------

// Realiza una conversión ADC simple.
// Entrada: canal ADC. Salida: valor de 10 bits entre 0 y 1023.
uint16_t adc_leer_muestra(uint8_t canal)
{
    uint16_t valor;

    ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
    }

    valor = ADCL;
    valor |= ((uint16_t)ADCH << 8);

    return valor;
}

// Promedia varias conversiones para reducir variaciones de lectura.
// Entrada: canal ADC. Salida: promedio de MUESTRAS_ADC lecturas.
uint16_t adc_leer_promedio(uint8_t canal)
{
    uint32_t suma = 0;
    uint8_t i;

    // Primera conversión descartada después de seleccionar el canal
    adc_leer_muestra(canal);

    for (i = 0; i < MUESTRAS_ADC; i++)
        suma += adc_leer_muestra(canal);

    return (uint16_t)(suma / MUESTRAS_ADC);
}


//------------------------------------------------------------
// CONTROL DE ILUMINACIÓN
//------------------------------------------------------------

// Lee iluminación y posición, calcula el error y actualiza el motor.
void actualizar_control(void)
{
    int16_t error;
    uint16_t error_abs;
    uint8_t pwm;

    ldr_actual = adc_leer_promedio(LDR_CANAL);
    posicion_actual = adc_leer_promedio(POSICION_CANAL);

    error = (int16_t)setpoint - (int16_t)ldr_actual;
    error_abs = (error < 0) ? (uint16_t)(-error) : (uint16_t)error;

    if (estado_motor == MOTOR_DETENIDO ||
        estado_motor == MOTOR_LIMITE_MAX ||
        estado_motor == MOTOR_LIMITE_MIN)
    {
        if (error_abs <= ERROR_ARRANQUE)
        {
            motor_detener();
            return;
        }
    }
    else if (error_abs <= ERROR_PARADA)
    {
        motor_detener();
        return;
    }

    pwm = calcular_pwm(error_abs);

    if (error > 0)
    {
        // Sentido horario: abre la persiana y aumenta la iluminación
        if (posicion_actual >= POSICION_MAX)
        {
            motor_detener();
            estado_motor = MOTOR_LIMITE_MAX;
        }
        else
        {
            motor_horario(pwm);
        }
    }
    else if (error < 0)
    {
        // Sentido antihorario: cierra la persiana y reduce la iluminación
        if (posicion_actual <= POSICION_MIN)
        {
            motor_detener();
            estado_motor = MOTOR_LIMITE_MIN;
        }
        else
        {
            motor_antihorario(pwm);
        }
    }
    else
    {
        motor_detener();
    }
}

// Calcula un PWM proporcional a la magnitud del error y lo limita a 255.
// Entrada: error absoluto. Salida: valor OCR2B entre PWM_MINIMO y 255.
uint8_t calcular_pwm(uint16_t error_abs)
{
    uint32_t pwm;

    if (error_abs <= ERROR_PARADA)
        return 0;

    if (error_abs >= 1023)
        return PWM_MAXIMO;

    pwm = PWM_MINIMO;
    pwm += ((uint32_t)(error_abs - ERROR_PARADA) *
            (PWM_MAXIMO - PWM_MINIMO)) /
           (1023 - ERROR_PARADA);

    if (pwm > PWM_MAXIMO)
        pwm = PWM_MAXIMO;

    return (uint8_t)pwm;
}

void motor_horario(uint8_t pwm)
{
    MOTOR_PORT |= (1 << MOTOR_IN1);
    MOTOR_PORT &= ~(1 << MOTOR_IN2);

    OCR2B = pwm;
    pwm_actual = pwm;
    estado_motor = MOTOR_HORARIO;
}

void motor_antihorario(uint8_t pwm)
{
    MOTOR_PORT &= ~(1 << MOTOR_IN1);
    MOTOR_PORT |= (1 << MOTOR_IN2);

    OCR2B = pwm;
    pwm_actual = pwm;
    estado_motor = MOTOR_ANTIHORARIO;
}

void motor_detener(void)
{
    MOTOR_PORT &= ~((1 << MOTOR_IN1) | (1 << MOTOR_IN2));

    OCR2B = 0;
    pwm_actual = 0;
    estado_motor = MOTOR_DETENIDO;
}

const char *nombre_estado_motor(estado_motor_t estado)
{
    switch (estado)
    {
        case MOTOR_HORARIO:
            return "HORARIO";

        case MOTOR_ANTIHORARIO:
            return "ANTIHORARIO";

        case MOTOR_LIMITE_MAX:
            return "LIMITE_MAX";

        case MOTOR_LIMITE_MIN:
            return "LIMITE_MIN";

        case MOTOR_DETENIDO:
        default:
            return "DETENIDO";
    }
}


//------------------------------------------------------------
// TELEMETRÍA
//------------------------------------------------------------

// Envía setpoint, iluminación, posición, PWM y sentido de giro.
void enviar_telemetria(void)
{
    uart_enviar_cadena("DATA,");
    uart_enviar_ulong(tiempo_ms);
    uart_enviar_caracter(',');
    uart_enviar_uint(setpoint);
    uart_enviar_caracter(',');
    uart_enviar_uint(ldr_actual);
    uart_enviar_caracter(',');
    uart_enviar_uint(posicion_actual);
    uart_enviar_caracter(',');
    uart_enviar_uint(pwm_actual);
    uart_enviar_caracter(',');
    uart_enviar_cadena(nombre_estado_motor(estado_motor));
    uart_enviar_cadena("\r\n");
}


//------------------------------------------------------------
// INTERRUPCIONES
//------------------------------------------------------------

// Base temporal de 1 ms. Control cada 100 ms y telemetría cada 500 ms.
ISR(TIMER0_COMPA_vect)
{
    contador_control_ms++;
    contador_telemetria_ms++;

    if (contador_control_ms >= 100)
    {
        contador_control_ms = 0;
        control_pendiente = 1;
    }

    if (contador_telemetria_ms >= 500)
    {
        contador_telemetria_ms = 0;
        telemetria_pendiente = 1;
    }
}
