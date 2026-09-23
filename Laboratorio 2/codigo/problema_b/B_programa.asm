;============================================================
; Laboratorio 2 - Tecnologías de Microprocesamiento
; Grupo 7
; Problema B - DAC R-2R con LUT
;============================================================

.include "m328pdef.inc"


;------------------------------------------------------------
; Definición de registros
;------------------------------------------------------------
.def temp    = r16
.def temp2   = r17
.def dato    = r18
.def muestra = r19
.def indice  = r20
.def limite  = r21


;------------------------------------------------------------
; Vectores de interrupción
;------------------------------------------------------------
.cseg

.org 0x0000
    rjmp inicio

.org OC1Aaddr
    rjmp isr_timer1


;------------------------------------------------------------
; Inicialización
;------------------------------------------------------------
inicio:

    ; Inicialización del Stack Pointer
    ldi temp, HIGH(RAMEND)
    out SPH, temp

    ldi temp, LOW(RAMEND)
    out SPL, temp

    ; Configuración de periféricos
    call configurar_gpio
    call configurar_usart
    call configurar_timer1

    ; Habilitar interrupciones globales
    sei


;------------------------------------------------------------
; Programa principal
;------------------------------------------------------------
main:

    rjmp main


;------------------------------------------------------------
; CONFIGURACIÓN GPIO
;------------------------------------------------------------
configurar_gpio:

    ; PD2-PD7 como salidas para DAC bits 0-5
    ; PD0 y PD1 reservados para RX/TX
    ldi temp, 0b11111100
    out DDRD, temp

    ; PB0-PB1 como salidas para DAC bits 6-7
    ldi temp, 0b00000011
    out DDRB, temp

    ; DAC inicialmente en cero
    clr temp
    out PORTD, temp
    out PORTB, temp

    ret


;------------------------------------------------------------
; CONFIGURACIÓN USART
;------------------------------------------------------------
configurar_usart:

    ; Calculado con F=16MHz y B_rate=9600
	; UBRR0 = 103 = 0x0067	
    ldi temp, 0x00
    sts UBRR0H, temp

    ldi temp, 0x67
    sts UBRR0L, temp
	
	; Modo 0 - Asíncrono normal
    clr temp
    sts UCSR0A, temp

    ; Habilitar RX y TX
    ldi temp, (1 << RXEN0) | (1 << TXEN0)
    sts UCSR0B, temp
    
	; Sin paridad, 1 bit stop, 8 bits de datos
    ldi temp, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, temp

    ret


;------------------------------------------------------------
; CONFIGURACIÓN TIMER1
;------------------------------------------------------------
configurar_timer1:

    ; Operación normal de los pines OC1A/OC1B
    ; WGM11:WGM10 = 00
    clr temp
    sts TCCR1A, temp

    ; Modo CTC: WGM12 = 1
    ; Timer inicialmente detenido: CS12:CS10 = 000
    ldi temp, (1 << WGM12)
    sts TCCR1B, temp

    ; Contador inicialmente en cero
    clr temp
    sts TCNT1H, temp
    sts TCNT1L, temp

    ; OCR1A = 0x1869 calculado para 2560Hz
    ldi temp, 0x18
    sts OCR1AH, temp

    ldi temp, 0x69
    sts OCR1AL, temp

    ; Habilitar interrupción por Compare Match A
    ldi temp, (1 << OCIE1A)
    sts TIMSK1, temp

    ret


;------------------------------------------------------------
; ISR TIMER1
;------------------------------------------------------------
isr_timer1:

    reti