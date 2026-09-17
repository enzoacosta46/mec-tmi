;============================================================
; Laboratorio 2 - Tecnologías de Microprocesamiento
; Grupo 7
; Problema D - Parte Emisor
;============================================================

.include "m328pdef.inc"

;------------------------------------------------------------
; Definición de registros
;------------------------------------------------------------
.def temp = r16
.def dato = r17

;------------------------------------------------------------
; Vector de RESET
;------------------------------------------------------------
.cseg
.org 0x0000
    rjmp inicio


;------------------------------------------------------------
; Inicialización
;------------------------------------------------------------
inicio:

    ; Inicialización del Stack Pointer
    ldi r16, HIGH(RAMEND)
    out SPH, r16

    ldi r16, LOW(RAMEND)
    out SPL, r16

	; Configurar pines
    call configurar_gpio
	call configurar_usart


;------------------------------------------------------------
; Programa principal
;------------------------------------------------------------
main:
    ; Leer estado completo del Puerto B
    in dato, PINB

    ; Invertir los pulsadores que son pull-up
    com dato

    ; Conservar únicamente PB2 a PB0
    andi dato, 0b00000111

    ; Transmitir valor 0-7
    call enviar_dato

    rjmp main


;------------------------------------------------------------
; Configuración GPIO
;------------------------------------------------------------
configurar_gpio:

    ; Puerto B como entrada
    ldi temp, 0x00
    out DDRB, temp

    ; Activar pull-up en PB0, PB1 y PB2
	; Son los pulsadores de nuestro programa
    ldi temp, 0b00000111
    out PORTB, temp

	; PD1 / TXD como salida
    sbi DDRD, DDD1

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

	; Habilitar TX, deshabilitar RX
	ldi temp, (1 << TXEN0)
    sts UCSR0B, temp

	; Sin paridad, 1 bit stop, 8 bits de datos
	ldi temp, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, temp

	ret

	
;------------------------------------------------------------
; FUNCIÓN: TRANSMITIR DATO POR USART
;------------------------------------------------------------
; Entrada:
;   dato (r17) = byte a transmitir
;
; Salida:
;   dato escrito en UDR0
;------------------------------------------------------------
enviar_dato:

esperar_tx:

    ; Leer estado de USART
    lds temp, UCSR0A

    ; UDRE0 = 1 cuando UDR0 está disponible
    sbrs temp, UDRE0

    ; Si todavía no está disponible, esperar
    rjmp esperar_tx

    ; Escribir dato para comenzar transmisión
    sts UDR0, dato

    ret