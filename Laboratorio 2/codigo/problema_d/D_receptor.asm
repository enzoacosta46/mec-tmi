;============================================================
; Laboratorio 2 - Tecnologías de Microprocesamiento
; Grupo 7
; Problema D - Parte Receptor
;============================================================

.include "m328pdef.inc"

;------------------------------------------------------------
; Definición de registros
;------------------------------------------------------------
.def temp   = r16
.def dato   = r18
.def patron = r19

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
    ; Esperar y recibir dato
    call recibir_dato

    ; Comprobar que el dato esté entre 0 y 7
    cpi dato, 8
    brsh main

    ; Generar patrón correspondiente
    call generar_patron

    ; Actualizar LEDs
    call mostrar_patron

    rjmp main


;------------------------------------------------------------
; Configuración GPIO
;------------------------------------------------------------
configurar_gpio:

    ; PD4-PD7 como salidas
    ldi temp, 0b11110000
    out DDRD, temp

    ; PB0-PB3 como salidas
    ldi temp, 0b00001111
    out DDRB, temp

    ; Todos los LEDs inicialmente apagados
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

	; Habilitar RX, deshabilitar TX
	ldi temp, (1 << RXEN0)
    sts UCSR0B, temp

	; Sin paridad, 1 bit stop, 8 bits de datos
	ldi temp, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, temp

	ret

	
;------------------------------------------------------------
; FUNCIÓN: RECIBIR DATO POR USART
;------------------------------------------------------------
; Salida:
;   dato (r18) = byte recibido
;------------------------------------------------------------
recibir_dato:

esperar_rx:

    ; Consultar estado de USART
    lds temp, UCSR0A

    ; RXC0 = 1 cuando existe un dato recibido
    sbrs temp, RXC0
    rjmp esperar_rx

    ; Leer dato recibido
    lds dato, UDR0

    ret
	
;------------------------------------------------------------
; FUNCIÓN: GENERAR PATRÓN DE LED
;------------------------------------------------------------
; Entrada:
;   dato = valor entre 0 y 7
;
; Salida:
;   patron = 1 << dato
;------------------------------------------------------------
generar_patron:

    ; Comenzar con LED 0
    ldi patron, 0b00000001

    ; Copiar valor recibido como contador
    mov temp, dato

desplazar_patron:

    ; Si temp = 0, el patrón está listo
    tst temp
    breq patron_listo

    ; Desplazar bit activo una posición
    lsl patron

    dec temp
    rjmp desplazar_patron

patron_listo:

    ret
	
;------------------------------------------------------------
; MOSTRAR PATRÓN EN LOS 8 LEDS
;------------------------------------------------------------
mostrar_patron:

    ; LEDs 0-3 -> PD4-PD7
    mov temp, patron

    ; Conservar bits 0-3
    andi temp, 0b00001111

    ; Mover nibble bajo al nibble alto
    swap temp

    out PORTD, temp

    ; LEDs 4-7 -> PB0-PB3
    mov temp, patron

    ; Mover bits 4-7 hacia 0-3
    swap temp

    ; Conservar nibble bajo
    andi temp, 0b00001111

    out PORTB, temp

    ret