.include "m328pdef.inc"

.def temp = r16
.def contador = r20

.cseg

.org 0x0000
    rjmp init

.org 0x0002
    rjmp isr_inc

.org 0x0004
    rjmp isr_dec

.org 0x000A
    rjmp isr_reset

.org 0x0034

init:
	cli

    ; Stack Pointer
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ; PB0-PB5 salidas: segmentos A-F
    ldi temp, 0b00111111
    out DDRB, temp

    ; PD7 salida (G), PD2-PD4 entradas
    ldi temp, (1 << DDD7)
    out DDRD, temp

    ; Pull-up en los tres pulsadores
    ldi temp, (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4)
    out PORTD, temp

    ; Display inicialmente apagado
    clr temp
    out PORTB, temp

    ; Contador inicia en 0
    clr contador

	; INT0 e INT1 por flanco de bajada
    ldi temp, (1 << ISC01) | (1 << ISC11)
    sts EICRA, temp

    ; Habilitar INT0 e INT1
    ldi temp, (1 << INT0) | (1 << INT1)
    out EIMSK, temp

    ; Habilitar PCINT en PORTD
    ldi temp, (1 << PCIE2)
    sts PCICR, temp

    ; PD4 = PCINT20
    ldi temp, (1 << PCINT20)
    sts PCMSK2, temp

	sei

	
;;;;;;;;;;;;;;;;;;;;;
main:
;;;;;;;;;;;;;;;;;;;;;
    rjmp main

; En las interrupciones se debe
; persistir el registro de estado
; en el stack.

; Incrementar
isr_inc:
    push temp
    in temp, SREG
    push temp

    cpi contador, 9
    breq inc_fin
    inc contador

inc_fin:
    pop temp
    out SREG, temp
    pop temp
    reti


; Decrementar
isr_dec:
    push temp
    in temp, SREG
    push temp

    tst contador
    breq dec_fin
    dec contador

dec_fin:
    pop temp
    out SREG, temp
    pop temp
    reti


; Reset
isr_reset:
    push temp
    in temp, SREG
    push temp

    ; PCINT también ocurre al soltar el botón
    sbic PIND, PIND4
    rjmp reset_fin

    clr contador

reset_fin:
    pop temp
    out SREG, temp
    pop temp
    reti