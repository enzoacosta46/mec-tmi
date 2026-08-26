.include "m328pdef.inc"

.def temp = r16
.def contador = r20

.cseg

.org 0x0000
    rjmp init

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