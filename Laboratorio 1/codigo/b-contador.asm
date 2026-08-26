.include "m328pdef.inc"

.def temp = r16
.def debounce = r18
.def boton_activo = r19
.def contador = r20
.def patron = r21

.cseg

.org 0x0000
    rjmp init

.org 0x0002
    rjmp isr_inc

.org 0x0004
    rjmp isr_dec

.org 0x000A
    rjmp isr_reset

.org 0x001C
    rjmp isr_timer0

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
	clr debounce
	clr boton_activo

    ; Guardar LUT en SRAM
    rcall guardar_codigos

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

	; Timer0 en CTC
    ldi temp, (1 << WGM01)
    out TCCR0A, temp

    ; Interrupción cada 1 ms
    ldi temp, 249
    out OCR0A, temp

    ; Prescaler 64
    ldi temp, (1 << CS01) | (1 << CS00)
    out TCCR0B, temp

    ; Habilitar interrupción Timer0
    ldi temp, (1 << OCIE0A)
    sts TIMSK0, temp

	sei

	
;;;;;;;;;;;;;;;;;;;;;
main:
;;;;;;;;;;;;;;;;;;;;;
    rcall mostrar_display
    rjmp main

; En las interrupciones se debe
; persistir el registro de estado
; en el stack.

; Incrementar
isr_inc:
    push temp
    in temp, SREG
    push temp

    ; Ignorar si hay un botón todavía activo
    tst boton_activo
    brne inc_fin

    ldi boton_activo, 1
    ldi debounce, 20

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

    tst boton_activo
    brne dec_fin

    ldi boton_activo, 2
    ldi debounce, 20

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

    ; PCINT también ocurre al liberar
    sbic PIND, PIND4
    rjmp reset_fin

    tst boton_activo
    brne reset_fin

    ldi boton_activo, 3
    ldi debounce, 20

    clr contador

reset_fin:
    pop temp
    out SREG, temp
    pop temp
    reti


; Timer0 - antirrebote
isr_timer0:
    push temp
    in temp, SREG
    push temp

    tst boton_activo
    breq timer_fin

    cpi boton_activo, 1
    breq revisar_inc

    cpi boton_activo, 2
    breq revisar_dec

    ; Botón Reset
    sbic PIND, PIND4
    rjmp liberado
    rjmp presionado


revisar_inc:
    sbic PIND, PIND2
    rjmp liberado
    rjmp presionado


revisar_dec:
    sbic PIND, PIND3
    rjmp liberado
    rjmp presionado


; Mientras continúe presionado,
; mantener el tiempo de debounce
presionado:
    ldi debounce, 20
    rjmp timer_fin


; Al liberarse debe permanecer
; estable durante 20 ms
liberado:
    dec debounce
    brne timer_fin

    clr boton_activo


timer_fin:
    pop temp
    out SREG, temp
    pop temp
    reti


; Obtener patrón desde la LUT
get_7seg_code:
    ldi r28, 0x00
    ldi r29, 0x01

    add r28, contador
    ld patron, Y

    ret


; Mostrar contador en display
mostrar_display:
    rcall get_7seg_code

    ; Segmentos A-F
    mov temp, patron
    andi temp, 0b00111111
    out PORTB, temp

    ; Segmento G
    cbi PORTD, PORTD7
    sbrc patron, 6
    sbi PORTD, PORTD7

    ret


; Guardar LUT desde 0x0100
guardar_codigos:
    ldi r28, 0x00
    ldi r29, 0x01

    ldi temp, 0x3F       ; 0
    st Y+, temp
    ldi temp, 0x06       ; 1
    st Y+, temp
    ldi temp, 0x5B       ; 2
    st Y+, temp
    ldi temp, 0x4F       ; 3
    st Y+, temp
    ldi temp, 0x66       ; 4
    st Y+, temp
    ldi temp, 0x6D       ; 5
    st Y+, temp
    ldi temp, 0x7D       ; 6
    st Y+, temp
    ldi temp, 0x07       ; 7
    st Y+, temp
    ldi temp, 0x7F       ; 8
    st Y+, temp
    ldi temp, 0x6F       ; 9
    st Y+, temp

    ret