.include "m328pdef.inc"

.def temp = r16
.def debounce = r18
.def boton_activo = r19

.def secuencia = r20
.def patron = r21
.def estado_seq = r22

.def anim_timer = r23
.def frame_ready = r24
.def reiniciar = r25


.cseg

.org 0x0000
    rjmp init

.org 0x0002
    rjmp isr_sig

.org 0x0004
    rjmp isr_ant

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


    ; PD5-PD7 salidas para LED1-LED3
    ; PD2-PD4 entradas para pulsadores
    ldi temp, (1 << DDD5) | (1 << DDD6) | (1 << DDD7)
    out DDRD, temp

    ; PB0-PB4 salidas para LED4-LED8
    ldi temp, 0b00011111
    out DDRB, temp

    ; Pull-up en los tres pulsadores
    ldi temp, (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4)
    out PORTD, temp

    ; LEDs inicialmente apagados
    clr temp
    out PORTB, temp


    ; Variables iniciales
    clr debounce
    clr boton_activo
    clr estado_seq
    clr frame_ready
    clr reiniciar

    ldi secuencia, 1


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


    ; Inicializar primera secuencia
    rcall inicializar_secuencia

    sei


;;;;;;;;;;;;;;;;;;;;;
main:
;;;;;;;;;;;;;;;;;;;;;

    ; Si cambió la secuencia, reiniciarla
    tst reiniciar
    breq revisar_frame

    rcall inicializar_secuencia


revisar_frame:

    ; Esperar al próximo paso de animación
    tst frame_ready
    breq main

    clr frame_ready

    rcall actualizar_secuencia
    rcall mostrar_patron

    rjmp main



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; INTERRUPCIONES DE PULSADORES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; Siguiente secuencia
isr_sig:
    push temp
    in temp, SREG
    push temp

    tst boton_activo
    brne sig_fin

    ldi boton_activo, 1
    ldi debounce, 20

    cpi secuencia, 8
    breq sig_inicio

    inc secuencia
    rjmp sig_cambio


sig_inicio:
    ldi secuencia, 1


sig_cambio:
    ldi reiniciar, 1


sig_fin:
    pop temp
    out SREG, temp
    pop temp
    reti



; Secuencia anterior
isr_ant:
    push temp
    in temp, SREG
    push temp

    tst boton_activo
    brne ant_fin

    ldi boton_activo, 2
    ldi debounce, 20

    cpi secuencia, 1
    breq ant_final

    dec secuencia
    rjmp ant_cambio


ant_final:
    ldi secuencia, 8


ant_cambio:
    ldi reiniciar, 1


ant_fin:
    pop temp
    out SREG, temp
    pop temp
    reti



; Reset a secuencia 1
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

    ldi secuencia, 1
    ldi reiniciar, 1


reset_fin:
    pop temp
    out SREG, temp
    pop temp
    reti



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; TIMER0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

isr_timer0:
    push temp
    in temp, SREG
    push temp


    ; Tiempo entre pasos de animación
    dec anim_timer
    brne timer_debounce

    ; 200 ms entre patrones
    ldi anim_timer, 200
    ldi frame_ready, 1


timer_debounce:

    ; Antirrebote de pulsadores
    tst boton_activo
    breq timer_fin

    cpi boton_activo, 1
    breq revisar_sig

    cpi boton_activo, 2
    breq revisar_ant


    ; Botón Reset
    sbic PIND, PIND4
    rjmp liberado
    rjmp presionado


revisar_sig:
    sbic PIND, PIND2
    rjmp liberado
    rjmp presionado


revisar_ant:
    sbic PIND, PIND3
    rjmp liberado
    rjmp presionado


presionado:
    ldi debounce, 20
    rjmp timer_fin


liberado:
    dec debounce
    brne timer_fin

    clr boton_activo


timer_fin:
    pop temp
    out SREG, temp
    pop temp
    reti



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; CONTROL DE SECUENCIAS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

inicializar_secuencia:

    clr estado_seq
    clr frame_ready
    clr reiniciar

    ldi anim_timer, 200


    cpi secuencia, 1
    breq init_seq1

    cpi secuencia, 2
    breq init_seq2

    cpi secuencia, 3
    breq init_seq3

    cpi secuencia, 4
    breq init_seq4

    cpi secuencia, 5
    breq init_seq5

    cpi secuencia, 6
    breq init_seq6

    cpi secuencia, 7
    breq init_seq7

    ; Secuencia 8
    rjmp init_seq8


init_seq1:
    ldi patron, 0b00000001
    rjmp init_fin


init_seq2:
    ldi patron, 0b10000000
    rjmp init_fin


init_seq3:
    ldi patron, 0b00000001
    rjmp init_fin


init_seq4:
    ldi patron, 0b00000001
    rjmp init_fin


init_seq5:
    ldi patron, 0b01010101
    rjmp init_fin


init_seq6:
    ldi patron, 0b10000001
    rjmp init_fin


init_seq7:
    ldi patron, 0b00001111
    rjmp init_fin


init_seq8:
    clr patron
    clr estado_seq


init_fin:
    rcall mostrar_patron
    ret



; Seleccionar rutina de la secuencia actual
actualizar_secuencia:

    cpi secuencia, 1
    breq actualizar_seq1

    cpi secuencia, 2
    breq actualizar_seq2

    cpi secuencia, 3
    breq actualizar_seq3

    cpi secuencia, 4
    breq actualizar_seq4

    cpi secuencia, 5
    breq actualizar_seq5

    cpi secuencia, 6
    breq actualizar_seq6

    cpi secuencia, 7
    breq actualizar_seq7

    cpi secuencia, 8
    breq actualizar_seq8

    ret


actualizar_seq1:
    rcall secuencia1
    ret

actualizar_seq2:
    rcall secuencia2
    ret

actualizar_seq3:
    rcall secuencia3
    ret

actualizar_seq4:
    rcall secuencia4
    ret

actualizar_seq5:
    rcall secuencia5
    ret

actualizar_seq6:
    rcall secuencia6
    ret

actualizar_seq7:
    rcall secuencia7
    ret

actualizar_seq8:
    rcall secuencia8
    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 1
; LED hacia la izquierda
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia1:

    lsl patron
    brne seq1_fin

    ldi patron, 0b00000001

seq1_fin:
    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 2
; LED hacia la derecha
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia2:

    lsr patron
    brne seq2_fin

    ldi patron, 0b10000000

seq2_fin:
    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 3
; Ida y vuelta
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia3:

    ; estado_seq = 0: hacia la izquierda
    ; estado_seq = 1: hacia la derecha

    tst estado_seq
    brne seq3_derecha


seq3_izquierda:

    cpi patron, 0b10000000
    brne seq3_lsl

    ldi estado_seq, 1
    lsr patron
    ret


seq3_lsl:
    lsl patron
    ret


seq3_derecha:

    cpi patron, 0b00000001
    brne seq3_lsr

    clr estado_seq
    lsl patron
    ret


seq3_lsr:
    lsr patron
    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 4
; Llenado y vaciado
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia4:

    ; estado_seq = 0: llenar
    ; estado_seq = 1: vaciar

    tst estado_seq
    brne seq4_vaciar


seq4_llenar:

    cpi patron, 0b11111111
    brne seq4_rol

    ldi estado_seq, 1

    clc
    ror patron
    ret


seq4_rol:
    sec
    rol patron
    ret


seq4_vaciar:

    cpi patron, 0b00000001
    brne seq4_ror

    clr estado_seq
    sec
    rol patron
    ret


seq4_ror:
    clc
    ror patron
    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 5
; LEDs alternados
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia5:

    ; 01010101 <-> 10101010
    ldi temp, 0b11111111
    eor patron, temp

    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 6
; Extremos hacia el centro y regreso
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia6:

    ; estado_seq = 0: hacia el centro
    ; estado_seq = 1: hacia los extremos

    tst estado_seq
    brne seq6_afuera


seq6_adentro:

    ; Al llegar al centro, cambiar dirección
    cpi patron, 0b00011000
    brne seq6_mover_adentro

    ldi estado_seq, 1
    rjmp seq6_mover_afuera


seq6_mover_adentro:

    ; Parte izquierda
    mov temp, patron
    andi temp, 0b11110000
    lsr temp

    ; Parte derecha
    andi patron, 0b00001111
    lsl patron

    or patron, temp

    ret


seq6_afuera:

    ; Al llegar a los extremos, cambiar dirección
    cpi patron, 0b10000001
    brne seq6_mover_afuera

    clr estado_seq
    rjmp seq6_mover_adentro


seq6_mover_afuera:

    ; Parte izquierda
    mov temp, patron
    andi temp, 0b11110000
    lsl temp

    ; Parte derecha
    andi patron, 0b00001111
    lsr patron

    or patron, temp

    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 7
; Alternar ambas mitades
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia7:

    ; 00001111 <-> 11110000
    swap patron

    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SECUENCIA 8
; Conteo binario reflejado
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

secuencia8:

    ; Contador de 4 bits
    inc estado_seq
    andi estado_seq, 0b00001111

    ; Copiar valor al patrón
    mov patron, estado_seq

    ; Repetir el nibble en la mitad superior
    mov temp, patron
    swap temp

    or patron, temp

    ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; SALIDA DE LOS 8 LEDS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

mostrar_patron:

    ; LED1-LED3 -> PD5-PD7
    mov temp, patron
    andi temp, 0b00000111

    lsl temp
    lsl temp
    lsl temp
    lsl temp
    lsl temp

    ; Mantener pull-up PD2-PD4
    ori temp, 0b00011100
    out PORTD, temp


    ; LED4-LED8 -> PB0-PB4
    mov temp, patron

    lsr temp
    lsr temp
    lsr temp

    andi temp, 0b00011111
    out PORTB, temp

    ret