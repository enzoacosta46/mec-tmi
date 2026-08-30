.include "m328pdef.inc"

.def temp = r16
.def estado = r17
.def debounce = r18
.def boton_activo = r19
.def carga = r20
.def flags = r21

; Estados
.equ ST_LISTO           = 0
.equ ST_ESPERA_PUERTA   = 1
.equ ST_ESPERA_LLENADO  = 2
.equ ST_LAVADO_GIRO     = 3
.equ ST_LAVADO_PAUSA    = 4
.equ ST_CENTRIFUGADO    = 5
.equ ST_SECADO_DER      = 6
.equ ST_SECADO_PAUSA    = 7
.equ ST_SECADO_IZQ      = 8
.equ ST_FIN             = 9

; Cargas
.equ CARGA_LIGERA = 0
.equ CARGA_MEDIA  = 1
.equ CARGA_PESADA = 2

; Bits de flags
.equ FLAG_INICIO = 0


.cseg

.org 0x0000
    rjmp init

.org 0x0002
    rjmp isr_inicio

.org 0x0004
    rjmp isr_carga

.org 0x001C
    rjmp isr_timer0

.org 0x0034


;;;;;;;;;;;;;;;;;;;;;
init:
;;;;;;;;;;;;;;;;;;;;;
    cli

    ; Stack Pointer
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    ldi temp, LOW(RAMEND)
    out SPL, temp

    ; PORTD
    ; PD2: Inicio
    ; PD3: Selección de carga
    ; PD4: Puerta
    ; PD5: Sensor de agua
    ; PD6-PD7: Motor
    ldi temp, (1 << DDD6) | (1 << DDD7)
    out DDRD, temp

    ; Pull-up en entradas
    ldi temp, (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5)
    out PORTD, temp

    ; PB0-PB2: LEDs de carga
    ldi temp, (1 << DDB0) | (1 << DDB1) | (1 << DDB2)
    out DDRB, temp

    clr temp
    out PORTB, temp

    ; PC0-PC4: LEDs de estado
    ldi temp, 0b00011111
    out DDRC, temp

    clr temp
    out PORTC, temp

    ; Estado inicial
    ldi estado, ST_LISTO
    ldi carga, CARGA_LIGERA

    clr debounce
    clr boton_activo
    clr flags


    ; INT0 e INT1 por flanco de bajada
    ldi temp, (1 << ISC01) | (1 << ISC11)
    sts EICRA, temp

    ; Habilitar INT0 e INT1
    ldi temp, (1 << INT0) | (1 << INT1)
    out EIMSK, temp


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
    rcall procesar_estado
    rcall actualizar_leds

    rjmp main


;;;;;;;;;;;;;;;;;;;;;
; Máquina de estados
;;;;;;;;;;;;;;;;;;;;;
procesar_estado:

    cpi estado, ST_LISTO
    breq estado_listo

    cpi estado, ST_ESPERA_PUERTA
    breq estado_espera_puerta

    cpi estado, ST_ESPERA_LLENADO
    breq estado_espera_llenado

    ret


estado_listo:

    ; Esperar pulsador de inicio
    sbrs flags, FLAG_INICIO
    ret

    ; Limpiar evento
    andi flags, ~(1 << FLAG_INICIO)

    ldi estado, ST_ESPERA_PUERTA
    ret


estado_espera_puerta:

    ; 0 = puerta cerrada
    ; 1 = puerta abierta
    sbic PIND, PIND4
    ret

    ldi estado, ST_ESPERA_LLENADO
    ret


estado_espera_llenado:

    ; Si se abre la puerta, volver a esperar
    sbic PIND, PIND4
    rjmp volver_espera_puerta

    ; 0 = llenado alcanzado
    ; 1 = todavía no lleno
    sbic PIND, PIND5
    ret

    ldi estado, ST_LAVADO_GIRO
    ret


volver_espera_puerta:
    ldi estado, ST_ESPERA_PUERTA
    ret


;;;;;;;;;;;;;;;;;;;;;
; Actualización de LEDs
;;;;;;;;;;;;;;;;;;;;;
actualizar_leds:

    ; LEDs de carga
    cpi carga, CARGA_LIGERA
    breq led_carga_ligera

    cpi carga, CARGA_MEDIA
    breq led_carga_media

    ; Pesada
    ldi temp, (1 << PORTB2)
    out PORTB, temp
    rjmp actualizar_estado


led_carga_ligera:
    ldi temp, (1 << PORTB0)
    out PORTB, temp
    rjmp actualizar_estado


led_carga_media:
    ldi temp, (1 << PORTB1)
    out PORTB, temp


actualizar_estado:

    clr temp

    cpi estado, ST_LISTO
    breq led_listo

    cpi estado, ST_LAVADO_GIRO
    breq led_lavado

    cpi estado, ST_LAVADO_PAUSA
    breq led_lavado

    cpi estado, ST_CENTRIFUGADO
    breq led_centrifugado

    cpi estado, ST_SECADO_DER
    breq led_secado

    cpi estado, ST_SECADO_PAUSA
    breq led_secado

    cpi estado, ST_SECADO_IZQ
    breq led_secado

    cpi estado, ST_FIN
    breq led_fin

    ; Estados de espera: ningún LED de proceso
    out PORTC, temp
    ret


led_listo:
    ldi temp, (1 << PORTC0)
    out PORTC, temp
    ret


led_lavado:
    ldi temp, (1 << PORTC1)
    out PORTC, temp
    ret


led_centrifugado:
    ldi temp, (1 << PORTC2)
    out PORTC, temp
    ret


led_secado:
    ldi temp, (1 << PORTC3)
    out PORTC, temp
    ret


led_fin:
    ldi temp, (1 << PORTC4)
    out PORTC, temp
    ret


;;;;;;;;;;;;;;;;;;;;;
; INT0 - Inicio
;;;;;;;;;;;;;;;;;;;;;
isr_inicio:
    push temp
    in temp, SREG
    push temp

    tst boton_activo
    brne inicio_fin

    ldi boton_activo, 1
    ldi debounce, 20

    ; Inicio solo tiene efecto en LISTO
    cpi estado, ST_LISTO
    brne inicio_fin

    ori flags, (1 << FLAG_INICIO)


inicio_fin:
    pop temp
    out SREG, temp
    pop temp
    reti


;;;;;;;;;;;;;;;;;;;;;
; INT1 - Selección de carga
;;;;;;;;;;;;;;;;;;;;;
isr_carga:
    push temp
    in temp, SREG
    push temp

    tst boton_activo
    brne carga_fin

    ldi boton_activo, 2
    ldi debounce, 20

    ; La carga solo cambia en LISTO
    cpi estado, ST_LISTO
    brne carga_fin

    inc carga

    cpi carga, 3
    brne carga_fin

    clr carga


carga_fin:
    pop temp
    out SREG, temp
    pop temp
    reti


;;;;;;;;;;;;;;;;;;;;;
; Timer0 - antirrebote
;;;;;;;;;;;;;;;;;;;;;
isr_timer0:
    push temp
    in temp, SREG
    push temp

    tst boton_activo
    breq timer_fin

    cpi boton_activo, 1
    breq revisar_inicio

    ; Botón selección
    sbic PIND, PIND3
    rjmp liberado
    rjmp presionado


revisar_inicio:
    sbic PIND, PIND2
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