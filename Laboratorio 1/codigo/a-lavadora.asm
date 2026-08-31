.include "m328pdef.inc"

.def temp = r16
.def estado = r17
.def debounce = r18
.def boton_activo = r19
.def carga = r20
.def flags = r21
.def tiempo = r22
.def ciclos_lavado = r23
.def tick_ms = r24


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

; Flags
.equ FLAG_INICIO = 0
.equ FLAG_PAUSA  = 1


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
    clr tiempo
    clr ciclos_lavado
    clr tick_ms


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
    rcall seguridad_puerta
    rcall procesar_estado
    rcall actualizar_leds

    rjmp main


;;;;;;;;;;;;;;;;;;;;;
; Seguridad de puerta
;;;;;;;;;;;;;;;;;;;;;
seguridad_puerta:

    ; Sólo aplica durante el proceso
    cpi estado, ST_LAVADO_GIRO
    brlo seguridad_fuera_proceso

    cpi estado, ST_FIN
    brsh seguridad_fuera_proceso

    ; PD4 = 0 -> puerta cerrada
    ; PD4 = 1 -> puerta abierta
    sbic PIND, PIND4
    rjmp puerta_abierta

    ; Quitar pausa
    andi flags, 0b11111101
    ret


puerta_abierta:
    ori flags, (1 << FLAG_PAUSA)
    rcall motor_stop
    ret


seguridad_fuera_proceso:
    andi flags, 0b11111101
    ret


;;;;;;;;;;;;;;;;;;;;;
; Máquina de estados
;;;;;;;;;;;;;;;;;;;;;
procesar_estado:

    cpi estado, ST_LISTO
    brne revisar_st_puerta
    rjmp estado_listo

revisar_st_puerta:
    cpi estado, ST_ESPERA_PUERTA
    brne revisar_st_llenado
    rjmp estado_espera_puerta

revisar_st_llenado:
    cpi estado, ST_ESPERA_LLENADO
    brne revisar_st_lavado_giro
    rjmp estado_espera_llenado

revisar_st_lavado_giro:
    cpi estado, ST_LAVADO_GIRO
    brne revisar_st_lavado_pausa
    rjmp estado_lavado_giro

revisar_st_lavado_pausa:
    cpi estado, ST_LAVADO_PAUSA
    brne revisar_st_centrifugado
    rjmp estado_lavado_pausa

revisar_st_centrifugado:
    cpi estado, ST_CENTRIFUGADO
    brne revisar_st_secado_der
    rjmp estado_centrifugado

revisar_st_secado_der:
    cpi estado, ST_SECADO_DER
    brne revisar_st_secado_pausa
    rjmp estado_secado_der

revisar_st_secado_pausa:
    cpi estado, ST_SECADO_PAUSA
    brne revisar_st_secado_izq
    rjmp estado_secado_pausa

revisar_st_secado_izq:
    cpi estado, ST_SECADO_IZQ
    brne revisar_st_fin
    rjmp estado_secado_izq

revisar_st_fin:
    cpi estado, ST_FIN
    brne estado_desconocido
    rjmp estado_fin

estado_desconocido:
    ret


;;;;;;;;;;;;;;;;;;;;;
; LISTO
;;;;;;;;;;;;;;;;;;;;;
estado_listo:
    rcall motor_stop

    ; Esperar pulsador de inicio
    sbrs flags, FLAG_INICIO
    ret

    ; Limpiar evento
    andi flags, ~(1 << FLAG_INICIO)

    clr ciclos_lavado
    clr tiempo

    ldi estado, ST_ESPERA_PUERTA
    ret


;;;;;;;;;;;;;;;;;;;;;
; ESPERA PUERTA
;;;;;;;;;;;;;;;;;;;;;
estado_espera_puerta:
    rcall motor_stop

    ; Esperar puerta cerrada
    sbic PIND, PIND4
    ret

    ldi estado, ST_ESPERA_LLENADO
    ret


;;;;;;;;;;;;;;;;;;;;;
; ESPERA LLENADO
;;;;;;;;;;;;;;;;;;;;;
estado_espera_llenado:
    rcall motor_stop

    ; Si se abre la puerta, volver a esperar
    sbic PIND, PIND4
    rjmp volver_espera_puerta

    ; Esperar llenado
    sbic PIND, PIND5
    ret

    clr ciclos_lavado

    ldi estado, ST_LAVADO_GIRO
    rcall cargar_lavado_giro
    ret


volver_espera_puerta:
    ldi estado, ST_ESPERA_PUERTA
    ret


;;;;;;;;;;;;;;;;;;;;;
; LAVADO - GIRO
;;;;;;;;;;;;;;;;;;;;;
estado_lavado_giro:

    ; Proceso pausado por puerta
    sbrc flags, FLAG_PAUSA
    ret

    rcall motor_derecha

    tst tiempo
    brne lavado_giro_fin

    rcall motor_stop

    ldi estado, ST_LAVADO_PAUSA
    rcall cargar_lavado_pausa


lavado_giro_fin:
    ret


;;;;;;;;;;;;;;;;;;;;;
; LAVADO - PAUSA
;;;;;;;;;;;;;;;;;;;;;
estado_lavado_pausa:

    sbrc flags, FLAG_PAUSA
    ret

    rcall motor_stop

    tst tiempo
    brne lavado_pausa_fin

    ; Ciclo giro + pausa completado
    inc ciclos_lavado

    cpi ciclos_lavado, 5
    breq lavado_completo

    ldi estado, ST_LAVADO_GIRO
    rcall cargar_lavado_giro
    ret


lavado_completo:
    ldi estado, ST_CENTRIFUGADO
    rcall cargar_centrifugado


lavado_pausa_fin:
    ret


;;;;;;;;;;;;;;;;;;;;;
; CENTRIFUGADO
;;;;;;;;;;;;;;;;;;;;;
estado_centrifugado:

    sbrc flags, FLAG_PAUSA
    ret

    rcall motor_derecha

    tst tiempo
    brne centrifugado_fin

    rcall motor_stop

    ldi estado, ST_SECADO_DER
    rcall cargar_secado_giro


centrifugado_fin:
    ret


;;;;;;;;;;;;;;;;;;;;;
; SECADO - DERECHA
;;;;;;;;;;;;;;;;;;;;;
estado_secado_der:

    sbrc flags, FLAG_PAUSA
    ret

    rcall motor_derecha

    tst tiempo
    brne secado_der_fin

    rcall motor_stop

    ldi estado, ST_SECADO_PAUSA
    rcall cargar_secado_pausa


secado_der_fin:
    ret


;;;;;;;;;;;;;;;;;;;;;
; SECADO - PAUSA
;;;;;;;;;;;;;;;;;;;;;
estado_secado_pausa:

    sbrc flags, FLAG_PAUSA
    ret

    rcall motor_stop

    tst tiempo
    brne secado_pausa_fin

    ldi estado, ST_SECADO_IZQ
    rcall cargar_secado_giro


secado_pausa_fin:
    ret


;;;;;;;;;;;;;;;;;;;;;
; SECADO - IZQUIERDA
;;;;;;;;;;;;;;;;;;;;;
estado_secado_izq:

    sbrc flags, FLAG_PAUSA
    ret

    rcall motor_izquierda

    tst tiempo
    brne secado_izq_fin

    rcall motor_stop

    ldi estado, ST_FIN


secado_izq_fin:
    ret


;;;;;;;;;;;;;;;;;;;;;
; FIN
;;;;;;;;;;;;;;;;;;;;;
estado_fin:
    rcall motor_stop

    ; Inicio permite volver a LISTO
    sbrs flags, FLAG_INICIO
    ret

    andi flags, 0b11111110

    clr tiempo
    clr ciclos_lavado

    ldi estado, ST_LISTO
    ret


;;;;;;;;;;;;;;;;;;;;;
; Tiempos de lavado
; tiempo se expresa en 100 ms
;;;;;;;;;;;;;;;;;;;;;

; Giro:
; ligera = 2 s
; media  = 3 s
; pesada = 4 s
cargar_lavado_giro:

    cpi carga, CARGA_LIGERA
    breq lavado_giro_ligera

    cpi carga, CARGA_MEDIA
    breq lavado_giro_media

    ldi temp, 40
    rjmp iniciar_tiempo


lavado_giro_ligera:
    ldi temp, 20
    rjmp iniciar_tiempo


lavado_giro_media:
    ldi temp, 30
    rjmp iniciar_tiempo


; Pausa:
; ligera = 1 s
; media  = 2 s
; pesada = 3 s
cargar_lavado_pausa:

    cpi carga, CARGA_LIGERA
    breq lavado_pausa_ligera

    cpi carga, CARGA_MEDIA
    breq lavado_pausa_media

    ldi temp, 30
    rjmp iniciar_tiempo


lavado_pausa_ligera:
    ldi temp, 10
    rjmp iniciar_tiempo


lavado_pausa_media:
    ldi temp, 20
    rjmp iniciar_tiempo


; Centrifugado:
; ligera = 15 s
; media  = 18 s
; pesada = 21 s
cargar_centrifugado:

    cpi carga, CARGA_LIGERA
    breq centrifugado_ligera

    cpi carga, CARGA_MEDIA
    breq centrifugado_media

    ldi temp, 210
    rjmp iniciar_tiempo


centrifugado_ligera:
    ldi temp, 150
    rjmp iniciar_tiempo


centrifugado_media:
    ldi temp, 180
    rjmp iniciar_tiempo


; Secado - giro:
; ligera = 5 s
; media  = 7 s
; pesada = 9 s
cargar_secado_giro:

    cpi carga, CARGA_LIGERA
    breq secado_giro_ligera

    cpi carga, CARGA_MEDIA
    breq secado_giro_media

    ldi temp, 90
    rjmp iniciar_tiempo


secado_giro_ligera:
    ldi temp, 50
    rjmp iniciar_tiempo


secado_giro_media:
    ldi temp, 70
    rjmp iniciar_tiempo


; Secado - pausa:
; ligera = 3 s
; media  = 5 s
; pesada = 7 s
cargar_secado_pausa:

    cpi carga, CARGA_LIGERA
    breq secado_pausa_ligera

    cpi carga, CARGA_MEDIA
    breq secado_pausa_media

    ldi temp, 70
    rjmp iniciar_tiempo


secado_pausa_ligera:
    ldi temp, 30
    rjmp iniciar_tiempo


secado_pausa_media:
    ldi temp, 50


iniciar_tiempo:
    clr tick_ms
    mov tiempo, temp
    ret


;;;;;;;;;;;;;;;;;;;;;
; Motor
;;;;;;;;;;;;;;;;;;;;;
motor_stop:
    cbi PORTD, PORTD6
    cbi PORTD, PORTD7
    ret


motor_derecha:
    cbi PORTD, PORTD7
    sbi PORTD, PORTD6
    ret


motor_izquierda:
    cbi PORTD, PORTD6
    sbi PORTD, PORTD7
    ret


;;;;;;;;;;;;;;;;;;;;;
; LEDs
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

    ; Estados de espera
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

    ; Inicio se acepta en LISTO o FIN
    cpi estado, ST_LISTO
    breq aceptar_inicio

    cpi estado, ST_FIN
    brne inicio_fin


aceptar_inicio:
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
; Timer0 - antirrebote y temporización
;;;;;;;;;;;;;;;;;;;;;
isr_timer0:
    push temp
    in temp, SREG
    push temp


    ; Antirrebote
    tst boton_activo
    breq temporizacion

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
    rjmp temporizacion


liberado:
    dec debounce
    brne temporizacion

    clr boton_activo


; Generar unidades de 100 ms
temporizacion:
    inc tick_ms

    cpi tick_ms, 100
    brlo timer_fin

    clr tick_ms

    ; No hay tiempo activo
    tst tiempo
    breq timer_fin

    ; Pausar temporizacion con puerta abierta
    sbrc flags, FLAG_PAUSA
    rjmp timer_fin

    dec tiempo


timer_fin:
    pop temp
    out SREG, temp
    pop temp
    reti