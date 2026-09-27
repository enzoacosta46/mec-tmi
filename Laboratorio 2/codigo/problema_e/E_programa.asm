.include "m328pdef.inc"

; ============================================================
; Laboratorio 2 - Tecnologias de Microprocesamiento
; Problema E - Automatizacion de una puerta de garaje
; ============================================================


; ------------------------------------------------------------
; REGISTROS
; ------------------------------------------------------------

.def temp        = r16
.def estado      = r17
.def dato        = r18
.def evento_seg  = r19
.def rearme      = r20


; ------------------------------------------------------------
; ESTADOS DE LA MAQUINA
; ------------------------------------------------------------

.equ P_CERRADA         = 0
.equ P_ABRIENDO        = 1
.equ P_ABIERTA         = 2
.equ P_CERRANDO        = 3
.equ PARADA_EMERGENCIA = 4
.equ P_DETENIDA        = 5


; ------------------------------------------------------------
; SALIDAS - PUERTO B
; ------------------------------------------------------------

.equ MOTOR_ABRIR  = PB2
.equ MOTOR_CERRAR = PB3
.equ ALARMA       = PB4


; ------------------------------------------------------------
; ENTRADAS - PUERTO C
; ------------------------------------------------------------

.equ BTN_ABRIR  = PC0
.equ BTN_CERRAR = PC1
.equ S_ABIERTA  = PC2
.equ S_CERRADA  = PC3


; ------------------------------------------------------------
; SENSOR DE OBSTACULO
; ------------------------------------------------------------

.equ S_OBSTACULO = PB0


; ============================================================
; TABLA DE VECTORES
; ============================================================

.cseg

.org 0x0000
    rjmp RESET

.org 0x0006
    rjmp ISR_OBSTACULO

.org 0x0034


; ============================================================
; RESET / INICIALIZACION
; ============================================================

RESET:

    cli

    ldi temp, HIGH(RAMEND)
    out SPH, temp

    ldi temp, LOW(RAMEND)
    out SPL, temp

    ldi temp, (1 << MOTOR_ABRIR) | (1 << MOTOR_CERRAR) | (1 << ALARMA)
    out DDRB, temp

    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    cbi DDRB, S_OBSTACULO
    sbi PORTB, S_OBSTACULO

    cbi DDRC, BTN_ABRIR
    cbi DDRC, BTN_CERRAR
    cbi DDRC, S_ABIERTA
    cbi DDRC, S_CERRADA

    sbi PORTC, BTN_ABRIR
    sbi PORTC, BTN_CERRAR
    sbi PORTC, S_ABIERTA
    sbi PORTC, S_CERRADA

    ldi temp, (1 << PCIE0)
    sts PCICR, temp

    ldi temp, (1 << PCINT0)
    sts PCMSK0, temp

    ldi temp, (1 << PCIF0)
    out PCIFR, temp

    ; USART0 - 9600 baudios, 8 bits, 1 stop, sin paridad

    clr temp
    sts UCSR0A, temp

    ldi temp, HIGH(103)
    sts UBRR0H, temp

    ldi temp, LOW(103)
    sts UBRR0L, temp

    ldi temp, (1 << TXEN0)
    sts UCSR0B, temp

    ldi temp, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, temp

    sbi DDRD, DDD1

    clr evento_seg
    clr rearme

    ; S_CERRADA activa = 0
    sbic PINC, S_CERRADA
    rjmp RESET_CHECK_ABIERTA

    ldi estado, P_CERRADA
    rjmp RESET_ESTADO_LISTO

RESET_CHECK_ABIERTA:

    ; S_ABIERTA activa = 0
    sbic PINC, S_ABIERTA
    rjmp RESET_INTERMEDIA

    ldi estado, P_ABIERTA
    rjmp RESET_ESTADO_LISTO

RESET_INTERMEDIA:

    ldi estado, P_DETENIDA

RESET_ESTADO_LISTO:

    ; Habilitar interrupciones globales
    sei

    ; Informar estado inicial
    cpi estado, P_CERRADA
    brne RESET_MSG_ABIERTA

    ldi ZL, LOW(M_CERRADA << 1)
    ldi ZH, HIGH(M_CERRADA << 1)
    rcall CADENA_USART
    rjmp MAIN_LOOP

RESET_MSG_ABIERTA:

    cpi estado, P_ABIERTA
    brne RESET_MSG_DETENIDA

    ldi ZL, LOW(M_ABIERTA << 1)
    ldi ZH, HIGH(M_ABIERTA << 1)
    rcall CADENA_USART
    rjmp MAIN_LOOP

RESET_MSG_DETENIDA:

    ldi ZL, LOW(M_DETENIDA << 1)
    ldi ZH, HIGH(M_DETENIDA << 1)
    rcall CADENA_USART

MAIN_LOOP:

    tst evento_seg
    breq MAIN_DESPACHO

    clr evento_seg

    ldi ZL, LOW(M_OBSTACULO << 1)
    ldi ZH, HIGH(M_OBSTACULO << 1)
    rcall CADENA_USART

    ldi ZL, LOW(M_SEGURIDAD << 1)
    ldi ZH, HIGH(M_SEGURIDAD << 1)
    rcall CADENA_USART


MAIN_DESPACHO:

    cpi estado, P_CERRADA
    brne MAIN_CHECK_ABRIENDO
    rjmp EST_CERRADA

MAIN_CHECK_ABRIENDO:

    cpi estado, P_ABRIENDO
    brne MAIN_CHECK_ABIERTA
    rjmp EST_ABRIENDO

MAIN_CHECK_ABIERTA:

    cpi estado, P_ABIERTA
    brne MAIN_CHECK_CERRANDO
    rjmp EST_ABIERTA

MAIN_CHECK_CERRANDO:

    cpi estado, P_CERRANDO
    brne MAIN_CHECK_EMERGENCIA
    rjmp EST_CERRANDO

MAIN_CHECK_EMERGENCIA:

    cpi estado, PARADA_EMERGENCIA
    brne MAIN_CHECK_DETENIDA
    rjmp EST_EMERGENCIA

MAIN_CHECK_DETENIDA:

    cpi estado, P_DETENIDA
    brne MAIN_ESTADO_INVALIDO
    rjmp EST_DETENIDA

MAIN_ESTADO_INVALIDO:

    cli
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA
    ldi estado, P_DETENIDA
    sei
    rjmp MAIN_LOOP


; ESTADO: PUERTA CERRADA

EST_CERRADA:

    ; Salidas en reposo
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ; Esperar pulsador ABRIR (activo en 0)
    sbic PINC, BTN_ABRIR
    rjmp MAIN_LOOP

    rcall INICIAR_APERTURA
    rjmp MAIN_LOOP


; ESTADO: PUERTA ABRIENDO

EST_ABRIENDO:
    cli
	
    cpi estado, P_ABRIENDO
    breq ABRIENDO_ESTADO_OK

    sei
    rjmp MAIN_LOOP

ABRIENDO_ESTADO_OK:

    ; Si el obstaculo ya esta activo, actuar aunque se haya perdido
    sbis PINB, S_OBSTACULO
    rjmp ABRIENDO_OBSTACULO

    ; Final de carrera de puerta abierta: activo en 0
    sbic PINC, S_ABIERTA
    rjmp ABRIENDO_CONTINUA

    ; Llegada al extremo abierto
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, P_ABIERTA

    sei

    ldi ZL, LOW(M_ABIERTA << 1)
    ldi ZH, HIGH(M_ABIERTA << 1)
    rcall CADENA_USART

    rjmp MAIN_LOOP

ABRIENDO_CONTINUA:

    sei
    rjmp MAIN_LOOP

ABRIENDO_OBSTACULO:

    ; Mismo resultado que la ISR, usado como respaldo por polling.
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, PARADA_EMERGENCIA
    ldi evento_seg, 1
    clr rearme

    sei
    rjmp MAIN_LOOP


; ESTADO: PUERTA ABIERTA

EST_ABIERTA:

    ; Salidas en reposo
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ; Esperar pulsador CERRAR (activo en 0)
    sbic PINC, BTN_CERRAR
    rjmp MAIN_LOOP

    rcall INICIAR_CIERRE
    rjmp MAIN_LOOP


; ESTADO: PUERTA CERRANDO

EST_CERRANDO:

    cli

    ; Confirmar que seguimos realmente en cierre.
    cpi estado, P_CERRANDO
    breq CERRANDO_ESTADO_OK

    sei
    rjmp MAIN_LOOP

CERRANDO_ESTADO_OK:

    ; Respaldo de seguridad por polling
    sbis PINB, S_OBSTACULO
    rjmp CERRANDO_OBSTACULO

    ; Final de carrera de puerta cerrada: activo en 0
    sbic PINC, S_CERRADA
    rjmp CERRANDO_CONTINUA

    ; Llegada al extremo cerrado
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, P_CERRADA

    sei

    ldi ZL, LOW(M_CERRADA << 1)
    ldi ZH, HIGH(M_CERRADA << 1)
    rcall CADENA_USART

    rjmp MAIN_LOOP

CERRANDO_CONTINUA:

    sei
    rjmp MAIN_LOOP

CERRANDO_OBSTACULO:

    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, PARADA_EMERGENCIA
    ldi evento_seg, 1
    clr rearme

    sei
    rjmp MAIN_LOOP


; ESTADO: PUERTA DETENIDA EN POSICION INTERMEDIA

EST_DETENIDA:

    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ; ABRIR tiene prioridad si ambos se presionaran a la vez.
    sbic PINC, BTN_ABRIR
    rjmp DETENIDA_CHECK_CERRAR

    rcall INICIAR_APERTURA
    rjmp MAIN_LOOP

DETENIDA_CHECK_CERRAR:

    sbic PINC, BTN_CERRAR
    rjmp MAIN_LOOP

    rcall INICIAR_CIERRE
    rjmp MAIN_LOOP


; ESTADO: PARADA DE EMERGENCIA

EST_EMERGENCIA:

    ; Mantener siempre el sistema detenido
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    sbis PINB, S_OBSTACULO
    rjmp EMERG_OBSTACULO_PRESENTE

    tst rearme
    brne EMERG_ESPERAR_ORDEN

    sbis PINC, BTN_ABRIR
    rjmp MAIN_LOOP

    sbis PINC, BTN_CERRAR
    rjmp MAIN_LOOP

    ldi rearme, 1
    rjmp MAIN_LOOP

EMERG_OBSTACULO_PRESENTE:

    clr rearme
    rjmp MAIN_LOOP

EMERG_ESPERAR_ORDEN:

    ; Nueva pulsacion ABRIR
    sbic PINC, BTN_ABRIR
    rjmp EMERG_CHECK_CERRAR

    clr rearme
    rcall INICIAR_APERTURA
    rjmp MAIN_LOOP

EMERG_CHECK_CERRAR:

    ; Nueva pulsacion CERRAR
    sbic PINC, BTN_CERRAR
    rjmp MAIN_LOOP

    clr rearme
    rcall INICIAR_CIERRE
    rjmp MAIN_LOOP


; INICIAR APERTURA

INICIAR_APERTURA:

    cli

    ; Si ya esta completamente abierta, no mover.
    sbic PINC, S_ABIERTA
    rjmp INI_ABRIR_CHECK_OBSTACULO

    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA
    ldi estado, P_ABIERTA

    sei

    ldi ZL, LOW(M_ABIERTA << 1)
    ldi ZH, HIGH(M_ABIERTA << 1)
    rcall CADENA_USART
    ret

INI_ABRIR_CHECK_OBSTACULO:

    ; No iniciar movimiento si el sensor ya detecta un obstaculo.
    sbis PINB, S_OBSTACULO
    rjmp INI_ABRIR_BLOQUEADA

    ; Arranque de apertura
    cbi PORTB, MOTOR_CERRAR
    sbi PORTB, MOTOR_ABRIR
    sbi PORTB, ALARMA
    ldi estado, P_ABRIENDO

    sei

    ldi ZL, LOW(M_ABRIENDO << 1)
    ldi ZH, HIGH(M_ABRIENDO << 1)
    rcall CADENA_USART
    ret

INI_ABRIR_BLOQUEADA:

    ; Obstaculo presente antes de comenzar: se trata como parada
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, PARADA_EMERGENCIA
    ldi evento_seg, 1
    clr rearme

    sei
    ret


; INICIAR CIERRE

INICIAR_CIERRE:

    cli

    ; Si ya esta completamente cerrada, no mover.
    sbic PINC, S_CERRADA
    rjmp INI_CERRAR_CHECK_OBSTACULO

    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA
    ldi estado, P_CERRADA

    sei

    ldi ZL, LOW(M_CERRADA << 1)
    ldi ZH, HIGH(M_CERRADA << 1)
    rcall CADENA_USART
    ret

INI_CERRAR_CHECK_OBSTACULO:

    ; No iniciar movimiento si el sensor ya detecta un obstaculo.
    sbis PINB, S_OBSTACULO
    rjmp INI_CERRAR_BLOQUEADA

    ; Arranque de cierre
    cbi PORTB, MOTOR_ABRIR
    sbi PORTB, MOTOR_CERRAR
    sbi PORTB, ALARMA
    ldi estado, P_CERRANDO

    sei

    ldi ZL, LOW(M_CERRANDO << 1)
    ldi ZH, HIGH(M_CERRANDO << 1)
    rcall CADENA_USART
    ret

INI_CERRAR_BLOQUEADA:

    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, PARADA_EMERGENCIA
    ldi evento_seg, 1
    clr rearme

    sei
    ret


; USART - TRANSMITIR UN CARACTER

MENSAJE_USART:

ESPERAR_USART:

    lds temp, UCSR0A
    sbrs temp, UDRE0
    rjmp ESPERAR_USART

    sts UDR0, dato
    ret


; USART - TRANSMITIR UNA CADENA DESDE FLASH

CADENA_USART:

    lpm dato, Z+

    tst dato
    breq FINAL_MENSAJE

    rcall MENSAJE_USART
    rjmp CADENA_USART

FINAL_MENSAJE:

    ret


; INTERRUPCION - SENSOR DE OBSTACULO
; PB0 / PCINT0

ISR_OBSTACULO:

    ; Guardar correctamente r16 original y SREG.
    push temp
    in temp, SREG
    push temp

    ; Si PB0 = 1, fue liberacion del sensor: no hacer nada.
    sbic PINB, S_OBSTACULO
    rjmp FIN_ISR

    ; Solo detener por obstaculo durante movimiento.
    cpi estado, P_ABRIENDO
    breq ISR_PARAR

    cpi estado, P_CERRANDO
    brne FIN_ISR

ISR_PARAR:

    ; Accion critica e inmediata de seguridad
    cbi PORTB, MOTOR_ABRIR
    cbi PORTB, MOTOR_CERRAR
    cbi PORTB, ALARMA

    ldi estado, PARADA_EMERGENCIA
    ldi evento_seg, 1
    clr rearme

FIN_ISR:

    ; Restaurar SREG y r16 original
    pop temp
    out SREG, temp
    pop temp

    reti


; MENSAJES USART

M_ABRIENDO:
    .db "Puerta abriendo", 13, 10, 0

M_ABIERTA:
    .db "Puerta abierta ", 13, 10, 0

M_CERRANDO:
    .db "Puerta cerrando", 13, 10, 0

M_CERRADA:
    .db "Puerta cerrada ", 13, 10, 0

M_OBSTACULO:
    .db "Obstaculo detectado", 13, 10, 0

M_SEGURIDAD:
    .db "Movimiento detenido por seguridad", 13, 10, 0

M_DETENIDA:
    .db "Puerta detenida - posicion intermedia", 13, 10, 0
