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

    ; Mostrar menú antes de utilizar Z para las LUT
    call enviar_menu

    ; Habilitar interrupciones globales
    sei


;------------------------------------------------------------
; Programa principal
;------------------------------------------------------------
main:

    ; Esperar un dato por USART
    call recibir_dato

    ; '1' -> Señal 14
    cpi dato, '1'
    breq opcion_senal14

    ; '2' -> Señal 7
    cpi dato, '2'
    breq opcion_senal7

    ; Ignorar cualquier otro carácter
    rjmp main


opcion_senal14:

    call seleccionar_senal14
    rjmp main


opcion_senal7:

    call seleccionar_senal7
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
; USART - TRANSMISIÓN
;------------------------------------------------------------
enviar_dato:

esperar_tx:

    ; Esperar hasta que UDR0 est? disponible
    lds temp, UCSR0A
    sbrs temp, UDRE0
    rjmp esperar_tx

    ; Enviar byte
    sts UDR0, dato

    ret


;------------------------------------------------------------
; USART - RECEPCIÓN
;------------------------------------------------------------
recibir_dato:

esperar_rx:

    ; Esperar hasta recibir un byte
    lds temp, UCSR0A
    sbrs temp, RXC0
    rjmp esperar_rx

    ; Leer dato recibido
    lds dato, UDR0

    ret


;------------------------------------------------------------
; MENÚ SERIAL
;------------------------------------------------------------
enviar_menu:

    ; Z apunta al inicio del mensaje en memoria Flash
    ldi ZL, LOW(menu << 1)
    ldi ZH, HIGH(menu << 1)

enviar_menu_loop:

    ; Leer siguiente carácter
    lpm dato, Z+

    ; Fin del texto al encontrar 0x00
    tst dato
    breq enviar_menu_fin

    call enviar_dato
    rjmp enviar_menu_loop

enviar_menu_fin:

    ret


;------------------------------------------------------------
; SELECCIÓN SEÑAL 14
;------------------------------------------------------------
seleccionar_senal14:

    ; Evitar interrupción mientras
    ; se modifican los punteros de la LUT
    cli

    ; Detener Timer1 manteniendo modo CTC
    ldi temp, (1 << WGM12)
    sts TCCR1B, temp

    ; X guarda el inicio de la LUT
    ldi XL, LOW(lut_senal14 << 1)
    ldi XH, HIGH(lut_senal14 << 1)

    ; Z comienza en la misma posición
    mov ZL, XL
    mov ZH, XH

    ; Índice inicial
    clr indice

    ; 0 representa 256 muestras por overflow
    clr limite

    ; Mostrar inmediatamente la primera muestra
    call siguiente_muestra

    ; Reiniciar contador Timer1
    clr temp
    sts TCNT1H, temp
    sts TCNT1L, temp

    ; Limpiar posible Compare Match pendiente
    ldi temp, (1 << OCF1A)
    out TIFR1, temp

    ; CTC + prescaler 1
    ldi temp, (1 << WGM12) | (1 << CS10)
    sts TCCR1B, temp

    ; Habilitar nuevamente interrupciones
    sei

    ret


;------------------------------------------------------------
; SELECCIÓN SEÑAL 7
;------------------------------------------------------------
seleccionar_senal7:

    ; Evitar interrupción mientras
    ; se modifican los punteros de la LUT
    cli

    ; Detener Timer1 manteniendo modo CTC
    ldi temp, (1 << WGM12)
    sts TCCR1B, temp

    ; X guarda el inicio de la LUT
    ldi XL, LOW(lut_senal7 << 1)
    ldi XH, HIGH(lut_senal7 << 1)

    ; Z comienza en la misma posici?n
    mov ZL, XL
    mov ZH, XH

    ; Índice inicial
    clr indice

    ; Se?al 7 contiene 64 muestras
    ldi limite, 64

    ; Mostrar inmediatamente la primera muestra
    call siguiente_muestra

    ; Reiniciar contador Timer1
    clr temp
    sts TCNT1H, temp
    sts TCNT1L, temp

    ; Limpiar posible Compare Match pendiente
    ldi temp, (1 << OCF1A)
    out TIFR1, temp

    ; CTC + prescaler 1
    ldi temp, (1 << WGM12) | (1 << CS10)
    sts TCCR1B, temp

    ; Habilitar nuevamente interrupciones
    sei

    ret


;------------------------------------------------------------
; SIGUIENTE MUESTRA DE LA LUT
;------------------------------------------------------------
siguiente_muestra:

    ; Leer byte apuntado por Z y avanzar
    lpm muestra, Z+

    ; Enviar muestra al DAC
    call actualizar_dac

    ; Incrementar n?mero de muestras recorridas
    inc indice

    ; Verificar final de la LUT
    cp indice, limite
    brne siguiente_muestra_fin

    ; Volver al inicio de la LUT
    mov ZL, XL
    mov ZH, XH

    clr indice


siguiente_muestra_fin:

    ret


;------------------------------------------------------------
; ACTUALIZACIÓN DEL DAC R-2R
;------------------------------------------------------------
actualizar_dac:

    ; Bits 0-5 de la muestra -> PD2-PD7
    mov temp, muestra

    ; Conservar solamente b0-b5
    andi temp, 0b00111111

    ; Desplazar hacia PD2-PD7
    lsl temp
    lsl temp

    ; Bits 6-7 de la muestra -> PB0-PB1
    mov temp2, muestra

    ; Desplazar b6-b7 hasta las posiciones 0-1
    lsr temp2
    lsr temp2
    lsr temp2
    lsr temp2
    lsr temp2
    lsr temp2

    ; Actualizar ambos puertos consecutivamente
    out PORTD, temp
    out PORTB, temp2

    ret


;------------------------------------------------------------
; ISR TIMER1
;------------------------------------------------------------
isr_timer1:

    ; Guardar registros modificados por la ISR
    push temp

    ; Guardar SREG
    in temp, SREG
    push temp

    push temp2
    push muestra

    ; Obtener y mostrar la siguiente muestra
    call siguiente_muestra

    ; Restaurar registros
    pop muestra
    pop temp2

    ; Restaurar SREG
    pop temp
    out SREG, temp

    pop temp

    reti


;============================================================
; DATOS EN MEMORIA DE PROGRAMA
;============================================================


;------------------------------------------------------------
; Menú USART
;------------------------------------------------------------
menu:

    .db 13, 10, "PROBLEMA B - DAC R-2R ", 13, 10
    .db "1 - Senal 14", 13, 10
    .db "2 - Senal 7 ", 13, 10
    .db "Seleccione: ", 0, 0


;------------------------------------------------------------
; LUT SEÑAL 7 - 64 muestras
;------------------------------------------------------------
lut_senal7:

    .db 0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78
    .db 0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8, 0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8
    .db 0xFF, 0xF7, 0xEF, 0xE7, 0xDF, 0xD7, 0xCF, 0xC7, 0xBF, 0xB7, 0xAF, 0xA7, 0x9F, 0x97, 0x8F, 0x87
    .db 0x7F, 0x77, 0x6F, 0x67, 0x5F, 0x57, 0x4F, 0x47, 0x3F, 0x37, 0x2F, 0x27, 0x1F, 0x17, 0x0F, 0x07


;------------------------------------------------------------
; LUT SEÑAL 14 - 256 muestras
;------------------------------------------------------------
lut_senal14:

    .db 0x00, 0x06, 0x0C, 0x12, 0x18, 0x1F, 0x25, 0x2B, 0x31, 0x37, 0x3D, 0x44, 0x4A, 0x4F, 0x55, 0x5B
    .db 0x61, 0x67, 0x6D, 0x72, 0x78, 0x7D, 0x83, 0x88, 0x8D, 0x92, 0x97, 0x9C, 0xA1, 0xA6, 0xAB, 0xAF
    .db 0xB4, 0xB8, 0xBC, 0xC1, 0xC5, 0xC9, 0xCC, 0xD0, 0xD4, 0xD7, 0xDA, 0xDD, 0xE0, 0xE3, 0xE6, 0xE9
    .db 0xEB, 0xED, 0xF0, 0xF2, 0xF4, 0xF5, 0xF7, 0xF8, 0xFA, 0xFB, 0xFC, 0xFD, 0xFD, 0xFE, 0xFE, 0xFE
    .db 0xFF, 0xFE, 0xFE, 0xFE, 0xFD, 0xFD, 0xFC, 0xFB, 0xFA, 0xF8, 0xF7, 0xF5, 0xF4, 0xF2, 0xF0, 0xED
    .db 0xEB, 0xE9, 0xE6, 0xE3, 0xE0, 0xDD, 0xDA, 0xD7, 0xD4, 0xD0, 0xCC, 0xC9, 0xC5, 0xC1, 0xBC, 0xB8
    .db 0xB4, 0xAF, 0xAB, 0xA6, 0xA1, 0x9C, 0x97, 0x92, 0x8D, 0x88, 0x83, 0x7D, 0x78, 0x72, 0x6D, 0x67
    .db 0x61, 0x5B, 0x55, 0x4F, 0x4A, 0x44, 0x3D, 0x37, 0x31, 0x2B, 0x25, 0x1F, 0x18, 0x12, 0x0C, 0x06

    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00