;============================================================
; Laboratorio 2 - Tecnologías de Microprocesamiento
; Grupo 7
; Problema A - Matriz de LEDs con UART
;============================================================

.include "m328pdef.inc"


;------------------------------------------------------------
; Definición de registros
;------------------------------------------------------------
.def temp              = r16
.def temp2             = r17
.def dato              = r18
.def modo_actual       = r19
.def fila              = r20
.def patron            = r21
.def contador_scroll   = r22
.def periodo_scroll    = r23
.def indice_caracter   = r24
.def indice_columna    = r25
.def scroll_pendiente  = r15


;------------------------------------------------------------
; Vectores de interrupción
;------------------------------------------------------------
.cseg

.org 0x0000
    rjmp inicio

.org OC0Aaddr
    rjmp isr_timer0

; Comenzar programa después de la tabla de vectores
.org 0x0034


;------------------------------------------------------------
; Inicialización
;------------------------------------------------------------
inicio:

    cli

    ; Inicialización del Stack Pointer
    ldi temp, HIGH(RAMEND)
    out SPH, temp

    ldi temp, LOW(RAMEND)
    out SPL, temp
	
    ; Configuración de periféricos
    call configurar_gpio
    call configurar_usart

    ; Cargar datos gráficos en SRAM
    call guardar_imagenes
    call guardar_fuente
    call guardar_mensaje
    call limpiar_buffer

    ; Valores iniciales
    clr fila
    clr modo_actual

    clr contador_scroll
    ldi periodo_scroll, 150

    clr scroll_pendiente
    clr indice_caracter
    clr indice_columna

    ; Mostrar bienvenida y menú
    call enviar_menu

    ; Configurar Timer0
    call configurar_timer0

    ; Habilitar interrupciones globales
    sei


;------------------------------------------------------------
; Programa principal
;------------------------------------------------------------
main:

    ; Consultar USART sin bloquear el programa
    call revisar_usart

    ; Verificar si corresponde mover el mensaje
    tst scroll_pendiente
    breq main

    ; Consumir solicitud de desplazamiento
    clr scroll_pendiente

    ; El desplazamiento solamente se realiza
    ; cuando está seleccionado el modo mensaje
    tst modo_actual
    brne main

    call avanzar_mensaje

    rjmp main


;------------------------------------------------------------
; CONFIGURACIÓN GPIO
;------------------------------------------------------------
configurar_gpio:

    ; PD4-PD7 como salidas: filas 1-4
    ; PD0 reservado para RX
    ldi temp, 0b11110000
    out DDRD, temp

    ; PD1 / TXD como salida
    sbi DDRD, DDD1

    ; PB0-PB3: filas 5-8
    ; PB4-PB5: columnas 1-2
    ldi temp, 0b00111111
    out DDRB, temp

    ; PC0-PC5: columnas 3-8
    ldi temp, 0b00111111
    out DDRC, temp

    ; Filas 1-4 inicialmente desactivadas
    ldi temp, 0b11110000
    out PORTD, temp

    ; Filas 5-8 desactivadas
    ; Columnas 1-2 apagadas
    ldi temp, 0b00001111
    out PORTB, temp

    ; Columnas 3-8 apagadas
    clr temp
    out PORTC, temp

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
; CONFIGURACIÓN TIMER0
;------------------------------------------------------------
configurar_timer0:

    ; Timer0 en modo CTC
    ldi temp, (1 << WGM01)
    out TCCR0A, temp

    ; OCR0A = 249 calculado para
    ; Interrupción cada 1 ms
    ldi temp, 249
    out OCR0A, temp

    ; Prescaler 64
    ldi temp, (1 << CS01) | (1 << CS00)
    out TCCR0B, temp

    ; Habilitar interrupción por Compare Match A
    ldi temp, (1 << OCIE0A)
    sts TIMSK0, temp

    ret


;------------------------------------------------------------
; USART - TRANSMISIÓN
;------------------------------------------------------------
enviar_dato:

esperar_tx:

    ; Esperar hasta que UDR0 esté disponible
    lds temp, UCSR0A
    sbrs temp, UDRE0
    rjmp esperar_tx

    ; Enviar byte
    sts UDR0, dato

    ret


;------------------------------------------------------------
; USART - RECEPCIÓN NO BLOQUEANTE
;------------------------------------------------------------
revisar_usart:

    ; Consultar si existe un dato recibido
    lds temp, UCSR0A
    sbrs temp, RXC0
    ret

    ; Leer dato recibido
    lds dato, UDR0

    ; '1' -> Mensaje
    cpi dato, '1'
    breq opcion_mensaje

    ; '2' -> Carita sonriendo
    cpi dato, '2'
    breq opcion_sonrisa

    ; '3' -> Carita guiñando
    cpi dato, '3'
    breq opcion_guino

    ; '4' -> Corazón
    cpi dato, '4'
    breq opcion_corazon

    ; '5' -> :3
    cpi dato, '5'
    breq opcion_3

    ; '6' -> Asterisco
    cpi dato, '6'
    breq opcion_asterisco

    ; '7' -> XD
    cpi dato, '7'
    breq opcion_xd

	; '+' -> Aumentar velocidad
    cpi dato, '+'
    breq aumentar_velocidad

    ; '-' -> Disminuir velocidad
    cpi dato, '-'
    breq disminuir_velocidad

    ; Ignorar cualquier otro carácter
    ret


opcion_mensaje:

    clr modo_actual

    ; Reiniciar desplazamiento desde el comienzo
    clr contador_scroll
    clr scroll_pendiente
    clr indice_caracter
    clr indice_columna

    call limpiar_buffer

    ret


opcion_sonrisa:

    ldi modo_actual, 1
    ret


opcion_guino:

    ldi modo_actual, 2
    ret


opcion_corazon:

    ldi modo_actual, 3
    ret


opcion_3:

    ldi modo_actual, 4
    ret


opcion_asterisco:

    ldi modo_actual, 5
    ret


opcion_xd:

    ldi modo_actual, 6
    ret

;------------------------------------------------------------
; CONTROL DE VELOCIDAD
;------------------------------------------------------------
aumentar_velocidad:

    ; Mínimo: 100 ms por columna
    cpi periodo_scroll, 100
    breq velocidad_fin

    ; Menor período = mayor velocidad
    subi periodo_scroll, 25

    clr contador_scroll
    clr scroll_pendiente

    ret


disminuir_velocidad:

    ; Máximo: 250 ms por columna
    cpi periodo_scroll, 250
    breq velocidad_fin

    ; Mayor período = menor velocidad
    ldi temp, 25
    add periodo_scroll, temp

    clr contador_scroll
    clr scroll_pendiente


velocidad_fin:

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
; OBTENER PATRÓN DE LA FILA ACTUAL
;------------------------------------------------------------
get_fila:

    ; modo_actual = 0 -> buffer del mensaje
    tst modo_actual
    breq get_fila_mensaje

    ; Figura seleccionada

    ; Las imágenes comienzan en SRAM 0x0100
    ldi YL, 0x00
    ldi YH, 0x01

    ; Convertir modo 1-6 a índice de imagen 0-5
    mov temp, modo_actual
    dec temp

    ; Cada imagen ocupa 8 bytes
    ; índice_imagen * 8
    lsl temp
    lsl temp
    lsl temp

    ; Sumar fila actual
    add temp, fila
    add YL, temp

    ; Leer patrón
    ld patron, Y

    ret


get_fila_mensaje:

    ; Buffer visible en SRAM 0x0170
    ldi YL, 0x70
    ldi YH, 0x01

    ; Sumar fila actual
    mov temp, fila
    add YL, temp

    ; Leer patrón de la fila
    ld patron, Y

    ret


;------------------------------------------------------------
; ISR TIMER0 - MULTIPLEXADO
;------------------------------------------------------------
isr_timer0:

    ; Guardar registro temporal
    push temp

    ; Guardar SREG
    in temp, SREG
    push temp

    ; Desactivar todas las filas

    sbi PORTD, PORTD4
    sbi PORTD, PORTD5
    sbi PORTD, PORTD6
    sbi PORTD, PORTD7

    sbi PORTB, PORTB0
    sbi PORTB, PORTB1
    sbi PORTB, PORTB2
    sbi PORTB, PORTB3

    ; Apagar todas las columnas

    cbi PORTB, PORTB4
    cbi PORTB, PORTB5

    clr temp
    out PORTC, temp

    ; Obtener patrón correspondiente a la fila

    call get_fila

    ; Columnas 1-2

    sbrc patron, 7
    sbi PORTB, PORTB4

    sbrc patron, 6
    sbi PORTB, PORTB5

    ; Columnas 3-8

    sbrc patron, 5
    sbi PORTC, PORTC0

    sbrc patron, 4
    sbi PORTC, PORTC1

    sbrc patron, 3
    sbi PORTC, PORTC2

    sbrc patron, 2
    sbi PORTC, PORTC3

    sbrc patron, 1
    sbi PORTC, PORTC4

    sbrc patron, 0
    sbi PORTC, PORTC5

    ; Activar fila correspondiente

    cpi fila, 0
    breq fila_1

    cpi fila, 1
    breq fila_2

    cpi fila, 2
    breq fila_3

    cpi fila, 3
    breq fila_4

    cpi fila, 4
    breq fila_5

    cpi fila, 5
    breq fila_6

    cpi fila, 6
    breq fila_7

    ; Fila 8
    cbi PORTB, PORTB3
    rjmp siguiente_fila


fila_1:

    cbi PORTD, PORTD4
    rjmp siguiente_fila


fila_2:

    cbi PORTD, PORTD5
    rjmp siguiente_fila


fila_3:

    cbi PORTD, PORTD6
    rjmp siguiente_fila


fila_4:

    cbi PORTD, PORTD7
    rjmp siguiente_fila


fila_5:

    cbi PORTB, PORTB0
    rjmp siguiente_fila


fila_6:

    cbi PORTB, PORTB1
    rjmp siguiente_fila


fila_7:

    cbi PORTB, PORTB2


siguiente_fila:

    inc fila

    cpi fila, 8
    brne revisar_scroll

    clr fila


revisar_scroll:

    ; Solo contar tiempo en modo mensaje
    tst modo_actual
    brne timer0_fin

    ; Timer0 interrumpe cada 1 ms
    inc contador_scroll

    ; Esperar el período seleccionado
    cp contador_scroll, periodo_scroll
    brlo timer0_fin

    ; Solicitar un nuevo paso de scroll
    clr contador_scroll
    clr scroll_pendiente
    inc scroll_pendiente


timer0_fin:

    ; Restaurar SREG
    pop temp
    out SREG, temp

    ; Restaurar registro temporal
    pop temp

    reti


;------------------------------------------------------------
; LIMPIAR BUFFER DEL MENSAJE
;------------------------------------------------------------
limpiar_buffer:

    ; Buffer visible desde SRAM 0x0170
    ldi XL, 0x70
    ldi XH, 0x01

    ; Ocho filas apagadas
    clr temp2
    ldi temp, 8

limpiar_buffer_loop:

    st X+, temp2

    dec temp
    brne limpiar_buffer_loop

    ret


;------------------------------------------------------------
; GUARDAR IMÁGENES EN SRAM
;------------------------------------------------------------
guardar_imagenes:

    ; Las seis imágenes comienzan en 0x0100
    ldi YL, 0x00
    ldi YH, 0x01

    ; Carita sonriendo

    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b00111100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp

    ; Carita guiñando

    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b00100000
    st Y+, temp
    ldi temp, 0b00101110
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b00111100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp

    ; Corazón

    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b01100110
    st Y+, temp
    ldi temp, 0b10011001
    st Y+, temp
    ldi temp, 0b10000001
    st Y+, temp
    ldi temp, 0b10000001
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00011000
    st Y+, temp

    ; :3

    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b01011010
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp

    ; Asterisco

    ldi temp, 0b10011001
    st Y+, temp
    ldi temp, 0b01011010
    st Y+, temp
    ldi temp, 0b00111100
    st Y+, temp
    ldi temp, 0b11111111
    st Y+, temp
    ldi temp, 0b11111111
    st Y+, temp
    ldi temp, 0b00111100
    st Y+, temp
    ldi temp, 0b01011010
    st Y+, temp
    ldi temp, 0b10011001
    st Y+, temp

    ; XD

    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00011000
    st Y+, temp
    ldi temp, 0b00100100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp
    ldi temp, 0b01111110
    st Y+, temp
    ldi temp, 0b01000010
    st Y+, temp
    ldi temp, 0b00111100
    st Y+, temp
    ldi temp, 0b00000000
    st Y+, temp

    ret

;------------------------------------------------------------
; GUARDAR FUENTE EN SRAM
;------------------------------------------------------------
; Cada carácter ocupa 5 bytes, uno por columna.
; Dirección inicial: 0x0130
;------------------------------------------------------------
guardar_fuente:

    ldi YL, 0x30
    ldi YH, 0x01


    ; F
    ldi temp, 0xFE
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x80
    st Y+, temp


    ; R
    ldi temp, 0xFE
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x98
    st Y+, temp
    ldi temp, 0x94
    st Y+, temp
    ldi temp, 0x62
    st Y+, temp


    ; A
    ldi temp, 0x7E
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x90
    st Y+, temp
    ldi temp, 0x7E
    st Y+, temp


    ; Y
    ldi temp, 0xC0
    st Y+, temp
    ldi temp, 0x20
    st Y+, temp
    ldi temp, 0x1E
    st Y+, temp
    ldi temp, 0x20
    st Y+, temp
    ldi temp, 0xC0
    st Y+, temp


    ; B
    ldi temp, 0xFE
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x6C
    st Y+, temp


    ; E
    ldi temp, 0xFE
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x82
    st Y+, temp


    ; N
    ldi temp, 0xFE
    st Y+, temp
    ldi temp, 0x20
    st Y+, temp
    ldi temp, 0x10
    st Y+, temp
    ldi temp, 0x08
    st Y+, temp
    ldi temp, 0xFE
    st Y+, temp


    ; T
    ldi temp, 0x80
    st Y+, temp
    ldi temp, 0x80
    st Y+, temp
    ldi temp, 0xFE
    st Y+, temp
    ldi temp, 0x80
    st Y+, temp
    ldi temp, 0x80
    st Y+, temp


    ; O
    ldi temp, 0x7C
    st Y+, temp
    ldi temp, 0x82
    st Y+, temp
    ldi temp, 0x82
    st Y+, temp
    ldi temp, 0x82
    st Y+, temp
    ldi temp, 0x7C
    st Y+, temp


    ; S
    ldi temp, 0x62
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x92
    st Y+, temp
    ldi temp, 0x8C
    st Y+, temp

    ret

;------------------------------------------------------------
; GUARDAR MENSAJE EN SRAM
;------------------------------------------------------------
; Dirección inicial: 0x0162
; Los valores 0-9 seleccionan caracteres de la fuente.
; 0xFF representa el espacio entre las dos palabras.
;------------------------------------------------------------
guardar_mensaje:

    ldi YL, 0x62
    ldi YH, 0x01

    ; F R A Y
    ldi temp, 0
    st Y+, temp

    ldi temp, 1
    st Y+, temp

    ldi temp, 2
    st Y+, temp

    ldi temp, 3
    st Y+, temp

    ; Espacio
    ldi temp, 0xFF
    st Y+, temp

    ; B E N T O S
    ldi temp, 4
    st Y+, temp

    ldi temp, 5
    st Y+, temp

    ldi temp, 6
    st Y+, temp

    ldi temp, 7
    st Y+, temp

    ldi temp, 8
    st Y+, temp

    ldi temp, 9
    st Y+, temp

    ret

;------------------------------------------------------------
; AVANZAR MENSAJE
;------------------------------------------------------------
avanzar_mensaje:

    ; indice_caracter = 11 indica que la frase terminó
    ; y se está vaciando completamente la matriz
    cpi indice_caracter, 11
    breq vaciado_final

    ; Mensaje desde SRAM 0x0162
    ldi XL, 0x62
    ldi XH, 0x01

    mov temp, indice_caracter
    add XL, temp

    ; temp2 = índice del carácter o 0xFF
    ld temp2, X

    ; 0xFF representa el espacio entre palabras
    cpi temp2, 0xFF
    breq espacio_palabras

    ; ---Carácter normal, 5 columnas
    cpi indice_columna, 5
    brlo obtener_columna

    ; ---Final de un carácter, vaciado final
    cpi indice_caracter, 10
    breq comenzar_vaciado

    ; Introducir una columna de separación
    clr temp2
    call desplazar_buffer

    ; Pasar al carácter siguiente
    clr indice_columna
    inc indice_caracter

    ret


;------------------------------------------------------------
; Obtener columna desde la fuente
;------------------------------------------------------------
obtener_columna:

    ; Fuente desde SRAM 0x0130
    ldi XL, 0x30
    ldi XH, 0x01

    ; offset = índice_caracter_fuente * 5
    mov temp, temp2

    ; *4
    lsl temp
    lsl temp

    ; +1 = *5
    add temp, temp2

    ; Sumar columna actual
    add temp, indice_columna

    ; Dirección final
    add XL, temp

    ; Leer nueva columna
    ld temp2, X

    ; Introducirla en la matriz
    call desplazar_buffer

    ; Siguiente columna del carácter
    inc indice_columna

    ret


;------------------------------------------------------------
; Espacio entre FRAY y BENTOS
;------------------------------------------------------------
espacio_palabras:

    ; Agregar otras dos columnas de separación 
    clr temp2
    call desplazar_buffer

    inc indice_columna

    cpi indice_columna, 2
    brlo avanzar_fin

    ; Continuar con la B
    clr indice_columna
    inc indice_caracter

    ret


;------------------------------------------------------------
; Comenzar vaciado final
;------------------------------------------------------------
comenzar_vaciado:

    ; Estado especial posterior a la S
    ldi indice_caracter, 11
    clr indice_columna


;------------------------------------------------------------
; Introducir 8 columnas apagadas
;------------------------------------------------------------
vaciado_final:

    clr temp2
    call desplazar_buffer

    inc indice_columna

    cpi indice_columna, 8
    brlo avanzar_fin

    ; Volver al comienzo de FRAY BENTOS
    clr indice_caracter
    clr indice_columna


avanzar_fin:

    ret

;------------------------------------------------------------
; DESPLAZAR BUFFER UNA COLUMNA
;------------------------------------------------------------
; Entrada:
;   temp2 = nueva columna
;------------------------------------------------------------
desplazar_buffer:

    ; Buffer visible desde SRAM 0x0170
    ldi XL, 0x70
    ldi XH, 0x01

    ; Recorrer las ocho filas
    ldi dato, 8


desplazar_buffer_loop:

    ; Leer fila actual
    ld temp, X

    ; Mover contenido hacia la izquierda
    lsl temp

    ; El bit 7 de temp2 corresponde
    ; a la nueva información de esta fila
    sbrc temp2, 7
    ori temp, 0b00000001

    ; Guardar fila actualizada
    st X+, temp

    ; Preparar bit de la fila siguiente
    lsl temp2

    dec dato
    brne desplazar_buffer_loop

    ret

;============================================================
; DATOS EN MEMORIA DE PROGRAMA
;============================================================


;------------------------------------------------------------
; Menú USART
;------------------------------------------------------------
menu:

    .db 13, 10, "PROBLEMA A - MATRIZ DE LEDS ", 13, 10
    .db "Bienvenido", 13, 10, 13, 10
    .db "1 - Mensaje Fray Bentos ", 13, 10
    .db "2 - Carita sonriendo", 13, 10
    .db "3 - Carita guinando ", 13, 10
    .db "4 - Corazon ", 13, 10
    .db "5 - :3", 13, 10
    .db "6 - Asterisco ", 13, 10
    .db "7 - XD", 13, 10
    .db "+ - Aumentar velocidad", 13, 10
    .db "- - Disminuir velocidad ", 13, 10
    .db "Seleccione: ", 0, 0