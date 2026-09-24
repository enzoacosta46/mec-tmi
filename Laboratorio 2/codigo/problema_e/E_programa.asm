

.include "m328pdef.inc"

.def temp     = r16
.def estado   = r17

.equ P_CERRADA          = 0        
.equ P_ABRIENDO         = 1
.equ P_ABIERTA          = 2
.equ P_CERRANDO         = 3
.equ PARADA_EMERGENCIA  = 4

.equ MOTOR_A  = PB2                
.equ MOTOR_C  = PB3
.equ ALARMA   = PB4

.cseg
.org 0x0000
   rjmp RESET

.org PCINT0addr
   rjmp ISR_OBSTACULO

RESET:
   
   ldi temp, low(RAMEND)
   out SPL, temp
   ldi temp, high(RAMEND)
   out SPH, temp

   ldi temp, (1<<MOTOR_A)|(1<<MOTOR_C)|(1<<ALARMA)
   out DDRB, temp

   cbi PORTB, MOTOR_A
   cbi PORTB, MOTOR_C
   cbi PORTB, ALARMA 

   cbi DDRB,  PB0 
   sbi PORTB, PB0

   ldi temp, (1<<PCIE0)
   sts PCICR, temp

   ldi temp, (1<<PCINT0)
   sts PCMSK0, temp

   ldi temp, 103
   sts UBRR0L, temp
   ldi temp, 0
   sts UBRR0H, temp

   ldi temp, (1<<TXEN0)
   sts UCSR0B, temp

   ldi temp, (1<<UCSZ01)|(1<<UCSZ00)
   sts UCSR0C, temp

   ldi estado, P_CERRADA
   sei

MAIN_LOOP:
 
   lds temp, estado

   cpi temp, P_CERRADA
   breq EST_CERRADA_FUNCA

   cpi temp, P_ABRIENDO
   breq EST_ABRIENDO_FUNCA

   cpi temp, P_ABIERTA
   breq EST_ABIERTA_FUNCA

   cpi temp, P_CERRANDO
   breq EST_CERRANDO_FUNCA

   cpi temp, PARADA_EMERGENCIA
   breq EST_EMERGENCIA_FUNCA

   rjmp MAIN_LOOP


EST_CERRADA_FUNCA:
   
   cbi PORTB, MOTOR_A
   cbi PORTB, MOTOR_C
   cbi PORTB, ALARMA

   rjmp MAIN_LOOP

EST_ABRIENDO_FUNCA:

   sbi PORTB, MOTOR_A
   cbi PORTB, MOTOR_C
   sbi PORTB, ALARMA

   rjmp MAIN_LOOP

EST_ABIERTA_FUNCA:

   cbi PORTB, MOTOR_A
   cbi PORTB, MOTOR_C
   cbi PORTB, ALARMA

   rjmp MAIN_LOOP

EST_CERRANDO_FUNCA:

   cbi PORTB, MOTOR_A
   sbi PORTB, MOTOR_C
   sbi PORTB, ALARMA

   rjmp MAIN_LOOP

EST_EMERGENCIA_FUNCA:

   cbi PORTB, MOTOR_A
   cbi PORTB, MOTOR_C
   cbi PORTB, ALARMA

   rjmp MAIN_LOOP


MENSAJE_USART:
   
   lds temp, UCSR0A
   sbrs temp, UDRE0
   rjmp MENSAJE_USART
   sts UDR0, r16 
   ret

CADENA_USART:
    lpm r16, Z+                
    cpi r16, 0                 
    breq FINAL_MENSAJE
    rcall MENSAJE_USART
    rjmp CADENA_USART
FINAL_MENSAJE:
    ret

M_ABRIENDO:   .db "Puerta abriendo", 13, 10, 0
M_ABIERTA:    .db "Puerta abierta", 13, 10, 0
M_CERRANDO:   .db "Puerta cerrando", 13, 10, 0
M_CERRADA:    .db "Puerta cerrada", 13, 10, 0
M_OBSTACULO:  .db "Obstaculo detectado", 13, 10, 0
M_SEGURIDAD:  .db "Movimiento detenido por seguridad", 13, 10, 0

ISR_OBSTACULO:
    
    in temp, SREG
    push temp
    push r16
    push r30
    push r31

	cbi PORTB, MOTOR_A
    cbi PORTB, MOTOR_C
    cbi PORTB, ALARMA

	ldi temp, PARADA_EMERGENCIA
    sts estado, temp

	ldi r30, low(M_OBSTACULO*2)
    ldi r31, high(M_OBSTACULO*2)
    rcall CADENA_USART

	ldi r30, low(M_SEGURIDAD*2)
    ldi r31, high(M_SEGURIDAD*2)
    rcall CADENA_USART

	pop r31
    pop r30
    pop r16
    pop temp
    out SREG, temp
    reti
