

.include "m328pdef.inc"

.def temp     = r16
.def estado   = r17

.equ P_CERRADA          = 0        
.equ P_ABRIENDO         = 1
.equ P_ABIERTA          = 2
.equ P_CERRANDO         = 3
.equ PARADA_EMERGENCIA  = 4

.equ MOTOR_A  = PB2                
.equ MOTOR_B  = PB3
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
   cbi PORTB, MOTOR_B
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
   sts UCSR0C, TEMP

   ldi temp, (1<<UCSZ01)|(1<<UCSZ00)
   sts UCSR0C, temp

   ldi estado, P_CERRADA
   sei

MAIN_LOOP:
