//============================================================
// Laboratorio 4 - Tecnolog�as de Microprocesamiento
// Grupo 7
// Problema C - Sistema de alarma
//============================================================

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


//------------------------------------------------------------
// Definiciones y constantes
//------------------------------------------------------------

#define LDR_ABIERTA_ADC 750U
#define LDR_CERRADA_ADC 650U
#define LDR_MUESTREO_MS 20UL
#define LDR_ESTABILIDAD_MS 100UL
#define BOTON_REBOTE_MS 30UL

#define RX_TAMANO 64U
#define TX_TAMANO 256U
#define COMANDO_MAX 32U

#define USAR_HC05_STATE 0

typedef enum {
  DESARMADO,
  ARMADO,
  ALARMADO,
  SILENCIADO
}
Estado;
typedef enum {
  AUTO,
  MANUAL
}
Modo;
typedef enum {
  NINGUNA,
  VENTANA,
  PRESENCIA,
  REMOTA
}
Zona;

//------------------------------------------------------------
// Variables globales
//------------------------------------------------------------

static Estado estado = DESARMADO;
static Modo modo = AUTO;
static Zona origen_alarma = NINGUNA;

static volatile uint32_t contador_ms = 0;

// Buffers circulares de recepci�n y transmisi�n USART.
static volatile uint8_t rx_datos[RX_TAMANO];
static volatile uint8_t rx_escritura = 0, rx_lectura = 0;
static volatile uint8_t rx_error = 0;
static volatile uint8_t tx_datos[TX_TAMANO];
static volatile uint8_t tx_escritura = 0, tx_lectura = 0;
static volatile uint8_t tx_descartadas = 0;

// Datos de sensores y antirrebote.
static bool ventana_abierta = false;
static bool ldr_lista = false;
static uint16_t ldr_adc = 0;
static uint16_t ldr_suma = 0;
static uint8_t ldr_muestras = 0;
static bool ldr_candidata = false;
static bool ldr_candidata_inicializada = false;
static uint32_t ldr_cambio_ms = 0;
static uint32_t ldr_ultima_muestra_ms = 0;

static bool presencia = false;
static bool boton_lectura_anterior = false;
static uint32_t boton_ultimo_cambio_ms = 0;
static bool boton_listo = false;

#if USAR_HC05_STATE
static bool enlace_anterior = false;
#endif

//------------------------------------------------------------
// Prototipos de funciones
//------------------------------------------------------------

void configurar_gpio(void);

static void actualizar_salidas(void);
static void uart_iniciar(void);
static void timer1_iniciar(void);
static void timer2_buzzer_iniciar(void);
static void adc_iniciar(void);
static uint32_t tiempo_ms(void);
static
const char * texto_estado(Estado valor);
static
const char * texto_zona(Zona zona);
static bool zonas_libres(void);
static bool uart_recibir_byte(uint8_t * dato);
static bool uart_linea(const char * texto);
static void respuesta_error(const char * comando,
  const char * motivo);
static void respuesta_ok(const char * comando);
static void informar_estado(void);
static uint16_t adc0_leer(void);
static void notificar_deteccion(Zona zona);
static void activar_alarma(Zona zona);
static void zona_activada(Zona zona);
static void ldr_procesar(uint32_t ahora);
static void pulsador_procesar(uint32_t ahora);
static void procesar_comando(const char * comando);
static void comandos_procesar(void);
#if USAR_HC05_STATE
static void enlace_procesar(void);
#endif

//------------------------------------------------------------
// Programa principal
//------------------------------------------------------------

int main(void) {
  // Configuraci�n inicial
  configurar_gpio();
  timer2_buzzer_iniciar();
  actualizar_salidas();
  adc_iniciar();
  uart_iniciar();
  timer1_iniciar();
  boton_lectura_anterior = !(PIND & (1U << PIND2));
  sei();

  while (1) {
    uint32_t ahora = tiempo_ms();

    // Priorizar los comandos remotos respecto a nuevas detecciones.
    comandos_procesar();
    pulsador_procesar(ahora);
    ldr_procesar(ahora);

    // En AUTO, una zona activa dispara la alarma desde ARMADO.
    if (estado == ARMADO && modo == AUTO && (ventana_abierta || presencia)) {
      activar_alarma(ventana_abierta ? VENTANA : PRESENCIA);
    }

    #if USAR_HC05_STATE
    enlace_procesar();
    #endif
  }

  return 0;
}

//------------------------------------------------------------
// CONFIGURACI�N GPIO
//------------------------------------------------------------

void configurar_gpio(void) {
  // LEDs en D8/D9
  DDRB |= (1U << DDB0) | (1U << DDB1);
  PORTB &= (uint8_t) ~((1U << PORTB0) | (1U << PORTB1));

  // Buzzer pasivo con Timer2 PWM en OC2B (D3); boton en D2 con pull-up. 
  DDRD |= (1U << DDD3);
  PORTD &= (uint8_t) ~(1U << PORTD3);
  DDRD &= (uint8_t) ~(1U << DDD2);
  PORTD |= (1U << PORTD2);

  #if USAR_HC05_STATE
  DDRD &= (uint8_t) ~(1U << DDD4);
  PORTD &= (uint8_t) ~(1U << PORTD4);
  #endif

  // A0 como entrada ADC sin pull-up. 
  DDRC &= (uint8_t) ~(1U << DDC0);
  PORTC &= (uint8_t) ~(1U << PORTC0);
  DIDR0 |= (1U << ADC0D);
}

//------------------------------------------------------------
// FUNCIONES AUXILIARES
//------------------------------------------------------------

static void actualizar_salidas(void) {
  bool led_verde = (estado != DESARMADO);
  bool led_rojo = (estado == ALARMADO || estado == SILENCIADO);
  bool buzzer = (estado == ALARMADO);

  if (led_verde) PORTB |= (1U << PORTB0);
  else PORTB &= (uint8_t) ~(1U << PORTB0);

  if (led_rojo) PORTB |= (1U << PORTB1);
  else PORTB &= (uint8_t) ~(1U << PORTB1);

  // Moore: el estado habilita o deshabilita el tono de Timer2 en OC2B.
  if (buzzer) {
    TCCR2A |= (1U << COM2B1); // PWM no inversor en PD3/OC2B.
  } else {
    TCCR2A &= (uint8_t) ~(1U << COM2B1); // Desconectar OC2B.
    PORTD &= (uint8_t) ~(1U << PORTD3); // Mantener salida baja.
  }
}

static void uart_iniciar(void) {
  // 16 MHz / (16 * (103+1)) = 9615 baudios
  UCSR0A = 0; // Velocidad normal, no U2X0.
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0C = (1U << UCSZ01) | (1U << UCSZ00); // 8N1.
  UCSR0B = (1U << RXEN0) | (1U << TXEN0) | (1U << RXCIE0);
}

static void timer1_iniciar(void) {
  // CTC, 16 MHz / 64 / (249+1) = 1000 Hz = 1 ms.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 249;
  TCCR1B = (1U << WGM12) | (1U << CS11) | (1U << CS10);
  TIMSK1 |= (1U << OCIE1A);
}

static void timer2_buzzer_iniciar(void) {
  // Buzzer pasivo: Timer2 Fast PWM
  // 16 MHz / (64 * (124 + 1)) = 2000 Hz; duty ~50 %.
  // COM2B1 permanece desactivado hasta el estado ALARMADO.
  TCCR2A = (1U << WGM21) | (1U << WGM20);
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 124U;
  OCR2B = 62U;
  TCCR2B = (1U << WGM22) | (1U << CS22); // Fast PWM, /64.
}

static void adc_iniciar(void) {
  // AVcc = referencia, canal ADC0, prescaler 128: ADC a 125 kHz.
  ADMUX = (1U << REFS0);
  ADCSRA = (1U << ADEN) | (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);
}

static uint32_t tiempo_ms(void) {
  uint32_t copia;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    copia = contador_ms;
  }
  return copia;
}

static
const char * texto_estado(Estado valor) {
  switch (valor) {
  case DESARMADO:
    return "DESARMADO";
  case ARMADO:
    return "ARMADO";
  case ALARMADO:
    return "ALARMADO";
  case SILENCIADO:
    return "SILENCIADO";
  default:
    return "DESCONOCIDO";
  }
}

static
const char * texto_zona(Zona zona) {
  switch (zona) {
  case VENTANA:
    return "VENTANA";
  case PRESENCIA:
    return "PRESENCIA";
  case REMOTA:
    return "REMOTA";
  default:
    return "NINGUNA";
  }
}

static bool zonas_libres(void) {
  return ldr_lista && boton_listo && !ventana_abierta && !presencia;
}

static bool uart_recibir_byte(uint8_t * dato) {
  if (rx_lectura == rx_escritura) return false;
  * dato = rx_datos[rx_lectura];
  rx_lectura = (uint8_t)((rx_lectura + 1U) & (RX_TAMANO - 1U));
  return true;
}

static bool uart_linea(const char * texto) {
  size_t n = strlen(texto);
  bool enviado = false;
  if (n > (TX_TAMANO - 3U)) return false;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    uint8_t libres = (uint8_t)(tx_lectura - tx_escritura - 1U);
    if ((n + 2U) <= libres) {
      for (size_t i = 0; i < n; ++i) {
        tx_datos[tx_escritura++] = (uint8_t) texto[i];
      }
      tx_datos[tx_escritura++] = '\r';
      tx_datos[tx_escritura++] = '\n';
      UCSR0B |= (1U << UDRIE0);
      enviado = true;
    } else {
      if (tx_descartadas < 255U) tx_descartadas++;
    }
  }
  return enviado;
}

static void respuesta_error(const char * comando,
  const char * motivo) {
  char linea[90];
  (void) snprintf(linea, sizeof(linea), "ERROR;CMD:%s;MOTIVO:%s", comando, motivo);
  (void) uart_linea(linea);
}

static void respuesta_ok(const char * comando) {
  char linea[90];
  (void) snprintf(linea, sizeof(linea), "OK;CMD:%s;ESTADO:%s", comando, texto_estado(estado));
  (void) uart_linea(linea);
}

static void informar_estado(void) {
  char linea[125];
  (void) snprintf(linea, sizeof(linea),
    "ESTADO:%s;MODO:%s;VENTANA:%s;PRESENCIA:%s;ORIGEN:%s;ADC:%u",
    texto_estado(estado), (modo == AUTO) ? "AUTO" : "MANUAL",
    (!ldr_lista) ? "SIN_LECTURA" : (ventana_abierta ? "ABIERTA" : "CERRADA"),
    presencia ? "SI" : "NO", texto_zona(origen_alarma), (unsigned) ldr_adc);
  (void) uart_linea(linea);
}

static uint16_t adc0_leer(void) {
  ADCSRA |= (1U << ADSC);
  while (ADCSRA & (1U << ADSC)) {
    // Conversion corta, del orden de 0.1 ms.
  }
  return ADC;
}

static void notificar_deteccion(Zona zona) {
  char linea[65];
  (void) snprintf(linea, sizeof(linea), "EVENTO:DETECCION;ZONA:%s", texto_zona(zona));
  (void) uart_linea(linea);
}

static void activar_alarma(Zona zona) {
  char linea[65];
  origen_alarma = zona;
  estado = ALARMADO;
  actualizar_salidas();
  (void) snprintf(linea, sizeof(linea), "ALARMA:ON;ZONA:%s", texto_zona(zona));
  (void) uart_linea(linea);
}

static void zona_activada(Zona zona) {
  if (estado == ARMADO) {
    if (modo == AUTO) activar_alarma(zona);
    else notificar_deteccion(zona);
  } else if (estado == ALARMADO || estado == SILENCIADO) {
    // Notificar nueva zona, sin volver a disparar la sirena.
    notificar_deteccion(zona);
  }
}

static void ldr_procesar(uint32_t ahora) {
  bool valor_candidato;
  if ((uint32_t)(ahora - ldr_ultima_muestra_ms) < LDR_MUESTREO_MS) return;
  ldr_ultima_muestra_ms = ahora;
  ldr_suma += adc0_leer();
  ldr_muestras++;
  if (ldr_muestras < 4U) return;

  ldr_adc = (uint16_t)(ldr_suma / 4U);
  ldr_suma = 0;
  ldr_muestras = 0;

  // Dos umbrales; el intervalo central conserva el candidato previo.
  if (ldr_adc >= LDR_ABIERTA_ADC) valor_candidato = true;
  else if (ldr_adc <= LDR_CERRADA_ADC) valor_candidato = false;
  else return;

  if (!ldr_lista) {
    // Validar tambien la primera condicion por 100 ms antes de armar.
    if (!ldr_candidata_inicializada || valor_candidato != ldr_candidata) {
      ldr_candidata = valor_candidato;
      ldr_cambio_ms = ahora;
      ldr_candidata_inicializada = true;
    } else if ((uint32_t)(ahora - ldr_cambio_ms) >= LDR_ESTABILIDAD_MS) {
      ventana_abierta = ldr_candidata;
      ldr_lista = true;
    }
    return;
  }

  if (valor_candidato != ldr_candidata) {
    ldr_candidata = valor_candidato;
    ldr_cambio_ms = ahora;
  }
  if (ventana_abierta != ldr_candidata &&
    (uint32_t)(ahora - ldr_cambio_ms) >= LDR_ESTABILIDAD_MS) {
    ventana_abierta = ldr_candidata;
    if (ventana_abierta) zona_activada(VENTANA);
  }
}

static void pulsador_procesar(uint32_t ahora) {
  bool lectura = !(PIND & (1U << PIND2)); // LOW = presionado.
  if (lectura != boton_lectura_anterior) {
    boton_lectura_anterior = lectura;
    boton_ultimo_cambio_ms = ahora;
  }
  if ((uint32_t)(ahora - boton_ultimo_cambio_ms) >= BOTON_REBOTE_MS) {
    boton_listo = true;
    if (presencia != lectura) {
      presencia = lectura;
      if (presencia) zona_activada(PRESENCIA);
    }
  }
}

static void procesar_comando(const char * comando) {
  if (strcmp(comando, "PING") == 0) {
    (void) uart_linea("PONG");
  } else if (strcmp(comando, "ESTADO?") == 0) {
    informar_estado();
  } else if (strcmp(comando, "MODO:AUTO") == 0) {
    modo = AUTO;
    respuesta_ok(comando);
    if (estado == ARMADO && (ventana_abierta || presencia)) {
      activar_alarma(ventana_abierta ? VENTANA : PRESENCIA);
    }
  } else if (strcmp(comando, "MODO:MANUAL") == 0) {
    modo = MANUAL;
    respuesta_ok(comando);
  } else if (strcmp(comando, "DESARMAR") == 0) {
    estado = DESARMADO;
    origen_alarma = NINGUNA;
    actualizar_salidas();
    respuesta_ok(comando);
  } else if (strcmp(comando, "ARMAR") == 0) {
    if (estado != DESARMADO && estado != SILENCIADO) {
      respuesta_error(comando, "ESTADO_INVALIDO");
    } else if (!ldr_lista || !boton_listo) {
      respuesta_error(comando, "SENSORES_NO_LISTOS");
    } else if (!zonas_libres()) {
      respuesta_error(comando, "ZONA_ACTIVA");
    } else {
      estado = ARMADO;
      origen_alarma = NINGUNA;
      actualizar_salidas();
      respuesta_ok(comando);
    }
  } else if (strcmp(comando, "ALARMA:ON") == 0) {
    if (estado != ARMADO || modo != MANUAL) {
      respuesta_error(comando, "ESTADO_O_MODO_INVALIDO");
    } else {
      activar_alarma(REMOTA);
      respuesta_ok(comando);
    }
  } else if (strcmp(comando, "SILENCIAR") == 0) {
    if (estado != ALARMADO) {
      respuesta_error(comando, "ESTADO_INVALIDO");
    } else {
      estado = SILENCIADO;
      actualizar_salidas();
      respuesta_ok(comando);
      (void) uart_linea("ALARMA:SILENCIADA");
    }
  } else {
    respuesta_error("DESCONOCIDO", "COMANDO_INVALIDO");
  }
}

static void comandos_procesar(void) {
  static char linea[COMANDO_MAX + 1U];
  static uint8_t longitud = 0;
  static bool descartar = false;
  uint8_t dato;
  uint8_t atendidos = 0;

  if (rx_error) {
    // Descarta hasta LF para no ejecutar fragmentos de una trama rota.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      rx_lectura = rx_escritura;
      rx_error = 0;
    }
    longitud = 0;
    descartar = true;
    respuesta_error("UART", "RECEPCION");
  }

  // Limite de trabajo por ciclo, para no desatender los sensores.
  while (atendidos++ < RX_TAMANO && uart_recibir_byte( & dato)) {
    if (dato == '\r') continue; // Acepta LF y CRLF.
    if (dato == '\n') {
      if (!descartar && longitud != 0U) {
        linea[longitud] = '\0';
        procesar_comando(linea);
      }
      descartar = false;
      longitud = 0;
    } else if (!descartar) {
      if (longitud < COMANDO_MAX) {
        linea[longitud++] = (char) dato;
      } else {
        longitud = 0;
        descartar = true;
        respuesta_error("UART", "TRAMA_LARGA");
      }
    }
  }
}

#if USAR_HC05_STATE
static void enlace_procesar(void) {
  bool enlace = !!(PIND & (1U << PIND4));
  if (enlace != enlace_anterior) {
    enlace_anterior = enlace;
    if (enlace) informar_estado(); // Recuperar estado al reconectar.
  }
}
#endif

//------------------------------------------------------------
// INTERRUPCIONES
//------------------------------------------------------------

ISR(TIMER1_COMPA_vect) {
  contador_ms++;
}

ISR(USART_RX_vect) {
  uint8_t estado_uart = UCSR0A;
  uint8_t dato = UDR0;
  uint8_t siguiente;

  if (estado_uart & ((1U << FE0) | (1U << DOR0) | (1U << UPE0))) {
    rx_error = 1;
    return;
  }
  siguiente = (uint8_t)((rx_escritura + 1U) & (RX_TAMANO - 1U));
  if (siguiente == rx_lectura) {
    rx_error = 1; // Buffer lleno: se descarta la trama completa.
    return;
  }
  rx_datos[rx_escritura] = dato;
  rx_escritura = siguiente;
}

ISR(USART_UDRE_vect) {
  if (tx_lectura == tx_escritura) {
    UCSR0B &= (uint8_t) ~(1U << UDRIE0);
  } else {
    UDR0 = tx_datos[tx_lectura];
    tx_lectura++; // modulo 256 por overflow natural
  }
}