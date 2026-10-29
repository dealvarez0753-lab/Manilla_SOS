// a7670sa.chip.c
// Emulador simplificado del modulo SIMCom A7670SA (LTE Cat 1 + GNSS) para Wokwi.
// Responde por UART a los comandos AT que usa el firmware de la manilla:
//   AT, ATE0, AT+CPIN?, AT+CMGF=1, AT+CSQ, AT+CEREG?,
//   AT+CGNSSPWR=1, AT+CGNSSINFO, AT+CMGS="<numero>"
// Los SMS "entregados" se muestran en la pestana Chips Console.

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINEA 200
#define MAX_SALIDA 512

typedef struct {
  uart_dev_t uart;
  uint32_t attr_senal;
  uint32_t attr_fix;
  uint32_t attr_demora;
  uint32_t attr_lat;
  uint32_t attr_lon;

  char linea[MAX_LINEA];
  int linea_len;

  bool modo_sms;
  char sms_destino[32];
  char sms_texto[MAX_LINEA];
  int sms_len;
  int sms_contador;

  char cola[MAX_SALIDA];   // datos pendientes por transmitir
  int cola_len;
  char tx[MAX_SALIDA];     // datos en transmision
  bool ocupado;

  timer_t temporizador;
  char diferido[MAX_SALIDA];  // respuesta que se envia con retardo
  bool gnss_encendido;
} chip_state_t;

// ---------- Transmision por UART con cola ----------

static void transmitir(chip_state_t *c) {
  if (c->ocupado || c->cola_len == 0) return;
  int n = c->cola_len;
  memcpy(c->tx, c->cola, n);
  c->cola_len = 0;
  c->ocupado = true;
  if (!uart_write(c->uart, (uint8_t *)c->tx, n)) {
    c->ocupado = false;
  }
}

static void responder(chip_state_t *c, const char *s) {
  int n = strlen(s);
  if (c->cola_len + n > MAX_SALIDA) n = MAX_SALIDA - c->cola_len;
  memcpy(c->cola + c->cola_len, s, n);
  c->cola_len += n;
  transmitir(c);
}

static void on_uart_write_done(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  c->ocupado = false;
  transmitir(c);
}

// ---------- Respuestas con retardo (simulan tiempos de red / GNSS) ----------

static void on_temporizador(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  if (c->diferido[0]) {
    responder(c, c->diferido);
    c->diferido[0] = 0;
  }
}

static void responder_despues(chip_state_t *c, const char *s, uint32_t ms) {
  strncat(c->diferido, s, MAX_SALIDA - strlen(c->diferido) - 1);
  timer_start(c->temporizador, ms * 1000, false);
}

// ---------- GNSS: grados decimales -> formato ddmm.mmmmmm ----------

static void a_nmea(double v, bool es_longitud, char *salida, char *hemisferio) {
  double a = v < 0 ? -v : v;
  int grados = (int)a;
  double minutos = (a - grados) * 60.0;
  if (es_longitud) {
    sprintf(salida, "%03d%09.6f", grados, minutos);
    *hemisferio = v < 0 ? 'W' : 'E';
  } else {
    sprintf(salida, "%02d%09.6f", grados, minutos);
    *hemisferio = v < 0 ? 'S' : 'N';
  }
}

// ---------- Interprete de comandos AT ----------

static void procesar_comando(chip_state_t *c, const char *cmd) {
  int senal = (int)attr_read(c->attr_senal);
  char buf[MAX_SALIDA];

  if (strcmp(cmd, "AT") == 0 || strcmp(cmd, "ATE0") == 0 || strcmp(cmd, "AT+CMGF=1") == 0) {
    responder(c, "\r\nOK\r\n");

  } else if (strcmp(cmd, "AT+CPIN?") == 0) {
    responder(c, "\r\n+CPIN: READY\r\n\r\nOK\r\n");

  } else if (strcmp(cmd, "AT+CSQ") == 0) {
    sprintf(buf, "\r\n+CSQ: %d,99\r\n\r\nOK\r\n", senal);
    responder(c, buf);

  } else if (strcmp(cmd, "AT+CEREG?") == 0) {
    sprintf(buf, "\r\n+CEREG: 0,%d\r\n\r\nOK\r\n", senal > 0 ? 1 : 2);
    responder(c, buf);

  } else if (strcmp(cmd, "AT+CGNSSPWR=1") == 0) {
    c->gnss_encendido = true;
    responder(c, "\r\nOK\r\n");
    responder_despues(c, "\r\n+CGNSSPWR: READY!\r\n", 1000);

  } else if (strcmp(cmd, "AT+CGNSSINFO") == 0) {
    if (c->gnss_encendido && attr_read(c->attr_fix) == 1) {
      char lat[20], lon[20], ns, ew;
      a_nmea(attr_read_float(c->attr_lat), false, lat, &ns);
      a_nmea(attr_read_float(c->attr_lon), true, lon, &ew);
      // <modo>,<sat GPS>,<sat GLONASS>,<sat BEIDOU>,<lat>,<N/S>,<lon>,<E/W>,
      // <fecha>,<hora UTC>,<altitud>,<velocidad>,<rumbo>,<PDOP>,<HDOP>,<VDOP>
      sprintf(buf, "\r\n+CGNSSINFO: 2,08,04,00,%s,%c,%s,%c,121026,173000.0,2527.0,0.0,0.0,1.3,0.9,0.9\r\n\r\nOK\r\n",
              lat, ns, lon, ew);
      responder(c, buf);
    } else {
      responder(c, "\r\n+CGNSSINFO: ,,,,,,,,,,,,,,,\r\n\r\nOK\r\n");
    }

  } else if (strncmp(cmd, "AT+CMGS=\"", 9) == 0) {
    if (senal == 0) {
      responder(c, "\r\n+CMS ERROR: 331\r\n");  // sin servicio de red
      return;
    }
    const char *ini = cmd + 9;
    const char *fin = strchr(ini, '"');
    int n = fin ? (int)(fin - ini) : (int)strlen(ini);
    if (n > 31) n = 31;
    memcpy(c->sms_destino, ini, n);
    c->sms_destino[n] = 0;
    c->modo_sms = true;
    c->sms_len = 0;
    responder(c, "\r\n> ");

  } else {
    responder(c, "\r\nERROR\r\n");
  }
}

static void on_uart_rx_data(void *user_data, uint8_t byte) {
  chip_state_t *c = (chip_state_t *)user_data;

  if (c->modo_sms) {
    if (byte == 26) {  // Ctrl+Z: enviar
      c->sms_texto[c->sms_len] = 0;
      c->modo_sms = false;
      c->sms_contador++;
      printf("[RED CELULAR] SMS #%d entregado a %s: %s\n", c->sms_contador, c->sms_destino, c->sms_texto);
      char buf[64];
      sprintf(buf, "\r\n+CMGS: %d\r\n\r\nOK\r\n", c->sms_contador);
      responder_despues(c, buf, attr_read(c->attr_demora));
    } else if (byte == 27) {  // ESC: cancelar
      c->modo_sms = false;
      responder(c, "\r\nOK\r\n");
    } else if (c->sms_len < MAX_LINEA - 1) {
      c->sms_texto[c->sms_len++] = (char)byte;
    }
    return;
  }

  if (byte == '\r' || byte == '\n') {
    if (c->linea_len > 0) {
      c->linea[c->linea_len] = 0;
      c->linea_len = 0;
      procesar_comando(c, c->linea);
    }
  } else if (c->linea_len < MAX_LINEA - 1) {
    c->linea[c->linea_len++] = (char)byte;
  }
}

// ---------- Inicializacion ----------

void chip_init(void) {
  chip_state_t *c = malloc(sizeof(chip_state_t));
  memset(c, 0, sizeof(chip_state_t));

  c->attr_senal  = attr_init("signalQuality", 20);
  c->attr_fix    = attr_init("gnssFix", 1);
  c->attr_demora = attr_init("networkDelayMs", 1500);
  c->attr_lat    = attr_init_float("latitude", 1.2136f);    // Pasto (aprox.)
  c->attr_lon    = attr_init_float("longitude", -77.2811f);

  const timer_config_t cfg_timer = {
    .callback = on_temporizador,
    .user_data = c,
  };
  c->temporizador = timer_init(&cfg_timer);

  const uart_config_t cfg_uart = {
    .tx = pin_init("TX", INPUT_PULLUP),
    .rx = pin_init("RX", INPUT),
    .baud_rate = 115200,
    .rx_data = on_uart_rx_data,
    .write_done = on_uart_write_done,
    .user_data = c,
  };
  c->uart = uart_init(&cfg_uart);

  printf("A7670SA emulado listo (LTE Cat 1 + GNSS)\n");
}