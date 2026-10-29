/*
  Manilla IoT - Sistema de Alerta de Emergencia Portatil
  Universidad CESMAG - Soluciones IoT 2026

  Simulacion en Wokwi:
    ESP32 DevKit + emulador del modulo SIMCom A7670SA (LTE Cat 1 + GNSS)
    El emulador es un chip personalizado (a7670sa.chip.c) que responde
    a los mismos comandos AT que usa el modulo real.

  Funcionamiento:
    1. La usuaria mantiene presionado el boton SOS durante 3 segundos.
    2. El ESP32 pide la ubicacion al GNSS del modulo (AT+CGNSSINFO).
    3. Si no hay posicion (p. ej. bajo techo) usa la ultima conocida.
    4. Envia un SMS con enlace de Google Maps a cada contacto (AT+CMGS).
    5. Vibra 3 veces para confirmar el envio de forma silenciosa.
*/

#include <Arduino.h>

// ---------------- CAPA FISICA: pines ----------------
const int PIN_BOTON   = 4;    // Pulsador SOS conectado a GND (INPUT_PULLUP)
const int PIN_MOTOR   = 25;   // Motor de vibracion (LED naranja en la simulacion)
const int PIN_LED_RED = 26;   // Indicador de registro en red celular (prototipo)
const int MODEM_RX    = 16;   // RX del ESP32  <- TX del modulo
const int MODEM_TX    = 17;   // TX del ESP32  -> RX del modulo

HardwareSerial modem(2);

// ---------------- CONFIGURACION ----------------
const char* CONTACTOS[] = { "+573122011725", "+573227408912" };
const int NUM_CONTACTOS = sizeof(CONTACTOS) / sizeof(CONTACTOS[0]);

const unsigned long T_PULSACION_SOS = 3000;   // ms sostenido para activar la alerta
const unsigned long T_REBOTE        = 50;     // ms para descartar rebotes del boton
const unsigned long T_ESPERA_GNSS   = 10000;  // ms maximo buscando posicion al activar
const unsigned long T_REFRESCO_GNSS = 30000;  // ms entre lecturas de ubicacion en reposo
const int REINTENTOS_SMS = 3;

// ---------------- CAPA LOGICA: estados ----------------
enum Estado { REPOSO, ACTIVANDO, UBICANDO, ENVIANDO, CONFIRMADO, FALLO };
Estado estado = REPOSO;

double ultLat = 0, ultLon = 0;
bool hayUbicacion = false;
unsigned long tUltimaUbicacion = 0;

unsigned long tPresion = 0;
unsigned long tSuelto = 0;
unsigned long tActivacion = 0;
unsigned long tUltimoRefresco = 0;
bool ubicacionFresca = false;

// ======================================================
//               CAPA DE RED: comandos AT
// ======================================================

void limpiarEntrada() {
  while (modem.available()) modem.read();
}

// Espera hasta que aparezca 'token' (o "ERROR") en la respuesta del modulo
String esperar(const char* token, unsigned long timeout) {
  String r;
  unsigned long t0 = millis();
  while (millis() - t0 < timeout) {
    while (modem.available()) r += (char)modem.read();
    if (r.indexOf(token) >= 0 || r.indexOf("ERROR") >= 0) break;
    delay(5);
  }
  return r;
}

String enviarAT(const String& cmd, unsigned long timeout = 2000) {
  limpiarEntrada();
  modem.print(cmd);
  modem.print("\r");
  String r = esperar("\r\nOK\r\n", timeout);
  Serial.print("[RED] ");
  Serial.print(cmd);
  Serial.print(" -> ");
  String limpio = r;
  limpio.replace("\r\n", " ");
  limpio.trim();
  Serial.println(limpio.length() ? limpio : "(sin respuesta)");
  return r;
}

bool redRegistrada() {
  String r = enviarAT("AT+CEREG?");
  // ,1 = registrada en red local ; ,5 = registrada en roaming
  return r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0;
}

bool enviarSMS(const char* numero, const String& texto) {
  limpiarEntrada();
  modem.print("AT+CMGS=\"");
  modem.print(numero);
  modem.print("\"\r");
  String r = esperar(">", 5000);
  if (r.indexOf(">") < 0) {
    Serial.println("[RED] El modulo no acepto el SMS (sin senal o sin SIM).");
    return false;
  }
  modem.print(texto);
  modem.write(26);  // Ctrl+Z: fin del mensaje
  r = esperar("\r\nOK\r\n", 15000);
  return r.indexOf("+CMGS:") >= 0 && r.indexOf("OK") >= 0;
}

void iniciarModem() {
  Serial.println("[RED] Iniciando modulo A7670SA...");
  for (int i = 0; i < 10; i++) {
    if (enviarAT("AT").indexOf("OK") >= 0) break;
    delay(500);
  }
  enviarAT("ATE0");            // sin eco de comandos
  enviarAT("AT+CPIN?");        // estado de la SIM
  enviarAT("AT+CMGF=1");       // SMS en modo texto
  enviarAT("AT+CGNSSPWR=1");   // encender GNSS
  enviarAT("AT+CSQ");          // calidad de senal
  digitalWrite(PIN_LED_RED, redRegistrada() ? HIGH : LOW);
}

// ======================================================
//            CAPA DE PROCESO: datos de ubicacion
// ======================================================

// Convierte formato NMEA ddmm.mmmm (o dddmm.mmmm) a grados decimales
double nmeaADecimal(double v, bool negativo) {
  int grados = (int)(v / 100);
  double minutos = v - grados * 100;
  double d = grados + minutos / 60.0;
  return negativo ? -d : d;
}

// Lee AT+CGNSSINFO y extrae latitud/longitud
bool leerGNSS(double &lat, double &lon) {
  String r = enviarAT("AT+CGNSSINFO");
  int p = r.indexOf("+CGNSSINFO:");
  if (p < 0) return false;
  int fin = r.indexOf('\r', p);
  if (fin < 0) fin = r.length();
  String datos = r.substring(p + 11, fin);
  datos.trim();

  // Separar los campos por comas
  String c[24];
  int n = 0, ini = 0;
  for (int i = 0; i <= (int)datos.length() && n < 24; i++) {
    if (i == (int)datos.length() || datos[i] == ',') {
      c[n++] = datos.substring(ini, i);
      ini = i + 1;
    }
  }

  // Buscar el patron <lat>,<N/S>,<lon>,<E/W> (robusto a variantes de firmware)
  for (int i = 1; i + 2 < n; i++) {
    if ((c[i] == "N" || c[i] == "S") && (c[i + 2] == "E" || c[i + 2] == "W") &&
        c[i - 1].length() > 0 && c[i + 1].length() > 0) {
      lat = nmeaADecimal(c[i - 1].toDouble(), c[i] == "S");
      lon = nmeaADecimal(c[i + 1].toDouble(), c[i + 2] == "W");
      return true;
    }
  }
  return false;  // sin posicion (campos vacios)
}

void actualizarUbicacion() {
  double lat, lon;
  if (leerGNSS(lat, lon)) {
    ultLat = lat;
    ultLon = lon;
    hayUbicacion = true;
    tUltimaUbicacion = millis();
  }
}

String construirMensaje() {
  String msg = "SOS ALERTA Manilla IoT: necesito ayuda. ";
  if (ubicacionFresca) {
    msg += "Ubicacion: ";
  } else if (hayUbicacion) {
    unsigned long seg = (millis() - tUltimaUbicacion) / 1000;
    msg += "Ultima ubicacion (hace " + String(seg) + " s): ";
  } else {
    msg += "Ubicacion no disponible.";
    return msg;
  }
  msg += "https://maps.google.com/?q=" + String(ultLat, 6) + "," + String(ultLon, 6);
  return msg;
}

// ======================================================
//            CAPA FISICA: actuador (vibracion)
// ======================================================

void vibrar(int veces, int ms) {
  for (int i = 0; i < veces; i++) {
    digitalWrite(PIN_MOTOR, HIGH);
    delay(ms);
    digitalWrite(PIN_MOTOR, LOW);
    delay(ms);
  }
}

// ======================================================
//                       SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  modem.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  Serial.println();
  Serial.println("=== Manilla IoT SOS - Universidad CESMAG ===");
  delay(500);
  iniciarModem();
  actualizarUbicacion();
  tUltimoRefresco = millis();
  Serial.println("[LOGICA] Sistema en REPOSO. Mantener el boton SOS 3 s para alertar.");
}

// ======================================================
//               CAPA LOGICA: maquina de estados
// ======================================================

void loop() {
  bool presionado = (digitalRead(PIN_BOTON) == LOW);

  switch (estado) {

    case REPOSO:
      if (presionado) {
        tPresion = millis();
        tSuelto = 0;
        estado = ACTIVANDO;
        Serial.println("[LOGICA] Boton presionado: mantener 3 s para activar...");
      } else if (millis() - tUltimoRefresco >= T_REFRESCO_GNSS) {
        // Refresco periodico: guarda la ultima ubicacion valida
        actualizarUbicacion();
        digitalWrite(PIN_LED_RED, redRegistrada() ? HIGH : LOW);
        tUltimoRefresco = millis();
      }
      break;

    case ACTIVANDO:
      if (!presionado) {
        if (tSuelto == 0) tSuelto = millis();
        if (millis() - tSuelto >= T_REBOTE) {
          Serial.println("[LOGICA] Pulsacion corta descartada: no se envia alerta.");
          estado = REPOSO;
        }
      } else {
        tSuelto = 0;
        if (millis() - tPresion >= T_PULSACION_SOS) {
          tActivacion = millis();
          vibrar(1, 150);  // aviso discreto: alerta aceptada
          Serial.println("[LOGICA] *** ALERTA ACTIVADA ***");
          estado = UBICANDO;
        }
      }
      break;

    case UBICANDO: {
      ubicacionFresca = false;
      unsigned long t0 = millis();
      while (millis() - t0 < T_ESPERA_GNSS) {
        double lat, lon;
        if (leerGNSS(lat, lon)) {
          ultLat = lat;
          ultLon = lon;
          hayUbicacion = true;
          tUltimaUbicacion = millis();
          ubicacionFresca = true;
          break;
        }
        delay(1000);
      }
      Serial.print("[METRICA] Activacion -> ubicacion: ");
      Serial.print(millis() - tActivacion);
      Serial.print(" ms (");
      Serial.print(ubicacionFresca ? "GNSS actual" : (hayUbicacion ? "ultima conocida" : "sin ubicacion"));
      Serial.println(")");
      estado = ENVIANDO;
      break;
    }

    case ENVIANDO: {
      String msg = construirMensaje();
      Serial.print("[PROCESO] Mensaje (");
      Serial.print(msg.length());
      Serial.print(" caracteres): ");
      Serial.println(msg);

      int entregados = 0;
      for (int i = 0; i < NUM_CONTACTOS; i++) {
        bool ok = false;
        for (int intento = 1; intento <= REINTENTOS_SMS && !ok; intento++) {
          ok = enviarSMS(CONTACTOS[i], msg);
          if (!ok) delay(1000);
        }
        if (ok) entregados++;
        Serial.print("[METRICA] SMS a ");
        Serial.print(CONTACTOS[i]);
        Serial.print(ok ? " ENVIADO a los " : " FALLIDO a los ");
        Serial.print(millis() - tActivacion);
        Serial.println(" ms de la activacion");
      }
      Serial.print("[METRICA] Alertas entregadas: ");
      Serial.print(entregados);
      Serial.print("/");
      Serial.println(NUM_CONTACTOS);
      estado = (entregados > 0) ? CONFIRMADO : FALLO;
      break;
    }

    case CONFIRMADO:
      vibrar(3, 150);  // confirmacion silenciosa: alerta enviada
      Serial.println("[LOGICA] Alerta confirmada. Volviendo a REPOSO.");
      while (digitalRead(PIN_BOTON) == LOW) delay(10);  // evita re-disparo
      estado = REPOSO;
      break;

    case FALLO:
      vibrar(1, 1000);  // vibracion larga: no se pudo enviar
      Serial.println("[LOGICA] No se pudo enviar la alerta. Revisar senal/SIM.");
      while (digitalRead(PIN_BOTON) == LOW) delay(10);
      estado = REPOSO;
      break;
  }

  delay(20);
}