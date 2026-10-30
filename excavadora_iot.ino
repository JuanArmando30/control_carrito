/*
  PROYECTO: Excavadora IoT controlada desde la API (Swagger UI -> WSS -> ESP32)
  PLACA: ESP32 clasica de 38 pines

  BIBLIOTECAS (Administrador de bibliotecas de Arduino IDE):
  - WiFiManager         (tzapu)
  - WebSockets          (Markus Sattler)   -> incluye WebSocketsClient.h
  - ArduinoJson         (Benoit Blanchon)  -> version 7.x

  FUNCIONES:
  - Portal Wi-Fi con WiFiManager (igual que el programa anterior).
  - Se conecta por WSS al canal del vehiculo en la API.
  - Ejecuta los movimientos que llegan de la API y los muestra en Monitor Serie.
  - Movimiento individual: se ejecuta y se detiene solo (seguridad).
  - DEMO: ejecuta cada paso respetando duracion_ms y pausa_ms.
  - Seguridad: si se pierde el WSS o el Wi-Fi, TODO se detiene.
  - El estado_inicial NO mueve el carrito (solo se muestra).
*/

#include <WiFi.h>
#include <WiFiManager.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// --------------------------------------------------
// 1. CONFIGURACION GENERAL
// --------------------------------------------------

// ---- Wi-Fi ----
const char* PASSWORD_AP = "Shuhua30.";
const int PIN_RESET_WIFI = 27;
const unsigned long TIEMPO_RESET_MS = 5000;
const unsigned long INTERVALO_RECONEXION_MS = 15000;
const unsigned long INTERVALO_ESTADO_MS = 30000;

// ---- API publica (WSS) ----
const char* HOST_API = "vehiculos-juan.duckdns.org";
const uint16_t PUERTO_API = 443;
const int ID_VEHICULO = 1;

// Tiempo maximo que dura un movimiento INDIVIDUAL enviado desde la API.
// Pasado ese tiempo el carrito se detiene solo. Usa 0 para desactivarlo
// (no recomendado: el carrito seguiria hasta recibir DETENER).
const unsigned long DURACION_MAXIMA_INDIVIDUAL_MS = 3000;

// ---- 74HC595 ----
const uint8_t PIN_DATA  = 23;
const uint8_t PIN_CLOCK = 18;
const uint8_t PIN_LATCH = 5;

// ---- Codigos de comando (deben coincidir con el catalogo en la base) ----
const int CODIGO_DETENER   = 9;
const int CODIGO_LUZ       = 10;
const int CODIGO_SONIDO    = 11;
const int CODIGO_ENCENDIDO = 12;

// Salidas de los pulsos (se activan junto al movimiento actual)
const uint16_t MASCARA_ENCENDIDO = 0x0080;
const uint16_t MASCARA_LUZ       = 0x0040;
const uint16_t MASCARA_SONIDO    = 0x0020;

// Tabla: codigo del comando en la base -> salida del 74HC595
struct ComandoHW {
  int codigo;
  uint16_t mascara;
  const char* nombre;
};

const ComandoHW TABLA_COMANDOS[] = {
  { 1, 0x0001, "AVANZAR"         },
  { 2, 0x0004, "RETROCEDER"      },
  { 3, 0x0200, "GIRAR_IZQUIERDA" },
  { 4, 0x0100, "GIRAR_DERECHA"   },
  { 5, 0x0400, "SUBIR_BRAZO"     },  // alzar garra
  { 6, 0x0800, "BAJAR_BRAZO"     },  // bajar garra
  { 7, 0x0002, "ABRIR_PALA"      },  // girar garra izquierda
  { 8, 0x0010, "CERRAR_PALA"     }   // girar garra derecha
};
const int TOTAL_COMANDOS = sizeof(TABLA_COMANDOS) / sizeof(TABLA_COMANDOS[0]);

// ---- Objetos y variables ----
WiFiManager gestorWiFi;
WebSocketsClient clienteWS;
String nombreDispositivo;
String rutaWS;

// Boton fisico
unsigned long inicioPulsacion = 0;
bool pulsacionEnCurso = false;
bool resetEjecutado = false;

// Wi-Fi
unsigned long ultimoIntentoConexion = 0;
unsigned long ultimaImpresionEstado = 0;
bool estadoConexionAnterior = false;
bool conexionAnteriorConocida = false;

// WebSocket
bool wsIniciado = false;
bool wsConectado = false;
// Estado de ejecucion de la DEMO
bool demoActiva = false;

// Salidas
uint16_t movimientoActivo = 0;
bool movimientoTemporizado = false;
unsigned long movimientoFin = 0;

uint16_t pulsoMascara = 0;
bool pulsoActivo = false;
unsigned long pulsoFin = 0;

unsigned long contadorEncendido = 0;


// --------------------------------------------------
// 2. SALIDAS: REGISTROS 74HC595
// --------------------------------------------------

void actualizarSalidas(uint16_t valor) {

  digitalWrite(PIN_LATCH, LOW);

  // Registro superior: bits 08-15
  shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, (uint8_t)(valor >> 8));

  // Registro inferior: bits 00-07
  shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, (uint8_t)(valor & 0xFF));

  digitalWrite(PIN_LATCH, HIGH);
}

void refrescarSalidas() {
  uint16_t valor = movimientoActivo;

  if (pulsoActivo) {
    valor |= pulsoMascara;
  }

  actualizarSalidas(valor);
}

// Detiene el movimiento y los pulsos.
// Solo imprime si realmente habia algo activo.
void detener(const char* motivo) {

  bool habiaActividad = (movimientoActivo != 0) || pulsoActivo;

  movimientoActivo = 0;
  movimientoTemporizado = false;
  pulsoMascara = 0;
  pulsoActivo = false;
  actualizarSalidas(0);

  if (habiaActividad) {
    Serial.print("[MOV] DETENIDO -> ");
    Serial.println(motivo);
  }
}

void iniciarPulso(uint16_t mascara, unsigned long duracionMs, const char* nombre) {

  pulsoMascara = mascara;
  pulsoActivo = true;
  pulsoFin = millis() + duracionMs;
  refrescarSalidas();

  Serial.printf("[MOV] %s (pulso de %lu ms)\n", nombre, duracionMs);
}

// Apaga los pulsos y detiene el movimiento si se cumplio su tiempo.
// Usa restas con signo para funcionar aunque millis() desborde.
void gestionarTemporizadores() {

  unsigned long ahora = millis();

  if (pulsoActivo && (long)(ahora - pulsoFin) >= 0) {
    pulsoActivo = false;
    pulsoMascara = 0;
    refrescarSalidas();
  }

  if (movimientoTemporizado && (long)(ahora - movimientoFin) >= 0) {
    detener("tiempo de movimiento cumplido");
  }
}


// --------------------------------------------------
// 3. EJECUTAR UN COMANDO (codigo de la base de datos)
// --------------------------------------------------

const ComandoHW* buscarComando(int codigo) {

  for (int i = 0; i < TOTAL_COMANDOS; i++) {
    if (TABLA_COMANDOS[i].codigo == codigo) {
      return &TABLA_COMANDOS[i];
    }
  }

  return nullptr;
}

// duracionMs = 0 -> movimiento individual (usa el tiempo maximo de seguridad)
// duracionMs > 0 -> paso de DEMO (dura exactamente duracion_ms)
void ejecutarComando(int codigo, unsigned long duracionMs, const char* origen) {

  if (codigo == CODIGO_DETENER) {
    detener("comando DETENER");
    return;
  }

  if (codigo == CODIGO_LUZ) {
    iniciarPulso(MASCARA_LUZ, 500, "LUZ");
    return;
  }

  if (codigo == CODIGO_SONIDO) {
    iniciarPulso(MASCARA_SONIDO, 500, "SONIDO");
    return;
  }

  if (codigo == CODIGO_ENCENDIDO) {
    contadorEncendido++;

    if (contadorEncendido % 2 == 1) {
      iniciarPulso(MASCARA_ENCENDIDO, 500, "ENCENDER ESCAVADORA");
    } else {
      iniciarPulso(MASCARA_ENCENDIDO, 1500, "APAGAR ESCAVADORA");
    }

    return;
  }

  const ComandoHW* comando = buscarComando(codigo);

  if (comando == nullptr) {
    Serial.printf("[MOV] Codigo %d sin accion definida en la ESP32 (ignorado)\n", codigo);
    return;
  }

  movimientoActivo = comando->mascara;
  refrescarSalidas();

  unsigned long limite = (duracionMs > 0) ? duracionMs : DURACION_MAXIMA_INDIVIDUAL_MS;

  if (limite > 0) {
    movimientoTemporizado = true;
    movimientoFin = millis() + limite;
  } else {
    movimientoTemporizado = false;
  }

  Serial.printf("[MOV] %s (codigo %d) -> salida 0x%04X por %lu ms [origen: %s]\n",
                comando->nombre, codigo, comando->mascara, limite, origen);
}


// --------------------------------------------------
// 4. WEBSOCKET: PROCESAR LOS EVENTOS DE LA API
// --------------------------------------------------


void procesarMensaje(uint8_t* payload, size_t length) {

  JsonDocument doc;

  DeserializationError error =
      deserializeJson(doc, (const char*)payload, length);

  if (error) {
    Serial.print("[WSS] Mensaje JSON invalido: ");
    Serial.println(error.c_str());
    return;
  }

  const char* evento = doc["evento"] | "";
  const char* comando = doc["comando"] | "";

  int codigo = doc["codigo"] | -1;

  JsonObject datos = doc["datos"];

  Serial.printf(
    "[WSS] Evento: %s | comando: %s | codigo: %d\n",
    evento, comando, codigo
  );

  // ------------------------------------------
  // ESTADO INICIAL
  // ------------------------------------------

  if (strcmp(evento, "estado_inicial") == 0) {

    Serial.println(
      "[WSS] Estado inicial recibido (sin ejecutar movimientos)."
    );

  }

  // ------------------------------------------
  // MOVIMIENTO INDIVIDUAL
  // ------------------------------------------

  else if (strcmp(evento, "movimiento_individual") == 0) {

    ejecutarComando(codigo, 0, "POST individual");

  }

  // ------------------------------------------
  // DEMO INICIADA
  // ------------------------------------------

  else if (strcmp(evento, "demo_iniciada") == 0) {

    demoActiva = true;

    int total = datos["total_movimientos"] | 0;

    Serial.println();
    Serial.println("[DEMO] DEMO INICIADA.");
    Serial.printf("[DEMO] Pasos programados: %d\n", total);

  }

  // ------------------------------------------
  // MOVIMIENTOS DE LA DEMO
  // ------------------------------------------

  else if (strcmp(evento, "demo_movimiento") == 0) {

    // IMPORTANTE:
    // No ejecutar ningun paso si la DEMO esta detenida.

    if (!demoActiva) {

      Serial.println(
        "[DEMO] Movimiento ignorado: DEMO inactiva."
      );

      return;
    }

    int orden = datos["orden"] | 0;
    int total = datos["total_movimientos"] | 0;

    unsigned long duracion =
        datos["duracion_ms"] | 0UL;

    unsigned long pausa =
        datos["pausa_ms"] | 0UL;

    Serial.printf(
      "[DEMO] Paso %d/%d | duracion %lu ms | pausa %lu ms\n",
      orden, total, duracion, pausa
    );

    ejecutarComando(codigo, duracion, "DEMO");

  }

  // ------------------------------------------
  // DEMO FINALIZADA
  // ------------------------------------------

  else if (strcmp(evento, "demo_finalizada") == 0) {

    demoActiva = false;

    Serial.println("[DEMO] Finalizada.");

    detener("DEMO finalizada");

  }

  // ------------------------------------------
  // DEMO CANCELADA
  // ------------------------------------------

  else if (strcmp(evento, "demo_cancelada") == 0) {

    // Primero bloquear nuevos movimientos
    demoActiva = false;

    Serial.println("[DEMO] CANCELACION RECIBIDA.");

    detener("DEMO cancelada");

    Serial.println(
      "[DEMO] No se ejecutaran mas movimientos de esta DEMO."
    );

  }

  // ------------------------------------------
  // ERROR DE LA API
  // ------------------------------------------

  else if (strcmp(evento, "error") == 0) {

    demoActiva = false;

    Serial.println("[WSS] La API informo un error.");

    detener("evento de error");

  }

  // ------------------------------------------
  // OTROS EVENTOS
  // ------------------------------------------

  // estado_actualizado y pong se ignoran.
}


void eventoWebSocket(WStype_t tipo, uint8_t* payload, size_t length) {

  switch (tipo) {

    case WStype_CONNECTED:
      wsConectado = true;
      Serial.println();
      Serial.println("[WSS] CONECTADO al canal del vehiculo.");
      break;

    case WStype_DISCONNECTED:

      if (wsConectado) {
        Serial.println("[WSS] DESCONECTADO.");
      }

      wsConectado = false;

      // Bloquear cualquier DEMO que estuviera ejecutandose
      demoActiva = false;

      // Detener todas las salidas por seguridad
      detener("conexion WSS perdida");

      break;
    
    case WStype_TEXT:
      procesarMensaje(payload, length);
      break;

    case WStype_ERROR:
      Serial.println("[WSS] Error de comunicacion.");
      break;

    default:
      break;
  }
}

void iniciarWebSocket() {

  rutaWS = "/ws/vehiculos/" + String(ID_VEHICULO);

  Serial.printf("[WSS] Conectando a wss://%s%s\n", HOST_API, rutaWS.c_str());

  clienteWS.beginSSL(HOST_API, PUERTO_API, rutaWS.c_str());
  clienteWS.onEvent(eventoWebSocket);

  // Reintentar cada 5 s y verificar que el servidor siga vivo.
  clienteWS.setReconnectInterval(5000);
  clienteWS.enableHeartbeat(15000, 3000, 2);
}

void gestionarWebSocket() {

  if (WiFi.status() != WL_CONNECTED) {
    // Sin Wi-Fi no hay control remoto: parar por seguridad.
    detener("Wi-Fi perdido");
    return;
  }

  if (!wsIniciado) {
    iniciarWebSocket();
    wsIniciado = true;
  }

  clienteWS.loop();
}


// --------------------------------------------------
// 5. WI-FI (SIN CAMBIOS RESPECTO AL PROGRAMA ANTERIOR)
// --------------------------------------------------

String generarNombreDispositivo() {

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac.toUpperCase();
  String sufijo = mac.substring(mac.length() - 6);

  return "EXCAVADORA-" + sufijo;
}

void mostrarInformacionWiFi() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("[WiFi] Estado: DESCONECTADO");
    return;
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("       EXCAVADORA IoT - WI-FI");
  Serial.println("========================================");

  Serial.print("Dispositivo: ");
  Serial.println(nombreDispositivo);

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("Estado: CONECTADO");

  Serial.print("SSID conectado: ");
  Serial.println(WiFi.SSID());

  Serial.print("Direccion IPv4: ");
  Serial.println(WiFi.localIP());

  Serial.print("Mascara de subred: ");
  Serial.println(WiFi.subnetMask());

  Serial.print("Puerta de enlace: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("Intensidad RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  Serial.println("========================================");
  Serial.println();

  // No mostrar ni imprimir la contrasena Wi-Fi.
}

void restablecerWiFi(const char* motivo) {

  // Antes de reiniciar, dejar el carrito detenido.
  detener("restablecimiento Wi-Fi");

  Serial.println();
  Serial.println("=== RESTABLECIMIENTO WI-FI ===");
  Serial.print("Motivo: ");
  Serial.println(motivo);

  Serial.println("[1] Desactivando reconexion automatica...");
  WiFi.setAutoReconnect(false);

  Serial.println("[2] Borrando credenciales de Wi-Fi...");
  bool resultado = WiFi.disconnect(false, true);

  Serial.print("[3] Resultado: ");
  Serial.println(resultado ? "OK" : "ERROR");

  Serial.println("[4] Reiniciando...");
  Serial.flush();

  delay(300);
  ESP.restart();
}

void comprobarBotonReset() {

  bool botonPresionado = (digitalRead(PIN_RESET_WIFI) == LOW);

  if (botonPresionado) {

    if (!pulsacionEnCurso) {
      inicioPulsacion = millis();
      pulsacionEnCurso = true;

      Serial.println("[RESET] Boton detectado. Mantener 5 segundos.");
    }

    if (!resetEjecutado && millis() - inicioPulsacion >= TIEMPO_RESET_MS) {

      resetEjecutado = true;
      restablecerWiFi("Boton fisico GPIO27");
    }

  } else {
    pulsacionEnCurso = false;
    resetEjecutado = false;
  }
}

void gestionarConexionWiFi() {

  bool conectado = (WiFi.status() == WL_CONNECTED);

  if (!conexionAnteriorConocida || conectado != estadoConexionAnterior) {

    conexionAnteriorConocida = true;
    estadoConexionAnterior = conectado;

    if (conectado) {
      Serial.println();
      Serial.println("[WiFi] Conexion establecida.");
      mostrarInformacionWiFi();
    } else {
      Serial.println();
      Serial.println("[WiFi] Conexion perdida o pendiente.");
    }
  }

  if (conectado && millis() - ultimaImpresionEstado >= INTERVALO_ESTADO_MS) {
    ultimaImpresionEstado = millis();
    mostrarInformacionWiFi();
  }

  if (gestorWiFi.getConfigPortalActive()) {
    return;
  }

  if (!conectado && millis() - ultimoIntentoConexion >= INTERVALO_RECONEXION_MS) {

    ultimoIntentoConexion = millis();
    Serial.println("[WiFi] Intentando recuperar la conexion...");
    WiFi.reconnect();
  }
}


// --------------------------------------------------
// 6. PRUEBAS LOCALES POR MONITOR SERIE
// --------------------------------------------------

void imprimirAyuda() {
  Serial.println("Comandos por Monitor Serie (codigos de la base de datos):");
  Serial.println("  1 AVANZAR          5 SUBIR_BRAZO");
  Serial.println("  2 RETROCEDER       6 BAJAR_BRAZO");
  Serial.println("  3 GIRAR_IZQUIERDA  7 ABRIR_PALA");
  Serial.println("  4 GIRAR_DERECHA    8 CERRAR_PALA");
  Serial.println("  9 o 0 = DETENER");
  Serial.println("  L = luz   S = sonido   E = encender/apagar");
  Serial.println("  R = borrar credenciales Wi-Fi y reiniciar");
  Serial.println("Boton GPIO27: mantener pulsado 5 segundos (borra Wi-Fi)");
  Serial.println();
}

void comprobarComandoSerial() {

  while (Serial.available() > 0) {

    char c = Serial.read();

    if (c >= '1' && c <= '9') {
      ejecutarComando(c - '0', 0, "Monitor Serie");

    } else if (c == '0') {
      demoActiva = false;
      detener("DEMO detenida localmente por Monitor Serie");

    } else if (c == 'l' || c == 'L') {
      ejecutarComando(CODIGO_LUZ, 0, "Monitor Serie");

    } else if (c == 's' || c == 'S') {
      ejecutarComando(CODIGO_SONIDO, 0, "Monitor Serie");

    } else if (c == 'e' || c == 'E') {
      ejecutarComando(CODIGO_ENCENDIDO, 0, "Monitor Serie");

    } else if (c == 'r' || c == 'R') {
      restablecerWiFi("Comando recibido por Monitor Serie");
    }
  }
}

// --------------------------------------------------
// 7. INICIALIZACION
// --------------------------------------------------

void setup() {

  // Lo primero: salidas en estado seguro (todo apagado).
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  actualizarSalidas(0);

  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" EXCAVADORA IoT - CONTROL POR API (WSS)");
  Serial.println("========================================");

  pinMode(PIN_RESET_WIFI, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  nombreDispositivo = generarNombreDispositivo();

  Serial.print("Dispositivo: ");
  Serial.println(nombreDispositivo);

  Serial.print("Direccion MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println();
  Serial.println("[WiFi] Buscando una red guardada...");

  gestorWiFi.setDebugOutput(false);
  gestorWiFi.setConfigPortalBlocking(false);
  gestorWiFi.setConnectTimeout(20);
  gestorWiFi.setConfigPortalTimeout(0);

  bool conectado = gestorWiFi.autoConnect(nombreDispositivo.c_str(), PASSWORD_AP);

  if (conectado) {
    Serial.println("[WiFi] Conexion inicial correcta.");
    mostrarInformacionWiFi();
  } else {
    Serial.println();
    Serial.println("[WiFi] No hay conexion establecida.");
    Serial.println("[WiFi] El portal puede estar activo.");
    Serial.print("Punto de acceso: ");
    Serial.println(nombreDispositivo);
    Serial.println("Conectate al AP y abre http://192.168.4.1");
  }

  Serial.println();
  imprimirAyuda();
}


// --------------------------------------------------
// 8. BUCLE PRINCIPAL NO BLOQUEANTE
// --------------------------------------------------

void loop() {

  gestorWiFi.process();
  comprobarComandoSerial();
  comprobarBotonReset();
  gestionarConexionWiFi();
  gestionarWebSocket();
  gestionarTemporizadores();
}
