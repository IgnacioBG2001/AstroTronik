#include <Arduino.h>
#include "driver/pcnt.h"
#define ENABLE_BLE 1
#if ENABLE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif


// ENUMS Y ESTRUCTURAS
enum MoveKind {
  MOVE_NONE,
  MOVE_GOTO,
  MOVE_NUDGE,
  MOVE_HOME,
};

struct RoutineStep {
  const char* label;
  bool home;
  bool status;
  float altDeg;
  float azDeg;
};

struct TrapezoidalProfile {
  float currentPos;
  float currentVel;
  float maxVel;
  float maxAcc;

  void init(float initialPos, float vMax, float aMax) {
    currentPos = initialPos;
    currentVel = 0.0f;
    maxVel = vMax;
    maxAcc = aMax;
  }

  float update(float targetPos, float deltaT) {
    float error = targetPos - currentPos;

    if (fabs(error) < 1.0f && fabs(currentVel) < (maxAcc * deltaT)) {
      currentPos = targetPos;
      currentVel = 0.0f;
      return currentPos;
    }

    float maxStoppingVel = sqrtf(2.0f * maxAcc * fabs(error));
    float targetVel = (error > 0.0f ? 1.0f : -1.0f) * fminf(maxVel, maxStoppingVel);

    float maxDeltaV = maxAcc * deltaT;
    float deltaV = targetVel - currentVel;

    if (deltaV > maxDeltaV) {
      currentVel += maxDeltaV;
    } else if (deltaV < -maxDeltaV) {
      currentVel -= maxDeltaV;
    } else {
      currentVel = targetVel;
    }

    currentPos += currentVel * deltaT;
    return currentPos;
  }
};

// PROTOTIPOS DE FUNCIÓN 
void startMove(float altDeg, float azDeg, MoveKind kind);
void handleJsonCommand(String json);
void sendResponse(const String& message);
void sendStatus();
void stopMotors();
void apagarHardwareMotores();
void updateMovementCompletion();
void initPCNTEncoder(pcnt_unit_t unit, int pinA, int pinB);
int32_t getEncoderPosition(pcnt_unit_t unit, volatile int32_t *mult, volatile int32_t *offset);
bool calcularYControlarMotor(int32_t pos, int32_t target, float deltaT, float &eprev, float &eintegral, float &dFilt, float pwmMin, int canal1, int canal2);
float getPhysicalDegreesAz();


// DEFINICIÓN DE PINES Y CANALES PWM
// Motor 1: Altitud
const int pinIN1_Alt = 16;
const int pinIN2_Alt = 17;
#define ENCA_Alt 25 
#define ENCB_Alt 26 
const int canalPWM1_Alt = 0;
const int canalPWM2_Alt = 1;

// Motor 2: Azimut
const int pinIN1_Az = 19;
const int pinIN2_Az = 18;
#define ENCA_Az 32  
#define ENCB_Az 33  
const int canalPWM1_Az  = 2;
const int canalPWM2_Az  = 3;

// Parámetros PWM
const int freqPWM = 20000;
const int resPWM  = 8; // 8 bits (0 - 255)


// PARÁMETROS MECÁNICOS Y CONVERSIÓN
const float PULSOS_POR_REVOLUCION = 44000.0f;
const float GRADOS_A_PULSOS = PULSOS_POR_REVOLUCION / 360.0f;
const float PULSOS_A_GRADOS = 360.0f / PULSOS_POR_REVOLUCION;

const int32_t TOLERANCIA_ERROR = 6; // Tolerancia de ~0.049 grados

const float VMAX_ALT = 1460.0f;  // ~12°/s
const float AMAX_ALT = 730.0f;   // ~6°/s^2

const float VMAX_AZ  = 3050.0f;  // ~25°/s
const float AMAX_AZ  = 1830.0f;  // ~15°/s^2

const float ALT_MIN_DEG = -90.0f;
const float ALT_MAX_DEG = 90.0f;
const float AZ_MIN_DEG  = -360.0f;
const float AZ_MAX_DEG  = 360.0f;

// COMPENSACIÓN DE BACKLASH (HOLGURA MECÁNICA)
const float BACKLASH_DEG_AZ = 20.0f; // Grados de juego medidos en Azimut
const float BACKLASH_PULSES_AZ = BACKLASH_DEG_AZ * GRADOS_A_PULSOS;
// Estado del flanco engranado: +1 (adelante), -1 (atrás)
int lastFlankAz = 1;
float prevTargetDegAz = 0.0f;

// Parametros de Comunicación
const unsigned long ROUTINE_PAUSE_MS = 600;
const unsigned long WATCHDOG_TIMEOUT_MS = 1000;
const unsigned int COMMAND_MAX_LENGTH = 256;


// PARÁMETROS DE CONTROL PID Y FILTRO
const float kp = 1.2f;
const float kd = 0.08f;
const float ki = 0.002f;

const float PWM_MIN_ALT = 124.0f;
const float PWM_MIN_AZ  = 124.0f;
const float LPF_ALPHA   = 0.80f;

const uint32_t SAMPLE_TIME_MS = 10; // 100 Hz determinista
const float DELTA_T = SAMPLE_TIME_MS / 1000.0f;

// HARDWARE PCNT (CONTADOR DE ENCODERS)
#define PCNT_H_LIM_VAL  20000
#define PCNT_L_LIM_VAL -20000

volatile int32_t multAlt = 0;
volatile int32_t multAz  = 0;
volatile int32_t offsetAlt = 0;
volatile int32_t offsetAz  = 0;

static void IRAM_ATTR pcnt_overflow_isr(void *arg) {
  uint32_t status = 0;
  
  pcnt_get_event_status(PCNT_UNIT_0, &status);
  if (status & PCNT_EVT_H_LIM) multAlt++;
  else if (status & PCNT_EVT_L_LIM) multAlt--;

  pcnt_get_event_status(PCNT_UNIT_1, &status);
  if (status & PCNT_EVT_H_LIM) multAz++;
  else if (status & PCNT_EVT_L_LIM) multAz--;
}

void initPCNTEncoder(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t cfg_ch0 = {
    .pulse_gpio_num = pinA,
    .ctrl_gpio_num  = pinB,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_DEC,
    .neg_mode       = PCNT_COUNT_INC,
    .counter_h_lim  = PCNT_H_LIM_VAL,
    .counter_l_lim  = PCNT_L_LIM_VAL,
    .unit           = unit,
    .channel        = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&cfg_ch0);

  pcnt_config_t cfg_ch1 = {
    .pulse_gpio_num = pinB,
    .ctrl_gpio_num  = pinA,
    .lctrl_mode     = PCNT_MODE_KEEP,
    .hctrl_mode     = PCNT_MODE_REVERSE,
    .pos_mode       = PCNT_COUNT_DEC,
    .neg_mode       = PCNT_COUNT_INC,
    .counter_h_lim  = PCNT_H_LIM_VAL,
    .counter_l_lim  = PCNT_L_LIM_VAL,
    .unit           = unit,
    .channel        = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&cfg_ch1);

  pcnt_set_filter_value(unit, 250);
  pcnt_filter_enable(unit);

  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

int32_t getEncoderPosition(pcnt_unit_t unit, volatile int32_t *mult, volatile int32_t *offset) {
  int16_t hw_count = 0;
  pcnt_get_counter_value(unit, &hw_count);
  return (*mult * PCNT_H_LIM_VAL) + hw_count + *offset;
}

// --- Función auxiliar para obtener los grados reales del tubo ---
float getPhysicalDegreesAz() {
  int32_t posMotorAz = getEncoderPosition(PCNT_UNIT_1, &multAz, &offsetAz);
  
  if (lastFlankAz == -1) {
    posMotorAz += (int32_t)BACKLASH_PULSES_AZ;
  }
  return posMotorAz * PULSOS_A_GRADOS;
}


// CORE 1: LAZO DE CONTROL PID

TrapezoidalProfile profAlt;
TrapezoidalProfile profAz;

portMUX_TYPE targetMux = portMUX_INITIALIZER_UNLOCKED;
float sharedTargetDegAlt = 0.0f;
float sharedTargetDegAz  = 0.0f;
volatile bool movementActive = false;
volatile bool motionCompletedFlag = false;
MoveKind activeMoveKind = MOVE_NONE;

bool calcularYControlarMotor(
  int32_t pos, 
  int32_t target, 
  float deltaT, 
  float &eprev, 
  float &eintegral, 
  float &dFilt,
  float pwmMin, 
  int canal1, 
  int canal2
) {
  int32_t e = pos - target;

  float dedt_raw = (float)(e - eprev) / deltaT;
  dFilt = (LPF_ALPHA * dFilt) + ((1.0f - LPF_ALPHA) * dedt_raw);

  eintegral += (float)e * deltaT;
  if (eintegral > 15000.0f) eintegral = 15000.0f;
  else if (eintegral < -15000.0f) eintegral = -15000.0f;

  float u = (kp * (float)e) + (kd * dFilt) + (ki * eintegral);
  float pwr = fabs(u);

  if (abs(e) <= TOLERANCIA_ERROR) {
    pwr = 0.0f;
    eintegral = 0.0f;
  } else {
    pwr = pwmMin + (pwr * (255.0f - pwmMin) / 255.0f);
    if (pwr > 255.0f) pwr = 255.0f;
  }

  bool sentido = (u < 0);
  if (sentido) {
    ledcWrite(canal1, (uint32_t)pwr);
    ledcWrite(canal2, 0);
  } else {
    ledcWrite(canal1, 0);
    ledcWrite(canal2, (uint32_t)pwr);
  }

  eprev = (float)e;
  return (abs(e) > TOLERANCIA_ERROR);
}

void taskPIDControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_TIME_MS);

  profAlt.init((float)getEncoderPosition(PCNT_UNIT_0, &multAlt, &offsetAlt), VMAX_ALT, AMAX_ALT);
  profAz.init((float)getEncoderPosition(PCNT_UNIT_1, &multAz, &offsetAz),   VMAX_AZ, AMAX_AZ);

  float localTargetAlt = 0.0f, localTargetAz = 0.0f;
  float eprevAlt = 0.0f, eintegralAlt = 0.0f, dFiltAlt = 0.0f;
  float eprevAz  = 0.0f, eintegralAz  = 0.0f, dFiltAz  = 0.0f;
  bool isMoving = false;

  for (;;) {
    // 1. Obtener setpoint final deseado
    portENTER_CRITICAL(&targetMux);
    localTargetAlt = sharedTargetDegAlt;
    localTargetAz  = sharedTargetDegAz;
    isMoving       = movementActive;
    portEXIT_CRITICAL(&targetMux);

    // 2. Detección de cambio de sentido en Azimut
    if (localTargetAz > prevTargetDegAz + 0.01f) {
      lastFlankAz = 1;  // Movimiento positivo
      prevTargetDegAz = localTargetAz;
    } else if (localTargetAz < prevTargetDegAz - 0.01f) {
      lastFlankAz = -1; // Movimiento negativo
      prevTargetDegAz = localTargetAz;
    }

    // 3. Convertir a pulsos con compensación de Backlash
    float finalPulsesAlt = localTargetAlt * GRADOS_A_PULSOS;
    float finalPulsesAz  = localTargetAz  * GRADOS_A_PULSOS;

    // Si nos movemos en sentido negativo, sumamos el recorrido extra para cerrar el juego
    if (lastFlankAz == -1) {
      finalPulsesAz -= BACKLASH_PULSES_AZ;
    }

    float setpointInstantaneoAlt = profAlt.update(finalPulsesAlt, DELTA_T);
    float setpointInstantaneoAz  = profAz.update(finalPulsesAz,  DELTA_T);

    int32_t posAlt = getEncoderPosition(PCNT_UNIT_0, &multAlt, &offsetAlt);
    int32_t posAz  = getEncoderPosition(PCNT_UNIT_1, &multAz,  &offsetAz);

    bool movAlt = calcularYControlarMotor(
      posAlt, (int32_t)setpointInstantaneoAlt, DELTA_T, 
      eprevAlt, eintegralAlt, dFiltAlt, 
      PWM_MIN_ALT, canalPWM1_Alt, canalPWM2_Alt
    );

    bool movAz = calcularYControlarMotor(
      posAz, (int32_t)setpointInstantaneoAz, DELTA_T, 
      eprevAz, eintegralAz, dFiltAz, 
      PWM_MIN_AZ, canalPWM1_Az, canalPWM2_Az
    );

    bool profileDone = (fabs(profAlt.currentVel) < 1.0f) && (fabs(profAz.currentVel) < 1.0f);
    bool atTargetAlt = abs(posAlt - (int32_t)finalPulsesAlt) <= TOLERANCIA_ERROR;
    bool atTargetAz  = abs(posAz  - (int32_t)finalPulsesAz)  <= TOLERANCIA_ERROR;

    if (isMoving && profileDone && atTargetAlt && atTargetAz) {
      portENTER_CRITICAL(&targetMux);
      movementActive = false;
      motionCompletedFlag = true;
      portEXIT_CRITICAL(&targetMux);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// CORE 0: BLE Y COMUNICACIONES
#if ENABLE_BLE
const char* BLE_DEVICE_NAME = "AstroPilot-OE4";
const char* BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const char* BLE_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const char* BLE_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

BLECharacteristic* txCharacteristic = nullptr;
bool bleClientConnected = false;
#endif

const RoutineStep OE4_ROUTINE[] = {
  {"HOME", true, false, 0.0f, 0.0f},
  {"GOTO alt=0 az=45", false, false, 0.0f, 45.0f},
  {"GOTO alt=0 az=30", false, false, 0.0f, 30.0f},
  {"GOTO alt=0 az=120", false, false, 0.0f, 120.0f},
  {"HOME", true, false, 0.0f, 0.0f},
  {"STATUS", false, true, 0.0f, 0.0f},
};
const int OE4_ROUTINE_COUNT = sizeof(OE4_ROUTINE) / sizeof(OE4_ROUTINE[0]);

String serialBuffer;
bool routineActive = false;
bool routineWaiting = false;
int routineIndex = 0;
unsigned long routineNextAtMs = 0;

bool watchdogEnabled = false;
unsigned long lastPingAtMs = 0;

#if ENABLE_BLE
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
    Serial.println("BLE conectado");
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    Serial.println("BLE desconectado. Advertising reiniciado.");
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = String(characteristic->getValue().c_str());
    value.trim();
    if (value.length() == 0) return;
    if (value.length() > COMMAND_MAX_LENGTH) {
      sendResponse("ERROR: comando demasiado largo");
      return;
    }

    Serial.print("BLE RX: ");
    Serial.println(value);
    handleJsonCommand(value);
  }
};

void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(BLE_SERVICE_UUID);
  txCharacteristic = service->createCharacteristic(
    BLE_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic* rxCharacteristic = service->createCharacteristic(
    BLE_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  rxCharacteristic->setCallbacks(new RxCallbacks());
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE listo: AstroPilot-OE4 (Core 0)");
}
#else
void setupBle() {}
#endif

void sendResponse(const String& message) {
  Serial.println(message);

#if ENABLE_BLE
  if (bleClientConnected && txCharacteristic != nullptr) {
    txCharacteristic->setValue((uint8_t*)message.c_str(), message.length());
    txCharacteristic->notify();
  }
#endif
}

float normalizeAzimuth(float azDeg) {
  while (azDeg < 0.0f) azDeg += 360.0f;
  while (azDeg >= 360.0f) azDeg -= 360.0f;
  return azDeg;
}

bool isNumericLiteral(String raw) {
  raw.trim();
  if (raw.length() == 0) return false;

  bool hasDigit = false;
  bool hasDecimalPoint = false;
  
  for (int i = 0; i < raw.length(); i++) {
    char ch = raw[i];
    if (ch >= '0' && ch <= '9') {
      hasDigit = true;
      continue;
    }
    if ((ch == '-' || ch == '+') && i == 0) continue;
    if (ch == '.' && !hasDecimalPoint) {
      hasDecimalPoint = true;
      continue;
    }
    return false;
  }
  return hasDigit;
}

bool extractJsonString(String json, const String& key, String& value) {
  String marker = String("\"") + key + String("\"");
  int keyIndex = json.indexOf(marker);
  if (keyIndex < 0) return false;
  int colonIndex = json.indexOf(':', keyIndex + marker.length());
  if (colonIndex < 0) return false;
  int firstQuote = json.indexOf('"', colonIndex + 1);
  if (firstQuote < 0) return false;
  int secondQuote = json.indexOf('"', firstQuote + 1);
  if (secondQuote < 0) return false;
  value = json.substring(firstQuote + 1, secondQuote);
  value.trim();
  return value.length() > 0;
}

bool hasJsonKey(String json, const String& key) {
  String marker = String("\"") + key + String("\"");
  return json.indexOf(marker) >= 0;
}

bool extractJsonBool(String json, const String& key, bool& value) {
  String marker = String("\"") + key + String("\"");
  int keyIndex = json.indexOf(marker);
  if (keyIndex < 0) return false;

  int colonIndex = json.indexOf(':', keyIndex + marker.length());
  if (colonIndex < 0) return false;

  int endIndex = colonIndex + 1;
  while (endIndex < json.length() && json[endIndex] != ',' && json[endIndex] != '}') {
    endIndex++;
  }

  String raw = json.substring(colonIndex + 1, endIndex);
  raw.trim();
  raw.toLowerCase();
  
  if (raw == "true") {
    value = true;
    return true;
  }
  if (raw == "false") {
    value = false;
    return true;
  }

  return false;
}

bool extractJsonFloat(String json, const String& key, float& value) {
  String marker = String("\"") + key + String("\"");
  int keyIndex = json.indexOf(marker);
  if (keyIndex < 0) return false;

  int colonIndex = json.indexOf(':', keyIndex + marker.length());
  if (colonIndex < 0) return false;

  int endIndex = colonIndex + 1;
  while (endIndex < json.length() && json[endIndex] != ',' && json[endIndex] != '}') {
    endIndex++;
  }
  
  String raw = json.substring(colonIndex + 1, endIndex);
  raw.trim();
  if (!isNumericLiteral(raw)) return false;
  
  value = raw.toFloat();
  return true;
}


// ACCIONES DE MOVIMIENTO Y RUTINAS
void sendStatus() {
  int32_t pAlt = getEncoderPosition(PCNT_UNIT_0, &multAlt, &offsetAlt);
  float realAzDeg = getPhysicalDegreesAz(); // <- Grados físicos reales corregidos

  String status = String("STATUS -> posicion interna actual: alt=") +
                  String(pAlt) +
                  String(" pulsos (") +
                  String(pAlt * PULSOS_A_GRADOS, 2) +
                  String(" deg), az=") +
                  String(realAzDeg * GRADOS_A_PULSOS) +
                  String(" pulsos (") +
                  String(realAzDeg, 2) +
                  String(" deg), moving=") +
                  String(movementActive ? "true" : "false") +
                  String(", watchdog=") +
                  String(watchdogEnabled ? "true" : "false");
  sendResponse(status);
}

void cancelRoutine(const String& reason) {
  if (routineActive) sendResponse(reason);
  routineActive = false;
  routineWaiting = false;
  routineIndex = 0;
}

void apagarHardwareMotores() {
  ledcWrite(canalPWM1_Alt, 0); ledcWrite(canalPWM2_Alt, 0);
  ledcWrite(canalPWM1_Az,  0); ledcWrite(canalPWM2_Az,  0);

  int32_t pAlt = getEncoderPosition(PCNT_UNIT_0, &multAlt, &offsetAlt);
  int32_t pAz  = getEncoderPosition(PCNT_UNIT_1, &multAz,  &offsetAz);

  portENTER_CRITICAL(&targetMux);
  sharedTargetDegAlt = pAlt * PULSOS_A_GRADOS;
  sharedTargetDegAz  = getPhysicalDegreesAz();
  profAlt.init((float)pAlt, VMAX_ALT, AMAX_ALT);
  profAz.init((float)pAz, VMAX_AZ, AMAX_AZ);
  movementActive = false;
  motionCompletedFlag = false;
  activeMoveKind = MOVE_NONE;
  portEXIT_CRITICAL(&targetMux);
}

void startMove(float altDeg, float azDeg, MoveKind kind) {
  portENTER_CRITICAL(&targetMux);
  sharedTargetDegAlt = altDeg;
  sharedTargetDegAz  = azDeg;
  activeMoveKind     = kind;
  movementActive     = true;
  motionCompletedFlag = false;
  portEXIT_CRITICAL(&targetMux);
}

void startGoto(float altDeg, float azDeg) {
  long altPulses = (long)round(altDeg * GRADOS_A_PULSOS);
  long azPulses  = (long)round(azDeg  * GRADOS_A_PULSOS);

  sendResponse(
    String("Comando: GOTO alt=") + String(altDeg, 2) +
    String(" deg, az=") + String(azDeg, 2) +
    String(" deg -> alt=") + String(altPulses) +
    String(" pulsos, az=") + String(azPulses) + String(" pulsos")
  );
  startMove(altDeg, azDeg, MOVE_GOTO);
}

void startNudge(float deltaAltDeg, float deltaAzDeg) {
  portENTER_CRITICAL(&targetMux);
  float baseAlt = sharedTargetDegAlt;
  float baseAz  = sharedTargetDegAz;
  portEXIT_CRITICAL(&targetMux);

  float nextAltDeg = baseAlt + deltaAltDeg;
  float nextAzDeg  = normalizeAzimuth(baseAz + deltaAzDeg);

  if (nextAltDeg < ALT_MIN_DEG) nextAltDeg = ALT_MIN_DEG;
  if (nextAltDeg > ALT_MAX_DEG) nextAltDeg = ALT_MAX_DEG;

  sendResponse(
    String("Comando: NUDGE dAlt=") + String(deltaAltDeg, 2) +
    String(" deg, dAz=") + String(deltaAzDeg, 2) +
    String(" deg -> alt=") + String(nextAltDeg, 2) +
    String(" deg, az=") + String(nextAzDeg, 2) + String(" deg")
  );
  startMove(nextAltDeg, nextAzDeg, MOVE_NUDGE);
}

void setCurrentLogicalPosition(float altDeg, float azDeg) {
  if (movementActive) {
    sendResponse("ERROR: SET_POSITION requiere motores detenidos");
    return;
  }

  int32_t altPulses = (int32_t)round(altDeg * GRADOS_A_PULSOS);
  int32_t azPulses  = (int32_t)round(azDeg  * GRADOS_A_PULSOS);

  int16_t hw_count_alt = 0, hw_count_az = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &hw_count_alt);
  pcnt_get_counter_value(PCNT_UNIT_1, &hw_count_az);

  portENTER_CRITICAL(&targetMux);
  offsetAlt = altPulses - ((multAlt * PCNT_H_LIM_VAL) + hw_count_alt);
  offsetAz  = azPulses  - ((multAz  * PCNT_H_LIM_VAL) + hw_count_az);

  sharedTargetDegAlt = altDeg;
  sharedTargetDegAz  = azDeg;

  profAlt.init((float)altPulses, VMAX_ALT, AMAX_ALT);
  profAz.init((float)azPulses,   VMAX_AZ, AMAX_AZ);

  movementActive = false;
  motionCompletedFlag = false;
  activeMoveKind = MOVE_NONE;
  portEXIT_CRITICAL(&targetMux);

  sendResponse(
    String("SET_POSITION OK -> alt=") + String(altDeg, 4) +
    String(" deg, az=") + String(azDeg, 4) +
    String(" deg")
  );
  sendStatus();
}

void startHome() {
  sendResponse("Comando: HOME -> Volver a Cero");
  startMove(0.0f, 0.0f, MOVE_HOME);
}

void stopMotors() {
  apagarHardwareMotores();
  cancelRoutine("Rutina OE4 cancelada por STOP");
  sendResponse("STOP Completado -> movimiento detenido");
  sendStatus();
}

void handlePing() {
  lastPingAtMs = millis();
  sendResponse("{\"status\":\"OK\",\"msg\":\"PONG\"}");
}

void setWatchdogEnabled(bool enabled) {
  watchdogEnabled = enabled;
  lastPingAtMs = millis();
  sendResponse(
    String("{\"status\":\"OK\",\"watchdog\":") +
    String(watchdogEnabled ? "true" : "false") +
    String("}")
  );
}

void stopMotorsForWatchdog(unsigned long elapsedMs) {
  apagarHardwareMotores();
  cancelRoutine("Rutina cancelada por pérdida de conexión (Watchdog)");
  sendResponse(
    String("ALERTA: conexión perdida. Motores detenidos en ") +
    String(elapsedMs) +
    String(" ms")
  );
}

void updateWatchdog() {
  if (!watchdogEnabled || !movementActive) return;
  if (lastPingAtMs == 0) return;

  unsigned long elapsedMs = millis() - lastPingAtMs;
  if (elapsedMs <= WATCHDOG_TIMEOUT_MS) return;

  stopMotorsForWatchdog(elapsedMs);
}

void scheduleNextRoutineStep() {
  if (!routineActive) return;

  if (routineIndex >= OE4_ROUTINE_COUNT) {
    routineActive = false;
    routineWaiting = false;
    sendResponse("TEST_ROUTINE Completado");
    return;
  }

  routineWaiting = true;
  routineNextAtMs = millis() + ROUTINE_PAUSE_MS;
}

void runRoutineStep() {
  if (!routineActive || movementActive) return;

  if (routineIndex >= OE4_ROUTINE_COUNT) {
    routineActive = false;
    sendResponse("TEST_ROUTINE Completado");
    return;
  }

  const RoutineStep& step = OE4_ROUTINE[routineIndex];
  sendResponse(
    String("Rutina OE4 paso ") + String(routineIndex + 1) +
    String("/") + String(OE4_ROUTINE_COUNT) +
    String(": ") + String(step.label)
  );

  if (step.status) {
    sendStatus();
    routineIndex++;
    scheduleNextRoutineStep();
    return;
  }

  if (step.home) {
    startHome();
    return;
  }

  startGoto(step.altDeg, step.azDeg);
}

void startTestRoutine() {
  if (movementActive) {
    sendResponse("TEST_ROUTINE no iniciado: movimiento activo. Envie STOP o espere a que termine.");
    return;
  }

  routineActive = true;
  routineWaiting = false;
  routineIndex = 0;
  sendResponse("TEST_ROUTINE iniciado: HOME -> 0/45 -> 0/30 -> 0/120 -> HOME -> STATUS");
  runRoutineStep();
}

void updateRoutine() {
  if (!routineActive || !routineWaiting) return;

  if ((long)(millis() - routineNextAtMs) >= 0) {
    routineWaiting = false;
    runRoutineStep();
  }
}

void updateMovementCompletion() {
  MoveKind completedKind = activeMoveKind;
  activeMoveKind = MOVE_NONE;

  if (completedKind == MOVE_GOTO) {
    sendResponse("GOTO Completado");
  } else if (completedKind == MOVE_NUDGE) {
    sendResponse("NUDGE Completado");
  } else if (completedKind == MOVE_HOME) {
    sendResponse("HOME Completado");

    int32_t pAlt = getEncoderPosition(PCNT_UNIT_0, &multAlt, &offsetAlt);
    int32_t pAz  = getEncoderPosition(PCNT_UNIT_1, &multAz,  &offsetAz);

    sendResponse(
      String("Comando: Volver a Cero -> posicion interna actual: alt=") +
      String(pAlt) +
      String(" pulsos, az=") +
      String(pAz) +
      String(" pulsos")
    );
  }

  if (routineActive) {
    routineIndex++;
    scheduleNextRoutineStep();
  }
}


// PARSERS JSON Y SERIAL

void handleJsonCommand(String json) {
  json.trim();
  if (json.length() > COMMAND_MAX_LENGTH) { sendResponse("ERROR: comando demasiado largo"); return; }
  if (!json.startsWith("{") || !json.endsWith("}")) { sendResponse("ERROR: JSON invalido"); return; }
  String cmd;
  if (!extractJsonString(json, "cmd", cmd)) { sendResponse("ERROR: campos requeridos ausentes"); return; }

  cmd.toUpperCase();

  if (cmd == "STOP") { stopMotors(); return; }
  if (cmd == "PING") { handlePing(); return; }
  if (cmd == "ENABLE_WATCHDOG") {
    bool enabled = false;
    if (!hasJsonKey(json, "enabled")) { sendResponse("ERROR: campos requeridos ausentes"); return; }
    if (!extractJsonBool(json, "enabled", enabled)) { sendResponse("ERROR: JSON invalido"); return; }
    setWatchdogEnabled(enabled);
    return;
  }
  if (cmd == "STATUS") { sendStatus(); return; }
  if (cmd == "TEST_ROUTINE") { startTestRoutine(); return; }

  if (routineActive) { cancelRoutine("Rutina OE4 cancelada por comando manual"); }

  if (cmd == "HOME") { startHome(); return; }
  if (cmd == "GOTO") {
    float altDeg = 0.0f;
    float azDeg = 0.0f;
    if (!hasJsonKey(json, "alt") || !hasJsonKey(json, "az")) { sendResponse("ERROR: campos requeridos ausentes"); return; }
    if (!extractJsonFloat(json, "alt", altDeg) || !extractJsonFloat(json, "az", azDeg)) { sendResponse("ERROR: alt/az no numericos"); return; }
    if (altDeg < ALT_MIN_DEG || altDeg > ALT_MAX_DEG) { sendResponse("ERROR: altitud fuera de rango"); return; }
    if (azDeg < AZ_MIN_DEG || azDeg > AZ_MAX_DEG) { sendResponse("ERROR: azimut fuera de rango"); return; }
    startGoto(altDeg, azDeg);
    return;
  }
  if (cmd == "NUDGE") {
    float deltaAltDeg = 0.0f;
    float deltaAzDeg = 0.0f;
    if (!hasJsonKey(json, "dAlt") || !hasJsonKey(json, "dAz")) { sendResponse("ERROR: campos requeridos ausentes"); return; }
    if (!extractJsonFloat(json, "dAlt", deltaAltDeg) || !extractJsonFloat(json, "dAz", deltaAzDeg)) { sendResponse("ERROR: dAlt/dAz no numericos"); return; }
    startNudge(deltaAltDeg, deltaAzDeg);
    return;
  }
  if (cmd == "SET_POSITION") {
    float altDeg = 0.0f;
    float azDeg = 0.0f;
    if (!hasJsonKey(json, "alt") || !hasJsonKey(json, "az")) { sendResponse("ERROR: campos requeridos ausentes"); return; }
    if (!extractJsonFloat(json, "alt", altDeg) || !extractJsonFloat(json, "az", azDeg)) { sendResponse("ERROR: alt/az no numericos"); return; }
    if (altDeg < ALT_MIN_DEG || altDeg > ALT_MAX_DEG) { sendResponse("ERROR: altitud fuera de rango"); return; }
    if (azDeg < AZ_MIN_DEG || azDeg > AZ_MAX_DEG) { sendResponse("ERROR: azimut fuera de rango"); return; }
    setCurrentLogicalPosition(altDeg, azDeg);
    return;
  }
  sendResponse("ERROR: comando desconocido");
}

void handleSerialInput(String input) {
  input.trim();
  if (input.length() == 0) return;
  if (input.startsWith("{")) { handleJsonCommand(input); return; }

  String upper = input;
  upper.toUpperCase();

  if (upper == "HOME") { handleJsonCommand("{\"cmd\":\"HOME\"}"); return; }
  if (upper == "STATUS") { handleJsonCommand("{\"cmd\":\"STATUS\"}"); return; }
  if (upper == "PING") { handleJsonCommand("{\"cmd\":\"PING\"}"); return; }
  if (upper == "ENABLE_WATCHDOG") { handleJsonCommand("{\"cmd\":\"ENABLE_WATCHDOG\",\"enabled\":true}"); return; }
  if (upper == "DISABLE_WATCHDOG") { handleJsonCommand("{\"cmd\":\"ENABLE_WATCHDOG\",\"enabled\":false}"); return; }
  if (upper == "STOP") { handleJsonCommand("{\"cmd\":\"STOP\"}"); return; }
  if (upper == "TEST_ROUTINE") { handleJsonCommand("{\"cmd\":\"TEST_ROUTINE\"}"); return; }

  if (upper.startsWith("GOTO:")) {
    int commaIndex = input.indexOf(',');
    if (commaIndex < 0) { sendResponse("ERROR: formato esperado GOTO:alt,az"); return; }
    String altRaw = input.substring(5, commaIndex);
    String azRaw = input.substring(commaIndex + 1);
    altRaw.trim(); azRaw.trim();
    if (!isNumericLiteral(altRaw) || !isNumericLiteral(azRaw)) { sendResponse("ERROR: alt/az no numericos"); return; }
    String json = String("{\"cmd\":\"GOTO\",\"alt\":") + altRaw + String(",\"az\":") + azRaw + String("}");
    handleJsonCommand(json);
    return;
  }
  if (upper.startsWith("NUDGE:")) {
    int commaIndex = input.indexOf(',');
    if (commaIndex < 0) { sendResponse("ERROR: formato esperado NUDGE:dAlt,dAz"); return; }
    String altRaw = input.substring(6, commaIndex);
    String azRaw = input.substring(commaIndex + 1);
    altRaw.trim(); azRaw.trim();
    if (!isNumericLiteral(altRaw) || !isNumericLiteral(azRaw)) { sendResponse("ERROR: dAlt/dAz no numericos"); return; }
    String json = String("{\"cmd\":\"NUDGE\",\"dAlt\":") + altRaw + String(",\"dAz\":") + azRaw + String("}");
    handleJsonCommand(json);
    return;
  }
  if (upper.startsWith("SET_POSITION:")) {
    int commaIndex = input.indexOf(',');
    if (commaIndex < 0) { sendResponse("ERROR: formato esperado SET_POSITION:alt,az"); return; }
    String altRaw = input.substring(13, commaIndex);
    String azRaw = input.substring(commaIndex + 1);
    altRaw.trim(); azRaw.trim();
    if (!isNumericLiteral(altRaw) || !isNumericLiteral(azRaw)) { sendResponse("ERROR: alt/az no numericos"); return; }
    String json = String("{\"cmd\":\"SET_POSITION\",\"alt\":") + altRaw + String(",\"az\":") + azRaw + String("}");
    handleJsonCommand(json);
    return;
  }
  sendResponse("ERROR: JSON invalido");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialBuffer.length() > 0) {
        handleSerialInput(serialBuffer);
        serialBuffer = "";
      }
      continue;
    }
    if (serialBuffer.length() < COMMAND_MAX_LENGTH) {
      serialBuffer += ch;
    } else {
      serialBuffer = "";
      sendResponse("ERROR: comando demasiado largo");
    }
  }
}


// TAREA DEDICADA DE COMUNICACIONES
void taskComms(void *pvParameters) {
  unsigned long ultimoPrint = 0;

  for (;;) {
    // Leer comandos desde el buffer Serial
    readSerialCommands();

    // Gestionar la bandera atómica de movimiento completado
    if (motionCompletedFlag) {
      portENTER_CRITICAL(&targetMux);
      motionCompletedFlag = false;
      portEXIT_CRITICAL(&targetMux);
      updateMovementCompletion();
    }

    // Ejecutar máquinas de estado de rutinas y watchdog
    updateRoutine();
    updateWatchdog();

    // Telemetría periódica no bloqueante
    // 4. Telemetría periódica no bloqueante
    if (millis() - ultimoPrint > 500) {
      ultimoPrint = millis();

      int32_t posAlt = getEncoderPosition(PCNT_UNIT_0, &multAlt, &offsetAlt);
      float gradosAlt = posAlt * PULSOS_A_GRADOS;
      float gradosAz  = getPhysicalDegreesAz(); // <- Grados físicos reales corregidos

      portENTER_CRITICAL(&targetMux);
      float tAlt = sharedTargetDegAlt;
      float tAz  = sharedTargetDegAz;
      portEXIT_CRITICAL(&targetMux);

      Serial.print("Target: ["); Serial.print(tAlt, 2); Serial.print("°, "); Serial.print(tAz, 2); Serial.print("°] | ");
      Serial.print("Real: ["); Serial.print(gradosAlt, 2); Serial.print("°, "); Serial.print(gradosAz, 2); Serial.println("°]");
    }

    // Ceder CPU a la pila BLE interna (Core 0)
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


// SETUP Y LOOP PRINCIPAL

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(ENCA_Alt, INPUT_PULLUP);
  pinMode(ENCB_Alt, INPUT_PULLUP);
  pinMode(ENCA_Az,  INPUT_PULLUP);
  pinMode(ENCB_Az,  INPUT_PULLUP);

  // Hardware PCNT
  initPCNTEncoder(PCNT_UNIT_0, ENCB_Alt, ENCA_Alt);
  initPCNTEncoder(PCNT_UNIT_1, ENCA_Az,  ENCB_Az);

  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_overflow_isr, NULL);
  pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_overflow_isr, NULL);

  // Canales PWM (LEDC)
  ledcSetup(canalPWM1_Alt, freqPWM, resPWM);
  ledcSetup(canalPWM2_Alt, freqPWM, resPWM);
  ledcSetup(canalPWM1_Az,  freqPWM, resPWM);
  ledcSetup(canalPWM2_Az,  freqPWM, resPWM);

  ledcAttachPin(pinIN1_Alt, canalPWM1_Alt);
  ledcAttachPin(pinIN2_Alt, canalPWM2_Alt);
  ledcAttachPin(pinIN1_Az,  canalPWM1_Az);
  ledcAttachPin(pinIN2_Az,  canalPWM2_Az);

  // Inicializar BLE
  setupBle();

  // Crear Tarea de Comunicaciones en Core 0 (Prioridad 1)
  xTaskCreatePinnedToCore(
    taskComms,
    "Task_Comms",
    4096,
    NULL,
    1,
    NULL,
    0 // Fijado a CORE 0
  );

  // Crear Tarea de Control PID en Core 1 (Prioridad 5 - Real Time)
  xTaskCreatePinnedToCore(
    taskPIDControl,
    "Task_PID",
    4096,
    NULL,
    5,
    NULL,
    1 // Fijado a CORE 1
  );

  sendResponse("AstroPilot OE4 - Dual Core Activo (Core 0: BLE/Serial, Core 1: PID 100Hz)");
  sendStatus();
}

void loop() {
  // liberado para no consumir ciclos en Core 1
  vTaskDelay(portMAX_DELAY);
}