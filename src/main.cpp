#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <math.h>

// =====================================================
// OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// =====================================================
// PINOUT
// =====================================================

// I2C compartido OLED + MPU6500
const int SCL_PIN = D1;              // GPIO5
const int SDA_PIN = D2;              // GPIO4

// Botones
const int SPM_UP_BUTTON_PIN = D3;    // GPIO0
const int SPM_DOWN_BUTTON_PIN = D4;  // GPIO2
const int MAIN_BUTTON_PIN = D5;      // GPIO14

// Cue
const int BUZZER_PIN = D6;           // GPIO12
const int BEAT_LED_PIN = D7;         // GPIO13

// Láser siempre encendido
const int LASER_PIN = D0;            // GPIO16

// Sensor FSR
const int FSR_PIN = A0;

// D8 queda libre

// =====================================================
// MPU6500
// =====================================================

// Dirección I2C encontrada con el scanner
const uint8_t MPU_ADDRESS = 0x68;

// Registros MPU6500
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_CONFIG2 = 0x1D;

const uint8_t REG_ACCEL_XOUT_H = 0x3B;

const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_PWR_MGMT_2 = 0x6C;

const uint8_t REG_WHO_AM_I = 0x75;

// WHO_AM_I esperado para MPU6500
const uint8_t MPU6500_ID = 0x70;

bool mpuAvailable = false;

// -----------------------------------------------------
// Escalas seleccionadas:
//
// Accelerometer = +/- 8 g
//   4096 LSB/g
//
// Gyroscope = +/- 1000 deg/s
//   32.8 LSB/(deg/s)
// -----------------------------------------------------

const float ACCEL_SCALE = 4096.0;
const float GYRO_SCALE = 32.8;

const float GRAVITY = 9.80665;

// =====================================================
// WIFI
// =====================================================

const char* AP_SSID =
  "Baston_Parkinson";

const char* AP_PASSWORD =
  "baston123";

WiFiUDP udp;

const unsigned int UDP_PORT = 4210;

IPAddress broadcastIP(
  192,
  168,
  4,
  255
);

// =====================================================
// SPM
// =====================================================

int baseSPM = 108;

const int MIN_SPM = 60;
const int MAX_SPM = 160;

// Ciclo:
//
// 100 -> 105 -> 110 -> 90 -> 95 -> 100
//
const int percentageModes[] = {
  100,
  105,
  110,
  90,
  95
};

const int NUM_MODES = 5;

int modeIndex = 0;

int targetSPM = 108;

unsigned long beatInterval = 556;

// =====================================================
// CUE
// =====================================================

const int BUZZER_FREQUENCY = 1000;

// Buzzer + LED permanecen activos 100 ms
const unsigned long CUE_DURATION = 100;

bool deviceOn = false;
bool cueActive = false;

unsigned long nextBeatTime = 0;
unsigned long cueStartTime = 0;

// =====================================================
// BOTONES
// =====================================================

const unsigned long DEBOUNCE_TIME = 40;

const unsigned long MAIN_LONG_PRESS_TIME = 3000;

// Mantener +/- durante 600 ms
// para comenzar repetición.
const unsigned long HOLD_START_DELAY = 600;

// Después cambia cada 120 ms.
const unsigned long REPEAT_INTERVAL = 120;

// =====================================================
// BOTON PRINCIPAL
// =====================================================

bool mainLastReading = HIGH;
bool mainStableState = HIGH;

unsigned long mainLastDebounceTime = 0;
unsigned long mainPressStart = 0;

bool mainLongPressHandled = false;

// =====================================================
// BOTON SPM +
// =====================================================

bool upLastReading = HIGH;
bool upStableState = HIGH;

unsigned long upLastDebounceTime = 0;
unsigned long upPressStart = 0;
unsigned long upLastRepeat = 0;

// =====================================================
// BOTON SPM -
// =====================================================

bool downLastReading = HIGH;
bool downStableState = HIGH;

unsigned long downLastDebounceTime = 0;
unsigned long downPressStart = 0;
unsigned long downLastRepeat = 0;

// =====================================================
// FSR
// =====================================================

// Valores provisionales.
//
// Después los calibraremos usando las lecturas
// reales de tu FSR.
//
const int CONTACT_ON_THRESHOLD = 300;
const int CONTACT_OFF_THRESHOLD = 220;

int fsrRaw = 0;

bool caneContact = false;

unsigned long contactStartTime = 0;
unsigned long swingStartTime = 0;

unsigned long lastContactStartTime = 0;

unsigned long lastContactDuration = 0;
unsigned long lastSwingDuration = 0;
unsigned long lastCycleDuration = 0;

// =====================================================
// CADENCIA DEL BASTON
// =====================================================

// Contactos del bastón por minuto.
//
// NO equivale directamente a SPM del paciente.
//
float caneCadence = 0.0;

// =====================================================
// VARIABILIDAD DEL CICLO
// =====================================================

const int CYCLE_WINDOW = 10;

unsigned long cycleBuffer[CYCLE_WINDOW];

int cycleCount = 0;
int cycleIndex = 0;

float cycleMean = 0.0;
float cycleCV = 0.0;

// =====================================================
// ADQUISICION
// =====================================================

// 100 Hz
//
// 1 muestra cada 10 ms
//
const unsigned long SAMPLE_INTERVAL_US = 10000;

unsigned long lastSampleMicros = 0;

unsigned long sampleSequence = 0;

// =====================================================
// OLED
// =====================================================

unsigned long lastDisplayUpdate = 0;

const unsigned long DISPLAY_INTERVAL = 300;

// =====================================================
// FUNCIONES I2C MPU6500
// =====================================================

bool mpuWriteRegister(
  uint8_t reg,
  uint8_t value
) {

  Wire.beginTransmission(
    MPU_ADDRESS
  );

  Wire.write(reg);
  Wire.write(value);

  uint8_t error =
    Wire.endTransmission();

  return error == 0;
}

// =====================================================

bool mpuReadRegister(
  uint8_t reg,
  uint8_t &value
) {

  Wire.beginTransmission(
    MPU_ADDRESS
  );

  Wire.write(reg);

  if (
    Wire.endTransmission(false) != 0
  ) {

    return false;
  }

  uint8_t received =
    Wire.requestFrom(
      MPU_ADDRESS,
      (uint8_t)1
    );

  if (received != 1) {
    return false;
  }

  value = Wire.read();

  return true;
}

// =====================================================
// INICIALIZAR MPU6500
// =====================================================

bool initializeMPU6500() {

  uint8_t whoAmI = 0;

  if (
    !mpuReadRegister(
      REG_WHO_AM_I,
      whoAmI
    )
  ) {

    Serial.println(
      "Error leyendo WHO_AM_I"
    );

    return false;
  }

  Serial.print(
    "WHO_AM_I = 0x"
  );

  if (whoAmI < 0x10) {
    Serial.print("0");
  }

  Serial.println(
    whoAmI,
    HEX
  );

  // Verificar chip
  if (
    whoAmI != MPU6500_ID
  ) {

    Serial.println(
      "El dispositivo NO se identifica como MPU6500"
    );

    return false;
  }

  Serial.println(
    "MPU6500 detectado correctamente."
  );

  // ---------------------------------------------------
  // Reset completo
  // ---------------------------------------------------

  mpuWriteRegister(
    REG_PWR_MGMT_1,
    0x80
  );

  delay(100);

  // ---------------------------------------------------
  // Clock PLL
  // ---------------------------------------------------

  mpuWriteRegister(
    REG_PWR_MGMT_1,
    0x01
  );

  // Activar todos los ejes
  mpuWriteRegister(
    REG_PWR_MGMT_2,
    0x00
  );

  delay(10);

  // ---------------------------------------------------
  // Filtro digital
  //
  // DLPF_CFG = 4
  // ---------------------------------------------------

  mpuWriteRegister(
    REG_CONFIG,
    0x04
  );

  // ---------------------------------------------------
  // Sample Rate
  //
  // Con DLPF activo:
  // 1 kHz / (1 + 9)
  // = 100 Hz
  // ---------------------------------------------------

  mpuWriteRegister(
    REG_SMPLRT_DIV,
    9
  );

  // ---------------------------------------------------
  // Gyroscope
  //
  // FS_SEL = 2
  //
  // +/- 1000 deg/s
  // ---------------------------------------------------

  mpuWriteRegister(
    REG_GYRO_CONFIG,
    0x10
  );

  // ---------------------------------------------------
  // Accelerometer
  //
  // AFS_SEL = 2
  //
  // +/- 8 g
  // ---------------------------------------------------

  mpuWriteRegister(
    REG_ACCEL_CONFIG,
    0x10
  );

  // Filtro acelerómetro
  mpuWriteRegister(
    REG_ACCEL_CONFIG2,
    0x04
  );

  delay(20);

  Serial.println(
    "MPU6500 configurado:"
  );

  Serial.println(
    "Acelerometro: +/-8 g"
  );

  Serial.println(
    "Giroscopio: +/-1000 deg/s"
  );

  Serial.println(
    "Muestreo: 100 Hz"
  );

  return true;
}

// =====================================================
// LEER MPU6500
// =====================================================

bool readMPU6500(
  float &ax,
  float &ay,
  float &az,

  float &gx,
  float &gy,
  float &gz
) {

  Wire.beginTransmission(
    MPU_ADDRESS
  );

  Wire.write(
    REG_ACCEL_XOUT_H
  );

  if (
    Wire.endTransmission(false) != 0
  ) {

    return false;
  }

  // 14 bytes:
  //
  // AX AY AZ TEMP GX GY GZ
  //
  uint8_t count =
    Wire.requestFrom(
      MPU_ADDRESS,
      (uint8_t)14
    );

  if (count != 14) {

    return false;
  }

  int16_t axRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  int16_t ayRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  int16_t azRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  // Temperatura
  // Por ahora no la utilizamos.
  int16_t tempRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  (void)tempRaw;

  int16_t gxRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  int16_t gyRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  int16_t gzRaw =
    (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );

  // ---------------------------------------------------
  // Convertir aceleración a m/s²
  // ---------------------------------------------------

  ax =
    ((float)axRaw /
     ACCEL_SCALE) *
    GRAVITY;

  ay =
    ((float)ayRaw /
     ACCEL_SCALE) *
    GRAVITY;

  az =
    ((float)azRaw /
     ACCEL_SCALE) *
    GRAVITY;

  // ---------------------------------------------------
  // Convertir gyro a grados/s
  // ---------------------------------------------------

  gx =
    (float)gxRaw /
    GYRO_SCALE;

  gy =
    (float)gyRaw /
    GYRO_SCALE;

  gz =
    (float)gzRaw /
    GYRO_SCALE;

  return true;
}

// =====================================================
// CALCULAR SPM OBJETIVO
// =====================================================

void calculateTargetSPM() {

  int percentage =
    percentageModes[modeIndex];

  targetSPM =
    (baseSPM *
     percentage +
     50) /
    100;

  if (targetSPM < 1) {
    targetSPM = 1;
  }

  beatInterval =
    60000UL /
    targetSPM;
}

// =====================================================
// CONSERVAR FASE
// =====================================================

void preserveBeatPhase(
  unsigned long oldInterval
) {

  if (!deviceOn) {
    return;
  }

  if (oldInterval == 0) {
    return;
  }

  unsigned long now =
    millis();

  int32_t remainingSigned =
    (int32_t)(
      nextBeatTime -
      now
    );

  if (remainingSigned <= 0) {

    nextBeatTime =
      now;

    return;
  }

  unsigned long remainingOld =
    (unsigned long)
    remainingSigned;

  if (
    remainingOld >
    oldInterval
  ) {

    remainingOld =
      oldInterval;
  }

  unsigned long newRemaining =
    ((uint64_t)remainingOld *
     beatInterval) /
    oldInterval;

  nextBeatTime =
    now +
    newRemaining;
}

// =====================================================
// ESTADISTICAS DE CICLO
// =====================================================

void calculateCycleStatistics() {

  if (cycleCount == 0) {

    caneCadence = 0;
    cycleMean = 0;
    cycleCV = 0;

    return;
  }

  float sum = 0;

  for (
    int i = 0;
    i < cycleCount;
    i++
  ) {

    sum +=
      cycleBuffer[i];
  }

  cycleMean =
    sum /
    cycleCount;

  if (cycleMean > 0) {

    caneCadence =
      60000.0 /
      cycleMean;
  }

  float sumSquares = 0;

  for (
    int i = 0;
    i < cycleCount;
    i++
  ) {

    float difference =
      cycleBuffer[i] -
      cycleMean;

    sumSquares +=
      difference *
      difference;
  }

  float standardDeviation =
    sqrt(
      sumSquares /
      cycleCount
    );

  if (cycleMean > 0) {

    cycleCV =
      (
        standardDeviation /
        cycleMean
      ) *
      100.0;
  }
}

// =====================================================
// AGREGAR CICLO
// =====================================================

void addCycle(
  unsigned long cycleDuration
) {

  cycleBuffer[cycleIndex] =
    cycleDuration;

  cycleIndex++;

  if (
    cycleIndex >=
    CYCLE_WINDOW
  ) {

    cycleIndex = 0;
  }

  if (
    cycleCount <
    CYCLE_WINDOW
  ) {

    cycleCount++;
  }

  calculateCycleStatistics();
}

// =====================================================
// FSR
// =====================================================

void processFSR(
  unsigned long now
) {

  fsrRaw =
    analogRead(
      FSR_PIN
    );

  // ---------------------------------------------------
  // CONTACTO
  // ---------------------------------------------------

  if (
    !caneContact &&
    fsrRaw >=
      CONTACT_ON_THRESHOLD
  ) {

    caneContact =
      true;

    if (
      swingStartTime >
      0
    ) {

      lastSwingDuration =
        now -
        swingStartTime;
    }

    // Nuevo ciclo
    if (
      lastContactStartTime >
      0
    ) {

      lastCycleDuration =
        now -
        lastContactStartTime;

      addCycle(
        lastCycleDuration
      );
    }

    lastContactStartTime =
      now;

    contactStartTime =
      now;
  }

  // ---------------------------------------------------
  // LEVANTAMIENTO
  // ---------------------------------------------------

  else if (
    caneContact &&
    fsrRaw <=
      CONTACT_OFF_THRESHOLD
  ) {

    caneContact =
      false;

    lastContactDuration =
      now -
      contactStartTime;

    swingStartTime =
      now;
  }
}

// =====================================================
// OLED
// =====================================================

void updateDisplay() {

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    12,
    0
  );

  display.print(
    "BASTON PARKINSON"
  );

  display.drawLine(
    0,
    11,
    127,
    11,
    SSD1306_WHITE
  );

  // ---------------------------------------------------
  // Cueing apagado
  // ---------------------------------------------------

  if (!deviceOn) {

    display.setCursor(
      12,
      17
    );

    display.print(
      "CUEING: APAGADO"
    );

    display.setCursor(
      5,
      30
    );

    display.print(
      "BASE: "
    );

    display.print(
      baseSPM
    );

    display.print(
      " SPM"
    );

    display.setCursor(
      5,
      42
    );

    if (mpuAvailable) {

      display.print(
        "MPU6500: OK"
      );
    }

    else {

      display.print(
        "MPU6500: ERROR"
      );
    }

    display.setCursor(
      5,
      54
    );

    display.print(
      "Laser: ON"
    );

    display.display();

    return;
  }

  // ---------------------------------------------------
  // Cueing encendido
  // ---------------------------------------------------

  display.setCursor(
    2,
    15
  );

  display.print(
    "OBJ:"
  );

  display.setTextSize(2);

  display.setCursor(
    27,
    13
  );

  display.print(
    targetSPM
  );

  display.setTextSize(1);

  display.setCursor(
    67,
    20
  );

  display.print(
    "SPM"
  );

  display.setCursor(
    94,
    20
  );

  display.print(
    percentageModes[
      modeIndex
    ]
  );

  display.print("%");

  // Base + bastón
  display.setCursor(
    2,
    36
  );

  display.print("B:");

  display.print(
    baseSPM
  );

  display.print(" C:");

  if (
    caneCadence >
    0
  ) {

    display.print(
      (int)
      round(
        caneCadence
      )
    );
  }

  else {

    display.print("--");
  }

  display.print(
    " CPM"
  );

  // Contacto + CV
  display.setCursor(
    2,
    50
  );

  display.print(
    "CONT:"
  );

  if (caneContact) {

    display.print(
      "SI "
    );
  }

  else {

    display.print(
      "NO "
    );
  }

  display.print(
    "CV:"
  );

  display.print(
    cycleCV,
    1
  );

  display.print("%");

  display.display();
}

// =====================================================
// CAMBIAR BASE SPM
// =====================================================

void changeBaseSPM(
  int change
) {

  int newSPM =
    baseSPM +
    change;

  if (
    newSPM <
    MIN_SPM
  ) {

    newSPM =
      MIN_SPM;
  }

  if (
    newSPM >
    MAX_SPM
  ) {

    newSPM =
      MAX_SPM;
  }

  if (
    newSPM !=
    baseSPM
  ) {

    unsigned long oldInterval =
      beatInterval;

    baseSPM =
      newSPM;

    calculateTargetSPM();

    preserveBeatPhase(
      oldInterval
    );

    updateDisplay();
  }
}

// =====================================================
// CUE
// =====================================================

void startCue() {

  // Buzzer
  tone(
    BUZZER_PIN,
    BUZZER_FREQUENCY
  );

  // LED
  digitalWrite(
    BEAT_LED_PIN,
    HIGH
  );

  cueStartTime =
    millis();

  cueActive =
    true;
}

// =====================================================

void stopCue() {

  noTone(
    BUZZER_PIN
  );

  digitalWrite(
    BEAT_LED_PIN,
    LOW
  );

  cueActive =
    false;
}

// =====================================================
// ON / OFF CUEING
// =====================================================

void toggleDevice() {

  deviceOn =
    !deviceOn;

  if (deviceOn) {

    // Siempre empezar en 100%
    modeIndex = 0;

    calculateTargetSPM();

    // Primer cue inmediato
    nextBeatTime =
      millis();
  }

  else {

    stopCue();
  }

  // El láser NO cambia aquí.
  // Permanece siempre ON.

  updateDisplay();
}

// =====================================================
// CAMBIAR PORCENTAJE
// =====================================================

void changePercentage() {

  unsigned long oldInterval =
    beatInterval;

  modeIndex++;

  if (
    modeIndex >=
    NUM_MODES
  ) {

    modeIndex = 0;
  }

  calculateTargetSPM();

  preserveBeatPhase(
    oldInterval
  );

  updateDisplay();
}

// =====================================================
// BOTON PRINCIPAL
// =====================================================

void handleMainButton() {

  unsigned long now =
    millis();

  bool reading =
    digitalRead(
      MAIN_BUTTON_PIN
    );

  if (
    reading !=
    mainLastReading
  ) {

    mainLastDebounceTime =
      now;

    mainLastReading =
      reading;
  }

  if (
    now -
    mainLastDebounceTime >
    DEBOUNCE_TIME
  ) {

    if (
      reading !=
      mainStableState
    ) {

      mainStableState =
        reading;

      // Presionado
      if (
        mainStableState ==
        LOW
      ) {

        mainPressStart =
          now;

        mainLongPressHandled =
          false;
      }

      // Liberado
      else {

        unsigned long pressDuration =
          now -
          mainPressStart;

        if (
          !mainLongPressHandled &&
          pressDuration <
            MAIN_LONG_PRESS_TIME
        ) {

          if (deviceOn) {

            changePercentage();
          }
        }
      }
    }
  }

  // Pulsación larga
  if (
    mainStableState ==
      LOW &&

    !mainLongPressHandled &&

    now -
      mainPressStart >=
      MAIN_LONG_PRESS_TIME
  ) {

    mainLongPressHandled =
      true;

    toggleDevice();
  }
}

// =====================================================
// SPM +
// =====================================================

void handleSPMUpButton() {

  unsigned long now =
    millis();

  bool reading =
    digitalRead(
      SPM_UP_BUTTON_PIN
    );

  if (
    reading !=
    upLastReading
  ) {

    upLastDebounceTime =
      now;

    upLastReading =
      reading;
  }

  if (
    now -
    upLastDebounceTime >
    DEBOUNCE_TIME
  ) {

    if (
      reading !=
      upStableState
    ) {

      upStableState =
        reading;

      if (
        upStableState ==
        LOW
      ) {

        upPressStart =
          now;

        upLastRepeat =
          now;

        changeBaseSPM(
          +1
        );
      }
    }
  }

  // Mantener presionado
  if (
    upStableState ==
    LOW
  ) {

    if (
      now -
      upPressStart >=
      HOLD_START_DELAY
    ) {

      if (
        now -
        upLastRepeat >=
        REPEAT_INTERVAL
      ) {

        upLastRepeat =
          now;

        changeBaseSPM(
          +1
        );
      }
    }
  }
}

// =====================================================
// SPM -
// =====================================================

void handleSPMDownButton() {

  unsigned long now =
    millis();

  bool reading =
    digitalRead(
      SPM_DOWN_BUTTON_PIN
    );

  if (
    reading !=
    downLastReading
  ) {

    downLastDebounceTime =
      now;

    downLastReading =
      reading;
  }

  if (
    now -
    downLastDebounceTime >
    DEBOUNCE_TIME
  ) {

    if (
      reading !=
      downStableState
    ) {

      downStableState =
        reading;

      if (
        downStableState ==
        LOW
      ) {

        downPressStart =
          now;

        downLastRepeat =
          now;

        changeBaseSPM(
          -1
        );
      }
    }
  }

  // Mantener presionado
  if (
    downStableState ==
    LOW
  ) {

    if (
      now -
      downPressStart >=
      HOLD_START_DELAY
    ) {

      if (
        now -
        downLastRepeat >=
        REPEAT_INTERVAL
      ) {

        downLastRepeat =
          now;

        changeBaseSPM(
          -1
        );
      }
    }
  }
}

// =====================================================
// ENVIAR DATOS UDP
// =====================================================

void sendDataUDP(
  float ax,
  float ay,
  float az,

  float gx,
  float gy,
  float gz
) {

  char packet[350];

  int length =
    snprintf(
      packet,
      sizeof(packet),

      "%lu,%lu,%d,%d,"
      "%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,"
      "%.2f,%lu,%lu,%lu,"
      "%.2f,%d,%d,%d,%d,%d",

      sampleSequence,
      millis(),

      fsrRaw,
      caneContact ? 1 : 0,

      ax,
      ay,
      az,

      gx,
      gy,
      gz,

      caneCadence,

      lastContactDuration,
      lastSwingDuration,
      lastCycleDuration,

      cycleCV,

      baseSPM,
      targetSPM,

      percentageModes[
        modeIndex
      ],

      deviceOn ? 1 : 0,
      cueActive ? 1 : 0
    );

  udp.beginPacket(
    broadcastIP,
    UDP_PORT
  );

  udp.write(
    (uint8_t*)packet,
    length
  );

  udp.endPacket();

  sampleSequence++;
}

// =====================================================
// ADQUISICION 100 Hz
// =====================================================

void acquireSensorData() {

  unsigned long nowMicros =
    micros();

  if (
    nowMicros -
      lastSampleMicros <
    SAMPLE_INTERVAL_US
  ) {

    return;
  }

  lastSampleMicros +=
    SAMPLE_INTERVAL_US;

  // Recuperar si hubo retraso grande
  if (
    nowMicros -
      lastSampleMicros >
    SAMPLE_INTERVAL_US *
      5
  ) {

    lastSampleMicros =
      nowMicros;
  }

  unsigned long now =
    millis();

  // ---------------------------------------------------
  // FSR
  // ---------------------------------------------------

  processFSR(
    now
  );

  // ---------------------------------------------------
  // MPU6500
  // ---------------------------------------------------

  float ax = NAN;
  float ay = NAN;
  float az = NAN;

  float gx = NAN;
  float gy = NAN;
  float gz = NAN;

  if (mpuAvailable) {

    bool ok =
      readMPU6500(
        ax,
        ay,
        az,

        gx,
        gy,
        gz
      );

    if (!ok) {

      Serial.println(
        "Error leyendo MPU6500"
      );
    }
  }

  // ---------------------------------------------------
  // WIFI
  // ---------------------------------------------------

  sendDataUDP(
    ax,
    ay,
    az,

    gx,
    gy,
    gz
  );
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(
    115200
  );

  delay(500);

  Serial.println();
  Serial.println(
    "============================"
  );

  Serial.println(
    " BASTON PARKINSON - INICIO"
  );

  Serial.println(
    "============================"
  );

  // ===================================================
  // BOTONES
  // ===================================================

  pinMode(
    MAIN_BUTTON_PIN,
    INPUT_PULLUP
  );

  pinMode(
    SPM_UP_BUTTON_PIN,
    INPUT_PULLUP
  );

  pinMode(
    SPM_DOWN_BUTTON_PIN,
    INPUT_PULLUP
  );

  // ===================================================
  // SALIDAS
  // ===================================================

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  pinMode(
    BEAT_LED_PIN,
    OUTPUT
  );

  pinMode(
    LASER_PIN,
    OUTPUT
  );

  noTone(
    BUZZER_PIN
  );

  digitalWrite(
    BEAT_LED_PIN,
    LOW
  );

  // ===================================================
  // LASER SIEMPRE ON
  // ===================================================

  digitalWrite(
    LASER_PIN,
    HIGH
  );

  Serial.println(
    "Laser: ON"
  );

  // ===================================================
  // I2C
  // ===================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  delay(50);

  // ===================================================
  // OLED
  // ===================================================

  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      0x3C
    )
  ) {

    Serial.println(
      "OLED: ERROR"
    );

    while (true) {
      delay(100);
    }
  }

  Serial.println(
    "OLED: OK"
  );

  // ===================================================
  // MPU6500
  // ===================================================

  mpuAvailable =
    initializeMPU6500();

  if (mpuAvailable) {

    Serial.println(
      "MPU6500: OK"
    );
  }

  else {

    Serial.println(
      "MPU6500: ERROR"
    );
  }

  // ===================================================
  // WIFI
  // ===================================================

  WiFi.mode(
    WIFI_AP
  );

  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD
  );

  WiFi.setSleepMode(
    WIFI_NONE_SLEEP
  );

  udp.begin(
    UDP_PORT
  );

  Serial.println();
  Serial.println(
    "WiFi creado:"
  );

  Serial.println(
    AP_SSID
  );

  Serial.print(
    "IP ESP8266: "
  );

  Serial.println(
    WiFi.softAPIP()
  );

  Serial.print(
    "Puerto UDP: "
  );

  Serial.println(
    UDP_PORT
  );

  // ===================================================
  // CONFIGURACION
  // ===================================================

  baseSPM = 108;

  modeIndex = 0;

  calculateTargetSPM();

  updateDisplay();

  lastSampleMicros =
    micros();

  Serial.println();
  Serial.println(
    "Sistema listo."
  );

  Serial.println(
    "============================"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // ===================================================
  // BOTONES
  // ===================================================

  handleMainButton();

  handleSPMUpButton();

  handleSPMDownButton();

  // ===================================================
  // SENSORES
  // ===================================================

  acquireSensorData();

  unsigned long now =
    millis();

  // ===================================================
  // CUEING
  // ===================================================

  if (deviceOn) {

    if (
      (int32_t)(
        now -
        nextBeatTime
      ) >= 0
    ) {

      startCue();

      nextBeatTime +=
        beatInterval;

      // No acumular retrasos
      while (
        (int32_t)(
          now -
          nextBeatTime
        ) >= 0
      ) {

        nextBeatTime +=
          beatInterval;
      }
    }
  }

  // ===================================================
  // TERMINAR BEEP + LED
  // ===================================================

  if (cueActive) {

    if (
      now -
      cueStartTime >=
      CUE_DURATION
    ) {

      stopCue();
    }
  }

  // ===================================================
  // OLED
  // ===================================================

  if (
    now -
      lastDisplayUpdate >=
    DISPLAY_INTERVAL
  ) {

    lastDisplayUpdate =
      now;

    updateDisplay();
  }
}