// V1.1 TSRCT-PCB Nominal Dual-Channel Temperature Controller
// Shipped pair: V1.1_TSRCT_PCB_Nominal_Logger.py (V2 diagnostics retained).
// Board: Arduino Nano Every, Arduino megaAVR Boards core, ATmega4809.
// Library: LiquidCrystal_I2C (Frank de Brabander 1.1.2 API: init/backlight).
// Keep this sketch in a folder named V1.1_TSRCT_PCB_Nominal_Dual_Channel.
// Validation here: host logic/replay checks. Target compilation and physical
// sensor/SSR, watchdog and closed-loop bench checks remain required for release.
//
// V1.1: Pt100/430 ohm, independently configurable derivative filters,
// manuscript conditional integration, RTD recovery, command timeout/watchdog,
// safe startup, 50 Hz rejection, and CRC-16 protected telemetry.
// Both channels default to CONTROL. Boot and watchdog reset NEVER arm heating.
// Verify the selected gains, sensor/SSR mapping and fault behaviour on the plant.
// GPIO duty below is scheduler bookkeeping, not an electrical measurement.
// Independent heater thermal fuses remain necessary for a stuck-on SSR.
// Nominal-operation firmware for TSRCT-PCB-01 Rev B / Arduino Nano Every (ATmega4809).
//
// Fixed hardware mapping retained from the validated dual-channel firmware:
//   Channel 1 = RTD1 (CS7) -> independent PID 1 -> SSR A6
//   Channel 2 = RTD2 (CS6) -> independent PID 2 -> SSR A7
//   Software SPI: MOSI D8, MISO D9, SCK D10
//   Push-button D2, buzzer D3, 20x4 I2C LCD at 0x27
//
// Channel modes:
//   CHANNEL_OFF        : RTD acquisition ignored; PID off; SSR forced off; channel faults ignored.
//   CHANNEL_SENSE_ONLY : RTD measurement active; PID off; SSR forced off; RTD/overtemp safety active.
//   CHANNEL_CONTROL    : RTD measurement + PID + SSR actuation active; RTD/overtemp safety active.
//
// Master state machine:
//   SYS_IDLE   : sensing continues for enabled channels; both SSR outputs forced off.
//   SYS_ACTIVE : CONTROL channels run independently; SENSE_ONLY channels remain measurement-only.
//   SYS_ALARM  : latched global alarm; BOTH SSR outputs forced off until the fault clears and
//                the push-button is pressed. Alarm acknowledgement returns to IDLE only.
//
// Safety philosophy:
//   Any active-channel overtemperature, persistent RTD failure, or stale RTD signal disables
//   BOTH heater channels. An OFF channel is intentionally excluded from fault qualification.
//
// Serial telemetry:
//   One D-prefixed row is emitted at 1 Hz continuously (IDLE, ACTIVE and ALARM) for the companion
//   Python logger. Configuration/event lines begin with '#'.

#include <Arduino.h>
#if !defined(__AVR_ATmega4809__)
#error "V1.1 requires an Arduino Nano Every (ATmega4809), not a classic Nano."
#endif
#include <avr/wdt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/atomic.h>


// ============================================================================
// LOCAL NON-BLOCKING MAX31865 DRIVER
// Derived from the supplied Adafruit MAX31865 implementation (BSD licence).
// ============================================================================
#define MAX31865_CONFIG_REG       0x00
#define MAX31865_CONFIG_BIAS      0x80
#define MAX31865_CONFIG_MODEAUTO  0x40
#define MAX31865_CONFIG_1SHOT     0x20
#define MAX31865_CONFIG_3WIRE     0x10
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ  0x01
#define MAX31865_RTDMSB_REG       0x01
#define MAX31865_HFAULTMSB_REG    0x03
#define MAX31865_HFAULTLSB_REG    0x04
#define MAX31865_LFAULTMSB_REG    0x05
#define MAX31865_LFAULTLSB_REG    0x06
#define MAX31865_FAULTSTAT_REG    0x07
#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0, MAX31865_3WIRE = 1, MAX31865_4WIRE = 0
} max31865_numwires_t;

class LocalMAX31865 {
 public:
  LocalMAX31865(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t sck)
      : csPin(cs), mosiPin(mosi), misoPin(miso), sckPin(sck) {}

  bool begin(max31865_numwires_t wires, bool filter50Hz) {
    pinMode(csPin, OUTPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(sckPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    digitalWrite(sckPin, LOW);
    digitalWrite(mosiPin, LOW);

    uint8_t cfg = readRegister8(MAX31865_CONFIG_REG);
    cfg &= ~(MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO |
             MAX31865_CONFIG_1SHOT | MAX31865_CONFIG_FILT50HZ);
    if (wires == MAX31865_3WIRE) cfg |= MAX31865_CONFIG_3WIRE;
    else cfg &= ~MAX31865_CONFIG_3WIRE;
    if (filter50Hz) cfg |= MAX31865_CONFIG_FILT50HZ;
    expectedConfig = cfg & (MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_3WIRE |
                            MAX31865_CONFIG_FILT50HZ);
    writeRegister8(MAX31865_CONFIG_REG, cfg);
    setThresholds(0, 0xFFFF);
    clearFault();
    return configurationMatches();
  }

  bool configurationMatches() {
    const uint8_t mask = MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_3WIRE |
                         MAX31865_CONFIG_FILT50HZ;
    return (readConfig() & mask) == expectedConfig;
  }

  uint8_t readFault() {
    return readRegister8(MAX31865_FAULTSTAT_REG);
  }

  void clearFault() {
    uint8_t c = readRegister8(MAX31865_CONFIG_REG);
    c &= ~0x2C;
    c |= MAX31865_CONFIG_FAULTSTAT;
    writeRegister8(MAX31865_CONFIG_REG, c);
  }

  void enableBias(bool enabled) {
    uint8_t c = readRegister8(MAX31865_CONFIG_REG);
    if (enabled) c |= MAX31865_CONFIG_BIAS;
    else c &= ~MAX31865_CONFIG_BIAS;
    writeRegister8(MAX31865_CONFIG_REG, c);
  }

  void triggerOneShot() {
    uint8_t c = readRegister8(MAX31865_CONFIG_REG);
    c |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, c);
  }

  uint16_t readRawRTD(bool &faultPresent, uint8_t &faultCode) {
    uint16_t r = readRegister16(MAX31865_RTDMSB_REG);
    faultCode = readFault();
    faultPresent = ((r & 1U) != 0U) || (faultCode != 0U);
    return r >> 1;
  }

  float calculateTemperature(uint16_t raw, float nominal, float ref) {
    float Z1, Z2, Z3, Z4, Rt, temp;
    Rt = raw;
    Rt /= 32768.0f;
    Rt *= ref;
    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
    Z3 = (4.0f * RTD_B) / nominal;
    Z4 = 2.0f * RTD_B;
    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;
    if (temp >= 0.0f) return temp;

    Rt /= nominal;
    Rt *= 100.0f;
    float p = Rt;
    temp = -242.02f;
    temp += 2.2228f * p;
    p *= Rt; temp += 2.5859e-3f * p;
    p *= Rt; temp -= 4.8260e-6f * p;
    p *= Rt; temp -= 2.8183e-8f * p;
    p *= Rt; temp += 1.5243e-10f * p;
    return temp;
  }

  uint8_t readConfig() {
    return readRegister8(MAX31865_CONFIG_REG);
  }

 private:
  uint8_t csPin, mosiPin, misoPin, sckPin;
  uint8_t expectedConfig = 0;

  uint8_t transfer8(uint8_t out) {
    uint8_t in = 0;
    for (uint8_t m = 0x80; m; m >>= 1) {
      digitalWrite(mosiPin, (out & m) ? HIGH : LOW);
      digitalWrite(sckPin, HIGH);
      delayMicroseconds(1);
      digitalWrite(sckPin, LOW);
      if (digitalRead(misoPin)) in |= m;
      delayMicroseconds(1);
    }
    return in;
  }

  uint8_t readRegister8(uint8_t a) {
    digitalWrite(csPin, LOW);
    transfer8(a & 0x7F);
    uint8_t v = transfer8(0xFF);
    digitalWrite(csPin, HIGH);
    return v;
  }

  uint16_t readRegister16(uint8_t a) {
    digitalWrite(csPin, LOW);
    transfer8(a & 0x7F);
    uint16_t v = (uint16_t)transfer8(0xFF) << 8;
    v |= transfer8(0xFF);
    digitalWrite(csPin, HIGH);
    return v;
  }

  void writeRegister8(uint8_t a, uint8_t v) {
    digitalWrite(csPin, LOW);
    transfer8(a | 0x80);
    transfer8(v);
    digitalWrite(csPin, HIGH);
  }

  void setThresholds(uint16_t l, uint16_t u) {
    writeRegister8(MAX31865_LFAULTLSB_REG, l & 0xFF);
    writeRegister8(MAX31865_LFAULTMSB_REG, l >> 8);
    writeRegister8(MAX31865_HFAULTLSB_REG, u & 0xFF);
    writeRegister8(MAX31865_HFAULTMSB_REG, u >> 8);
  }
};


// ============================================================================
// NOMINAL OPERATION - USER SETTINGS
// Edit this block for normal operation. No other changes should normally be
// required between runs.
// ============================================================================
enum ChannelMode : uint8_t {
  CHANNEL_OFF        = 0,
  CHANNEL_SENSE_ONLY = 1,
  CHANNEL_CONTROL    = 2
};

// Gains below are Trial 2 SIMC PI from main_final-2.pdf, Table 2.
// They describe the characterised dry endplate, not every attached thermal plant.
// Optional manuscript iSIMC PID (parallel, ms output):
// Kp=266.0290, Ki=3.545478, Kd=995.2872.
// The manuscript's PID comparison used derivative filtering DISABLED.
// Edit each channel's Kp/Ki/Kd and filter settings together when changing tuning.
// -------- Channel 1 --------
const ChannelMode CH1_MODE = CHANNEL_CONTROL;
const float CH1_TARGET_C = 70.0f;
// Parallel-controller gains in the same millisecond-domain used by the validated firmware:
//   P [ms] = Kp * error [C]
//   I [ms] += Ki * error [C] * Ts [s]
//   D [ms] = -Kd * filtered_dT/dt [C/s]
const float CH1_KP = 189.02060609f;
const float CH1_KI = 1.994331574f;
const float CH1_KD = 0.0f;
const bool CH1_DERIVATIVE_FILTER_ENABLED = true;
const float CH1_DERIVATIVE_TAU_S = 3.0f;  // alpha = Ts/(tau+Ts); 3 s -> 0.25
const float CH1_OVERTEMP_C = 90.0f;

// -------- Channel 2 --------
const ChannelMode CH2_MODE = CHANNEL_CONTROL;
const float CH2_TARGET_C = 60.0f;
const float CH2_KP = 189.02060609f;
const float CH2_KI = 1.994331574f;
const float CH2_KD = 0.0f;
const bool CH2_DERIVATIVE_FILTER_ENABLED = true;
const float CH2_DERIVATIVE_TAU_S = 3.0f;  // alpha = Ts/(tau+Ts); 3 s -> 0.25
const float CH2_OVERTEMP_C = 90.0f;


// ============================================================================
// ADVANCED CONTROL / SAFETY SETTINGS
// Normally leave these unchanged unless deliberately retuning the controller
// or adapting the RTD hardware.
// ============================================================================
// Both channels: Pt100 with physically installed 430 ohm reference resistors.
const float R_REF = 430.0f;
const float RNOMINAL = 100.0f;
const bool RTD_FILTER_50HZ = true;
const uint8_t RTD_WIRE_COUNT = 3; // must match physical wiring: 2, 3 or 4
const max31865_numwires_t RTD_WIRES =
    RTD_WIRE_COUNT == 3 ? MAX31865_3WIRE : MAX31865_2WIRE;

const float RTD_MIN_C = -20.0f;
const float RTD_MAX_C = 200.0f;
const float MAX_RTD_STEP_C = 2.0f;
const uint8_t MAX_CONSECUTIVE_RTD_FAILURES = 5;
const unsigned long RTD_BIAS_SETTLE_US = 10000UL;
const unsigned long RTD_CONVERSION_US = 70000UL; // >66 ms maximum at 50 Hz
const unsigned long RTD_SAMPLE_INTERVAL_MS = 160UL;
const unsigned long RTD_STALE_TIMEOUT_MS = 1000UL;
const unsigned long RTD_STARTUP_GRACE_MS = 2000UL;

const unsigned long CONTROL_MIN_VALID_DT_MS = 500UL;
const unsigned long CONTROL_MAX_VALID_DT_MS = 1500UL;
const float CONTROL_SAMPLE_TIME_S = 1.0f;
const uint16_t PWM_WINDOW_MS = 1000U;
const float OUTPUT_MIN_MS = 0.0f;
const float OUTPUT_MAX_MS = (float)PWM_WINDOW_MS;
const float INTEGRAL_MIN_MS = 0.0f;
const float INTEGRAL_MAX_MS = (float)PWM_WINDOW_MS;
const float SP_RAMP_RATE_C_PER_S = 0.1f;
const float ALARM_RESET_HYSTERESIS_C = 2.0f;
const uint8_t RTD_RECOVERY_SAMPLES = 5; // consecutive readings while heaters are off
const uint16_t SSR_COMMAND_TIMEOUT_MS = 2500U; // enforced inside 1 ms ISR
// Independent MCU watchdog: ~8 s; serviced only after a complete foreground loop.
// The earlier ISR command timeout stops heating if the foreground stalls.
const unsigned long CONFIG_REPEAT_INTERVAL_MS = 30000UL;
const char FIRMWARE_VERSION[] = "V1.1";

const unsigned long LCD_UPDATE_INTERVAL_MS = 500UL;
const unsigned long SERIAL_LOG_INTERVAL_MS = 1000UL;


// ============================================================================
// FIXED TSRCT-PCB-01 REV B HARDWARE CONFIGURATION
// Do not change unless the PCB/harness is physically modified.
// ============================================================================
const uint8_t buttonPin = 2;
const uint8_t buzzerPin = 3;
const uint8_t RTD2_CS_PIN = 6;
const uint8_t RTD1_CS_PIN = 7;
const uint8_t SPI_MOSI_PIN = 8;
const uint8_t SPI_MISO_PIN = 9;
const uint8_t SPI_SCK_PIN = 10;
const uint8_t ssrPin_C1 = A6;
const uint8_t ssrPin_C2 = A7;
const uint8_t lcdAddress = 0x27;


// ============================================================================
// SYSTEM / FAULT DEFINITIONS
// ============================================================================
enum SystemState : uint8_t {
  SYS_IDLE   = 0,
  SYS_ACTIVE = 1,
  SYS_ALARM  = 2
};

enum FaultFlag : uint16_t {
  FAULT_NONE          = 0x0000,
  FAULT_CH1_OVERTEMP  = 0x0001,
  FAULT_CH2_OVERTEMP  = 0x0002,
  FAULT_CH1_RTD       = 0x0004,
  FAULT_CH2_RTD       = 0x0008,
  FAULT_CH1_STALE     = 0x0010,
  FAULT_CH2_STALE     = 0x0020,
  FAULT_CONFIGURATION = 0x0040,
  FAULT_CONTROL_TIMEOUT = 0x0080,
  FAULT_WATCHDOG_RESET = 0x0100
};

struct ChannelConfig {
  ChannelMode mode;
  float targetC;
  float kp;
  float ki;
  float kd;
  float overtempC;
  bool derivativeFilter;
  float derivativeTauS;
};

struct PidChannel {
  float setpoint;
  float target;
  float outputMs;
  float calculatedMs;
  float pMs;
  float iMs;
  float dMs;
  float unclampedMs;
  float previousMeasurementC;
  float filteredDydtCPerS;
  float lastDtS;
  int8_t saturationState;
  int8_t integralClampState;
  bool integralClampActive;
  bool integralHoldActive;
  bool enabled;
};

const ChannelConfig channelConfig[2] = {
  {CH1_MODE, CH1_TARGET_C, CH1_KP, CH1_KI, CH1_KD, CH1_OVERTEMP_C, CH1_DERIVATIVE_FILTER_ENABLED, CH1_DERIVATIVE_TAU_S},
  {CH2_MODE, CH2_TARGET_C, CH2_KP, CH2_KI, CH2_KD, CH2_OVERTEMP_C, CH2_DERIVATIVE_FILTER_ENABLED, CH2_DERIVATIVE_TAU_S}
};

LiquidCrystal_I2C lcd(lcdAddress, 20, 4);
LocalMAX31865 rtd1(RTD1_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);
LocalMAX31865 rtd2(RTD2_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);
PidChannel pid[2];

float tempRtd[2] = {NAN, NAN};
float lastValidRtd[2] = {NAN, NAN};
float recoveryCandidateC[2] = {NAN, NAN};
uint8_t recoveryCount[2] = {0, 0};
bool qualifiedOvertemp[2] = {false, false};
uint8_t rtdFailureCount[2] = {0, 0};
uint8_t rtdFaultCode[2] = {0, 0};
unsigned long lastRtdSampleMillis[2] = {0, 0};
uint32_t rtdSampleCount[2] = {0, 0};

enum RtdAsyncState : uint8_t {
  RTD_IDLE = 0,
  RTD_WAIT_BIAS = 1,
  RTD_WAIT_CONVERSION = 2
};
RtdAsyncState rtdState = RTD_IDLE;
uint32_t rtdStateStartUs = 0;
unsigned long rtdCycleStartMillis = 0;

// One TCB2 ISR controls two fully independent SSR outputs in the same exact
// 1000 ms hardware window. Index 0 maps RTD1 to A6; index 1 maps RTD2 to A7.
volatile bool ssrTimerEnabled = false;
volatile uint16_t ssrCommandAgeMs = 0;
volatile bool ssrCommandTimeout = false;
uint32_t telemetrySequence = 0;
uint32_t controlWindowSequence = 0;
unsigned long lastConfigMillis = 0;
uint8_t resetCause = 0;
volatile bool ssrPhysicalState[2] = {false, false};
volatile uint16_t ssrPendingOnMs[2] = {0, 0};
volatile uint16_t ssrActiveOnMs[2] = {0, 0};
volatile uint16_t ssrHighTicksCurrent[2] = {0, 0};
volatile uint16_t ssrHighTicksLast[2] = {0, 0};
volatile uint16_t ssrCommandMsLast[2] = {0, 0};
volatile uint16_t ssrWindowTickMs = 0;
volatile uint32_t ssrWindowSequence = 0;
volatile uint32_t ssrCompletedWindowSequence = 0;
uint32_t ssrWindowSequenceServiced = 0;
uint32_t ssrCompletedWindowSequenceSeen = 0;
uint16_t completedCommandMsLast[2] = {0, 0};
uint32_t ssrHighUsLast[2] = {0, 0};
float actualGpioDutyPct[2] = {0.0f, 0.0f};
uint32_t actualWindowUsLast = 1000000UL;

uint8_t windowsPassedLastUpdate = 0;
bool controlGapActive = false;
uint32_t missedWindowCount = 0;
unsigned long lastControlMillis = 0;
uint32_t lastLoopStartUs = 0;
uint32_t loopMaxUsCurrent = 0;
uint32_t loopMaxUsLast = 0;

SystemState systemState = SYS_IDLE;
uint16_t faultFlagsLatched = FAULT_NONE;
unsigned long activeStartMillis = 0;
unsigned long lastRampUpdateMillis = 0;
unsigned long lastLcdUpdateMillis = 0;
unsigned long lastSerialLogMillis = 0;
unsigned long bootMillis = 0;

int buttonState = HIGH;
int lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50UL;
unsigned long lastAlarmBeepMillis = 0;
bool alarmBuzzerOn = false;


// ============================================================================
// PROTOTYPES
// ============================================================================
void serviceRTDAcquisition();
float validateAsyncTemperature(LocalMAX31865&, uint16_t, bool, uint8_t);
float gateRTD(float, float&);
uint8_t readMAX31865Config(uint8_t);
bool buttonPressed();

bool channelSensingEnabled(uint8_t);
bool channelControlEnabled(uint8_t);
const char* channelModeName(ChannelMode);
const char* channelModeShortName(ChannelMode);
const char* systemStateName(SystemState);
const char* systemStateShortName(SystemState);
bool validateConfiguration();
uint16_t currentFaultConditions(bool useResetHysteresis);
void evaluateSafety();
void enterAlarm(uint16_t, const __FlashStringHelper*);
void handleAlarm();
bool acknowledgeAlarmIfSafe();

void configureSsrHardwareTimer();
void startSsrHardwareWindows();
void stopSsrHardwareWindows();
void publishPendingSsrCommands(float, float);
void snapshotCompletedSsrWindow();

void initialisePidForIdle();
void resetPidChannel(uint8_t, float, bool);
bool computePidChannel(uint8_t, float, bool);
void clearPidOutput(uint8_t);
void disableAllControl();
bool beginNominalControl();
void updateSetpointRamps();
void serviceControlWindow();

void printConfiguration();
void updateLCD();
void updateSerialLog();
void printTemperatureOrNan(Print&, float, uint8_t);
void acceptRtdSample(uint8_t, float);
void printFaultFlagsText(uint16_t);


// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Configure OFF before UART delays, I2C or sensor initialisation.
  digitalWrite(ssrPin_C1, LOW);
  digitalWrite(ssrPin_C2, LOW);
  pinMode(ssrPin_C1, OUTPUT);
  pinMode(ssrPin_C2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  resetCause = RSTCTRL.RSTFR;
  RSTCTRL.RSTFR = resetCause; // write-one-to-clear reset flags
  wdt_reset();
  wdt_disable();
  wdt_enable(WDT_PERIOD_8KCLK_gc);
  Serial.begin(115200);
  delay(500);

  bootMillis = millis();

  Serial.println(F("# ======================================================"));
  Serial.println(F("# TSRCT-PCB Nominal Dual-Channel Controller V1.1"));
  Serial.print(F("# CONFIG: RESET_CAUSE=")); Serial.println(resetCause);
  Serial.println(F("# Hardware       : TSRCT-PCB-01 Rev B / Arduino Nano Every"));
  Serial.println(F("# Mapping        : CH1 RTD1/CS7->A6, CH2 RTD2/CS6->A7"));
  Serial.println(F("# SPI            : software SPI MOSI D8, MISO D9, SCK D10"));
  Serial.println(F("# SSR timing     : TCB2 hardware-defined 1000 ms window"));
  Serial.println(F("# Safety         : any enabled-channel fault disables BOTH SSRs"));
  Serial.println(F("# Serial         : continuous 1 Hz nominal telemetry"));
  Serial.println(F("# ======================================================"));

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("TSRCT Nominal"));
  lcd.setCursor(0, 1); lcd.print(F("Initialising..."));
  lcd.setCursor(0, 2); lcd.print(F("SSRs forced OFF"));
  lcd.setCursor(0, 3); lcd.print(F("Please wait"));

  Serial.print(F("# rtd1.begin() : "));
  Serial.println(rtd1.begin(RTD_WIRES, RTD_FILTER_50HZ) ? F("OK") : F("FAILED"));
  Serial.print(F("# rtd2.begin() : "));
  Serial.println(rtd2.begin(RTD_WIRES, RTD_FILTER_50HZ) ? F("OK") : F("FAILED"));
  if (rtd1.readFault()) rtd1.clearFault();
  if (rtd2.readFault()) rtd2.clearFault();
  Serial.print(F("# U1 CONFIG    : 0x")); Serial.println(readMAX31865Config(RTD1_CS_PIN), HEX);
  Serial.print(F("# U4 CONFIG    : 0x")); Serial.println(readMAX31865Config(RTD2_CS_PIN), HEX);

  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ssrPin_C1, OUTPUT);
  pinMode(ssrPin_C2, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ssrPin_C1, LOW);
  digitalWrite(ssrPin_C2, LOW);

  configureSsrHardwareTimer();
  stopSsrHardwareWindows();

  initialisePidForIdle();
  rtdCycleStartMillis = millis() - RTD_SAMPLE_INTERVAL_MS;
  unsigned long now = millis();
  lastControlMillis = now;
  lastRampUpdateMillis = now;
  lastLcdUpdateMillis = now;
  lastSerialLogMillis = now;

  printConfiguration();

  if (resetCause & RSTCTRL_WDRF_bm) {
    enterAlarm(FAULT_WATCHDOG_RESET, F("WATCHDOG_RESET - BOTH_SSR_DISABLED"));
  } else if (!validateConfiguration()) {
    enterAlarm(FAULT_CONFIGURATION, F("CONFIGURATION_ERROR - BOTH_SSR_DISABLED"));
  } else {
    Serial.println(F("# EVENT: READY_IDLE - press button to arm CONTROL channels"));
  }
}


// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  uint32_t loopNowUs = micros();
  if (lastLoopStartUs != 0) {
    uint32_t dt = loopNowUs - lastLoopStartUs;
    if (dt > loopMaxUsCurrent) loopMaxUsCurrent = dt;
  }
  lastLoopStartUs = loopNowUs;

  serviceRTDAcquisition();
  snapshotCompletedSsrWindow();
  evaluateSafety();

  if (buttonPressed()) {
    if (systemState == SYS_IDLE) {
      if (beginNominalControl()) {
        Serial.println(F("# EVENT: CONTROL_ARMED"));
      } else {
        Serial.println(F("# EVENT: CONTROL_START_REJECTED - check CONTROL mode and RTD validity"));
      }
    } else if (systemState == SYS_ACTIVE) {
      disableAllControl();
      systemState = SYS_IDLE;
      initialisePidForIdle();
      Serial.println(F("# EVENT: CONTROL_DISARMED_BY_USER"));
    } else if (systemState == SYS_ALARM) {
      acknowledgeAlarmIfSafe();
    }
  }

  if (systemState == SYS_ACTIVE) {
    updateSetpointRamps();
    serviceControlWindow();
  } else if (systemState == SYS_ALARM) {
    stopSsrHardwareWindows();
    handleAlarm();
  }

  updateLCD();
  updateSerialLog();
  if ((unsigned long)(millis() - lastConfigMillis) >= CONFIG_REPEAT_INTERVAL_MS) {
    printConfiguration(); // lets a passive logger attach after startup
  }
  wdt_reset();
}


// ============================================================================
// CHANNEL MODE / CONFIGURATION HELPERS
// ============================================================================
bool channelSensingEnabled(uint8_t ch) {
  return channelConfig[ch].mode != CHANNEL_OFF;
}

bool channelControlEnabled(uint8_t ch) {
  return channelConfig[ch].mode == CHANNEL_CONTROL;
}

const char* channelModeName(ChannelMode mode) {
  switch (mode) {
    case CHANNEL_OFF: return "OFF";
    case CHANNEL_SENSE_ONLY: return "SENSE_ONLY";
    case CHANNEL_CONTROL: return "CONTROL";
    default: return "INVALID";
  }
}

const char* channelModeShortName(ChannelMode mode) {
  switch (mode) {
    case CHANNEL_OFF: return "OFF";
    case CHANNEL_SENSE_ONLY: return "MON";
    case CHANNEL_CONTROL: return "CTRL";
    default: return "ERR";
  }
}

const char* systemStateName(SystemState state) {
  switch (state) {
    case SYS_IDLE: return "IDLE";
    case SYS_ACTIVE: return "ACTIVE";
    case SYS_ALARM: return "ALARM";
    default: return "UNKNOWN";
  }
}

const char* systemStateShortName(SystemState state) {
  switch (state) {
    case SYS_IDLE: return "IDLE";
    case SYS_ACTIVE: return "RUN";
    case SYS_ALARM: return "ALRM";
    default: return "ERR";
  }
}

bool validateConfiguration() {
  bool valid = true;

  for (uint8_t ch = 0; ch < 2; ch++) {
    const ChannelConfig &cfg = channelConfig[ch];
    if (cfg.mode > CHANNEL_CONTROL) valid = false;
    if (!isfinite(cfg.targetC) || !isfinite(cfg.kp) || !isfinite(cfg.ki) ||
        !isfinite(cfg.kd) || !isfinite(cfg.overtempC) ||
        !isfinite(cfg.derivativeTauS) || cfg.derivativeTauS < 0.0f) valid = false;

    if (cfg.mode == CHANNEL_CONTROL) {
      if (cfg.targetC < RTD_MIN_C || cfg.targetC >= cfg.overtempC) valid = false;
      if (cfg.kp < 0.0f || cfg.ki < 0.0f || cfg.kd < 0.0f) valid = false;
    }

    if (cfg.mode != CHANNEL_OFF) {
      if (cfg.overtempC <= RTD_MIN_C || cfg.overtempC > RTD_MAX_C) valid = false;
    }
  }

  if (PWM_WINDOW_MS != 1000U) valid = false;  // ISR/control contract is intentionally 1000 ms.
  if (RTD_WIRE_COUNT < 2 || RTD_WIRE_COUNT > 4) valid = false;
  if (!isfinite(SP_RAMP_RATE_C_PER_S) || SP_RAMP_RATE_C_PER_S <= 0.0f) valid = false;
  if (!isfinite(R_REF) || !isfinite(RNOMINAL) || R_REF <= RNOMINAL || RNOMINAL <= 0) valid = false;
  if (RTD_CONVERSION_US < (RTD_FILTER_50HZ ? 66000UL : 55000UL)) valid = false;
  return valid;
}


// ============================================================================
// RTD ACQUISITION
// ============================================================================
float validateAsyncTemperature(LocalMAX31865 &rtd,
                               uint16_t raw,
                               bool faultPresent,
                               uint8_t faultCode) {
  if (faultPresent || faultCode != 0 || raw == 0 || raw >= 32767U ||
      !rtd.configurationMatches() || (rtd.readConfig() & MAX31865_CONFIG_1SHOT)) {
    rtd.clearFault();
    return NAN;
  }
  float t = rtd.calculateTemperature(raw, RNOMINAL, R_REF);
  return (!isfinite(t) || t < RTD_MIN_C || t > RTD_MAX_C) ? NAN : t;
}

float gateRTD(float value, float &lastValid) {
  if (!isfinite(value)) return NAN;
  if (!isnan(lastValid) && fabs(value - lastValid) > MAX_RTD_STEP_C) return NAN;
  lastValid = value;
  return value;
}

// While not heating, requalify a stable sensor without remaining tied forever
// to the last temperature before disconnection. Heating never auto-restarts.
void acceptRtdSample(uint8_t ch, float value) {
  qualifiedOvertemp[ch] = isfinite(value) && value >= channelConfig[ch].overtempC;
  float accepted = NAN;
  if (systemState == SYS_ACTIVE) {
    recoveryCount[ch] = 0;
    recoveryCandidateC[ch] = NAN;
    accepted = gateRTD(value, lastValidRtd[ch]);
  } else if (isfinite(value)) {
    if (!isfinite(recoveryCandidateC[ch]) || fabs(value - recoveryCandidateC[ch]) > MAX_RTD_STEP_C) {
      recoveryCount[ch] = 1;
    } else if (recoveryCount[ch] < RTD_RECOVERY_SAMPLES) {
      recoveryCount[ch]++;
    }
    recoveryCandidateC[ch] = value;
    if (recoveryCount[ch] >= RTD_RECOVERY_SAMPLES) accepted = lastValidRtd[ch] = value;
  } else {
    recoveryCount[ch] = 0;
    recoveryCandidateC[ch] = NAN;
  }
  tempRtd[ch] = accepted;
  rtdSampleCount[ch]++;
  if (isfinite(accepted)) {
    lastRtdSampleMillis[ch] = millis(); // age of VALID measurement, not attempt
    rtdFailureCount[ch] = 0;
  } else if (rtdFailureCount[ch] < 255) {
    rtdFailureCount[ch]++;
  }
}

void serviceRTDAcquisition() {
  const bool useCh1 = channelSensingEnabled(0);
  const bool useCh2 = channelSensingEnabled(1);

  if (!useCh1 && !useCh2) {
    tempRtd[0] = NAN;
    tempRtd[1] = NAN;
    rtdState = RTD_IDLE;
    return;
  }

  uint32_t nowUs = micros();
  unsigned long nowMs = millis();

  switch (rtdState) {
    case RTD_IDLE:
      if ((unsigned long)(nowMs - rtdCycleStartMillis) >= RTD_SAMPLE_INTERVAL_MS) {
        rtdCycleStartMillis = nowMs;

        if (useCh1) {
          rtd1.clearFault();
          rtd1.enableBias(true);
        }
        if (useCh2) {
          rtd2.clearFault();
          rtd2.enableBias(true);
        }

        rtdStateStartUs = micros();
        rtdState = RTD_WAIT_BIAS;
      }
      break;

    case RTD_WAIT_BIAS:
      if ((uint32_t)(nowUs - rtdStateStartUs) >= RTD_BIAS_SETTLE_US) {
        if (useCh1) rtd1.triggerOneShot();
        if (useCh2) rtd2.triggerOneShot();
        rtdStateStartUs = micros();
        rtdState = RTD_WAIT_CONVERSION;
      }
      break;

    case RTD_WAIT_CONVERSION:
      if ((uint32_t)(nowUs - rtdStateStartUs) >= RTD_CONVERSION_US) {
        if (useCh1) {
          bool f1 = false;
          uint16_t raw1 = rtd1.readRawRTD(f1, rtdFaultCode[0]);
          rtd1.enableBias(false);
          acceptRtdSample(0, validateAsyncTemperature(
              rtd1, raw1, f1, rtdFaultCode[0]));
        } else {
          tempRtd[0] = NAN;
          rtdFailureCount[0] = 0;
          rtdFaultCode[0] = 0;
        }

        if (useCh2) {
          bool f2 = false;
          uint16_t raw2 = rtd2.readRawRTD(f2, rtdFaultCode[1]);
          rtd2.enableBias(false);
          acceptRtdSample(1, validateAsyncTemperature(
              rtd2, raw2, f2, rtdFaultCode[1]));
        } else {
          tempRtd[1] = NAN;
          rtdFailureCount[1] = 0;
          rtdFaultCode[1] = 0;
        }

        rtdState = RTD_IDLE;
      }
      break;
  }
}

uint8_t readMAX31865Config(uint8_t csPin) {
  if (csPin == RTD1_CS_PIN) return rtd1.readConfig();
  if (csPin == RTD2_CS_PIN) return rtd2.readConfig();
  return 0xFF;
}


// ============================================================================
// SAFETY / ALARM
// ============================================================================
uint16_t currentFaultConditions(bool useResetHysteresis) {
  uint16_t flags = FAULT_NONE;
  unsigned long now = millis();

  if (!validateConfiguration()) flags |= FAULT_CONFIGURATION;
  if (ssrCommandTimeout) flags |= FAULT_CONTROL_TIMEOUT;

  for (uint8_t ch = 0; ch < 2; ch++) {
    if (!channelSensingEnabled(ch)) continue;

    const float threshold = channelConfig[ch].overtempC -
                            (useResetHysteresis ? ALARM_RESET_HYSTERESIS_C : 0.0f);

    if (qualifiedOvertemp[ch] || (isfinite(tempRtd[ch]) && tempRtd[ch] >= threshold)) {
      flags |= (ch == 0) ? FAULT_CH1_OVERTEMP : FAULT_CH2_OVERTEMP;
    }

    if (rtdFailureCount[ch] >= MAX_CONSECUTIVE_RTD_FAILURES) {
      flags |= (ch == 0) ? FAULT_CH1_RTD : FAULT_CH2_RTD;
    }

    bool stale = false;
    if (lastRtdSampleMillis[ch] == 0) {
      stale = (unsigned long)(now - bootMillis) > RTD_STARTUP_GRACE_MS;
    } else {
      stale = (unsigned long)(now - lastRtdSampleMillis[ch]) > RTD_STALE_TIMEOUT_MS;
    }
    if (stale) flags |= (ch == 0) ? FAULT_CH1_STALE : FAULT_CH2_STALE;
  }

  return flags;
}

void evaluateSafety() {
  if (systemState == SYS_ALARM) return;

  uint16_t activeFaults = currentFaultConditions(false);
  if (activeFaults != FAULT_NONE) {
    enterAlarm(activeFaults, F("GLOBAL_SAFETY_TRIP - BOTH_SSR_DISABLED"));
  }
}

void enterAlarm(uint16_t flags, const __FlashStringHelper* msg) {
  faultFlagsLatched |= flags;
  disableAllControl();
  systemState = SYS_ALARM;
  Serial.print(F("# EVENT: "));
  Serial.print(msg);
  Serial.print(F(",fault_flags=0x"));
  Serial.println(faultFlagsLatched, HEX);
}

void handleAlarm() {
  unsigned long now = millis();
  if (now - lastAlarmBeepMillis > 300UL) {
    lastAlarmBeepMillis = now;
    alarmBuzzerOn = !alarmBuzzerOn;
    if (alarmBuzzerOn) tone(buzzerPin, 2000);
    else noTone(buzzerPin);
  }
}

bool acknowledgeAlarmIfSafe() {
  // A reset is blocked until the physical fault has cleared. The overtemperature
  // reset check includes hysteresis so a channel must cool below threshold-hysteresis.
  uint16_t remaining = currentFaultConditions(true);
  // Acknowledge a past timeout only after the foreground has recovered.
  remaining &= ~FAULT_CONTROL_TIMEOUT;
  for (uint8_t ch = 0; ch < 2; ch++) {
    if (channelSensingEnabled(ch) && !isfinite(tempRtd[ch]))
      remaining |= ch == 0 ? FAULT_CH1_RTD : FAULT_CH2_RTD;
  }
  if (remaining != FAULT_NONE) {
    Serial.print(F("# EVENT: ALARM_RESET_BLOCKED,fault_flags=0x"));
    Serial.println(remaining, HEX);
    return false;
  }

  noTone(buzzerPin);
  alarmBuzzerOn = false;
  faultFlagsLatched = FAULT_NONE;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ssrCommandTimeout = false; }
  disableAllControl();
  initialisePidForIdle();
  systemState = SYS_IDLE;
  Serial.println(F("# EVENT: ALARM_ACKNOWLEDGED_RETURNED_TO_IDLE"));
  return true;
}

void printFaultFlagsText(uint16_t flags) {
  bool first = true;
  if (flags == FAULT_NONE) {
    Serial.print(F("NONE"));
    return;
  }

  #define PRINT_FAULT_NAME(mask, text) \
    if (flags & (mask)) { \
      if (!first) Serial.print('|'); \
      Serial.print(F(text)); \
      first = false; \
    }

  PRINT_FAULT_NAME(FAULT_CH1_OVERTEMP, "CH1_OVERTEMP");
  PRINT_FAULT_NAME(FAULT_CH2_OVERTEMP, "CH2_OVERTEMP");
  PRINT_FAULT_NAME(FAULT_CH1_RTD, "CH1_RTD");
  PRINT_FAULT_NAME(FAULT_CH2_RTD, "CH2_RTD");
  PRINT_FAULT_NAME(FAULT_CH1_STALE, "CH1_STALE");
  PRINT_FAULT_NAME(FAULT_CH2_STALE, "CH2_STALE");
  PRINT_FAULT_NAME(FAULT_CONFIGURATION, "CONFIG");
  PRINT_FAULT_NAME(FAULT_CONTROL_TIMEOUT, "CONTROL_TIMEOUT");
  PRINT_FAULT_NAME(FAULT_WATCHDOG_RESET, "WATCHDOG_RESET");

  #undef PRINT_FAULT_NAME
}


// ============================================================================
// PID CONTROL
// ============================================================================
void initialisePidForIdle() {
  for (uint8_t ch = 0; ch < 2; ch++) {
    pid[ch].setpoint = channelConfig[ch].targetC;
    pid[ch].target = channelConfig[ch].targetC;
    pid[ch].outputMs = 0.0f;
    pid[ch].calculatedMs = 0.0f;
    pid[ch].pMs = 0.0f;
    pid[ch].iMs = 0.0f;
    pid[ch].dMs = 0.0f;
    pid[ch].unclampedMs = 0.0f;
    pid[ch].previousMeasurementC = NAN;
    pid[ch].filteredDydtCPerS = 0.0f;
    pid[ch].lastDtS = 0.0f;
    pid[ch].saturationState = 0;
    pid[ch].integralClampState = 0;
    pid[ch].integralClampActive = false;
    pid[ch].integralHoldActive = false;
    pid[ch].enabled = false;
  }
}

void resetPidChannel(uint8_t ch, float measurement, bool enableControl) {
  pid[ch].outputMs = 0.0f;
  pid[ch].calculatedMs = 0.0f;
  pid[ch].pMs = 0.0f;
  pid[ch].iMs = 0.0f;
  pid[ch].dMs = 0.0f;
  pid[ch].unclampedMs = 0.0f;
  pid[ch].previousMeasurementC = measurement;
  pid[ch].filteredDydtCPerS = 0.0f;
  pid[ch].lastDtS = 0.0f;
  pid[ch].saturationState = 0;
  pid[ch].integralClampState = 0;
  pid[ch].integralClampActive = false;
  pid[ch].integralHoldActive = false;
  pid[ch].enabled = enableControl;
}

void clearPidOutput(uint8_t ch) {
  pid[ch].outputMs = 0.0f;
  pid[ch].calculatedMs = 0.0f;
  pid[ch].pMs = 0.0f;
  pid[ch].dMs = 0.0f;
  pid[ch].unclampedMs = 0.0f;
  pid[ch].saturationState = 0;
  pid[ch].integralClampState = 0;
  pid[ch].integralClampActive = false;
  pid[ch].integralHoldActive = false;
}

bool computePidChannel(uint8_t ch, float measurement, bool validStep) {
  PidChannel &c = pid[ch];
  const ChannelConfig &cfg = channelConfig[ch];

  if (!c.enabled || cfg.mode != CHANNEL_CONTROL || !isfinite(measurement)) {
    clearPidOutput(ch);
    c.previousMeasurementC = NAN;
    c.filteredDydtCPerS = 0.0f;
    return false;
  }

  const float error = c.setpoint - measurement;
  c.pMs = cfg.kp * error;

  if (!validStep || isnan(c.previousMeasurementC)) {
    c.previousMeasurementC = measurement;
    c.filteredDydtCPerS = 0.0f;
    c.dMs = 0.0f;
    c.integralClampState = 0;
    c.integralClampActive = false;
    c.integralHoldActive = true;
    c.unclampedMs = c.pMs + c.iMs;
  } else {
    const float rawDydt = (measurement - c.previousMeasurementC) / CONTROL_SAMPLE_TIME_S;
    c.previousMeasurementC = measurement;

    if (cfg.derivativeFilter && cfg.kd != 0.0f) {
      const float alpha = CONTROL_SAMPLE_TIME_S /
                          (cfg.derivativeTauS + CONTROL_SAMPLE_TIME_S);
      c.filteredDydtCPerS += alpha * (rawDydt - c.filteredDydtCPerS);
      c.dMs = -cfg.kd * c.filteredDydtCPerS;
    } else {
      c.filteredDydtCPerS = (cfg.kd != 0.0f) ? rawDydt : 0.0f;
      c.dMs = (cfg.kd != 0.0f) ? (-cfg.kd * c.filteredDydtCPerS) : 0.0f;
    }

    const float rawI = c.iMs + cfg.ki * error * CONTROL_SAMPLE_TIME_S;
    const float candidateI = constrain(rawI, INTEGRAL_MIN_MS, INTEGRAL_MAX_MS);
    const float candidateOutput = c.pMs + candidateI + c.dMs;
    // main_final-2.pdf equations (5)-(8): permit integration that helps recover
    // from saturation; hold it when error drives further into saturation.
    c.integralHoldActive = (candidateOutput > OUTPUT_MAX_MS && error > 0.0f) ||
                           (candidateOutput < OUTPUT_MIN_MS && error < 0.0f);
    c.integralClampState = 0;
    if (!c.integralHoldActive) {
      c.iMs = candidateI;
      if (rawI > INTEGRAL_MAX_MS) c.integralClampState = 1;
      else if (rawI < INTEGRAL_MIN_MS) c.integralClampState = -1;
    }
    c.integralClampActive = c.integralClampState != 0;
    c.unclampedMs = c.pMs + c.iMs + c.dMs;
  }

  if (c.unclampedMs > OUTPUT_MAX_MS) c.saturationState = 1;
  else if (c.unclampedMs < OUTPUT_MIN_MS) c.saturationState = -1;
  else c.saturationState = 0;

  c.calculatedMs = constrain(c.unclampedMs, OUTPUT_MIN_MS, OUTPUT_MAX_MS);
  if (!isfinite(c.unclampedMs) || !isfinite(c.iMs) || !isfinite(c.dMs)) {
    enterAlarm(FAULT_CONFIGURATION, F("NONFINITE_PID_RESULT - BOTH_SSR_DISABLED"));
    return false;
  }
  c.outputMs = c.calculatedMs;
  return true;
}

void disableAllControl() {
  for (uint8_t ch = 0; ch < 2; ch++) {
    pid[ch].enabled = false;
    pid[ch].outputMs = 0.0f;
    pid[ch].calculatedMs = 0.0f;
    pid[ch].pMs = 0.0f;
    pid[ch].iMs = 0.0f;
    pid[ch].dMs = 0.0f;
    pid[ch].unclampedMs = 0.0f;
    pid[ch].saturationState = 0;
    pid[ch].integralClampState = 0;
    pid[ch].integralClampActive = false;
    pid[ch].integralHoldActive = false;
  }
  stopSsrHardwareWindows();
}

bool beginNominalControl() {
  if (!validateConfiguration()) return false;
  if (currentFaultConditions(false) != FAULT_NONE) return false;

  bool anyControl = false;
  for (uint8_t ch = 0; ch < 2; ch++) {
    if (channelControlEnabled(ch)) {
      anyControl = true;
      if (isnan(tempRtd[ch])) return false;
    }
    if (channelConfig[ch].mode == CHANNEL_SENSE_ONLY && isnan(tempRtd[ch])) return false;
  }
  if (!anyControl) return false;

  disableAllControl();

  unsigned long now = millis();
  for (uint8_t ch = 0; ch < 2; ch++) {
    pid[ch].target = channelConfig[ch].targetC;

    if (channelControlEnabled(ch)) {
      pid[ch].setpoint = constrain(tempRtd[ch], RTD_MIN_C, RTD_MAX_C);
      resetPidChannel(ch, tempRtd[ch], true);
    } else {
      // SENSE_ONLY and OFF channels never accumulate PID state and never actuate.
      pid[ch].setpoint = channelConfig[ch].targetC;
      resetPidChannel(ch, tempRtd[ch], false);
    }
  }

  activeStartMillis = now;
  lastControlMillis = now;
  lastRampUpdateMillis = now;
  missedWindowCount = 0;
  controlGapActive = false;
  windowsPassedLastUpdate = 0;

  publishPendingSsrCommands(0.0f, 0.0f);
  startSsrHardwareWindows();
  systemState = SYS_ACTIVE;

  Serial.print(F("# EVENT: CH1_CONTROLLER,Kp=")); Serial.print(channelConfig[0].kp, 8);
  Serial.print(F(",Ki=")); Serial.print(channelConfig[0].ki, 9);
  Serial.print(F(",Kd=")); Serial.print(channelConfig[0].kd, 8);
  Serial.print(F(",target_C=")); Serial.println(channelConfig[0].targetC, 3);

  Serial.print(F("# EVENT: CH2_CONTROLLER,Kp=")); Serial.print(channelConfig[1].kp, 8);
  Serial.print(F(",Ki=")); Serial.print(channelConfig[1].ki, 9);
  Serial.print(F(",Kd=")); Serial.print(channelConfig[1].kd, 8);
  Serial.print(F(",target_C=")); Serial.println(channelConfig[1].targetC, 3);
  return true;
}

void updateSetpointRamps() {
  unsigned long now = millis();
  unsigned long dtMs = now - lastRampUpdateMillis;
  if (dtMs < 100UL) return;

  const float step = SP_RAMP_RATE_C_PER_S * (dtMs / 1000.0f);
  lastRampUpdateMillis = now;

  for (uint8_t ch = 0; ch < 2; ch++) {
    if (!channelControlEnabled(ch)) continue;
    if (pid[ch].setpoint < pid[ch].target) {
      pid[ch].setpoint = min(pid[ch].setpoint + step, pid[ch].target);
    } else if (pid[ch].setpoint > pid[ch].target) {
      pid[ch].setpoint = max(pid[ch].setpoint - step, pid[ch].target);
    }
  }
}

void serviceControlWindow() {
  uint32_t sequence;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sequence = ssrWindowSequence;
  }

  if (sequence == ssrWindowSequenceServiced) return;

  uint32_t passed = sequence - ssrWindowSequenceServiced;
  ssrWindowSequenceServiced = sequence;
  windowsPassedLastUpdate = passed > 255UL ? 255U : (uint8_t)passed;
  controlGapActive = passed > 1UL;
  if (passed > 1UL) missedWindowCount += passed - 1UL;

  unsigned long now = millis();
  unsigned long actualDtMs = now - lastControlMillis;
  lastControlMillis = now;
  const bool validStep = (passed == 1UL) &&
                         (actualDtMs >= CONTROL_MIN_VALID_DT_MS) &&
                         (actualDtMs <= CONTROL_MAX_VALID_DT_MS);

  float commands[2] = {0.0f, 0.0f};
  for (uint8_t ch = 0; ch < 2; ch++) {
    pid[ch].lastDtS = actualDtMs / 1000.0f;

    if (channelControlEnabled(ch)) {
      computePidChannel(ch, tempRtd[ch], validStep);
      commands[ch] = pid[ch].calculatedMs;
    } else {
      // Explicitly guarantee zero commanded duty in OFF and SENSE_ONLY modes.
      clearPidOutput(ch);
      commands[ch] = 0.0f;
    }
  }

  controlWindowSequence = sequence + 1UL; // these PID terms target NEXT window
  publishPendingSsrCommands(commands[0], commands[1]);
}


// ============================================================================
// DUAL HARDWARE SSR - retained from validated dual-channel implementation
// ============================================================================
static inline void ssrWriteFromISR(uint8_t ch, bool on) {
  if (on != ssrPhysicalState[ch]) {
    ssrPhysicalState[ch] = on;
    digitalWrite(ch == 0 ? ssrPin_C1 : ssrPin_C2, on ? HIGH : LOW);
  }
}

ISR(TCB2_INT_vect) {
  TCB2.INTFLAGS = TCB_CAPT_bm;

  if (!ssrTimerEnabled) {
    ssrWriteFromISR(0, false);
    ssrWriteFromISR(1, false);
    ssrWindowTickMs = 0;
    return;
  }

  if (++ssrCommandAgeMs >= SSR_COMMAND_TIMEOUT_MS) {
    ssrCommandTimeout = true;
    ssrTimerEnabled = false;
    for (uint8_t ch = 0; ch < 2; ch++) {
      ssrPendingOnMs[ch] = 0;
      ssrActiveOnMs[ch] = 0;
      ssrWriteFromISR(ch, false);
    }
    return;
  }

  if (ssrWindowTickMs == 0) {
    for (uint8_t ch = 0; ch < 2; ch++) {
      ssrActiveOnMs[ch] = ssrPendingOnMs[ch];
      ssrHighTicksCurrent[ch] = 0;
      ssrWriteFromISR(ch, ssrActiveOnMs[ch] > 0);
    }
    ssrWindowSequence++;
  }

  for (uint8_t ch = 0; ch < 2; ch++) {
    if (ssrPhysicalState[ch]) ssrHighTicksCurrent[ch]++;
  }

  ssrWindowTickMs++;

  for (uint8_t ch = 0; ch < 2; ch++) {
    if (ssrActiveOnMs[ch] < PWM_WINDOW_MS && ssrWindowTickMs >= ssrActiveOnMs[ch]) {
      ssrWriteFromISR(ch, false);
    }
  }

  if (ssrWindowTickMs >= PWM_WINDOW_MS) {
    for (uint8_t ch = 0; ch < 2; ch++) {
      ssrHighTicksLast[ch] = ssrHighTicksCurrent[ch];
      ssrCommandMsLast[ch] = ssrActiveOnMs[ch];
    }
    ssrCompletedWindowSequence++;
    ssrWindowTickMs = 0;
  }
}

void configureSsrHardwareTimer() {
  const uint32_t counts = (F_CPU / 2UL) / 1000UL;
  static_assert(counts >= 1UL && counts <= 65536UL, "TCB2 1 ms count out of range");
  static_assert(PWM_WINDOW_MS == 1000U, "Nominal firmware requires a 1000 ms SSR window");

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCB2.CTRLA = 0;
    TCB2.CTRLB = TCB_CNTMODE_INT_gc;
    TCB2.CCMP = (uint16_t)(counts - 1UL);
    TCB2.CNT = 0;
    TCB2.INTFLAGS = TCB_CAPT_bm;
    TCB2.INTCTRL = TCB_CAPT_bm;
    TCB2.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
  }
}

void publishPendingSsrCommands(float ch1Ms, float ch2Ms) {
  uint16_t a = (uint16_t)lroundf(constrain(ch1Ms, 0.0f, (float)PWM_WINDOW_MS));
  uint16_t b = (uint16_t)lroundf(constrain(ch2Ms, 0.0f, (float)PWM_WINDOW_MS));
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ssrCommandTimeout) return; // foreground recovery must not re-arm output
    ssrCommandAgeMs = 0;
    ssrPendingOnMs[0] = a;
    ssrPendingOnMs[1] = b;
  }
}

void startSsrHardwareWindows() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ssrTimerEnabled = false;
    ssrWindowTickMs = 0;
    ssrWindowSequence = 0;
    ssrCommandAgeMs = 0;
    ssrCompletedWindowSequence = 0;
    for (uint8_t ch = 0; ch < 2; ch++) {
      ssrPendingOnMs[ch] = 0;
      ssrActiveOnMs[ch] = 0;
      ssrHighTicksCurrent[ch] = 0;
      ssrHighTicksLast[ch] = 0;
      ssrCommandMsLast[ch] = 0;
      ssrPhysicalState[ch] = false;
    }
    digitalWrite(ssrPin_C1, LOW);
    digitalWrite(ssrPin_C2, LOW);
    ssrTimerEnabled = true;
  }

  controlWindowSequence = 0;
  ssrWindowSequenceServiced = 0;
  ssrCompletedWindowSequenceSeen = 0;
  for (uint8_t ch = 0; ch < 2; ch++) {
    completedCommandMsLast[ch] = 0;
    ssrHighUsLast[ch] = 0;
    actualGpioDutyPct[ch] = 0.0f;
  }
  actualWindowUsLast = 1000000UL;
  loopMaxUsCurrent = 0;
  loopMaxUsLast = 0;
  lastLoopStartUs = 0;
}

void stopSsrHardwareWindows() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ssrTimerEnabled = false;
    for (uint8_t ch = 0; ch < 2; ch++) {
      ssrPendingOnMs[ch] = 0;
      ssrActiveOnMs[ch] = 0;
      ssrHighTicksCurrent[ch] = 0;
      ssrHighTicksLast[ch] = 0;
      ssrCommandMsLast[ch] = 0;
      ssrPhysicalState[ch] = false;
    }
    digitalWrite(ssrPin_C1, LOW);
    digitalWrite(ssrPin_C2, LOW);
  }

  // Operational telemetry must immediately report zero duty whenever the
  // system is IDLE or ALARM, rather than retaining the previous completed window.
  for (uint8_t ch = 0; ch < 2; ch++) {
    completedCommandMsLast[ch] = 0;
    ssrHighUsLast[ch] = 0;
    actualGpioDutyPct[ch] = 0.0f;
  }
}

void snapshotCompletedSsrWindow() {
  uint16_t high[2];
  uint16_t cmd[2];
  uint32_t sequence;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sequence = ssrCompletedWindowSequence;
    for (uint8_t ch = 0; ch < 2; ch++) {
      high[ch] = ssrHighTicksLast[ch];
      cmd[ch] = ssrCommandMsLast[ch];
    }
  }

  if (sequence == ssrCompletedWindowSequenceSeen) return;
  ssrCompletedWindowSequenceSeen = sequence;

  for (uint8_t ch = 0; ch < 2; ch++) {
    completedCommandMsLast[ch] = cmd[ch];
    ssrHighUsLast[ch] = (uint32_t)high[ch] * 1000UL;
    actualGpioDutyPct[ch] = (float)high[ch] * 100.0f / (float)PWM_WINDOW_MS;
  }

  loopMaxUsLast = loopMaxUsCurrent;
  loopMaxUsCurrent = 0;
}


// ============================================================================
// BUTTON / USER INTERFACE
// ============================================================================
bool buttonPressed() {
  bool pressed = false;
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) lastDebounceTime = millis();
  if (millis() - lastDebounceTime > debounceDelay && reading != buttonState) {
    buttonState = reading;
    if (buttonState == LOW) pressed = true;
  }
  lastButtonReading = reading;
  return pressed;
}

void updateLCD() {
  unsigned long now = millis();
  if (now - lastLcdUpdateMillis < LCD_UPDATE_INTERVAL_MS) return;
  do {
    lastLcdUpdateMillis += LCD_UPDATE_INTERVAL_MS;
  } while (now - lastLcdUpdateMillis >= LCD_UPDATE_INTERVAL_MS);

  char line[21];
  char t1[8], t2[8], s1[8], s2[8];

  if (channelConfig[0].mode == CHANNEL_OFF) strcpy(t1, "OFF ");
  else if (isnan(tempRtd[0])) strcpy(t1, "--.-");
  else dtostrf(tempRtd[0], 4, 1, t1);

  if (channelConfig[1].mode == CHANNEL_OFF) strcpy(t2, "OFF ");
  else if (isnan(tempRtd[1])) strcpy(t2, "--.-");
  else dtostrf(tempRtd[1], 4, 1, t2);

  if (channelConfig[0].mode == CHANNEL_OFF) strcpy(s1, "--.-");
  else if (!isfinite(pid[0].setpoint) || pid[0].setpoint < RTD_MIN_C || pid[0].setpoint > RTD_MAX_C) strcpy(s1, "ERR");
  else dtostrf(pid[0].setpoint, 4, 1, s1);

  if (channelConfig[1].mode == CHANNEL_OFF) strcpy(s2, "--.-");
  else if (!isfinite(pid[1].setpoint) || pid[1].setpoint < RTD_MIN_C || pid[1].setpoint > RTD_MAX_C) strcpy(s2, "ERR");
  else dtostrf(pid[1].setpoint, 4, 1, s2);

  lcd.setCursor(0, 0);
  if (systemState == SYS_ALARM) {
    snprintf(line, 21, "ALARM 0x%04X       ", faultFlagsLatched);
  } else {
    snprintf(line, 21, "%-4s C1:%-4s C2:%-4s", systemStateShortName(systemState),
             channelModeShortName(channelConfig[0].mode), channelModeShortName(channelConfig[1].mode));
  }
  lcd.print(line);

  lcd.setCursor(0, 1);
  snprintf(line, 21, "C1:%5.5s S:%5.5s%3u%%", t1, s1,
           (unsigned)min(100U, (unsigned)(completedCommandMsLast[0] / 10U)));
  lcd.print(line);

  lcd.setCursor(0, 2);
  snprintf(line, 21, "C2:%5.5s S:%5.5s%3u%%", t2, s2,
           (unsigned)min(100U, (unsigned)(completedCommandMsLast[1] / 10U)));
  lcd.print(line);

  lcd.setCursor(0, 3);
  if (systemState == SYS_ALARM) {
    snprintf(line, 21, "Clear fault + reset ");
  } else if (systemState == SYS_ACTIVE) {
    snprintf(line, 21, "Press: disarm       ");
  } else {
    snprintf(line, 21, "Press: arm control  ");
  }
  lcd.print(line);
}


// ============================================================================
// SERIAL CONFIGURATION / TELEMETRY
// ============================================================================
void printConfiguration() {
  lastConfigMillis = millis();
  Serial.println(F("# FIRMWARE: TSRCT-PCB Nominal Dual-Channel Controller V1.1"));
  Serial.println(F("# CONFIG: FIRMWARE_VERSION=V1.1,SCHEMA_VERSION=1.1,FRAME_CRC=CRC16_CCITT_FALSE,ANTI_WINDUP=CONDITIONAL"));
  Serial.print(F("# CONFIG: CH1_MODE=")); Serial.print(channelModeName(channelConfig[0].mode));
  Serial.print(F(",CH1_TARGET_C=")); Serial.print(channelConfig[0].targetC, 3);
  Serial.print(F(",CH1_KP=")); Serial.print(channelConfig[0].kp, 8);
  Serial.print(F(",CH1_KI=")); Serial.print(channelConfig[0].ki, 9);
  Serial.print(F(",CH1_KD=")); Serial.print(channelConfig[0].kd, 8);
  Serial.print(F(",CH1_OVERTEMP_C=")); Serial.print(channelConfig[0].overtempC, 3);
  Serial.print(F(",CH1_DERIVATIVE_FILTER=")); Serial.print(channelConfig[0].derivativeFilter ? 1 : 0);
  Serial.print(F(",CH1_DERIVATIVE_TAU_S=")); Serial.print(channelConfig[0].derivativeTauS, 3);
  Serial.print(F(",CH1_DERIVATIVE_ALPHA=")); Serial.println(CONTROL_SAMPLE_TIME_S / (channelConfig[0].derivativeTauS + CONTROL_SAMPLE_TIME_S), 6);

  Serial.print(F("# CONFIG: CH2_MODE=")); Serial.print(channelModeName(channelConfig[1].mode));
  Serial.print(F(",CH2_TARGET_C=")); Serial.print(channelConfig[1].targetC, 3);
  Serial.print(F(",CH2_KP=")); Serial.print(channelConfig[1].kp, 8);
  Serial.print(F(",CH2_KI=")); Serial.print(channelConfig[1].ki, 9);
  Serial.print(F(",CH2_KD=")); Serial.print(channelConfig[1].kd, 8);
  Serial.print(F(",CH2_OVERTEMP_C=")); Serial.print(channelConfig[1].overtempC, 3);
  Serial.print(F(",CH2_DERIVATIVE_FILTER=")); Serial.print(channelConfig[1].derivativeFilter ? 1 : 0);
  Serial.print(F(",CH2_DERIVATIVE_TAU_S=")); Serial.print(channelConfig[1].derivativeTauS, 3);
  Serial.print(F(",CH2_DERIVATIVE_ALPHA=")); Serial.println(CONTROL_SAMPLE_TIME_S / (channelConfig[1].derivativeTauS + CONTROL_SAMPLE_TIME_S), 6);

  Serial.print(F("# CONFIG: R_REF_OHM=")); Serial.print(R_REF, 1);
  Serial.print(F(",RNOMINAL_OHM=")); Serial.print(RNOMINAL, 1);
  Serial.print(F(",RTD_SAMPLE_MS=")); Serial.print(RTD_SAMPLE_INTERVAL_MS);
  Serial.print(F(",PWM_WINDOW_MS=")); Serial.print(PWM_WINDOW_MS);
  Serial.print(F(",SP_RAMP_C_PER_S=")); Serial.print(SP_RAMP_RATE_C_PER_S, 3);
  Serial.print(F(",RTD_WIRES=")); Serial.print(RTD_WIRE_COUNT);
  Serial.print(F(",RTD_FILTER_HZ=")); Serial.print(RTD_FILTER_50HZ ? 50 : 60);
  Serial.print(F(",RTD_CONVERSION_US=")); Serial.print(RTD_CONVERSION_US);
  Serial.print(F(",COMMAND_TIMEOUT_MS=")); Serial.println(SSR_COMMAND_TIMEOUT_MS);
  Serial.println(F("# NOTE: GPIO duty/window values are scheduler estimates; PID terms target control_window_sequence."));

  Serial.println(F("# FIELDS: uptime_s,active_elapsed_s,sys_state,alarm,fault_flags,ch1_mode,ch2_mode,ch1_C,ch2_C,ch1_setpoint_C,ch2_setpoint_C,ch1_target_C,ch2_target_C,ch1_duty_pct,ch2_duty_pct,ch1_scheduled_gpio_duty_pct,ch2_scheduled_gpio_duty_pct,ch1_p_pct,ch1_i_pct,ch1_d_pct,ch2_p_pct,ch2_i_pct,ch2_d_pct,ch1_unclamped_pct,ch2_unclamped_pct,ch1_saturation,ch2_saturation,ch1_i_clamp_state,ch2_i_clamp_state,ch1_i_clamp_active,ch2_i_clamp_active,ch1_filtered_dydt_C_per_s,ch2_filtered_dydt_C_per_s,control_dt_s,windows_passed,control_gap_active,missed_window_count,scheduled_window_us,ch1_ssr_high_us,ch2_ssr_high_us,rtd_state,ch1_rtd_age_ms,ch2_rtd_age_ms,ch1_rtd_sample_count,ch2_rtd_sample_count,ch1_rtd_failure_count,ch2_rtd_failure_count,ch1_rtd_fault_code,ch2_rtd_fault_code,loop_max_us,telemetry_seq,control_window_sequence,completed_window_sequence,ch1_i_hold_active,ch2_i_hold_active"));
}

void printTemperatureOrNan(Print &out, float value, uint8_t digits) {
  if (!isfinite(value)) out.print(F("nan"));
  else out.print(value, digits);
}

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection/xor-out.
// Frame: D,<55 comma-separated values>*HHHH\r\n. CRC covers ASCII D through
// the last value, excluding '*', checksum and line ending. Streamed, no big buffer.
class CrcTelemetryWriter : public Print {
 public:
  uint16_t crc = 0xFFFF;
  using Print::write;
  size_t write(uint8_t b) override {
    crc ^= (uint16_t)b << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    return Serial.write(b);
  }
};

void updateSerialLog() {
  unsigned long now = millis();
  if (now - lastSerialLogMillis < SERIAL_LOG_INTERVAL_MS) return;
  do {
    lastSerialLogMillis += SERIAL_LOG_INTERVAL_MS;
  } while (now - lastSerialLogMillis >= SERIAL_LOG_INTERVAL_MS);

  const float scalePctPerMs = 100.0f / (float)PWM_WINDOW_MS;
  const float activeElapsed = (systemState == SYS_ACTIVE && activeStartMillis > 0)
                                ? (now - activeStartMillis) / 1000.0f
                                : -1.0f;

  const unsigned long ch1Age = (channelSensingEnabled(0) && lastRtdSampleMillis[0] > 0)
                                 ? (now - lastRtdSampleMillis[0])
                                 : 0xFFFFFFFFUL;
  const unsigned long ch2Age = (channelSensingEnabled(1) && lastRtdSampleMillis[1] > 0)
                                 ? (now - lastRtdSampleMillis[1])
                                 : 0xFFFFFFFFUL;

  CrcTelemetryWriter telemetry;
  telemetrySequence++;
  telemetry.print(F("D,"));
  telemetry.print(now / 1000.0f, 3); telemetry.print(',');
  telemetry.print(activeElapsed, 3); telemetry.print(',');
  telemetry.print((uint8_t)systemState); telemetry.print(',');
  telemetry.print(systemState == SYS_ALARM ? 1 : 0); telemetry.print(',');
  telemetry.print(faultFlagsLatched); telemetry.print(',');
  telemetry.print((uint8_t)channelConfig[0].mode); telemetry.print(',');
  telemetry.print((uint8_t)channelConfig[1].mode); telemetry.print(',');

  printTemperatureOrNan(telemetry, tempRtd[0], 4); telemetry.print(',');
  printTemperatureOrNan(telemetry, tempRtd[1], 4); telemetry.print(',');
  telemetry.print(pid[0].setpoint, 4); telemetry.print(',');
  telemetry.print(pid[1].setpoint, 4); telemetry.print(',');
  telemetry.print(channelConfig[0].targetC, 4); telemetry.print(',');
  telemetry.print(channelConfig[1].targetC, 4); telemetry.print(',');

  telemetry.print(completedCommandMsLast[0] * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(completedCommandMsLast[1] * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(actualGpioDutyPct[0], 3); telemetry.print(',');
  telemetry.print(actualGpioDutyPct[1], 3); telemetry.print(',');

  telemetry.print(pid[0].pMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[0].iMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[0].dMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[1].pMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[1].iMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[1].dMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[0].unclampedMs * scalePctPerMs, 3); telemetry.print(',');
  telemetry.print(pid[1].unclampedMs * scalePctPerMs, 3); telemetry.print(',');

  telemetry.print((int)pid[0].saturationState); telemetry.print(',');
  telemetry.print((int)pid[1].saturationState); telemetry.print(',');
  telemetry.print((int)pid[0].integralClampState); telemetry.print(',');
  telemetry.print((int)pid[1].integralClampState); telemetry.print(',');
  telemetry.print(pid[0].integralClampActive ? 1 : 0); telemetry.print(',');
  telemetry.print(pid[1].integralClampActive ? 1 : 0); telemetry.print(',');
  telemetry.print(pid[0].filteredDydtCPerS, 6); telemetry.print(',');
  telemetry.print(pid[1].filteredDydtCPerS, 6); telemetry.print(',');

  telemetry.print(pid[0].lastDtS, 4); telemetry.print(',');
  telemetry.print(windowsPassedLastUpdate); telemetry.print(',');
  telemetry.print(controlGapActive ? 1 : 0); telemetry.print(',');
  telemetry.print(missedWindowCount); telemetry.print(',');
  telemetry.print(actualWindowUsLast); telemetry.print(',');
  telemetry.print(ssrHighUsLast[0]); telemetry.print(',');
  telemetry.print(ssrHighUsLast[1]); telemetry.print(',');

  telemetry.print((uint8_t)rtdState); telemetry.print(',');
  telemetry.print(ch1Age); telemetry.print(',');
  telemetry.print(ch2Age); telemetry.print(',');
  telemetry.print(rtdSampleCount[0]); telemetry.print(',');
  telemetry.print(rtdSampleCount[1]); telemetry.print(',');
  telemetry.print(rtdFailureCount[0]); telemetry.print(',');
  telemetry.print(rtdFailureCount[1]); telemetry.print(',');
  telemetry.print(rtdFaultCode[0]); telemetry.print(',');
  telemetry.print(rtdFaultCode[1]); telemetry.print(',');
  telemetry.print(loopMaxUsLast); telemetry.print(',');
  telemetry.print(telemetrySequence); telemetry.print(',');
  telemetry.print(controlWindowSequence); telemetry.print(',');
  telemetry.print(ssrCompletedWindowSequenceSeen); telemetry.print(',');
  telemetry.print(pid[0].integralHoldActive ? 1 : 0); telemetry.print(',');
  telemetry.print(pid[1].integralHoldActive ? 1 : 0);
  char checksum[5];
  snprintf(checksum, sizeof(checksum), "%04X", (unsigned)telemetry.crc);
  Serial.print('*');
  Serial.println(checksum);

  // Ensure the complete 1 Hz telemetry row has left the UART
  // before constructing the next serial message.
  Serial.flush();
}
