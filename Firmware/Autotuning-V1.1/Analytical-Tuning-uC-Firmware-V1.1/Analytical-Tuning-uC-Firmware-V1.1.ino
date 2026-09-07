// Analytical-Tuning-uC-Firmware-V1.1.ino
// TSRCT-PCB-01 Rev B / Arduino Nano Every (ATmega4809), 115200 baud.
// Companion: Analyitical-Tuning-Logger.py (V1.1 internally).
// Save in folder Analytical-Tuning-uC-Firmware-V1.1; select Nano Every.
// Dependency: LiquidCrystal_I2C with init()/backlight() API.
// Based on the supplied TSRCT_PCB_StepID_Autotune_v2 firmware; driver shared
// with nominal V1.1, derived from Adafruit MAX31865 (BSD licence).
//
// Button: IDLE -> qualified 0% baseline -> fixed open-loop heater step.
// During baseline/step: press to ABORT. After completion: press returns to IDLE.
// ALARM: repair fault/cool required sensors, then press to acknowledge to IDLE.
// Heating never restarts automatically. Only the selected channel actuates.
// The PC logger is passive and computes FOPDT / SIMC PI / iSIMC PID afterwards.
// Closing Python does not stop the heater test; use the local button to abort.
// A series thermal fuse is independent protection against a stuck-on SSR.
//
// V1.1: Pt100/430 ohm, channel selection, validated rounded step duty, recent
// baseline window, boundary timestamp, ISR duration/heartbeat cutoffs, watchdog,
// sensor requalification, repeated terminal telemetry and CRC16 framing.
// EXPERIMENTAL TEST-BENCH CHECKS PASSED
#include <Arduino.h>
#if !defined(__AVR_ATmega4809__)
#error "Select Arduino Nano Every (ATmega4809), not classic Nano."
#endif
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>

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
// USER SETTINGS -- flash one selected heater/sensor pair per identification run.
// ============================================================================
const uint8_t TEST_CHANNEL = 1;        // 1 = RTD1/CS7 -> A6; 2 = RTD2/CS6 -> A7
const bool MONITOR_OTHER_CHANNEL = true; // false permits a single-sensor installation
const float R_REF = 430.0f;            // both physical reference resistors: 430 ohm
const float RNOMINAL = 100.0f;         // both probes: Pt100
const uint8_t RTD_WIRE_COUNT = 3;      // match the physical 2/3/4-wire connection
const bool RTD_FILTER_50HZ = true;     // Australian mains; false selects 60 Hz
const float STEP_DUTY_PERCENT = 15.0f;
const uint32_t RUN_DURATION_MS = 3UL * 60UL * 60UL * 1000UL;
const uint32_t BASELINE_MIN_DURATION_MS = 120000UL;
const uint32_t BASELINE_MAX_DURATION_MS = 600000UL;
const uint8_t BASELINE_WINDOW_SAMPLES = 60;
const float BASELINE_MAX_P2P_C = 0.20f;
const float CH1_OVERTEMP_C = 85.0f;
const float CH2_OVERTEMP_C = 85.0f;

// ADVANCED SETTINGS
const uint16_t PWM_WINDOW_MS = 1000U;
const uint32_t RTD_BIAS_SETTLE_US = 10000UL;
const uint32_t RTD_CONVERSION_US = 70000UL; // >66 ms maximum for 50 Hz conversion
const uint32_t RTD_SAMPLE_INTERVAL_MS = 160UL;
const uint32_t RTD_STALE_TIMEOUT_MS = 1000UL;
const float RTD_MIN_C = -20.0f, RTD_MAX_C = 200.0f, MAX_RTD_STEP_C = 2.0f;
const uint8_t MAX_CONSECUTIVE_RTD_FAILURES = 5, RTD_RECOVERY_SAMPLES = 5;
const float ALARM_RESET_HYSTERESIS_C = 2.0f;
const uint16_t SSR_COMMAND_TIMEOUT_MS = 2500U;
const uint32_t SERIAL_LOG_INTERVAL_MS = 1000UL, HEADER_INTERVAL_MS = 30000UL;
const uint8_t TEST_INDEX = TEST_CHANNEL == 2 ? 1 : 0;
const max31865_numwires_t RTD_WIRES = RTD_WIRE_COUNT == 3 ? MAX31865_3WIRE : MAX31865_2WIRE;

const uint8_t buttonPin=2, buzzerPin=3, RTD1_CS_PIN=7, RTD2_CS_PIN=6;
const uint8_t SPI_MOSI_PIN=8, SPI_MISO_PIN=9, SPI_SCK_PIN=10;
const uint8_t ssrPin_C1=A6, ssrPin_C2=A7;
LiquidCrystal_I2C lcd(0x27,20,4);
LocalMAX31865 rtd1(RTD1_CS_PIN,SPI_MOSI_PIN,SPI_MISO_PIN,SPI_SCK_PIN);
LocalMAX31865 rtd2(RTD2_CS_PIN,SPI_MOSI_PIN,SPI_MISO_PIN,SPI_SCK_PIN);

enum SystemState : uint8_t {
  STATE_IDLE=0, STATE_BASELINE=1, STATE_STEP_RUNNING=2,
  STATE_ALARM=3, STATE_COMPLETE=4, STATE_ABORTED=5
};
enum FaultFlags : uint16_t {
  FAULT_NONE=0, FAULT_CH1_OVERTEMP=0x0001, FAULT_CH2_OVERTEMP=0x0002,
  FAULT_CH1_RTD=0x0004, FAULT_CH2_RTD=0x0008, FAULT_RTD_STALE=0x0010,
  FAULT_CONFIGURATION=0x0020, FAULT_COMMAND_TIMEOUT=0x0040,
  FAULT_WATCHDOG_RESET=0x0080
};
SystemState systemState=STATE_IDLE;
uint16_t faultFlagsLatched=0;
uint16_t stepOnMs=0; // calculated only after configuration validation
float appliedDutyPct=0;
uint8_t resetCause=0;

float tempRtd[2]={NAN,NAN}, lastValidRtd[2]={NAN,NAN};
float recoveryCandidateC[2]={NAN,NAN};
uint8_t recoveryCount[2]={0,0}, rtdFailureCount[2]={0,0}, rtdFaultCode[2]={0,0};
bool qualifiedOvertemp[2]={false,false};
uint32_t lastRtdSampleMillis[2]={0,0}, rtdSampleCount[2]={0,0};
uint8_t rtdState=0;
uint32_t rtdStateStartUs=0, rtdCycleStartMillis=0;

volatile bool ssrTimerEnabled=false, ssrCommandTimeout=false;
volatile uint16_t ssrCommandAgeMs=0, ssrWindowTickMs=0;
volatile bool ssrPhysicalState[2]={false,false};
volatile uint16_t ssrPendingOnMs[2]={0,0}, ssrActiveOnMs[2]={0,0};
volatile uint16_t ssrHighTicksCurrent[2]={0,0}, ssrHighTicksLast[2]={0,0};
volatile uint16_t ssrCommandMsLast[2]={0,0};
volatile uint32_t ssrWindowSequence=0, ssrCompletedWindowSequence=0;
volatile bool stepLatchRequested=false, ssrStepApplied=false, ssrRunComplete=false;
volatile uint32_t ssrStepStartMillis=0, ssrStepStartSequence=0, ssrStepElapsedMs=0;
uint32_t completedWindowSequence=0;
uint16_t completedCommandMs=0;
uint32_t scheduledHighUs=0;
float scheduledDutyPct=0;

uint32_t experimentStartMillis=0, stepStartMillis=0, terminalMillis=0;
uint32_t stepStartWindowSequence=0, runId=0, telemetrySequence=0;
bool stepCommandArmed=false, forceLog=true;
uint32_t lastBaselineSampleMillis=0, lastLogMillis=0, lastHeaderMillis=0;
uint32_t lastLcdMillis=0, lastHeartbeatMillis=0, lastAlarmBeepMillis=0;
uint32_t lastLoopStartUs=0, loopMaxUs=0;
bool alarmBuzzerOn=false;
float baselineSamples[BASELINE_WINDOW_SAMPLES];
uint8_t baselineIndex=0, baselineCount=0;
float baselineMeanC=NAN, baselineP2PC=NAN;
int buttonState=HIGH, lastButtonReading=HIGH;
uint32_t lastDebounceTime=0;

// Explicit prototypes keep Arduino's sketch preprocessor independent of custom types.
bool validateConfiguration();
bool channelRequired(uint8_t);
float overtempThreshold(uint8_t);
bool rtdFresh(uint8_t);
void resetBaselineWindow();
void acceptRtdSample(uint8_t,float);
float validateAsyncTemperature(LocalMAX31865&,uint16_t,bool,uint8_t);
void serviceRTDAcquisition();
void configureSsrHardwareTimer();
void startSsrHardwareWindows();
void stopSsrHardwareWindows();
void snapshotCompletedSsrWindow();
uint16_t safetyFaults(bool);
void enterAlarm(uint16_t,const __FlashStringHelper*);
void evaluateSafety();
bool beginBaseline();
void abortRun(const __FlashStringHelper*);
void serviceBaselineQualification();
void serviceStepActivation();
bool acknowledgeAlarmIfSafe();
bool checkButtonPress();
void emitHeader();
void logSerial();
void updateLCD();
void printValue(Print&,float,uint8_t);
const char* stateName(uint8_t);

bool validateConfiguration() {
  if (TEST_CHANNEL < 1 || TEST_CHANNEL > 2 || RTD_WIRE_COUNT < 2 || RTD_WIRE_COUNT > 4) return false;
  if (!isfinite(R_REF) || !isfinite(RNOMINAL) || RNOMINAL <= 0 || R_REF <= RNOMINAL) return false;
  if (!isfinite(STEP_DUTY_PERCENT) || STEP_DUTY_PERCENT <= 0 || STEP_DUTY_PERCENT > 100) return false;
  if (STEP_DUTY_PERCENT * PWM_WINDOW_MS / 100.0f < 0.5f) return false;
  if (RUN_DURATION_MS < 1000UL || RUN_DURATION_MS > 86400000UL) return false;
  if (BASELINE_MIN_DURATION_MS < (uint32_t)BASELINE_WINDOW_SAMPLES * 1000UL ||
      BASELINE_MAX_DURATION_MS <= BASELINE_MIN_DURATION_MS || BASELINE_MAX_DURATION_MS > 3600000UL) return false;
  if (!isfinite(BASELINE_MAX_P2P_C) || BASELINE_MAX_P2P_C <= 0) return false;
  if (!isfinite(CH1_OVERTEMP_C) || !isfinite(CH2_OVERTEMP_C) ||
      CH1_OVERTEMP_C <= RTD_MIN_C || CH2_OVERTEMP_C <= RTD_MIN_C ||
      CH1_OVERTEMP_C > RTD_MAX_C || CH2_OVERTEMP_C > RTD_MAX_C) return false;
  return RTD_CONVERSION_US >= (RTD_FILTER_50HZ ? 66000UL : 55000UL);
}

bool channelRequired(uint8_t ch) { return ch == TEST_INDEX || MONITOR_OTHER_CHANNEL; }
float overtempThreshold(uint8_t ch) { return ch == 0 ? CH1_OVERTEMP_C : CH2_OVERTEMP_C; }
bool rtdFresh(uint8_t ch) {
  return lastRtdSampleMillis[ch] != 0 &&
      (uint32_t)(millis()-lastRtdSampleMillis[ch]) <= RTD_STALE_TIMEOUT_MS;
}

float validateAsyncTemperature(LocalMAX31865 &rtd,uint16_t raw,bool fault,uint8_t code) {
  if (fault || code || raw == 0 || raw >= 32767U || !rtd.configurationMatches() ||
      (rtd.readConfig() & MAX31865_CONFIG_1SHOT)) {
    rtd.clearFault();
    return NAN;
  }
  float value=rtd.calculateTemperature(raw,RNOMINAL,R_REF);
  return !isfinite(value) || value < RTD_MIN_C || value > RTD_MAX_C ? NAN : value;
}

void acceptRtdSample(uint8_t ch,float value) {
  qualifiedOvertemp[ch]=isfinite(value) && value >= overtempThreshold(ch);
  float accepted=NAN;
  if (systemState == STATE_BASELINE || systemState == STATE_STEP_RUNNING) {
    recoveryCount[ch]=0;
    recoveryCandidateC[ch]=NAN;
    if (isfinite(value) && (!isfinite(lastValidRtd[ch]) || fabs(value-lastValidRtd[ch]) <= MAX_RTD_STEP_C))
      accepted=lastValidRtd[ch]=value;
  } else if (isfinite(value)) {
    if (!isfinite(recoveryCandidateC[ch]) || fabs(value-recoveryCandidateC[ch]) > MAX_RTD_STEP_C)
      recoveryCount[ch]=1;
    else if (recoveryCount[ch] < RTD_RECOVERY_SAMPLES) recoveryCount[ch]++;
    recoveryCandidateC[ch]=value;
    if (recoveryCount[ch] >= RTD_RECOVERY_SAMPLES) accepted=lastValidRtd[ch]=value;
  } else {
    recoveryCount[ch]=0;
    recoveryCandidateC[ch]=NAN;
  }
  tempRtd[ch]=accepted;
  rtdSampleCount[ch]++;
  if (isfinite(accepted)) {
    lastRtdSampleMillis[ch]=millis(); // age of valid acquisition, never of a failed attempt
    rtdFailureCount[ch]=0;
  } else {
    if (rtdFailureCount[ch] < 255) rtdFailureCount[ch]++;
    if (channelRequired(ch) && systemState == STATE_BASELINE && !stepCommandArmed)
      resetBaselineWindow();
  }
}

void serviceRTDAcquisition() {
  uint32_t nowUs=micros(), nowMs=millis();
  if (rtdState == 0 && (uint32_t)(nowMs-rtdCycleStartMillis) >= RTD_SAMPLE_INTERVAL_MS) {
    rtdCycleStartMillis=nowMs;
    rtd1.clearFault(); rtd2.clearFault();
    rtd1.enableBias(true); rtd2.enableBias(true);
    rtdStateStartUs=micros(); rtdState=1;
  } else if (rtdState == 1 && (uint32_t)(nowUs-rtdStateStartUs) >= RTD_BIAS_SETTLE_US) {
    rtd1.triggerOneShot(); rtd2.triggerOneShot();
    rtdStateStartUs=micros(); rtdState=2;
  } else if (rtdState == 2 && (uint32_t)(nowUs-rtdStateStartUs) >= RTD_CONVERSION_US) {
    bool f1=false,f2=false;
    uint16_t a=rtd1.readRawRTD(f1,rtdFaultCode[0]), b=rtd2.readRawRTD(f2,rtdFaultCode[1]);
    rtd1.enableBias(false); rtd2.enableBias(false);
    acceptRtdSample(0,validateAsyncTemperature(rtd1,a,f1,rtdFaultCode[0]));
    acceptRtdSample(1,validateAsyncTemperature(rtd2,b,f2,rtdFaultCode[1]));
    rtdState=0;
  }
}

static inline void ssrWriteFromISR(uint8_t ch,bool on) {
  if (on != ssrPhysicalState[ch]) {
    ssrPhysicalState[ch]=on;
    digitalWrite(ch == 0 ? ssrPin_C1 : ssrPin_C2,on ? HIGH : LOW);
  }
}

ISR(TCB2_INT_vect) {
  TCB2.INTFLAGS=TCB_CAPT_bm;
  if (!ssrTimerEnabled) {
    ssrWriteFromISR(0,false); ssrWriteFromISR(1,false);
    ssrWindowTickMs=0;
    return;
  }
  // Both deadlines run independently of foreground LCD/UART/I2C work.
  if (++ssrCommandAgeMs >= SSR_COMMAND_TIMEOUT_MS ||
      (ssrStepApplied && ssrStepElapsedMs >= RUN_DURATION_MS)) {
    if (ssrCommandAgeMs >= SSR_COMMAND_TIMEOUT_MS) ssrCommandTimeout=true;
    else ssrRunComplete=true;
    ssrTimerEnabled=false;
    for (uint8_t ch=0;ch<2;ch++) {
      ssrPendingOnMs[ch]=ssrActiveOnMs[ch]=0;
      ssrWriteFromISR(ch,false);
    }
    return;
  }
  if (ssrWindowTickMs == 0) {
    ssrWindowSequence++;
    for (uint8_t ch=0;ch<2;ch++) {
      ssrActiveOnMs[ch]=ch == TEST_INDEX ? ssrPendingOnMs[ch] : 0;
      ssrHighTicksCurrent[ch]=0;
    }
    if (stepLatchRequested && !ssrStepApplied && ssrActiveOnMs[TEST_INDEX] > 0) {
      // Time zero is recorded in the actual scheduler boundary, before GPIO ON.
      ssrStepStartMillis=millis();
      ssrStepStartSequence=ssrWindowSequence;
      ssrStepElapsedMs=0;
      ssrStepApplied=true;
    }
    for (uint8_t ch=0;ch<2;ch++) ssrWriteFromISR(ch,ssrActiveOnMs[ch] > 0);
  }
  for (uint8_t ch=0;ch<2;ch++) if (ssrPhysicalState[ch]) ssrHighTicksCurrent[ch]++;
  ssrWindowTickMs++;
  for (uint8_t ch=0;ch<2;ch++)
    if (ssrActiveOnMs[ch] < PWM_WINDOW_MS && ssrWindowTickMs >= ssrActiveOnMs[ch]) ssrWriteFromISR(ch,false);
  if (ssrWindowTickMs >= PWM_WINDOW_MS) {
    for (uint8_t ch=0;ch<2;ch++) {
      ssrHighTicksLast[ch]=ssrHighTicksCurrent[ch];
      ssrCommandMsLast[ch]=ssrActiveOnMs[ch];
    }
    ssrCompletedWindowSequence=ssrWindowSequence;
    ssrWindowTickMs=0;
  }
  if (ssrStepApplied) ssrStepElapsedMs++;
}

void configureSsrHardwareTimer() {
  const uint32_t counts=(F_CPU/2UL)/1000UL;
  static_assert(counts >= 1UL && counts <= 65536UL,"TCB2 1 ms count out of range");
  static_assert(PWM_WINDOW_MS == 1000U,"V1.1 requires a 1000 ms SSR window");
  static_assert(BASELINE_WINDOW_SAMPLES >= 2 && BASELINE_WINDOW_SAMPLES <= 120,"Baseline window must be 2..120 samples");
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCB2.CTRLA=0; TCB2.CTRLB=TCB_CNTMODE_INT_gc;
    TCB2.CCMP=(uint16_t)(counts-1UL); TCB2.CNT=0;
    TCB2.INTFLAGS=TCB_CAPT_bm; TCB2.INTCTRL=TCB_CAPT_bm;
    TCB2.CTRLA=TCB_CLKSEL_CLKDIV2_gc|TCB_ENABLE_bm;
  }
}

void stopSsrHardwareWindows() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ssrTimerEnabled=false; stepLatchRequested=false;
    for (uint8_t ch=0;ch<2;ch++) {
      ssrPendingOnMs[ch]=ssrActiveOnMs[ch]=0;
      ssrHighTicksCurrent[ch]=ssrHighTicksLast[ch]=ssrCommandMsLast[ch]=0;
      ssrPhysicalState[ch]=false;
    }
    digitalWrite(ssrPin_C1,LOW); digitalWrite(ssrPin_C2,LOW);
  }
  completedCommandMs=0; scheduledHighUs=0; scheduledDutyPct=0;
}

void startSsrHardwareWindows() {
  stopSsrHardwareWindows();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ssrWindowSequence=ssrCompletedWindowSequence=0;
    ssrWindowTickMs=ssrCommandAgeMs=0;
    ssrStepApplied=ssrRunComplete=false;
    ssrStepStartMillis=ssrStepStartSequence=ssrStepElapsedMs=0;
    ssrTimerEnabled=true;
  }
  completedWindowSequence=0;
  lastHeartbeatMillis=millis();
}

void snapshotCompletedSsrWindow() {
  uint32_t sequence;
  uint16_t high,command;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sequence=ssrCompletedWindowSequence;
    high=ssrHighTicksLast[TEST_INDEX]; command=ssrCommandMsLast[TEST_INDEX];
  }
  if (sequence == completedWindowSequence) return;
  completedWindowSequence=sequence;
  completedCommandMs=command; scheduledHighUs=(uint32_t)high*1000UL;
  scheduledDutyPct=high*100.0f/PWM_WINDOW_MS;
}

uint16_t safetyFaults(bool resetCheck) {
  uint16_t flags=0;
  if (!validateConfiguration()) flags|=FAULT_CONFIGURATION;
  if (ssrCommandTimeout && !resetCheck) flags|=FAULT_COMMAND_TIMEOUT;
  bool active=systemState == STATE_BASELINE || systemState == STATE_STEP_RUNNING;
  for (uint8_t ch=0;ch<2;ch++) {
    if (!channelRequired(ch)) continue;
    float limit=overtempThreshold(ch)-(resetCheck ? ALARM_RESET_HYSTERESIS_C : 0.0f);
    if (qualifiedOvertemp[ch] || (isfinite(tempRtd[ch]) && tempRtd[ch] >= limit))
      flags|=ch == 0 ? FAULT_CH1_OVERTEMP : FAULT_CH2_OVERTEMP;
    if ((resetCheck && !isfinite(tempRtd[ch])) || (active && rtdFailureCount[ch] >= MAX_CONSECUTIVE_RTD_FAILURES))
      flags|=ch == 0 ? FAULT_CH1_RTD : FAULT_CH2_RTD;
    if ((active || resetCheck) && !rtdFresh(ch)) flags|=FAULT_RTD_STALE;
  }
  return flags;
}

void enterAlarm(uint16_t flags,const __FlashStringHelper *reason) {
  stopSsrHardwareWindows();
  faultFlagsLatched|=flags; systemState=STATE_ALARM;
  stepCommandArmed=false; terminalMillis=millis(); forceLog=true;
  Serial.print(F("# EVENT: ")); Serial.print(reason);
  Serial.print(F(",fault_flags=0x")); Serial.println(faultFlagsLatched,HEX);
}
void evaluateSafety() {
  if (systemState == STATE_ALARM) return;
  uint16_t flags=safetyFaults(false);
  if (flags) enterAlarm(flags,F("GLOBAL_SAFETY_TRIP - BOTH_SSR_DISABLED"));
}

void resetBaselineWindow() {
  baselineIndex=baselineCount=0;
  baselineMeanC=baselineP2PC=NAN;
}

bool beginBaseline() {
  if (!validateConfiguration() || safetyFaults(true)) return false;
  stepOnMs=(uint16_t)lroundf(STEP_DUTY_PERCENT*PWM_WINDOW_MS/100.0f);
  appliedDutyPct=stepOnMs*100.0f/PWM_WINDOW_MS;
  stopSsrHardwareWindows(); resetBaselineWindow();
  faultFlagsLatched=0; stepCommandArmed=false;
  experimentStartMillis=millis(); terminalMillis=stepStartMillis=0;
  stepStartWindowSequence=0; runId++;
  lastBaselineSampleMillis=millis()-1000UL;
  systemState=STATE_BASELINE; forceLog=true;
  startSsrHardwareWindows();
  emitHeader();
  Serial.println(F("# EVENT: BASELINE_STARTED"));
  return true;
}

void serviceBaselineQualification() {
  if (systemState != STATE_BASELINE || stepCommandArmed) return;
  uint32_t now=millis(), elapsed=now-experimentStartMillis;
  if (elapsed >= BASELINE_MAX_DURATION_MS) { abortRun(F("BASELINE_NOT_STABLE_TIMEOUT")); return; }
  uint32_t dt=now-lastBaselineSampleMillis;
  if (dt < 1000UL) return;
  lastBaselineSampleMillis=now;
  if (dt > 1500UL) resetBaselineWindow();
  for (uint8_t ch=0;ch<2;ch++) {
    if (channelRequired(ch) && (!isfinite(tempRtd[ch]) || !rtdFresh(ch))) {
      resetBaselineWindow(); return;
    }
  }
  baselineSamples[baselineIndex]=tempRtd[TEST_INDEX];
  baselineIndex=(baselineIndex+1U)%BASELINE_WINDOW_SAMPLES;
  if (baselineCount < BASELINE_WINDOW_SAMPLES) baselineCount++;
  float sum=0, lo=RTD_MAX_C, hi=RTD_MIN_C;
  for (uint8_t i=0;i<baselineCount;i++) {
    sum+=baselineSamples[i]; lo=min(lo,baselineSamples[i]); hi=max(hi,baselineSamples[i]);
  }
  baselineMeanC=sum/baselineCount; baselineP2PC=hi-lo;
  if (elapsed < BASELINE_MIN_DURATION_MS || baselineCount < BASELINE_WINDOW_SAMPLES ||
      baselineP2PC > BASELINE_MAX_P2P_C) return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ssrCommandTimeout) return;
    ssrPendingOnMs[TEST_INDEX]=stepOnMs;
    ssrPendingOnMs[1-TEST_INDEX]=0;
    stepLatchRequested=true;
  }
  stepCommandArmed=true;
  Serial.println(F("# EVENT: BASELINE_QUALIFIED_STEP_ARMED"));
}

void serviceStepActivation() {
  if (systemState != STATE_BASELINE || !stepCommandArmed) return;
  bool applied;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    applied=ssrStepApplied;
    if (applied) {
      stepStartMillis=ssrStepStartMillis;
      stepStartWindowSequence=ssrStepStartSequence;
    }
  }
  if (!applied) return;
  systemState=STATE_STEP_RUNNING; stepCommandArmed=false; forceLog=true;
  Serial.print(F("# EVENT: STEP_APPLIED,start_uptime_ms=")); Serial.print(stepStartMillis);
  Serial.print(F(",start_window=")); Serial.print(stepStartWindowSequence);
  Serial.print(F(",applied_duty_pct=")); Serial.println(appliedDutyPct,3);
}

void abortRun(const __FlashStringHelper *reason) {
  stopSsrHardwareWindows(); stepCommandArmed=false;
  systemState=STATE_ABORTED; terminalMillis=millis(); forceLog=true;
  Serial.print(F("# EVENT: TEST_ABORTED,reason=")); Serial.println(reason);
}

bool acknowledgeAlarmIfSafe() {
  uint16_t remaining=safetyFaults(true);
  if (remaining) {
    Serial.print(F("# EVENT: ALARM_RESET_BLOCKED,fault_flags=0x")); Serial.println(remaining,HEX);
    return false;
  }
  stopSsrHardwareWindows();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ssrCommandTimeout=false; }
  faultFlagsLatched=0; noTone(buzzerPin); alarmBuzzerOn=false;
  systemState=STATE_IDLE; experimentStartMillis=stepStartMillis=terminalMillis=0; forceLog=true;
  Serial.println(F("# EVENT: ALARM_ACKNOWLEDGED_RETURNED_TO_IDLE"));
  return true;
}

bool checkButtonPress() {
  bool pressed=false; int reading=digitalRead(buttonPin);
  if (reading != lastButtonReading) lastDebounceTime=millis();
  if ((uint32_t)(millis()-lastDebounceTime) > 50UL && reading != buttonState) {
    buttonState=reading; pressed=buttonState == LOW;
  }
  lastButtonReading=reading; return pressed;
}

const char* stateName(uint8_t state) {
  switch(state) {
    case 0:return "IDLE"; case 1:return "BASELINE"; case 2:return "STEP";
    case 3:return "ALARM"; case 4:return "COMPLETE"; case 5:return "ABORTED";
    default:return "INVALID";
  }
}

void printValue(Print &out,float value,uint8_t digits) {
  if (isfinite(value)) out.print(value,digits); else out.print(F("nan"));
}

void emitHeader() {
  lastHeaderMillis=millis();
  Serial.println(F("# FIRMWARE: Analytical-Tuning-uC-Firmware V1.1"));
  Serial.println(F("# CONFIG: schema=analytical-1.1,crc=CRC16_CCITT_FALSE,firmware_version=V1.1"));
  Serial.print(F("# CONFIG: test_channel=")); Serial.print(TEST_CHANNEL);
  Serial.print(F(",monitor_other=")); Serial.print(MONITOR_OTHER_CHANNEL ? 1 : 0);
  Serial.print(F(",requested_duty_pct=")); Serial.print(STEP_DUTY_PERCENT,4);
  Serial.print(F(",applied_duty_pct=")); Serial.print(appliedDutyPct,3);
  Serial.print(F(",pwm_window_ms=")); Serial.print(PWM_WINDOW_MS);
  Serial.print(F(",duration_s=")); Serial.println(RUN_DURATION_MS/1000.0f,3);
  Serial.print(F("# CONFIG: rnominal_ohm=")); Serial.print(RNOMINAL,1);
  Serial.print(F(",rref_ohm=")); Serial.print(R_REF,1);
  Serial.print(F(",rtd_wire_count=")); Serial.print(RTD_WIRE_COUNT);
  Serial.print(F(",rtd_filter_hz=")); Serial.print(RTD_FILTER_50HZ ? 50 : 60);
  Serial.print(F(",rtd_conversion_us=")); Serial.print(RTD_CONVERSION_US);
  Serial.print(F(",reset_cause=")); Serial.println(resetCause);
  Serial.print(F("# CONFIG: baseline_min_s=")); Serial.print(BASELINE_MIN_DURATION_MS/1000.0f,3);
  Serial.print(F(",baseline_window_samples=")); Serial.print(BASELINE_WINDOW_SAMPLES);
  Serial.print(F(",baseline_max_p2p_C=")); Serial.print(BASELINE_MAX_P2P_C,4);
  Serial.print(F(",baseline_timeout_s=")); Serial.println(BASELINE_MAX_DURATION_MS/1000.0f,3);
  Serial.print(F("# CONFIG: ch1_overtemp_C=")); Serial.print(CH1_OVERTEMP_C,3);
  Serial.print(F(",ch2_overtemp_C=")); Serial.print(CH2_OVERTEMP_C,3);
  Serial.print(F(",rtd_stale_ms=")); Serial.print(RTD_STALE_TIMEOUT_MS);
  Serial.print(F(",command_timeout_ms=")); Serial.println(SSR_COMMAND_TIMEOUT_MS);
  Serial.println(F("# NOTE: CH1=RTD1/CS7->A6; CH2=RTD2/CS6->A7; unselected heater always OFF."));
  Serial.println(F("# NOTE: scheduled duty is ISR bookkeeping, not an independent GPIO/power measurement."));
  Serial.println(F("# FIELDS: uptime_s,experiment_s,step_s,ch1_C,ch2_C,test_channel,monitor_other,state,alarm,fault_flags,ch1_valid,ch2_valid,ch1_rtd_age_ms,ch2_rtd_age_ms,ch1_rtd_sample_count,ch2_rtd_sample_count,ch1_rtd_failure_count,ch2_rtd_failure_count,ch1_rtd_fault_code,ch2_rtd_fault_code,commanded_duty_pct,scheduled_gpio_duty_pct,completed_command_duty_pct,scheduled_window_us,ssr_high_us,window_sequence,completed_window_sequence,step_start_window_sequence,step_start_uptime_ms,run_id,telemetry_seq,rtd_state,loop_max_us,baseline_mean_C,baseline_p2p_C,baseline_count,requested_duty_pct,applied_duty_pct,target_duration_s"));
}

// CRC-16/CCITT-FALSE, ASCII bytes from D through last value, excluding *HHHH/CRLF.
class CrcTelemetryWriter : public Print {
 public:
  uint16_t crc=0xFFFF;
  using Print::write;
  size_t write(uint8_t b) override {
    crc^=(uint16_t)b<<8;
    for (uint8_t i=0;i<8;i++) crc=(crc&0x8000U) ? (uint16_t)((crc<<1)^0x1021U) : (uint16_t)(crc<<1);
    return Serial.write(b);
  }
};

void logSerial() {
  uint32_t now=millis();
  if (!forceLog && (uint32_t)(now-lastLogMillis) < SERIAL_LOG_INTERVAL_MS) return;
  forceLog=false; lastLogMillis=now; telemetrySequence++;
  uint32_t stamp=terminalMillis ? terminalMillis : now;
  float experimentS=experimentStartMillis ? (uint32_t)(stamp-experimentStartMillis)/1000.0f : -1.0f;
  float stepS=stepStartMillis ? (uint32_t)(stamp-stepStartMillis)/1000.0f : -1.0f;
  uint32_t sequence; uint16_t currentMs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sequence=ssrWindowSequence; currentMs=ssrActiveOnMs[TEST_INDEX]; }
  CrcTelemetryWriter out;
  out.print(F("D,"));
  out.print(now/1000.0f,3); out.print(',');
  out.print(experimentS,3); out.print(',');
  out.print(stepS,3); out.print(',');
  printValue(out,tempRtd[0],4); out.print(',');
  printValue(out,tempRtd[1],4); out.print(',');
  out.print(TEST_CHANNEL); out.print(',');
  out.print(MONITOR_OTHER_CHANNEL ? 1 : 0); out.print(',');
  out.print((uint8_t)systemState); out.print(',');
  out.print(systemState == STATE_ALARM ? 1 : 0); out.print(',');
  out.print(faultFlagsLatched); out.print(',');
  out.print(isfinite(tempRtd[0]) && rtdFresh(0) ? 1 : 0); out.print(',');
  out.print(isfinite(tempRtd[1]) && rtdFresh(1) ? 1 : 0); out.print(',');
  out.print(lastRtdSampleMillis[0] ? (uint32_t)(now-lastRtdSampleMillis[0]) : 0xFFFFFFFFUL); out.print(',');
  out.print(lastRtdSampleMillis[1] ? (uint32_t)(now-lastRtdSampleMillis[1]) : 0xFFFFFFFFUL); out.print(',');
  out.print(rtdSampleCount[0]); out.print(',');
  out.print(rtdSampleCount[1]); out.print(',');
  out.print(rtdFailureCount[0]); out.print(',');
  out.print(rtdFailureCount[1]); out.print(',');
  out.print(rtdFaultCode[0]); out.print(',');
  out.print(rtdFaultCode[1]); out.print(',');
  out.print(currentMs*100.0f/PWM_WINDOW_MS,3); out.print(',');
  out.print(scheduledDutyPct,3); out.print(',');
  out.print(completedCommandMs*100.0f/PWM_WINDOW_MS,3); out.print(',');
  out.print(1000000UL); out.print(',');
  out.print(scheduledHighUs); out.print(',');
  out.print(sequence); out.print(',');
  out.print(completedWindowSequence); out.print(',');
  out.print(stepStartWindowSequence); out.print(',');
  out.print(stepStartMillis); out.print(',');
  out.print(runId); out.print(',');
  out.print(telemetrySequence); out.print(',');
  out.print(rtdState); out.print(',');
  out.print(loopMaxUs); out.print(',');
  printValue(out,baselineMeanC,4); out.print(',');
  printValue(out,baselineP2PC,4); out.print(',');
  out.print(baselineCount); out.print(',');
  out.print(STEP_DUTY_PERCENT,4); out.print(',');
  out.print(appliedDutyPct,3); out.print(',');
  out.print(RUN_DURATION_MS/1000.0f,3);
  char checksum[5]; snprintf(checksum,sizeof(checksum),"%04X",(unsigned)out.crc);
  Serial.print('*'); Serial.println(checksum);
}

void updateLCD() {
  if ((uint32_t)(millis()-lastLcdMillis) < 500UL) return;
  lastLcdMillis=millis();
  char line[21],a[8],b[8];
  if (isfinite(tempRtd[0])) dtostrf(tempRtd[0],5,1,a); else strcpy(a," --.-");
  if (isfinite(tempRtd[1])) dtostrf(tempRtd[1],5,1,b); else strcpy(b," --.-");
  snprintf(line,sizeof(line),"TUNING V1.1 CH%u     ",(unsigned)TEST_CHANNEL); lcd.setCursor(0,0); lcd.print(line);
  snprintf(line,sizeof(line),"1:%5.5sC 2:%5.5sC   ",a,b); lcd.setCursor(0,1); lcd.print(line);
  snprintf(line,sizeof(line),"%-20s",stateName(systemState)); lcd.setCursor(0,2); lcd.print(line);
  if (systemState == STATE_ALARM) snprintf(line,sizeof(line),"FAULT 0x%04X        ",(unsigned)faultFlagsLatched);
  else if (systemState == STATE_STEP_RUNNING) snprintf(line,sizeof(line),"Step %5lu s        ",
      min(99999UL,(unsigned long)((uint32_t)(millis()-stepStartMillis)/1000UL)));
  else if (systemState == STATE_BASELINE) snprintf(line,sizeof(line),"Baseline %2u/%2u     ",(unsigned)baselineCount,(unsigned)BASELINE_WINDOW_SAMPLES);
  else snprintf(line,sizeof(line),"Press button        ");
  lcd.setCursor(0,3); lcd.print(line);
}

void setup() {
  // OFF precedes all UART delays, I2C and sensor initialisation.
  digitalWrite(ssrPin_C1,LOW); digitalWrite(ssrPin_C2,LOW);
  pinMode(ssrPin_C1,OUTPUT); pinMode(ssrPin_C2,OUTPUT);
  pinMode(buzzerPin,OUTPUT); digitalWrite(buzzerPin,LOW);
  pinMode(buttonPin,INPUT); // external PCB pull-up
  resetCause=RSTCTRL.RSTFR; RSTCTRL.RSTFR=resetCause;
  wdt_reset(); wdt_disable(); wdt_enable(WDT_PERIOD_8KCLK_gc); // approximately 8 s
  Serial.begin(115200); delay(300);
  Wire.begin(); lcd.init(); lcd.backlight(); lcd.clear();
  Serial.print(F("# RTD1_INIT: ")); Serial.println(rtd1.begin(RTD_WIRES,RTD_FILTER_50HZ) ? F("OK") : F("FAILED"));
  Serial.print(F("# RTD2_INIT: ")); Serial.println(rtd2.begin(RTD_WIRES,RTD_FILTER_50HZ) ? F("OK") : F("FAILED"));
  configureSsrHardwareTimer(); stopSsrHardwareWindows();
  rtdCycleStartMillis=millis()-RTD_SAMPLE_INTERVAL_MS;
  if (validateConfiguration()) {
    stepOnMs=(uint16_t)lroundf(STEP_DUTY_PERCENT*PWM_WINDOW_MS/100.0f);
    appliedDutyPct=stepOnMs*100.0f/PWM_WINDOW_MS;
  }
  emitHeader();
  if (!validateConfiguration()) enterAlarm(FAULT_CONFIGURATION,F("CONFIGURATION_ERROR"));
  else if (resetCause & RSTCTRL_WDRF_bm) enterAlarm(FAULT_WATCHDOG_RESET,F("WATCHDOG_RESET"));
  else Serial.println(F("# EVENT: READY_IDLE - PRESS BUTTON TO START BASELINE"));
}

void loop() {
  uint32_t nowUs=micros();
  if (lastLoopStartUs) loopMaxUs=max(loopMaxUs,(uint32_t)(nowUs-lastLoopStartUs));
  lastLoopStartUs=nowUs;
  serviceRTDAcquisition(); snapshotCompletedSsrWindow(); serviceStepActivation(); evaluateSafety();
  if (checkButtonPress()) {
    if (systemState == STATE_IDLE) {
      if (!beginBaseline()) Serial.println(F("# EVENT: TEST_START_REJECTED - CHECK CONFIGURATION AND REQUIRED SENSORS"));
    } else if (systemState == STATE_BASELINE || systemState == STATE_STEP_RUNNING) abortRun(F("USER_ABORT"));
    else if (systemState == STATE_ALARM) acknowledgeAlarmIfSafe();
    else {
      stopSsrHardwareWindows(); systemState=STATE_IDLE;
      experimentStartMillis=stepStartMillis=terminalMillis=0; forceLog=true;
      Serial.println(F("# EVENT: RESET_TO_IDLE"));
    }
  }
  serviceBaselineQualification();
  if (systemState == STATE_STEP_RUNNING && ssrRunComplete) {
    stopSsrHardwareWindows(); systemState=STATE_COMPLETE;
    terminalMillis=stepStartMillis+RUN_DURATION_MS; forceLog=true;
    Serial.println(F("# EVENT: RUN_COMPLETE - BOTH_SSR_DISABLED"));
  }
  if (systemState == STATE_ALARM && (uint32_t)(millis()-lastAlarmBeepMillis) >= 300UL) {
    lastAlarmBeepMillis=millis(); alarmBuzzerOn=!alarmBuzzerOn;
    if (alarmBuzzerOn) tone(buzzerPin,2000); else noTone(buzzerPin);
  } else if (systemState != STATE_ALARM && alarmBuzzerOn) { noTone(buzzerPin); alarmBuzzerOn=false; }
  updateLCD();
  if ((uint32_t)(millis()-lastHeaderMillis) >= HEADER_INTERVAL_MS) emitHeader();
  logSerial(); // all states, including repeated machine-readable terminal records
  if ((systemState == STATE_BASELINE || systemState == STATE_STEP_RUNNING) &&
      (uint32_t)(millis()-lastHeartbeatMillis) >= 500UL) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { if (!ssrCommandTimeout) ssrCommandAgeMs=0; }
    lastHeartbeatMillis=millis();
  }
  wdt_reset(); // serviced only after a complete foreground iteration
}
