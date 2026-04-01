#include <Arduino.h>
#include <math.h>

// ============================================================
//  ESP32-S3 BIN CAROUSEL (TB6600 / DRV8825) + GATE + INVENTORY
//  CLEANED: keeps BOOT/EN diagnostics, removes switch/index spam.
//  UPDATED: latched E-STOP using input 'e' during motor motion.
//           After E-STOP, only 'quit' is accepted.
// ============================================================

// ============================================================
//                    PIN / HW CONFIG
// ============================================================

// ===== Motor driver pins =====
const int PUL_PIN = 21;
const int DIR_PIN = 3;
const int EN_PIN  = 2;   // Stepper driver enable pin

// ===== ESP32 boot button pin =====
constexpr int BOOT_BTN_PIN = 0;  // GPIO0 strap/button
RTC_DATA_ATTR uint32_t bootCount = 0;

// ===== Index switch =====
const int INDEX_PIN          = 13;
const int INDEX_ACTIVE_LEVEL = LOW;   // INPUT_PULLUP => pressed = LOW

// ===== Break-beam gate =====
const int BEAM_PIN                    = 9;
const bool BEAM_ACTIVE_LOW            = true;
const unsigned long BEAM_DEBOUNCE_MS  = 20;
const unsigned long BEAM_ARM_DELAY_MS = 2000;

// ===== Ultrasonic =====
const int TRIG_PIN = 15;
const int ECHO_PIN = 17;

const bool EN_ACTIVE_LOW = true;

// ===== Stepper config =====
const int  MICROSTEP          = 8;
const int  FULL_STEPS_PER_REV = 200;
const long STEPS_PER_REV      = (long)FULL_STEPS_PER_REV * MICROSTEP;

const unsigned int PULSE_HIGH_US = 20;
const unsigned int EN_SETTLE_MS  = 2;

// ============================================================
//                         SPEED
// ============================================================

const unsigned int SEEK_DELAY_US     = 1500;
const unsigned int OFFSET_DELAY_US   = 1500;
const unsigned int APPROACH_DELAY_US = 2600;
const int          APPROACH_WINDOW_STEPS = 140;

const bool CCW_DIR_LEVEL = LOW;

// ============================================================
//              SWITCH FILTERING / SAFETY
// ============================================================

const int  PRESS_CONFIRM_POLLS   = 5;
const int  RELEASE_CONFIRM_POLLS = 7;
const unsigned int POLL_GAP_US   = 250;

const long MAX_STEPS_TO_FIND_INDEX = STEPS_PER_REV * 3;
const int  POST_HIT_ESCAPE_STEPS   = 30;

// ============================================================
//                SINGLE "OFFSET KNOB" (DEGREES)
// ============================================================

const float INDEX_TO_STOP_DEG = 33.0f;
const float TRIM_DEG          = -18.0f;

// ============================================================
//                 HOMING: DOUBLE-PRESS DETECTION
// ============================================================

const float ENCODER_GAP_RATIO  = 0.60f;
const int   HOME_SAMPLE_EVENTS = 10;

// ============================================================
//                 ULTRASONIC ROBUST AVERAGE (4s)
// ============================================================

const unsigned long US_WINDOW_MS       = 4000;
const unsigned long US_PING_PERIOD_MS  = 25;
const unsigned long US_ECHO_TIMEOUT_US = 12000UL;

const int   US_BLOCK_SIZE     = 7;
const float US_TRIM_FRACTION  = 0.20f;

// ============================================================
//            INVENTORY THRESHOLD (TUNABLE, CM)
// ============================================================

const float INVENTORY_THRESH_CM = 11.5f;

// ============================================================
//                      UART2 (JETSON) SETUP
// ============================================================

static const uint32_t JETSON_BAUD = 115200;
const int JETSON_UART_TX = 8;   // ESP32 TX -> Jetson RX
const int JETSON_UART_RX = 11;  // ESP32 RX <- Jetson TX

HardwareSerial Jetson(2);

// ============================================================
//                        STATE
// ============================================================

bool homed = false;
int  currentBin = -1;
long encoderGapThresholdSteps = 0;
String line;

bool beamStableBlocked  = false;
bool beamLastRawBlocked = false;
unsigned long beamLastChangeMs = 0;

bool gateLockout = false;
unsigned long gateOpenedAtMs = 0;

bool inventoryPending = false;
int  lastSelectedBin  = -1;

// BOOT button runtime edge detection
bool bootBtnLast = HIGH;

// Motor driver state tracker
bool driverEnabledState = false;

// E-STOP state
bool estopLatched = false;
bool motionActive = false;

// ============================================================
//                  PRINT/IO HELPERS
// ============================================================

inline void hostPrint(const __FlashStringHelper *s) { Jetson.print(s); Serial.print(s); }
inline void hostPrint(const String &s)              { Jetson.print(s); Serial.print(s); }
inline void hostPrint(const char *s)                { Jetson.print(s); Serial.print(s); }
inline void hostPrint(long v)                       { Jetson.print(v); Serial.print(v); }
inline void hostPrint(int v)                        { Jetson.print(v); Serial.print(v); }
inline void hostPrint(float v, int digits = 2)     { Jetson.print(v, digits); Serial.print(v, digits); }

inline void hostPrintln()                              { Jetson.println(); Serial.println(); }
inline void hostPrintln(const __FlashStringHelper *s) { Jetson.println(s); Serial.println(s); }
inline void hostPrintln(const String &s)              { Jetson.println(s); Serial.println(s); }
inline void hostPrintln(const char *s)                { Jetson.println(s); Serial.println(s); }
inline void hostPrintln(long v)                       { Jetson.println(v); Serial.println(v); }
inline void hostPrintln(int v)                        { Jetson.println(v); Serial.println(v); }

// ============================================================
//                  FORWARD DECLARATIONS
// ============================================================

inline void stepPulse(unsigned int delayUs);
void startupPromptAndHome();
void resetToBeginningState();

// ============================================================
//                       HELPERS
// ============================================================

inline void enableDriver(bool on) {
  driverEnabledState = on;
  bool level = EN_ACTIVE_LOW ? !on : on;
  digitalWrite(EN_PIN, level ? HIGH : LOW);
}

void toggleDriverState() {
  enableDriver(!driverEnabledState);
  if (driverEnabledState) hostPrintln(F("Motor driver ENABLED."));
  else                    hostPrintln(F("Motor driver DISABLED."));
}

inline bool indexPressedRaw() {
  return (digitalRead(INDEX_PIN) == INDEX_ACTIVE_LEVEL);
}

inline long degToSteps(float deg) {
  while (deg < 0)    deg += 360.0f;
  while (deg >= 360) deg -= 360.0f;
  return (long)((deg / 360.0f) * (float)STEPS_PER_REV + 0.5f);
}

inline bool bootButtonPressedRaw() {
  return digitalRead(BOOT_BTN_PIN) == LOW;
}

void clearHostInputBuffers() {
  while (Jetson.available()) Jetson.read();
  while (Serial.available()) Serial.read();
  line = "";
}

bool pollEstopDuringMotion() {
  while (Jetson.available()) {
    char c = (char)Jetson.read();
    if (c == 'e' || c == 'E') {
      estopLatched = true;
      motionActive = false;
      enableDriver(false);
      hostPrintln(F("E-STOP ACTIVE"));
      hostPrintln(F("Type 'quit' to reset."));
      return true;
    }
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'e' || c == 'E') {
      estopLatched = true;
      motionActive = false;
      enableDriver(false);
      hostPrintln(F("E-STOP ACTIVE"));
      hostPrintln(F("Type 'quit' to reset."));
      return true;
    }
  }

  return false;
}

// ============================================================
//               BOOT / EN DIAGNOSTIC HELPERS
// ============================================================

void printBootDiagnostics() {
  hostPrintln();
  hostPrintln(F("=================================="));
  hostPrintln(F("ESP32-S3 BOOT/EN STATUS"));
  hostPrint(F("Boot count: "));
  hostPrintln((long)bootCount);

  if (bootButtonPressedRaw()) {
    hostPrintln(F("BOOT was HELD at startup (GPIO0 LOW)."));
  } else {
    hostPrintln(F("BOOT was NOT held at startup (GPIO0 HIGH)."));
  }

  hostPrintln(F("Runtime test: press/release BOOT and watch messages."));
  hostPrintln(F("EN test: press EN; board resets, this header prints again, bootCount increments."));
  hostPrintln(F("=================================="));
}

void processBootButtonMessages() {
  bool now = digitalRead(BOOT_BTN_PIN);

  if (bootBtnLast == HIGH && now == LOW) {
    hostPrintln(F("BOOT pressed (GPIO0 LOW)"));
  } else if (bootBtnLast == LOW && now == HIGH) {
    hostPrintln(F("BOOT released (GPIO0 HIGH)"));
  }

  bootBtnLast = now;
}

// ============================================================
//                   BREAK-BEAM DEBOUNCE
// ============================================================

inline bool beamBlockedRaw() {
  int v = digitalRead(BEAM_PIN);
  return BEAM_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void updateBeamDebounce() {
  bool raw = beamBlockedRaw();
  unsigned long now = millis();

  if (raw != beamLastRawBlocked) {
    beamLastRawBlocked = raw;
    beamLastChangeMs = now;
  }

  if ((now - beamLastChangeMs) >= BEAM_DEBOUNCE_MS) {
    bool prev = beamStableBlocked;
    beamStableBlocked = raw;

    if (beamStableBlocked != prev) {
      if (beamStableBlocked) {
        gateLockout    = true;
        gateOpenedAtMs = 0;
        hostPrintln(F("GATE: BLOCKED"));
      } else {
        gateOpenedAtMs = now;
        hostPrintln(F("GATE: OPEN"));
      }
    }
  }

  if (gateLockout && !beamStableBlocked && gateOpenedAtMs != 0) {
    if ((millis() - gateOpenedAtMs) >= BEAM_ARM_DELAY_MS) {
      gateLockout      = false;
      gateOpenedAtMs   = 0;
      inventoryPending = true;
      hostPrintln(F("GATE: Ready"));
    }
  }
}

// ============================================================
//                   MOTION PRIMITIVES
// ============================================================

inline void stepPulse(unsigned int delayUs) {
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(delayUs);
}

bool stepCCW(long n, unsigned int delayUs) {
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(true);
  delay(EN_SETTLE_MS);

  motionActive = true;

  for (long i = 0; i < n; i++) {
    if (pollEstopDuringMotion()) {
      enableDriver(false);
      motionActive = false;
      return false;
    }
    stepPulse(delayUs);
  }

  motionActive = false;
  enableDriver(false);
  return true;
}

bool confirmPressed() {
  int ok = 0;
  for (int i = 0; i < PRESS_CONFIRM_POLLS; i++) {
    if (indexPressedRaw()) ok++;
    else ok = 0;
    delayMicroseconds(POLL_GAP_US);
  }
  return ok >= PRESS_CONFIRM_POLLS;
}

bool confirmReleased() {
  int ok = 0;
  for (int i = 0; i < RELEASE_CONFIRM_POLLS; i++) {
    if (!indexPressedRaw()) ok++;
    else ok = 0;
    delayMicroseconds(POLL_GAP_US);
  }
  return ok >= RELEASE_CONFIRM_POLLS;
}

bool seekNextPressEventCCW(long &stepsMoved) {
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(true);
  delay(EN_SETTLE_MS);

  motionActive = true;
  stepsMoved = 0;
  long guard = 0;

  while (!confirmReleased()) {
    if (pollEstopDuringMotion()) {
      enableDriver(false);
      motionActive = false;
      return false;
    }

    stepPulse(SEEK_DELAY_US);
    stepsMoved++;

    if (++guard > MAX_STEPS_TO_FIND_INDEX) {
      enableDriver(false);
      motionActive = false;
      return false;
    }
  }

  guard = 0;
  int approachCountdown = 0;

  while (true) {
    if (pollEstopDuringMotion()) {
      enableDriver(false);
      motionActive = false;
      return false;
    }

    bool rawNow = indexPressedRaw();
    if (rawNow) approachCountdown = APPROACH_WINDOW_STEPS;

    unsigned int useDelay = (approachCountdown > 0) ? APPROACH_DELAY_US : SEEK_DELAY_US;
    stepPulse(useDelay);
    stepsMoved++;

    if (++guard > MAX_STEPS_TO_FIND_INDEX) {
      enableDriver(false);
      motionActive = false;
      return false;
    }

    if (approachCountdown > 0) approachCountdown--;

    if (confirmPressed()) {
      for (int i = 0; i < POST_HIT_ESCAPE_STEPS; i++) {
        if (pollEstopDuringMotion()) {
          enableDriver(false);
          motionActive = false;
          return false;
        }

        stepPulse(APPROACH_DELAY_US);
        stepsMoved++;
      }

      long relGuard = 0;
      while (!confirmReleased()) {
        if (pollEstopDuringMotion()) {
          enableDriver(false);
          motionActive = false;
          return false;
        }

        stepPulse(APPROACH_DELAY_US);
        stepsMoved++;
        if (++relGuard > MAX_STEPS_TO_FIND_INDEX) {
          enableDriver(false);
          motionActive = false;
          return false;
        }
      }

      enableDriver(false);
      motionActive = false;
      return true;
    }
  }
}

// ============================================================
//                    SORT HELPERS
// ============================================================

static int cmpLong(const void *a, const void *b) {
  long A = *(const long*)a, B = *(const long*)b;
  return (A < B) ? -1 : (A > B) ? 1 : 0;
}

static int cmpFloat(const void *a, const void *b) {
  float A = *(const float*)a, B = *(const float*)b;
  return (A < B) ? -1 : (A > B) ? 1 : 0;
}

bool applyIndexOffset() {
  long offsetSteps = degToSteps(INDEX_TO_STOP_DEG + TRIM_DEG);
  if (offsetSteps > 0) return stepCCW(offsetSteps, OFFSET_DELAY_US);
  return true;
}

// ============================================================
//                      ULTRASONIC
// ============================================================

float pingDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, US_ECHO_TIMEOUT_US);
  if (duration == 0) return NAN;
  return (duration * 0.0343f) / 2.0f;
}

float medianOfBlockCm() {
  float s[US_BLOCK_SIZE];
  int count = 0;
  while (count < US_BLOCK_SIZE) {
    float d = pingDistanceCm();
    if (!isnan(d)) s[count++] = d;
    delay(US_PING_PERIOD_MS);
  }
  qsort(s, US_BLOCK_SIZE, sizeof(float), cmpFloat);
  return s[US_BLOCK_SIZE / 2];
}

float readRobustDistance4sCm() {
  const int MAX_BLOCKS = (int)(US_WINDOW_MS / (US_BLOCK_SIZE * US_PING_PERIOD_MS)) + 10;
  float meds[MAX_BLOCKS];
  int mCount = 0;

  unsigned long t0 = millis();
  while ((millis() - t0) < US_WINDOW_MS) {
    float m = medianOfBlockCm();
    if (!isnan(m)) meds[mCount++] = m;
    if (mCount >= MAX_BLOCKS) break;
  }

  if (mCount < 5) return NAN;

  qsort(meds, mCount, sizeof(float), cmpFloat);

  int trim = (int)(mCount * US_TRIM_FRACTION + 0.5f);
  if (trim * 2 >= mCount) trim = (mCount - 1) / 2;

  double sum = 0.0;
  int used = 0;
  for (int i = trim; i < (mCount - trim); i++) {
    sum += meds[i];
    used++;
  }
  if (used <= 0) return NAN;

  return (float)(sum / (double)used);
}

// ============================================================
//                       HOMING ROUTINE
// ============================================================

bool homeRoutine() {
  homed                    = false;
  currentBin               = -1;
  encoderGapThresholdSteps = 0;

  long moved = 0;
  if (!seekNextPressEventCCW(moved)) {
    if (!estopLatched) hostPrintln(F("ERROR: Could not find index switch."));
    return false;
  }

  long gaps[HOME_SAMPLE_EVENTS];
  for (int i = 0; i < HOME_SAMPLE_EVENTS; i++) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) {
      if (!estopLatched) hostPrintln(F("ERROR: Lost index switch during homing."));
      return false;
    }
    gaps[i] = g;
  }

  long sorted[HOME_SAMPLE_EVENTS];
  for (int i = 0; i < HOME_SAMPLE_EVENTS; i++) sorted[i] = gaps[i];
  qsort(sorted, HOME_SAMPLE_EVENTS, sizeof(long), cmpLong);
  long normalGap = sorted[HOME_SAMPLE_EVENTS / 2];

  encoderGapThresholdSteps = (long)(normalGap * ENCODER_GAP_RATIO + 0.5f);

  while (true) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) {
      if (!estopLatched) hostPrintln(F("ERROR: Lost index switch during homing scan."));
      return false;
    }

    if (g < encoderGapThresholdSteps) {
      if (!applyIndexOffset()) return false;

      currentBin      = 0;
      homed           = true;
      lastSelectedBin = 0;
      hostPrintln(F("Homing complete"));
      hostPrintln(F("Now at BIN0"));
      return true;
    }
  }
}

// ============================================================
//                    BIN MOVEMENT
// ============================================================

bool moveOneBin() {
  if (!homed) {
    hostPrintln(F("ERROR: Not homed."));
    return false;
  }

  long tmp = 0;

  if (currentBin == 3) {
    if (!seekNextPressEventCCW(tmp)) return false;
    if (!seekNextPressEventCCW(tmp)) return false;

    if (!applyIndexOffset()) return false;
    currentBin = 0;
    return true;
  }

  if (!seekNextPressEventCCW(tmp)) return false;
  if (!applyIndexOffset()) return false;
  currentBin++;
  return true;
}

bool moveBins(int n) {
  for (int i = 0; i < n; i++) {
    if (!moveOneBin()) return false;
  }
  return true;
}

bool moveToBin(int targetBin) {
  if (!homed) {
    hostPrintln(F("ERROR: Not homed."));
    return false;
  }
  if (targetBin < 0 || targetBin > 3) {
    hostPrintln(F("ERROR: targetBin must be 0..3"));
    return false;
  }

  int steps = (targetBin - currentBin + 4) % 4;
  if (steps == 0) return true;

  return moveBins(steps);
}

// ============================================================
//               INVENTORY SEQUENCE
// ============================================================

bool runInventorySequence() {
  if (!homed) {
    hostPrintln(F("ERROR: Not homed."));
    return false;
  }
  if (beamStableBlocked) {
    hostPrintln(F("ERROR: Gate BLOCKED."));
    return false;
  }
  if (!inventoryPending) {
    hostPrintln(F("Inventory not pending."));
    return false;
  }
  if (lastSelectedBin < 0 || lastSelectedBin > 3) {
    hostPrintln(F("ERROR: Unknown lastSelectedBin."));
    return false;
  }

  if (!moveBins(2)) return false;
  if (estopLatched) return false;

  float d = readRobustDistance4sCm();

  if (isnan(d)) {
    hostPrintln(F("LO"));
    hostPrintln(F("(Distance(cm) = NAN)"));
  } else {
    hostPrintln((d < INVENTORY_THRESH_CM) ? F("HI") : F("LO"));
    hostPrint(F("(Distance(cm) = "));
    hostPrint(d, 3);
    hostPrintln(F(")"));
  }

  inventoryPending = false;
  return true;
}

// ============================================================
//                   SERIAL LINE READER
// ============================================================

bool readLineFromHosts(String &out) {
  while (Jetson.available()) {
    char c = (char)Jetson.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 64) out += c;
  }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 64) out += c;
  }
  return false;
}

// ============================================================
//                   STARTUP / HOMING PROMPT
// ============================================================

void startupPromptAndHome() {
  hostPrintln(F("\nPower-up: Homing is required to define BIN0."));
  hostPrintln(F("Type 'h' + Enter to start homing."));
  hostPrintln(F("Type 'toggle' + Enter to toggle motor driver enable."));

  String bootLine = "";

  while (true) {
    updateBeamDebounce();
    processBootButtonMessages();

    if (!readLineFromHosts(bootLine)) continue;

    bootLine.trim();

    if (estopLatched) {
      if (bootLine.equalsIgnoreCase("quit")) {
        resetToBeginningState();
        hostPrintln(F("Type 'h' + Enter to start homing."));
        hostPrintln(F("Type 'toggle' + Enter to toggle motor driver enable."));
      } else {
        hostPrintln(F("E-STOP ACTIVE. Type 'quit' to reset."));
      }
      bootLine = "";
      continue;
    }

    if (bootLine.equalsIgnoreCase("h")) {
      if (homeRoutine()) return;
      if (!estopLatched) hostPrintln(F("Homing failed. Type 'h' to try again."));
    } else if (bootLine.equalsIgnoreCase("toggle")) {
      toggleDriverState();
    } else {
      hostPrintln(F("Please type 'h' to home or 'toggle' to toggle motor driver."));
    }

    bootLine = "";
  }
}

// ============================================================
//                   COMMAND HELPERS
// ============================================================

bool parseBinCommand(const String &s, int &binOut) {
  String t = s;
  t.trim();
  t.toLowerCase();
  if (!t.startsWith(F("bin"))) return false;
  if (t.length() != 4) return false;
  char d = t[3];
  if (d < '0' || d > '3') return false;
  binOut = d - '0';
  return true;
}

void resetToBeginningState() {
  homed                    = false;
  currentBin               = -1;
  encoderGapThresholdSteps = 0;
  gateLockout              = false;
  gateOpenedAtMs           = 0;
  inventoryPending         = false;
  lastSelectedBin          = -1;
  estopLatched             = false;
  motionActive             = false;
  enableDriver(false);
  clearHostInputBuffers();
  hostPrintln(F("\n[QUIT] Returning to startup. Homing required."));
}

// ============================================================
//                        SETUP
// ============================================================

void setup() {
  delay(50);

  pinMode(PUL_PIN,   OUTPUT);
  pinMode(DIR_PIN,   OUTPUT);
  pinMode(EN_PIN,    OUTPUT);

  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  pinMode(INDEX_PIN, INPUT_PULLUP);
  delay(10);

  pinMode(BEAM_PIN,  INPUT_PULLUP);
  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);

  digitalWrite(PUL_PIN,  LOW);
  digitalWrite(DIR_PIN,  CCW_DIR_LEVEL);
  digitalWrite(TRIG_PIN, LOW);
  enableDriver(false);

  Serial.begin(115200);
  delay(300);

  Jetson.begin(JETSON_BAUD, SERIAL_8N1, JETSON_UART_RX, JETSON_UART_TX);
  delay(200);

  bootCount++;
  bootBtnLast = digitalRead(BOOT_BTN_PIN);

  hostPrintln(F("\n[BOOT] ESP32-S3 starting..."));
  hostPrint(F("[BOOT] Jetson UART2 RX="));
  hostPrint(JETSON_UART_RX);
  hostPrint(F(" TX="));
  hostPrintln(JETSON_UART_TX);

  printBootDiagnostics();

  beamLastRawBlocked = beamBlockedRaw();
  beamStableBlocked  = beamLastRawBlocked;
  beamLastChangeMs   = millis();
  if (beamStableBlocked) {
    gateLockout = true;
    hostPrintln(F("GATE: BLOCKED"));
  }

  startupPromptAndHome();
}

// ============================================================
//                        MAIN LOOP
// ============================================================

void loop() {
  updateBeamDebounce();
  processBootButtonMessages();

  if (estopLatched) {
    if (!readLineFromHosts(line)) return;

    line.trim();

    if (line.equalsIgnoreCase("quit")) {
      resetToBeginningState();
      updateBeamDebounce();
      startupPromptAndHome();
    } else if (line.length() > 0) {
      hostPrintln(F("E-STOP ACTIVE. Type 'quit' to reset."));
    }

    line = "";
    return;
  }

  if (!readLineFromHosts(line)) return;

  line.trim();

  if (line.length() == 0) {
    line = "";
    return;
  }

  if (line.equalsIgnoreCase("quit")) {
    resetToBeginningState();
    updateBeamDebounce();
    startupPromptAndHome();
    line = "";
    return;
  }

  if (line.equalsIgnoreCase("h")) {
    hostPrintln(F("IGNORED: 'h' is only accepted at startup."));
    line = "";
    return;
  }

  if (line.equalsIgnoreCase("toggle")) {
    hostPrintln(F("IGNORED: 'toggle' is only accepted at startup."));
    line = "";
    return;
  }

  if (line.equalsIgnoreCase("i")) {
    if (!inventoryPending) {
      hostPrintln(F("Nothing to inventory right now."));
    } else if (beamStableBlocked) {
      hostPrintln(F("Gate is BLOCKED."));
    } else if (gateLockout) {
      hostPrintln(F("Gate re-arming."));
    } else {
      if (!runInventorySequence() && !estopLatched) {
        hostPrintln(F("ERROR: inventory sequence failed."));
      }
    }
    line = "";
    return;
  }

  int targetBin = -1;
  if (parseBinCommand(line, targetBin)) {
    if (gateLockout) {
      hostPrintln(F("GATE LOCKOUT"));
    } else if (inventoryPending) {
      hostPrintln(F("INVENTORY REQUIRED"));
    } else {
      if (!moveToBin(targetBin)) {
        if (!estopLatched) hostPrintln(F("ERROR: move failed."));
      } else {
        lastSelectedBin = targetBin;
        hostPrint(F("Now at BIN"));
        hostPrintln(currentBin);

        updateBeamDebounce();
        if (beamStableBlocked) {
          gateLockout    = true;
          gateOpenedAtMs = 0;
          hostPrintln(F("GATE: BLOCKED"));
        }
      }
    }
    line = "";
    return;
  }

  hostPrint(F("Unknown command: '"));
  hostPrint(line);
  hostPrintln(F("'"));
  line = "";
}