// ============================================================
//  ESP32-S3 BIN CAROUSEL (TB6600) + GATE (BREAK-BEAM) + INVENTORY (HC-SR04)
//  UPDATED: uses UART1 for Jetson commands/responses, keeps USB Serial for debug.
//
//  Jetson <-> ESP32 wiring (recommended):
//    Jetson TX  -> ESP32 GPIO18 (UART1 RX)
//    Jetson RX  -> ESP32 GPIO17 (UART1 TX)
//    Jetson GND -> ESP32 GND
//  Baud: 115200
//
//  Notes:
//   - Commands are read from UART1 (Jetson). USB Serial is debug-only.
//   - All "protocol" prints (HI/LO, GATE:, Homing complete, Now at BIN, etc.)
//     are sent to Jetson (UART1) AND mirrored to USB Serial for visibility.
//
//  CHANGE REQUEST IMPLEMENTED:
//   - Homing ('h') is ONLY accepted at startup prompt (beginning state).
//   - During normal operation, 'h' is ignored and does NOT re-home.
//   - New command: 'quit' -> returns to beginning (startup prompt) and requires homing again.
// ============================================================

#include <Arduino.h>
#include <math.h> // for NAN / isnan

// ============================================================
//                    PIN / HW CONFIG
// ============================================================

// ===== TB6600 pins (common-cathode: PUL-/DIR-/ENA- -> GND) =====
const int PUL_PIN = 5;   // TB6600 PUL+
const int DIR_PIN = 6;   // TB6600 DIR+
const int EN_PIN  = 7;   // TB6600 ENA+   (break-beam logic will NOT touch this)

// ===== Index switch =====
const int INDEX_PIN = 3;                // Microswitch NO -> GPIO3, COM -> GND
const int INDEX_ACTIVE_LEVEL = LOW;     // INPUT_PULLUP => pressed = LOW

// ===== Break-beam gate =====
const int BEAM_PIN = 8;                 // break-beam output
const bool BEAM_ACTIVE_LOW = true;      // common: blocked = LOW
const unsigned long BEAM_DEBOUNCE_MS = 20;
const unsigned long BEAM_ARM_DELAY_MS = 2000; // must be OPEN for 2s before accepting next step

// ===== Ultrasonic (HC-SR04) =====
const int TRIG_PIN = 16;
const int ECHO_PIN = 17;

// ===== Enable polarity =====
const bool EN_ACTIVE_LOW = true;        // many TB6600 boards: LOW = enabled

// ===== Stepper config =====
const int MICROSTEP = 8;
const int FULL_STEPS_PER_REV = 200;
const long STEPS_PER_REV = (long)FULL_STEPS_PER_REV * MICROSTEP;

const unsigned int PULSE_HIGH_US = 20;
const unsigned int EN_SETTLE_MS  = 2;

// ============================================================
//                         SPEED
// ============================================================

const unsigned int SEEK_DELAY_US     = 1500;
const unsigned int OFFSET_DELAY_US   = 1500;

const unsigned int APPROACH_DELAY_US = 2600;
const int APPROACH_WINDOW_STEPS      = 140;

const bool CCW_DIR_LEVEL = LOW;

// ============================================================
//              SWITCH FILTERING / SAFETY
// ============================================================

const int PRESS_CONFIRM_POLLS   = 5;
const int RELEASE_CONFIRM_POLLS = 7;
const unsigned int POLL_GAP_US  = 250;

const long MAX_STEPS_TO_FIND_INDEX = STEPS_PER_REV * 3;
const int POST_HIT_ESCAPE_STEPS    = 30;

// ============================================================
//                SINGLE "OFFSET KNOB" (DEGREES)
// ============================================================

const float INDEX_TO_STOP_DEG = 30.0f;
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

const int US_BLOCK_SIZE = 7;
const float US_TRIM_FRACTION = 0.20f;

// ============================================================
//            INVENTORY THRESHOLD (TUNABLE, CM)
// ============================================================

const float INVENTORY_THRESH_CM = 12.0f;

// ============================================================
//                      UART1 (JETSON) SETUP
// ============================================================

static const uint32_t JETSON_BAUD = 115200;
// We explicitly set pins:
const int JETSON_UART_TX = 9;  // ESP32 -> Jetson RX
const int JETSON_UART_RX = 11; // ESP32 <- Jetson TX

HardwareSerial Jetson(2);

// ============================================================
//                        STATE
// ============================================================

bool estop = false;
bool homed = false;
int  currentBin = -1; // 0..3
long encoderGapThresholdSteps = 0;
String line;

// ----- Break-beam state -----
bool beamStableBlocked = false;
bool beamLastRawBlocked = false;
unsigned long beamLastChangeMs = 0;

bool gateLockout = false;                // when true, block selecting bins
unsigned long gateOpenedAtMs = 0;        // timestamp when beam became stably OPEN

// New: after reinsert + 2s, require inventory before allowing new selections
bool inventoryPending = false;

// Track which bin was last presented/pulled (for inventory rotation)
int lastSelectedBin = -1;

// ============================================================
//                  PRINT/IO HELPERS (USB + Jetson)
// ============================================================

// Send to Jetson always; also mirror to USB Serial for debugging.
inline void hostPrint(const __FlashStringHelper *s) { Jetson.print(s); Serial.print(s); }
inline void hostPrint(const String &s)              { Jetson.print(s); Serial.print(s); }
inline void hostPrint(const char *s)                { Jetson.print(s); Serial.print(s); }
inline void hostPrint(long v)                       { Jetson.print(v); Serial.print(v); }
inline void hostPrint(int v)                        { Jetson.print(v); Serial.print(v); }
inline void hostPrint(float v, int digits = 2)      { Jetson.print(v, digits); Serial.print(v, digits); }

inline void hostPrintln()                           { Jetson.println(); Serial.println(); }
inline void hostPrintln(const __FlashStringHelper *s){ Jetson.println(s); Serial.println(s); }
inline void hostPrintln(const String &s)            { Jetson.println(s); Serial.println(s); }
inline void hostPrintln(const char *s)              { Jetson.println(s); Serial.println(s); }
inline void hostPrintln(long v)                     { Jetson.println(v); Serial.println(v); }
inline void hostPrintln(int v)                      { Jetson.println(v); Serial.println(v); }

// ============================================================
//                       HELPERS
// ============================================================

inline void enableDriver(bool on) {
  bool level = EN_ACTIVE_LOW ? !on : on;
  digitalWrite(EN_PIN, level ? HIGH : LOW);
}

inline bool indexPressedRaw() {
  return (digitalRead(INDEX_PIN) == INDEX_ACTIVE_LEVEL);
}

inline long degToSteps(float deg) {
  while (deg < 0)    deg += 360.0f;
  while (deg >= 360) deg -= 360.0f;
  return (long)((deg / 360.0f) * (float)STEPS_PER_REV + 0.5f);
}

// Read E-stop from Jetson UART (and also USB Serial, optional)
inline void pollEstop() {
  while (Jetson.available() > 0) {
    char c = (char)Jetson.read();
    if (c == 'e' || c == 'E') estop = true;
  }
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'e' || c == 'E') estop = true;
  }
}

inline bool stepPulseChecked(unsigned int delayUs) {
  pollEstop();
  if (estop) return false;

  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(delayUs);

  pollEstop();
  return !estop;
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

// ---- Break-beam read + debounce ----
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
        gateLockout = true;
        gateOpenedAtMs = 0;
        hostPrintln(F("GATE: BLOCKED (bin pulled out). Locking new requests."));
      } else {
        gateOpenedAtMs = now;
        hostPrintln(F("GATE: OPEN. Waiting 2 seconds before allowing next step..."));
      }
    }
  }

  if (gateLockout && !beamStableBlocked && gateOpenedAtMs != 0) {
    if ((millis() - gateOpenedAtMs) >= BEAM_ARM_DELAY_MS) {
      gateLockout = false;
      gateOpenedAtMs = 0;

      inventoryPending = true;
      hostPrintln(F("GATE: Ready. Press 'i' to perform inventory sensing."));
    }
  }
}

bool stepCCW(long n, unsigned int delayUs) {
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(true);
  delay(EN_SETTLE_MS);

  for (long i = 0; i < n; i++) {
    if (!stepPulseChecked(delayUs)) { enableDriver(false); return false; }
  }

  enableDriver(false);
  return true;
}

// ============================================================
//   Seek NEXT press event (fresh press) with slow-approach profile
// ============================================================

bool seekNextPressEventCCW(long &stepsMoved) {
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(true);
  delay(EN_SETTLE_MS);

  stepsMoved = 0;
  long guard = 0;

  while (!confirmReleased()) {
    if (!stepPulseChecked(SEEK_DELAY_US)) { enableDriver(false); return false; }
    stepsMoved++;
    if (++guard > MAX_STEPS_TO_FIND_INDEX) { enableDriver(false); return false; }
  }

  guard = 0;
  int approachCountdown = 0;

  while (true) {
    if (indexPressedRaw()) approachCountdown = APPROACH_WINDOW_STEPS;

    unsigned int useDelay = (approachCountdown > 0) ? APPROACH_DELAY_US : SEEK_DELAY_US;
    if (!stepPulseChecked(useDelay)) { enableDriver(false); return false; }
    stepsMoved++;
    if (++guard > MAX_STEPS_TO_FIND_INDEX) { enableDriver(false); return false; }

    if (approachCountdown > 0) approachCountdown--;

    if (confirmPressed()) {
      for (int i = 0; i < POST_HIT_ESCAPE_STEPS; i++) {
        if (!stepPulseChecked(APPROACH_DELAY_US)) { enableDriver(false); return false; }
        stepsMoved++;
      }

      long relGuard = 0;
      while (!confirmReleased()) {
        if (!stepPulseChecked(APPROACH_DELAY_US)) { enableDriver(false); return false; }
        stepsMoved++;
        if (++relGuard > MAX_STEPS_TO_FIND_INDEX) { enableDriver(false); return false; }
      }

      enableDriver(false);
      return true;
    }
  }
}

// Median helper for long
static int cmpLong(const void *a, const void *b) {
  long A = *(const long*)a;
  long B = *(const long*)b;
  if (A < B) return -1;
  if (A > B) return  1;
  return 0;
}

// Median helper for float
static int cmpFloat(const void *a, const void *b) {
  float A = *(const float*)a;
  float B = *(const float*)b;
  if (A < B) return -1;
  if (A > B) return  1;
  return 0;
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
    pollEstop();
    if (estop) return NAN;

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
    pollEstop();
    if (estop) return NAN;

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
  hostPrintln(F("\n=== HOMING ROUTINE ==="));
  hostPrintln(F("Rotating CCW to find unique double-press (encoder then real notch)..."));
  hostPrintln(F("(type 'e' to stop)"));

  estop = false;
  homed = false;
  currentBin = -1;
  encoderGapThresholdSteps = 0;

  long moved = 0;
  if (!seekNextPressEventCCW(moved)) return false;

  long gaps[HOME_SAMPLE_EVENTS];
  for (int i = 0; i < HOME_SAMPLE_EVENTS; i++) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) return false;
    gaps[i] = g;
  }

  long sorted[HOME_SAMPLE_EVENTS];
  for (int i = 0; i < HOME_SAMPLE_EVENTS; i++) sorted[i] = gaps[i];
  qsort(sorted, HOME_SAMPLE_EVENTS, sizeof(long), cmpLong);
  long normalGap = sorted[HOME_SAMPLE_EVENTS / 2];

  encoderGapThresholdSteps = (long)(normalGap * ENCODER_GAP_RATIO + 0.5f);

  hostPrint(F("Estimated normal gap = ")); hostPrintln(normalGap);
  hostPrint(F("Homing threshold (short gap) = ")); hostPrintln(encoderGapThresholdSteps);
  hostPrintln(F("Now scanning until a short gap is detected..."));

  while (true) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) return false;

    if (g < encoderGapThresholdSteps) {
      hostPrint(F("Detected SHORT gap = ")); hostPrintln(g);
      hostPrintln(F("encoder -> REAL notch. Using REAL notch as BIN0 reference."));

      if (!applyIndexOffset()) return false;

      currentBin = 0;
      homed = true;
      lastSelectedBin = 0;
      hostPrintln(F("Homing complete. Now at BIN0."));
      return true;
    }
  }
}

// ============================================================
//                 NORMAL MOVE ONE BIN
// ============================================================

bool moveOneBin() {
  if (!homed) {
    hostPrintln(F("ERROR: Not homed. Use 'h' first."));
    return false;
  }

  if (currentBin == 3) {
    long tmp = 0;

    if (!seekNextPressEventCCW(tmp)) return false;
    hostPrintln(F("Hitting Encoder notch (ignored)"));

    if (!seekNextPressEventCCW(tmp)) return false;
    hostPrintln(F("Hitting REAL notch for BIN0"));

    if (!applyIndexOffset()) return false;

    currentBin = 0;
    return true;
  }

  long tmp = 0;
  if (!seekNextPressEventCCW(tmp)) return false;

  if (!applyIndexOffset()) return false;

  currentBin = currentBin + 1;
  return true;
}

bool moveBins(int n) {
  for (int i = 0; i < n; i++) {
    if (!moveOneBin()) return false;
  }
  return true;
}

// Move from currentBin to targetBin using CCW-only minimal steps
bool moveToBin(int targetBin) {
  if (!homed) {
    hostPrintln(F("ERROR: Not homed. Use 'h' first."));
    return false;
  }
  if (targetBin < 0 || targetBin > 3) {
    hostPrintln(F("ERROR: targetBin must be 0..3"));
    return false;
  }

  int steps = (targetBin - currentBin + 4) % 4; // CCW steps needed
  if (steps == 0) {
    hostPrint(F("Already at BIN")); hostPrintln(targetBin);
    return true;
  }

  hostPrint(F("Moving CCW "));
  hostPrint(steps);
  hostPrint(F(" bin(s) to BIN"));
  hostPrintln(targetBin);

  if (!moveBins(steps)) return false;
  return true;
}

// ============================================================
//               INVENTORY SEQUENCE (press 'i')
// ============================================================

bool runInventorySequence() {
  if (!homed) {
    hostPrintln(F("ERROR: Not homed."));
    return false;
  }
  if (beamStableBlocked) {
    hostPrintln(F("ERROR: Gate is BLOCKED. Push bin fully in first."));
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

  hostPrintln(F("\n=== INVENTORY SENSING ==="));
  hostPrintln(F("Rotating 2 bins (180 deg) to ultrasonic sensor..."));

  if (!moveBins(2)) return false;

  hostPrintln(F("Measuring distance for ~4 seconds (robust average)..."));
  float d = readRobustDistance4sCm();
  if (estop) return false;

  if (isnan(d)) {
    hostPrintln(F("LO")); // fail-safe
    hostPrintln(F("(Distance(cm) = NAN)"));
  } else {
    const bool isHI = (d < INVENTORY_THRESH_CM);
    hostPrintln(isHI ? F("HI") : F("LO"));

    hostPrint(F("(Distance(cm) = "));
    hostPrint(d, 3);
    hostPrintln(F(")"));
  }

  inventoryPending = false;
  hostPrintln(F("Inventory complete. You may now select another bin."));
  return true;
}

// ============================================================
//                      UI / COMMANDS
// ============================================================

void doEstop(const __FlashStringHelper *why) {
  estop = true;
  enableDriver(false);
  hostPrintln();
  hostPrint(F("E-STOP: "));
  hostPrintln(why);
  hostPrintln(F("Motor disabled. Reset board to clear E-stop."));
}

void printHelp() {
  hostPrintln(F("\n=== Index + Gate + Inventory (CCW-only) ==="));
  hostPrintln(F("Commands: i, bin0, bin1, bin2, bin3, quit, e"));
  hostPrintln(F("  (Startup only) h   -> homing routine (required after boot/quit)"));
  hostPrintln(F("  bin#               -> move to BIN0..BIN3"));
  hostPrintln(F("  i                  -> inventory sensing (only after bin reinserted + 2s)"));
  hostPrintln(F("  quit               -> return to startup (requires homing again)"));
  hostPrintln(F("  e                  -> E-stop"));

  hostPrint(F("Inventory threshold (cm) = "));
  hostPrint(INVENTORY_THRESH_CM, 3);
  hostPrintln();

  if (homed) {
    hostPrint(F("Status: HOMED, currentBin=BIN")); hostPrintln(currentBin);
  } else {
    hostPrintln(F("Status: NOT HOMED"));
  }

  if (inventoryPending) {
    hostPrintln(F("Status: INVENTORY PENDING (press 'i')"));
  }
}

// Reset only the "session state" (does not reboot ESP32)
void resetToBeginningState() {
  homed = false;
  currentBin = -1;
  encoderGapThresholdSteps = 0;

  // Reset inventory/gate workflow
  gateLockout = false;
  gateOpenedAtMs = 0;
  inventoryPending = false;
  lastSelectedBin = -1;

  // Clear any partial line buffer
  line = "";

  hostPrintln(F("\n[QUIT] Returning to startup. Homing will be required again."));
}

// Startup prompt: require homing attempt from Jetson (UART1) or USB Serial
bool startupPromptAndHome() {
  hostPrintln(F("\nPower-up: Homing is required to define BIN0."));
  hostPrintln(F("Type 'h' then press Enter to start homing."));

  String bootLine = "";

  while (true) {
    pollEstop();
    if (estop) return false;

    updateBeamDebounce();

    // Prefer Jetson UART, but allow USB for convenience.
    while (Jetson.available()) {
      char c = (char)Jetson.read();
      if (c == '\r') continue;

      if (c == '\n') {
        bootLine.trim();
        if (bootLine.equalsIgnoreCase("h")) {
          hostPrintln(F("Homing..."));
          if (!homeRoutine()) {
            if (estop) return false;
            hostPrintln(F("ERROR: Homing failed. Type 'h' to try again."));
            bootLine = "";
            break;
          }
          return true;
        } else {
          hostPrintln(F("Please type 'h' to home (required)."));
          bootLine = "";
        }
      } else {
        if (bootLine.length() < 40) bootLine += c;
      }
    }

    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;

      if (c == '\n') {
        bootLine.trim();
        if (bootLine.equalsIgnoreCase("h")) {
          hostPrintln(F("Homing..."));
          if (!homeRoutine()) {
            if (estop) return false;
            hostPrintln(F("ERROR: Homing failed. Type 'h' to try again."));
            bootLine = "";
            break;
          }
          return true;
        } else {
          hostPrintln(F("Please type 'h' to home (required)."));
          bootLine = "";
        }
      } else {
        if (bootLine.length() < 40) bootLine += c;
      }
    }
  }
}

bool parseBinCommand(const String &s, int &binOut) {
  String t = s;
  t.trim();
  t.toLowerCase();
  if (!t.startsWith("bin")) return false;
  if (t.length() != 4) return false;          // "bin0" exactly
  char d = t[3];
  if (d < '0' || d > '3') return false;
  binOut = d - '0';
  return true;
}

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(INDEX_PIN, INPUT_PULLUP);

  pinMode(BEAM_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(false);

  // USB debug serial
  Serial.begin(115200);
  delay(200);

  // Jetson UART1
  Jetson.begin(JETSON_BAUD, SERIAL_8N1, JETSON_UART_RX, JETSON_UART_TX);
  delay(200);

  hostPrintln(F("\n[BOOT] ESP32-S3 starting..."));
  hostPrint(F("[BOOT] Jetson UART1 on RX="));
  hostPrint(JETSON_UART_RX);
  hostPrint(F(" TX="));
  hostPrintln(JETSON_UART_TX);

  // Initialize beam debounce baseline
  beamLastRawBlocked = beamBlockedRaw();
  beamStableBlocked = beamLastRawBlocked;
  beamLastChangeMs = millis();
  if (beamStableBlocked) {
    gateLockout = true;
    hostPrintln(F("GATE: BLOCKED at startup."));
  } else {
    gateLockout = false;
  }

  if (!startupPromptAndHome()) return;

  printHelp();
  hostPrintln(F("\nEnter command:"));
}

static inline void printEnterCommand() {
  hostPrintln(F("Enter command:"));
}

// Read one complete line from Jetson UART (preferred). Also allows USB Serial.
// Returns true if a line (ending in '\n') was completed in `out`.
bool readLineFromHosts(String &out) {
  // Jetson first
  while (Jetson.available()) {
    char c = (char)Jetson.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 40) out += c;
  }
  // USB second
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 40) out += c;
  }
  return false;
}

void loop() {
  pollEstop();
  if (estop) return;

  updateBeamDebounce();

  if (readLineFromHosts(line)) {
    line.trim();
    if (line.length() == 0) { line = ""; printEnterCommand(); return; }

    if (line.equalsIgnoreCase("e")) {
      doEstop(F("user command"));
      line = "";
      return;
    }

    // New: quit -> return to startup prompt and require homing again
    if (line.equalsIgnoreCase("quit")) {
      resetToBeginningState();

      // Re-sync gate state right away (so prompt reflects reality)
      updateBeamDebounce();

      if (!startupPromptAndHome()) return;

      printHelp();
      hostPrintln(F("\nEnter command:"));
      line = "";
      return;
    }

    // Homing is ONLY accepted at startup prompt; ignore here.
    if (line.equalsIgnoreCase("h")) {
      hostPrintln(F("IGNORED: 'h' is only accepted at startup."));
      hostPrintln(F("Type 'quit' to return to startup and home again."));
      line = "";
      printEnterCommand();
      return;
    }

    if (line.equalsIgnoreCase("i")) {
      if (!inventoryPending) {
        hostPrintln(F("Nothing to inventory right now."));
      } else if (beamStableBlocked) {
        hostPrintln(F("Gate is BLOCKED. Push bin fully in first."));
      } else if (gateLockout) {
        hostPrintln(F("Gate re-arming. Wait for READY message."));
      } else {
        if (!runInventorySequence()) hostPrintln(F("ERROR: inventory sequence failed."));
      }

      line = "";
      printEnterCommand();
      return;
    }

    int targetBin = -1;
    bool isBinCmd = parseBinCommand(line, targetBin);

    if (isBinCmd) {
      if (gateLockout) {
        hostPrintln(F("GATE LOCKOUT: bin is out (beam blocked) or waiting 2s after reinsertion."));
        hostPrintln(F("Push bin back in (beam OPEN). Then wait 2 seconds."));
        line = "";
        printEnterCommand();
        return;
      }
      if (inventoryPending) {
        hostPrintln(F("INVENTORY REQUIRED: Press 'i' to measure before selecting another bin."));
        line = "";
        printEnterCommand();
        return;
      }

      hostPrint(F("Selecting BIN"));
      hostPrintln(targetBin);

      if (!moveToBin(targetBin)) {
        hostPrintln(F("ERROR: move to bin failed."));
      } else {
        lastSelectedBin = targetBin;

        hostPrint(F("Done. Now at BIN"));
        hostPrintln(currentBin);

        updateBeamDebounce();
        if (beamStableBlocked) {
          gateLockout = true;
          gateOpenedAtMs = 0;
          hostPrintln(F("GATE: BLOCKED detected after selection. Locking new requests."));
        }
      }

      line = "";
      printEnterCommand();
      return;
    }

    hostPrintln(F("Unknown command. Use 'i', 'bin0..bin3', 'quit', or 'e'."));
    line = "";
    printEnterCommand();
  }
}
