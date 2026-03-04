// ============================================================
//  ESP32-S3 BIN CAROUSEL (TB6600) + GATE (BREAK-BEAM) + INVENTORY (HC-SR04)
//  UPDATED: E-stop removed entirely for consistent serial input.
//
//  Jetson <-> ESP32 wiring:
//    Jetson TX  -> ESP32 GPIO11 (UART2 RX)
//    Jetson RX  -> ESP32 GPIO9  (UART2 TX)
//    Jetson GND -> ESP32 GND
//  Baud: 115200
//
//  Notes:
//   - Commands are read from UART2 (Jetson). USB Serial is debug-only.
//   - All protocol prints are sent to Jetson AND mirrored to USB Serial.
//   - E-stop removed: serial buffers are ONLY drained in readLineFromHosts(),
//     so no characters are ever silently consumed during motor movement.
//
//  Commands (after homing):
//   bin0..bin3  -> move carousel to that bin
//   i           -> inventory sensing (only after bin reinserted + 2s)
//   quit        -> return to startup state (requires homing again)
//   (Startup only) h -> run homing routine
// ============================================================

#include <Arduino.h>
#include <math.h>

// ============================================================
//                    PIN / HW CONFIG
// ============================================================

const int PUL_PIN = 5;
const int DIR_PIN = 6;
const int EN_PIN  = 7;

const int INDEX_PIN          = 3;
const int INDEX_ACTIVE_LEVEL = LOW;   // INPUT_PULLUP => pressed = LOW

const int BEAM_PIN                    = 8;
const bool BEAM_ACTIVE_LOW            = true;
const unsigned long BEAM_DEBOUNCE_MS  = 20;
const unsigned long BEAM_ARM_DELAY_MS = 2000;

const int TRIG_PIN = 16;
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

const int   US_BLOCK_SIZE     = 7;
const float US_TRIM_FRACTION  = 0.20f;

// ============================================================
//            INVENTORY THRESHOLD (TUNABLE, CM)
// ============================================================

const float INVENTORY_THRESH_CM = 12.0f;

// ============================================================
//                      UART2 (JETSON) SETUP
// ============================================================

static const uint32_t JETSON_BAUD = 115200;
const int JETSON_UART_TX = 9;   // ESP32 TX -> Jetson RX
const int JETSON_UART_RX = 11;  // ESP32 RX <- Jetson TX

HardwareSerial Jetson(2);

// ============================================================
//                        STATE
// ============================================================

bool homed         = false;
int  currentBin    = -1;
long encoderGapThresholdSteps = 0;
String line;

bool beamStableBlocked  = false;
bool beamLastRawBlocked = false;
unsigned long beamLastChangeMs = 0;

bool gateLockout       = false;
unsigned long gateOpenedAtMs = 0;

bool inventoryPending = false;
int  lastSelectedBin  = -1;

// ============================================================
//                  PRINT/IO HELPERS
// ============================================================

inline void hostPrint(const __FlashStringHelper *s) { Jetson.print(s);    Serial.print(s);    }
inline void hostPrint(const String &s)              { Jetson.print(s);    Serial.print(s);    }
inline void hostPrint(const char *s)                { Jetson.print(s);    Serial.print(s);    }
inline void hostPrint(long v)                       { Jetson.print(v);    Serial.print(v);    }
inline void hostPrint(int v)                        { Jetson.print(v);    Serial.print(v);    }
inline void hostPrint(float v, int digits = 2)      { Jetson.print(v, digits); Serial.print(v, digits); }

inline void hostPrintln()                            { Jetson.println();   Serial.println();   }
inline void hostPrintln(const __FlashStringHelper *s){ Jetson.println(s);  Serial.println(s);  }
inline void hostPrintln(const String &s)             { Jetson.println(s);  Serial.println(s);  }
inline void hostPrintln(const char *s)               { Jetson.println(s);  Serial.println(s);  }
inline void hostPrintln(long v)                      { Jetson.println(v);  Serial.println(v);  }
inline void hostPrintln(int v)                       { Jetson.println(v);  Serial.println(v);  }

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

// Single step pulse — no serial reads inside motor routines.
inline void stepPulse(unsigned int delayUs) {
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(delayUs);
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
        hostPrintln(F("GATE: BLOCKED (bin pulled out). Locking new requests."));
      } else {
        gateOpenedAtMs = now;
        hostPrintln(F("GATE: OPEN. Waiting 2 seconds before allowing next step..."));
      }
    }
  }

  if (gateLockout && !beamStableBlocked && gateOpenedAtMs != 0) {
    if ((millis() - gateOpenedAtMs) >= BEAM_ARM_DELAY_MS) {
      gateLockout    = false;
      gateOpenedAtMs = 0;
      inventoryPending = true;
      hostPrintln(F("GATE: Ready. Press 'i' to perform inventory sensing."));
    }
  }
}

// ============================================================
//                   MOTION PRIMITIVES
// ============================================================

void stepCCW(long n, unsigned int delayUs) {
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(true);
  delay(EN_SETTLE_MS);
  for (long i = 0; i < n; i++) stepPulse(delayUs);
  enableDriver(false);
}

// Seek next press event (fresh press) with slow-approach profile.
// Returns false only if MAX_STEPS_TO_FIND_INDEX is exceeded (mechanical fault).
bool seekNextPressEventCCW(long &stepsMoved) {
  digitalWrite(DIR_PIN, CCW_DIR_LEVEL);
  enableDriver(true);
  delay(EN_SETTLE_MS);

  stepsMoved = 0;
  long guard = 0;

  // Wait for release if already pressed
  while (!confirmReleased()) {
    stepPulse(SEEK_DELAY_US);
    stepsMoved++;
    if (++guard > MAX_STEPS_TO_FIND_INDEX) { enableDriver(false); return false; }
  }

  guard = 0;
  int approachCountdown = 0;

  while (true) {
    if (indexPressedRaw()) approachCountdown = APPROACH_WINDOW_STEPS;

    unsigned int useDelay = (approachCountdown > 0) ? APPROACH_DELAY_US : SEEK_DELAY_US;
    stepPulse(useDelay);
    stepsMoved++;
    if (++guard > MAX_STEPS_TO_FIND_INDEX) { enableDriver(false); return false; }

    if (approachCountdown > 0) approachCountdown--;

    if (confirmPressed()) {
      for (int i = 0; i < POST_HIT_ESCAPE_STEPS; i++) {
        stepPulse(APPROACH_DELAY_US);
        stepsMoved++;
      }

      long relGuard = 0;
      while (!confirmReleased()) {
        stepPulse(APPROACH_DELAY_US);
        stepsMoved++;
        if (++relGuard > MAX_STEPS_TO_FIND_INDEX) { enableDriver(false); return false; }
      }

      enableDriver(false);
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
  if (offsetSteps > 0) stepCCW(offsetSteps, OFFSET_DELAY_US);
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
  for (int i = trim; i < (mCount - trim); i++) { sum += meds[i]; used++; }
  if (used <= 0) return NAN;

  return (float)(sum / (double)used);
}

// ============================================================
//                       HOMING ROUTINE
// ============================================================

bool homeRoutine() {
  hostPrintln(F("\n=== HOMING ROUTINE ==="));
  hostPrintln(F("Rotating CCW to find unique double-press (encoder then real notch)..."));

  homed                   = false;
  currentBin              = -1;
  encoderGapThresholdSteps = 0;

  long moved = 0;
  if (!seekNextPressEventCCW(moved)) {
    hostPrintln(F("ERROR: Could not find index switch. Check wiring."));
    return false;
  }

  long gaps[HOME_SAMPLE_EVENTS];
  for (int i = 0; i < HOME_SAMPLE_EVENTS; i++) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) {
      hostPrintln(F("ERROR: Lost index switch during gap sampling."));
      return false;
    }
    gaps[i] = g;
  }

  long sorted[HOME_SAMPLE_EVENTS];
  for (int i = 0; i < HOME_SAMPLE_EVENTS; i++) sorted[i] = gaps[i];
  qsort(sorted, HOME_SAMPLE_EVENTS, sizeof(long), cmpLong);
  long normalGap = sorted[HOME_SAMPLE_EVENTS / 2];

  encoderGapThresholdSteps = (long)(normalGap * ENCODER_GAP_RATIO + 0.5f);

  hostPrint(F("Estimated normal gap = "));   hostPrintln(normalGap);
  hostPrint(F("Homing threshold (short) = ")); hostPrintln(encoderGapThresholdSteps);
  hostPrintln(F("Scanning for short gap (real notch)..."));

  while (true) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) {
      hostPrintln(F("ERROR: Lost index switch during homing scan."));
      return false;
    }

    if (g < encoderGapThresholdSteps) {
      hostPrint(F("Detected SHORT gap = ")); hostPrintln(g);
      hostPrintln(F("Real notch found. Setting BIN0 reference."));

      applyIndexOffset();

      currentBin      = 0;
      homed           = true;
      lastSelectedBin = 0;
      hostPrintln(F("Homing complete. Now at BIN0."));
      return true;
    }
  }
}

// ============================================================
//                    BIN MOVEMENT
// ============================================================

bool moveOneBin() {
  if (!homed) { hostPrintln(F("ERROR: Not homed.")); return false; }

  long tmp = 0;

  if (currentBin == 3) {
    if (!seekNextPressEventCCW(tmp)) return false;
    hostPrintln(F("Passing encoder notch."));

    if (!seekNextPressEventCCW(tmp)) return false;
    hostPrintln(F("Real notch -> BIN0."));

    applyIndexOffset();
    currentBin = 0;
    return true;
  }

  if (!seekNextPressEventCCW(tmp)) return false;
  applyIndexOffset();
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
  if (!homed) { hostPrintln(F("ERROR: Not homed.")); return false; }
  if (targetBin < 0 || targetBin > 3) { hostPrintln(F("ERROR: targetBin must be 0..3")); return false; }

  int steps = (targetBin - currentBin + 4) % 4;
  if (steps == 0) {
    hostPrint(F("Already at BIN")); hostPrintln(targetBin);
    return true;
  }

  hostPrint(F("Moving CCW ")); hostPrint(steps); hostPrint(F(" bin(s) to BIN")); hostPrintln(targetBin);
  return moveBins(steps);
}

// ============================================================
//               INVENTORY SEQUENCE
// ============================================================

bool runInventorySequence() {
  if (!homed)               { hostPrintln(F("ERROR: Not homed."));                           return false; }
  if (beamStableBlocked)    { hostPrintln(F("ERROR: Gate BLOCKED. Push bin in first."));     return false; }
  if (!inventoryPending)    { hostPrintln(F("Inventory not pending."));                       return false; }
  if (lastSelectedBin < 0 || lastSelectedBin > 3) {
    hostPrintln(F("ERROR: Unknown lastSelectedBin.")); return false;
  }

  hostPrintln(F("\n=== INVENTORY SENSING ==="));
  hostPrintln(F("Rotating 2 bins to ultrasonic sensor..."));

  if (!moveBins(2)) return false;

  hostPrintln(F("Measuring distance (~4 seconds)..."));
  float d = readRobustDistance4sCm();

  if (isnan(d)) {
    hostPrintln(F("LO"));
    hostPrintln(F("(Distance(cm) = NAN)"));
  } else {
    hostPrintln((d < INVENTORY_THRESH_CM) ? F("HI") : F("LO"));
    hostPrint(F("(Distance(cm) = ")); hostPrint(d, 3); hostPrintln(F(")"));
  }

  inventoryPending = false;
  hostPrintln(F("Inventory complete. You may now select another bin."));
  return true;
}

// ============================================================
//                   SERIAL LINE READER
// ============================================================

// Reads one complete line from Jetson UART (preferred) or USB Serial.
// Characters accumulate in `out` across calls; returns true when '\n' seen.
// No serial reads happen anywhere else in the code.
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

  String bootLine = "";

  while (true) {
    updateBeamDebounce();

    if (readLineFromHosts(bootLine)) {
      bootLine.trim();

      if (bootLine.equalsIgnoreCase("h")) {
        hostPrintln(F("Homing..."));
        if (homeRoutine()) return;           // success -> back to caller
        hostPrintln(F("Homing failed. Type 'h' to try again."));
      } else {
        hostPrintln(F("Please type 'h' to home (required)."));
      }
      bootLine = "";
    }
  }
}

// ============================================================
//                   COMMAND HELPERS
// ============================================================

void printHelp() {
  hostPrintln(F("\n=== Bin Carousel ==="));
  hostPrintln(F("Commands: bin0, bin1, bin2, bin3, i, quit"));
  hostPrintln(F("  bin#  -> move to BIN0..BIN3"));
  hostPrintln(F("  i     -> inventory sensing (only after bin reinserted + 2s)"));
  hostPrintln(F("  quit  -> return to startup (requires homing again)"));
  hostPrint(F("Inventory threshold (cm) = ")); hostPrint(INVENTORY_THRESH_CM, 3); hostPrintln();

  if (homed) { hostPrint(F("Status: HOMED, currentBin=BIN")); hostPrintln(currentBin); }
  else        { hostPrintln(F("Status: NOT HOMED")); }

  if (inventoryPending) hostPrintln(F("Status: INVENTORY PENDING (press 'i')"));
}

static inline void printEnterCommand() { hostPrintln(F("Enter command:")); }

bool parseBinCommand(const String &s, int &binOut) {
  String t = s;
  t.trim(); t.toLowerCase();
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
  line                     = "";
  hostPrintln(F("\n[QUIT] Returning to startup. Homing required."));
}

// ============================================================
//                        SETUP
// ============================================================

void setup() {
  pinMode(PUL_PIN,   OUTPUT);
  pinMode(DIR_PIN,   OUTPUT);
  pinMode(EN_PIN,    OUTPUT);
  pinMode(INDEX_PIN, INPUT_PULLUP);
  pinMode(BEAM_PIN,  INPUT_PULLUP);
  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);

  digitalWrite(PUL_PIN,  LOW);
  digitalWrite(DIR_PIN,  CCW_DIR_LEVEL);
  digitalWrite(TRIG_PIN, LOW);
  enableDriver(false);

  Serial.begin(115200);
  delay(200);

  Jetson.begin(JETSON_BAUD, SERIAL_8N1, JETSON_UART_RX, JETSON_UART_TX);
  delay(200);

  hostPrintln(F("\n[BOOT] ESP32-S3 starting..."));
  hostPrint(F("[BOOT] Jetson UART2 RX=")); hostPrint(JETSON_UART_RX);
  hostPrint(F(" TX="));                   hostPrintln(JETSON_UART_TX);

  // Initialise beam state without triggering false events
  beamLastRawBlocked = beamBlockedRaw();
  beamStableBlocked  = beamLastRawBlocked;
  beamLastChangeMs   = millis();
  if (beamStableBlocked) {
    gateLockout = true;
    hostPrintln(F("GATE: BLOCKED at startup."));
  }

  startupPromptAndHome();

  printHelp();
  printEnterCommand();
}

// ============================================================
//                        MAIN LOOP
// ============================================================

void loop() {
  updateBeamDebounce();

  if (!readLineFromHosts(line)) return;   // nothing complete yet

  line.trim();

  if (line.length() == 0) {
    line = "";
    printEnterCommand();
    return;
  }

  // ---- quit ----
  if (line.equalsIgnoreCase("quit")) {
    resetToBeginningState();
    updateBeamDebounce();
    startupPromptAndHome();
    printHelp();
    printEnterCommand();
    line = "";
    return;
  }

  // ---- 'h' outside startup ----
  if (line.equalsIgnoreCase("h")) {
    hostPrintln(F("IGNORED: 'h' is only accepted at startup."));
    hostPrintln(F("Type 'quit' to return to startup and re-home."));
    line = "";
    printEnterCommand();
    return;
  }

  // ---- inventory ----
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

  // ---- bin0..bin3 ----
  int targetBin = -1;
  if (parseBinCommand(line, targetBin)) {
    if (gateLockout) {
      hostPrintln(F("GATE LOCKOUT: push bin back in and wait 2 seconds."));
    } else if (inventoryPending) {
      hostPrintln(F("INVENTORY REQUIRED: press 'i' before selecting another bin."));
    } else {
      hostPrint(F("Selecting BIN")); hostPrintln(targetBin);

      if (!moveToBin(targetBin)) {
        hostPrintln(F("ERROR: move failed (mechanical fault?)."));
      } else {
        lastSelectedBin = targetBin;
        hostPrint(F("Done. Now at BIN")); hostPrintln(currentBin);

        updateBeamDebounce();
        if (beamStableBlocked) {
          gateLockout    = true;
          gateOpenedAtMs = 0;
          hostPrintln(F("GATE: BLOCKED after selection. Locking new requests."));
        }
      }
    }
    line = "";
    printEnterCommand();
    return;
  }

  // ---- unknown ----
  hostPrintln(F("Unknown command. Use: bin0..bin3, i, quit"));
  line = "";
  printEnterCommand();
}
