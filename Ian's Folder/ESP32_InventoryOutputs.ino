#include <Arduino.h>
#include <math.h>   // NAN / isnan
#include <string.h> // strlen

// ============================================================
//                    PIN / HW CONFIG
// ============================================================

// ===== TB6600 pins (common-cathode: PUL-/DIR-/ENA- -> GND) =====
const int PUL_PIN = 5;   // TB6600 PUL+
const int DIR_PIN = 6;   // TB6600 DIR+h
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

const float INDEX_TO_STOP_DEG = 31.0f;
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
//        INVENTORY LOG LINE (CSV-READY OVER UART)
// ============================================================
// Format:  INV,binX,HI|LO,<millis>
// Example: INV,bin2,HI,123456789
const bool PRINT_INV_LOG_LINE = true;

// ============================================================
//                        STATE
// ============================================================

bool estop = false;
bool homed = false;
int  currentBin = -1; // 0..3
long encoderGapThresholdSteps = 0;

// ----- Break-beam state -----
bool beamStableBlocked = false;
bool beamLastRawBlocked = false;
unsigned long beamLastChangeMs = 0;

bool gateLockout = false;                // when true, block selecting bins
unsigned long gateOpenedAtMs = 0;        // timestamp when beam became stably OPEN

// After reinsert + 2s, require inventory before allowing new selections
bool inventoryPending = false;

// Track which bin was last presented/pulled (for inventory rotation)
int lastSelectedBin = -1;

// ============================================================
//   COMMAND QUEUE (makes "one press" feel consistent)
// ============================================================

bool inventoryRequested = false; // typed 'i' early -> run when READY
bool binRequested = false;       // typed bin# early -> run when allowed
int  requestedBin = -1;

// ============================================================
//   ROBUST LINE INPUT (no Arduino String; avoids heap churn)
// ============================================================

static const size_t CMD_BUF_LEN = 48;
char cmdBuf[CMD_BUF_LEN];
size_t cmdLen = 0;

inline void resetCmdBuf() {
  cmdLen = 0;
  cmdBuf[0] = '\0';
}

bool readCommandLine() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      // swallow extra CR/LF already buffered
      while (Serial.peek() == '\n' || Serial.peek() == '\r') Serial.read();
      cmdBuf[cmdLen] = '\0';
      return true;
    }

    // backspace / delete support
    if (c == '\b' || c == 127) {
      if (cmdLen > 0) cmdLen--;
      continue;
    }

    if (cmdLen < (CMD_BUF_LEN - 1)) {
      cmdBuf[cmdLen++] = c;
    }
  }
  return false;
}

char* trimInPlace(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) {
    s[n - 1] = '\0';
    n--;
  }
  return s;
}

bool equalsIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = ca - 'A' + 'a';
    if (cb >= 'A' && cb <= 'Z') cb = cb - 'A' + 'a';
    if (ca != cb) return false;
    a++; b++;
  }
  return (*a == '\0' && *b == '\0');
}

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

// We keep 'e' as the hard stop, but we DO NOT eat arbitrary bytes here anymore.
// We'll only check for 'e' in the line-reader (and also during motion if bytes arrive).
inline void pollEstopDuringMotion() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'e' || c == 'E') estop = true;
  }
}

inline bool stepPulseChecked(unsigned int delayUs) {
  pollEstopDuringMotion();
  if (estop) return false;

  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(delayUs);

  pollEstopDuringMotion();
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

// ============================================================
// Forward declarations
// ============================================================

bool runInventorySequence();
bool moveToBin(int targetBin);
bool parseBinCommand(const char *s, int &binOut);
void tryRunQueuedCommands();

// ============================================================
// INVENTORY LOG PRINT (CSV-ready line with millis timestamp)
// ============================================================

void printInventoryLogLine(int bin, bool isHI) {
  if (!PRINT_INV_LOG_LINE) return;
  if (bin < 0 || bin > 3) return;

  // OLD:
  // Serial.print(F("INV,bin"));

  // NEW:
  Serial.print(F("bin"));
  Serial.print(bin);
  Serial.print(F(","));
  Serial.print(isHI ? F("HI") : F("LO"));
  Serial.print(F(","));
  Serial.println(millis());
}

// ============================================================
// NEW: helpers to run queued commands when allowed
// ============================================================

void tryRunQueuedCommands() {
  if (estop) return;

  // If inventory is pending and user already requested it, run it as soon as allowed.
  if (inventoryRequested) {
    if (inventoryPending && !gateLockout && !beamStableBlocked) {
      Serial.println(F("Auto-running queued inventory request..."));
      inventoryRequested = false;
      if (!runInventorySequence()) Serial.println(F("ERROR: inventory sequence failed."));
      Serial.println(F("Enter command:"));
      return; // inventory takes time; let loop continue later
    }
  }

  // If a bin move was requested, run it as soon as allowed.
  if (binRequested) {
    if (!gateLockout && !inventoryPending) {
      int b = requestedBin;
      binRequested = false;
      requestedBin = -1;

      Serial.print(F("Auto-running queued bin request: BIN"));
      Serial.println(b);

      if (!moveToBin(b)) {
        Serial.println(F("ERROR: move to bin failed."));
      } else {
        lastSelectedBin = b;
        Serial.print(F("Done. Now at BIN"));
        Serial.println(currentBin);
      }
      Serial.println(F("Enter command:"));
    }
  }
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
        Serial.println(F("GATE: BLOCKED (bin pulled out). Locking new requests."));
      } else {
        gateOpenedAtMs = now;
        Serial.println(F("GATE: OPEN. Waiting 2 seconds before allowing next step..."));
      }
    }
  }

  // If locked out and OPEN, see if timer expired -> now require inventory
  if (gateLockout && !beamStableBlocked && gateOpenedAtMs != 0) {
    if ((millis() - gateOpenedAtMs) >= BEAM_ARM_DELAY_MS) {
      gateLockout = false;
      gateOpenedAtMs = 0;

      inventoryPending = true;
      Serial.println(F("GATE: Ready. Press 'i' to perform inventory sensing."));

      // If user already typed 'i', auto-run it now
      tryRunQueuedCommands();
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
    pollEstopDuringMotion();
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
    pollEstopDuringMotion();
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
  Serial.println(F("\n=== HOMING ROUTINE ==="));
  Serial.println(F("Rotating CCW to find unique double-press (encoder then real notch)..."));
  Serial.println(F("(type 'e' to stop)"));

  estop = false;
  homed = false;
  currentBin = -1;
  encoderGapThresholdSteps = 0;

  // Clear queued commands so homing always starts clean
  inventoryRequested = false;
  binRequested = false;
  requestedBin = -1;

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

  Serial.print(F("Estimated normal gap = ")); Serial.println(normalGap);
  Serial.print(F("Homing threshold (short gap) = ")); Serial.println(encoderGapThresholdSteps);
  Serial.println(F("Now scanning until a short gap is detected..."));

  while (true) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) return false;

    if (g < encoderGapThresholdSteps) {
      Serial.print(F("Detected SHORT gap = ")); Serial.println(g);
      Serial.println(F("encoder -> REAL notch. Using REAL notch as BIN0 reference."));

      if (!applyIndexOffset()) return false;

      currentBin = 0;
      homed = true;
      lastSelectedBin = 0;
      Serial.println(F("Homing complete. Now at BIN0."));
      return true;
    }
  }
}

// ============================================================
//                 NORMAL MOVE ONE BIN
// ============================================================

bool moveOneBin() {
  if (!homed) {
    Serial.println(F("ERROR: Not homed. Use 'h' first."));
    return false;
  }

  if (currentBin == 3) {
    long tmp = 0;

    if (!seekNextPressEventCCW(tmp)) return false;
    Serial.println(F("Hitting Encoder notch (ignored)"));

    if (!seekNextPressEventCCW(tmp)) return false;
    Serial.println(F("Hitting REAL notch for BIN0"));

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

bool moveToBin(int targetBin) {
  if (!homed) {
    Serial.println(F("ERROR: Not homed. Use 'h' first."));
    return false;
  }
  if (targetBin < 0 || targetBin > 3) {
    Serial.println(F("ERROR: targetBin must be 0..3"));
    return false;
  }

  int steps = (targetBin - currentBin + 4) % 4; // CCW steps needed
  if (steps == 0) {
    Serial.print(F("Already at BIN")); Serial.println(targetBin);
    return true;
  }

  Serial.print(F("Moving CCW "));
  Serial.print(steps);
  Serial.print(F(" bin(s) to BIN"));
  Serial.println(targetBin);

  if (!moveBins(steps)) return false;
  return true;
}

// ============================================================
//               INVENTORY SEQUENCE (press 'i')
// ============================================================

bool runInventorySequence() {
  if (!homed) {
    Serial.println(F("ERROR: Not homed."));
    return false;
  }
  if (beamStableBlocked) {
    Serial.println(F("ERROR: Gate is BLOCKED. Push bin fully in first."));
    return false;
  }
  if (!inventoryPending) {
    Serial.println(F("Inventory not pending."));
    return false;
  }
  if (lastSelectedBin < 0 || lastSelectedBin > 3) {
    Serial.println(F("ERROR: Unknown lastSelectedBin."));
    return false;
  }

  Serial.println(F("\n=== INVENTORY SENSING ==="));
  Serial.println(F("Rotating 2 bins (180 deg) to ultrasonic sensor..."));

  if (!moveBins(2)) return false;

  Serial.println(F("Measuring distance for ~4 seconds (robust average)..."));
  float d = readRobustDistance4sCm();
  if (estop) return false;

  bool isHI = false;

  if (isnan(d)) {
    // If reading failed, treat as LO for logging purposes (safer)
    Serial.println(F("LO"));
    Serial.println(F("(Distance(cm) = NAN)"));
    isHI = false;
  } else {
    isHI = (d < INVENTORY_THRESH_CM);
    Serial.println(isHI ? F("HI") : F("LO"));
    Serial.print(F("(Distance(cm) = "));
    Serial.print(d, 3);
    Serial.println(F(")"));
  }

  // NEW: CSV-ready one-line output for Jetson parsing
  // Uses millis() as timestamp (Option #1)
  printInventoryLogLine(lastSelectedBin, isHI);

  inventoryPending = false;
  Serial.println(F("Inventory complete. You may now select another bin."));

  // After clearing inventoryPending, a queued bin command can now run
  tryRunQueuedCommands();

  return true;
}

// ============================================================
//                      UI / COMMANDS
// ============================================================

void doEstop(const __FlashStringHelper *why) {
  estop = true;
  enableDriver(false);
  Serial.println();
  Serial.print(F("E-STOP: "));
  Serial.println(why);
  Serial.println(F("Motor disabled. Reset board to clear E-stop."));
}

void printHelp() {
  Serial.println(F("\n=== Index + Gate + Inventory (CCW-only) ==="));
  Serial.println(F("Commands: h, i, bin0, bin1, bin2, bin3, e"));
  Serial.println(F("  h        -> homing routine (find encoder double-press, set BIN0)"));
  Serial.println(F("  bin#     -> move to BIN0..BIN3"));
  Serial.println(F("  i        -> inventory sensing (only after bin reinserted + 2s)"));
  Serial.println(F("  e        -> E-stop"));

  Serial.print(F("Inventory threshold (cm) = "));
  Serial.println(INVENTORY_THRESH_CM, 3);

  if (homed) {
    Serial.print(F("Status: HOMED, currentBin=BIN")); Serial.println(currentBin);
  } else {
    Serial.println(F("Status: NOT HOMED"));
  }

  if (inventoryPending) {
    Serial.println(F("Status: INVENTORY PENDING (press 'i')"));
  }

  Serial.println(F("\nInventory log format (CSV-ready):"));
  Serial.println(F("  INV,binX,HI|LO,<millis>"));
}

// Startup prompt: require homing attempt
bool startupPromptAndHome() {
  Serial.println(F("\nPower-up: Homing is required to define BIN0."));
  Serial.println(F("Type 'h' then press Enter to start homing."));

  resetCmdBuf();

  while (true) {
    // allow e-stop during boot prompt
    pollEstopDuringMotion();
    if (estop) return false;

    updateBeamDebounce();
    tryRunQueuedCommands();

    if (readCommandLine()) {
      char *cmd = trimInPlace(cmdBuf);

      if (equalsIgnoreCase(cmd, "e")) {
        doEstop(F("user command"));
        resetCmdBuf();
        return false;
      }

      if (equalsIgnoreCase(cmd, "h")) {
        Serial.println(F("Homing..."));
        resetCmdBuf();

        if (!homeRoutine()) {
          if (estop) return false;
          Serial.println(F("ERROR: Homing failed. Type 'h' to try again."));
          continue;
        }
        return true;
      }

      if (cmd[0] != '\0') {
        Serial.println(F("Please type 'h' to home (required)."));
      }

      resetCmdBuf();
    }
  }
}

bool parseBinCommand(const char *s, int &binOut) {
  // expects exactly: bin0..bin3 (case-insensitive)
  if (!s) return false;

  char t0 = s[0], t1 = s[1], t2 = s[2], t3 = s[3], t4 = s[4];
  if (t4 != '\0') return false; // must be length 4

  if (t0 >= 'A' && t0 <= 'Z') t0 = t0 - 'A' + 'a';
  if (t1 >= 'A' && t1 <= 'Z') t1 = t1 - 'A' + 'a';
  if (t2 >= 'A' && t2 <= 'Z') t2 = t2 - 'A' + 'a';

  if (!(t0 == 'b' && t1 == 'i' && t2 == 'n')) return false;
  if (t3 < '0' || t3 > '3') return false;

  binOut = t3 - '0';
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

  Serial.begin(115200);
  delay(300);

  resetCmdBuf();

  // Initialize beam debounce baseline
  beamLastRawBlocked = beamBlockedRaw();
  beamStableBlocked = beamLastRawBlocked;
  beamLastChangeMs = millis();
  if (beamStableBlocked) {
    gateLockout = true;
    Serial.println(F("GATE: BLOCKED at startup."));
  } else {
    gateLockout = false;
  }

  if (!startupPromptAndHome()) return;

  printHelp();
  Serial.println(F("\nEnter command:"));
}

void loop() {
  // estop during idle
  if (estop) return;

  updateBeamDebounce();

  // queued commands can auto-run when READY
  tryRunQueuedCommands();

  if (readCommandLine()) {
    char *cmd = trimInPlace(cmdBuf);

    if (cmd[0] == '\0') {
      Serial.println(F("Enter command:"));
      resetCmdBuf();
      return;
    }

    if (equalsIgnoreCase(cmd, "e")) {
      doEstop(F("user command"));
      resetCmdBuf();
      return;
    }

    if (equalsIgnoreCase(cmd, "h")) {
      Serial.println(F("Homing... (type 'e' to stop)"));
      resetCmdBuf();
      if (!homeRoutine()) Serial.println(F("ERROR: homing failed."));
      Serial.println(F("Enter command:"));
      return;
    }

    if (equalsIgnoreCase(cmd, "i")) {
      // queue inventory so one press works even during re-arm delay
      inventoryRequested = true;

      if (!inventoryPending) {
        if (beamStableBlocked) Serial.println(F("Gate is BLOCKED. Push bin fully in first."));
        else if (gateLockout)  Serial.println(F("Gate re-arming. Will run inventory when READY."));
        else                   Serial.println(F("Nothing to inventory right now."));
      } else if (beamStableBlocked) {
        Serial.println(F("Gate is BLOCKED. Push bin fully in first."));
      } else if (gateLockout) {
        Serial.println(F("Gate re-arming. Will run inventory when READY."));
      } else {
        // allowed right now -> run immediately
        inventoryRequested = false;
        if (!runInventorySequence()) Serial.println(F("ERROR: inventory sequence failed."));
      }

      Serial.println(F("Enter command:"));
      resetCmdBuf();
      return;
    }

    int targetBin = -1;
    if (parseBinCommand(cmd, targetBin)) {
      if (gateLockout) {
        binRequested = true;
        requestedBin = targetBin;
        Serial.println(F("GATE LOCKOUT: queued bin request. Push bin back in and wait 2 seconds."));
        Serial.println(F("Enter command:"));
        resetCmdBuf();
        return;
      }

      if (inventoryPending) {
        binRequested = true;
        requestedBin = targetBin;
        Serial.println(F("INVENTORY REQUIRED: queued bin request. Press 'i' (or it will auto-run if already requested)."));
        Serial.println(F("Enter command:"));
        resetCmdBuf();
        return;
      }

      Serial.print(F("Selecting BIN"));
      Serial.println(targetBin);

      if (!moveToBin(targetBin)) {
        Serial.println(F("ERROR: move to bin failed."));
      } else {
        lastSelectedBin = targetBin;

        Serial.print(F("Done. Now at BIN"));
        Serial.println(currentBin);

        updateBeamDebounce();
        if (beamStableBlocked) {
          gateLockout = true;
          gateOpenedAtMs = 0;
          Serial.println(F("GATE: BLOCKED detected after selection. Locking new requests."));
        }
      }

      Serial.println(F("Enter command:"));
      resetCmdBuf();
      return;
    }

    Serial.println(F("Unknown command. Use 'h', 'i', 'bin0..bin3', or 'e'."));
    Serial.println(F("Enter command:"));
    resetCmdBuf();
    return;
  }

  // allow estop to be caught even if user doesn't press enter
  pollEstopDuringMotion();
}
