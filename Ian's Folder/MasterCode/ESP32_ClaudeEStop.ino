// ============================================================
//  ESP32-S3 BIN CAROUSEL (TB6600) + GATE (BREAK-BEAM) + INVENTORY (HC-SR04)
//
//  E-STOP STRATEGY (key improvement over prior version):
//   - A dedicated ring buffer sits between the raw UART and the command parser.
//   - During MOTOR MOTION, pollEstop() scans only for 'e'/'E' in the live
//     UART stream and deposits all OTHER characters into the ring buffer
//     instead of discarding them. Nothing is ever thrown away.
//   - During IDLE (loop()), readLineFromHosts() drains the ring buffer first,
//     then live UART — so characters queued during a move are processed in
//     order, undamaged.
//   - Result: 'e' during motion = immediate e-stop; commands typed during
//     motion = queued and executed afterwards; no characters lost.
//
//  Jetson <-> ESP32 wiring:
//    Jetson TX  -> ESP32 GPIO11 (UART2 RX)
//    Jetson RX  -> ESP32 GPIO9  (UART2 TX)
//    Jetson GND -> ESP32 GND
//  Baud: 115200
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
const int INDEX_ACTIVE_LEVEL = LOW;

const int  BEAM_PIN                   = 8;
const bool BEAM_ACTIVE_LOW            = true;
const unsigned long BEAM_DEBOUNCE_MS  = 20;
const unsigned long BEAM_ARM_DELAY_MS = 2000;

const int TRIG_PIN = 16;
const int ECHO_PIN = 17;

const bool EN_ACTIVE_LOW = true;

const int  MICROSTEP          = 8;
const int  FULL_STEPS_PER_REV = 200;
const long STEPS_PER_REV      = (long)FULL_STEPS_PER_REV * MICROSTEP;

const unsigned int PULSE_HIGH_US = 20;
const unsigned int EN_SETTLE_MS  = 2;

// ============================================================
//                         SPEED
// ============================================================

const unsigned int SEEK_DELAY_US      = 1500;
const unsigned int OFFSET_DELAY_US    = 1500;
const unsigned int APPROACH_DELAY_US  = 2600;
const int          APPROACH_WINDOW_STEPS = 140;
const bool         CCW_DIR_LEVEL      = LOW;

// ============================================================
//                  SWITCH FILTERING
// ============================================================

const int  PRESS_CONFIRM_POLLS   = 5;
const int  RELEASE_CONFIRM_POLLS = 7;
const unsigned int POLL_GAP_US   = 250;

const long MAX_STEPS_TO_FIND_INDEX = STEPS_PER_REV * 3;
const int  POST_HIT_ESCAPE_STEPS   = 30;

// ============================================================
//                  OFFSET / HOMING
// ============================================================

const float INDEX_TO_STOP_DEG  = 29.0f;
const float TRIM_DEG           = -18.0f;
const float ENCODER_GAP_RATIO  = 0.60f;
const int   HOME_SAMPLE_EVENTS = 10;

// ============================================================
//                  ULTRASONIC
// ============================================================

const unsigned long US_WINDOW_MS       = 4000;
const unsigned long US_PING_PERIOD_MS  = 25;
const unsigned long US_ECHO_TIMEOUT_US = 12000UL;
const int           US_BLOCK_SIZE      = 7;
const float         US_TRIM_FRACTION   = 0.20f;
const float         INVENTORY_THRESH_CM = 12.0f;

// ============================================================
//                  UART2 (JETSON)
// ============================================================

static const uint32_t JETSON_BAUD   = 115200;
const int             JETSON_UART_TX = 9;
const int             JETSON_UART_RX = 11;
HardwareSerial Jetson(2);

// ============================================================
//          RING BUFFER — survives characters typed during motion
// ============================================================

// 256 bytes: power-of-2 allows cheap masking instead of modulo.
// Large enough to hold several commands typed while the motor runs.
static const uint16_t RB_SIZE = 256;
static const uint16_t RB_MASK = RB_SIZE - 1;

struct RingBuf {
  volatile uint16_t head;   // write index
  volatile uint16_t tail;   // read  index
  char              buf[RB_SIZE];

  void   clear()            { head = tail = 0; }
  bool   empty()  const     { return head == tail; }
  bool   full()   const     { return ((head + 1) & RB_MASK) == tail; }
  void   push(char c)       { if (!full()) { buf[head] = c; head = (head + 1) & RB_MASK; } }
  char   pop()              { char c = buf[tail]; tail = (tail + 1) & RB_MASK; return c; }
  char   peek()   const     { return buf[tail]; }
} rb;

// ============================================================
//                  GLOBAL STATE
// ============================================================

bool   estop                    = false;
bool   homed                    = false;
int    currentBin               = -1;
long   encoderGapThresholdSteps = 0;
String line;

bool          beamStableBlocked  = false;
bool          beamLastRawBlocked = false;
unsigned long beamLastChangeMs   = 0;
bool          gateLockout        = false;
unsigned long gateOpenedAtMs     = 0;
bool          inventoryPending   = false;
int           lastSelectedBin    = -1;

// ============================================================
//                  PRINT HELPERS
// ============================================================

inline void hostPrint(const __FlashStringHelper *s) { Jetson.print(s);         Serial.print(s);         }
inline void hostPrint(const String &s)              { Jetson.print(s);         Serial.print(s);         }
inline void hostPrint(const char *s)                { Jetson.print(s);         Serial.print(s);         }
inline void hostPrint(long v)                       { Jetson.print(v);         Serial.print(v);         }
inline void hostPrint(int v)                        { Jetson.print(v);         Serial.print(v);         }
inline void hostPrint(float v, int d = 2)           { Jetson.print(v, d);      Serial.print(v, d);      }
inline void hostPrintln()                           { Jetson.println();        Serial.println();        }
inline void hostPrintln(const __FlashStringHelper*s){ Jetson.println(s);       Serial.println(s);       }
inline void hostPrintln(const String &s)            { Jetson.println(s);       Serial.println(s);       }
inline void hostPrintln(const char *s)              { Jetson.println(s);       Serial.println(s);       }
inline void hostPrintln(long v)                     { Jetson.println(v);       Serial.println(v);       }
inline void hostPrintln(int v)                      { Jetson.println(v);       Serial.println(v);       }

// ============================================================
//   pollEstop() — called ONLY inside motor routines
//
//   Scans both UARTs for 'e'/'E' (triggers e-stop immediately).
//   Every OTHER character is pushed into the ring buffer so it
//   is NOT lost — it will be processed by readLineFromHosts()
//   once the motor stops.
// ============================================================

inline void pollEstop() {
  auto drain = [](HardwareSerial &port) {
    while (port.available()) {
      char c = (char)port.read();
      if (c == 'e' || c == 'E') {
        estop = true;
      } else {
        rb.push(c);   // preserve for later command parsing
      }
    }
  };
  drain(Jetson);
  drain(Serial);
}

// ============================================================
//                  MOTION PRIMITIVES
// ============================================================

inline void enableDriver(bool on) {
  digitalWrite(EN_PIN, (EN_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
}

inline bool indexPressedRaw() {
  return digitalRead(INDEX_PIN) == INDEX_ACTIVE_LEVEL;
}

inline long degToSteps(float deg) {
  while (deg <   0) deg += 360.0f;
  while (deg >= 360) deg -= 360.0f;
  return (long)((deg / 360.0f) * (float)STEPS_PER_REV + 0.5f);
}

// Single step pulse — polls e-stop before and after each pulse.
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
    if (indexPressedRaw()) ok++; else ok = 0;
    delayMicroseconds(POLL_GAP_US);
  }
  return ok >= PRESS_CONFIRM_POLLS;
}

bool confirmReleased() {
  int ok = 0;
  for (int i = 0; i < RELEASE_CONFIRM_POLLS; i++) {
    if (!indexPressedRaw()) ok++; else ok = 0;
    delayMicroseconds(POLL_GAP_US);
  }
  return ok >= RELEASE_CONFIRM_POLLS;
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

// ============================================================
//                  SORT HELPERS
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
  long s = degToSteps(INDEX_TO_STOP_DEG + TRIM_DEG);
  if (s > 0) return stepCCW(s, OFFSET_DELAY_US);
  return true;
}

// ============================================================
//                  ULTRASONIC
// ============================================================

float pingDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_ECHO_TIMEOUT_US);
  return (dur == 0) ? NAN : (dur * 0.0343f) / 2.0f;
}

float medianOfBlockCm() {
  float s[US_BLOCK_SIZE];
  int count = 0;
  while (count < US_BLOCK_SIZE) {
    // pollEstop() here preserves non-estop chars via ring buffer
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

  double sum = 0.0; int used = 0;
  for (int i = trim; i < (mCount - trim); i++) { sum += meds[i]; used++; }
  return (used <= 0) ? NAN : (float)(sum / (double)used);
}

// ============================================================
//                  HOMING
// ============================================================

bool homeRoutine() {
  hostPrintln(F("\n=== HOMING ROUTINE ==="));
  hostPrintln(F("Rotating CCW to find unique double-press (encoder then real notch)..."));
  hostPrintln(F("(send 'e' to e-stop)"));

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
  hostPrint(F("Normal gap = "));       hostPrintln(normalGap);
  hostPrint(F("Short-gap threshold = ")); hostPrintln(encoderGapThresholdSteps);
  hostPrintln(F("Scanning for real notch..."));

  while (true) {
    long g = 0;
    if (!seekNextPressEventCCW(g)) return false;
    if (g < encoderGapThresholdSteps) {
      hostPrint(F("SHORT gap = ")); hostPrintln(g);
      hostPrintln(F("Real notch found. Setting BIN0."));
      if (!applyIndexOffset()) return false;
      currentBin = 0; homed = true; lastSelectedBin = 0;
      hostPrintln(F("Homing complete. Now at BIN0."));
      return true;
    }
  }
}

// ============================================================
//                  BIN MOVEMENT
// ============================================================

bool moveOneBin() {
  if (!homed) { hostPrintln(F("ERROR: Not homed.")); return false; }
  long tmp = 0;
  if (currentBin == 3) {
    if (!seekNextPressEventCCW(tmp)) return false;
    hostPrintln(F("Passing encoder notch."));
    if (!seekNextPressEventCCW(tmp)) return false;
    hostPrintln(F("Real notch -> BIN0."));
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
  for (int i = 0; i < n; i++) if (!moveOneBin()) return false;
  return true;
}

bool moveToBin(int targetBin) {
  if (!homed) { hostPrintln(F("ERROR: Not homed.")); return false; }
  if (targetBin < 0 || targetBin > 3) { hostPrintln(F("ERROR: targetBin 0..3")); return false; }
  int steps = (targetBin - currentBin + 4) % 4;
  if (steps == 0) { hostPrint(F("Already at BIN")); hostPrintln(targetBin); return true; }
  hostPrint(F("Moving CCW ")); hostPrint(steps); hostPrint(F(" -> BIN")); hostPrintln(targetBin);
  return moveBins(steps);
}

// ============================================================
//                  BREAK-BEAM DEBOUNCE
// ============================================================

inline bool beamBlockedRaw() {
  return BEAM_ACTIVE_LOW ? (digitalRead(BEAM_PIN) == LOW) : (digitalRead(BEAM_PIN) == HIGH);
}

void updateBeamDebounce() {
  bool raw = beamBlockedRaw();
  unsigned long now = millis();

  if (raw != beamLastRawBlocked) { beamLastRawBlocked = raw; beamLastChangeMs = now; }

  if ((now - beamLastChangeMs) >= BEAM_DEBOUNCE_MS) {
    bool prev = beamStableBlocked;
    beamStableBlocked = raw;
    if (beamStableBlocked != prev) {
      if (beamStableBlocked) {
        gateLockout = true; gateOpenedAtMs = 0;
        hostPrintln(F("GATE: BLOCKED. Locking new requests."));
      } else {
        gateOpenedAtMs = now;
        hostPrintln(F("GATE: OPEN. Waiting 2s..."));
      }
    }
  }

  if (gateLockout && !beamStableBlocked && gateOpenedAtMs != 0) {
    if ((millis() - gateOpenedAtMs) >= BEAM_ARM_DELAY_MS) {
      gateLockout = false; gateOpenedAtMs = 0;
      inventoryPending = true;
      hostPrintln(F("GATE: Ready. Press 'i' to inventory."));
    }
  }
}

// ============================================================
//                  INVENTORY
// ============================================================

bool runInventorySequence() {
  if (!homed)            { hostPrintln(F("ERROR: Not homed."));                      return false; }
  if (beamStableBlocked) { hostPrintln(F("ERROR: Gate BLOCKED. Push bin in."));      return false; }
  if (!inventoryPending) { hostPrintln(F("Inventory not pending."));                  return false; }
  if (lastSelectedBin < 0 || lastSelectedBin > 3) {
    hostPrintln(F("ERROR: Unknown lastSelectedBin.")); return false;
  }

  hostPrintln(F("\n=== INVENTORY SENSING ==="));
  hostPrintln(F("Rotating 2 bins to ultrasonic sensor..."));
  if (!moveBins(2)) return false;

  hostPrintln(F("Measuring (~4s)..."));
  float d = readRobustDistance4sCm();
  if (estop) return false;

  if (isnan(d)) {
    hostPrintln(F("LO")); hostPrintln(F("(NAN)"));
  } else {
    hostPrintln((d < INVENTORY_THRESH_CM) ? F("HI") : F("LO"));
    hostPrint(F("(Distance(cm) = ")); hostPrint(d, 3); hostPrintln(F(")"));
  }

  inventoryPending = false;
  hostPrintln(F("Inventory complete. You may now select another bin."));
  return true;
}

// ============================================================
//   readLineFromHosts()
//
//   Called ONLY from idle loop / startup prompt — never from motor code.
//   Drains ring buffer first (characters saved during motion), then
//   live UART. Returns true when '\n' is received and line is ready.
// ============================================================

bool readLineFromHosts(String &out) {
  // 1. Drain ring buffer (characters preserved during motor moves)
  while (!rb.empty()) {
    char c = rb.pop();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 64) out += c;
  }
  // 2. Drain live Jetson UART
  while (Jetson.available()) {
    char c = (char)Jetson.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 64) out += c;
  }
  // 3. Drain live USB Serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    if (out.length() < 64) out += c;
  }
  return false;
}

// ============================================================
//                  UI HELPERS
// ============================================================

void doEstop(const __FlashStringHelper *why) {
  estop = true;
  enableDriver(false);
  hostPrintln();
  hostPrint(F("E-STOP: ")); hostPrintln(why);
  hostPrintln(F("Motor disabled. Reset board to clear."));
}

void printHelp() {
  hostPrintln(F("\n=== Bin Carousel ==="));
  hostPrintln(F("Commands: bin0..bin3, i, quit, e"));
  hostPrintln(F("  (Startup only) h -> homing"));
  hostPrint(F("Inventory threshold = ")); hostPrint(INVENTORY_THRESH_CM, 1); hostPrintln(F(" cm"));
  if (homed) { hostPrint(F("Status: HOMED at BIN")); hostPrintln(currentBin); }
  else         { hostPrintln(F("Status: NOT HOMED")); }
  if (inventoryPending) hostPrintln(F("INVENTORY PENDING -> press 'i'"));
}

static inline void printEnterCommand() { hostPrintln(F("Enter command:")); }

bool parseBinCommand(const String &s, int &binOut) {
  String t = s; t.trim(); t.toLowerCase();
  if (!t.startsWith(F("bin")) || t.length() != 4) return false;
  char d = t[3];
  if (d < '0' || d > '3') return false;
  binOut = d - '0';
  return true;
}

void resetToBeginningState() {
  homed = false; currentBin = -1; encoderGapThresholdSteps = 0;
  gateLockout = false; gateOpenedAtMs = 0;
  inventoryPending = false; lastSelectedBin = -1;
  line = "";
  rb.clear();   // discard any buffered input from the previous session
  hostPrintln(F("\n[QUIT] Returning to startup. Homing required."));
}

// ============================================================
//   startupPromptAndHome()
//
//   Uses readLineFromHosts() so the ring buffer is respected here
//   too — if characters were buffered before quit completed, they
//   won't interfere with the 'h' prompt.
// ============================================================

void startupPromptAndHome() {
  hostPrintln(F("\nHoming required. Send 'h' + Enter."));
  String bootLine = "";

  while (true) {
    updateBeamDebounce();

    if (readLineFromHosts(bootLine)) {
      bootLine.trim();
      if (bootLine.equalsIgnoreCase("h")) {
        hostPrintln(F("Homing..."));
        if (homeRoutine()) return;
        if (estop) {
          hostPrintln(F("E-STOP during homing. Reset board."));
          while (true) delay(1000);   // halt
        }
        hostPrintln(F("Homing failed. Send 'h' to retry."));
      } else {
        hostPrintln(F("Please send 'h' to home (required)."));
      }
      bootLine = "";
    }
  }
}

// ============================================================
//                  SETUP
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

  rb.clear();

  Serial.begin(115200);
  delay(200);
  Jetson.begin(JETSON_BAUD, SERIAL_8N1, JETSON_UART_RX, JETSON_UART_TX);
  delay(200);

  hostPrintln(F("\n[BOOT] ESP32-S3 starting..."));
  hostPrint(F("[BOOT] Jetson UART2 RX=")); hostPrint(JETSON_UART_RX);
  hostPrint(F(" TX=")); hostPrintln(JETSON_UART_TX);

  beamLastRawBlocked = beamBlockedRaw();
  beamStableBlocked  = beamLastRawBlocked;
  beamLastChangeMs   = millis();
  if (beamStableBlocked) { gateLockout = true; hostPrintln(F("GATE: BLOCKED at startup.")); }

  startupPromptAndHome();
  printHelp();
  printEnterCommand();
}

// ============================================================
//                  MAIN LOOP
// ============================================================

void loop() {
  // E-stop is sticky after being set; motor routines already disabled the driver.
  if (estop) return;

  updateBeamDebounce();

  if (!readLineFromHosts(line)) return;

  line.trim();
  if (line.length() == 0) { line = ""; printEnterCommand(); return; }

  // ---- e-stop (explicit idle command) ----
  if (line.equalsIgnoreCase("e")) {
    doEstop(F("user command"));
    line = "";
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
    hostPrintln(F("IGNORED: 'h' only accepted at startup. Use 'quit' to re-home."));
    line = ""; printEnterCommand(); return;
  }

  // ---- inventory ----
  if (line.equalsIgnoreCase("i")) {
    if (!inventoryPending)      hostPrintln(F("Nothing to inventory right now."));
    else if (beamStableBlocked) hostPrintln(F("Gate BLOCKED. Push bin in first."));
    else if (gateLockout)       hostPrintln(F("Gate re-arming. Wait for READY message."));
    else if (!runInventorySequence()) hostPrintln(F("ERROR: inventory failed."));
    line = ""; printEnterCommand(); return;
  }

  // ---- bin0..bin3 ----
  int targetBin = -1;
  if (parseBinCommand(line, targetBin)) {
    if (gateLockout) {
      hostPrintln(F("GATE LOCKOUT: push bin in and wait 2s."));
    } else if (inventoryPending) {
      hostPrintln(F("INVENTORY REQUIRED: press 'i' first."));
    } else {
      hostPrint(F("Selecting BIN")); hostPrintln(targetBin);
      if (!moveToBin(targetBin)) {
        if (estop) return;
        hostPrintln(F("ERROR: move failed."));
      } else {
        lastSelectedBin = targetBin;
        hostPrint(F("Done. Now at BIN")); hostPrintln(currentBin);
        updateBeamDebounce();
        if (beamStableBlocked) {
          gateLockout = true; gateOpenedAtMs = 0;
          hostPrintln(F("GATE: BLOCKED after selection."));
        }
      }
    }
    line = ""; printEnterCommand(); return;
  }

  // ---- unknown ----
  hostPrint(F("Unknown command: '"));
  hostPrint(line);
  hostPrintln(F("'. Use: bin0..bin3, i, quit, e"));
  line = ""; printEnterCommand();
}
