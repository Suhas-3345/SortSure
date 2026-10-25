// SortSure - ESP32 DevKit (38-pin) real-hardware controller
// Role: deterministic sensing, interlocks, chute + trapdoor control, safe-state logic.
// The Raspberry Pi only classifies; it can never move an actuator.
//
// Libraries (Arduino Library Manager): "HX711" (bogde), "VL53L1X" (Pololu),
// "Adafruit INA219", "ESP32Servo". Board: ESP32 Dev Module.
//
// UNTESTED ON HARDWARE. Bring up one subsystem at a time (see README order),
// keep the stepper supply fused, and test with the chute disconnected first.

#include <Wire.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <HX711.h>
#include <VL53L1X.h>
#include <Adafruit_INA219.h>

// ---------------- pin map ----------------
#define PIN_STEP     18   // A4988 STEP
#define PIN_DIR      19   // A4988 DIR
#define PIN_EN       23   // to interlock gate EN_ESP: HIGH = disabled (external 10k pull-up to 3V3)
#define PIN_SERVO    13   // trapdoor servo signal (servo powered from 5V rail, not from the ESP32)
#define PIN_HX_DT     4   // HX711 DOUT
#define PIN_HX_SCK    5   // HX711 SCK
#define PIN_SDA      21   // I2C: VL53L1X (0x29) + INA219 (0x40)
#define PIN_SCL      22
#define PIN_PI_RX    16   // Serial2 RX <- Pi TXD (GPIO14, header pin 8). 3.3 V both sides.
#define PIN_PI_TX    17   // Serial2 TX -> Pi RXD (GPIO15, header pin 10)
#define PIN_BEAM     27   // intake beam-break, LOW = item entered
#define PIN_METAL    26   // IND_OUT via optocoupler (external 10k pull-up), LOW = metal
#define PIN_DROP     25   // drop beam, LOW = item passed
#define PIN_HOME     33   // chute home / ORGANIC index, LOW = aligned
#define PIN_IDX_DRY  35   // DRY index sensor, LOW = aligned (input-only pin: needs external 10k pull-up)
#define PIN_IDX_RES  39   // RESIDUAL index sensor, LOW = aligned (input-only: external 10k pull-up)
#define PIN_HAND     32   // hand / second-item beam at intake, LOW = blocked
#define PIN_COVER    14   // cover switch to GND when closed. HIGH (open or wire broken) = unsafe
#define PIN_LIGHT    15   // MOSFET gate for LED ring (10k gate pull-down)
#define PIN_STATUS    2   // on-board LED

// ---------------- tunables (calibrate on the real machine) ----------------
#define USE_INDEX_SENSORS 1
#define DIR_FORWARD_LEVEL HIGH            // swap to LOW if the chute turns the wrong way
const int   STEPS_PER_REV     = 200;
const int   POS_STEPS[3]      = {0, 67, 133};   // organic(home), dry, residual - trim with index sensors
const int   ALIGN_SEARCH_STEPS = 8;
const int   STEP_DELAY_US     = 1200;
const int   DOOR_CLOSED       = 0;
const int   DOOR_OPEN         = 90;
const float CONF_HIGH         = 0.85;     // placeholders: tune from validation data
const float CONF_LOW          = 0.50;
const int   MASS_MIN_G        = 5;
const int   MASS_MAX_G        = 2000;
const int   HEIGHT_MAX_MM     = 150;
const float CURRENT_LIMIT_MA  = 1800;     // supply-side limit; see note on stepper stall detection
const int   CURRENT_STRIKES   = 3;
const float DRIFT_LIMIT_G     = 40;
const unsigned long DECISION_TIMEOUT_MS = 8000;
const unsigned long DROP_TIMEOUT_MS     = 4000;
const unsigned long INTERLOCK_WAIT_MS   = 10000;
const unsigned long LIGHT_SETTLE_MS     = 400;
const int   PI_TIMEOUTS_TO_DEGRADE = 3;

enum State { ST_IDLE, ST_WAIT_DECISION, ST_LOCKOUT };
enum Route { S_ORGANIC = 0, S_DRY = 1, S_RESIDUAL = 2, S_NEEDMORE = 3 };
const char* ROUTE_NAME[3] = {"organic", "dry", "residual"};

HX711 scale;
VL53L1X tof;
Adafruit_INA219 ina;
Servo door;
Preferences prefs;

State state = ST_IDLE;
int  curPos = 0;
bool retried = false;
bool piDegraded = false;
int  piTimeouts = 0;
bool massOk = false, tofOk = false, curOk = false;
float calFactor = 1.0;
int   tofEmptyMm = 0;
unsigned long decisionStart = 0, lastTelemetry = 0, lastIdleCheck = 0, lastBlink = 0;
String piBuf, usbBuf;

int  gMass = 0, gHeight = 0;
bool gMetal = false;
unsigned long cntOrg = 0, cntDry = 0, cntRes = 0, cntFaults = 0, cntConflicts = 0;

// ---------------- sensor access ----------------
bool metalPresent()    { return digitalRead(PIN_METAL) == LOW; }
bool itemEntered()     { return digitalRead(PIN_BEAM) == LOW; }
bool dropDetected()    { return digitalRead(PIN_DROP) == LOW; }
bool interlockBlocked(){ return digitalRead(PIN_HAND) == LOW || digitalRead(PIN_COVER) == HIGH; }

int readMassG() {
  if (!massOk) return -1;
  if (!scale.wait_ready_timeout(300)) { massOk = false; Serial.println("EVENT,loadcell_timeout"); return -1; }
  return (int)scale.get_units(5);
}
int readTofMm() {
  if (!tofOk) return -1;
  uint16_t d = tof.read();
  if (tof.timeoutOccurred()) { tofOk = false; Serial.println("EVENT,tof_timeout"); return -1; }
  return d;
}
int readHeightMm() {
  int d = readTofMm();
  if (d < 0) return -1;
  int h = tofEmptyMm - d;           // sensor looks down at the tray
  return h < 0 ? 0 : h;
}
void captureTofEmpty() {
  long sum = 0; int n = 0;
  for (int i = 0; i < 10; i++) { int d = readTofMm(); if (d > 0) { sum += d; n++; } delay(60); }
  if (n > 0) { tofEmptyMm = sum / n; prefs.putInt("tofEmpty", tofEmptyMm); }
  Serial.print("EVENT,tof_empty_mm="); Serial.println(tofEmptyMm);
}
void calibrateScale(float knownG) {
  if (!massOk || knownG <= 0) { Serial.println("ERR,cal_unavailable"); return; }
  scale.set_scale();                              // factor 1
  long raw = scale.get_value(15);                 // average minus offset (tare first with empty tray)
  calFactor = raw / knownG;
  scale.set_scale(calFactor);
  prefs.putFloat("cal", calFactor);
  Serial.print("EVENT,cal_factor="); Serial.println(calFactor, 4);
}
bool sensorsHealthy() { return massOk && tofOk; }

// ---------------- helpers ----------------
void doorClose() { door.write(DOOR_CLOSED); }
void doorOpen()  { door.write(DOOR_OPEN); }

void enterLockout(const char* why) {
  digitalWrite(PIN_LIGHT, LOW);
  doorClose();
  digitalWrite(PIN_EN, HIGH);                    // disable stepper driver
  state = ST_LOCKOUT;
  cntFaults++;
  Serial.print("FAULT,LOCKOUT,"); Serial.println(why);
  Serial.println("ALERT,maintenance required. Send R to reset.");
}

bool waitInterlockClear(unsigned long maxMs) {
  unsigned long t0 = millis();
  while (interlockBlocked()) {
    if (millis() - t0 > maxMs) return false;
    delay(10);
  }
  return true;
}

// Signed step move. Returns false on interlock timeout or sustained over-current.
// NOTE: a stepper driver in chopper mode draws roughly constant current even when
// stalled, so current is only a gross-fault check. Lost steps are caught by the index
// sensors in alignTo(); that is the real jam/alignment check.
bool moveSteps(int n) {
  if (n == 0) return true;
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_DIR, n > 0 ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
  int count = abs(n), strikes = 0;
  for (int i = 0; i < count; i++) {
    if (!waitInterlockClear(INTERLOCK_WAIT_MS)) return false;
    if (curOk && (i % 8 == 0)) {
      if (ina.getCurrent_mA() > CURRENT_LIMIT_MA) {
        if (++strikes >= CURRENT_STRIKES) { Serial.println("EVENT,motor_overcurrent"); return false; }
      } else strikes = 0;
    }
    digitalWrite(PIN_STEP, HIGH); delayMicroseconds(STEP_DELAY_US);
    digitalWrite(PIN_STEP, LOW);  delayMicroseconds(STEP_DELAY_US);
  }
  return true;
}

bool aligned(int idx) {
  switch (idx) {
    case 0:  return digitalRead(PIN_HOME) == LOW;
    case 1:  return digitalRead(PIN_IDX_DRY) == LOW;
    default: return digitalRead(PIN_IDX_RES) == LOW;
  }
}

// Confirm chute is on the commanded outlet; small search window absorbs step error.
bool alignTo(int idx) {
#if USE_INDEX_SENSORS
  if (aligned(idx)) return true;
  for (int dirn = 1; dirn >= -1; dirn -= 2) {
    for (int i = 0; i < ALIGN_SEARCH_STEPS; i++) {
      if (!moveSteps(dirn)) return false;
      if (aligned(idx)) { curPos = POS_STEPS[idx]; return true; }
    }
    if (!moveSteps(-dirn * ALIGN_SEARCH_STEPS)) return false;
  }
  return false;
#else
  return true;
#endif
}

bool homeChute() {
  doorClose();
  if (!waitInterlockClear(INTERLOCK_WAIT_MS)) return false;
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_DIR, !DIR_FORWARD_LEVEL);
  for (int i = 0; i < STEPS_PER_REV + 20; i++) {
    if (digitalRead(PIN_HOME) == LOW) { curPos = 0; return true; }
    digitalWrite(PIN_STEP, HIGH); delayMicroseconds(STEP_DELAY_US);
    digitalWrite(PIN_STEP, LOW);  delayMicroseconds(STEP_DELAY_US);
  }
  return false;
}

// ---------------- decision policy ----------------
Route decide(const String& cls, float conf, bool metal, bool* conflict) {
  *conflict = false;
  if (cls == "OTH") return S_RESIDUAL;
  if (conf < CONF_LOW) return S_RESIDUAL;
  bool expectMetal = (cls == "CAN");
  if (metal != expectMetal) { *conflict = true; return S_RESIDUAL; }
  if (conf < CONF_HIGH) return retried ? S_RESIDUAL : S_NEEDMORE;
  return (cls == "ORG") ? S_ORGANIC : S_DRY;
}

// ---------------- one sorting cycle ----------------
void executeRoute(Route s) {
  digitalWrite(PIN_LIGHT, LOW);
  int target = POS_STEPS[s];
  if (!moveSteps(target - curPos)) { enterLockout("chute_move_failed"); return; }
  curPos = target;
  if (!alignTo(s)) { enterLockout("chute_not_aligned"); return; }      // trapdoor stays closed
  Serial.print("EVENT,aligned,"); Serial.println(ROUTE_NAME[s]);

  if (!waitInterlockClear(INTERLOCK_WAIT_MS)) { enterLockout("interlock_timeout"); return; }
  doorOpen();
  unsigned long t0 = millis();
  bool dropped = false;
  while (millis() - t0 < DROP_TIMEOUT_MS) { if (dropDetected()) { dropped = true; break; } delay(5); }

  if (!dropped) {                                                      // one controlled recovery attempt
    Serial.println("EVENT,drop_not_confirmed,recovery_pulse");
    doorClose(); delay(300); doorOpen();
    t0 = millis();
    while (millis() - t0 < 2000) { if (dropDetected()) { dropped = true; break; } delay(5); }
  }
  if (!dropped) { enterLockout("drop_not_confirmed"); return; }

  doorClose();
  delay(300);
  if (s == S_ORGANIC) cntOrg++; else if (s == S_DRY) cntDry++; else cntRes++;
  if (!moveSteps(-curPos)) { enterLockout("home_return_failed"); return; }
  curPos = 0;
  if (!alignTo(S_ORGANIC)) { enterLockout("home_lost_steps"); return; }
  Serial.print("RESULT,"); Serial.println(ROUTE_NAME[s]);
  state = ST_IDLE;
}

// ---------------- Pi link ----------------
void sendRequest() {
  String m = "REQ,mass_g=" + String(gMass) + ",metal=" + String(gMetal ? 1 : 0) + ",height_mm=" + String(gHeight);
  Serial2.println(m);                  // to Pi
  Serial.println(m);                   // mirror to USB
}

void handleDecisionLine(String line) {
  line.trim();
  if (!line.startsWith("RES,")) return;
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c2 < 0) { Serial.println("ERR,bad_format"); return; }
  String cls = line.substring(c1 + 1, c2);
  float conf = line.substring(c2 + 1).toFloat();
  piTimeouts = 0; piDegraded = false;
  bool conflict;
  Route s = decide(cls, conf, gMetal, &conflict);
  if (conflict) { cntConflicts++; Serial.println("EVENT,sensor_conflict,routed_residual"); }
  if (s == S_NEEDMORE) {
    retried = true;
    decisionStart = millis();
    Serial.println("EVENT,mid_confidence,requesting_one_more_view");
    sendRequest();
    return;
  }
  Serial.print("DECISION,"); Serial.print(cls); Serial.print(","); Serial.println(conf, 2);
  executeRoute(s);
}

void startCycle() {
  doorClose();
  digitalWrite(PIN_LIGHT, HIGH);                 // lighting on for the camera
  Serial.println("EVENT,item_entered,intake_flap_closed");
  delay(LIGHT_SETTLE_MS);
  gMass = readMassG();
  gHeight = readHeightMm();
  gMetal = metalPresent();
  retried = false;

  if (!sensorsHealthy()) {                       // invalid sensor -> degraded, residual only
    Serial.println("EVENT,sensor_invalid,degraded_residual");
    executeRoute(S_RESIDUAL);
    return;
  }
  if (gMass < MASS_MIN_G || gMass > MASS_MAX_G || gHeight > HEIGHT_MAX_MM) {
    Serial.println("EVENT,out_of_supported_range,routed_residual");
    executeRoute(S_RESIDUAL);
    return;
  }
  if (piDegraded) {
    Serial.println("EVENT,pi_degraded,routed_residual");
    executeRoute(S_RESIDUAL);
    return;
  }
  sendRequest();
  decisionStart = millis();
  state = ST_WAIT_DECISION;
}

// ---------------- serial helpers ----------------
bool readLine(Stream& port, String& buf, String& out) {
  while (port.available()) {
    char c = port.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { out = buf; buf = ""; return true; }
    } else if (buf.length() < 80) buf += c;
  }
  return false;
}

void telemetry() {
  String j = String("DASH,{\"organic\":") + cntOrg + ",\"dry\":" + cntDry + ",\"residual\":" + cntRes +
             ",\"conflicts\":" + cntConflicts + ",\"faults\":" + cntFaults +
             ",\"mass_ok\":" + (massOk ? 1 : 0) + ",\"tof_ok\":" + (tofOk ? 1 : 0) +
             ",\"pi_degraded\":" + (piDegraded ? 1 : 0) + ",\"state\":" + (int)state + "}";
  Serial.println(j);
  Serial2.println(j);                            // Pi can forward this to the facility dashboard
}

void handleCommand(String c) {                   // USB maintenance commands (idle or lockout)
  c.trim();
  if (c == "R" && state == ST_LOCKOUT) {
    Serial.println("EVENT,manual_reset");
    if (homeChute() && alignTo(S_ORGANIC)) { state = ST_IDLE; Serial.println("READY"); }
    else enterLockout("homing_failed");
  } else if (c == "TARE" && massOk) { scale.tare(15); Serial.println("EVENT,tared"); }
  else if (c.startsWith("CAL,")) calibrateScale(c.substring(4).toFloat());
  else if (c == "ZERO") captureTofEmpty();
  else if (c == "STATUS") {
    Serial.print("STATUS,mass_ok="); Serial.print(massOk); Serial.print(",tof_ok="); Serial.print(tofOk);
    Serial.print(",cur_ok="); Serial.print(curOk); Serial.print(",cal="); Serial.print(calFactor, 4);
    Serial.print(",tof_empty_mm="); Serial.println(tofEmptyMm);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, HIGH);          // driver disabled until homed
  pinMode(PIN_LIGHT, OUTPUT); digitalWrite(PIN_LIGHT, LOW);
  pinMode(PIN_STATUS, OUTPUT);
  pinMode(PIN_BEAM, INPUT_PULLUP); pinMode(PIN_METAL, INPUT_PULLUP);
  pinMode(PIN_DROP, INPUT_PULLUP); pinMode(PIN_HOME, INPUT_PULLUP);
  pinMode(PIN_HAND, INPUT_PULLUP); pinMode(PIN_COVER, INPUT_PULLUP);
  pinMode(PIN_IDX_DRY, INPUT); pinMode(PIN_IDX_RES, INPUT);      // external pull-ups (input-only pins)
  door.attach(PIN_SERVO);
  doorClose();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  prefs.begin("sortsure", false);
  calFactor  = prefs.getFloat("cal", 1.0);
  tofEmptyMm = prefs.getInt("tofEmpty", 0);

  scale.begin(PIN_HX_DT, PIN_HX_SCK);
  massOk = scale.wait_ready_timeout(1000);
  if (massOk) { scale.set_scale(calFactor); scale.tare(10); }
  tof.setTimeout(200);
  tofOk = tof.init();
  if (tofOk) {
    tof.setDistanceMode(VL53L1X::Short);
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(50);
    if (tofEmptyMm <= 0) captureTofEmpty();
  }
  curOk = ina.begin();

  Serial.println("SortSure controller boot");
  Serial.print("SENSORS,mass="); Serial.print(massOk); Serial.print(",tof="); Serial.print(tofOk);
  Serial.print(",current="); Serial.println(curOk);
  if (!homeChute()) { enterLockout("homing_failed"); return; }   // never accept items with unknown position
  Serial.println("READY");
}

void loop() {
  String line;
  if (readLine(Serial2, piBuf, line) && state == ST_WAIT_DECISION) handleDecisionLine(line);
  if (readLine(Serial, usbBuf, line)) {
    if (line.startsWith("RES,") && state == ST_WAIT_DECISION) handleDecisionLine(line);   // manual override for bench tests
    else handleCommand(line);
  }

  switch (state) {
    case ST_IDLE:
      if (itemEntered() && !interlockBlocked()) { startCycle(); break; }
      if (millis() - lastIdleCheck > 10000 && !itemEntered() && massOk) {   // empty-tray zero check
        lastIdleCheck = millis();
        float m = scale.get_units(3);
        if (fabs(m) < DRIFT_LIMIT_G) scale.tare(5);
        else Serial.println("EVENT,tray_not_empty_or_drift");
      }
      if (millis() - lastBlink > 500) { lastBlink = millis(); digitalWrite(PIN_STATUS, !digitalRead(PIN_STATUS)); }
      break;
    case ST_WAIT_DECISION:
      if (millis() - decisionStart > DECISION_TIMEOUT_MS) {
        piTimeouts++;
        Serial.print("EVENT,pi_timeout,count="); Serial.println(piTimeouts);
        if (piTimeouts >= PI_TIMEOUTS_TO_DEGRADE) piDegraded = true;
        executeRoute(S_RESIDUAL);                    // no answer -> residual
      }
      break;
    case ST_LOCKOUT:
      digitalWrite(PIN_STATUS, HIGH);
      break;
  }

  if (millis() - lastTelemetry > 10000) { lastTelemetry = millis(); telemetry(); }
}
