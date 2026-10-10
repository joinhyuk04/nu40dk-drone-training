/* NU40-DK / MPU6050 or onboard LSM6DS3 tilt drone controller. Serial 115200.
 * Upload target: nucode:nrf52:nu40dk. No external IMU library.
 * Qwiic SDA=P0.26, SCL=P0.27, VCC=3.3V, GND=GND.
 * Protocol: T:<roll degrees>,<pitch degrees> at 20Hz (<=20 bytes).
 * Commands: GET:STATUS, CALIBRATE, GAME:running/paused/hit/score.
 * SW1 start/pause, SW2 calibrate, SW3 restart, SW4 rotate controls.
 */
#include <Arduino.h>
#include <bluefruit.h>
#include <Wire.h>
#include <math.h>

BLEService droneService("8f6d1001-6a4b-4c32-9a78-4d5e6f708192");
BLECharacteristic events("8f6d1002-6a4b-4c32-9a78-4d5e6f708192");
BLECharacteristic commands("8f6d1003-6a4b-4c32-9a78-4d5e6f708192");
const uint8_t BUTTONS[] = {PIN_BUTTON1, PIN_BUTTON2, PIN_BUTTON3, PIN_BUTTON4};
// SW1 P0.11, SW2 P0.12, SW3 P0.24, SW4 P0.25 (LOW when pressed).
const uint8_t LEDS[] = {PIN_LED1, PIN_LED2, PIN_LED3, PIN_LED4};
// LED0 P0.13, LED1 P0.14, LED2 P0.15, LED3 P0.16; BSP controls polarity.
char boardName[16], pendingEvent[21] = {0};
QueueHandle_t commandQueue;
bool ready = false, scanning = false, settling = false, filterValid = false;
bool candidate[4], raw[4], stable[4], running = false, calibrating = false;
const uint8_t IMU_ADDRESSES[] = {0x68,0x69,0x6A,0x6B};
bool lsm = false;
uint8_t address = 0, scanAddress = 1, nextCandidate = 0, failures = 0, ledMask = 255;
uint32_t lastProbe = 0, settleAt = 0, lastSample = 0, lastPacket = 0, lastStatus = 0, lastDebug = 0;
uint32_t changedAt[4], flashAt = 0;
uint8_t flashMask = 0;
float roll = 0, pitch = 0, zeroRoll = 0, zeroPitch = 0, biasX = 0, biasY = 0;
float sumGX = 0, sumGY = 0, sumRoll = 0, sumPitch = 0;
uint8_t calibrationCount = 0;

void makeName() {
  const char letters[] = "ABCDEFGHJKLMNPQRSTUVWXYZ", digits[] = "23456789";
  uint64_t v = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
  v ^= v >> 30; v *= UINT64_C(0xbf58476d1ce4e5b9);
  v ^= v >> 27; v *= UINT64_C(0x94d049bb133111eb); v ^= v >> 31;
  memcpy(boardName, "NU40-DRN-", 9);
  for (uint8_t i = 0; i < 3; ++i) {
    boardName[9 + 2*i] = letters[v % 24]; v /= 24;
    boardName[10 + 2*i] = digits[v % 8]; v /= 8;
  }
  boardName[15] = 0;
}

void emit(const char *message) {
  events.write(message, strlen(message));
  strncpy(pendingEvent, message, 20); pendingEvent[20] = 0;
  Serial.print("[EVENT] "); Serial.println(message);
}
void written(uint16_t conn, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
  (void)conn; (void)chr;
  if (!commandQueue || len == 0 || len > 20) return;
  char text[21] = {0}; memcpy(text, data, len); xQueueSend(commandQueue, text, 0);
}
void printStatus() {
  Serial.print("[BLE] "); Serial.print(boardName); Serial.print(" | IMU: ");
  if (ready) { Serial.print(lsm ? "READY LSM6DS3 @ 0x" : "READY MPU6050 @ 0x"); Serial.println(address, HEX); }
  else Serial.println("MISSING; retry every 5s");
}
void startCalibration() {
  if (!ready) { emit("IMU:MISSING"); return; }
  calibrating = true; calibrationCount = 0;
  sumGX = sumGY = sumRoll = sumPitch = 0; emit("CAL:START");
}
void serviceCommands() {
  char text[21];
  for (uint8_t n = 0; n < 4 && xQueueReceive(commandQueue, text, 0) == pdTRUE; ++n) {
    if (!strcmp(text,"GET:STATUS")) emit(ready ? "IMU:READY" : "IMU:MISSING");
    else if (!strcmp(text,"CALIBRATE")) startCalibration();
    else if (!strcmp(text,"CAL:CANCEL")) calibrating = false;
    else if (!strcmp(text,"GAME:running")) running = true;
    else if (!strcmp(text,"GAME:paused")) running = false;
    else if (!strcmp(text,"GAME:hit")) { flashAt = millis(); flashMask = 15; }
    else if (!strcmp(text,"GAME:score")) { flashAt = millis(); flashMask = 5; }
  }
}
bool readRegs(uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) { while (Wire.available()) Wire.read(); return false; }
  for (size_t i = 0; i < count; ++i) data[i] = Wire.read(); return true;
}
bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
int16_t wordAt(const uint8_t *p) { return lsm ? (int16_t)(((uint16_t)p[1] << 8) | p[0]) : (int16_t)(((uint16_t)p[0] << 8) | p[1]); }
float wrapAngle(float v) { while (v > 180) v -= 360; while (v < -180) v += 360; return v; }
void tiltPacket(char *packet, size_t size) {
  // Do not rely on printf float support in the installed ARM C library.
  long r = lroundf(wrapAngle(roll-zeroRoll)*10), p = lroundf(wrapAngle(pitch-zeroPitch)*10);
  snprintf(packet,size,"T:%s%ld.%ld,%s%ld.%ld",
    r<0 ? "-" : "",labs(r)/10,labs(r)%10,
    p<0 ? "-" : "",labs(p)/10,labs(p)%10);
}
void missing(uint32_t now) {
  ready = false; settling = false; filterValid = false; calibrating = false;
  failures = 0; lastProbe = now; emit("IMU:MISSING"); printStatus();
}
void scan(uint32_t now) {
  scanning = true; scanAddress = 1; nextCandidate = 0;
  for (uint8_t i = 0; i < 4; ++i) candidate[i] = false; lastProbe = now;
  Serial.println("[I2C] Full scan 0x01..0x7E / SDA 26 SCL 27 / 100kHz");
}
bool configureImu(bool verify) {
  const uint8_t mpuRegs[] = {0x6B,0x1A,0x19,0x1B,0x1C}, mpuValues[] = {1,3,4,0,0};
  // LSM6DS3: 104Hz, +/-2g, +/-245dps, BDU + register auto-increment.
  const uint8_t lsmRegs[] = {0x12,0x10,0x11}, lsmValues[] = {0x44,0x40,0x40};
  const uint8_t *regs = lsm ? lsmRegs : mpuRegs, *values = lsm ? lsmValues : mpuValues;
  uint8_t count = lsm ? 3 : 5, value = 0;
  for (uint8_t j = 0; j < count; ++j) {
    if (verify) { if (!readRegs(regs[j],&value,1) || value != values[j]) return false; }
    else if (!writeReg(regs[j],values[j])) return false;
  }
  return true;
}
void tryImu(uint32_t now) {
  while (nextCandidate < 4) {
    uint8_t i = nextCandidate++; if (!candidate[i]) continue;
    address = IMU_ADDRESSES[i]; lsm = i >= 2; uint8_t who = 0;
    if (!readRegs(lsm ? 0x0F : 0x75,&who,1) || who == 0 || who == 255) continue;
    Serial.print(lsm ? "[IMU] LSM6DS3 0x" : "[IMU] MPU6050 0x");
    Serial.print(address,HEX); Serial.print(" WHO_AM_I=0x"); Serial.println(who,HEX);
    if (lsm && who != 0x69 && who != 0x6A) { Serial.println("[IMU] Unknown ST device; skipped"); continue; }
    if (!lsm && who != 0x68 && who != 0x69) Serial.println("[IMU] WARNING: compatible register mode");
    if (!configureImu(false)) continue;
    settling = true; settleAt = now; return;
  }
  missing(now);
}
void serviceImu(uint32_t now) {
  if (scanning) {
    Wire.beginTransmission(scanAddress);
    if (Wire.endTransmission() == 0) {
      Serial.print("[I2C] Found 0x"); Serial.println(scanAddress,HEX);
      for (uint8_t i = 0; i < 4; ++i) if (scanAddress == IMU_ADDRESSES[i]) candidate[i] = true;
    }
    if (++scanAddress > 126) { scanning = false; Serial.println("[I2C] Scan complete"); tryImu(now); }
    return;
  }
  if (settling) {
    if (now - settleAt < 100) return;
    bool ok = configureImu(true); uint8_t data[14];
    if (!readRegs(lsm ? 0x22 : 0x3B,data,lsm ? 12 : 14)) ok = false;
    settling = false;
    if (!ok) { Serial.println("[IMU] Verification failed"); tryImu(now); return; }
    ready = true; filterValid = false; failures = 0; zeroRoll = zeroPitch = biasX = biasY = 0;
    lastSample = now; emit("IMU:READY"); printStatus(); return;
  }
  if (!ready) { if (now-lastProbe >= 5000) scan(now); return; }
  if (now-lastSample < 20) return;
  float dt = (now-lastSample)*0.001f;
  if (dt > 0.08f) filterValid = false;
  lastSample = now;
  uint8_t data[14];
  if (!readRegs(lsm ? 0x22 : 0x3B,data,lsm ? 12 : 14)) {
    filterValid = false;
    if (++failures >= 3) { Serial.println("[IMU] Connection lost"); missing(now); }
    return;
  }
  failures = 0;
  const uint8_t *acc = data + (lsm ? 6 : 0), *gyr = data + (lsm ? 0 : 8);
  float ax = wordAt(acc)* (lsm ? 0.000061f : 1.0f/16384), ay = wordAt(acc+2)*(lsm ? 0.000061f : 1.0f/16384), az = wordAt(acc+4)*(lsm ? 0.000061f : 1.0f/16384);
  float gx = wordAt(gyr)*(lsm ? 0.00875f : 1.0f/131), gy = wordAt(gyr+2)*(lsm ? 0.00875f : 1.0f/131);
  float ar = atan2f(ay,az)*180.0f/PI;
  float ap = atan2f(-ax,sqrtf(ay*ay+az*az))*180.0f/PI;
  float magnitude = sqrtf(ax*ax+ay*ay+az*az);
  if (!filterValid) { roll = ar; pitch = ap; filterValid = true; }
  else {
    roll = wrapAngle(roll + (gx-biasX)*dt); pitch = wrapAngle(pitch + (gy-biasY)*dt);
    // Reject strong translation; blend gravity with gyro for steady control.
    if (magnitude > 0.75f && magnitude < 1.25f) {
      roll = wrapAngle(roll + 0.06f*wrapAngle(ar-roll));
      pitch = wrapAngle(pitch + 0.06f*wrapAngle(ap-pitch));
    }
  }
  if (calibrating) {
    if (fabsf(gx)>8 || fabsf(gy)>8 || magnitude<0.90f || magnitude>1.10f) {
      calibrationCount = 0; sumGX = sumGY = sumRoll = sumPitch = 0;
      return; // Need 60 consecutive still samples; keep BLE/buttons responsive.
    }
    sumGX += gx; sumGY += gy; sumRoll += ar; sumPitch += ap;
    if (++calibrationCount >= 60) {
      biasX = sumGX/60; biasY = sumGY/60; zeroRoll = sumRoll/60; zeroPitch = sumPitch/60;
      roll = zeroRoll; pitch = zeroPitch; calibrating = false; emit("CAL:DONE"); lastPacket = now;
    }
  }
}
void serviceBle(uint32_t now) {
  if (!Bluefruit.connected() || !events.notifyEnabled()) { pendingEvent[0] = 0; return; }
  if (pendingEvent[0]) {
    if (events.notify(pendingEvent,strlen(pendingEvent))) pendingEvent[0] = 0;
    return;
  }
  if (!ready || calibrating || !filterValid || now-lastPacket < 50) return;
  lastPacket = now; char packet[21];
  tiltPacket(packet,sizeof(packet));
  events.write(packet,strlen(packet)); events.notify(packet,strlen(packet));
}
void serviceButtons(uint32_t now) {
  const char *names[] = {"SW1","SW2","SW3","SW4"};
  for (uint8_t i = 0; i < 4; ++i) {
    bool pressed = digitalRead(BUTTONS[i]) == LOW;
    if (pressed != raw[i]) { raw[i] = pressed; changedAt[i] = now; }
    if (pressed != stable[i] && now-changedAt[i] >= 35) {
      stable[i] = pressed; if (pressed) emit(names[i]);
    }
  }
}
void serviceLeds(uint32_t now) {
  uint8_t mask;
  if (flashMask && now-flashAt < 400) mask = ((now-flashAt)/80)%2 ? 0 : flashMask;
  else if (calibrating) mask = 1 << ((now/130)%4);
  else if (!ready) mask = (now/700)%2 ? 0 : 15;
  else if (!running) mask = (now/1100)%2 ? 0 : 1;
  else {
    float r = wrapAngle(roll-zeroRoll), p = wrapAngle(pitch-zeroPitch);
    mask = (r < -4 ? 1 : 0) | (p > 4 ? 2 : 0) | (p < -4 ? 4 : 0) | (r > 4 ? 8 : 0);
  }
  if (mask != ledMask) {
    ledMask = mask;
    for (uint8_t i = 0; i < 4; ++i) { if (mask&(1<<i)) ledOn(LEDS[i]); else ledOff(LEDS[i]); }
  }
}
void setup() {
  Serial.begin(115200); makeName();
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(BUTTONS[i],INPUT_PULLUP); pinMode(LEDS[i],OUTPUT); ledOff(LEDS[i]);
    raw[i] = stable[i] = digitalRead(BUTTONS[i]) == LOW; changedAt[i] = millis();
  }
  commandQueue = xQueueCreate(16,21);
  Bluefruit.begin(); Bluefruit.autoConnLed(false); Bluefruit.setName(boardName); Bluefruit.setTxPower(4);
  droneService.begin();
  events.setProperties(CHR_PROPS_READ|CHR_PROPS_NOTIFY); events.setPermission(SECMODE_OPEN,SECMODE_NO_ACCESS);
  events.setMaxLen(20); events.begin();
  commands.setProperties(CHR_PROPS_WRITE); commands.setPermission(SECMODE_NO_ACCESS,SECMODE_OPEN);
  commands.setMaxLen(20); commands.setWriteCallback(written); commands.begin();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(droneService); Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true); Bluefruit.Advertising.setInterval(32,244);
  Bluefruit.Advertising.setFastTimeout(30); Bluefruit.Advertising.start(0);
  Wire.setPins(26,27); Wire.begin(); Wire.setClock(100000);
  emit("IMU:MISSING"); printStatus(); scan(millis());
}
void loop() {
  uint32_t now = millis();
  if (commandQueue) serviceCommands();
  serviceButtons(now); serviceImu(now); serviceBle(now); serviceLeds(now);
  if (ready && filterValid && now-lastDebug >= 1000) {
    lastDebug = now; char packet[21]; tiltPacket(packet,sizeof(packet));
    Serial.print("[TILT] "); Serial.print(packet);
    Serial.print(" | BLE="); Serial.print(Bluefruit.connected() ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" NOTIFY="); Serial.print(events.notifyEnabled() ? "ON" : "OFF");
    Serial.print(" CAL="); Serial.println(calibrating ? "RUNNING" : "IDLE");
  }
  if (now-lastStatus >= 10000) { lastStatus = now; printStatus(); }
}
