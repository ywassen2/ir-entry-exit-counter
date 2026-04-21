#include <HardwareSerial.h>
#include <esp_system.h>

#define SIM_RX 16
#define SIM_TX 17

// Designated pins on the ESP32s
#define beamOne 34
#define beamTwo 35

bool beamOneTripped = false;
bool beamTwoTripped = false;

// Tracks the "first broken" sensor to determine direction
// 0 = none, 1 = 1 broke first, 2 = 2 broke first
int firstBroken = 0;

// Previous sensor states to detect transitions
int prevOne = 1;
int prevTwo = 1;

// People count
int peopleCount = 15;

// Debounce timing
unsigned long lastEventTime = 0;
const unsigned long DEBOUNCE_MS = 200;   // Ignore re-triggers within 200ms
const unsigned long TIMEOUT_MS  = 30000; // Reset sequence if 2nd beam not broken in 30s
unsigned long sequenceStartTime = 0;

unsigned long lastSend = 0;
const unsigned long INTERVAL = 10000;  // Send every 10 seconds

// Network / server configuration
const char APN[]        = "america.bics";
const char* SERVER_IP   = "167.172.146.172";
const int   SERVER_PORT = 3000;
const char* BUS_ID      = "WillVill/Main Campus";

HardwareSerial sim(2);


// ---------------------------------------------------------------------------
// waitFor — read from sim until ack string found or timeout, NO buffer flush
// ---------------------------------------------------------------------------
bool waitFor(const char* ack, uint32_t timeoutMs) {
  uint32_t start = millis();
  String response = "";
  while (millis() - start < timeoutMs) {
    while (sim.available()) response += (char)sim.read();
    if (response.indexOf(ack) >= 0) return true;
    delay(1);
  }
  return false;
}

// ---------------------------------------------------------------------------
// sendAT — write a command, wait up to timeoutMs for ack string
// ---------------------------------------------------------------------------
bool sendAT(const char* cmd, const char* ack, uint32_t timeoutMs = 3000) {
  while (sim.available()) sim.read();   // flush
  if (strlen(cmd) > 0) sim.println(cmd);
  uint32_t start = millis();
  String response = "";
  while (millis() - start < timeoutMs) {
    while (sim.available()) response += (char)sim.read();
    if (response.indexOf(ack) >= 0) return true;
    delay(1);
  }
  return false;
}

// ---------------------------------------------------------------------------
// initModem — configure radio for LTE-M / NB-IoT on eiotclub / AT&T BICS
// ---------------------------------------------------------------------------
bool initModem() {
  Serial.println(F("[INIT] Waiting for modem to boot..."));
  delay(3000);

  for (uint8_t i = 0; i < 60; i++) {
    if (sendAT("AT", "OK", 3000)) {
      Serial.println(F("[INIT] Modem responding"));
      break;
    }
    if (i == 59) {
      Serial.println(F("[INIT] Modem not responding after 60 attempts — restarting"));
      return false;
    }
    delay(500);
  }

  sendAT("ATE0",       "OK");
  sendAT("AT+CFUN=1",  "OK", 5000);
  sendAT("AT+CSCLK=0", "OK");

  sendAT("AT+CNMP=51", "OK", 3000);  // GSM + LTE
  sendAT("AT+CMNB=3",  "OK", 3000);  // Cat-M1 + NB-IoT auto

  sendAT("AT+CPIN?",   "READY");

  sendAT("AT+CGDCONT=1,\"IP\",\"america.bics\"", "OK", 3000);

  sendAT("AT+SAPBR=3,1,\"Contype\",\"GPRS\"",     "OK", 3000);
  sendAT("AT+SAPBR=3,1,\"APN\",\"america.bics\"", "OK", 3000);
  sendAT("AT+SAPBR=0,1",                           "OK", 5000);  // close if stale

  return true;
}

// ---------------------------------------------------------------------------
// waitForNetwork — poll CEREG until home (,1) or roaming (,5)
// ---------------------------------------------------------------------------
bool waitForNetwork(uint8_t maxAttempts = 60) {
  Serial.println(F("[NET] Waiting for LTE-M / NB-IoT registration..."));
  for (uint8_t i = 0; i < maxAttempts; i++) {
    sim.println("AT+CEREG?");
    delay(500);
    String resp = "";
    uint32_t t = millis();
    while (millis() - t < 1000) {
      while (sim.available()) resp += (char)sim.read();
      delay(1);
    }
    Serial.print(F("<< ")); Serial.println(resp);
    if (resp.indexOf(",1") >= 0 || resp.indexOf(",5") >= 0) {
      Serial.println(F("[NET] Registered!"));
      return true;
    }
    delay(2000);
  }
  Serial.println(F("[NET] Registration timed out"));
  return false;
}

// ---------------------------------------------------------------------------
// initHTTP — open a fresh HTTP session (fire-and-forget HTTPTERM first)
// ---------------------------------------------------------------------------
void initHTTP() {
  while (sim.available()) sim.read();
  sim.println("AT+HTTPTERM");
  delay(1000);
  while (sim.available()) sim.read();

  sendAT("AT+HTTPINIT",          "OK", 5000);
  sendAT("AT+HTTPPARA=\"CID\",1", "OK", 3000);
  Serial.println(F("[HTTP] Session initialised"));
}

// ---------------------------------------------------------------------------
// sendOccupancyData — HTTP GET with query params (avoids HTTPDATA firmware bug)
// ---------------------------------------------------------------------------
void sendOccupancyData(int count) {
  Serial.print(F("\n[SEND] Bus: "));
  Serial.print(BUS_ID);
  Serial.print(F(" | Count: "));
  Serial.println(count);

  // Build URL with query parameters — no body/HTTPDATA needed
  String url = "http://";
  url += SERVER_IP;
  url += ":";
  url += SERVER_PORT;
  url += "/api/occupancy?bus_id=";
  url += BUS_ID;
  url += "&count=";
  url += count;

  String httpUrl = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  if (!sendAT(httpUrl.c_str(), "OK", 3000)) {
    Serial.println(F("!! HTTPPARA URL failed — reinitialising HTTP session"));
    initHTTP();
    return;
  }

  // GET request — raw send so +HTTPACTION URC is not consumed by sendAT
  while (sim.available()) sim.read();
  sim.println("AT+HTTPACTION=0");
  Serial.println(F(">> AT+HTTPACTION=0 (GET)"));

  // Collect +HTTPACTION: 0,<status>,<len>
  String actionResp = "";
  uint32_t actionStart = millis();
  while (millis() - actionStart < 90000) {
    while (sim.available()) actionResp += (char)sim.read();
    if (actionResp.indexOf("+HTTPACTION") >= 0) break;
    if (actionResp.indexOf("ERROR") >= 0) {
      Serial.print(F("!! HTTPACTION ERROR: ")); Serial.println(actionResp);
      initHTTP();
      return;
    }
    delay(1);
  }
  Serial.print(F("<< ")); Serial.println(actionResp);

  // Read response body
  int readLen = 0;
  int lastComma = actionResp.lastIndexOf(',');
  if (lastComma >= 0) readLen = actionResp.substring(lastComma + 1).toInt();
  if (readLen > 0) {
    String readCmd = "AT+HTTPREAD=0,";
    readCmd += min(readLen, 512);
    sendAT(readCmd.c_str(), "+HTTPREAD:", 5000);
  }

  // Keep HTTP session alive — do NOT HTTPTERM between sends
  Serial.println(F("[SEND] Done"));
}


void setup() {
  Serial.begin(115200);

  while(true) {
  Serial.printf("RAW B1=%d  B2=%d\n", digitalRead(beamOne), digitalRead(beamTwo));
  delay(200);
}
  Serial.println(F("\n[BOOT] People counter + SIM7000x / eiotclub"));

  delay(10000);  // give modem time to finish booting
  sim.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(1000);

  if (!initModem()) {
    Serial.println(F("[FATAL] Modem init failed — restarting in 5s"));
    delay(5000);
    esp_restart();
  }
  if (!waitForNetwork()) {
    Serial.println(F("[FATAL] No network — restarting in 5s"));
    delay(5000);
    esp_restart();
  }

  sendAT("AT+SAPBR=1,1", "OK", 15000);
  delay(500);
  while (sim.available()) sim.read();

  initHTTP();

  pinMode(beamOne, INPUT);
  pinMode(beamTwo, INPUT);

  Serial.println(F("\n[READY] People counter running."));
}


void loop() {
  unsigned long now = millis();

  // Periodic send
  if (now - lastSend >= INTERVAL) {
    lastSend = now;
    sendOccupancyData(peopleCount);
  }

  beamOneTripped = !digitalRead(beamOne);
  beamTwoTripped = !digitalRead(beamTwo);

  // Fast beam state display (every 100ms)
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.printf("B1:%s  B2:%s  | first:%d  count:%d\n",
      beamOneTripped ? "BROKEN" : "OK    ",
      beamTwoTripped ? "BROKEN" : "OK    ",
      firstBroken, peopleCount);
  }

  // --- Timeout: reset sequence if person took too long ---
  if (firstBroken != 0 && (now - sequenceStartTime > TIMEOUT_MS)) {
    firstBroken = 0;
  }

  // --- Detect falling edge (beam just broken: HIGH → LOW) ---
  bool oneJustBroken = (prevOne == 1 && !beamOneTripped == 0);
  bool twoJustBroken = (prevTwo == 1 && !beamTwoTripped == 0);

  // --- Step 1: Record which sensor broke first ---
  if (firstBroken == 0) {
    if (oneJustBroken && (now - lastEventTime > DEBOUNCE_MS)) {
      firstBroken = 1;
      sequenceStartTime = now;
    } else if (twoJustBroken && (now - lastEventTime > DEBOUNCE_MS)) {
      firstBroken = 2;
      sequenceStartTime = now;
    }
  }

  // --- Step 2: Confirm direction when second sensor breaks ---
  else if (firstBroken == 1 && twoJustBroken) {
    peopleCount++;
    Serial.printf(">>> ENTRY  count=%d\n", peopleCount);
    lastEventTime = now;
    firstBroken = 0;
  }
  else if (firstBroken == 2 && oneJustBroken) {
    peopleCount = max(0, peopleCount - 1);
    Serial.printf("<<< EXIT   count=%d\n", peopleCount);
    lastEventTime = now;
    firstBroken = 0;
  }

  // Update previous states
  prevOne = !beamOneTripped;
  prevTwo = !beamTwoTripped;
}
