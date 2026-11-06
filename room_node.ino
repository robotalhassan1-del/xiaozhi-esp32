// Room node: ESP32 DevKit V1 at the wall switch box.
// It asks the robot's server (long polling) which of 3 lamps and 1 fan should be on,
// and drives a 4-channel relay module. The physical wall switches keep working
// (the relay contacts are wired in parallel with them).
//
// Wi-Fi name, password, server and token are NOT in this file. On the first start the board opens
// a Wi-Fi called "RoomNode-Setup"; connect with your phone and fill the form.
// Hold the BOOT button for 5 seconds to erase the settings and set it up again.

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define NUM_RELAYS 4
static const uint8_t RELAY_PINS[NUM_RELAYS] = {16, 17, 18, 19};          // IN1..IN4
static const char* RELAY_KEYS[NUM_RELAYS] = {"lamp1", "lamp2", "lamp3", "fan"};
#define RELAY_ACTIVE_LOW 1   // most 4-channel modules switch ON when the input pin is LOW. Set 0 if yours is the opposite.
#define STATUS_LED 2         // the blue LED on the DevKit board
#define RESET_BUTTON 0       // BOOT button

Preferences prefs;
WiFiManager wm;
char serverHost[64] = "robot-bridge.onrender.com";
char token[48] = "";
bool relayOn[NUM_RELAYS] = {false, false, false, false};
long lastVersion = -1;
bool serverOk = false;
volatile bool shouldSave = false;

static void writeRelay(int i, bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(RELAY_PINS[i], level ? HIGH : LOW);
}

static void setRelay(int i, bool on) {
  if (relayOn[i] == on) return;
  relayOn[i] = on;
  writeRelay(i, on);
  Serial.printf("Relay %s -> %s\n", RELAY_KEYS[i], on ? "ON" : "OFF");
}

static void checkResetButton() {
  if (digitalRead(RESET_BUTTON) != LOW) return;
  uint32_t start = millis();
  while (digitalRead(RESET_BUTTON) == LOW) {
    if (millis() - start > 5000) {
      Serial.println("Erasing Wi-Fi settings...");
      for (int i = 0; i < 6; i++) { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); delay(100); }
      wm.resetSettings();
      prefs.begin("room", false);
      prefs.clear();
      prefs.end();
      ESP.restart();
    }
    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  // All relays OFF first, before the pins become outputs.
  for (int i = 0; i < NUM_RELAYS; i++) {
    writeRelay(i, false);
    pinMode(RELAY_PINS[i], OUTPUT);
    writeRelay(i, false);
  }
  pinMode(STATUS_LED, OUTPUT);
  pinMode(RESET_BUTTON, INPUT_PULLUP);

  prefs.begin("room", false);
  String h = prefs.getString("host", serverHost);
  String t = prefs.getString("token", "");
  h.toCharArray(serverHost, sizeof(serverHost));
  t.toCharArray(token, sizeof(token));
  prefs.end();

  WiFi.mode(WIFI_STA);
  WiFiManagerParameter pHost("host", "Server (robot-bridge.onrender.com)", serverHost, 63);
  WiFiManagerParameter pToken("token", "Token (ROOM_TOKEN)", token, 47);
  wm.addParameter(&pHost);
  wm.addParameter(&pToken);
  wm.setSaveParamsCallback([]() { shouldSave = true; });
  wm.setConfigPortalTimeout(300);

  if (!wm.getWiFiIsSaved()) {
    // First start: open the setup Wi-Fi and wait for the phone.
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("No Wi-Fi saved: starting setup portal 'RoomNode-Setup'");
    bool ok = wm.startConfigPortal("RoomNode-Setup");
    strlcpy(serverHost, pHost.getValue(), sizeof(serverHost));
    strlcpy(token, pToken.getValue(), sizeof(token));
    prefs.begin("room", false);
    prefs.putString("host", serverHost);
    prefs.putString("token", token);
    prefs.end();
    if (!ok) ESP.restart();
  } else {
    // Saved network: just keep trying to join it (the hotspot may be switched on later).
    WiFi.setAutoReconnect(true);
    WiFi.begin();
  }
  Serial.printf("Server: %s\n", serverHost);
}

static void blinkStatus(int periodMs) {
  digitalWrite(STATUS_LED, (millis() / periodMs) % 2 ? HIGH : LOW);
}

static void pollServer() {
  WiFiClientSecure client;
  client.setInsecure();   // the token travels inside TLS; the server certificate is not checked
  HTTPClient http;
  String url = String("https://") + serverHost + "/room/state?token=" + token + "&since=" + String(lastVersion);
  http.setReuse(false);
  http.setConnectTimeout(10000);
  http.setTimeout(35000);
  if (!http.begin(client, url)) {
    serverOk = false;
    delay(3000);
    return;
  }
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
      for (int i = 0; i < NUM_RELAYS; i++) {
        JsonVariant s = doc["state"][RELAY_KEYS[i]];
        if (!s.isNull()) setRelay(i, s.as<int>() != 0);
      }
      lastVersion = doc["v"] | -1L;
      serverOk = true;
    } else {
      serverOk = false;
    }
  } else {
    Serial.printf("Poll failed, HTTP %d\n", code);
    serverOk = false;
    delay(3000);
  }
  http.end();
}

void loop() {
  static uint32_t lostSince = 0;
  checkResetButton();
  if (WiFi.status() != WL_CONNECTED) {
    serverOk = false;
    if (lostSince == 0) lostSince = millis();
    if (millis() - lostSince > 10UL * 60UL * 1000UL) ESP.restart();  // give the Wi-Fi stack a fresh start
    blinkStatus(125);   // fast blink: no Wi-Fi
    delay(50);
    return;
  }
  lostSince = 0;
  pollServer();
  if (serverOk) {
    digitalWrite(STATUS_LED, HIGH);      // steady: connected to the server
    delay(100);
  } else {
    for (int i = 0; i < 6; i++) { blinkStatus(500); delay(100); }   // slow blink: Wi-Fi ok, server not
  }
}
