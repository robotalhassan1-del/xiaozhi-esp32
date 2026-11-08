// Room node v2: ESP32 DevKit V1 at the wall switch box.
// It asks the robot's server (long polling) which of 3 lamps and 1 fan should be on and drives a
// 4-channel relay module. The wall switches keep working (relay contacts are wired in parallel).
//
// Wi-Fi: on every start it joins the hotspot it knows. If it cannot join within 30 seconds, it opens its own
// Wi-Fi "Alhassan_robot" (password below) with a web page where you choose a new hotspot.
// The server token is built into the firmware (put by the GitHub build from a secret), not asked on the page.

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ---- settings you may change ----
#define SERVER_HOST   "robot-bridge.onrender.com"
#define ROOM_TOKEN    "__ROOM_TOKEN__"          // replaced by the GitHub build (secret ROOM_TOKEN)
#define AP_NAME       "Galaxym11"          // name of the setup Wi-Fi
#define AP_PASSWORD   "Galaxy123"            // password of the setup Wi-Fi (8 characters or more)
#define PORTAL_AFTER_MS 30000                   // open the setup Wi-Fi after this long without a hotspot
#define RELAY_ACTIVE_LOW 1                      // 1 = relay switches ON when the pin is LOW (most modules)

// ---- pins ----
#define NUM_RELAYS 4
static const uint8_t RELAY_PINS[NUM_RELAYS] = {16, 17, 18, 19};   // IN1..IN4: lamp1, lamp2, lamp3, fan
static const uint8_t LED_PINS[NUM_RELAYS]   = {25, 26, 27, 32};   // indicator LED of each relay (with 330 ohm)
static const char*   RELAY_KEYS[NUM_RELAYS] = {"lamp1", "lamp2", "lamp3", "fan"};
#define STATUS_LED_EXT 33    // Wi-Fi status LED (with 330 ohm)
#define STATUS_LED_INT 2     // blue LED on the DevKit board
#define RESET_BUTTON 0       // BOOT button: hold 5 s to forget the hotspot

Preferences prefs;
WebServer web(80);
DNSServer dns;
char wifiSsid[33] = "";
char wifiPass[65] = "";
bool relayOn[NUM_RELAYS] = {false, false, false, false};
long lastVersion = -1;
bool serverOk = false;
bool portalOn = false;
uint32_t lostSince = 0;
uint32_t lastTry = 0;

static void statusLed(bool on) {
  digitalWrite(STATUS_LED_EXT, on ? HIGH : LOW);
  digitalWrite(STATUS_LED_INT, on ? HIGH : LOW);
}

static void blinkStatus(int periodMs) {
  statusLed((millis() / periodMs) % 2);
}

static void writeRelay(int i, bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(RELAY_PINS[i], level ? HIGH : LOW);
  digitalWrite(LED_PINS[i], on ? HIGH : LOW);
}

static void setRelay(int i, bool on) {
  if (relayOn[i] == on) return;
  relayOn[i] = on;
  writeRelay(i, on);
  Serial.printf("Relay %s -> %s\n", RELAY_KEYS[i], on ? "ON" : "OFF");
}

static String htmlEscape(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '"') o += "&quot;";
    else o += c;
  }
  return o;
}

// ---------------- setup page ----------------
static void handleRoot() {
  int n = WiFi.scanNetworks();
  String page =
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Alhassan Robot</title><style>body{font-family:sans-serif;margin:16px;max-width:480px}"
      "input,select,button{width:100%;box-sizing:border-box;font-size:18px;padding:10px;margin:6px 0}"
      "button{background:#1e63e0;color:#fff;border:0;border-radius:8px}</style></head><body>"
      "<h2>Alhassan Robot: saita hotspot</h2>"
      "<form method='POST' action='/save'>"
      "<label>Zabi hotspot (2.4 GHz):</label><select name='ssid_list'><option value=''>-- zabi --</option>";
  for (int i = 0; i < n; i++) {
    String name = WiFi.SSID(i);
    if (name.length() == 0) continue;
    page += "<option value=\"" + htmlEscape(name) + "\">" + htmlEscape(name) + " (" + String(WiFi.RSSI(i)) + ")</option>";
  }
  page +=
      "</select><label>ko ka rubuta sunan hotspot:</label><input name='ssid' maxlength='32' placeholder='sunan hotspot'>"
      "<label>Kalmar sirri:</label><input name='pass' type='password' maxlength='64'>"
      "<button type='submit'>Ajiye</button></form>"
      "<p>Hotspot na yanzu: " + htmlEscape(String(wifiSsid)) + "</p></body></html>";
  WiFi.scanDelete();
  web.send(200, "text/html", page);
}

static void handleSave() {
  String ssid = web.arg("ssid");
  if (ssid.length() == 0) ssid = web.arg("ssid_list");
  String pass = web.arg("pass");
  if (ssid.length() == 0 || ssid.length() > 32 || pass.length() > 64) {
    web.send(400, "text/html", "<meta charset='utf-8'><p>Sunan hotspot ba daidai ba. <a href='/'>Koma</a></p>");
    return;
  }
  prefs.begin("room", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
  web.send(200, "text/html", "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                             "<h3>An ajiye. Na'urar tana sake farawa...</h3>");
  delay(1500);
  ESP.restart();
}

static void handleRedirect() {
  web.sendHeader("Location", "http://192.168.4.1/", true);
  web.send(302, "text/plain", "");
}

static void startPortal() {
  if (portalOn) return;
  Serial.println("Opening setup Wi-Fi: " AP_NAME);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_NAME, AP_PASSWORD);
  delay(200);
  dns.start(53, "*", WiFi.softAPIP());
  web.begin();
  portalOn = true;
}

static void stopPortal() {
  if (!portalOn) return;
  Serial.println("Closing setup Wi-Fi");
  web.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  portalOn = false;
}

static void tryConnect() {
  if (wifiSsid[0] == 0) return;
  WiFi.disconnect();
  WiFi.begin(wifiSsid, wifiPass);
  lastTry = millis();
}

static void checkResetButton() {
  if (digitalRead(RESET_BUTTON) != LOW) return;
  uint32_t start = millis();
  while (digitalRead(RESET_BUTTON) == LOW) {
    if (millis() - start > 5000) {
      Serial.println("Forgetting the hotspot...");
      for (int i = 0; i < 6; i++) { statusLed(i % 2); delay(100); }
      prefs.begin("room", false);
      prefs.remove("ssid");
      prefs.remove("pass");
      prefs.end();
      ESP.restart();
    }
    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_RELAYS; i++) {      // everything OFF before the pins become outputs
    writeRelay(i, false);
    pinMode(RELAY_PINS[i], OUTPUT);
    pinMode(LED_PINS[i], OUTPUT);
    writeRelay(i, false);
  }
  pinMode(STATUS_LED_EXT, OUTPUT);
  pinMode(STATUS_LED_INT, OUTPUT);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  statusLed(false);

  prefs.begin("room", true);
  String s = prefs.getString("ssid", "");
  String p = prefs.getString("pass", "");
  prefs.end();
  s.toCharArray(wifiSsid, sizeof(wifiSsid));
  p.toCharArray(wifiPass, sizeof(wifiPass));

  web.on("/", HTTP_GET, handleRoot);
  web.on("/save", HTTP_POST, handleSave);
  web.onNotFound(handleRedirect);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  lostSince = millis();
  if (wifiSsid[0] == 0) {
    startPortal();                            // nothing saved yet: open the setup Wi-Fi at once
  } else {
    Serial.printf("Joining hotspot: %s\n", wifiSsid);
    tryConnect();
  }
}

static void pollServer() {
  WiFiClientSecure client;
  client.setInsecure();   // the token travels inside TLS; the server certificate is not checked
  HTTPClient http;
  String url = String("https://") + SERVER_HOST + "/room/state?token=" + ROOM_TOKEN + "&since=" + String(lastVersion);
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
        JsonVariant v = doc["state"][RELAY_KEYS[i]];
        if (!v.isNull()) setRelay(i, v.as<int>() != 0);
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
  checkResetButton();
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (portalOn) {
    dns.processNextRequest();
    web.handleClient();
  }

  if (connected) {
    lostSince = 0;
    // Close the setup Wi-Fi once we are online and nobody is using the page.
    if (portalOn && WiFi.softAPgetStationNum() == 0) stopPortal();
  } else {
    serverOk = false;
    if (lostSince == 0) lostSince = millis();
    if (!portalOn && millis() - lostSince >= PORTAL_AFTER_MS) startPortal();
    if (wifiSsid[0] != 0 && millis() - lastTry > 20000) tryConnect();
    blinkStatus(125);    // fast blink: looking for the hotspot
    delay(5);
    return;
  }

  if (portalOn) {        // somebody is using the setup page: keep it responsive
    statusLed(true);
    delay(5);
    return;
  }

  pollServer();
  if (serverOk) {
    statusLed(true);                       // steady: online
    delay(100);
  } else {
    for (int i = 0; i < 6; i++) { blinkStatus(500); delay(100); }   // slow blink: Wi-Fi ok, server not
  }
}
