// ----------------------------
// CYD Mushroom Monitor
// SCD40 (CO2/Temp/RH) -> MQTT + 3-screen TFT display
//
// Screen 1: date/time + live sensor readings (text)
// Screen 2: 6-hour humidity graph
// Screen 3: 6-hour CO2 graph
// Tap screen to cycle 1 -> 2 -> 3 -> 1 ...
//
// StatusLED reflects health:
//   boot()    - during setup()
//   ok()      - everything nominal
//   warning() - MQTT disconnected (sensor still OK)
//   error()   - SCD40 read/communication failure
//
// Graphs draw directly to the TFT (no sprite/double-buffer) -- an
// earlier sprite-based version was dropped because the ~150KB sprite
// allocation competed with WiFi/TLS heap usage and silently failed.
// Direct drawing means a brief flicker on redraw, which is a fine
// tradeoff for a sensor monitor that redraws every 20s, not a UI
// that needs to look polished under constant motion.
// ----------------------------

#include <SPI.h>
#include <Wire.h>
#include <time.h>

#include <XPT2046_Touchscreen.h>
#include <TFT_eSPI.h>
#include <StatusLED.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SensirionI2cScd4x.h>

#include "secrets.h"
// Expected in secrets.h (same directory, not checked into version control):
// #define WIFI_SSID "..."
// #define WIFI_PASSWORD "..."
// #define MQTT_SERVER "..."
// #define MQTT_PORT 8883
// #define MQTT_USER "..."
// #define MQTT_PASS "..."

// ----------------------------
// I2C pins (CYD)
// ----------------------------
#define I2C_SDA 27
#define I2C_SCL 22

// ----------------------------
// Touch screen pins (CYD non-default SPI)
// ----------------------------
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// ----------------------------
// Timing
// ----------------------------
static const unsigned long SENSOR_INTERVAL_MS = 20000UL;       // read + publish every 20s
static const unsigned long HISTORY_WINDOW_S    = 6UL * 60 * 60; // 6 hours
static const unsigned long HISTORY_SAMPLE_S    = SENSOR_INTERVAL_MS / 1000UL; // 20s
static const size_t HISTORY_LEN = HISTORY_WINDOW_S / HISTORY_SAMPLE_S; // 1080 points

// MQTT topic
static const char* MQTT_TOPIC = "DIRTPI/MUSH01/sensors";

// NTP / timezone (US Pacific, handles PST/PDT automatically)
static const char* NTP_SERVER = "pool.ntp.org";
static const char* TZ_STRING  = "PST8PDT,M3.2.0,M11.1.0";

// ----------------------------
// Globals: peripherals
// ----------------------------
SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
TFT_eSPI tft = TFT_eSPI();

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);
char mqttClientId[32];

SensirionI2cScd4x scd4x;
static char scdErrorMessage[64];

// ----------------------------
// Sensor history ring buffers
// ----------------------------
float humidityHistory[HISTORY_LEN];
float co2History[HISTORY_LEN];
size_t historyCount = 0;   // how many valid samples so far (<= HISTORY_LEN)
size_t historyHead = 0;    // index where the NEXT sample will be written

// Latest readings (for screen 1)
float lastTempC = NAN;
float lastHumidity = NAN;
float lastCO2 = NAN;
bool haveReading = false;

// ----------------------------
// Screen state
// ----------------------------
enum Screen { SCREEN_READINGS, SCREEN_HUMIDITY_GRAPH, SCREEN_CO2_GRAPH, SCREEN_COUNT };
Screen currentScreen = SCREEN_READINGS;
bool screenDirty = true; // force redraw on first loop / screen change

unsigned long lastSensorRead = 0;

// ----------------------------
// Graph layout constants (shared by both graph screens)
// ----------------------------
static const int GRAPH_X = 34;       // leave room for Y-axis labels
static const int GRAPH_Y = 28;
static const int GRAPH_W = 280;
static const int GRAPH_H = 165;
static const int GRAPH_Y_TICKS = 4;  // number of horizontal grid lines (excluding top/bottom)
static const int GRAPH_X_TICKS = 6;  // number of vertical time-axis ticks

// ----------------------------
// Forward declarations
// ----------------------------
void connectWiFi();
void reconnectMqtt();
void setupNtpClock();
void readAndPublishSensor();
void pushHistory(float humidity, float co2);
void drawReadingsScreen();
void drawGraphScreen(float* data, size_t count, size_t head, const char* title,
                      float yMin, float yMax, uint16_t lineColor, const char* unit);
void handleTouch();

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  StatusLED.begin();
  StatusLED.boot();

  // TFT init (do this BEFORE touch SPI setup -- tft.init() can disturb
  // SPI peripheral state, and we want touch's SPI config to be the
  // last one established before we start polling it)
  tft.init();
  tft.setRotation(1); // landscape
  tft.fillScreen(TFT_BLACK);

  // Touch SPI + init
  mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(mySpi);
  ts.setRotation(1);

  int x = 320 / 2;
  tft.drawCentreString("Connecting to:", x, 80, 2);
  tft.drawCentreString(WIFI_SSID, x, 100, 2);

  connectWiFi();

  tft.fillScreen(TFT_BLACK);
  tft.drawCentreString("Connected!", x, 80, 2);
  tft.drawCentreString(WiFi.localIP().toString().c_str(), x, 100, 2);
  delay(1200);

  // NTP clock
  tft.fillScreen(TFT_BLACK);
  tft.drawCentreString("Syncing time...", x, 100, 2);
  setupNtpClock();

  // Unique MQTT client ID from MAC (avoids ID collisions across devices)
  uint64_t mac = ESP.getEfuseMac();
  snprintf(mqttClientId, sizeof(mqttClientId), "mush01-%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);

  secureClient.setInsecure(); // TODO: replace with real cert validation later
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  // I2C + SCD40
  tft.fillScreen(TFT_BLACK);
  tft.drawCentreString("Starting SCD40...", x, 100, 2);
  Wire.begin(I2C_SDA, I2C_SCL);
  scd4x.begin(Wire, SCD41_I2C_ADDR_62); // SCD40 uses the same default address as SCD41

  // Make sure no previous measurement is running, then start fresh
  scd4x.stopPeriodicMeasurement();
  delay(500);
  uint16_t err = scd4x.startPeriodicMeasurement();
  if (err) {
    errorToString(err, scdErrorMessage, sizeof(scdErrorMessage));
    Serial.print("SCD40 start error: ");
    Serial.println(scdErrorMessage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCentreString("SCD40 init FAILED", x, 100, 2);
    StatusLED.error();
    // Don't hard-halt -- sensor errors should be visible/recoverable, not bricking the device.
    // We continue; readAndPublishSensor() will keep retrying and reflect failure on the LED.
  } else {
    Serial.println("SCD40 measurement started (first reading in ~5s, interval lower-bounded by sensor)");
  }

  tft.fillScreen(TFT_BLACK);
  StatusLED.ok();
  screenDirty = true;
}

// ============================================================
// Loop
// ============================================================
void loop() {
  StatusLED.update(); // non-blocking, must run every loop

  // --- MQTT connection upkeep ---
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconnectMqtt();
    }
    mqttClient.loop();
  }

  // --- Sensor read + publish on interval ---
  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = millis();
    readAndPublishSensor();
    screenDirty = true; // new data available, refresh whichever screen is showing
  }

  // --- Touch handling (screen advance) ---
  handleTouch();

  // --- Redraw current screen if needed ---
  if (screenDirty) {
    switch (currentScreen) {
      case SCREEN_READINGS:
        drawReadingsScreen();
        break;
      case SCREEN_HUMIDITY_GRAPH:
        drawGraphScreen(humidityHistory, historyCount, historyHead,
                         "Humidity - last 6h", 0.0f, 100.0f, TFT_CYAN, "%");
        break;
      case SCREEN_CO2_GRAPH:
        drawGraphScreen(co2History, historyCount, historyHead,
                         "CO2 - last 6h", 0.0f, 3000.0f, TFT_GREEN, "ppm");
        break;
      default:
        break;
    }
    screenDirty = false;
  }
}

// ============================================================
// WiFi
// ============================================================
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int dotCount = 0;
  int dotX = 120;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tft.fillCircle(dotX + (dotCount * 20), 130, 3, TFT_WHITE);
    dotCount = (dotCount + 1) % 8;
    if (dotCount == 0) {
      tft.fillRect(dotX, 120, 160, 20, TFT_BLACK);
    }
  }
  Serial.println("\nWiFi connected");
}

// ============================================================
// MQTT
// ============================================================
void reconnectMqtt() {
  // Single attempt per call, no internal delay() loop -- if it fails,
  // warning() stays active and we just try again next loop pass.
  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(mqttClientId, MQTT_USER, MQTT_PASS)) {
    Serial.println("connected");
    StatusLED.ok();
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
    StatusLED.warning();
  }
}

// ============================================================
// NTP
// ============================================================
void setupNtpClock() {
  configTzTime(TZ_STRING, NTP_SERVER);

  struct tm timeinfo;
  // Wait briefly for first sync; don't hang forever if NTP is unreachable.
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 20) {
    delay(250);
    attempts++;
  }
  if (attempts >= 20) {
    Serial.println("NTP sync timed out - clock may be wrong until it syncs later");
  } else {
    Serial.println("NTP time synced");
  }
}

// ============================================================
// Sensor read + publish + history push
// ============================================================
void readAndPublishSensor() {
  bool dataReady = false;
  uint16_t err = scd4x.getDataReadyStatus(dataReady);
  if (err) {
    errorToString(err, scdErrorMessage, sizeof(scdErrorMessage));
    Serial.print("getDataReadyStatus error: ");
    Serial.println(scdErrorMessage);
    StatusLED.error();
    haveReading = false;
    return;
  }
  if (!dataReady) {
    // Not an error - just no new sample yet at this poll. Leave LED state as-is.
    return;
  }

  uint16_t co2 = 0;
  float temperature = 0.0f;
  float humidity = 0.0f;
  err = scd4x.readMeasurement(co2, temperature, humidity);
  if (err) {
    errorToString(err, scdErrorMessage, sizeof(scdErrorMessage));
    Serial.print("readMeasurement error: ");
    Serial.println(scdErrorMessage);
    StatusLED.error();
    haveReading = false;
    return;
  }

  // Good reading
  lastTempC = temperature;
  lastHumidity = humidity;
  lastCO2 = (float)co2;
  haveReading = true;

  pushHistory(humidity, (float)co2);

  Serial.print("CO2: "); Serial.print(co2); Serial.print(" ppm  ");
  Serial.print("T: "); Serial.print(temperature, 1); Serial.print(" C  ");
  Serial.print("RH: "); Serial.print(humidity, 1); Serial.println(" %");

  // Publish to MQTT if connected
  if (mqttClient.connected()) {
    JsonDocument doc;
    doc["m_temperature"] = temperature;
    doc["m_humidity"] = humidity;
    doc["m_co2"] = co2;

    char payload[128];
    serializeJson(doc, payload);
    mqttClient.publish(MQTT_TOPIC, payload);
    StatusLED.ok();
  } else {
    // Sensor is fine, but we couldn't publish - that's a connectivity warning,
    // not a sensor error.
    StatusLED.warning();
  }
}

void pushHistory(float humidity, float co2) {
  humidityHistory[historyHead] = humidity;
  co2History[historyHead] = co2;
  historyHead = (historyHead + 1) % HISTORY_LEN;
  if (historyCount < HISTORY_LEN) {
    historyCount++;
  }
}

// ============================================================
// Screen 1: date/time + readings
// ============================================================
void drawReadingsScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int x = 320 / 2;

  // Date/time
  struct tm timeinfo;
  char dateBuf[32];
  char timeBuf[32];
  if (getLocalTime(&timeinfo)) {
    strftime(dateBuf, sizeof(dateBuf), "%a %b %d %Y", &timeinfo);
    strftime(timeBuf, sizeof(timeBuf), "%I:%M:%S %p", &timeinfo);
  } else {
    snprintf(dateBuf, sizeof(dateBuf), "Date unavailable");
    snprintf(timeBuf, sizeof(timeBuf), "--:--:--");
  }

  tft.setTextSize(1);
  tft.drawCentreString(dateBuf, x, 10, 4);
  tft.drawCentreString(timeBuf, x, 34, 4);

  tft.drawFastHLine(10, 64, 300, TFT_WHITE);

  if (!haveReading) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCentreString("No sensor data yet", x, 110, 4);
    return;
  }

  float tempF = lastTempC * 9.0f / 5.0f + 32.0f;

  int y = 80;
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawCentreString("CO2: " + String((int)(lastCO2 + 0.5f)) + " ppm", x, y, 4);

  y += 40;
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawCentreString("RH: " + String(lastHumidity, 1) + " %", x, y, 4);

  y += 40;
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawCentreString(String(lastTempC, 1) + " C  /  " + String(tempF, 1) + " F", x, y, 4);

  // MQTT connectivity indicator (small, bottom corner)
  tft.setTextSize(1);
  tft.setTextColor(mqttClient.connected() ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(mqttClient.connected() ? "MQTT OK" : "MQTT DOWN", 10, 220, 2);
}

// ============================================================
// Screens 2 & 3: history graphs
// Draws directly to the TFT. Includes Y-axis gridlines/labels (value
// + unit) and X-axis time ticks (hours ago, based on actual sample
// spacing) rather than just a bare line.
// ============================================================
void drawGraphScreen(float* data, size_t count, size_t head, const char* title,
                      float yMin, float yMax, uint16_t lineColor, const char* unit) {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(title, 160, 4, 2);

  // Outer plot box
  tft.drawRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, TFT_DARKGREY);

  // ---- Y-axis: horizontal gridlines + value labels ----
  tft.setTextDatum(MR_DATUM); // middle-right, so labels sit just left of the axis
  for (int i = 0; i <= GRAPH_Y_TICKS; i++) {
    float frac = (float)i / (float)GRAPH_Y_TICKS;       // 0 (bottom) .. 1 (top)
    int yPix = GRAPH_Y + GRAPH_H - (int)(frac * GRAPH_H);
    float value = yMin + frac * (yMax - yMin);

    // Skip the outermost lines exactly on the box border to avoid double-drawing,
    // but still draw their labels.
    if (i != 0 && i != GRAPH_Y_TICKS) {
      tft.drawFastHLine(GRAPH_X, yPix, GRAPH_W, TFT_DARKGREY);
    }
    tft.drawString(String((int)value), GRAPH_X - 4, yPix, 1);
  }

  // Unit label, rotated conceptually by just placing it above the axis labels
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(unit, 2, GRAPH_Y - 2, 1);

  // ---- X-axis: vertical ticks for elapsed time ----
  // Ticks are spaced evenly across the plotted window, labeled in
  // "hours ago" based on the actual sample interval, not a fixed
  // assumption -- if the buffer isn't full yet (device just booted),
  // the labeled span shrinks accordingly instead of pretending there
  // are 6 hours of data that don't exist yet.
  float spanSeconds = (count > 1) ? (float)(count - 1) * HISTORY_SAMPLE_S : 0;
  tft.setTextDatum(TC_DATUM);
  for (int i = 0; i <= GRAPH_X_TICKS; i++) {
    float frac = (float)i / (float)GRAPH_X_TICKS; // 0 (oldest) .. 1 (newest)
    int xPix = GRAPH_X + (int)(frac * GRAPH_W);

    if (i != 0 && i != GRAPH_X_TICKS) {
      tft.drawFastVLine(xPix, GRAPH_Y, GRAPH_H, TFT_DARKGREY);
    }

    float secondsAgo = spanSeconds * (1.0f - frac);
    float hoursAgo = secondsAgo / 3600.0f;
    String label = (i == GRAPH_X_TICKS) ? "now" : (String(hoursAgo, 1) + "h");
    tft.drawString(label, xPix, GRAPH_Y + GRAPH_H + 4, 1);
  }

  if (count < 2) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Gathering data...", 160, GRAPH_Y + GRAPH_H / 2, 2);
    return;
  }

  // ---- Plot the data line ----
  // Oldest-first index: when buffer isn't full yet, oldest is index 0;
  // once full, oldest is at `head` (the wraparound point).
  size_t oldestIdx = (count < HISTORY_LEN) ? 0 : head;

  float prevX = -1, prevY = -1;
  for (size_t i = 0; i < count; i++) {
    size_t idx = (oldestIdx + i) % HISTORY_LEN;
    float v = data[idx];
    if (v < yMin) v = yMin;
    if (v > yMax) v = yMax;

    float plotX = GRAPH_X + (GRAPH_W * (float)i / (float)(count - 1));
    float plotY = GRAPH_Y + GRAPH_H - (GRAPH_H * (v - yMin) / (yMax - yMin));

    if (prevX >= 0) {
      tft.drawLine((int)prevX, (int)prevY, (int)plotX, (int)plotY, lineColor);
    }
    prevX = plotX;
    prevY = plotY;
  }

  // Latest value, called out explicitly below the plot
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(lineColor, TFT_BLACK);
  float latestVal = data[(head + HISTORY_LEN - 1) % HISTORY_LEN];
  tft.drawString("Latest: " + String(latestVal, 1) + " " + unit,
                 GRAPH_X, GRAPH_Y + GRAPH_H + 16, 2);
}

// ============================================================
// Touch -> advance screen
// ============================================================
void handleTouch() {
  if (ts.tirqTouched() && ts.touched()) {
    TS_Point p = ts.getPoint();
    (void)p; // position unused - any tap just advances the screen
    currentScreen = (Screen)((currentScreen + 1) % SCREEN_COUNT);
    screenDirty = true;
    delay(150); // simple debounce so one physical tap doesn't register multiple times
  }
}