/**
 * CPEnvPro
 * Cardputer ADV + ENV Pro Unit (BME688) — synthwave-themed environment dashboard.
 *
 * Dependent libraries (Arduino Library Manager):
 *   M5Unified, M5GFX, M5Cardputer, M5UnitUnified, M5Utility, M5HAL,
 *   M5Unit-ENV, BME68x Sensor library, bsec2
 *
 * Config is a plain "key=value" text file, not JSON — no JSON library, hand
 * rolled JSON string just for the HTTP response. This is deliberate, not a
 * missed dependency: this sketch is meant to be installed as an app binary
 * (Sketch > Export Compiled Binary) into an existing OTA app slot on a
 * Launcher-managed device, and Launcher's actual OTA partitions on this
 * hardware are ~1.25MB each (confirmed from another sketch's build output
 * on this same device) — comfortably smaller than the default 4MB Arduino
 * scheme's nominal 1.3MB, and ArduinoJson alone was the difference between
 * fitting and not.
 *
 * Controls:
 *   `  (backtick, plain tap) - screen off / step back a level in network or logging mode
 *   Enter                    - screen on / confirm selection / refresh the graph
 *   m                        - open menu (from dashboard) / back to dashboard
 *   ; / .                    - scroll menu, status rows, scan list, or interval list
 *   , / /                    - graph mode only: previous / next metric
 *   space                    - toggle temperature unit C / F (dashboard, and the TEMP graph)
 *   Fn+Backspace             - cancel out of WiFi password or log filename entry
 *   (while typing a password or filename, all keys above type normally instead)
 *
 * Logging notes:
 *   - Timestamp column is blank unless network mode is on AND NTP has synced
 *     (checked via a plausible-epoch floor), in UTC. Column is always present
 *     so the CSV's shape never changes row to row.
 *   - 10 SEC is the fastest logging interval on purpose — it matches the
 *     screen-off sensor-poll throttle (SENSOR_POLL_IDLE_INTERVAL_MS), so
 *     logging never has to override power saving to stay fresh.
 *   - Each OFF->ON toggle (or filename save while already on) starts a new
 *     file: "<name>_<N>.csv". A reboot with logging left on resumes the same
 *     file instead of starting another — see startNewLogFile()/resumeLogFile().
 *
 * Graph mode notes:
 *   - Menu item is disabled until logging is on (no file to read otherwise).
 *   - The x-axis scale adapts to how long logging has actually been running,
 *     capped at 24h: probeGraphSpan() reads two small fixed-size chunks (the
 *     first data row and the most recent one, GRAPH_PROBE_SIZE bytes each)
 *     to get the file's total age cheaply, without scanning anything in
 *     between. If that's under 24h, the whole file is the window and the
 *     first row's own timestamp is the floor; past 24h, the floor becomes
 *     "most recent row minus 24h" instead — either way, no separate pass is
 *     spent measuring before the real (single) bucketed read.
 *   - That single read still walks backward in fixed chunks (see
 *     loadGraphData()/GRAPH_CHUNK_SIZE) averaging rows into GRAPH_MAX_POINTS
 *     buckets spaced across the window, stopping the moment it passes the
 *     floor — so a log that's grown over many days costs the same read time
 *     as a young one, never the whole file. A bucket with no rows in it is
 *     a real gap (NaN), not interpolated or zero-filled.
 *   - If neither probe finds a usable timestamp (e.g. network was never on
 *     anywhere near this file's lifetime), falls back to just the most
 *     recent GRAPH_MAX_POINTS raw rows, no time-based scaling.
 *   - Auto-refreshes on the same cadence as the logging interval while the
 *     graph is on screen. Enter still forces an immediate refresh without
 *     waiting for the next tick.
 */

#include <M5Cardputer.h>
#include <M5Unified.h>
#include <bsec2.h>  // must be included before M5UnitUnifiedENV.h so it enables BSEC2/IAQ mode
#include <M5UnitUnified.h>
#include <M5UnitUnifiedENV.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include "esp32-hal-bt.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <vector>

// Defined here, right after the includes, so it's visible at the point where
// the Arduino IDE inserts its auto-generated function prototypes.
enum AppState { STATE_DASHBOARD, STATE_MENU, STATE_NETWORK, STATE_LOGGING, STATE_GRAPH };
enum NetSubState { NET_STATUS, NET_PICKER, NET_PASSWORD };
enum NetConnState { NET_IDLE, NET_CONNECTING, NET_CONNECTED, NET_FAILED };
enum LogSubState { LOG_STATUS, LOG_FILENAME, LOG_INTERVAL };
// Which text field (if any) is currently consuming typed keystrokes. Shared
// by WiFi password entry and the log filename field so there's one keyboard
// consumer instead of two near-duplicate ones.
enum TextEntryField { TEXT_NONE, TEXT_WIFI_PASSWORD, TEXT_LOG_FILENAME };
// Auto-detected at boot by I2C address (see detectSensorKind()) so the same
// firmware works with whichever of the three ENV units is plugged in. Only
// ENV Pro (BME688) has gas/IAQ/CO2eq — ENV III/IV are temp/humidity/pressure
// only, so those fields just stay NaN and drawDashboard() adapts the layout.
enum SensorKind { SENSOR_NONE, SENSOR_PRO, SENSOR_ENV3, SENSOR_ENV4 };

// ---- palette (RGB565) ----------------------------------------------------
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static const uint16_t COL_BG_TOP    = rgb565(0x1a, 0x0b, 0x3d);
static const uint16_t COL_BG_BOTTOM = rgb565(0x2b, 0x0a, 0x3d);
static const uint16_t COL_CYAN      = rgb565(0x39, 0xff, 0xe0);
static const uint16_t COL_PINK      = rgb565(0xff, 0x3e, 0xa5);
static const uint16_t COL_PURPLE    = rgb565(0xb0, 0x6b, 0xff);
static const uint16_t COL_TRACK     = rgb565(0x3a, 0x2a, 0x55);
static const uint16_t COL_ROW_BG    = rgb565(0x24, 0x14, 0x40);
static const uint16_t COL_ROW_SEL   = rgb565(0x3d, 0x1f, 0x5c);
static const uint16_t COL_TEXT_DIM  = rgb565(0x8f, 0xd8, 0xcf);
static const uint16_t COL_WHITE     = rgb565(0xff, 0xff, 0xff);

// ---- app state -------------------------------------------------------------
AppState appState = STATE_DASHBOARD;

const char* kMenuItems[] = {"NETWORK MODE", "LOGGING MODE", "GRAPH MODE"};
const int kMenuCount = 3;
int menuSelected = 0;

bool screenOn = true;
bool useFahrenheit = false;
bool forceRedraw = true;

struct KeyEdges {
    bool backtick = false, enter = false, m = false, space = false, up = false, down = false, fnDel = false,
         left = false, right = false;
} prevKeys;

// ---- sensor state ------------------------------------------------------
SensorKind sensorKind = SENSOR_NONE;

m5::unit::UnitUnified Units;
m5::unit::UnitENVPro envUnitPro;
m5::unit::UnitENV3 envUnit3;
m5::unit::UnitENV4 envUnit4;
bool sensorReady = false;

struct SensorData {
    bool valid = false;
    float tempC = NAN, humidity = NAN, pressurePa = NAN, gasOhm = NAN, iaq = NAN, co2 = NAN;
} sensor;

// ---- display -------------------------------------------------------------
M5Canvas canvas(&M5Cardputer.Display);

constexpr int SCREEN_W = 240, SCREEN_H = 135;
constexpr int HEADER_H = 14;
constexpr unsigned long FRAME_INTERVAL_MS = 200;
unsigned long lastDrawMs = 0;

// ---- power management -----------------------------------------------------
constexpr uint32_t CPU_MHZ_ACTIVE = 240;
constexpr uint32_t CPU_MHZ_IDLE = 80;  // APB clock (I2C/BSEC timing) is unaffected by this; also the
                                        // Espressif-recommended floor for keeping WiFi alive
constexpr unsigned long SENSOR_POLL_IDLE_INTERVAL_MS = 10000;
unsigned long lastSensorPollMs = 0;

// ---- SD config -------------------------------------------------------------
constexpr int SD_SPI_SCK_PIN = 40, SD_SPI_MISO_PIN = 39, SD_SPI_MOSI_PIN = 14, SD_SPI_CS_PIN = 12;
static const char* CONFIG_PATH = "/cpenvpro.cfg";

struct SavedNetwork {
    String ssid;
    String password;
};
std::vector<SavedNetwork> savedNetworks;

// ---- network state ---------------------------------------------------------
static const char* MDNS_HOSTNAME = "cpenv";
constexpr unsigned long NET_CONNECT_TIMEOUT_MS = 15000;

NetSubState netSubState = NET_STATUS;
NetConnState netConnState = NET_IDLE;
bool networkEnabled = false;
String connectedSsid;
String pendingSsid, pendingPassword, passwordBuffer;
unsigned long netConnectStartMs = 0;
int netStatusSelected = 0;
int scanCount = 0;
int scanSelected = 0;

WebServer server(80);
bool serverStarted = false;

TextEntryField textEntryField = TEXT_NONE;

// ---- logging state ----------------------------------------------------
struct IntervalOption {
    int seconds;
    const char* label;
};
// 10 SEC is the floor: it matches SENSOR_POLL_IDLE_INTERVAL_MS, so logging
// never has to speed up sensor polling while the screen is asleep — the
// power throttle stays in full effect regardless of logging interval.
const IntervalOption kIntervalOptions[] = {{10, "10 SEC"}, {30, "30 SEC"}, {60, "1 MIN"}, {300, "5 MIN"}};
const int kIntervalCount = 4;

LogSubState logSubState = LOG_STATUS;
bool loggingEnabled = false;
String logFilename;       // base name the user set, e.g. "/readings.csv"
String logFilenameBuffer;  // in-progress text entry
String logFilenameError;
int logIntervalSec = 10;
int logStatusSelected = 0;
int logIntervalSelected = 0;  // index into kIntervalOptions, defaults to 10 SEC
unsigned long lastLogWriteMs = 0;
unsigned long logRowsWritten = 0;
bool logLastWriteOk = true;

// The actual file being written: "<base>_<logSequence><ext>". A new sequence
// number (and so a new file) is minted each time logging goes OFF->ON or the
// filename is (re)saved — never just from a reboot, where the existing
// sequence is reused so an already-running session keeps its file. See
// startNewLogFile() / resumeLogFile().
int logSequence = 0;
String logActiveFilename;

// ---- graph state -----------------------------------------------------
// csvCol matches logWriteTick()'s fixed header order: timestamp,temperature_c,
// humidity_pct,pressure_hpa,gas_ohm,iaq,co2eq_ppm,battery_pct.
// Index 0 must stay TEMP — drawGraphView()'s space-key C/F toggle is gated
// on graphMetricIndex == 0, not on the label text.
struct GraphMetricInfo {
    const char* label;
    const char* unit;
    uint16_t color;
    int csvCol;
    float scale;  // applied to the raw CSV value before display/plotting
};
const GraphMetricInfo kGraphMetrics[] = {
    {"TEMP", "C", COL_CYAN, 1, 1.0f},      {"HUM", "%", COL_PINK, 2, 1.0f},
    {"PRESS", "hPa", COL_PURPLE, 3, 1.0f}, {"VOC", "k", COL_CYAN, 4, 0.001f},
    {"IAQ", "", COL_PURPLE, 5, 1.0f},      {"CO2eq", "ppm", COL_PINK, 6, 1.0f},
};
const int kGraphMetricCountFull = 6;
const int kGraphMetricCountBasic = 3;  // TEMP/HUM/PRESS only, for ENV III/IV

constexpr int GRAPH_MAX_POINTS = 180;
constexpr unsigned long GRAPH_WINDOW_SEC = 24UL * 3600UL;
constexpr int GRAPH_CHUNK_SIZE = 512;  // backward-read chunk size, see loadGraphData()
constexpr int GRAPH_PROBE_SIZE = 768;  // fixed-size first/last-row read, see probeGraphSpan()

int graphMetricIndex = 0;
float graphBuffer[GRAPH_MAX_POINTS];
int graphPointCount = 0;
bool graphHasFile = false;
bool graphTimeAware = false;      // true if a real "now" was available while loading
double graphSpanSec = 0;          // actual time covered by the plotted points; only meaningful when graphTimeAware
unsigned long lastGraphRefreshMs = 0;

// ---- formatting helpers ---------------------------------------------------
String fmt1(float v) {
    if (isnan(v)) return "--";
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", v);
    return String(buf);
}
String fmt0(float v) {
    if (isnan(v)) return "--";
    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f", v);
    return String(buf);
}

// Graph mode's x-axis label — the actual time the plotted points span, e.g.
// "5 MIN" for a log that's only been running a few minutes, up to "24.0H".
String formatSpan(double spanSec) {
    if (spanSec < 60) return String((int)spanSec) + "S";
    if (spanSec < 3600) return fmt1((float)(spanSec / 60.0)) + "MIN";
    return fmt1((float)(spanSec / 3600.0)) + "H";
}

// ---- drawing ---------------------------------------------------------------
void drawBackground() {
    for (int y = 0; y < SCREEN_H; y++) {
        float t = (float)y / (SCREEN_H - 1);
        uint8_t r = (1 - t) * ((COL_BG_TOP >> 11) & 0x1F) + t * ((COL_BG_BOTTOM >> 11) & 0x1F);
        uint8_t g = (1 - t) * ((COL_BG_TOP >> 5) & 0x3F) + t * ((COL_BG_BOTTOM >> 5) & 0x3F);
        uint8_t b = (1 - t) * (COL_BG_TOP & 0x1F) + t * (COL_BG_BOTTOM & 0x1F);
        canvas.drawFastHLine(0, y, SCREEN_W, (r << 11) | (g << 5) | b);
    }
}

void drawBatteryIcon(int x, int y, int pct) {
    canvas.drawRect(x, y, 14, 7, COL_CYAN);
    canvas.fillRect(x + 14, y + 2, 2, 3, COL_CYAN);
    int fillW = pct < 0 ? 0 : constrain(map(pct, 0, 100, 0, 12), 0, 12);
    uint16_t fillColor = (pct >= 0 && pct <= 15) ? COL_PINK : COL_CYAN;
    if (fillW > 0) canvas.fillRect(x + 1, y + 1, fillW, 5, fillColor);
}

void drawHeader() {
    canvas.drawFastHLine(0, HEADER_H - 1, SCREEN_W, COL_CYAN);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_CYAN, COL_BG_TOP);
    canvas.setTextSize(1);
    canvas.drawString("ENV-PRO", 4, 3);

    int batteryPct = M5.Power.getBatteryLevel();
    char pctBuf[6];
    if (batteryPct < 0) {
        snprintf(pctBuf, sizeof(pctBuf), "--%%");
    } else {
        snprintf(pctBuf, sizeof(pctBuf), "%d%%", batteryPct);
    }
    canvas.setTextDatum(top_right);
    canvas.drawString(pctBuf, SCREEN_W - 20, 3);
    drawBatteryIcon(SCREEN_W - 18, 3, batteryPct);
    canvas.setTextDatum(top_left);
}

// r0/r1 = inner/outer radius, angles in degrees, 0 = 12 o'clock, clockwise.
void drawRingGauge(int cx, int cy, int r1, int r0, float pct, uint16_t color, const char* value,
                    const char* label) {
    pct = constrain(pct, 0.0f, 1.0f);
    canvas.fillArc(cx, cy, r0, r1, 0, 360, COL_TRACK);
    if (pct > 0.002f) canvas.fillArc(cx, cy, r0, r1, 0, 360.0f * pct, color);

    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_WHITE, TFT_TRANSPARENT);
    canvas.setTextSize(1);
    canvas.drawString(value, cx, cy - 5);
    canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
    canvas.drawString(label, cx, cy + 9);
    canvas.setTextDatum(top_left);
}

void drawMiniStat(int cx, int y, uint16_t color, const char* label, const char* value, float barPct) {
    canvas.setTextDatum(top_center);
    canvas.setTextColor(color, TFT_TRANSPARENT);
    canvas.setTextSize(1);
    canvas.drawString(label, cx, y);
    canvas.setTextColor(COL_WHITE, TFT_TRANSPARENT);
    canvas.drawString(value, cx, y + 9);

    int barW = 60, barH = 3;
    int bx = cx - barW / 2, by = y + 20;
    canvas.fillRect(bx, by, barW, barH, COL_TRACK);
    int fillW = constrain((int)(barW * constrain(barPct, 0.0f, 1.0f)), 0, barW);
    if (fillW > 0) canvas.fillRect(bx, by, fillW, barH, color);
    canvas.setTextDatum(top_left);
}

void drawDashboard() {
    drawBackground();
    drawHeader();

    float tempC = sensor.tempC;
    float tempDisplay = useFahrenheit ? (tempC * 9.0f / 5.0f + 32.0f) : tempC;
    String tempStr = fmt1(tempDisplay) + (useFahrenheit ? "F" : "C");
    float tempPct = isnan(tempC) ? 0 : (tempC - 15.0f) / (35.0f - 15.0f);

    float humPct = isnan(sensor.humidity) ? 0 : sensor.humidity / 100.0f;
    String humStr = fmt0(sensor.humidity) + "%";

    float pressHpa = sensor.pressurePa / 100.0f;
    float pressPct = isnan(pressHpa) ? 0 : (pressHpa - 950.0f) / (1050.0f - 950.0f);
    String pressStr = fmt0(pressHpa);

    // ENV III/IV have no gas sensor, so there's nothing to show in the mini-stat
    // row at all — rather than leave it blank, the gauges grow to fill the space.
    bool hasGas = (sensorKind == SENSOR_PRO);
    int gaugeY, r1, r0;
    if (hasGas) {
        gaugeY = HEADER_H + 43;
        r1 = 35;
        r0 = 27;
    } else {
        // Gauge centers are 80px apart (x=40,120,200), so diameter (2*r1) has
        // to stay under 80 or adjacent circles overlap — r1=48 didn't (96px
        // diameter). 38 leaves a 4px gap while still being noticeably bigger
        // than the Pro layout's cramped 35.
        gaugeY = HEADER_H + 60;
        r1 = 38;
        r0 = 29;
    }
    drawRingGauge(40, gaugeY, r1, r0, tempPct, COL_CYAN, tempStr.c_str(), "TEMP");
    drawRingGauge(120, gaugeY, r1, r0, humPct, COL_PINK, humStr.c_str(), "HUM");
    drawRingGauge(200, gaugeY, r1, r0, pressPct, COL_PURPLE, pressStr.c_str(), "PRESS");

    if (!hasGas) return;

    canvas.drawFastHLine(0, HEADER_H + 86, SCREEN_W, COL_TRACK);

    float gasKohm = sensor.gasOhm / 1000.0f;
    String vocStr = isnan(gasKohm) ? "--" : (fmt0(gasKohm) + "k");
    float vocPct = isnan(gasKohm) ? 0 : constrain(gasKohm / 200.0f, 0.0f, 1.0f);

    String co2Str = isnan(sensor.co2) ? "--" : fmt0(sensor.co2);
    float co2Pct = isnan(sensor.co2) ? 0 : (sensor.co2 - 400.0f) / (2000.0f - 400.0f);

    String iaqStr = isnan(sensor.iaq) ? "--" : fmt0(sensor.iaq);
    float iaqPct = isnan(sensor.iaq) ? 0 : sensor.iaq / 300.0f;

    int statY = HEADER_H + 91;
    drawMiniStat(40, statY, COL_CYAN, "VOC", vocStr.c_str(), vocPct);
    drawMiniStat(120, statY, COL_PINK, "CO2eq", co2Str.c_str(), co2Pct);
    drawMiniStat(200, statY, COL_PURPLE, "IAQ", iaqStr.c_str(), iaqPct);
}

void drawScrollableMenu() {
    drawBackground();
    drawHeader();

    const int rowH = 26;
    const int visibleRows = (SCREEN_H - HEADER_H) / rowH;
    int start = constrain(menuSelected - visibleRows / 2, 0, max(0, kMenuCount - visibleRows));

    for (int i = 0; i < visibleRows && (start + i) < kMenuCount; i++) {
        int idx = start + i;
        int y = HEADER_H + i * rowH;
        bool sel = idx == menuSelected;
        // Graph mode needs a log file to read, so it's disabled (not just
        // unselected-looking) until logging is turned on.
        bool disabled = (idx == 2 && !loggingEnabled);
        canvas.fillRect(4, y + 2, SCREEN_W - 8, rowH - 4, sel && !disabled ? COL_ROW_SEL : COL_ROW_BG);
        canvas.drawRect(4, y + 2, SCREEN_W - 8, rowH - 4, sel && !disabled ? COL_CYAN : COL_TRACK);
        canvas.setTextDatum(middle_left);
        if (disabled) {
            canvas.setTextColor(COL_TRACK, COL_ROW_BG);
        } else {
            canvas.setTextColor(sel ? COL_WHITE : COL_TEXT_DIM, sel ? COL_ROW_SEL : COL_ROW_BG);
        }
        canvas.drawString(kMenuItems[idx], 12, y + rowH / 2);
    }
    canvas.setTextDatum(top_left);

    if (start > 0) canvas.fillTriangle(SCREEN_W - 14, HEADER_H + 6, SCREEN_W - 8, HEADER_H + 6, SCREEN_W - 11,
                                        HEADER_H + 1, COL_CYAN);
    if (start + visibleRows < kMenuCount)
        canvas.fillTriangle(SCREEN_W - 14, SCREEN_H - 6, SCREEN_W - 8, SCREEN_H - 6, SCREEN_W - 11, SCREEN_H - 1,
                             COL_CYAN);
}

void drawPlaceholder(const char* title) {
    drawBackground();
    drawHeader();
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_CYAN, TFT_TRANSPARENT);
    canvas.setTextSize(1);
    canvas.drawString(title, SCREEN_W / 2, 65);
    canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
    canvas.drawString("COMING SOON - PRESS M", SCREEN_W / 2, 82);
    canvas.setTextDatum(top_left);
}

struct NetRow {
    String label;
    bool selectable;
};

void drawNetworkStatus() {
    drawBackground();
    drawHeader();

    std::vector<NetRow> rows;
    rows.push_back({networkEnabled ? "NETWORK: ON" : "NETWORK: OFF", true});
    if (networkEnabled) {
        rows.push_back({"SELECT NETWORK", true});
        String statusLabel;
        switch (netConnState) {
            case NET_IDLE: statusLabel = "NOT CONNECTED"; break;
            case NET_CONNECTING: statusLabel = "CONNECTING..."; break;
            case NET_CONNECTED: statusLabel = "CONNECTED: " + connectedSsid; break;
            case NET_FAILED: statusLabel = "CONNECT FAILED"; break;
        }
        rows.push_back({statusLabel, false});
        if (netConnState == NET_CONNECTED) {
            rows.push_back({"IP " + WiFi.localIP().toString(), false});
            rows.push_back({String(MDNS_HOSTNAME) + ".local", false});
        }
    }

    int y = HEADER_H + 2;
    int selIdx = -1;
    for (size_t i = 0; i < rows.size(); i++) {
        if (rows[i].selectable) {
            selIdx++;
            bool sel = selIdx == netStatusSelected;
            canvas.fillRect(4, y, SCREEN_W - 8, 22, sel ? COL_ROW_SEL : COL_ROW_BG);
            canvas.drawRect(4, y, SCREEN_W - 8, 22, sel ? COL_CYAN : COL_TRACK);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(sel ? COL_WHITE : COL_TEXT_DIM, sel ? COL_ROW_SEL : COL_ROW_BG);
            canvas.drawString(rows[i].label, 12, y + 11);
            y += 25;
        } else {
            canvas.setTextDatum(top_left);
            canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
            canvas.drawString(rows[i].label, 12, y + 2);
            y += 15;
        }
    }
    canvas.setTextDatum(top_left);
}

void drawNetworkPicker() {
    drawBackground();
    drawHeader();

    if (scanCount <= 0) {
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
        canvas.drawString("NO NETWORKS FOUND", SCREEN_W / 2, SCREEN_H / 2);
        canvas.setTextDatum(top_left);
        return;
    }

    const int rowH = 22;
    const int visibleRows = (SCREEN_H - HEADER_H) / rowH;
    int start = constrain(scanSelected - visibleRows / 2, 0, max(0, scanCount - visibleRows));

    for (int i = 0; i < visibleRows && (start + i) < scanCount; i++) {
        int idx = start + i;
        int y = HEADER_H + i * rowH;
        bool sel = idx == scanSelected;
        canvas.fillRect(4, y + 1, SCREEN_W - 8, rowH - 2, sel ? COL_ROW_SEL : COL_ROW_BG);
        canvas.drawRect(4, y + 1, SCREEN_W - 8, rowH - 2, sel ? COL_CYAN : COL_TRACK);
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(sel ? COL_WHITE : COL_TEXT_DIM, sel ? COL_ROW_SEL : COL_ROW_BG);
        String label = WiFi.SSID(idx);
        if (WiFi.encryptionType(idx) != WIFI_AUTH_OPEN) label += " *";
        canvas.drawString(label, 12, y + rowH / 2);
    }
    canvas.setTextDatum(top_left);
}

void drawNetworkPassword() {
    drawBackground();
    drawHeader();

    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_CYAN, TFT_TRANSPARENT);
    canvas.drawString(pendingSsid, 8, 22);

    // Shown in clear text on purpose — this keyboard is small and easy to
    // mistype on, and masking makes that harder to catch, not more secure.
    canvas.fillRect(4, 42, SCREEN_W - 8, 22, COL_ROW_BG);
    canvas.drawRect(4, 42, SCREEN_W - 8, 22, COL_CYAN);
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(COL_WHITE, COL_ROW_BG);
    canvas.drawString(passwordBuffer, 10, 53);

    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
    canvas.drawString("ENTER CONNECT", 8, 74);
    canvas.drawString("FN+DEL CANCEL", 8, 86);

    if (netConnState == NET_CONNECTING) {
        canvas.setTextColor(COL_PINK, TFT_TRANSPARENT);
        canvas.drawString("CONNECTING...", 8, 104);
    } else if (netConnState == NET_FAILED) {
        canvas.setTextColor(COL_PINK, TFT_TRANSPARENT);
        canvas.drawString("FAILED - TRY AGAIN", 8, 104);
    }
}

void drawNetworkMode() {
    switch (netSubState) {
        case NET_STATUS: drawNetworkStatus(); break;
        case NET_PICKER: drawNetworkPicker(); break;
        case NET_PASSWORD: drawNetworkPassword(); break;
    }
}

bool loggingActive() {
    return loggingEnabled && logActiveFilename.length() > 0;
}

void drawLoggingStatus() {
    drawBackground();
    drawHeader();

    std::vector<NetRow> rows;
    rows.push_back({loggingEnabled ? "LOGGING: ON" : "LOGGING: OFF", true});
    if (loggingEnabled) {
        rows.push_back({logFilename.length() ? ("FILE: " + logFilename) : "FILE: (not set)", true});
        rows.push_back({String("INTERVAL: ") + kIntervalOptions[logIntervalSelected].label, true});
        if (!loggingActive()) {
            rows.push_back({"SET A FILENAME TO START", false});
        } else {
            rows.push_back({"LOGGING - " + String(logRowsWritten) + " ROWS", false});
            if (!logLastWriteOk) rows.push_back({"LAST WRITE FAILED", false});
        }
    }

    int y = HEADER_H + 2;
    int selIdx = -1;
    for (size_t i = 0; i < rows.size(); i++) {
        if (rows[i].selectable) {
            selIdx++;
            bool sel = selIdx == logStatusSelected;
            canvas.fillRect(4, y, SCREEN_W - 8, 22, sel ? COL_ROW_SEL : COL_ROW_BG);
            canvas.drawRect(4, y, SCREEN_W - 8, 22, sel ? COL_CYAN : COL_TRACK);
            canvas.setTextDatum(middle_left);
            canvas.setTextColor(sel ? COL_WHITE : COL_TEXT_DIM, sel ? COL_ROW_SEL : COL_ROW_BG);
            canvas.drawString(rows[i].label, 12, y + 11);
            y += 25;
        } else {
            canvas.setTextDatum(top_left);
            canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
            canvas.drawString(rows[i].label, 12, y + 2);
            y += 15;
        }
    }
    canvas.setTextDatum(top_left);
}

void drawLoggingFilename() {
    drawBackground();
    drawHeader();

    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_CYAN, TFT_TRANSPARENT);
    canvas.drawString("FILENAME", 8, 22);

    canvas.fillRect(4, 42, SCREEN_W - 8, 22, COL_ROW_BG);
    canvas.drawRect(4, 42, SCREEN_W - 8, 22, COL_CYAN);
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(COL_WHITE, COL_ROW_BG);
    canvas.drawString(logFilenameBuffer, 10, 53);

    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
    canvas.drawString("ENTER SAVE", 8, 74);
    canvas.drawString("FN+DEL CANCEL", 8, 86);

    if (logFilenameError.length()) {
        canvas.setTextColor(COL_PINK, TFT_TRANSPARENT);
        canvas.drawString(logFilenameError, 8, 104);
    }
}

void drawLoggingInterval() {
    drawBackground();
    drawHeader();

    const int rowH = 22;
    for (int i = 0; i < kIntervalCount; i++) {
        int y = HEADER_H + i * rowH;
        bool sel = i == logIntervalSelected;
        canvas.fillRect(4, y + 1, SCREEN_W - 8, rowH - 2, sel ? COL_ROW_SEL : COL_ROW_BG);
        canvas.drawRect(4, y + 1, SCREEN_W - 8, rowH - 2, sel ? COL_CYAN : COL_TRACK);
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(sel ? COL_WHITE : COL_TEXT_DIM, sel ? COL_ROW_SEL : COL_ROW_BG);
        canvas.drawString(kIntervalOptions[i].label, 12, y + rowH / 2);
    }
    canvas.setTextDatum(top_left);
}

void drawLoggingMode() {
    switch (logSubState) {
        case LOG_STATUS: drawLoggingStatus(); break;
        case LOG_FILENAME: drawLoggingFilename(); break;
        case LOG_INTERVAL: drawLoggingInterval(); break;
    }
}

void render() {
    switch (appState) {
        case STATE_DASHBOARD: drawDashboard(); break;
        case STATE_MENU: drawScrollableMenu(); break;
        case STATE_NETWORK: drawNetworkMode(); break;
        case STATE_LOGGING: drawLoggingMode(); break;
        case STATE_GRAPH: drawGraphView(); break;
    }
    canvas.pushSprite(0, 0);
}

// ---- sensor detection --------------------------------------------------
bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Probes the Grove port directly, before any unit is registered, so the
// right concrete driver type can be picked at compile-known call sites (the
// three units have different accessor APIs — no shared "read temperature"
// interface to dispatch through). Uses the same board-resolved Port A pins
// wiring::addI2C() will use right after, so this doesn't fight it for the bus.
// ENV Pro (BME688) is 0x77 with nothing else on the bus; ENV III/IV both put
// their SHT3x/4x chip at 0x44, so the pressure chip's address (QMP6988 0x70
// vs BMP280 0x76) is what actually distinguishes them.
SensorKind detectSensorKind() {
    int sda = M5.getPin(m5::pin_name_t::port_a_sda);
    int scl = M5.getPin(m5::pin_name_t::port_a_scl);
    Wire.begin(sda, scl);
    delay(10);
    if (i2cPing(0x77)) return SENSOR_PRO;
    if (i2cPing(0x70)) return SENSOR_ENV3;
    if (i2cPing(0x76)) return SENSOR_ENV4;
    return SENSOR_NONE;
}

// ---- sensor polling ---------------------------------------------------------
void updateSensor() {
    if (!sensorReady) return;
    Units.update();
    switch (sensorKind) {
        case SENSOR_PRO:
            if (envUnitPro.updated()) {
                auto d = envUnitPro.latest();
                sensor.tempC = d.temperature();
                sensor.humidity = d.humidity();
                sensor.pressurePa = d.pressure();
                sensor.gasOhm = d.gas();
                sensor.iaq = d.iaq();
                sensor.co2 = d.co2();
                sensor.valid = true;
            }
            break;
        case SENSOR_ENV3:
            if (envUnit3.sht30.updated()) {
                sensor.tempC = envUnit3.sht30.temperature();
                sensor.humidity = envUnit3.sht30.humidity();
                sensor.valid = true;
            }
            if (envUnit3.qmp6988.updated()) {
                sensor.pressurePa = envUnit3.qmp6988.pressure();
                sensor.valid = true;
            }
            break;
        case SENSOR_ENV4:
            if (envUnit4.sht40.updated()) {
                sensor.tempC = envUnit4.sht40.temperature();
                sensor.humidity = envUnit4.sht40.humidity();
                sensor.valid = true;
            }
            if (envUnit4.bmp280.updated()) {
                sensor.pressurePa = envUnit4.bmp280.pressure();
                sensor.valid = true;
            }
            break;
        case SENSOR_NONE: break;
    }
}

// ---- SD config ---------------------------------------------------------
bool sdBegin() {
    SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
    return SD.begin(SD_SPI_CS_PIN, SPI, 25000000);
}

// Plain "key=value" lines, one network per ssid/pass pair in the order
// they were saved. Not JSON: this sketch fully controls both the writer and
// the reader, so a trivial line format is exactly as robust as JSON here
// and doesn't need a parser library (see the file header for why that
// matters on this device).
void loadConfig() {
    if (!sdBegin()) {
        SD.end();
        return;
    }
    if (SD.exists(CONFIG_PATH)) {
        File f = SD.open(CONFIG_PATH);
        if (f) {
            savedNetworks.clear();
            String ssid;
            bool haveSsid = false;
            while (f.available()) {
                String line = f.readStringUntil('\n');
                line.trim();
                int eq = line.indexOf('=');
                if (eq < 0) continue;
                String key = line.substring(0, eq);
                String val = line.substring(eq + 1);
                if (key == "enabled") {
                    networkEnabled = (val == "1");
                } else if (key == "ssid") {
                    ssid = val;
                    haveSsid = true;
                } else if (key == "pass" && haveSsid) {
                    savedNetworks.push_back({ssid, val});
                    haveSsid = false;
                } else if (key == "log_enabled") {
                    loggingEnabled = (val == "1");
                } else if (key == "log_file") {
                    logFilename = val;
                } else if (key == "log_interval") {
                    logIntervalSec = val.toInt();
                    for (int i = 0; i < kIntervalCount; i++) {
                        if (kIntervalOptions[i].seconds == logIntervalSec) logIntervalSelected = i;
                    }
                } else if (key == "log_seq") {
                    logSequence = val.toInt();
                }
            }
            f.close();
        }
    }
    SD.end();
}

void saveConfig() {
    if (!sdBegin()) {
        SD.end();
        return;
    }
    File f = SD.open(CONFIG_PATH, FILE_WRITE);
    if (f) {
        f.printf("enabled=%d\n", networkEnabled ? 1 : 0);
        for (auto& n : savedNetworks) {
            f.printf("ssid=%s\n", n.ssid.c_str());
            f.printf("pass=%s\n", n.password.c_str());
        }
        f.printf("log_enabled=%d\n", loggingEnabled ? 1 : 0);
        f.printf("log_file=%s\n", logFilename.c_str());
        f.printf("log_interval=%d\n", logIntervalSec);
        f.printf("log_seq=%d\n", logSequence);
        f.close();
    }
    SD.end();
}

bool findSavedPassword(const String& ssid, String& outPass) {
    for (auto& n : savedNetworks) {
        if (n.ssid == ssid) {
            outPass = n.password;
            return true;
        }
    }
    return false;
}

void upsertSavedNetwork(const String& ssid, const String& pass) {
    for (auto& n : savedNetworks) {
        if (n.ssid == ssid) {
            n.password = pass;
            saveConfig();
            return;
        }
    }
    savedNetworks.push_back({ssid, pass});
    saveConfig();
}

// ---- networking --------------------------------------------------------
String jsonNum(float v) {
    return isnan(v) ? "null" : String(v, 2);
}

void handleJsonRequest() {
    float tempF = isnan(sensor.tempC) ? NAN : (sensor.tempC * 9.0f / 5.0f + 32.0f);
    String json = "{";
    json += "\"device\":\"cpenv\",";
    json += "\"temperature_c\":" + jsonNum(sensor.tempC) + ",";
    json += "\"temperature_f\":" + jsonNum(tempF) + ",";
    json += "\"humidity_pct\":" + jsonNum(sensor.humidity) + ",";
    json += "\"pressure_hpa\":" + jsonNum(sensor.pressurePa / 100.0f) + ",";
    json += "\"gas_ohm\":" + jsonNum(sensor.gasOhm) + ",";
    json += "\"iaq\":" + jsonNum(sensor.iaq) + ",";
    json += "\"co2eq_ppm\":" + jsonNum(sensor.co2) + ",";
    json += "\"battery_pct\":" + String(M5.Power.getBatteryLevel()) + ",";
    json += "\"uptime_ms\":" + String(millis());
    json += "}";
    server.send(200, "application/json", json);
}

void startServices() {
    if (serverStarted) return;
    MDNS.begin(MDNS_HOSTNAME);
    MDNS.addService("http", "tcp", 80);
    server.on("/", HTTP_GET, handleJsonRequest);
    server.begin();
    serverStarted = true;
}

void stopServices() {
    if (!serverStarted) return;
    server.stop();
    MDNS.end();
    serverStarted = false;
}

void startConnect(const String& ssid, const String& pass) {
    pendingSsid = ssid;
    pendingPassword = pass;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    netConnState = NET_CONNECTING;
    netConnectStartMs = millis();
}

void tryAutoConnect() {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
        String pass;
        if (findSavedPassword(WiFi.SSID(i), pass)) {
            startConnect(WiFi.SSID(i), pass);
            return;
        }
    }
}

void updateNetworkConnection() {
    if (!networkEnabled || netConnState != NET_CONNECTING) return;
    if (WiFi.status() == WL_CONNECTED) {
        netConnState = NET_CONNECTED;
        connectedSsid = pendingSsid;
        upsertSavedNetwork(pendingSsid, pendingPassword);
        startServices();
        configTime(0, 0, "pool.ntp.org");  // UTC, no DST offset — see getTimestampString()
    } else if (millis() - netConnectStartMs > NET_CONNECT_TIMEOUT_MS) {
        netConnState = NET_FAILED;
    }
}

// Radios stay off everywhere except when the network toggle is on, independent
// of which screen is currently showing — the JSON endpoint has to keep serving
// even while the user is looking at the dashboard or the screen is asleep.
void setNetworkRadiosEnabled(bool enabled) {
    if (enabled) {
        WiFi.mode(WIFI_STA);
    } else {
        stopServices();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        netConnState = NET_IDLE;
        connectedSsid = "";
    }
}

void beginScan() {
    drawBackground();
    drawHeader();
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COL_CYAN, TFT_TRANSPARENT);
    canvas.drawString("SCANNING...", SCREEN_W / 2, SCREEN_H / 2);
    canvas.setTextDatum(top_left);
    canvas.pushSprite(0, 0);

    int n = WiFi.scanNetworks();
    scanCount = n > 0 ? n : 0;
    scanSelected = 0;
    netSubState = NET_PICKER;
}

void networkStatusMove(int dir) {
    int maxSel = networkEnabled ? 1 : 0;
    netStatusSelected = constrain(netStatusSelected + dir, 0, maxSel);
}

void networkStatusActivate() {
    if (netStatusSelected == 0) {
        networkEnabled = !networkEnabled;
        setNetworkRadiosEnabled(networkEnabled);
        saveConfig();
        if (networkEnabled) tryAutoConnect();
        netStatusSelected = 0;
    } else if (netStatusSelected == 1 && networkEnabled) {
        beginScan();
    }
    forceRedraw = true;
}

void networkPickerMove(int dir) {
    if (scanCount <= 0) return;
    scanSelected = (scanSelected + dir + scanCount) % scanCount;
}

void networkPickerActivate() {
    if (scanCount <= 0) return;
    String ssid = WiFi.SSID(scanSelected);
    bool isOpen = WiFi.encryptionType(scanSelected) == WIFI_AUTH_OPEN;
    String pass;
    if (isOpen) {
        startConnect(ssid, "");
        netSubState = NET_STATUS;
    } else if (findSavedPassword(ssid, pass)) {
        startConnect(ssid, pass);
        netSubState = NET_STATUS;
    } else {
        pendingSsid = ssid;
        passwordBuffer = "";
        netSubState = NET_PASSWORD;
        textEntryField = TEXT_WIFI_PASSWORD;
    }
    forceRedraw = true;
}

void passwordSubmit() {
    if (passwordBuffer.length() == 0) return;
    startConnect(pendingSsid, passwordBuffer);
    netSubState = NET_STATUS;
    textEntryField = TEXT_NONE;
    forceRedraw = true;
}

void passwordCancel() {
    passwordBuffer = "";
    netSubState = NET_PICKER;
    textEntryField = TEXT_NONE;
    forceRedraw = true;
}

void logStatusMove(int dir) {
    int maxSel = loggingEnabled ? 2 : 0;
    logStatusSelected = constrain(logStatusSelected + dir, 0, maxSel);
}

// "/readings.csv" + sequence 3 -> "/readings_3.csv"
String computeActiveFilename() {
    if (logFilename.length() == 0) return "";
    int dot = logFilename.lastIndexOf('.');
    String stem = (dot >= 0) ? logFilename.substring(0, dot) : logFilename;
    String ext = (dot >= 0) ? logFilename.substring(dot) : ".csv";
    return stem + "_" + String(logSequence) + ext;
}

// A genuinely new file: bumps the sequence, points writes at it, and resets
// the row counter. Called on an OFF->ON toggle or a filename (re)save —
// never on boot, where resumeLogFile() keeps whatever was already running.
void startNewLogFile() {
    if (logFilename.length() == 0) return;
    logSequence++;
    logActiveFilename = computeActiveFilename();
    logRowsWritten = 0;
    logLastWriteOk = true;
    saveConfig();
}

// Reuses the already-persisted sequence number so a reboot with logging left
// on keeps appending to the same file instead of starting a fresh one.
void resumeLogFile() {
    logActiveFilename = computeActiveFilename();
}

void logStatusActivate() {
    if (logStatusSelected == 0) {
        bool turningOn = !loggingEnabled;
        loggingEnabled = !loggingEnabled;
        if (turningOn && logFilename.length() > 0) startNewLogFile();
        saveConfig();
        logStatusSelected = 0;
    } else if (logStatusSelected == 1 && loggingEnabled) {
        logFilenameBuffer = logFilename.length() ? logFilename.substring(1) : "";  // drop leading '/' for editing
        logFilenameError = "";
        textEntryField = TEXT_LOG_FILENAME;
        logSubState = LOG_FILENAME;
    } else if (logStatusSelected == 2 && loggingEnabled) {
        logSubState = LOG_INTERVAL;
    }
    forceRedraw = true;
}

void logIntervalMove(int dir) {
    logIntervalSelected = (logIntervalSelected + dir + kIntervalCount) % kIntervalCount;
}

void logIntervalActivate() {
    logIntervalSec = kIntervalOptions[logIntervalSelected].seconds;
    saveConfig();
    logSubState = LOG_STATUS;
    forceRedraw = true;
}

// Only alnum/underscore/hyphen/dot survive; FAT filesystems choke on much
// else and the physical keyboard will happily type any of it.
void filenameSubmit() {
    String clean;
    for (size_t i = 0; i < logFilenameBuffer.length(); i++) {
        char c = logFilenameBuffer[i];
        if (isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.') clean += c;
    }
    if (clean.length() == 0) {
        logFilenameError = "INVALID NAME";
        return;
    }
    if (clean.indexOf('.') < 0) clean += ".csv";
    logFilename = "/" + clean;
    logFilenameError = "";
    textEntryField = TEXT_NONE;
    logSubState = LOG_STATUS;
    if (loggingEnabled) {
        startNewLogFile();  // also saveConfig()s
    } else {
        saveConfig();
    }
    forceRedraw = true;
}

void filenameCancel() {
    logFilenameBuffer = "";
    logFilenameError = "";
    textEntryField = TEXT_NONE;
    logSubState = LOG_STATUS;
    forceRedraw = true;
}

// Consumes typed characters for whichever text field is active (WiFi
// password or log filename). Only active while that field is on screen, so
// it never steals keystrokes from anywhere else.
void updateTextEntry() {
    if (textEntryField == TEXT_NONE) return;
    auto& kb = M5Cardputer.Keyboard;
    if (!kb.isChange() || !kb.isPressed()) return;
    auto st = kb.keysState();
    String& buf = (textEntryField == TEXT_WIFI_PASSWORD) ? passwordBuffer : logFilenameBuffer;
    for (char c : st.word) {
        if (buf.length() < 63) buf += c;
    }
    if (st.del && buf.length() > 0) {
        buf.remove(buf.length() - 1);
    }
}

// ---- logging -------------------------------------------------------------
String csvNum(float v) {
    return isnan(v) ? "" : String(v, 2);
}

// Blank until the network is on and NTP has actually synced (checked via a
// plausible-epoch sanity floor, ~Nov 2023) — keeps the timestamp column
// present-but-empty rather than shifting the CSV's column count around.
String getTimestampString() {
    if (!networkEnabled) return "";
    time_t now = time(nullptr);
    if (now < 1700000000) return "";
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buf);
}

void logWriteTick() {
    if (!sdBegin()) {
        SD.end();
        logLastWriteOk = false;
        return;
    }
    File f = SD.open(logActiveFilename, FILE_APPEND);
    if (!f) {
        SD.end();
        logLastWriteOk = false;
        return;
    }
    if (f.size() == 0) {
        f.println("timestamp,temperature_c,humidity_pct,pressure_hpa,gas_ohm,iaq,co2eq_ppm,battery_pct");
    }
    String row = getTimestampString() + ",";
    row += csvNum(sensor.tempC) + ",";
    row += csvNum(sensor.humidity) + ",";
    row += csvNum(sensor.pressurePa / 100.0f) + ",";
    row += csvNum(sensor.gasOhm) + ",";
    row += csvNum(sensor.iaq) + ",";
    row += csvNum(sensor.co2) + ",";
    row += String(M5.Power.getBatteryLevel());
    f.println(row);
    f.close();
    SD.end();
    logRowsWritten++;
    logLastWriteOk = true;
}

bool parseTimestamp(const String& s, time_t& outEpoch) {
    if (s.length() == 0) return false;
    struct tm tmVal = {};
    int y, mo, d, h, mi, se;
    if (sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &se) != 6) return false;
    tmVal.tm_year = y - 1900;
    tmVal.tm_mon = mo - 1;
    tmVal.tm_mday = d;
    tmVal.tm_hour = h;
    tmVal.tm_min = mi;
    tmVal.tm_sec = se;
    tmVal.tm_isdst = 0;
    outEpoch = mktime(&tmVal);
    return outEpoch != (time_t)-1;
}

// Streams the active log file top to bottom (bounded memory: a small ring
// buffer, never the whole file) and keeps at most the last GRAPH_MAX_POINTS
// readings for the selected metric. If a real clock is available (NTP has
// synced at some point this session — see getTimestampString()), only rows
// timestamped within the last 24h count, and rows with a blank timestamp are
// skipped rather than guessed at. Without a clock at all, falls back to
// simply the most recent GRAPH_MAX_POINTS rows regardless of age.
// Walks a log file backward in fixed chunks from the end, newest row first,
// stopping at GRAPH_WINDOW_SEC or the start of the file — so a multi-day log
// costs only as many reads as it takes to cover the window, never the whole
// file. `carry` holds the fragment of a line split across a chunk boundary;
// `combined` is that fragment glued onto the newly-read (earlier-in-file)
// chunk, scanned back-to-front for '\n's. Shared by scanGraphWindow() (pass
// 1, measures the window) and loadGraphData()'s bucketed pass 2 and no-clock
// fallback — kept as three separate copies rather than one parameterized
// helper so each stays simple enough to read as straight-line code.
// Cheap span estimate: reads one small fixed-size chunk from the start of
// the file (the first data row, right after the header) and one from the
// end (the most recent row) instead of scanning anything in between — see
// loadGraphData() for how this replaces a full backward scan. Returns false
// if neither probe turns up a parseable timestamp (e.g. network was never
// on anywhere near either end of the file), so the caller can fall back to
// the no-clock path.
bool probeGraphSpan(File& f, time_t& firstEpoch, time_t& lastEpoch) {
    char buf[GRAPH_PROBE_SIZE];

    f.seek(0);
    int n = f.read((uint8_t*)buf, GRAPH_PROBE_SIZE);
    if (n <= 0) return false;
    String head;
    head.reserve(n);
    for (int i = 0; i < n; i++) head += buf[i];
    bool foundFirst = false;
    int lineStart = 0;
    for (int i = 0; i <= (int)head.length() && !foundFirst; i++) {
        if (i < (int)head.length() && head[i] != '\n') continue;
        String line = head.substring(lineStart, i);
        lineStart = i + 1;
        line.trim();
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        String tsField = comma >= 0 ? line.substring(0, comma) : line;
        if (tsField == "timestamp") continue;
        if (parseTimestamp(tsField, firstEpoch)) foundFirst = true;
    }
    if (!foundFirst) return false;

    long fsize = f.size();
    long tailStart = (fsize > GRAPH_PROBE_SIZE) ? (fsize - GRAPH_PROBE_SIZE) : 0;
    f.seek(tailStart);
    n = f.read((uint8_t*)buf, (int)(fsize - tailStart));
    if (n <= 0) return false;
    String tail;
    tail.reserve(n);
    for (int i = 0; i < n; i++) tail += buf[i];
    bool foundLast = false;
    int end = tail.length();
    for (int i = (int)tail.length() - 1; i >= 0 && !foundLast; i--) {
        if (tail[i] != '\n') continue;
        String line = tail.substring(i + 1, end);
        end = i;
        line.trim();
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        String tsField = comma >= 0 ? line.substring(0, comma) : line;
        if (tsField == "timestamp") continue;
        if (parseTimestamp(tsField, lastEpoch)) foundLast = true;
    }
    return foundLast;
}

void loadGraphData() {
    graphPointCount = 0;
    graphHasFile = false;
    graphTimeAware = false;
    graphSpanSec = 0;
    lastGraphRefreshMs = millis();  // covers manual and auto-refresh alike — see loop()

    if (logActiveFilename.length() == 0) return;
    if (!sdBegin()) {
        SD.end();
        return;
    }
    if (!SD.exists(logActiveFilename)) {
        SD.end();
        return;
    }

    int col = kGraphMetrics[graphMetricIndex].csvCol;
    float scale = kGraphMetrics[graphMetricIndex].scale;

    File probeFile = SD.open(logActiveFilename);
    if (!probeFile) {
        SD.end();
        return;
    }
    graphHasFile = true;
    time_t firstEpoch = 0, lastEpoch = 0;
    bool haveSpan = probeGraphSpan(probeFile, firstEpoch, lastEpoch);
    probeFile.close();

    if (haveSpan) {
        graphTimeAware = true;
        // If the file's total history already fits in 24h, the first line IS
        // the window's oldest edge (your original idea, unmodified). Only once
        // the log outgrows 24h does the window floor become "last - 24h"
        // instead of the literal first line.
        double totalSpanSec = (double)(lastEpoch - firstEpoch);
        time_t oldestEpoch =
            (totalSpanSec > (double)GRAPH_WINDOW_SEC) ? (lastEpoch - (time_t)GRAPH_WINDOW_SEC) : firstEpoch;
        double spanSec = (double)(lastEpoch - oldestEpoch);
        double bucketDurationSec = (spanSec > 0) ? (spanSec / GRAPH_MAX_POINTS) : 1.0;

        // Always GRAPH_MAX_POINTS buckets spaced across the actual span, not
        // the row count — a bucket with no rows in it is a real gap (NaN,
        // skipped when drawing), which is what makes the x-axis genuinely
        // adapt to how long logging has been running instead of always 24h.
        float sumBuckets[GRAPH_MAX_POINTS];
        int nBuckets[GRAPH_MAX_POINTS];
        for (int i = 0; i < GRAPH_MAX_POINTS; i++) {
            sumBuckets[i] = 0;
            nBuckets[i] = 0;
        }

        File f = SD.open(logActiveFilename);
        if (!f) {
            SD.end();
            return;
        }

        long pos = f.size();
        String carry;
        char buf[GRAPH_CHUNK_SIZE];
        bool stop = false;

        while (pos > 0 && !stop) {
            int chunkLen = (int)((pos < GRAPH_CHUNK_SIZE) ? pos : GRAPH_CHUNK_SIZE);
            long chunkStart = pos - chunkLen;
            if (!f.seek(chunkStart)) break;
            int readLen = f.read((uint8_t*)buf, chunkLen);
            if (readLen <= 0) break;
            pos = chunkStart;

            String combined;
            combined.reserve(readLen + carry.length() + 1);
            for (int i = 0; i < readLen; i++) combined += buf[i];
            combined += carry;

            int end = combined.length();
            for (int i = (int)combined.length() - 1; i >= 0 && !stop; i--) {
                if (combined[i] != '\n') continue;
                String line = combined.substring(i + 1, end);
                end = i;
                line.trim();
                if (line.length() == 0) continue;

                int idx = 0, start = 0;
                String tsField, valField;
                bool haveTs = false, haveVal = false;
                for (int j = 0; j <= (int)line.length(); j++) {
                    if (j == (int)line.length() || line[j] == ',') {
                        if (idx == 0) {
                            tsField = line.substring(start, j);
                            haveTs = true;
                        } else if (idx == col) {
                            valField = line.substring(start, j);
                            haveVal = true;
                            break;
                        }
                        idx++;
                        start = j + 1;
                    }
                }
                if (!haveVal || valField.length() == 0 || tsField == "timestamp") continue;

                time_t rowEpoch = 0;
                bool rowHasTs = haveTs && parseTimestamp(tsField, rowEpoch);
                if (!rowHasTs) continue;  // can't place it in a bucket — skip, keep walking backward
                // Rows only get older as we walk further back, so the first one
                // before the window floor means everything before it is too.
                if (rowEpoch < oldestEpoch) {
                    stop = true;
                    break;
                }

                long bIdx = (long)((rowEpoch - oldestEpoch) / bucketDurationSec);
                if (bIdx < 0) bIdx = 0;
                if (bIdx >= GRAPH_MAX_POINTS) bIdx = GRAPH_MAX_POINTS - 1;
                sumBuckets[bIdx] += valField.toFloat() * scale;
                nBuckets[bIdx]++;
            }
            carry = combined.substring(0, end);
        }
        f.close();
        SD.end();

        graphPointCount = GRAPH_MAX_POINTS;
        for (int i = 0; i < GRAPH_MAX_POINTS; i++) {
            graphBuffer[i] = (nBuckets[i] > 0) ? (sumBuckets[i] / nBuckets[i]) : NAN;
        }
        graphSpanSec = spanSec;
        return;
    }

    // No usable timestamp near either end of the file (e.g. network was never
    // on anywhere close to this file's lifetime) — fall back to the most recent
    // GRAPH_MAX_POINTS raw rows, no time-based scaling or downsampling.
    File f = SD.open(logActiveFilename);
    if (!f) {
        SD.end();
        return;
    }
    graphHasFile = true;

    float collected[GRAPH_MAX_POINTS];
    int collectedCount = 0;
    long pos = f.size();
    String carry;
    char buf[GRAPH_CHUNK_SIZE];
    bool stop = false;

    while (pos > 0 && !stop) {
        int chunkLen = (int)((pos < GRAPH_CHUNK_SIZE) ? pos : GRAPH_CHUNK_SIZE);
        long chunkStart = pos - chunkLen;
        if (!f.seek(chunkStart)) break;
        int readLen = f.read((uint8_t*)buf, chunkLen);
        if (readLen <= 0) break;
        pos = chunkStart;

        String combined;
        combined.reserve(readLen + carry.length() + 1);
        for (int i = 0; i < readLen; i++) combined += buf[i];
        combined += carry;

        int end = combined.length();
        for (int i = (int)combined.length() - 1; i >= 0 && !stop; i--) {
            if (combined[i] != '\n') continue;
            String line = combined.substring(i + 1, end);
            end = i;
            line.trim();
            if (line.length() == 0) continue;

            int idx = 0, start = 0;
            String tsField, valField;
            bool haveVal = false;
            for (int j = 0; j <= (int)line.length(); j++) {
                if (j == (int)line.length() || line[j] == ',') {
                    if (idx == 0) {
                        tsField = line.substring(start, j);
                    } else if (idx == col) {
                        valField = line.substring(start, j);
                        haveVal = true;
                        break;
                    }
                    idx++;
                    start = j + 1;
                }
            }
            if (!haveVal || valField.length() == 0 || tsField == "timestamp") continue;

            if (collectedCount >= GRAPH_MAX_POINTS) {
                stop = true;
                break;
            }
            collected[collectedCount++] = valField.toFloat() * scale;
        }
        carry = combined.substring(0, end);
    }
    f.close();
    SD.end();

    graphPointCount = collectedCount;
    for (int i = 0; i < collectedCount; i++) {
        graphBuffer[i] = collected[collectedCount - 1 - i];  // newest-first -> chronological
    }
}

void drawGraphView() {
    drawBackground();
    drawHeader();

    const GraphMetricInfo& m = kGraphMetrics[graphMetricIndex];
    // Same space-key toggle as the dashboard, and the same shared preference
    // — only meaningful (and only wired up) on the TEMP graph.
    bool showFahrenheit = (graphMetricIndex == 0) && useFahrenheit;
    const char* unitLabel = showFahrenheit ? "F" : m.unit;
    auto dispVal = [&](int i) { return showFahrenheit ? (graphBuffer[i] * 9.0f / 5.0f + 32.0f) : graphBuffer[i]; };

    if (!graphHasFile) {
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
        canvas.drawString("NO LOG FILE", SCREEN_W / 2, SCREEN_H / 2);
        canvas.setTextDatum(top_left);
        return;
    }
    if (graphPointCount == 0) {
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
        canvas.drawString(String("NO ") + m.label + " DATA", SCREEN_W / 2, SCREEN_H / 2);
        canvas.setTextDatum(top_left);
        return;
    }

    // Downsampled buckets with no source rows are NaN (a real gap, not a
    // guess) — excluded from min/max and skipped when drawing line segments.
    float minV = INFINITY, maxV = -INFINITY;
    bool haveRange = false;
    for (int i = 0; i < graphPointCount; i++) {
        if (isnan(graphBuffer[i])) continue;
        float v = dispVal(i);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        haveRange = true;
    }
    if (!haveRange) {
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
        canvas.drawString(String("NO ") + m.label + " DATA", SCREEN_W / 2, SCREEN_H / 2);
        canvas.setTextDatum(top_left);
        return;
    }
    if (maxV - minV < 0.001f) {
        minV -= 1.0f;
        maxV += 1.0f;
    }

    const int plotTop = HEADER_H + 2, plotBottom = SCREEN_H - 16;
    const int plotLeft = 30, plotRight = SCREEN_W - 4;
    const int plotH = plotBottom - plotTop, plotW = plotRight - plotLeft;

    canvas.setTextDatum(top_left);
    canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
    canvas.drawString(fmt1(maxV), 2, plotTop);
    canvas.drawString(fmt1(minV), 2, plotBottom - 8);

    // Buckets are spaced by time, not by row density, so real readings are
    // routinely several buckets apart (e.g. a 10s logging interval only
    // fills 1 bucket in 7 once the span is more than ~4 minutes). Connecting
    // only strictly-adjacent buckets meant almost nothing ever had a
    // non-empty neighbor, so no line ever drew despite valid data. Instead,
    // connect each real bucket straight to the *next* real one, however many
    // empty buckets sit between — the standard "skip nulls" convention.
    auto plotX = [&](int idx) {
        return plotLeft + (graphPointCount > 1 ? (int)((long)idx * plotW / (graphPointCount - 1)) : 0);
    };
    auto plotY = [&](float v) { return plotBottom - (int)((v - minV) / (maxV - minV) * plotH); };

    int lastValidIdx = -1;
    for (int i = 0; i < graphPointCount; i++) {
        if (isnan(graphBuffer[i])) continue;
        if (lastValidIdx >= 0) {
            canvas.drawLine(plotX(lastValidIdx), plotY(dispVal(lastValidIdx)), plotX(i), plotY(dispVal(i)), m.color);
        }
        lastValidIdx = i;
    }

    canvas.setTextDatum(bottom_left);
    canvas.setTextColor(m.color, TFT_TRANSPARENT);
    String latest = lastValidIdx >= 0 ? fmt1(dispVal(lastValidIdx)) : "--";
    canvas.drawString(String(m.label) + " " + latest + unitLabel, 4, SCREEN_H - 2);

    canvas.setTextDatum(bottom_right);
    canvas.setTextColor(COL_TEXT_DIM, TFT_TRANSPARENT);
    canvas.drawString(graphTimeAware ? formatSpan(graphSpanSec) : (String(graphPointCount) + " PTS"),
                       SCREEN_W - 4, SCREEN_H - 2);
    canvas.setTextDatum(top_left);
}

// Real display sleep (panel SLPIN, not just backlight=0) plus a CPU downclock.
// The I2C/BSEC timing rides on the fixed APB clock, so this doesn't affect
// sensor readings, only how fast the CPU core itself runs between polls.
void setLowPowerMode(bool lowPower) {
    if (lowPower) {
        M5Cardputer.Display.sleep();
        setCpuFrequencyMhz(CPU_MHZ_IDLE);
    } else {
        setCpuFrequencyMhz(CPU_MHZ_ACTIVE);
        M5Cardputer.Display.wakeup();
    }
}

// All appState changes should go through here so leaving network/logging
// mode always resets their sub-screens, no matter which key triggered the exit.
void setAppState(AppState newState) {
    if (appState == STATE_NETWORK && newState != STATE_NETWORK) {
        netSubState = NET_STATUS;
        passwordBuffer = "";
    }
    if (appState == STATE_LOGGING && newState != STATE_LOGGING) {
        logSubState = LOG_STATUS;
        logFilenameBuffer = "";
        logFilenameError = "";
    }
    textEntryField = TEXT_NONE;
    appState = newState;
}

// ---- input handling ---------------------------------------------------------
void selectMenuItem() {
    switch (menuSelected) {
        case 0: setAppState(STATE_NETWORK); break;
        case 1: setAppState(STATE_LOGGING); break;
        case 2:
            if (!loggingEnabled) break;  // disabled — see drawScrollableMenu()
            graphMetricIndex = 0;        // always opens on TEMP
            setAppState(STATE_GRAPH);
            loadGraphData();
            break;
    }
}

// ENV III/IV have no gas sensor, so their graphs are limited to the first
// three metrics (temp/hum/press) — same rule drawDashboard() already uses.
void graphMove(int dir) {
    int count = (sensorKind == SENSOR_PRO) ? kGraphMetricCountFull : kGraphMetricCountBasic;
    graphMetricIndex = (graphMetricIndex + dir + count) % count;
    loadGraphData();
    forceRedraw = true;
}

void handleInput() {
    auto& kb = M5Cardputer.Keyboard;
    bool typingText = (textEntryField != TEXT_NONE);

    bool curBacktick = kb.isKeyPressed('`');
    bool curEnter = kb.keysState().enter;
    bool curM = kb.isKeyPressed('m');
    bool curSpace = kb.keysState().space;
    bool curUp = kb.isKeyPressed(';');
    bool curDown = kb.isKeyPressed('.');
    bool curLeft = kb.isKeyPressed(',');
    bool curRight = kb.isKeyPressed('/');
    // No dedicated Esc key on this keyboard reader; Fn+Backspace stands in as the cancel gesture.
    bool curCancel = kb.keysState().fn && kb.keysState().del;

    // While typing a password or filename, all of the hotkeys below are just characters.
    if (!typingText) {
        if (curBacktick && !prevKeys.backtick && screenOn) {
            if (appState == STATE_DASHBOARD) {
                screenOn = false;
                setLowPowerMode(true);
            } else if (appState == STATE_NETWORK && netSubState == NET_PICKER) {
                netSubState = NET_STATUS;
            } else if (appState == STATE_LOGGING && logSubState == LOG_INTERVAL) {
                logSubState = LOG_STATUS;
            } else {
                setAppState(STATE_DASHBOARD);
            }
        }

        if (curEnter && !prevKeys.enter) {
            if (!screenOn) {
                screenOn = true;
                setLowPowerMode(false);
                forceRedraw = true;
            } else if (appState == STATE_MENU) {
                selectMenuItem();
            } else if (appState == STATE_NETWORK) {
                if (netSubState == NET_STATUS) networkStatusActivate();
                else if (netSubState == NET_PICKER) networkPickerActivate();
            } else if (appState == STATE_LOGGING) {
                if (logSubState == LOG_STATUS) logStatusActivate();
                else if (logSubState == LOG_INTERVAL) logIntervalActivate();
            } else if (appState == STATE_GRAPH) {
                loadGraphData();  // manual refresh — logging keeps appending in the background
            }
        }

        if (screenOn) {
            if (curM && !prevKeys.m) {
                setAppState(appState == STATE_DASHBOARD ? STATE_MENU : STATE_DASHBOARD);
            }
            if (curSpace && !prevKeys.space && appState == STATE_DASHBOARD) {
                useFahrenheit = !useFahrenheit;
            }
            if (curSpace && !prevKeys.space && appState == STATE_GRAPH && graphMetricIndex == 0) {
                useFahrenheit = !useFahrenheit;  // same shared preference as the dashboard
                forceRedraw = true;
            }
            if (appState == STATE_MENU) {
                if (curUp && !prevKeys.up) menuSelected = (menuSelected - 1 + kMenuCount) % kMenuCount;
                if (curDown && !prevKeys.down) menuSelected = (menuSelected + 1) % kMenuCount;
            } else if (appState == STATE_NETWORK && netSubState == NET_STATUS) {
                if (curUp && !prevKeys.up) networkStatusMove(-1);
                if (curDown && !prevKeys.down) networkStatusMove(1);
            } else if (appState == STATE_NETWORK && netSubState == NET_PICKER) {
                if (curUp && !prevKeys.up) networkPickerMove(-1);
                if (curDown && !prevKeys.down) networkPickerMove(1);
            } else if (appState == STATE_LOGGING && logSubState == LOG_STATUS) {
                if (curUp && !prevKeys.up) logStatusMove(-1);
                if (curDown && !prevKeys.down) logStatusMove(1);
            } else if (appState == STATE_LOGGING && logSubState == LOG_INTERVAL) {
                if (curUp && !prevKeys.up) logIntervalMove(-1);
                if (curDown && !prevKeys.down) logIntervalMove(1);
            } else if (appState == STATE_GRAPH) {
                if (curRight && !prevKeys.right) graphMove(1);
                if (curLeft && !prevKeys.left) graphMove(-1);
            }
        }
    } else {
        // Text entry: Enter submits, Fn+Backspace cancels. Nothing else is a hotkey.
        if (curEnter && !prevKeys.enter) {
            if (textEntryField == TEXT_WIFI_PASSWORD) passwordSubmit();
            else if (textEntryField == TEXT_LOG_FILENAME) filenameSubmit();
        }
        if (curCancel && !prevKeys.fnDel) {
            if (textEntryField == TEXT_WIFI_PASSWORD) passwordCancel();
            else if (textEntryField == TEXT_LOG_FILENAME) filenameCancel();
        }
    }

    prevKeys = {curBacktick, curEnter, curM, curSpace, curUp, curDown, curCancel, curLeft, curRight};
}

// ---- setup / loop ---------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(100);

    // Bluetooth isn't used at all. WiFi stays in WIFI_OFF (its default, unset
    // state) until the user turns the network toggle on.
    btStop();

    canvas.setColorDepth(16);
    canvas.createSprite(SCREEN_W, SCREEN_H);
    canvas.setTextWrap(false, false);

    sensorKind = detectSensorKind();
    switch (sensorKind) {
        case SENSOR_PRO: {
            // M5Unit-ENV's default subscription list omits CO2-equivalent/breath-VOC,
            // so add them explicitly or those readings stay NaN forever.
            auto envCfg = envUnitPro.config();
            envCfg.subscribe_bits |= (1U << BSEC_OUTPUT_CO2_EQUIVALENT) | (1U << BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
            envUnitPro.config(envCfg);
            sensorReady = m5::unit::wiring::addI2C(Units, envUnitPro) && Units.begin();
            break;
        }
        case SENSOR_ENV3: sensorReady = m5::unit::wiring::addI2C(Units, envUnit3) && Units.begin(); break;
        case SENSOR_ENV4: sensorReady = m5::unit::wiring::addI2C(Units, envUnit4) && Units.begin(); break;
        case SENSOR_NONE: sensorReady = false; break;
    }
    if (!sensorReady) {
        canvas.fillScreen(TFT_BLACK);
        canvas.setTextColor(COL_PINK);
        canvas.setTextDatum(middle_center);
        canvas.drawString("NO ENV SENSOR FOUND", SCREEN_W / 2, SCREEN_H / 2);
        canvas.pushSprite(0, 0);
    }

    loadConfig();
    if (networkEnabled) {
        setNetworkRadiosEnabled(true);
        tryAutoConnect();
    }
    // Resume, don't start fresh: a reboot with logging already on should keep
    // appending to the same file, not mint a new one every power cycle.
    if (loggingEnabled) resumeLogFile();
}

void loop() {
    M5Cardputer.update();
    unsigned long now = millis();

    // Full-rate sensor polling while the screen is visible; throttled to once
    // per SENSOR_POLL_IDLE_INTERVAL_MS while asleep to cut CPU wake-ups. The
    // logging interval floor (10s, see kIntervalOptions) matches this exactly,
    // so logging never needs to override the throttle.
    if (screenOn || now - lastSensorPollMs >= SENSOR_POLL_IDLE_INTERVAL_MS) {
        lastSensorPollMs = now;
        updateSensor();
    }

    // Networking and logging run in the background regardless of screen
    // state or which screen is currently showing.
    updateNetworkConnection();
    if (serverStarted) server.handleClient();
    if (loggingActive() && (lastLogWriteMs == 0 || now - lastLogWriteMs >= (unsigned long)logIntervalSec * 1000UL)) {
        lastLogWriteMs = now;
        logWriteTick();
    }

    // Graph mode only, and only while it's actually on screen — no point
    // re-scanning the log file on the same cadence logging writes it when
    // nobody's looking. Same interval as logging, so the graph never shows
    // data staler than what's actually been written.
    if (appState == STATE_GRAPH && screenOn &&
        now - lastGraphRefreshMs >= (unsigned long)logIntervalSec * 1000UL) {
        loadGraphData();
        forceRedraw = true;
    }

    handleInput();
    updateTextEntry();

    if (screenOn) {
        if (forceRedraw || now - lastDrawMs >= FRAME_INTERVAL_MS) {
            lastDrawMs = now;
            forceRedraw = false;
            render();
        }
    } else {
        // Small idle delay so we're not busy-looping at full CPU while asleep.
        // Kept short (well under the TCA8418's ~10-event FIFO window) so a
        // quick Enter tap to wake the screen is never missed.
        delay(20);
    }
}
