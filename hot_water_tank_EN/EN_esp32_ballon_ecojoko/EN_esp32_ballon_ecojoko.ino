// 
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* ============================================================
   ESP32 – Hot water tank controlled by Ecojoko + Shelly 1PM Gen3
   ============================================================ */

// Program version: increment by hand on every change (format YYYY.MM.DD-n, comparable as text).
// FW_BUILD changes by itself on every compilation: if the time shown in System has not moved, the flash did not go through.
static const char FW_VERSION[] = "2026.11.18-1";
static const char FW_BUILD[]   = __DATE__ " " __TIME__;
constexpr uint32_t CONTROL_PERIOD_MS = 5000; // Control loop frequency (5 s)
constexpr uint32_t MIN_OFF_TO_ON_MS = 60000; // Minimum 1 min between an OFF and the next ON
// Shelly (local): separate timeouts. 2.5 s was too short: a single TCP retransmission (RTO ~1.5-3 s)
// after a lost packet was enough to produce a -11 while the Shelly normally answers in ~70 ms.
constexpr uint16_t SHELLY_CONNECT_TIMEOUT_MS = 1500; // TCP connection setup (failure = -1, no immediate retry)
constexpr uint16_t SHELLY_READ_TIMEOUT_MS    = 5000; // wait for the response (failure = -11)
constexpr uint8_t  SHELLY_READ_MAX_TRIES     = 2;    // poll read: 1 try + 1 retry, on -11 (immediate) and -1 (with delay, see SHELLY_RETRY_REFUSED_DELAY_MS)
constexpr uint16_t SHELLY_RETRY_REFUSED_DELAY_MS = 250; // -1 = Shelly overloaded / closing the port: give it time to free its slot before retrying
constexpr uint32_t ECO_HTTP_TIMEOUT_MS = 8000; // Ecojoko cloud (remote HTTPS, slower than the local Shelly)
constexpr uint32_t ECO_TLS_HANDSHAKE_TIMEOUT_S = 8; // Core default = 120 s: a TLS handshake blocks loop() until it completes

/* ---- Root TLS certificates (remote server verification) ----
   While these constants are empty, the code automatically falls back to setInsecure() (current behaviour,
   Ecojoko login/password sent without verifying the server) and the Serial log
   reminds you. To enable verification, paste here the ROOT certificate (PEM format, the LAST
   certificate printed by the command below, not the first) of each service:
     openssl s_client -connect service.ecojoko.com:443   -showcerts < /dev/null
     openssl s_client -connect api.open-meteo.com:443    -showcerts < /dev/null
   A root certificate changes very rarely (years), unlike the server's own certificate:
   no recurring maintenance once it is in place. */
static const char ECOJOKO_ROOT_CA[] = "";   // TODO: paste the PEM root certificate of service.ecojoko.com
static const char OPENMETEO_ROOT_CA[] = ""; // TODO: paste the PEM root certificate of api.open-meteo.com
constexpr int      ECO_BEGIN_FAILED = -100; // Internal code: https.begin() failed (distinct from the HTTPClient codes, -1 to -11)
constexpr uint32_t SHELLY_CONFIRM_DELAY_MS = 2000;
constexpr uint8_t  SHELLY_MAX_ATTEMPTS = 3;
constexpr uint32_t MAX_TICK_DT_SEC = 30; // Do not charge a long Wi-Fi/HTTP stall as heating time.
constexpr uint32_t WIFI_CHECK_PERIOD_MS = 10000;
constexpr uint32_t WIFI_RECONNECT_AFTER_MS = 20000;
constexpr uint8_t  NETWORK_BACKOFF_AFTER_FAILURES = 3;
constexpr uint32_t NETWORK_BACKOFF_MAX_MS = 60000;
constexpr uint32_t ERROR_CLEAR_DELAY_MS = 30000;
constexpr uint16_t MONTHLY_HISTORY_MAX = 60; // 5 years of monthly recap (~230 bytes/entry, ~14 KB in total); the yearly recap is derived from it
constexpr uint8_t  YEARLY_HISTORY_MAX = 5;   // number of years shown in the yearly recap (sum of the months of /monthly.json)
// LittleFS memory guard: below this free-space threshold, history is purged (oldest first).
// Never touched: /config.json, /index.html, /eco_gateway.json, /boot_log.txt, /sos_log.json.
constexpr size_t   STORAGE_MIN_FREE_BYTES = 32768;
constexpr uint32_t STORAGE_PURGE_RETRY_MS = 600000UL; // if the purge was not enough, do not retry for 10 min
constexpr uint8_t  PURGE_KEEP_DAYS   = 7;
constexpr uint8_t  PURGE_KEEP_MONTHS_1 = 24;
constexpr uint8_t  PURGE_KEEP_MONTHS_2 = 3;
// SOS log: one entry per buzzer alarm (reason + system state), automatically purged after SOS_KEEP_S.
constexpr char     SOS_PATH[]        = "/sos_log.json";
constexpr uint32_t SOS_KEEP_S        = 72UL * 3600UL;   // an alarm that ended more than 72 h ago is deleted
constexpr uint8_t  SOS_MAX_ENTRIES   = 30;              // safety cap (~350 bytes per entry)
constexpr uint32_t SOS_PRUNE_EVERY_MS = 3600000UL;      // hourly cleanup (in addition to the one done on every write / read)
constexpr uint32_t SOS_REOPEN_S      = 600;             // same cause back < 10 min later: reopen the entry (counter n) instead of creating a new one
constexpr char FRANCE_TZ[] = "CET-1CEST,M3.5.0,M10.5.0/3";

/* ---------- Forecast (full mode): internal parameters, see section 6a ---------- */
constexpr uint8_t  ANT_WX_MAX          = 72;   // hours of weather kept in RAM (yesterday + today + tomorrow)
constexpr uint8_t  ANT_REC_MAX         = 35;   // cycles (days) kept in /ant.json
constexpr uint16_t ANT_HR_N            = 192;  // forecast/actual hours kept (8 days, ring buffer) in /ant_hr.json
constexpr uint32_t ANT_HR_MIN_SEC      = 1800; // an hour is only kept if >= 30 min were measured
// Shading by hour of the day (section "Shading"): learned from the hourly forecast / actual, applied only if cfg.antShade.
constexpr uint8_t  ANT_SHADE_N         = 20;   // samples kept per hour of the day (sliding window of about 20 usable days)
constexpr uint8_t  ANT_SHADE_MIN_N     = 7;    // minimum samples for an hour to get a factor
constexpr uint8_t  ANT_SHADE_MIN_H     = 4;    // minimum number of hours with a factor (the reference level is computed on them)
constexpr float    ANT_SHADE_MIN_P     = 0.35f; // a sample is kept only if the forecast is >= 35 % of heaterW (otherwise too noisy)
constexpr float    ANT_SHADE_F_MIN     = 0.30f; // clamp of the factor
constexpr float    ANT_SHADE_F_MAX     = 1.20f;
constexpr uint32_t ANT_WX_REFRESH_MS   = 3UL * 3600UL * 1000UL; // new forecast every 3 h
constexpr uint32_t ANT_WX_RETRY_MS     = 15UL * 60UL * 1000UL;  // after a failure
constexpr uint32_t ANT_WX_TIMEOUT_MS   = 8000;
constexpr uint32_t ANT_WX_MAX_AGE_S    = 12UL * 3600UL;         // beyond this the forecast is considered stale
constexpr size_t   ANT_WX_MIN_HEAP     = 45000;                 // minimum largest free block for a TLS handshake
constexpr float    ANT_TANK_WH_PER_L   = 58.15f; // 1.163 Wh/(L.K) x 50 K useful temperature difference
constexpr float    ANT_SAFETY          = 1.3f;  // margin on future consumption (burst draw-offs)
constexpr float    ANT_DL_MIN_PCT      = 5.0f;  // floor of the deficit tolerated at the deadline (% of capacity)
constexpr float    ANT_SOLAR_HAIRCUT   = 0.8f;  // only 80 % of the forecast solar is counted
constexpr float    ANT_HYST_WH         = 150.0f;
constexpr float    ANT_BENEFIT         = 0.6f;  // a sunny hour is "useful" only if the energy costs <= 60 % of the HC price
constexpr uint32_t ANT_FORCE_MARGIN_SEC = 1800; // catch-up margin before the deadline
constexpr uint32_t ANT_GAP_MAX_SEC     = 300;   // max measurement gap (relay ON without power reading) before the interval is ignored
constexpr uint32_t ANT_HEAT_MIN_SEC    = 60;    // minimum continuous heating before a cut-off counts as "full"
constexpr uint32_t ANT_FULL_IDLE_SEC   = 120;   // power < thermostatW for 2 min with relay ON => tank is full
constexpr uint32_t ANT_REF_MAX_H       = 36;    // beyond this, the last "full" is too old
constexpr uint32_t ANT_MIN_COVER_SEC   = 14UL * 3600UL; // a cycle is usable only if >= 14 h were measured
constexpr uint8_t  ANT_MIN_NEED_DAYS   = 4;
constexpr uint8_t  ANT_MIN_REG_DAYS    = 8;     // minimum days to fit the temperature slope
constexpr float    ANT_MIN_REG_SPREAD  = 3.0f;  // minimum temperature spread (degC) for the slope to be meaningful
constexpr float    ANT_MAX_SLOPE       = 300.0f; // Wh/degC/day
constexpr float    ANT_PV_RATIO        = 0.7f;   // starting coefficient = kWp x 0.7 (W per W/m2): system efficiency ~0.8 reduced by a margin for the house's own consumption (conservative)
constexpr float    ANT_MIN_K_RAD       = 1500.0f; // minimum cumulated Wh/m2 to estimate the solar coefficient
// Heating log (full mode): one segment = one continuous heating run (stops < HEAT_SEG_GAP_SEC are merged) with ONE reason.
constexpr uint16_t HEAT_SEG_MAX        = 120;     // segments kept (~14 days), 12 bytes each in RAM
constexpr uint32_t HEAT_SEG_GAP_SEC    = 900;     // a shorter pause (thermostat) does not split the segment
constexpr uint32_t HEAT_SAVE_MIN_MS    = 600000UL; // /heat.json is written at most every 10 min (and at every cycle end)
constexpr uint32_t HISTORY_SAVE_PERIOD_MS = 600000UL; // /history.json is written at most every 10 min (was 60 s); forced saves kept: cycle end, checkpoint, import, automatic Ecojoko restart
// "Relay on" intervals (state CONFIRMED by the Shelly), shown under the log timeline: one per switch-on, 8 bytes each in RAM.
constexpr uint16_t RELAY_IV_MAX        = 160;     // ~14 days; stored in /heat.json together with the segments
// Shelly internal temperature (overheating diagnostic): one point every 5 min over 48 h, in RAM (4 bytes per point).
constexpr uint16_t STEMP_N        = 576;
constexpr uint32_t STEMP_STEP_SEC = 300;
constexpr uint8_t  HR_HC = 0, HR_SOLAR = 1, HR_FORCE = 2; // reasons: off-peak hours / profitable sun / plan catch-up

enum ErrorSource : uint8_t { ERROR_NONE, ERROR_SHELLY, ERROR_ECOJOKO, ERROR_STORAGE, ERROR_THERMAL };

/* Types used in function signatures: declared HERE, before any function, because Arduino automatically
   generates prototypes at the top of the file, before the type definitions further down. */
enum WdCause : uint8_t { WD_OK, WD_CRASH_LOOP, WD_RELAY, WD_MEMORY, WD_SHELLY, WD_COMMAND, WD_CLOCK, WD_OVERHEAT };

// One record per cycle (day) for the forecast: see section 6a.
struct AntRec {
  uint32_t cycle  = 0;
  uint32_t sec    = 0;     // duration covered by valid measurements between two "tank full" events
  float    wh     = 0.0f;  // energy injected over that duration (= tank consumption)
  float    rad    = 0.0f;  // cumulated radiation (Wh/m2) over the ticks where the surplus was known
  float    pot    = 0.0f;  // solar surplus usable by the tank (Wh) over those same ticks
  float    skipWh = 0.0f;  // shadow mode: HC energy the plan would have avoided
  int16_t  t10    = INT16_MIN; // average outdoor temperature x10 (INT16_MIN = unknown)
  uint8_t  wday   = 0;     // weekday of the cycle (0 = Sunday)
  float    predWh    = -1.0f; // FORECAST 24 h need at the start of the cycle (Wh; < 0 = unknown): used to judge the forecast
  float    predSolWh = -1.0f; // FORECAST usable solar surplus over 24 h at the start of the cycle (Wh; < 0 = unknown), to compare with pot
};

// An hourly slot to compare FORECAST and ACTUAL solar (ring indexed by epoch hour % ANT_HR_N).
// r = average actual solar surplus of the hour (W, capped at heaterW); p = forecast frozen at the start of the cycle (W). 0xFFFF = unknown.
struct AntHr {
  uint32_t h = 0;       // epoch hour (epoch / 3600); 0 = free slot
  uint16_t r = 0xFFFF;
  uint16_t p = 0xFFFF;
};

// History row (day or month) handled by the import: see section 7b.
struct ImpRow {
  uint32_t key = 0;
  uint32_t u[5] = {0, 0, 0, 0, 0};
  float    f[6] = {0, 0, 0, 0, 0, 0};
  char     mode[12] = "auto";
};

/* Link with the "watchdog" ESP32-C3 (section 9 below, and watchdog_c3/watchdog_c3.ino). */
void appSetup();
void appLoop();
bool wdPinAllowed(int p);
void wdInit();
void wdSetPins(int8_t hb, int8_t err);
void wdService(uint32_t now);
void wdNoteCommand(bool ok);
void wdNoteBoot(esp_reset_reason_t reason, bool rtcValid, bool stalled);
bool wdStartTest();
void wdSetTxPin(int8_t tx);
void wdMuteChanged();
int  wdSnoozeCmd(bool on, String &msg);
void wdFillStatus(JsonDocument &d);

/* ---------- Stall diagnostic (without a serial monitor) ----------
   A task on the other core watches loop(). If loop() has not run for STALL_RESTART_MS,
   the ESP32 restarts by itself and records the cause. The information survives a software restart (RTC memory)
   and is then written to /boot_log.txt, available at http://<esp-ip>/api/diag. */
constexpr uint32_t STALL_RESTART_MS   = 90000; // > worst legitimate case of one controlTask() pass (~45 s: Shelly poll 2 tries ~13 s + Ecojoko ~24 s + one Switch.Set ~6.5 s)
constexpr uint32_t STALL_CHECK_MS     = 5000;

/* ---- Web / control separation (FreeRTOS) ----
   The web server (loop() on the Arduino side, appLoop()) and the control loop (controlTask(), dedicated task)
   now run independently: a slow or blocked web request must never again delay
   relay control or the C3 watchdog heartbeat. ioMutex protects the shared state (cfg, rt,
   in-RAM histories, LittleFS access) between the two: taken by every web route (webRoute(), a single
   entry point) and by every controlTask() pass. Bounded waits (never portMAX_DELAY):
   if either side cannot get the lock in time, it gives up for THIS pass / THIS request
   rather than risk a mutual deadlock; the existing stall detector (diagLoopMs, updated by
   controlTask() even before taking the lock) remains the ultimate safety net.
   Fix (15/11) after a "task watchdog" loop (ESP_RST_TASK_WDT) on a very degraded Wi-Fi (RSSI
   -80 to -93 dBm): controlTask() makes the Ecojoko/Shelly calls (blocking, up to 8 s each) WHILE HOLDING
   ioMutex. Two consequences were observed together in these conditions:
   1) every web route (even /api/status, a simple poll) then waits for the lock for up to IO_MUTEX_WEB_WAIT_TICKS
      before running -> multi-second slow requests on endpoints that do nothing heavy (the "nothing for ~1 min"
      felt during an import was not the import: it was the wait for the lock);
      IO_MUTEX_WEB_WAIT_TICKS reduced to 8 s (~ECO_HTTP_TIMEOUT_MS, the time of a single Ecojoko round trip) so
      that the UI fails fast (503) rather than looking frozen.
   2) controlTask() was pinned to the SAME core (0) as the internal Wi-Fi/lwIP tasks. A blocking Ecojoko call
      on that core, during a very degraded Wi-Fi episode where Wi-Fi already uses a lot of CPU on that core,
      can be enough to starve the idle task of core 0 beyond the hardware watchdog delay ->
      ESP_RST_TASK_WDT reset (independent of the home-made stall detector, which never triggered).
      Before the web/control separation, these same blocking calls ran in the Arduino task (loopTask,
      core 1), away from Wi-Fi: controlTask() is therefore back on core 1 (see setup()) to recover
      that isolation, with stallMonitorTask remaining alone on core 0 (cross-monitoring of one core by the
      other, intentional). */
static SemaphoreHandle_t ioMutex = nullptr;
constexpr TickType_t IO_MUTEX_WEB_WAIT_TICKS  = pdMS_TO_TICKS(8000);  // ~ECO_HTTP_TIMEOUT_MS: a single Ecojoko round trip, not 50 s of silent waiting (see the 15/11 fix above)
constexpr TickType_t IO_MUTEX_CTRL_WAIT_TICKS = pdMS_TO_TICKS(20000); // very generous for running a single web handler (no network inside, just JSON/LittleFS)
constexpr uint32_t   CONTROL_TASK_DELAY_MS    = 200; // frequency of the fast checks (C3 watchdog, Shelly confirmation...); sample()/tick() remain paced by CONTROL_PERIOD_MS
constexpr uint32_t DIAG_RTC_MAGIC     = 0xB411A003; // changes whenever RTC variables are added: the first power-up is seen as "power was cut"
constexpr size_t   DIAG_LOG_MAX_BYTES = 3000;

RTC_NOINIT_ATTR static uint32_t rtcMagic;
RTC_NOINIT_ATTR static uint32_t rtcStallFlag;
RTC_NOINIT_ATTR static uint32_t rtcStallCount;
RTC_NOINIT_ATTR static uint32_t rtcLastUptimeS;
RTC_NOINIT_ATTR static uint32_t rtcLastEpoch;
RTC_NOINIT_ATTR static uint32_t rtcLastFreeHeap;
RTC_NOINIT_ATTR static uint32_t rtcLastMinHeap;
RTC_NOINIT_ATTR static int32_t  rtcLastRssi;
RTC_NOINIT_ATTR static char     rtcLastPhase[16];
RTC_NOINIT_ATTR static uint32_t rtcCrashCount; // successive crashes (panic, watchdog, stall) within the current window
RTC_NOINIT_ATTR static uint32_t rtcCrashSpanS; // time elapsed since the 1st crash of the series (sum of uptimes)

static const char *volatile diagPhase = "startup";
static volatile uint32_t diagLoopMs = 0;

/* ---------- Web instrumentation: WHICH request blocks loop()? ----------
   Each route is registered through webRoute(), which records the URI and the client IP. This information is copied to RTC memory
   by the monitoring task, so it can be read after a restart ("reboot" line of /boot_log.txt, /api/diag).
   Any request slower than WEB_SLOW_MS is also written to /boot_log.txt, even without a crash. */
constexpr uint32_t WEB_SLOW_MS = 2000;  // "slow request" threshold
constexpr uint8_t  WEB_RING    = 12;    // last requests kept in RAM (visible in /api/diag)
RTC_NOINIT_ATTR static char     rtcLastWebUri[24];
RTC_NOINIT_ATTR static char     rtcLastWebIp[16];
RTC_NOINIT_ATTR static uint32_t rtcLastWebAgeS; // 0 = no web processing in progress at the last snapshot, otherwise for how many seconds

static char diagWebUri[24] = "";
static char diagWebIp[16]  = "";
static volatile uint32_t diagWebStartMs = 0; // start of the handleClient() in progress (0 = none in progress)
struct WebReq { uint32_t epoch; uint32_t ms; char uri[24]; char ip[16]; };
static WebReq  webRing[WEB_RING];
static uint8_t webRingHead = 0, webRingCount = 0;
static uint32_t webLastSlowLogMs = 0;

// Marks the code section in progress (restored on exit): shows where loop() was blocked.
struct DiagPhaseScope {
  const char *prev;
  explicit DiagPhaseScope(const char *p) : prev(diagPhase) { diagPhase = p; }
  ~DiagPhaseScope() { diagPhase = prev; }
};

/* --------------- Persisted configuration (LittleFS) ---------------- */
struct Config {
  String wifiSsid, wifiPassword, ecoEmail, ecoPassword, shellyIp;
  String mode = "full"; // auto | manual | monitor | full (full = main mode: default, and restored after every flash)
  String lastFwBuild;   // FW_BUILD of the last boot: if it changes, a new firmware has just been flashed -> full mode
  bool   manualTimer        = false;
  uint16_t manualStart      = 7 * 60, manualEnd     = 10 * 60;
  uint32_t manualEpoch      = 0;
  bool     autoTime         = true;
  uint16_t hcStart          = 23 * 60, hcEnd        = 7 * 60;
  float    heaterW          = 2000.0f;
  float    thermostatW      = 100.0f;
  float    hp               = 0.2516f;
  float    hc               = 0.1696f;
  float    quotaH           = 4.0f;
  uint8_t  smoothingMin     = 3;
  uint16_t ecoGraceSec      = 15;
  int8_t   wdHbPin          = 13; // C3: "heartbeat" pin (-1 = disabled)
  int8_t   wdErrPin         = 14; // C3: "alarm" pin (-1 = disabled)
  uint16_t wdProblemMin     = 60; // minutes of problem tolerated before alerting (Shelly unreachable, failed commands, clock)
  int8_t   wdTxPin          = 26; // C3: "silence" serial link (TX of the ESP32 to RX of the C3; -1 = disabled = the alarm is never silenced)
  bool     muteOn           = false; // quiet window: the C3 does not sound during the window (except for overheating)
  uint16_t muteStart        = 23 * 60, muteEnd = 7 * 60; // start/end of the window (minutes after midnight)
  uint16_t snoozeMin        = 480;   // duration of the remote silence (red banner button), in minutes
  // Forecast (full mode): see section 6a
  String   antMode          = "shadow"; // off = classic full | shadow = computes without acting | on = drives the HC
  bool     antShade         = false;    // apply the learned shading factor to the decision (false = learned and displayed only)
  uint16_t tankLiters       = 200;      // tank volume (used to estimate the capacity if tankKWh = 0)
  float    tankKWh          = 0.0f;     // useful capacity in kWh (energy "cold -> full"); 0 = estimated: litres x 52.3 Wh
  uint8_t  reservePct       = 40;       // hot water reserve to keep before the first useful sun (% of capacity)
  uint16_t deadlineMin      = 18 * 60 + 30; // check before the evening: reserve + evening to cover (the tank does not have to be full)
  float    lat              = 50.40f;   // coordinates for the weather (approximate, editable in /ant)
  float    lon              = 3.15f;
  float    pvKwc            = 0.0f;     // installed peak power (kWp): used to derive a starting solar coefficient; 0 = none (learning only)
  // Network: optional static IP (automatic DHCP fallback) and diagnostic AP after boot
  bool     staticIp         = false;
  String   ip, gw, mask = "255.255.255.0", dns;
  uint16_t bootApSec        = 120;      // duration of the diagnostic AP after a successful boot (0 = disabled)
  // Shelly thermal protection: above shellyTMaxC (0 = threshold disabled) or if the Shelly reports "overtemp", the relay is
  // cut and no switch-on command is sent any more. Resumes when the Shelly is below shellyTResumeC AND after shellyHoldMin minutes.
  float    shellyTMaxC      = 85.0f;
  float    shellyTResumeC   = 65.0f;
  uint16_t shellyHoldMin    = 10;
  // ALARM threshold (watchdog buzzer, no delay) if the Shelly temperature exceeds it (0 = disabled). Independent of the cut-off.
  float    shellyTAlarmC    = 100.0f;
};

/* --------------- Runtime state (RAM) ---------------- */
struct Runtime {
  float ecoRaw         = NAN, ecoSmooth   = NAN, shellyW   = NAN, restHouse   = NAN;
  float shellySmooth   = NAN;   // smoothed Shelly power (same filter as ecoSmooth, avoids the inertia lag)
  bool  shellyOn       = false, forcedNight = false, probing   = false;
  bool  manualOn       = false;
  float shellyT        = NAN;   // Shelly internal temperature (tC), read with Switch.GetStatus
  bool  shellyOvertemp = false; // the Shelly is currently reporting errors:["overtemp"]
  uint32_t shellyOtEpoch = 0;   // last overheating seen (epoch, RAM: reset to 0 on restart)
  bool     thermHold = false;   // thermal protection active: no switch-on command is sent
  bool     thermCritLogged = false;   // "alarm threshold exceeded" alert already written for this episode
  uint32_t thermHoldStartMs = 0, thermHoldCount = 0;   // start of the current pause (re-armed while it is still too hot) / triggers since boot
  bool  ecoConnected   = false, shellyConnected = false; 
  uint32_t manualSetMs = 0;
  uint32_t cycleId              = 0, heatedSec      = 0, tier1Sec     = 0;
  uint32_t tier2Sec             = 0, nightSec       = 0, relayOnSec   = 0;
  uint32_t monthKey             = 0, monthHeatedSec = 0, monthTier1Sec = 0;
  uint32_t monthTier2Sec        = 0, monthNightSec  = 0, monthRelayOnSec = 0;
  // Real energy and cost (Shelly measurements x real solar surplus), see accumulateEnergyCost().
  // Replace the former estimate duration x nominal power (cfg.heaterW).
  float    cycleWh = 0.0f, paidWh = 0.0f, nativeEUR = 0.0f, paidEUR = 0.0f;
  float    monthCycleWh = 0.0f, monthPaidWh = 0.0f, monthNativeEUR = 0.0f, monthPaidEUR = 0.0f;
  uint32_t lastTick             = 0, lowPowerSince  = 0;
  uint32_t lastHistorySave      = 0;
  uint32_t lastStateChangeMs    = 0; // time of the last ON->OFF transition (for the debounce)
  uint32_t lastWifiCheckMs      = 0, wifiDisconnectedSinceMs = 0;
  bool     apRescue = false, bootApOn = false, bootApDone = false, staticFallback = false; // rescue AP (Wi-Fi problem) / 2 min diagnostic AP / already handled this boot / static IP abandoned
  uint32_t bootApStartMs        = 0, wifiUpSinceMs = 0, wifiLostSinceMs = 0;
  uint32_t lastShellyPollMs     = 0, lastShellyCmdMs = 0, lastEcoPollMs = 0;
  uint32_t lastShellySuccessMs  = 0, lastEcoSuccessMs = 0;
  uint32_t errorResolvedMs      = 0, errorDayKey = 0, errorCountToday = 0;
  uint32_t errorRepeatCount     = 0; // occurrences of the same error episode (same errorKey) since the 1st one
  uint8_t  shellyFailCount      = 0, ecoFailCount = 0;
  uint32_t shellyLastMs         = 0, shellyMaxMs = 0, shellyRetryRecovered = 0; // Shelly GET latency (ms) and -11 errors recovered by retry
  ErrorSource errorSource       = ERROR_NONE;
  String state    = "INIT", error     = "", errorKey = "", ecoCookie = "";
  String gatewayId = "", powerMeterId = ""; 
} rt;

/* --------------- Error log (RAM, rolling over 24 h) ---------------- */
constexpr uint8_t ERROR_LOG_CAPACITY = 40; // enough for 24 h of intermittent errors; RAM only, lost on reboot
struct ErrorLogEntry {
  uint32_t epoch = 0; // 0 if the clock was invalid at the time of the event
  ErrorSource source = ERROR_NONE;
  char message[72] = {0};
};
ErrorLogEntry errorLog[ERROR_LOG_CAPACITY];
uint8_t errorLogHead  = 0; // next slot to write
uint8_t errorLogCount = 0; // number of valid entries (up to ERROR_LOG_CAPACITY)

static const char* errorSourceName(ErrorSource s) {
  switch (s) {
    case ERROR_SHELLY:  return "SHELLY";
    case ERROR_ECOJOKO: return "ECOJOKO";
    case ERROR_STORAGE: return "STORAGE";
    case ERROR_THERMAL: return "THERMAL";
    default:            return "NONE";
  }
}

static void pushErrorLog(const String &message, ErrorSource source) {
  time_t epoch = time(nullptr);
  ErrorLogEntry &e = errorLog[errorLogHead];
  e.epoch  = (epoch < 946684800) ? 0 : (uint32_t)epoch;
  e.source = source;
  strncpy(e.message, message.c_str(), sizeof(e.message) - 1);
  e.message[sizeof(e.message) - 1] = 0;
  errorLogHead = (errorLogHead + 1) % ERROR_LOG_CAPACITY;
  if (errorLogCount < ERROR_LOG_CAPACITY) errorLogCount++;
}

// Rewrites in place the LAST entry of the log (the one just before errorLogHead), without advancing the ring:
// used when reportError() sees a repetition of the same episode (same deduplication key), to
// refresh the message/time of that episode instead of stacking one line per attempt.
static void updateLastErrorLog(const String &message, ErrorSource source) {
  if (errorLogCount == 0) { pushErrorLog(message, source); return; } // should not happen here, safety net
  uint8_t lastIdx = (errorLogHead + ERROR_LOG_CAPACITY - 1) % ERROR_LOG_CAPACITY;
  ErrorLogEntry &e = errorLog[lastIdx];
  time_t epoch = time(nullptr);
  e.epoch  = (epoch < 946684800) ? 0 : (uint32_t)epoch; // time of the last occurrence
  e.source = source;
  strncpy(e.message, message.c_str(), sizeof(e.message) - 1);
  e.message[sizeof(e.message) - 1] = 0;
}

Config cfg;
WebServer server(80);

// Valid IP? (used by the network configuration)
static bool parseIp(const String &s, IPAddress &out) {
  return s.length() >= 7 && out.fromString(s.c_str());
}

// Records the URI and IP of the request in progress (called at the start of every handler).
static void diagWebTag(const char *uri) {
  strncpy(diagWebUri, uri, sizeof(diagWebUri) - 1);
  diagWebUri[sizeof(diagWebUri) - 1] = '\0';
  String ip = server.client().remoteIP().toString();
  strncpy(diagWebIp, ip.c_str(), sizeof(diagWebIp) - 1);
  diagWebIp[sizeof(diagWebIp) - 1] = '\0';
}

// Replaces server.on(uri, method, handler): same behaviour, plus the instrumentation.
static void webRoute(const char *uri, HTTPMethod method, std::function<void()> fn) {
  server.on(uri, method, [uri, fn]() {
    diagWebTag(uri);
    // State shared (cfg/rt/histories/LittleFS) with controlTask(): see the comment near ioMutex.
    if (xSemaphoreTake(ioMutex, IO_MUTEX_WEB_WAIT_TICKS) == pdTRUE) {
      fn();
      xSemaphoreGive(ioMutex);
    } else {
      server.send(503, "text/plain", "System busy (control cycle in progress), try again in a few seconds");
    }
  });
}

/* --------------- Forecast: per-cycle records + state (RAM) ---------------- */
static AntRec  antRecs[ANT_REC_MAX];
static uint8_t antNRec = 0;
static AntHr   antHr[ANT_HR_N];            // hourly forecast / actual ring (see antHrIdx)
static uint32_t antHrCur = 0;              // epoch hour currently being accumulated (0 = none)
static float    antHrWh = 0.0f;            // usable surplus accumulated over the current hour (Wh)
static uint32_t antHrSec = 0;              // seconds measured over the current hour
// Shading: ratio actual / forecast x100 per hour of the day (UTC hour: the sun's position does not follow the DST change).
static uint8_t  antShadeS[24][ANT_SHADE_N]; // ring of samples per hour (only the first antShadeCnt[h] are valid)
static uint8_t  antShadeHead[24], antShadeCnt[24];
static float    antShadeF[24];             // learned factor (1.0 = neutral), valid only if antShadeOk[h]
static bool     antShadeOk[24];
static float    antShadeRef = NAN;         // reference level: median of the hourly medians

// Heating log: see heatLogTick(). lastTickWh/PaidWh are filled in by accumulateEnergyCost().
// sec/wh/paidWh are uint32 (not uint16): a segment longer than 18 h (65535 s) or above 65535 Wh
// used to be silently truncated before this fix; a system meant to run for several years must
// avoid this case, however rare.
struct HeatSeg {
  uint32_t t0 = 0;       // start (epoch)
  uint32_t sec = 0;      // duration from the start to the last second of heating
  uint32_t wh = 0;       // energy injected into the tank
  uint32_t paidWh = 0;   // of which bought from the grid (excluding solar surplus)
  uint8_t  reason = 0;   // HR_HC / HR_SOLAR / HR_FORCE
};
static HeatSeg  heatSegs[HEAT_SEG_MAX];
static uint16_t heatNSeg = 0;                 // the oldest is at index 0
static HeatSeg  heatCur;                      // segment in progress
static bool     heatCurOpen = false;
static uint32_t heatCurIdle = 0;
static float    heatCurWh = 0.0f, heatCurPaid = 0.0f;
static bool     heatDirty = false;
static uint32_t heatLastSaveMs = 0;
static float    lastTickWh = 0.0f, lastTickPaidWh = 0.0f;

// Heating log, "relay" line: see relayLogTick(). One interval = relay confirmed ON, from switch-on to switch-off.
struct RelayIv {
  uint32_t t0 = 0;       // start (epoch)
  uint32_t sec = 0;      // duration up to the last ON measurement (uint32, same reason as HeatSeg.sec)
};
static RelayIv  relayIvs[RELAY_IV_MAX];
static uint16_t relayNIv = 0;                 // the oldest is at index 0
static RelayIv  relayCur;                     // interval in progress
static bool     relayCurOpen = false;
static uint32_t relayCurUnknown = 0;          // seconds without a known Shelly state during the current interval

/* Shelly internal temperature: ring of STEMP_N points (one per STEMP_STEP_SEC). t10 = maximum of the interval in
   degC x10 (INT16_MIN = no measurement), w = average power (W). Fed by stempFeed() on every successful Shelly read. */
struct STempPt { int16_t t10; uint16_t w; };
static STempPt  stPts[STEMP_N];
static uint16_t stCount = 0, stHead = 0;      // valid points / next write index
static uint32_t stLastEpoch = 0;              // end of the last closed interval (multiple of STEMP_STEP_SEC)
static int16_t  stMax10 = INT16_MIN;          // interval in progress
static uint32_t stWSum = 0, stWN = 0;

static void stempPush(int16_t t10, uint16_t w) {
  stPts[stHead].t10 = t10; stPts[stHead].w = w;
  stHead = (uint16_t)((stHead + 1) % STEMP_N);
  if (stCount < STEMP_N) stCount++;
}

static void stempFeed(float tC, float w) {
  time_t ne = time(nullptr);
  if (ne < 946684800) return;
  uint32_t e = (uint32_t)ne;
  if (stLastEpoch == 0 || (e > stLastEpoch && e - stLastEpoch > (uint32_t)STEMP_N * STEMP_STEP_SEC)) {
    stCount = 0; stHead = 0; stMax10 = INT16_MIN; stWSum = 0; stWN = 0;   // first measurement or gap > 48 h: start again from zero
    stLastEpoch = e - (e % STEMP_STEP_SEC);
  }
  while (e >= stLastEpoch + STEMP_STEP_SEC) {   // closes the finished intervals (gaps without measurement give empty points)
    uint16_t wa = stWN ? (uint16_t)fminf(65535.0f, (float)stWSum / (float)stWN + 0.5f) : 0;
    stempPush(stMax10, wa);
    stMax10 = INT16_MIN; stWSum = 0; stWN = 0;
    stLastEpoch += STEMP_STEP_SEC;
  }
  if (!isnan(tC) && tC > -40.0f && tC < 150.0f) {
    int16_t v = (int16_t)lroundf(tC * 10.0f);
    if (stMax10 == INT16_MIN || v > stMax10) stMax10 = v;
  }
  if (!isnan(w) && w >= 0.0f) { stWSum += (uint32_t)w; stWN++; }
}

struct AntState {
  // Weather
  uint32_t wxStart = 0, wxEpoch = 0, wxLastTryMs = 0, wxLastOkMs = 0;
  uint8_t  wxN = 0, wxFail = 0;
  int16_t  wxT10[ANT_WX_MAX];
  uint16_t wxRad[ANT_WX_MAX];
  char     wxErr[24] = {0};
  // Tank state seen as a battery
  bool     refValid = false, refBad = false, wasHeating = false;
  uint32_t refEpoch = 0, lastFullEpoch = 0, gapSec = 0, heatRun = 0, idleRun = 0;
  float    whSince = 0.0f;                 // energy injected since the last "full"
  float    errAct = 0.0f, errPred = 0.0f;  // model / reality gap (sums with decaying weights)
  // Totals of the current cycle
  float    cycMeasWh = 0.0f, cycRadWh = 0.0f, cycPotWh = 0.0f, cycSkipWh = 0.0f;
  float    cycPredWh = -1.0f, cycPredSolWh = -1.0f; // forecast frozen at the start of the current cycle (< 0 = not taken yet)
  uint32_t cycMeasSec = 0, cycTSec = 0;
  int32_t  cycTSum10 = 0;
  // Learning
  float    needBase = 0.0f, slope = 0.0f, tRef = 0.0f, k = 0.0f, kSumRad = 0.0f, kLearn = 0.0f, kW = 0.0f; // k = effective coefficient; kLearn = learned only; kW = share of the learned part (0..1)
  float    tBase = NAN;   // average temperature of the cycles used to compute needBase (actual reference of the correction)
  bool     slopeValid = false, kValid = false;
  uint8_t  needN = 0, kN = 0;
  // Current decision
  bool     valid = false, heat = true, force = false, heatPlan = false, forcing = false;
  float    dNowWh = 0.0f, hWh = 0.0f, erechWh = 0.0f, needDayWh = 0.0f;
  uint32_t tsEpoch = 0, tdEpoch = 0;
  char     reason[28] = "startup";
};
static AntState ant;

/* --------------- Time utilities ---------------- */
static bool localNow(tm &local) {
  time_t epoch = time(nullptr);
  if (epoch < 946684800) return false; // NTP/manual clock not valid yet.
  localtime_r(&epoch, &local);
  return true;
}

static uint16_t nowMin() {
  tm local;
  return localNow(local) ? (uint16_t)(local.tm_hour * 60 + local.tm_min) : 0;
}

static String nowText() {
  tm local;
  if (!localNow(local)) return "invalid-clock";
  char out[24];
  strftime(out, sizeof(out), "%Y-%m-%d %H:%M:%S", &local);
  return String(out);
}

static uint32_t localDayKey() {
  tm local;
  return localNow(local) ? (uint32_t)(local.tm_year + 1900) * 1000UL + (uint32_t)local.tm_yday + 1 : 0;
}

static void refreshDailyErrorCount() {
  uint32_t dayKey = localDayKey();
  if (dayKey != 0 && rt.errorDayKey != 0 && dayKey != rt.errorDayKey) rt.errorCountToday = 0;
  if (dayKey != 0) rt.errorDayKey = dayKey;
}

// Variant with a deduplication key distinct from the displayed message: use it when the message contains
// a detail that varies on every attempt (duration, etc.) and would otherwise break deduplication on every call,
// flooding the log with one line per attempt instead of one line per episode (with a counter). The displayed
// message (rt.error, /api/status) is always kept up to date with the latest text received.
static void reportError(const String &dedupKey, const String &message, ErrorSource source) {
  refreshDailyErrorCount();
  bool sameEpisode = (rt.errorKey == dedupKey && rt.errorSource == source && rt.errorResolvedMs == 0);
  if (sameEpisode) {
    rt.errorRepeatCount++;
    String shown = message + (rt.errorRepeatCount > 1 ? " (x" + String(rt.errorRepeatCount) + ")" : "");
    updateLastErrorLog(shown, source);
    rt.error = shown;
  } else {
    rt.errorRepeatCount = 1;
    ++rt.errorCountToday;
    pushErrorLog(message, source);
    rt.error = message;
  }
  rt.errorKey = dedupKey;
  rt.errorSource = source;
  rt.errorResolvedMs = 0;
}

static void reportError(const String &message, ErrorSource source) {
  reportError(message, message, source); // no distinct key: deduplication identical to before (exact text)
}

static void markErrorResolved(ErrorSource source) {
  if (rt.error.length() && rt.errorSource == source && rt.errorResolvedMs == 0) rt.errorResolvedMs = millis();
}

static void clearResolvedError() {
  if (rt.errorResolvedMs != 0 && (uint32_t)(millis() - rt.errorResolvedMs) >= ERROR_CLEAR_DELAY_MS) {
    rt.error = "";
    rt.errorKey = "";
    rt.errorRepeatCount = 0;
    rt.errorSource = ERROR_NONE;
    rt.errorResolvedMs = 0;
  }
}

static void logNetworkFailure(const char *service, const char *step, int httpCode) {
  Serial.printf("[%s] network failure step=%s http=%d wifiStatus=%d rssi=%d time=%s\n",
                service, step, httpCode, (int)WiFi.status(), WiFi.RSSI(), nowText().c_str());
}

static uint32_t networkBackoffMs(uint8_t failCount) {
  if (failCount <= NETWORK_BACKOFF_AFTER_FAILURES) return CONTROL_PERIOD_MS;
  uint32_t delayMs = CONTROL_PERIOD_MS;
  for (uint8_t i = NETWORK_BACKOFF_AFTER_FAILURES + 1;
       i < failCount && delayMs < NETWORK_BACKOFF_MAX_MS; ++i) {
    delayMs = min(delayMs * 2UL, NETWORK_BACKOFF_MAX_MS);
  }
  return delayMs;
}

static bool networkPollDue(uint32_t lastPollMs, uint8_t failCount, uint32_t now) {
  return lastPollMs == 0 || (uint32_t)(now - lastPollMs) >= networkBackoffMs(failCount);
}

// Automatic restart: Ecojoko unreachable (Wi-Fi OK) for more than 30 min. The backoff is NOT modified.
constexpr uint32_t ECO_BLOCKED_RESTART_MS = 30UL * 60UL * 1000UL;

// Time (ms) for which Ecojoko has been unreachable while Wi-Fi works. 0 if all is well, if Ecojoko is not
// configured or if Wi-Fi is down (handled elsewhere). The count restarts at Wi-Fi reconnection: a Wi-Fi outage
// therefore never triggers a restart as soon as the network comes back.
static uint32_t ecoBlockedAgeMs(uint32_t now) {
  if (cfg.ecoEmail.length() == 0 || cfg.ecoPassword.length() == 0) return 0;
  if (rt.ecoFailCount == 0 || rt.wifiUpSinceMs == 0 || WiFi.status() != WL_CONNECTED) return 0;
  uint32_t sinceOk = (uint32_t)(now - rt.lastEcoSuccessMs); // lastEcoSuccessMs = 0: never succeeded since boot
  uint32_t sinceWifi = (uint32_t)(now - rt.wifiUpSinceMs);
  return sinceOk < sinceWifi ? sinceOk : sinceWifi;
}

static void recordShellyResult(bool success) {
  rt.shellyFailCount = success ? 0 : (rt.shellyFailCount < UINT8_MAX ? rt.shellyFailCount + 1 : UINT8_MAX);
  if (!success) {
    Serial.printf("[Shelly] consecutive failure=%u next_poll_in=%lums\n",
                  rt.shellyFailCount, (unsigned long)networkBackoffMs(rt.shellyFailCount));
  }
}

static void recordShellyPollResult(const char *origin, bool success) {
  if (strcmp(origin, "poll") == 0) recordShellyResult(success);
}

static void recordEcoResult(bool success) {
  rt.ecoFailCount = success ? 0 : (rt.ecoFailCount < UINT8_MAX ? rt.ecoFailCount + 1 : UINT8_MAX);
  if (!success) {
    Serial.printf("[Ecojoko] consecutive failure=%u next_poll_in=%lums\n",
                  rt.ecoFailCount, (unsigned long)networkBackoffMs(rt.ecoFailCount));
  }
}

static bool inHC(uint16_t m) {
  return cfg.hcStart < cfg.hcEnd 
         ? (m >= cfg.hcStart && m < cfg.hcEnd) 
         : (m >= cfg.hcStart || m < cfg.hcEnd);
}

static uint32_t cycleForNow() {
  tm local;
  if (!localNow(local)) return 0;
  // A quota day starts at the end of HC, using local French time (DST-safe).
  if ((uint16_t)(local.tm_hour * 60 + local.tm_min) < cfg.hcEnd) {
    local.tm_mday -= 1;
    mktime(&local);
  }
  return (uint32_t)(local.tm_year + 1900) * 1000UL + (uint32_t)local.tm_yday;
}

// Calendar month (YYYYMM) of the cycle that has just ended: at the moment the
// rollover fires, "now" is just past hcEnd, i.e. already in the new cycle's
// calendar day, so the closing cycle's date is "yesterday".
static uint32_t monthKeyForClosingCycle() {
  tm local;
  if (!localNow(local)) return 0;
  local.tm_mday -= 1;
  mktime(&local);
  return (uint32_t)(local.tm_year + 1900) * 100UL + (uint32_t)(local.tm_mon + 1);
}

// Calendar month (YYYYMM) of today, used to merge today's in-progress cycle
// into the current month's totals for display without persisting it early.
static uint32_t monthKeyForToday() {
  tm local;
  if (!localNow(local)) return 0;
  return (uint32_t)(local.tm_year + 1900) * 100UL + (uint32_t)(local.tm_mon + 1);
}

static uint32_t secondsUntilHcEnd(uint16_t minute) {
  if (!inHC(minute)) return 0;
  uint16_t remainingMin = cfg.hcEnd >= minute
                            ? cfg.hcEnd - minute
                            : (uint16_t)(1440U - minute + cfg.hcEnd);
  return (uint32_t)remainingMin * 60UL;
}

static bool inTimeWindow(uint16_t now, uint16_t start, uint16_t end) {
  return start < end ? (now >= start && now < end) : (now >= start || now < end);
}

static void applyClockConfig() {
  if (cfg.autoTime) {
    configTzTime(FRANCE_TZ, "pool.ntp.org", "fr.pool.ntp.org", "time.nist.gov");
    return;
  }

  setenv("TZ", FRANCE_TZ, 1);
  tzset();
  if (cfg.manualEpoch >= 946684800UL) { // 2000-01-01: reject an unset manual value.
    timeval tv = { (time_t)cfg.manualEpoch, 0 };
    settimeofday(&tv, nullptr);
  }
}

/* ============================================================
   1. CONFIGURATION MANAGEMENT (LittleFS)
   ============================================================ */
constexpr char PASS_MASK[] = "********"; // value returned in place of passwords by GET /api/config; sent back unchanged = "unchanged"

// Safety limits: an absurd value (0, negative, out of range, typo) falls back to the default value.
// Notably avoids smoothingMin = 0 (division by zero -> NaN smoothing -> no more solar decisions) and heaterW = 0.
static void sanitizeConfig() {
  if (cfg.mode != "auto" && cfg.mode != "manual" && cfg.mode != "monitor" && cfg.mode != "full") cfg.mode = "full";
  if (cfg.antMode != "off" && cfg.antMode != "shadow" && cfg.antMode != "on") cfg.antMode = "shadow";
  if (cfg.smoothingMin < 1) cfg.smoothingMin = 1;
  if (cfg.smoothingMin > 30) cfg.smoothingMin = 30;
  if (!(cfg.heaterW >= 100.0f && cfg.heaterW <= 10000.0f)) cfg.heaterW = 2000.0f;
  if (!(cfg.thermostatW >= 1.0f && cfg.thermostatW <= cfg.heaterW)) cfg.thermostatW = fminf(100.0f, cfg.heaterW);
  if (!(cfg.hp > 0.0f && cfg.hp <= 5.0f)) cfg.hp = 0.2516f;
  if (!(cfg.hc > 0.0f && cfg.hc <= 5.0f)) cfg.hc = 0.1696f;
  if (!(cfg.quotaH >= 0.0f && cfg.quotaH <= 24.0f)) cfg.quotaH = 4.0f;
  if (cfg.ecoGraceSec > 600) cfg.ecoGraceSec = 15;
  if (cfg.hcStart > 1439) cfg.hcStart = 23 * 60;
  if (cfg.hcEnd > 1439) cfg.hcEnd = 7 * 60;
  if (cfg.manualStart > 1439) cfg.manualStart = 7 * 60;
  if (cfg.manualEnd > 1439) cfg.manualEnd = 10 * 60;
  if (cfg.reservePct > 100) cfg.reservePct = 40;
  if (cfg.deadlineMin > 1439) cfg.deadlineMin = 18 * 60 + 30;
  if (cfg.tankLiters < 20 || cfg.tankLiters > 2000) cfg.tankLiters = 200;
  if (!(cfg.lat >= -90.0f && cfg.lat <= 90.0f)) cfg.lat = 50.40f;
  if (!(cfg.lon >= -180.0f && cfg.lon <= 180.0f)) cfg.lon = 3.15f;

  // Fields added later (watchdog, quiet window, forecast, network, Shelly thermal protection):
  // same limits as the /api/config (POST) validation, but applied HERE on every load (boot, POST,
  // and any settings restore), so that a corrupted /config.json or an incomplete import can
  // never make the system run with an out-of-range value. Silent correction (default value),
  // like the rest of this function: no boot blocking.
  if (!wdPinAllowed(cfg.wdHbPin)) cfg.wdHbPin = -1;
  if (!wdPinAllowed(cfg.wdErrPin) || (cfg.wdErrPin >= 0 && cfg.wdErrPin == cfg.wdHbPin)) cfg.wdErrPin = -1;
  if (!wdPinAllowed(cfg.wdTxPin) || (cfg.wdTxPin >= 0 && (cfg.wdTxPin == cfg.wdHbPin || cfg.wdTxPin == cfg.wdErrPin))) cfg.wdTxPin = -1;
  if (cfg.wdProblemMin < 5 || cfg.wdProblemMin > 720) cfg.wdProblemMin = 60;
  if (cfg.muteStart > 1439) cfg.muteStart = 23 * 60;
  if (cfg.muteEnd > 1439) cfg.muteEnd = 7 * 60;
  if (cfg.snoozeMin < 5 || cfg.snoozeMin > 720) cfg.snoozeMin = 480;
  if (!(cfg.tankKWh >= 0.0f && cfg.tankKWh <= 60.0f)) cfg.tankKWh = 0.0f;
  if (!(cfg.pvKwc >= 0.0f && cfg.pvKwc <= 50.0f)) cfg.pvKwc = 0.0f;
  if (cfg.bootApSec > 600) cfg.bootApSec = 120;
  if (!(cfg.shellyTMaxC >= 0.0f && cfg.shellyTMaxC <= 150.0f)) cfg.shellyTMaxC = 85.0f;
  if (!(cfg.shellyTResumeC >= 0.0f && cfg.shellyTResumeC <= 150.0f)) cfg.shellyTResumeC = 65.0f;
  if (cfg.shellyTMaxC > 0.0f && cfg.shellyTResumeC >= cfg.shellyTMaxC) cfg.shellyTResumeC = fmaxf(0.0f, cfg.shellyTMaxC - 10.0f);
  if (cfg.shellyHoldMin < 1 || cfg.shellyHoldMin > 240) cfg.shellyHoldMin = 10;
  if (!(cfg.shellyTAlarmC >= 0.0f && cfg.shellyTAlarmC <= 200.0f)) cfg.shellyTAlarmC = 100.0f;
  if (cfg.shellyIp.length()) {
    IPAddress a;
    if (!parseIp(cfg.shellyIp, a)) cfg.shellyIp = ""; // malformed IP: disabled rather than attempting requests to an invalid address
  }
  if (cfg.staticIp) {
    IPAddress a;
    bool netOk = parseIp(cfg.ip, a) && parseIp(cfg.gw, a) && parseIp(cfg.mask, a) && (!cfg.dns.length() || parseIp(cfg.dns, a));
    if (!netOk) cfg.staticIp = false; // inconsistent static IP config: DHCP fallback rather than booting with an invalid IP
  }
}

bool readConfig() {
  Serial.println("[DEBUG] Reading the configuration file...");
  File f = LittleFS.open("/config.json", "r"); 
  if (!f) {
    Serial.println("[DEBUG] No config.json file found.");
    return false; 
  }
  JsonDocument d; 
  DeserializationError err = deserializeJson(d, f);
  f.close();
  if (err || !d.is<JsonObject>()) {
    Serial.println("[DEBUG] JSON parsing error in the config.");
    return false;
  }
  
  cfg.wifiSsid         = d["wifiSsid"] | "";
  cfg.wifiPassword     = d["wifiPassword"] | "";
  cfg.ecoEmail         = d["ecoEmail"] | "";
  cfg.ecoPassword      = d["ecoPassword"] | "";
  cfg.shellyIp         = d["shellyIp"] | "";
  cfg.mode             = d["mode"] | cfg.mode;
  cfg.lastFwBuild      = d["lastFwBuild"] | "";
  cfg.manualTimer      = d["manualTimer"] | cfg.manualTimer; 
  cfg.manualStart      = d["manualStart"] | cfg.manualStart;
  cfg.manualEnd        = d["manualEnd"] | cfg.manualEnd;
  cfg.manualEpoch      = (uint32_t)(d["manualEpoch"] | 0);
  cfg.autoTime         = d["autoTime"] | cfg.autoTime;
  cfg.hcStart          = d["hcStart"] | cfg.hcStart;
  cfg.hcEnd            = d["hcEnd"] | cfg.hcEnd;
  cfg.heaterW          = (float)(d["heaterW"] | cfg.heaterW);
  cfg.thermostatW      = (float)(d["thermostatW"] | cfg.thermostatW);
  cfg.hp               = (float)(d["hp"] | cfg.hp);
  cfg.hc               = (float)(d["hc"] | cfg.hc);
  cfg.quotaH           = (float)(d["quotaH"] | cfg.quotaH);
  cfg.smoothingMin     = (uint8_t)(d["smoothingMin"] | cfg.smoothingMin);
  cfg.ecoGraceSec      = (uint16_t)(d["ecoGraceSec"] | cfg.ecoGraceSec);
  cfg.wdHbPin          = (int8_t)(d["wdHbPin"] | (int)cfg.wdHbPin);
  cfg.wdErrPin         = (int8_t)(d["wdErrPin"] | (int)cfg.wdErrPin);
  cfg.wdProblemMin     = (uint16_t)(d["wdProblemMin"] | (int)cfg.wdProblemMin);
  cfg.wdTxPin          = (int8_t)(d["wdTxPin"] | (int)cfg.wdTxPin);
  cfg.muteOn           = d["muteOn"] | cfg.muteOn;
  cfg.muteStart        = (uint16_t)(d["muteStart"] | (int)cfg.muteStart);
  cfg.muteEnd          = (uint16_t)(d["muteEnd"] | (int)cfg.muteEnd);
  cfg.snoozeMin        = (uint16_t)(d["snoozeMin"] | (int)cfg.snoozeMin);
  cfg.antMode          = d["antMode"] | cfg.antMode;
  cfg.antShade         = d["antShade"] | cfg.antShade;
  cfg.tankLiters       = (uint16_t)(d["tankLiters"] | (int)cfg.tankLiters);
  cfg.tankKWh          = (float)(d["tankKWh"] | cfg.tankKWh);
  cfg.reservePct       = (uint8_t)(d["reservePct"] | (int)cfg.reservePct);
  cfg.deadlineMin      = (uint16_t)(d["deadlineMin"] | (int)cfg.deadlineMin);
  cfg.lat              = (float)(d["lat"] | cfg.lat);
  cfg.lon              = (float)(d["lon"] | cfg.lon);
  cfg.pvKwc            = (float)(d["pvKwc"] | cfg.pvKwc);
  cfg.staticIp         = d["staticIp"] | cfg.staticIp;
  cfg.ip               = d["ip"] | "";
  cfg.gw               = d["gw"] | "";
  cfg.mask             = d["mask"] | "255.255.255.0";
  cfg.dns              = d["dns"] | "";
  cfg.bootApSec        = (uint16_t)(d["bootApSec"] | (int)cfg.bootApSec);
  cfg.shellyTMaxC      = d["shellyTMaxC"] | cfg.shellyTMaxC;
  cfg.shellyTResumeC   = d["shellyTResumeC"] | cfg.shellyTResumeC;
  cfg.shellyHoldMin    = (uint16_t)(d["shellyHoldMin"] | (int)cfg.shellyHoldMin);
  cfg.shellyTAlarmC = d["shellyTAlarmC"] | cfg.shellyTAlarmC;
  // All safety limits (pins, ranges, temperatures, IP...) are now checked
  // ONLY here, by sanitizeConfig(): no more double checking to maintain in parallel (it was this
  // gap between readConfig() and sanitizeConfig() that had let muteStart/muteEnd/snoozeMin/
  // tankKWh/pvKwc/bootApSec/shellyT* through when /config.json was loaded directly.)
  sanitizeConfig();
  
  Serial.println("[DEBUG] Configuration loaded successfully.");
  return true;
}

void saveConfig() {
  Serial.println("[DEBUG] Saving the configuration...");
  JsonDocument d;
  
  d["wifiSsid"]         = cfg.wifiSsid; 
  d["wifiPassword"]     = cfg.wifiPassword;
  d["ecoEmail"]         = cfg.ecoEmail; 
  d["ecoPassword"]      = cfg.ecoPassword;
  d["shellyIp"]         = cfg.shellyIp; 
  d["mode"]             = cfg.mode;
  d["lastFwBuild"]      = cfg.lastFwBuild;
  d["manualTimer"]      = cfg.manualTimer; 
  d["manualStart"]      = cfg.manualStart;
  d["manualEnd"]        = cfg.manualEnd;
  d["manualEpoch"]      = (uint32_t)cfg.manualEpoch; 
  d["autoTime"]         = cfg.autoTime;
  d["hcStart"]          = cfg.hcStart;
  d["hcEnd"]            = cfg.hcEnd;
  d["heaterW"]          = cfg.heaterW;
  d["thermostatW"]      = cfg.thermostatW;
  d["hp"]               = cfg.hp;
  d["hc"]               = cfg.hc;
  d["quotaH"]           = cfg.quotaH; 
  d["smoothingMin"]     = cfg.smoothingMin;
  d["ecoGraceSec"]      = cfg.ecoGraceSec;
  d["wdHbPin"]          = cfg.wdHbPin;
  d["wdErrPin"]         = cfg.wdErrPin;
  d["wdProblemMin"]     = cfg.wdProblemMin;
  d["wdTxPin"]          = cfg.wdTxPin;
  d["muteOn"]           = cfg.muteOn;
  d["muteStart"]        = cfg.muteStart;
  d["muteEnd"]          = cfg.muteEnd;
  d["snoozeMin"]        = cfg.snoozeMin;
  d["antMode"]          = cfg.antMode;
  d["antShade"]         = cfg.antShade;
  d["tankLiters"]       = cfg.tankLiters;
  d["tankKWh"]          = cfg.tankKWh;
  d["reservePct"]       = cfg.reservePct;
  d["deadlineMin"]      = cfg.deadlineMin;
  d["lat"]              = cfg.lat;
  d["lon"]              = cfg.lon;
  d["pvKwc"]            = cfg.pvKwc;
  d["staticIp"]         = cfg.staticIp;
  d["ip"]               = cfg.ip;
  d["gw"]               = cfg.gw;
  d["mask"]             = cfg.mask;
  d["dns"]              = cfg.dns;
  d["bootApSec"]        = cfg.bootApSec;
  d["shellyTMaxC"]      = cfg.shellyTMaxC;
  d["shellyTResumeC"]   = cfg.shellyTResumeC;
  d["shellyHoldMin"]    = cfg.shellyHoldMin;
  d["shellyTAlarmC"]    = cfg.shellyTAlarmC;

  File f = LittleFS.open("/config.json", "w"); 
  if (f) { 
    serializeJson(d, f); 
    f.close();
    Serial.println("[DEBUG] Configuration saved.");
  }
}

/* ============================================================
   2. SHELLY 1PM GEN3 API (RPC)
   ============================================================ */
static bool readShelly(const char *origin = "poll");

/* Shelly error type, readable in the web error log (without a serial monitor).
   The message is deliberately STABLE (no duration/RSSI): reportError() only pushes a new entry
   when the message changes, so the same type of failure repeated does not pollute the log.
   Duration and RSSI remain in the Serial log (logNetworkFailure + DURATION_MS line). */
static String shellyHttpDetail(int code) {
  if (WiFi.status() != WL_CONNECTED) return "ESP32 Wi-Fi down";
  if (code > 0) return "HTTP response " + String(code);
  const char *label;
  switch (code) {
    case HTTPC_ERROR_CONNECTION_REFUSED:  label = "connection refused";      break; // Shelly overloaded / closing the port
    case HTTPC_ERROR_SEND_HEADER_FAILED:
    case HTTPC_ERROR_SEND_PAYLOAD_FAILED: label = "request send failed";   break;
    case HTTPC_ERROR_NOT_CONNECTED:       label = "socket not connected";   break;
    case HTTPC_ERROR_CONNECTION_LOST:     label = "connection lost";       break; // cut during the exchange
    case HTTPC_ERROR_NO_STREAM:
    case HTTPC_ERROR_NO_HTTP_SERVER:      label = "non-HTTP response";       break;
    case HTTPC_ERROR_TOO_LESS_RAM:        label = "ESP32 RAM insufficient"; break;
    case HTTPC_ERROR_READ_TIMEOUT:        label = "response timeout";        break; // the Shelly did not answer in time
    default:                              label = "transport error";       break;
  }
  return String(label) + " (" + String(code) + ")";
}

/* ---- Shelly relay command: NON-blocking ----
   Before: shellySet() stayed in loop() for up to 3 x (Switch.Set + 2 s wait + re-read) ~ 45 s worst case, during which
   the web server no longer answered. Now: shellySet() sends ONE Switch.Set then returns. The confirmation (re-reading
   the state SHELLY_CONFIRM_DELAY_MS later) and the following attempts are done by shellyCmdService(), called on every
   pass of loop(): the web stays served during the wait. Final result (confirmed success / failure after 3 attempts):
   recordShellyResult(), wdNoteCommand() and the error log, exactly as before. */
struct ShellyCmd {
  bool     active         = false;
  bool     on             = false;
  uint8_t  attempt        = 0;     // attempt in progress (1..SHELLY_MAX_ATTEMPTS)
  uint32_t sentMs         = 0;     // time the current Switch.Set was sent
  int      lastCode       = -1;    // HTTP code of the last Switch.Set (-1 = not sent)
  bool     lastPrepared   = true;  // false if h.begin() failed
  bool     lastStatusRead = false; // last confirmation state read succeeded
} shellyCmd;

// Sends the Switch.Set of the current attempt (blocks for at most connect + read, ~6.5 s). Confirms nothing.
static void shellyCmdSend() {
  DiagPhaseScope phase("shelly_set");
  Serial.printf("[Shelly] attempt %u/%u STEP=START command=%s\n",
                shellyCmd.attempt, SHELLY_MAX_ATTEMPTS, shellyCmd.on ? "ON" : "OFF");
  HTTPClient h;
  h.setConnectTimeout(SHELLY_CONNECT_TIMEOUT_MS);
  h.setTimeout(SHELLY_READ_TIMEOUT_MS);
  h.setReuse(false); // Connection: close -> frees the Shelly's slot as soon as it has answered
  String url = "http://" + cfg.shellyIp + "/rpc/Switch.Set?id=0&on=" + (shellyCmd.on ? "true" : "false");
  int code = -1;
  shellyCmd.lastPrepared = h.begin(url);
  if (shellyCmd.lastPrepared) {
    Serial.printf("[Shelly] attempt %u STEP=HTTP_SEND command=%s\n", shellyCmd.attempt, shellyCmd.on ? "ON" : "OFF");
    code = h.GET();
    h.end();
    Serial.printf("[Shelly] attempt %u STEP=HTTP_RESPONSE code=%d\n", shellyCmd.attempt, code);
  } else {
    Serial.printf("[Shelly] attempt %u STEP=HTTP_PREPARATION_FAILED (no request sent)\n", shellyCmd.attempt);
  }
  shellyCmd.lastCode = code;
  shellyCmd.sentMs = millis();
}

// Requests the relay "on" state. Returns true if a command is sent or already in progress (confirmation comes later),
// false if nothing could be sent (missing IP, network backoff in progress).
static bool shellySet(bool on) {
  if (!cfg.shellyIp.length()) {
    reportError("Shelly IP missing", ERROR_SHELLY);
    wdNoteCommand(false);
    return false;
  }
  uint32_t now = millis();
  if (shellyCmd.active) {
    if (shellyCmd.on == on) return true; // same command already in progress: wait for its confirmation, do not resend
    Serial.printf("[Shelly] command %s replaces the command %s in progress\n", on ? "ON" : "OFF", shellyCmd.on ? "ON" : "OFF");
    // Opposite command: it replaces the one in progress, without going through the backoff (the Shelly has just answered).
  } else if (!networkPollDue(rt.lastShellyCmdMs, rt.shellyFailCount, now)) {
    Serial.printf("[Shelly] command=%s deferred (backoff in progress, consecutive failures=%u, next_try_in=%lums)\n",
                  on ? "ON" : "OFF", rt.shellyFailCount,
                  (unsigned long)(networkBackoffMs(rt.shellyFailCount) - (now - rt.lastShellyCmdMs)));
    return false;
  }
  rt.lastShellyCmdMs = now;
  shellyCmd.active = true;
  shellyCmd.on = on;
  shellyCmd.attempt = 1;
  shellyCmd.lastCode = -1;
  shellyCmd.lastPrepared = true;
  shellyCmd.lastStatusRead = false;
  shellyCmdSend();
  return true;
}

// Called on every pass of loop(): confirms the command in progress once the delay has elapsed, or starts a new attempt.
static void shellyCmdService(uint32_t now) {
  if (!shellyCmd.active) return;
  if ((uint32_t)(now - shellyCmd.sentMs) < SHELLY_CONFIRM_DELAY_MS) return;
  if (cfg.mode == "monitor") { shellyCmd.active = false; return; } // monitor: tank not controlled, nothing left to confirm

  DiagPhaseScope phase("shelly_confirm");
  const bool on = shellyCmd.on;
  const uint8_t attempt = shellyCmd.attempt;
  Serial.printf("[Shelly] attempt %u STEP=READ_STATE\n", attempt);
  bool statusRead = readShelly("confirmation");
  shellyCmd.lastStatusRead = statusRead;
  Serial.printf("[Shelly] attempt %u STEP=STATE_READ read=%s actual=%s power=%.1fW\n",
                attempt, statusRead ? "OK" : "FAIL", rt.shellyOn ? "ON" : "OFF", rt.shellyW);
  // A HTTP 200 only acknowledges the RPC request; it is not a confirmation that the relay has actually switched.
  if (shellyCmd.lastCode == 200 && statusRead && rt.shellyOn == on) {
    shellyCmd.active = false;
    markErrorResolved(ERROR_SHELLY);
    recordShellyResult(true);
    wdNoteCommand(true);
    Serial.printf("[Shelly] FINAL_RESULT=SUCCESS attempt=%u command=%s confirmed\n", attempt, on ? "ON" : "OFF");
    return;
  }
  if (attempt < SHELLY_MAX_ATTEMPTS) {
    if (on && rt.thermHold) { // thermal protection triggered in the meantime: never resend a switch-on command
      shellyCmd.active = false;
      Serial.println("[Shelly] ON command abandoned (thermal protection active)");
      return;
    }
    Serial.printf("[Shelly] attempt %u RESULT=FAILURE -> NEW_ATTEMPT\n", attempt);
    shellyCmd.attempt = attempt + 1;
    shellyCmdSend();
    return;
  }

  shellyCmd.active = false;
  String why;
  if (!shellyCmd.lastPrepared)         why = "request not prepared";
  else if (shellyCmd.lastCode != 200)  why = shellyHttpDetail(shellyCmd.lastCode);
  else if (!shellyCmd.lastStatusRead)  why = "state read failed";
  else                                 why = String("actual state ") + (rt.shellyOn ? "ON" : "OFF");
  reportError("Shelly: cmd " + String(on ? "ON" : "OFF") + " not confirmed - " + why, ERROR_SHELLY);
  recordShellyResult(false);
  wdNoteCommand(false);
  Serial.printf("[Shelly] FINAL_RESULT=FAILURE command=%s not confirmed after %u attempts\n", on ? "ON" : "OFF", SHELLY_MAX_ATTEMPTS);
}

static bool readShelly(const char *origin) {
  DiagPhaseScope phase("shelly_read");
  if (cfg.mode == "monitor") { 
    rt.shellyOn = false;
    rt.shellyW = NAN; 
    rt.shellyConnected = false; 
    recordShellyPollResult(origin, true);
    return true;
  }
  if (!cfg.shellyIp.length()) {
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    rt.shellyOn = false;
    recordShellyPollResult(origin, false);
    return false;
  }
  // Single, immediate retry only on -11, and only for the periodic poll: confirmation reads of
  // shellySet() are already covered by its own attempt loop.
  const uint8_t maxTries = (strcmp(origin, "poll") == 0) ? SHELLY_READ_MAX_TRIES : 1;
  String payload;
  int code = -1;
  int retryCause = 0; // code that triggered the retry (-1 or -11), for the after-the-fact success message
  for (uint8_t tryNo = 1; tryNo <= maxTries; ++tryNo) {
    HTTPClient h;
    h.setConnectTimeout(SHELLY_CONNECT_TIMEOUT_MS);
    h.setTimeout(SHELLY_READ_TIMEOUT_MS);
    h.setReuse(false); // Connection: close -> no socket kept open on the Shelly side
    if (!h.begin("http://" + cfg.shellyIp + "/rpc/Switch.GetStatus?id=0")) {
      Serial.printf("[Shelly] read origin=%s STEP=HTTP_PREPARATION_FAILED\n", origin);
      logNetworkFailure("Shelly", "http_preparation", -1);
      reportError("Invalid Shelly URL", ERROR_SHELLY);
      rt.shellyConnected = false;
      rt.shellyW = NAN;
      rt.shellyOn = false;
      recordShellyPollResult(origin, false);
      return false;
    }
    Serial.printf("[Shelly] read origin=%s STEP=HTTP_SEND try=%u/%u\n", origin, tryNo, maxTries);
    uint32_t t0 = millis();
    code = h.GET();
    uint32_t durMs = millis() - t0;
    rt.shellyLastMs = durMs;
    if (durMs > rt.shellyMaxMs) rt.shellyMaxMs = durMs;
    Serial.printf("[Shelly] read origin=%s STEP=HTTP_RESPONSE code=%d DURATION_MS=%lu try=%u/%u rssi=%d\n",
                  origin, code, (unsigned long)durMs, tryNo, maxTries, WiFi.RSSI());
    if (code == 200) {
      payload = h.getString();
      h.end();
      if (tryNo > 1) {
        rt.shellyRetryRecovered++;
        Serial.printf("[Shelly] read origin=%s %s RECOVERED by retry (try %u) total_recovered=%lu\n",
                      origin, (retryCause == HTTPC_ERROR_CONNECTION_REFUSED ? "-1" : "-11"), tryNo, (unsigned long)rt.shellyRetryRecovered);
      }
      break;
    }
    h.end();
    bool retryable = (code == HTTPC_ERROR_READ_TIMEOUT || code == HTTPC_ERROR_CONNECTION_REFUSED);
    if (!retryable || tryNo == maxTries) break; // other errors: no retry
    retryCause = code;
    if (code == HTTPC_ERROR_CONNECTION_REFUSED) {
      // -1: the Shelly refused/closed the connection (overloaded); give it a short delay to free
      // its slot before retrying, rather than an immediate retry that would probably hit the same problem.
      Serial.printf("[Shelly] read origin=%s connection refused (-1) -> retry in %ums\n", origin, SHELLY_RETRY_REFUSED_DELAY_MS);
      delay(SHELLY_RETRY_REFUSED_DELAY_MS);
    } else {
      Serial.printf("[Shelly] read origin=%s timeout (-11) -> immediate retry\n", origin);
    }
  }

  if (code != 200) {
    logNetworkFailure("Shelly", "read_http", code);
    reportError("Shelly: " + shellyHttpDetail(code), ERROR_SHELLY);
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false;
  }

  if (payload.length() == 0) { // 200 received but body not read (timeout while reading the body)
    logNetworkFailure("Shelly", "empty_body", code);
    reportError("Shelly: empty response body", ERROR_SHELLY);
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false;
  }

  JsonDocument d;
  DeserializationError err = deserializeJson(d, payload); 
  if (err || !d.is<JsonObject>()) { 
    reportError("Shelly: invalid JSON (" + String(err ? err.c_str() : "not an object") + ")", ERROR_SHELLY); 
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false; 
  }
  if (!d["output"].is<bool>() || !d["apower"].is<float>()) {
    reportError(String("Shelly: missing field (") + (!d["output"].is<bool>() ? "output" : "apower") + ")", ERROR_SHELLY);
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false;
  }
  rt.shellyConnected = true; 
  rt.shellyOn        = d["output"] | false; 
  rt.shellyW         = (float)(d["apower"] | 0.0);
  // Shelly internal temperature and overheating error (optional fields: their absence is not an error).
  JsonVariant tv = d["temperature"]["tC"];
  rt.shellyT = tv.is<float>() ? tv.as<float>() : NAN;
  bool ot = false;
  if (d["errors"].is<JsonArray>()) {
    for (JsonVariant ev : d["errors"].as<JsonArray>()) {
      if (ev.is<const char *>() && strcmp(ev.as<const char *>(), "overtemp") == 0) ot = true;
    }
  }
  if (ot && !rt.shellyOvertemp) Serial.println("[Shelly] internal OVERHEATING reported by the Shelly (overtemp)");
  rt.shellyOvertemp = ot;
  if (ot) { time_t nowT = time(nullptr); if (nowT >= 946684800) rt.shellyOtEpoch = (uint32_t)nowT; }
  stempFeed(rt.shellyT, rt.shellyW);
  Serial.printf("[Shelly] read origin=%s STEP=STATE_DECODED output=%s apower=%.1fW tC=%.1f\n",
                origin, rt.shellyOn ? "ON" : "OFF", rt.shellyW, isnan(rt.shellyT) ? -99.0f : rt.shellyT);
  recordShellyPollResult(origin, true);
  markErrorResolved(ERROR_SHELLY);
  return true;
}

/* ============================================================
   3. ECOJOKO API
   ============================================================ */

/* ---- Single, persistent Ecojoko TLS client ----
   A single WiFiClientSecure + a single HTTPClient for login, gateways and realtime_conso.
   - setReuse(true): if the server accepts keep-alive, the TLS socket stays open
     between two polls (no more handshake or allocation/release of the mbedtls context on every call).
   - The HTTPClient must be persistent too: its destructor closes the socket.
   - After any error the socket is explicitly closed (never a half-dead socket). */
static WiFiClientSecure &ecoTlsClient() {
  static WiFiClientSecure client;
  static bool configured = false;
  if (!configured) {
    if (ECOJOKO_ROOT_CA[0] != '\0') {
      client.setCACert(ECOJOKO_ROOT_CA);
    } else {
      client.setInsecure();
      Serial.println("[Security] ECOJOKO_ROOT_CA not set: Ecojoko connection WITHOUT certificate verification (setInsecure)");
    }
    client.setHandshakeTimeout(ECO_TLS_HANDSHAKE_TIMEOUT_S); // in seconds
    configured = true;
  }
  return client;
}

static HTTPClient &ecoHttpClient() {
  static HTTPClient https;
  static bool configured = false;
  if (!configured) {
    https.setReuse(true);
    https.setTimeout(ECO_HTTP_TIMEOUT_MS);
    configured = true;
  }
  return https;
}

// Closes the TLS socket (Wi-Fi lost, error, invalid response...). The next call performs a clean handshake again.
static void ecoTlsReset() {
  ecoTlsClient().stop();
}

// End of request. keepAlive=true only after a fully successful request: the socket then stays open
// (if the server allows it). Otherwise it is closed, because a partially read response would make it unusable.
static void ecoTlsEnd(bool keepAlive) {
  ecoHttpClient().end();
  if (!keepAlive) ecoTlsReset();
}

// Sends a request on the persistent client and returns the HTTP code (< 0 = transport error,
// ECO_BEGIN_FAILED = begin() refused). postBody != nullptr => POST (login), otherwise GET with the session cookie.
// The response body is read by the caller, who must always finish with ecoTlsEnd().
// If a REUSED socket was dead (idle connection closed by the server or Wi-Fi), the failure is
// immediate: we retry ONCE right away on a fresh connection (no 8 s timeout to wait for).
static int ecoRequest(const String &url, const String *postBody) {
  HTTPClient &https = ecoHttpClient();
  WiFiClientSecure &client = ecoTlsClient();

  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    bool reused = client.connected();
    if (!https.begin(client, url)) return ECO_BEGIN_FAILED;

    https.addHeader("Content-Type", "application/json");
    if (postBody) {
      static const char *headerKeys = "Set-Cookie";
      https.collectHeaders(&headerKeys, 1);
    } else {
      https.addHeader("Cookie", rt.ecoCookie);
    }

    int code = postBody ? https.POST(*postBody) : https.GET();
    if (code < 0 && reused && attempt == 0) {
      Serial.printf("[Ecojoko] reused TLS socket is dead (code=%d), new connection\n", code);
      ecoTlsEnd(false);
      continue;
    }
    Serial.printf("[Ecojoko] tls reused=%s heapFree=%u largestBlock=%u heapMin=%u\n",
                  reused && attempt == 0 ? "yes" : "no", (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getMaxAllocHeap(), (unsigned)ESP.getMinFreeHeap());
    return code;
  }
  return -1;
}

bool ecoLogin() {
  DiagPhaseScope phase("eco_login");
  Serial.println("[DEBUG] Start of ecoLogin()...");
  rt.ecoCookie = ""; // Never keep a potentially expired cookie after a failed login.
  if (cfg.ecoEmail.length() == 0 || cfg.ecoPassword.length() == 0) {
    Serial.println("[Ecojoko] Credentials missing in the config.");
    return false;
  }

  JsonDocument jsonPayload;
  jsonPayload["l"] = cfg.ecoEmail;
  jsonPayload["p"] = cfg.ecoPassword;
  String payloadStr;
  serializeJson(jsonPayload, payloadStr);

  HTTPClient &https = ecoHttpClient();
  int httpResponseCode = ecoRequest("https://service.ecojoko.com/login", &payloadStr);
  if (httpResponseCode == ECO_BEGIN_FAILED) {
    Serial.println("[Ecojoko] Unable to connect to the Login API.");
    logNetworkFailure("Ecojoko", "login_preparation_http", -1);
    return false;
  }
  Serial.printf("[Ecojoko] Login HTTP: %d\n", httpResponseCode);
  if (httpResponseCode == 200 || httpResponseCode == 201) {
    String cookieHeader = https.header("Set-Cookie");
    if (cookieHeader.length() > 0) {
      int semiColon = cookieHeader.indexOf(';');
      if (semiColon > 0) {
        rt.ecoCookie = cookieHeader.substring(0, semiColon);
      } else {
        rt.ecoCookie = cookieHeader;
      }
    }
  } else {
    Serial.printf("[Ecojoko] Authentication error, HTTP code: %d\n", httpResponseCode);
    logNetworkFailure("Ecojoko", "login_http", httpResponseCode);
  }
  // Login response body not read: we close the socket rather than risk leftover bytes
  // on the persistent connection (login is rare, one more handshake costs nothing).
  ecoTlsEnd(false);
  // The session cookie is a credential: only log whether it was received.
  Serial.printf("[Ecojoko] Session cookie: %s (%u characters)\n",
                rt.ecoCookie.length() ? "received" : "absent", rt.ecoCookie.length());
  return (rt.ecoCookie.length() > 0);
}

static void saveEcoGatewayCache() {
  JsonDocument doc;
  doc["gatewayId"] = rt.gatewayId;
  doc["powerMeterId"] = rt.powerMeterId;

  File out = LittleFS.open("/eco_gateway.tmp", "w");
  if (!out) {
    reportError("Unable to write /eco_gateway.json", ERROR_STORAGE);
    return;
  }

  size_t written = serializeJson(doc, out);
  out.close();
  if (!written || !LittleFS.rename("/eco_gateway.tmp", "/eco_gateway.json")) {
    LittleFS.remove("/eco_gateway.tmp");
    reportError("Failed to write /eco_gateway.json", ERROR_STORAGE);
  }
}

static void restoreEcoGatewayCache() {
  File in = LittleFS.open("/eco_gateway.json", "r");
  if (!in) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, in);
  in.close();
  if (err || !doc.is<JsonObject>() || !doc["gatewayId"].is<const char*>() ||
      !doc["powerMeterId"].is<const char*>()) {
    reportError("Invalid read of /eco_gateway.json", ERROR_STORAGE);
    return;
  }

  String gatewayId = doc["gatewayId"].as<String>();
  String powerMeterId = doc["powerMeterId"].as<String>();
  if (!gatewayId.length() || !powerMeterId.length()) return;

  rt.gatewayId = gatewayId;
  rt.powerMeterId = powerMeterId;
  Serial.println("[Ecojoko] Gateway identifiers restored from the cache");
}

static bool ecoDiscover() {
  DiagPhaseScope phase("eco_gateways");
  rt.gatewayId = "";
  rt.powerMeterId = "";
  if (!rt.ecoCookie.length()) { 
    reportError("No Ecojoko cookie for discovery", ERROR_ECOJOKO);
    return false; 
  }

  HTTPClient &https = ecoHttpClient();
  int httpResponseCode = ecoRequest("https://service.ecojoko.com/gateways", nullptr);
  if (httpResponseCode == ECO_BEGIN_FAILED) { 
    reportError("Unable to reach the gateways API", ERROR_ECOJOKO);
    logNetworkFailure("Ecojoko", "gateways_preparation_http", -1);
    return false; 
  }
  Serial.printf("[Ecojoko] Gateways HTTP: %d\n", httpResponseCode);
  if (httpResponseCode == 200) { 
    String payload = https.getString();
    Serial.println("[Ecojoko] RESPONSE /gateways:");
    Serial.println(payload);
    JsonDocument jsonDoc; 
    DeserializationError error = deserializeJson(jsonDoc, payload);

    if (!error && jsonDoc.is<JsonObject>() && jsonDoc["gateways"].is<JsonArray>() && jsonDoc["gateways"].size() > 0) { 
      Serial.printf("[Ecojoko] gateways found: %u\n", jsonDoc["gateways"].size());
      JsonObject gateway = jsonDoc["gateways"][0];
      JsonVariant gatewayId = gateway["gateway_id"];
      bool gatewayIdIsString = gatewayId.is<const char*>();
      bool gatewayIdIsNumber = gatewayId.is<uint32_t>();
      if (gateway.isNull() || (!gatewayIdIsString && !gatewayIdIsNumber) || !gateway["devices"].is<JsonArray>()) {
        reportError("Incomplete Ecojoko gateway", ERROR_ECOJOKO);
        Serial.println("[Ecojoko] gateway_id or devices missing / invalid type");
      } else {
        // Ecojoko returns gateway_id as a JSON number (e.g. 141907), while
        // readEcojoko() needs it as text to build the request URL.
        rt.gatewayId = gatewayIdIsNumber ? String(gatewayId.as<uint32_t>()) : gatewayId.as<String>();
        Serial.printf("[Ecojoko] gateway_id: %s\n", rt.gatewayId.c_str());

        bool foundDevice = false;
        JsonArray devices = gateway["devices"].as<JsonArray>();
        Serial.printf("[Ecojoko] devices in gateway: %u\n", devices.size());
        for (JsonObject device : devices) { 
          const char* type = device["device_type"] | "<absent>";
          JsonVariant deviceId = device["device_id"];
          bool deviceIdIsString = deviceId.is<const char*>();
          bool deviceIdIsNumber = deviceId.is<uint32_t>();
          String id = (deviceIdIsString || deviceIdIsNumber) ? deviceId.as<String>() : "<absent>";
          Serial.printf("[Ecojoko] device: type=%s id=%s\n", type, id.c_str());
          if (!device.isNull() && device["device_type"].is<const char*>() &&
              (deviceIdIsString || deviceIdIsNumber) &&
              device["device_type"].as<String>() == "POWER_METER") {
            rt.powerMeterId = deviceId.as<String>();
            foundDevice = true;
            break;
          }
        }
        if (!foundDevice || !rt.gatewayId.length() || !rt.powerMeterId.length()) { 
           reportError("Ecojoko meter not found in the gateway", ERROR_ECOJOKO);
           Serial.println("[Ecojoko] No usable POWER_METER");
        } else {
           Serial.printf("[Ecojoko] power_meter_id: %s\n", rt.powerMeterId.c_str());
        }
      }
    } else {
       reportError("Invalid or empty gateways JSON response", ERROR_ECOJOKO);
       Serial.printf("[Ecojoko] Parse gateways: %s; root object=%s; gateways array=%s; size=%u\n",
                     error.c_str(), jsonDoc.is<JsonObject>() ? "yes" : "no",
                     jsonDoc["gateways"].is<JsonArray>() ? "yes" : "no",
                     jsonDoc["gateways"].is<JsonArray>() ? jsonDoc["gateways"].size() : 0);
    }
  } else if (httpResponseCode == 401 || httpResponseCode == 403) { 
     rt.ecoCookie = ""; 
     reportError("Session expired or refused (Ecojoko)", ERROR_ECOJOKO);
  } else {
     reportError("Gateways API error: HTTP " + String(httpResponseCode), ERROR_ECOJOKO);
     logNetworkFailure("Ecojoko", "gateways_http", httpResponseCode);
  }

  bool success = rt.gatewayId.length() > 0 && rt.powerMeterId.length() > 0;
  ecoTlsEnd(success);
  if (success) saveEcoGatewayCache();
  return success;
}

static bool readEcojoko(float& watts) {
  DiagPhaseScope phase("eco_realtime");
  watts = NAN;
  // Login / discovery: we hand control back (no value this tick) but only a REAL failure counts in ecoFailCount,
  // otherwise every successful re-authentication inflated the counter (backoff, diagnostic AP, automatic restart).
  if (!rt.ecoCookie.length()) { 
    if (!ecoLogin()) recordEcoResult(false);
    return false;
  }
  if (!rt.gatewayId.length()) { 
    if (!ecoDiscover()) recordEcoResult(false);
    return false;
  }

  HTTPClient &https = ecoHttpClient();
  String url = "https://service.ecojoko.com/gateway/" + rt.gatewayId + "/device/" + rt.powerMeterId + "/realtime_conso";
  uint32_t tReq = millis();
  int httpResponseCode = ecoRequest(url, nullptr);
  uint32_t dtReq = millis() - tReq;
  if (httpResponseCode == ECO_BEGIN_FAILED) {
    reportError("Unable to reach the Ecojoko API", ERROR_ECOJOKO);
    logNetworkFailure("Ecojoko", "realtime_preparation_http", -1);
    recordEcoResult(false);
    return false;
  }

  if (httpResponseCode == 401 || httpResponseCode == 403) {
     rt.ecoCookie = ""; rt.gatewayId = ""; rt.powerMeterId = "";
     reportError("Session expired or refused on realtime_conso (Ecojoko)", ERROR_ECOJOKO);
     Serial.printf("[Ecojoko] realtime_conso failure: session refused (HTTP %d), cookie/gateway reset\n", httpResponseCode);
     ecoTlsEnd(false); recordEcoResult(false); return false;
  }

  if (httpResponseCode == 200) { 
    String payload = https.getString();
    JsonDocument jsonDoc; 
    DeserializationError error = deserializeJson(jsonDoc, payload);

    if (!error && jsonDoc.is<JsonObject>() && jsonDoc["real_time"].is<JsonObject>()) { 
      JsonObject rt_data = jsonDoc["real_time"];
      if (!rt_data["value"].is<float>()) {
        reportError("Incomplete Ecojoko response", ERROR_ECOJOKO);
      } else {
        watts = rt_data["value"].as<float>();
      }
      if (isnan(watts)) { ecoTlsEnd(false); recordEcoResult(false); return false; } 
    } else {
       reportError("Corrupted Ecojoko JSON", ERROR_ECOJOKO);
    }
  } else {
     // STABLE deduplication key (without dtReq, which varies on every attempt): a prolonged outage is
     // logged as ONE episode with an occurrence counter, not one line per attempt.
     reportError("eco_realtime_http_" + String(httpResponseCode),
                  "Ecojoko read error (HTTP: " + String(httpResponseCode) + ", " + String(dtReq) + " ms)", ERROR_ECOJOKO);
     logNetworkFailure("Ecojoko", "realtime_http", httpResponseCode);
     // We do NOT touch the cookie/gatewayId here: a network error or a
     // transient timeout is not proof that the session has expired.
     // Only a real 401/403 (handled above) must trigger a relogin.
  }
  bool success = !isnan(watts);
  ecoTlsEnd(success);
  recordEcoResult(success);
  if (success) markErrorResolved(ERROR_ECOJOKO);
  return success;
}

/* ============================================================
   4. SAMPLING & CALCULATIONS
   ============================================================ */
void sample() {
  uint32_t now = millis();
  if (cfg.mode != "monitor") {
    float prevShellyW = rt.shellyW;
    bool polled = networkPollDue(rt.lastShellyPollMs, rt.shellyFailCount, now);
    if (polled) {
      rt.lastShellyPollMs = now;
      rt.shellyConnected = readShelly();
      if (rt.shellyConnected) rt.lastShellySuccessMs = now;
    } else {
      // The control loop keeps running; only the failed network poll is delayed.
      rt.shellyConnected = false;
    }
    if (!rt.shellyConnected) {
      uint32_t graceMs = (uint32_t)cfg.ecoGraceSec * 1000UL;
      if (rt.lastShellySuccessMs != 0 && graceMs > 0 && (now - rt.lastShellySuccessMs) < graceMs) {
        // Brief outage: keep the last known Shelly power, no reset.
        rt.shellyW = prevShellyW;
        Serial.printf("[Shelly] data kept (grace) age=%lums limit=%lums\n",
                      (unsigned long)(now - rt.lastShellySuccessMs), (unsigned long)graceMs);
      } else {
        rt.shellyW = NAN;
      }
    }
  }

  float w = NAN;
  if (networkPollDue(rt.lastEcoPollMs, rt.ecoFailCount, now)) {
    rt.lastEcoPollMs = now;
    rt.ecoConnected = readEcojoko(w);
  } else {
    rt.ecoConnected = false;
  }
  uint32_t graceMs = (uint32_t)cfg.ecoGraceSec * 1000UL;
  if (rt.ecoConnected) {
    rt.lastEcoSuccessMs = now;
    rt.ecoRaw = w;
    float a = 1.0f / (cfg.smoothingMin * 12.0f);
    rt.ecoSmooth = isnan(rt.ecoSmooth) ? w : rt.ecoSmooth + a * (w - rt.ecoSmooth);
  } else if (rt.lastEcoSuccessMs != 0 && graceMs > 0 && (now - rt.lastEcoSuccessMs) < graceMs) {
    // Brief outage: keep the last known smoothed value, no reset.
    Serial.printf("[Ecojoko] data kept (grace) age=%lums limit=%lums\n",
                  (unsigned long)(now - rt.lastEcoSuccessMs), (unsigned long)graceMs);
  } else {
    // Prolonged outage (or grace disabled): the data is no longer reliable.
    rt.ecoRaw = NAN;
    rt.ecoSmooth = NAN;
  }

  // Smoothing of the Shelly power with the same filter as ecoSmooth: without it, the Shelly reacts
  // instantly (e.g. thermostat modulating at the end of heating) while ecoSmooth takes several
  // minutes to follow, which distorts restHouse during the transition and causes spurious
  // relay cut-offs/restarts (tier1 wrongly flips) while the solar surplus is still sufficient.
  float shSample = (cfg.mode != "monitor" && rt.shellyOn && !isnan(rt.shellyW)) ? rt.shellyW : 0.0f;
  if (!isnan(rt.ecoSmooth)) {
    float aSh = 1.0f / (cfg.smoothingMin * 12.0f);
    rt.shellySmooth = isnan(rt.shellySmooth) ? shSample : rt.shellySmooth + aSh * (shSample - rt.shellySmooth);
  } else {
    rt.shellySmooth = NAN; // no valid Ecojoko measurement: resynchronise at the next sample
  }
  float shellyP = isnan(rt.shellySmooth) ? 0.0f : rt.shellySmooth;
  rt.restHouse = isnan(rt.ecoSmooth) ? NAN : rt.ecoSmooth - shellyP;
}

/* ============================================================
   5. ARCHIVING & HISTORY (JsonDocument v7)
   ============================================================ */
/* ---------- Storage memory state / guard (LittleFS) ---------- */
static uint32_t storagePurgeCount = 0, storageLastPurgeEpoch = 0, storageLastPurgeTryMs = 0;
static char     storageLastPurgeWhat[48] = {0};
static void diagAppendLog(const String &line); // defined further down (section 7b)

// Effective threshold: 32 KB, reduced to 1/8 of the partition if it is small.
static size_t storageMinFree() {
  size_t eighth = LittleFS.totalBytes() / 8;
  return eighth < STORAGE_MIN_FREE_BYTES ? eighth : STORAGE_MIN_FREE_BYTES;
}

static size_t fsFreeBytes() {
  size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
  return total > used ? total - used : 0;
}

// Atomic JSON write (tmp + rename). If the tmp file does not fit (memory almost full),
// fall back to a direct write: the document is already entirely in RAM.
static bool writeJsonAtomic(const char *path, JsonDocument &doc) {
  String tmp = String(path) + ".tmp";
  File out = LittleFS.open(tmp, "w");
  if (out) {
    size_t written = serializeJson(doc, out);
    out.close();
    if (written && LittleFS.rename(tmp, path)) return true;
    LittleFS.remove(tmp);
  }
  File direct = LittleFS.open(path, "w");
  if (!direct) return false;
  size_t written = serializeJson(doc, direct);
  direct.close();
  return written > 0;
}

// Removes the n oldest entries of doc[key] to keep only keepNewest.
static bool trimJsonArrayFile(const char *path, const char *key, size_t keepNewest) {
  File in = LittleFS.open(path, "r");
  if (!in) return false;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, in);
  in.close();
  if (err || !doc[key].is<JsonArray>()) return false;
  JsonArray arr = doc[key].as<JsonArray>();
  if (arr.size() <= keepNewest) return false;
  while (arr.size() > keepNewest) arr.remove(0);
  return writeJsonAtomic(path, doc);
}

static void removeStaleTmpFiles() {
  String names[6];
  uint8_t n = 0;
  File root = LittleFS.open("/");
  if (!root) return;
  for (File f = root.openNextFile(); f && n < 6; f = root.openNextFile()) {
    String nm = f.name();
    if (!nm.startsWith("/")) nm = "/" + nm;
    if (nm.endsWith(".tmp")) names[n++] = nm;
  }
  root.close();
  for (uint8_t i = 0; i < n; i++) LittleFS.remove(names[i]);
}

/* ---------- Yearly recap: sum of the months of /monthly.json (no dedicated file) ---------- */
static const char *const TOTAL_KEYS_SEC[] = {"heatedSec", "relayOnSec", "tier1Sec", "tier2Sec", "nightSec"};
static const char *const TOTAL_KEYS_F[]   = {"energyKWh", "nativeEUR", "paidKWh", "paidEUR", "savedKWh", "savedEUR"};

// dst += src, field by field (missing field = 0).
static void addTotals(JsonObject dst, JsonObject src) {
  for (const char *k : TOTAL_KEYS_SEC) dst[k] = (uint32_t)(dst[k] | 0) + (uint32_t)(src[k] | 0);
  for (const char *k : TOTAL_KEYS_F)   dst[k] = (float)(dst[k] | 0.0f) + (float)(src[k] | 0.0f);
}

static JsonObject yearEntry(JsonArray years, uint32_t yearKey) {
  for (JsonObject y : years) if ((uint32_t)(y["yearKey"] | 0) == yearKey) return y;
  JsonObject n = years.add<JsonObject>();
  n["yearKey"] = yearKey;
  return n;
}

// Guard: if free memory drops below STORAGE_MIN_FREE_BYTES, purge the history
// in steps (detailed days, then oldest months), stopping as soon as there is enough space.
static void purgeHistoryIfLow() {
  size_t freeBefore = fsFreeBytes();
  if (freeBefore >= storageMinFree()) return;
  uint32_t now = millis();
  if (storageLastPurgeTryMs != 0 && (uint32_t)(now - storageLastPurgeTryMs) < STORAGE_PURGE_RETRY_MS) return;
  storageLastPurgeTryMs = now;

  String done;
  auto enough = []() { return fsFreeBytes() >= storageMinFree(); };
  auto mark = [&](const char *w) { if (done.length()) done += "+"; done += w; };

  removeStaleTmpFiles();
  // 0. heating log: a simple diagnostic tool, the first to sacrifice
  if (!enough() && LittleFS.exists("/heat.json") && LittleFS.remove("/heat.json")) { heatNSeg = 0; relayNIv = 0; mark("log"); }
  // 1. detailed days: the months already summarise them
  if (!enough() && trimJsonArrayFile("/history.json", "days", PURGE_KEEP_DAYS)) mark("days");
  // 2. oldest months (the yearly recap is derived from them: the oldest years get shorter)
  if (!enough() && trimJsonArrayFile("/monthly.json", "months", PURGE_KEEP_MONTHS_1)) mark("months<=24");
  if (!enough() && trimJsonArrayFile("/monthly.json", "months", PURGE_KEEP_MONTHS_2)) mark("months<=3");

  size_t freeAfter = fsFreeBytes();
  if (done.length()) {
    storagePurgeCount++;
    time_t ep = time(nullptr);
    storageLastPurgeEpoch = (ep < 946684800) ? 0 : (uint32_t)ep;
    strncpy(storageLastPurgeWhat, done.c_str(), sizeof(storageLastPurgeWhat) - 1);
    storageLastPurgeWhat[sizeof(storageLastPurgeWhat) - 1] = 0;
    pushErrorLog("Low memory: history purged (" + done + ")", ERROR_STORAGE);
    diagAppendLog("history purge (" + done + ") free " + String((unsigned long)freeBefore) + "->" +
                  String((unsigned long)freeAfter) + " bytes");
  } else {
    pushErrorLog("Low memory: nothing to purge in the history", ERROR_STORAGE);
  }
  Serial.printf("[Storage] purge=%s free %lu -> %lu bytes\n", done.length() ? done.c_str() : "none",
                (unsigned long)freeBefore, (unsigned long)freeAfter);
}

static void fillHistoryEntry(JsonObject o) {
  o["cycleId"]    = rt.cycleId;
  o["heatedSec"]  = rt.heatedSec;
  o["relayOnSec"] = rt.relayOnSec;
  o["tier1Sec"]   = rt.tier1Sec;
  o["tier2Sec"]   = rt.tier2Sec;
  o["nightSec"]   = rt.nightSec;
  // Real energy and cost, accumulated tick by tick in accumulateEnergyCost()
  // from the measured tank power and the real solar surplus -
  // more reliable than an estimate duration x nominal power.
  o["energyKWh"] = rt.cycleWh / 1000.0f;                // total real energy of the tank this cycle
  o["nativeEUR"] = rt.nativeEUR;                         // "natively": cost if everything had been heated in HC
  o["paidKWh"]   = rt.paidWh / 1000.0f;                  // energy actually bought from the grid
  o["paidEUR"]   = rt.paidEUR;                           // real cost
  o["savedKWh"]  = (rt.cycleWh - rt.paidWh) / 1000.0f;   // energy covered by solar (physical quantity)
  o["savedEUR"]  = rt.nativeEUR - rt.paidEUR;            // "saved" = natively - actually paid
  o["mode"]     = cfg.mode;
}

static void saveHistory() {
  purgeHistoryIfLow(); // memory guard: purge the history before writing if free space is low
  JsonDocument doc;
  JsonArray days = doc["days"].to<JsonArray>();
  File in = LittleFS.open("/history.json", "r");
  if (in) {
    JsonDocument tmp;
    DeserializationError err = deserializeJson(tmp, in);
    in.close();
    if (!err && tmp.is<JsonObject>() && tmp["days"].is<JsonArray>()) {
      // Copy while tmp is still alive. A JsonArray is only a non-owning view.
      JsonArray sourceDays = tmp["days"].as<JsonArray>();
      for (JsonObject src : sourceDays) {
        if (src.isNull()) continue;
        JsonObject dst = days.add<JsonObject>();
        dst["cycleId"]    = src["cycleId"] | 0;
        dst["heatedSec"]  = src["heatedSec"] | 0;
        dst["relayOnSec"] = src["relayOnSec"] | 0;
        dst["tier1Sec"]   = src["tier1Sec"] | 0;
        dst["tier2Sec"]   = src["tier2Sec"] | 0;
        dst["nightSec"]   = src["nightSec"] | 0;
        dst["energyKWh"]  = src["energyKWh"] | 0.0f;
        dst["nativeEUR"]  = src["nativeEUR"] | 0.0f;
        dst["paidKWh"]    = src["paidKWh"] | 0.0f;
        dst["paidEUR"]    = src["paidEUR"] | 0.0f;
        dst["savedKWh"]   = src["savedKWh"] | 0.0f;
        dst["savedEUR"]   = src["savedEUR"] | 0.0f;
        dst["mode"]       = src["mode"] | "auto";
      }
    } else if (err) {
      Serial.printf("[History] Invalid JSON ignored: %s\n", err.c_str());
    } else {
      Serial.println("[History] Invalid structure ignored");
    }
  }

  bool found = false;
  for (JsonObject day : days) {
    if ((uint32_t)(day["cycleId"] | 0) == rt.cycleId) { 
      found = true;
      fillHistoryEntry(day);
      break; 
    }
  }
  if (!found) { 
    JsonObject entry = days.add<JsonObject>();
    fillHistoryEntry(entry);
  }

  while (days.size() > 31) days.remove(0);

  File out = LittleFS.open("/history.tmp", "w");
  if (out) {
    size_t written = serializeJson(doc, out);
    out.close();
    if (!written || !LittleFS.rename("/history.tmp", "/history.json")) {
      LittleFS.remove("/history.tmp");
      reportError("Failed to write /history.json", ERROR_STORAGE);
    }
  } else {
    reportError("Unable to write /history.json", ERROR_STORAGE);
  }
  rt.lastHistorySave = millis(); 
}

static void fillMonthlyEntry(JsonObject o, uint32_t monthKey, uint32_t heatedSec, uint32_t relayOnSec,
                              uint32_t tier1Sec, uint32_t tier2Sec, uint32_t nightSec,
                              float cycleWh, float paidWh, float nativeEUR, float paidEUR) {
  o["monthKey"]   = monthKey;
  o["heatedSec"]  = heatedSec;
  o["relayOnSec"] = relayOnSec;
  o["tier1Sec"]   = tier1Sec;
  o["tier2Sec"]   = tier2Sec;
  o["nightSec"]   = nightSec;
  o["energyKWh"]  = cycleWh / 1000.0f;
  o["nativeEUR"]  = nativeEUR;
  o["paidKWh"]    = paidWh / 1000.0f;
  o["paidEUR"]    = paidEUR;
  o["savedKWh"]   = (cycleWh - paidWh) / 1000.0f;
  o["savedEUR"]   = nativeEUR - paidEUR;
}

// Persist the running current-month totals (rt.month*) into /monthly.json,
// keeping past months untouched and capping the archive at MONTHLY_HISTORY_MAX entries.
static void saveMonthly() {
  if (rt.monthKey == 0) return;
  purgeHistoryIfLow();
  JsonDocument doc;
  JsonArray months = doc["months"].to<JsonArray>();
  File in = LittleFS.open("/monthly.json", "r");
  if (in) {
    JsonDocument tmp;
    DeserializationError err = deserializeJson(tmp, in);
    in.close();
    if (!err && tmp.is<JsonObject>() && tmp["months"].is<JsonArray>()) {
      JsonArray sourceMonths = tmp["months"].as<JsonArray>();
      for (JsonObject src : sourceMonths) {
        if (src.isNull()) continue;
        if ((uint32_t)(src["monthKey"] | 0) == rt.monthKey) continue; // replaced by the up-to-date version below
        // Already closed month: re-serialised as is, never recomputed with the
        // current tariffs (otherwise a tariff change would retroactively
        // distort past months).
        JsonObject dst = months.add<JsonObject>();
        dst["monthKey"]   = src["monthKey"]   | 0;
        dst["heatedSec"]  = src["heatedSec"]  | 0;
        dst["relayOnSec"] = src["relayOnSec"] | 0;
        dst["tier1Sec"]   = src["tier1Sec"]   | 0;
        dst["tier2Sec"]   = src["tier2Sec"]   | 0;
        dst["nightSec"]   = src["nightSec"]   | 0;
        dst["energyKWh"]  = src["energyKWh"]  | 0.0f;
        dst["nativeEUR"]  = src["nativeEUR"]  | 0.0f;
        dst["paidKWh"]    = src["paidKWh"]    | 0.0f;
        dst["paidEUR"]    = src["paidEUR"]    | 0.0f;
        dst["savedKWh"]   = src["savedKWh"]   | 0.0f;
        dst["savedEUR"]   = src["savedEUR"]   | 0.0f;
      }
    } else if (err) {
      Serial.printf("[Monthly] Invalid JSON ignored: %s\n", err.c_str());
    }
  }

  JsonObject entry = months.add<JsonObject>();
  fillMonthlyEntry(entry, rt.monthKey, rt.monthHeatedSec, rt.monthRelayOnSec,
                   rt.monthTier1Sec, rt.monthTier2Sec, rt.monthNightSec,
                   rt.monthCycleWh, rt.monthPaidWh, rt.monthNativeEUR, rt.monthPaidEUR);

  while (months.size() > MONTHLY_HISTORY_MAX) months.remove(0);

  File out = LittleFS.open("/monthly.tmp", "w");
  if (out) {
    size_t written = serializeJson(doc, out);
    out.close();
    if (!written || !LittleFS.rename("/monthly.tmp", "/monthly.json")) {
      LittleFS.remove("/monthly.tmp");
      reportError("Failed to write /monthly.json", ERROR_STORAGE);
    }
  } else {
    reportError("Unable to write /monthly.json", ERROR_STORAGE);
  }
}

// Called once when a cycle (day) closes, with that day's final totals, right
// before the day counters reset to 0. Rolls the running month over if needed.
static void accumulateMonth(uint32_t heatedSec, uint32_t relayOnSec, uint32_t tier1Sec,
                             uint32_t tier2Sec, uint32_t nightSec,
                             float cycleWh, float paidWh, float nativeEUR, float paidEUR) {
  uint32_t mk = monthKeyForClosingCycle();
  if (mk == 0) return; // invalid clock: the month cannot be dated, skip this accumulation
  if (rt.monthKey != mk) {
    rt.monthKey = mk;
    rt.monthHeatedSec = rt.monthRelayOnSec = rt.monthTier1Sec = rt.monthTier2Sec = rt.monthNightSec = 0;
    rt.monthCycleWh = rt.monthPaidWh = rt.monthNativeEUR = rt.monthPaidEUR = 0.0f;
  }
  rt.monthHeatedSec  += heatedSec;
  rt.monthRelayOnSec += relayOnSec;
  rt.monthTier1Sec   += tier1Sec;
  rt.monthTier2Sec   += tier2Sec;
  rt.monthNightSec   += nightSec;
  rt.monthCycleWh    += cycleWh;
  rt.monthPaidWh     += paidWh;
  rt.monthNativeEUR  += nativeEUR;
  rt.monthPaidEUR    += paidEUR;
  saveMonthly();
}

static void restoreCurrentHistory() {
  File in = LittleFS.open("/history.json", "r");
  if (!in) return; 
  JsonDocument d; 
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (err || !d.is<JsonObject>()) return;
  JsonArray a = d["days"].as<JsonArray>();
  if (a.isNull()) return;
  for (JsonObject day : a) {
    if (day.isNull()) continue;
    if ((uint32_t)(day["cycleId"] | 0) == rt.cycleId) {
      rt.heatedSec = day["heatedSec"] | 0;
      rt.tier1Sec  = day["tier1Sec"] | 0;
      rt.tier2Sec  = day["tier2Sec"] | 0;
      rt.nightSec  = day["nightSec"] | 0;
      rt.relayOnSec = day["relayOnSec"] | 0;
      rt.cycleWh   = (float)(day["energyKWh"] | 0.0f) * 1000.0f;
      rt.paidWh    = (float)(day["paidKWh"]   | 0.0f) * 1000.0f;
      rt.nativeEUR = day["nativeEUR"] | 0.0f;
      rt.paidEUR   = day["paidEUR"]   | 0.0f;
      break;
    }
  }
}

// Restores the in-progress month's running totals across a reboot, so a
// mid-month restart doesn't wipe out days already committed to /monthly.json.
static void restoreCurrentMonth() {
  File in = LittleFS.open("/monthly.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (err || !d.is<JsonObject>()) return;
  JsonArray a = d["months"].as<JsonArray>();
  if (a.isNull()) return;
  uint32_t mk = monthKeyForToday();
  if (mk == 0) return;
  for (JsonObject m : a) {
    if (m.isNull()) continue;
    if ((uint32_t)(m["monthKey"] | 0) == mk) {
      rt.monthKey        = mk;
      rt.monthHeatedSec  = m["heatedSec"]  | 0;
      rt.monthRelayOnSec = m["relayOnSec"] | 0;
      rt.monthTier1Sec   = m["tier1Sec"]   | 0;
      rt.monthTier2Sec   = m["tier2Sec"]   | 0;
      rt.monthNightSec   = m["nightSec"]   | 0;
      rt.monthCycleWh    = (float)(m["energyKWh"] | 0.0f) * 1000.0f;
      rt.monthPaidWh     = (float)(m["paidKWh"]   | 0.0f) * 1000.0f;
      rt.monthNativeEUR  = m["nativeEUR"] | 0.0f;
      rt.monthPaidEUR    = m["paidEUR"]   | 0.0f;
      break;
    }
  }
}

/* ============================================================
   6. CONTROL STATE MACHINE (TICK)
   ============================================================ */
// Accumulates the real energy and cost of this tick from the
// measured tank power (Shelly apower) and the real solar surplus (restHouse) -
// never from the assumed nominal power (cfg.heaterW). It therefore stays
// correct whatever instantaneous power the tank actually draws
// (e.g. 150-400 W at the end of a heating cycle, when the thermostat modulates).
// - cycleWh/monthCycleWh: total real energy of the tank (whatever the tariff window)
// - paidWh/monthPaidWh   : share of that energy actually bought from the grid (excluding solar surplus)
// - nativeEUR            : "native" cost - as if everything had been heated in HC
// - paidEUR              : real cost, at the rate of the window (HC/HP) in force at each instant
static void accumulateEnergyCost(uint32_t dt, bool hc) {
  lastTickWh = 0.0f; lastTickPaidWh = 0.0f;
  if (!(rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW))) return;
  float ballonW = rt.shellyW;
  float freeW = 0.0f; // share of the tank covered by a real solar surplus at this instant
  if (!isnan(rt.restHouse)) freeW = min(ballonW, max(0.0f, -rt.restHouse));
  float paidW = ballonW - freeW;
  float wh = ballonW * dt / 3600.0f;
  float paidWhTick = paidW * dt / 3600.0f;
  lastTickWh = wh; lastTickPaidWh = paidWhTick;

  rt.cycleWh   += wh;
  rt.paidWh    += paidWhTick;
  rt.nativeEUR += wh / 1000.0f * cfg.hc;
  rt.paidEUR   += paidWhTick / 1000.0f * (hc ? cfg.hc : cfg.hp);
}

/* ---------- Heating log (full mode) ----------
   One segment = continuous heating with ONE reason, written when it stops (> 15 min) or changes:
   HC (off-peak hours), SOLAR (profitable sun: tier1), FORCE (plan catch-up, i.e. heating that would not have happened without it).
   Kept in RAM (HEAT_SEG_MAX) and copied to /heat.json (at most every 10 min, and at every cycle end).
   The segment in progress at the time of a restart is lost, except for a manual checkpoint just before (POST /api/checkpoint). Read: /api/heat. */
static void heatSegClose() {
  if (!heatCurOpen) return;
  heatCurOpen = false; heatCurIdle = 0;
  if (heatCurWh < 1.0f) return;
  HeatSeg s = heatCur;
  s.wh = (uint32_t)fminf(4000000000.0f, fmaxf(0.0f, heatCurWh + 0.5f));
  s.paidWh = (uint32_t)fminf(4000000000.0f, fmaxf(0.0f, heatCurPaid + 0.5f));
  if (heatNSeg >= HEAT_SEG_MAX) {
    memmove(&heatSegs[0], &heatSegs[1], (size_t)(HEAT_SEG_MAX - 1) * sizeof(HeatSeg));
    heatNSeg = HEAT_SEG_MAX - 1;
  }
  heatSegs[heatNSeg++] = s;
  heatDirty = true;
}

/* "Relay" line of the log: state CONFIRMED by the Shelly (rt.shellyOn comes from Switch.GetStatus), not the command sent.
   Only transitions are kept (one interval = start + duration), not one sample per tick.
   Unknown state (Shelly unreachable): the interval is not extended and not closed right away; beyond ANT_GAP_MAX_SEC it is
   closed at the last ON measurement, so a Wi-Fi outage creates neither a false ON nor a false OFF. The interval is also closed at the cycle change (like the heating segments). */
static void relayIvClose() {
  if (!relayCurOpen) return;
  relayCurOpen = false; relayCurUnknown = 0;
  if (relayNIv >= RELAY_IV_MAX) {
    memmove(&relayIvs[0], &relayIvs[1], (size_t)(RELAY_IV_MAX - 1) * sizeof(RelayIv));
    relayNIv = RELAY_IV_MAX - 1;
  }
  relayIvs[relayNIv++] = relayCur;
  heatDirty = true;
}

static void relayLogTick(uint32_t dt) {
  time_t ne = time(nullptr);
  if (ne < 946684800) return;
  if (!rt.shellyConnected) {
    if (relayCurOpen) { relayCurUnknown += dt; if (relayCurUnknown >= ANT_GAP_MAX_SEC) relayIvClose(); }
    return;
  }
  if (rt.shellyOn) {
    if (!relayCurOpen) { relayCur = RelayIv(); relayCur.t0 = (uint32_t)ne - dt; relayCurOpen = true; }
    relayCurUnknown = 0;
    relayCur.sec = (uint32_t)ne - relayCur.t0;
  } else {
    relayIvClose();
  }
}

/* withOpen = true (manual checkpoint, /api/checkpoint): also adds the heating IN PROGRESS as an ordinary segment.
   After a restart it is read back as a finished segment; the heating that resumes afterwards forms a new segment.
   In normal operation (withOpen = false) only finished segments are written. */
static bool heatSaveImpl(bool withOpen) {
  heatDirty = false; heatLastSaveMs = millis();
  purgeHistoryIfLow();
  JsonDocument d;
  JsonArray a = d["segs"].to<JsonArray>();
  bool addOpen = withOpen && heatCurOpen && heatCurWh >= 1.0f;
  // heatLoad() stops at HEAT_SEG_MAX segments: if the log is full, the oldest is left out so the most recent is not lost.
  uint16_t first = (addOpen && heatNSeg >= HEAT_SEG_MAX) ? 1 : 0;
  for (uint16_t i = first; i < heatNSeg; ++i) {
    JsonArray r = a.add<JsonArray>();
    r.add(heatSegs[i].t0); r.add(heatSegs[i].sec); r.add(heatSegs[i].wh); r.add(heatSegs[i].paidWh); r.add(heatSegs[i].reason);
  }
  if (addOpen) {
    JsonArray r = a.add<JsonArray>();
    r.add(heatCur.t0); r.add(heatCur.sec);
    r.add((uint32_t)fminf(4000000000.0f, fmaxf(0.0f, heatCurWh + 0.5f))); r.add((uint32_t)fminf(4000000000.0f, fmaxf(0.0f, heatCurPaid + 0.5f))); r.add(heatCur.reason);
  }
  // "Relay" line: [start epoch, duration s]. With withOpen, the interval in progress is written as a finished interval.
  JsonArray rl = d["rel"].to<JsonArray>();
  bool addRel = withOpen && relayCurOpen;
  uint16_t rfirst = (addRel && relayNIv >= RELAY_IV_MAX) ? 1 : 0;
  for (uint16_t i = rfirst; i < relayNIv; ++i) {
    JsonArray r = rl.add<JsonArray>();
    r.add(relayIvs[i].t0); r.add(relayIvs[i].sec);
  }
  if (addRel) {
    JsonArray r = rl.add<JsonArray>();
    r.add(relayCur.t0); r.add(relayCur.sec);
  }
  bool ok = writeJsonAtomic("/heat.json", d);
  if (!ok) reportError("Failed to write /heat.json", ERROR_STORAGE);
  return ok;
}

static void heatSave() { heatSaveImpl(false); }

static void heatLoad() {
  File in = LittleFS.open("/heat.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (err || !d["segs"].is<JsonArray>()) return;
  heatNSeg = 0;
  for (JsonArray r : d["segs"].as<JsonArray>()) {
    if (heatNSeg >= HEAT_SEG_MAX) break;
    uint32_t t0 = r[0] | 0UL;
    if (t0 < 946684800UL) continue;
    HeatSeg &s = heatSegs[heatNSeg++];
    s.t0 = t0; s.sec = (uint32_t)(r[1] | 0UL); s.wh = (uint32_t)(r[2] | 0UL); s.paidWh = (uint32_t)(r[3] | 0UL);
    s.reason = (uint8_t)(r[4] | 0);
  }
  relayNIv = 0;
  if (d["rel"].is<JsonArray>()) {
    for (JsonArray r : d["rel"].as<JsonArray>()) {
      if (relayNIv >= RELAY_IV_MAX) break;
      uint32_t t0 = r[0] | 0UL, sec = r[1] | 0UL;
      if (t0 < 946684800UL || sec == 0UL || sec > 604800UL) continue; // 7 d: anti-corruption guard, not a physical limit
      relayIvs[relayNIv].t0 = t0; relayIvs[relayNIv].sec = sec; relayNIv++;
    }
  }
}

// Called on every tick of full mode, AFTER accumulateEnergyCost() (whose tick energy it reuses).
static void heatLogTick(uint32_t dt, bool heating, uint8_t reason) {
  time_t ne = time(nullptr);
  if (ne < 946684800) return;
  if (heating) {
    if (heatCurOpen && heatCur.reason != reason) heatSegClose();
    if (!heatCurOpen) {
      heatCur = HeatSeg();
      heatCur.t0 = (uint32_t)ne - dt; heatCur.reason = reason;
      heatCurWh = 0.0f; heatCurPaid = 0.0f; heatCurIdle = 0; heatCurOpen = true;
    }
    heatCurIdle = 0;
    heatCurWh += lastTickWh; heatCurPaid += lastTickPaidWh;
    heatCur.sec = (uint32_t)ne - heatCur.t0;
  } else if (heatCurOpen) {
    heatCurIdle += dt;
    if (heatCurIdle >= HEAT_SEG_GAP_SEC) heatSegClose();
  }
  if (heatDirty && (uint32_t)(millis() - heatLastSaveMs) >= HEAT_SAVE_MIN_MS) heatSave();
}

/* ============================================================
   6a. FORECAST (full mode): learned daily need + weather + planning
   ============================================================
   Principle (tracking page: /ant, JSON details: /api/ant):
   - The tank is seen as a battery. When the thermostat cuts (relay ON but power < thermostatW
     after a heating run), the tank is FULL: this is the only state measurement available without a sensor.
     The energy injected between two "full" events is exactly what was consumed (draw-offs + losses):
     it is the daily need, learned without any additional sensor.
   - The "useful" sun of the coming hours is estimated from the forecast radiation (Open-Meteo) and a
     learned coefficient k (surplus really available to the tank / measured radiation).
   - The forecast outdoor temperature corrects the need (tank losses in the garage) through a learned slope.
   - Night (HC): only the energy the next day's sun will not be able to provide is heated.
   - Day: check before the evening (adjustable time). At that time the tank does NOT need to be full: it must
     keep the reserve plus enough to last the evening until the HC. Otherwise just the shortfall is heated, at the last moment (in HP).
   - Any missing or doubtful data => classic full mode behaviour (complete HC). */

static bool antClockOk() { return time(nullptr) >= 946684800; }

// Useful tank capacity (Wh): energy to go from "out of hot water" to "full" (thermostat cut).
// Direct input (cfg.tankKWh, e.g. litres x 1.163 x (Tmax - cold water T) from the datasheet) or, failing that, estimated from the volume.
static float antCapWh() {
  return cfg.tankKWh > 0.0f ? cfg.tankKWh * 1000.0f : (float)cfg.tankLiters * ANT_TANK_WH_PER_L;
}

static void antSetReason(const char *r) {
  strncpy(ant.reason, r, sizeof(ant.reason) - 1);
  ant.reason[sizeof(ant.reason) - 1] = 0;
}

/* ---------- In-RAM weather access ---------- */
static int antIdx(uint32_t e) {
  if (ant.wxN == 0 || e < ant.wxStart) return -1;
  uint32_t i = (e - ant.wxStart) / 3600UL;
  return i < ant.wxN ? (int)i : -1;
}
static float antRadAt(uint32_t e) {
  int i = antIdx(e);
  return i < 0 ? NAN : (float)ant.wxRad[i];
}
static float antTempAt(uint32_t e) {
  int i = antIdx(e);
  return (i < 0 || ant.wxT10[i] == INT16_MIN) ? NAN : (float)ant.wxT10[i] / 10.0f;
}
static bool antWxFresh(uint32_t ne) {
  return ant.wxN > 0 && ne >= ant.wxEpoch && (ne - ant.wxEpoch) < ANT_WX_MAX_AGE_S;
}
static bool antCovers(uint32_t a, uint32_t b) {
  return ant.wxN > 0 && a >= ant.wxStart && b <= ant.wxStart + (uint32_t)ant.wxN * 3600UL;
}

/* ---------- Useful sun ----------
   An hour is "useful" when the forecast surplus makes heating CLEARLY cheaper than the HC:
   real cost per kWh = (heaterW - surplus) / heaterW * hp, kept if <= ANT_BENEFIT * hc.
   (Stricter than tier1: avoiding the HC for barely profitable sun, then having to catch up in HP, costs more.) */
static float antThresholdW() {
  float r = (cfg.hp > 0.0f) ? 1.0f - ANT_BENEFIT * cfg.hc / cfg.hp : 0.5f;
  r = fminf(0.95f, fmaxf(0.05f, r));
  return cfg.heaterW * r;
}
// Shading factor applied to the DECISION for the slot starting at epoch e: 1.0 unless enabled AND learned for that hour.
static float antShadeApply(uint32_t e) {
  if (!cfg.antShade) return 1.0f;
  uint8_t h = (uint8_t)((e / 3600UL) % 24UL);
  return antShadeOk[h] ? antShadeF[h] : 1.0f;
}
static bool antHourUseful(int i) {
  return ant.kValid && (ant.k * antShadeApply(ant.wxStart + (uint32_t)i * 3600UL) * (float)ant.wxRad[i]) >= antThresholdW();
}
// Seconds of useful sun in [a, b[.
static uint32_t antUsefulSec(uint32_t a, uint32_t b) {
  if (!ant.kValid || ant.wxN == 0 || b <= a) return 0;
  uint32_t total = 0;
  uint32_t from = a < ant.wxStart ? ant.wxStart : a;
  for (uint32_t i = (from - ant.wxStart) / 3600UL; i < ant.wxN; ++i) {
    uint32_t hs = ant.wxStart + i * 3600UL, he = hs + 3600UL;
    if (hs >= b) break;
    uint32_t lo = hs > a ? hs : a, hi = he < b ? he : b;
    if (hi > lo && antHourUseful((int)i)) total += hi - lo;
  }
  return total;
}
// First instant of useful sun in [a, limit[ (0 if none).
static uint32_t antFirstUseful(uint32_t a, uint32_t limit) {
  if (!ant.kValid || ant.wxN == 0) return 0;
  uint32_t from = a < ant.wxStart ? ant.wxStart : a;
  for (uint32_t i = (from - ant.wxStart) / 3600UL; i < ant.wxN; ++i) {
    uint32_t hs = ant.wxStart + i * 3600UL;
    if (hs >= limit) break;
    if (antHourUseful((int)i)) return hs > a ? hs : a;
  }
  return 0;
}

/* ---------- Learning: daily need, temperature slope, solar coefficient ---------- */
static float antMedian(float *v, uint8_t n) {
  for (uint8_t i = 1; i < n; ++i) {
    float x = v[i];
    int j = (int)i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; --j; }
    v[j + 1] = x;
  }
  return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

// Need (Wh / 24 h) of a cycle, or < 0 if it is not usable (not enough measured duration).
static float antRecNeed(const AntRec &r) {
  return (r.sec >= ANT_MIN_COVER_SEC && r.wh > 0.0f) ? r.wh * 86400.0f / (float)r.sec : -1.0f;
}

static float antMean(const float *v, uint8_t n) {
  float s = 0.0f;
  for (uint8_t i = 0; i < n; ++i) s += v[i];
  return s / (float)n;
}

// Base need (Wh / 24 h). Puts in *tBaseOut (may be nullptr) the average temperature of the cycles used
// to compute this need (same mix of last 14 cycles / same weekday), or NAN if too few
// temperatures are known: the temperature correction is then applied relative to THAT reference.
static float antNeedBase(uint8_t wdayTarget, float *tBaseOut) {
  float all[14], same[6], tAll[14], tSame[6];
  uint8_t na = 0, ns = 0, nta = 0, nts = 0;
  for (int i = (int)antNRec - 1; i >= 0; --i) {
    float n = antRecNeed(antRecs[i]);
    if (n < 0.0f) continue;
    bool hasT = antRecs[i].t10 != INT16_MIN;
    float t = hasT ? (float)antRecs[i].t10 / 10.0f : 0.0f;
    if (na < 14) { all[na++] = n; if (hasT) tAll[nta++] = t; }
    if (ns < 6 && antRecs[i].wday == wdayTarget) { same[ns++] = n; if (hasT) tSame[nts++] = t; }
  }
  if (tBaseOut) *tBaseOut = NAN;
  if (na == 0) return 0.0f;
  float base = antMedian(all, na);
  float tBase = nta >= 3 ? antMean(tAll, nta) : NAN;
  if (ns >= 2) {
    base = 0.5f * base + 0.5f * antMedian(same, ns); // weekdays / weekend
    if (!isnan(tBase) && nts >= 2) tBase = 0.5f * tBase + 0.5f * antMean(tSame, nts);
  }
  if (tBaseOut) *tBaseOut = tBase;
  return base;
}

static void antRefresh() {
  float xs[ANT_REC_MAX], ys[ANT_REC_MAX];
  uint8_t nq = 0, nt = 0;
  float tmin = 1e9f, tmax = -1e9f;
  for (int i = (int)antNRec - 1; i >= 0; --i) {
    float n = antRecNeed(antRecs[i]);
    if (n < 0.0f) continue;
    ++nq;
    if (antRecs[i].t10 != INT16_MIN) {
      float t = (float)antRecs[i].t10 / 10.0f;
      xs[nt] = t; ys[nt] = n; ++nt;
      tmin = fminf(tmin, t); tmax = fmaxf(tmax, t);
    }
  }
  ant.needN = nq;
  ant.slopeValid = false; ant.slope = 0.0f;
  if (nt >= ANT_MIN_REG_DAYS && (tmax - tmin) >= ANT_MIN_REG_SPREAD) {
    float mx = 0.0f, my = 0.0f;
    for (uint8_t i = 0; i < nt; ++i) { mx += xs[i]; my += ys[i]; }
    mx /= nt; my /= nt;
    float cov = 0.0f, var = 0.0f;
    for (uint8_t i = 0; i < nt; ++i) { cov += (xs[i] - mx) * (ys[i] - my); var += (xs[i] - mx) * (xs[i] - mx); }
    if (var > 1.0f) {
      float s = cov / var;
      if (s < 0.0f) { ant.slope = fmaxf(s, -ANT_MAX_SLOPE); ant.slopeValid = true; ant.tRef = mx; }
    }
  }
  float sumRad = 0.0f, sumPot = 0.0f;
  uint8_t nk = 0;
  for (int i = (int)antNRec - 1; i >= 0 && nk < 21; --i) {
    if (antRecs[i].rad <= 0.0f) continue;
    sumRad += antRecs[i].rad; sumPot += antRecs[i].pot; ++nk;
  }
  ant.kN = nk; ant.kSumRad = sumRad;
  ant.kLearn = sumRad > 0.0f ? sumPot / sumRad : 0.0f;
  if (nk >= 3 && sumRad >= ANT_MIN_K_RAD) {
    // Enough learning: learned coefficient only (original behaviour).
    ant.kValid = true; ant.k = ant.kLearn; ant.kW = 1.0f;
  } else if (cfg.pvKwc > 0.0f) {
    // Starting coefficient, progressively replaced by the learned one: the weight of the learned one follows the
    // measured cumulated radiation (up to ANT_MIN_K_RAD) and the number of days (up to 3). Continuous at the switch-over.
    float w = fminf(1.0f, sumRad / ANT_MIN_K_RAD) * fminf(1.0f, (float)nk / 3.0f);
    ant.kValid = true; ant.kW = w;
    ant.k = (1.0f - w) * (cfg.pvKwc * ANT_PV_RATIO) + w * ant.kLearn;
  } else {
    ant.kValid = false; ant.k = 0.0f; ant.kW = 0.0f; // no starting coefficient: original behaviour
  }
}

static float antErrRatio() { return ant.errPred > 100.0f ? ant.errAct / ant.errPred : 1.0f; }

// Expected need over the next 24 h (Wh), corrected by the forecast temperature.
static float antNeedDayAt(uint32_t ne) {
  time_t t = (time_t)(ne + 12UL * 3600UL); // weekday of the middle of the window
  tm lt;
  localtime_r(&t, &lt);
  float tBase;
  float base = antNeedBase((uint8_t)lt.tm_wday, &tBase);
  ant.needBase = base;
  ant.tBase = tBase;
  float need = base;
  if (ant.slopeValid && base > 0.0f) {
    float s = 0.0f;
    uint8_t n = 0;
    for (uint32_t e = ne; e < ne + 24UL * 3600UL; e += 3600UL) {
      float tt = antTempAt(e);
      if (!isnan(tt)) { s += tt; ++n; }
    }
    if (n >= 12) {
      need = base + ant.slope * (s / (float)n - (isnan(tBase) ? ant.tRef : tBase));
      need = fminf(2.0f * base, fmaxf(0.5f * base, need));
    }
  }
  return need;
}

/* ---------- Shading by hour of the day (learned from the hourly forecast / actual) ----------
   A tree shadow comes back at the same sun position, a cloud does not. For each hour of the day, ratio = actual / forecast
   over the last ANT_SHADE_N usable days: the median wipes out the clouds. Factor = hourly median / reference (median of
   the hourly medians), so the overall level (already carried by k) is not counted twice; clamped to [0.3, 1.2].
   Samples are taken against the RAW forecast (antHr.p), never the shaded one: otherwise enabling the factor would feed
   its own learning. Learned and displayed all the time; only used by the decision if cfg.antShade. /ant_shade.json. */
static void antShadeRecompute() {
  float med[24], tmp[ANT_SHADE_N], lvl[24];
  uint8_t nv = 0;
  for (uint8_t h = 0; h < 24; ++h) {
    antShadeOk[h] = false;
    if (antShadeCnt[h] < ANT_SHADE_MIN_N) continue;
    for (uint8_t j = 0; j < antShadeCnt[h]; ++j) tmp[j] = (float)antShadeS[h][j];
    med[h] = antMedian(tmp, antShadeCnt[h]) / 100.0f;
    antShadeOk[h] = true; // provisional: confirmed below
    lvl[nv++] = med[h];
  }
  antShadeRef = NAN;
  float ref = nv >= ANT_SHADE_MIN_H ? antMedian(lvl, nv) : 0.0f;
  if (!(ref > 0.05f)) { for (uint8_t h = 0; h < 24; ++h) antShadeOk[h] = false; return; }
  antShadeRef = ref;
  for (uint8_t h = 0; h < 24; ++h)
    if (antShadeOk[h]) antShadeF[h] = fminf(ANT_SHADE_F_MAX, fmaxf(ANT_SHADE_F_MIN, med[h] / ref));
}

// Called when an hour is closed: r = actual (W), p = raw forecast frozen at the start of the cycle (W).
static void antShadeAdd(uint32_t hh, uint16_t r, uint16_t p) {
  if (r == 0xFFFF || p == 0xFFFF || !((float)p >= ANT_SHADE_MIN_P * cfg.heaterW)) return;
  uint8_t h = (uint8_t)(hh % 24UL);
  float q = 100.0f * (float)r / (float)p;
  antShadeS[h][antShadeHead[h]] = (uint8_t)(q > 250.0f ? 250 : (int)(q + 0.5f));
  antShadeHead[h] = (uint8_t)((antShadeHead[h] + 1) % ANT_SHADE_N);
  if (antShadeCnt[h] < ANT_SHADE_N) antShadeCnt[h]++;
  antShadeRecompute();
}

// File: {"s":[[ratios of UTC hour 0, oldest first],[hour 1],...]}. Written with the hourly ring (cycle change, manual checkpoint).
static void antShadeSave() {
  JsonDocument d;
  JsonArray a = d["s"].to<JsonArray>();
  for (uint8_t h = 0; h < 24; ++h) {
    JsonArray x = a.add<JsonArray>();
    for (uint8_t k = 0; k < antShadeCnt[h]; ++k)
      x.add(antShadeS[h][antShadeCnt[h] < ANT_SHADE_N ? k : (antShadeHead[h] + k) % ANT_SHADE_N]);
  }
  if (!writeJsonAtomic("/ant_shade.json", d)) reportError("Failed to write /ant_shade.json", ERROR_STORAGE);
}

static void antShadeLoad() {
  File in = LittleFS.open("/ant_shade.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (err || !d["s"].is<JsonArray>()) return;
  uint8_t h = 0;
  for (JsonVariantConst xv : d["s"].as<JsonArrayConst>()) {
    if (h >= 24) break;
    uint8_t n = 0;
    if (xv.is<JsonArrayConst>()) {
      for (JsonVariantConst v : xv.as<JsonArrayConst>()) {
        int q = v | -1;
        if (q < 0 || q > 250 || n >= ANT_SHADE_N) continue;
        antShadeS[h][n++] = (uint8_t)q;
      }
    }
    antShadeCnt[h] = n; antShadeHead[h] = (uint8_t)(n % ANT_SHADE_N);
    h++;
  }
  antShadeRecompute();
}

/* ---------- Forecast / actual solar PER HOUR (display only: no effect on the decision) ----------
   Ring of ANT_HR_N slots. Actual: average usable surplus (min(heaterW, -restHouse)), same definition as "pot".
   Forecast: min(heaterW, k x radiation) computed in antCapturePrediction (forecast frozen at the start of the cycle).
   Persisted in /ant_hr.json once a day (cycle change) and at the manual checkpoint: no extra flash wear. */
static uint16_t antHrIdx(uint32_t hh) {
  uint16_t i = (uint16_t)(hh % ANT_HR_N);
  if (antHr[i].h != hh) { antHr[i].h = hh; antHr[i].r = 0xFFFF; antHr[i].p = 0xFFFF; } // slot of an older hour: recycled
  return i;
}

static uint16_t antHrW(float w) { return w <= 0.0f ? 0 : (w >= 65000.0f ? 65000 : (uint16_t)(w + 0.5f)); }

// Closes the current hour: writes the average actual value if the hour was measured enough.
static void antHrClose() {
  if (antHrCur != 0 && antHrSec >= ANT_HR_MIN_SEC) {
    AntHr &x = antHr[antHrIdx(antHrCur)];
    x.r = antHrW(antHrWh * 3600.0f / (float)antHrSec);
    antShadeAdd(antHrCur, x.r, x.p); // shading statistic (against the raw forecast)
  }
  antHrWh = 0.0f; antHrSec = 0;
}

// Called on every tick with the instantaneous usable surplus (W); hh = current epoch hour.
static void antHrAccumulate(uint32_t hh, float usableW, uint32_t dt) {
  if (hh != antHrCur) { antHrClose(); antHrCur = hh; }
  antHrWh += usableW * (float)dt / 3600.0f;
  antHrSec += dt;
}

// doc["hr"] = {h:[epoch hours], r:[actual W], p:[forecast W]} (-1 = unknown): same format for the file, the raw export and the import.
static void antHrFillDoc(JsonDocument &d) {
  JsonObject o = d["hr"].to<JsonObject>();
  JsonArray ha = o["h"].to<JsonArray>(), ra = o["r"].to<JsonArray>(), pa = o["p"].to<JsonArray>();
  for (uint16_t i = 0; i < ANT_HR_N; ++i) {
    const AntHr &x = antHr[i];
    if (x.h == 0 || (x.r == 0xFFFF && x.p == 0xFFFF)) continue;
    ha.add(x.h); ra.add(x.r == 0xFFFF ? -1 : (int)x.r); pa.add(x.p == 0xFFFF ? -1 : (int)x.p);
  }
}

static void antHrSave() {
  JsonDocument d;
  antHrFillDoc(d);
  if (!writeJsonAtomic("/ant_hr.json", d)) reportError("Failed to write /ant_hr.json", ERROR_STORAGE);
}

// Merge without overwriting (file read at boot, habits import): a more recent slot is kept,
// an already known value is not replaced. Returns the number of slots touched.
static int antHrMerge(JsonVariantConst hv) {
  if (!hv["h"].is<JsonArrayConst>() || !hv["r"].is<JsonArrayConst>() || !hv["p"].is<JsonArrayConst>()) return 0;
  JsonArrayConst ha = hv["h"].as<JsonArrayConst>(), ra = hv["r"].as<JsonArrayConst>(), pa = hv["p"].as<JsonArrayConst>();
  size_t n = min(ha.size(), min(ra.size(), pa.size()));
  int touched = 0;
  for (size_t j = 0; j < n; ++j) {
    uint32_t hh = (uint32_t)(ha[j] | 0UL);
    int r = ra[j] | -1, p = pa[j] | -1;
    if (hh < 262968UL) continue; // before 2000: invalid
    AntHr &x = antHr[hh % ANT_HR_N];
    if (x.h != hh) { if (x.h > hh) continue; x.h = hh; x.r = 0xFFFF; x.p = 0xFFFF; }
    if (x.r == 0xFFFF && r >= 0 && r <= 65000) x.r = (uint16_t)r;
    if (x.p == 0xFFFF && p >= 0 && p <= 65000) x.p = (uint16_t)p;
    touched++;
  }
  return touched;
}

static void antHrLoad() {
  File in = LittleFS.open("/ant_hr.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (!err) antHrMerge(d["hr"]);
}

/* ---------- Forecast frozen at the start of the cycle (to compare it with the actual) ----------
   Taken only once per cycle, within the 3 h after the end of the HC (after a restart in the middle of a cycle nothing is
   taken: the cycle would be incomplete anyway). Two values, each over 24 h:
   - expected need (Wh), to compare with the need measured over the cycle;
   - forecast usable solar surplus (Wh) = hourly sum of min(heaterW, k x forecast radiation), to compare with pot.
   No effect on the decision: needBase / tBase (written by antNeedDayAt) are restored. */
static void antCapturePrediction(uint32_t ne) {
  if (ant.cycPredWh >= 0.0f && ant.cycPredSolWh >= 0.0f) return;
  int sinceHcEnd = ((int)nowMin() - (int)cfg.hcEnd + 1440) % 1440;
  if (sinceHcEnd >= 180) return;
  if (!antWxFresh(ne)) return;
  if (ant.cycPredWh < 0.0f && ant.needN >= ANT_MIN_NEED_DAYS) {
    float sb = ant.needBase, st = ant.tBase;
    float need = antNeedDayAt(ne);
    ant.needBase = sb; ant.tBase = st;
    if (need > 0.0f) ant.cycPredWh = need;
  }
  if (ant.cycPredSolWh < 0.0f && ant.kValid) {
    uint32_t h0 = ne - ne % 3600UL;
    if (antCovers(h0, h0 + 24UL * 3600UL)) {
      float sol = 0.0f;
      for (uint8_t i = 0; i < 24; ++i) {
        float rd = antRadAt(h0 + (uint32_t)i * 3600UL);
        if (!isnan(rd)) {
          float w = fminf(cfg.heaterW, ant.k * rd);
          sol += fminf(cfg.heaterW, ant.k * antShadeApply(h0 + (uint32_t)i * 3600UL) * rd); // with shading if enabled
          antHr[antHrIdx(h0 / 3600UL + i)].p = antHrW(w); // hourly forecast, frozen like the cycle forecast (RAW: base of the shading statistic)
        }
      }
      ant.cycPredSolWh = sol;
    }
  }
}

/* ---------- Persistence: /ant.json (one record per cycle, written once a day) ---------- */
static void antFillRecsDoc(JsonDocument &d) {
  JsonArray a = d["recs"].to<JsonArray>();
  for (uint8_t i = 0; i < antNRec; ++i) {
    const AntRec &r = antRecs[i];
    JsonObject o = a.add<JsonObject>();
    o["c"] = r.cycle; o["w"] = r.wday; o["s"] = r.sec;
    o["wh"] = r.wh; o["r"] = r.rad; o["p"] = r.pot; o["sk"] = r.skipWh;
    if (r.t10 != INT16_MIN) o["t"] = r.t10;
    if (r.predWh >= 0.0f)    o["pn"] = r.predWh;    // need forecast (Wh), absent if unknown
    if (r.predSolWh >= 0.0f) o["ps"] = r.predSolWh; // solar surplus forecast (Wh), absent if unknown
  }
}

static void antSave() {
  JsonDocument d;
  antFillRecsDoc(d);
  if (!writeJsonAtomic("/ant.json", d)) reportError("Failed to write /ant.json", ERROR_STORAGE);
}

static void antLoad() {
  File in = LittleFS.open("/ant.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (err || !d["recs"].is<JsonArray>()) return;
  antNRec = 0;
  for (JsonObject o : d["recs"].as<JsonArray>()) {
    if (antNRec >= ANT_REC_MAX) break;
    AntRec &r = antRecs[antNRec++];
    r.cycle  = (uint32_t)(o["c"] | 0);
    r.wday   = (uint8_t)(o["w"] | 0);
    r.sec    = (uint32_t)(o["s"] | 0);
    r.wh     = (float)(o["wh"] | 0.0f);
    r.rad    = (float)(o["r"] | 0.0f);
    r.pot    = (float)(o["p"] | 0.0f);
    r.skipWh = (float)(o["sk"] | 0.0f);
    r.t10    = o["t"].is<int>() ? (int16_t)o["t"].as<int>() : INT16_MIN;
    r.predWh    = (float)(o["pn"] | -1.0f);
    r.predSolWh = (float)(o["ps"] | -1.0f);
    if (!(r.predWh >= 0.0f))    r.predWh = -1.0f;    // includes NaN
    if (!(r.predSolWh >= 0.0f)) r.predSolWh = -1.0f;
  }
  antRefresh();
}

// Called at the cycle change, before the day counters are reset.
static void antCycleClose(uint32_t closingCycle) {
  if (cfg.antMode == "off") return;
  tm lt;
  uint8_t wd = 0;
  if (localNow(lt)) wd = (uint8_t)((lt.tm_wday + 6) % 7); // the cycle that is closing is yesterday's
  AntRec r;
  r.cycle = closingCycle; r.wday = wd; r.sec = ant.cycMeasSec; r.wh = ant.cycMeasWh;
  r.rad = ant.cycRadWh; r.pot = ant.cycPotWh; r.skipWh = ant.cycSkipWh;
  r.t10 = ant.cycTSec >= 3600UL ? (int16_t)(ant.cycTSum10 / (int32_t)ant.cycTSec) : INT16_MIN;
  r.predWh = ant.cycPredWh; r.predSolWh = ant.cycPredSolWh;
  if (antNRec >= ANT_REC_MAX) {
    for (uint8_t i = 1; i < ANT_REC_MAX; ++i) antRecs[i - 1] = antRecs[i];
    antNRec = ANT_REC_MAX - 1;
  }
  antRecs[antNRec++] = r;
  ant.cycMeasWh = 0.0f; ant.cycMeasSec = 0; ant.cycRadWh = 0.0f; ant.cycPotWh = 0.0f;
  ant.cycSkipWh = 0.0f; ant.cycTSum10 = 0; ant.cycTSec = 0;
  ant.cycPredWh = -1.0f; ant.cycPredSolWh = -1.0f;
  antSave();
  antHrSave(); // forecast / actual per hour (8-day ring)
  antShadeSave(); // shading statistic (per hour of the day)
  antRefresh();
}

/* ---------- MANUAL checkpoint of the cycle counters (/ant_cyc.json) ----------
   Written only by the "Save now" button (no regular checkpoint: no flash wear). Read back once
   at boot then DELETED. Ignored if it dates from another cycle. The counters (measurement, radiation, surplus,
   temperature, frozen forecast, model/actual gap) are restored as they are; the "tank full" state (measurement reference)
   is restored only if the stop lasted at most ANT_GAP_MAX_SEC, otherwise measurement resumes at the next "full". */
static bool antCycSave() {
  if (cfg.antMode == "off" || rt.cycleId == 0) return false;
  uint32_t ne = (uint32_t)time(nullptr);
  if (ne < 946684800UL) return false;
  JsonDocument d;
  d["cycle"] = rt.cycleId; d["at"] = ne;
  d["mw"] = ant.cycMeasWh; d["ms"] = ant.cycMeasSec; d["rw"] = ant.cycRadWh; d["pw"] = ant.cycPotWh; d["sk"] = ant.cycSkipWh;
  d["ts"] = ant.cycTSum10; d["tn"] = ant.cycTSec;
  d["pn"] = ant.cycPredWh; d["ps"] = ant.cycPredSolWh;
  d["ea"] = ant.errAct; d["ep"] = ant.errPred; d["lf"] = ant.lastFullEpoch;
  if (ant.refValid && !ant.refBad) { d["re"] = ant.refEpoch; d["wh"] = ant.whSince; }
  bool ok = writeJsonAtomic("/ant_cyc.json", d);
  antHrSave(); // the hourly ring follows the same rhythm (manual button / controlled restart)
  antShadeSave();
  return ok;
}

static bool antCycRestored = false; // a single attempt per boot

static void antCycRestore(uint32_t ne) {
  File in = LittleFS.open("/ant_cyc.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  LittleFS.remove("/ant_cyc.json"); // single use: the file is cleaned up in all cases
  if (err) return;
  uint32_t at = (uint32_t)(d["at"] | 0UL);
  if ((uint32_t)(d["cycle"] | 0UL) != rt.cycleId || at == 0 || at > ne) { Serial.println("[Forecast] checkpoint ignored (other cycle)"); return; }
  float mw = d["mw"] | 0.0f, rw = d["rw"] | 0.0f, pw = d["pw"] | 0.0f, sk = d["sk"] | 0.0f;
  float pn = d["pn"] | -1.0f, ps = d["ps"] | -1.0f, ea = d["ea"] | 0.0f, ep = d["ep"] | 0.0f;
  uint32_t ms = (uint32_t)(d["ms"] | 0UL), tn = (uint32_t)(d["tn"] | 0UL);
  int32_t ts = (int32_t)(d["ts"] | 0);
  bool ok = std::isfinite(mw) && mw >= 0.0f && mw <= 1.0e6f && std::isfinite(rw) && rw >= 0.0f && rw <= 50000.0f &&
            std::isfinite(pw) && pw >= 0.0f && pw <= 1.0e6f && std::isfinite(sk) && sk >= 0.0f && sk <= 1.0e6f &&
            std::isfinite(ea) && ea >= 0.0f && ea <= 1.0e9f && std::isfinite(ep) && ep >= 0.0f && ep <= 1.0e9f &&
            ms <= 200000UL && tn <= 200000UL && (int64_t)labs((long)ts) <= (int64_t)600 * (int64_t)tn + 1;
  if (!ok) { Serial.println("[Forecast] invalid checkpoint, ignored"); return; }
  ant.cycMeasWh = mw; ant.cycMeasSec = ms; ant.cycRadWh = rw; ant.cycPotWh = pw; ant.cycSkipWh = sk;
  ant.cycTSum10 = ts; ant.cycTSec = tn;
  ant.cycPredWh    = (std::isfinite(pn) && pn >= 0.0f && pn <= 1.0e6f) ? pn : -1.0f;
  ant.cycPredSolWh = (std::isfinite(ps) && ps >= 0.0f && ps <= 1.0e6f) ? ps : -1.0f;
  ant.errAct = ea; ant.errPred = ep;
  uint32_t lf = (uint32_t)(d["lf"] | 0UL);
  if (lf <= ne) ant.lastFullEpoch = lf;
  uint32_t re = (uint32_t)(d["re"] | 0UL);
  bool refBack = false;
  if (re > 0 && re <= ne && (ne - at) <= ANT_GAP_MAX_SEC && (ne - re) <= ANT_REF_MAX_H * 3600UL) {
    float wh = d["wh"] | 0.0f;
    if (std::isfinite(wh) && wh >= 0.0f && wh <= 1.0e6f) {
      ant.refEpoch = re; ant.whSince = wh; ant.gapSec = 0; ant.refBad = false; ant.refValid = true; ant.wasHeating = false;
      refBack = true;
    }
  }
  Serial.printf("[Forecast] checkpoint restored: measurement=%lus, downtime=%lus, \"full\" reference=%s\n",
                (unsigned long)ms, (unsigned long)(ne - at), refBack ? "restored" : "to redo");
}

/* ---------- "Tank full" detection ---------- */
static void antOnFull(uint32_t ne) {
  if (ant.refValid && !ant.refBad && ne > ant.refEpoch) {
    uint32_t sec = ne - ant.refEpoch;
    if (sec >= 60UL && sec <= ANT_REF_MAX_H * 3600UL) {
      ant.cycMeasWh += ant.whSince;
      ant.cycMeasSec += sec;
      if (ant.needN >= ANT_MIN_NEED_DAYS && sec >= 900UL) {
        // Model / reality gap: sum(actual) / sum(forecast) over the last "full" events (decaying weights).
        float pred = (ant.needBase / 24.0f) * (float)sec / 3600.0f;
        ant.errAct  = ant.errAct  * 0.95f + ant.whSince;
        ant.errPred = ant.errPred * 0.95f + pred;
      }
    }
  }
  ant.refEpoch = ne; ant.whSince = 0.0f; ant.gapSec = 0; ant.refBad = false; ant.refValid = true;
  ant.lastFullEpoch = ne;
  Serial.printf("[Forecast] tank full detected time=%s\n", nowText().c_str());
}

/* ---------- Planner ---------- */
static uint32_t antSecondsUntilMinute(uint16_t target) { // next occurrence of this minute of the day (1..86400 s)
  tm lt;
  if (!localNow(lt)) return 0;
  int nowSec = lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec;
  int d = (int)target * 60 - nowSec;
  if (d <= 0) d += 86400;
  return (uint32_t)d;
}

static bool antDeadlineOk() {
  return cfg.hcStart > cfg.hcEnd && cfg.deadlineMin > cfg.hcEnd && cfg.deadlineMin < cfg.hcStart;
}

static void antPlan(uint32_t ne) {
  ant.valid = false; ant.heat = true; ant.force = false;
  ant.dNowWh = 0.0f; ant.hWh = 0.0f; ant.erechWh = 0.0f; ant.tsEpoch = 0; ant.tdEpoch = 0;
  if (cfg.antMode == "off")  { antSetReason("off"); return; }
  if (!antClockOk())         { antSetReason("clock"); return; }
  if (!ant.refValid)         { antSetReason("tank: unknown state"); ant.forcing = false; ant.heatPlan = false; return; }
  if (!antWxFresh(ne))       { antSetReason("weather missing"); return; }
  if (ant.needN < ANT_MIN_NEED_DAYS) { antSetReason("learning need"); return; }
  if (!ant.kValid)           { antSetReason("learning solar"); return; }
  if (!antDeadlineOk())      { antSetReason("invalid deadline"); return; }
  float capWh = antCapWh();
  if (capWh < 500.0f)        { antSetReason("tank capacity?"); return; }
  if (ne < ant.refEpoch)     { antSetReason("clock"); return; }

  float needDay = antNeedDayAt(ne);
  if (needDay <= 0.0f)       { antSetReason("zero need"); return; }
  ant.needDayWh = needDay;
  float rate = needDay / 24.0f; // Wh / h
  float hSince = (float)(ne - ant.refEpoch) / 3600.0f;
  if (hSince > (float)ANT_REF_MAX_H) { antSetReason("last full too old"); return; }
  float dNow = fminf(capWh, fmaxf(0.0f, rate * hSince - ant.whSince)); // estimated tank deficit (Wh)
  ant.dNowWh = dNow;
  float safety = ANT_SAFETY * fminf(1.5f, fmaxf(1.0f, antErrRatio()));
  float dMaxA = capWh * (100.0f - (float)cfg.reservePct) / 100.0f; // max deficit tolerated at the first useful sun
  // Max deficit tolerated at the deadline: the tank must still hold the reserve PLUS enough to last the evening
  // until the start of the HC (when night heating takes over).
  float eveningWh = rate * safety * (float)((int)cfg.hcStart - (int)cfg.deadlineMin) / 60.0f;
  float dMaxD = fmaxf(capWh * ANT_DL_MIN_PCT / 100.0f, dMaxA - eveningWh);
  uint16_t minute = nowMin();
  bool hc = inHC(minute);

  if (hc) {
    uint32_t tdE = ne + antSecondsUntilMinute(cfg.deadlineMin);
    uint32_t tsE = antFirstUseful(ne, ne + 30UL * 3600UL);
    ant.tdEpoch = tdE; ant.tsEpoch = tsE;
    if (!antCovers(ne, tdE)) { antSetReason("incomplete weather"); return; }
    ant.valid = true;
    if (tsE == 0 || tsE >= tdE) { // no useful sun before the deadline: complete HC, like classic full
      ant.heat = true; ant.heatPlan = true;
      antSetReason("no useful sun");
      return;
    }
    float dTs   = dNow + rate * safety * (float)(tsE - ne) / 3600.0f;           // deficit at the first useful sun, without heating
    float erech = ANT_SOLAR_HAIRCUT * cfg.heaterW * (float)antUsefulSec(tsE, tdE) / 3600.0f;
    float cTd   = rate * safety * (float)(tdE - tsE) / 3600.0f;
    float xMax  = fmaxf(0.0f, fminf(dMaxA, dMaxD + erech - cTd));                // admissible deficit at the first useful sun
    float H     = fmaxf(0.0f, dTs - xMax);                                        // energy to supply tonight
    ant.erechWh = erech; ant.hWh = H;
    ant.heat = ant.heatPlan ? (H > 0.0f) : (H > ANT_HYST_WH);
    ant.heatPlan = ant.heat;
    antSetReason(ant.heat ? "HC: heating needed" : "HC avoided (sun forecast)");
  } else if (antDeadlineOk() && inTimeWindow(minute, cfg.hcEnd, cfg.deadlineMin)) {
    uint32_t tdE = ne + antSecondsUntilMinute(cfg.deadlineMin);
    ant.tdEpoch = tdE;
    if (!antCovers(ne, tdE)) { antSetReason("incomplete weather"); return; }
    ant.valid = true;
    uint32_t usefulSec = antUsefulSec(ne, tdE);
    float cTd     = rate * safety * (float)(tdE - ne) / 3600.0f;
    float solarWh = ANT_SOLAR_HAIRCUT * cfg.heaterW * (float)usefulSec / 3600.0f;
    float F       = fmaxf(0.0f, dNow + cTd - solarWh - dMaxD);   // energy to supply in addition to the expected solar
    float reqSec  = F / cfg.heaterW * 3600.0f;
    float availSec = (float)(tdE - ne) - (float)usefulSec;       // time available outside useful solar hours
    ant.hWh = F; ant.erechWh = solarWh;
    bool force = false;
    if (F > 0.0f) force = ant.forcing ? true : (reqSec + (float)ANT_FORCE_MARGIN_SEC >= availSec);
    ant.forcing = force; ant.force = force;
    antSetReason(force ? "catch-up before deadline" : (F > 0.0f ? "waiting for the sun" : "deadline ok"));
  } else {
    ant.valid = true;
    ant.forcing = false;
    antSetReason("off plan");
  }
}

/* ---------- Called on every tick (before the mode branches) ---------- */
static void antTick(uint32_t dt) {
  if (cfg.antMode == "off") { ant.valid = false; ant.heat = true; ant.force = false; antSetReason("off"); return; }
  uint32_t ne = (uint32_t)time(nullptr);
  bool clockOk = ne >= 946684800UL;
  if (clockOk && !antCycRestored && rt.cycleId != 0) { antCycRestored = true; antCycRestore(ne); }

  if (clockOk) {
    float tNow = antTempAt(ne);
    if (!isnan(tNow)) { ant.cycTSum10 += (int32_t)(tNow * 10.0f) * (int32_t)dt; ant.cycTSec += dt; }
    float radNow = antRadAt(ne);
    if (!isnan(radNow) && !isnan(rt.restHouse)) {
      // Real solar potential for the tank (house surplus, capped at the tank power)
      // and forecast radiation over the SAME ticks: their ratio gives k.
      ant.cycRadWh += radNow * (float)dt / 3600.0f;
      ant.cycPotWh += fminf(cfg.heaterW, fmaxf(0.0f, -rt.restHouse)) * (float)dt / 3600.0f;
    }
    // Hourly actual (forecast vs actual display): same usable surplus, without requiring the weather.
    if (!isnan(rt.restHouse)) antHrAccumulate(ne / 3600UL, fminf(cfg.heaterW, fmaxf(0.0f, -rt.restHouse)), dt);
  }

  bool heating = false;
  if (cfg.mode == "monitor") {
    ant.refValid = false; ant.wasHeating = false; ant.heatRun = 0; ant.idleRun = 0;
  } else {
    bool relayOK = rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW);
    if (rt.shellyOn && !relayOK) {
      // Relay ON but no measurement: the injected energy is unknown over this tick.
      if (ant.refValid) { ant.gapSec += dt; if (ant.gapSec > ANT_GAP_MAX_SEC) ant.refBad = true; }
    }
    if (relayOK) {
      ant.whSince += rt.shellyW * (float)dt / 3600.0f;
      heating = rt.shellyW >= cfg.thermostatW;
      if (heating) {
        ant.heatRun += dt; ant.idleRun = 0;
        if (ant.heatRun >= ANT_HEAT_MIN_SEC) ant.wasHeating = true;
      } else {
        ant.idleRun += dt; ant.heatRun = 0;
        if (ant.wasHeating && ant.idleRun >= ANT_FULL_IDLE_SEC && clockOk) {
          ant.wasHeating = false;
          antOnFull(ne);
        }
      }
    } else {
      ant.heatRun = 0; ant.idleRun = 0;
      if (!rt.shellyOn) ant.wasHeating = false;
    }
  }

  if (clockOk) antCapturePrediction(ne);
  antPlan(ne);

  // Observation mode: HC energy the plan would have avoided (indicator to validate before switching to "on").
  if (cfg.antMode == "shadow" && ant.valid && !ant.heat && heating && inHC(nowMin()) && !isnan(rt.shellyW)) {
    ant.cycSkipWh += rt.shellyW * (float)dt / 3600.0f;
  }

  static uint32_t lastLogMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastLogMs >= 60000UL) {
    lastLogMs = nowMs;
    Serial.printf("[Forecast] mode=%s valid=%s heatHC=%s catchup=%s reason=%s deficit=%.0fWh need24h=%.0fWh H=%.0fWh solar=%.0fWh k=%.3f(%u) needDays=%u\n",
                  cfg.antMode.c_str(), ant.valid ? "yes" : "no", ant.heat ? "yes" : "no", ant.force ? "yes" : "no",
                  ant.reason, ant.dNowWh, ant.needDayWh, ant.hWh, ant.erechWh, ant.k, (unsigned)ant.kN, (unsigned)ant.needN);
  }
}

/* ---------- Weather (Open-Meteo, free, no key) ---------- */
static bool antFetchWeather() {
  DiagPhaseScope phase("weather");
  if ((size_t)ESP.getMaxAllocHeap() < ANT_WX_MIN_HEAP) {
    strncpy(ant.wxErr, "low memory", sizeof(ant.wxErr) - 1);
    return false;
  }
  ecoTlsReset(); // a single TLS context at a time: release Ecojoko's (it will redo a handshake)
  WiFiClientSecure client;
  if (OPENMETEO_ROOT_CA[0] != '\0') {
    client.setCACert(OPENMETEO_ROOT_CA);
  } else {
    client.setInsecure();
    Serial.println("[Security] OPENMETEO_ROOT_CA not set: weather connection WITHOUT certificate verification (setInsecure)");
  }
  client.setHandshakeTimeout(ECO_TLS_HANDSHAKE_TIMEOUT_S);
  HTTPClient http;
  http.setReuse(false);
  http.setTimeout(ANT_WX_TIMEOUT_MS);
  char url[256];
  snprintf(url, sizeof(url),
           "https://api.open-meteo.com/v1/forecast?latitude=%.3f&longitude=%.3f"
           "&hourly=temperature_2m,shortwave_radiation&timeformat=unixtime&past_days=1&forecast_days=2"
           "&timezone=Europe%%2FParis", cfg.lat, cfg.lon);
  if (!http.begin(client, url)) {
    strncpy(ant.wxErr, "begin refused", sizeof(ant.wxErr) - 1);
    return false;
  }
  int code = http.GET();
  if (code != 200) {
    snprintf(ant.wxErr, sizeof(ant.wxErr), "HTTP %d", code);
    http.end();
    Serial.printf("[Forecast] weather failure http=%d\n", code);
    return false;
  }
  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  JsonArray tms = doc["hourly"]["time"].as<JsonArray>();
  JsonArray tp  = doc["hourly"]["temperature_2m"].as<JsonArray>();
  JsonArray rd  = doc["hourly"]["shortwave_radiation"].as<JsonArray>();
  if (err || tms.isNull() || tp.isNull() || rd.isNull()) {
    strncpy(ant.wxErr, "Invalid JSON", sizeof(ant.wxErr) - 1);
    return false;
  }
  size_t n = tms.size();
  if (n < 24 || n > ANT_WX_MAX || tp.size() < n || rd.size() < n) {
    strncpy(ant.wxErr, "unexpected size", sizeof(ant.wxErr) - 1);
    return false;
  }
  uint32_t first = tms[0].as<uint32_t>();
  if (first < 946684800UL || (uint32_t)tms[1].as<uint32_t>() - first != 3600UL) {
    strncpy(ant.wxErr, "not hourly?", sizeof(ant.wxErr) - 1);
    return false;
  }
  /* Open-Meteo: shortwave_radiation stamped T = mean of [T-1h, T] (the PRECEDING hour). The slots used by
     antIdx() are [wxStart+i*3600, +3600[ : shift the start by 1 h so slot i covers the averaged period.
     (Temperature is instantaneous and shifts by 1 h too: negligible.) */
  ant.wxStart = first - 3600UL;
  ant.wxN = (uint8_t)n;
  for (size_t i = 0; i < n; ++i) {
    ant.wxT10[i] = tp[i].isNull() ? INT16_MIN : (int16_t)lroundf(tp[i].as<float>() * 10.0f);
    float r = rd[i].isNull() ? 0.0f : rd[i].as<float>();
    ant.wxRad[i] = (uint16_t)(r < 0.0f ? 0.0f : (r > 1500.0f ? 1500.0f : r));
  }
  ant.wxEpoch = (uint32_t)time(nullptr);
  ant.wxErr[0] = 0;
  Serial.printf("[Forecast] weather ok: %u h from %lu, heapFree=%u\n", (unsigned)ant.wxN,
                (unsigned long)ant.wxStart, (unsigned)ESP.getFreeHeap());
  return true;
}

// Called in loop() when the loop is not already long. First fetch ~20 s after boot.
static void antService(uint32_t now) {
  if (cfg.antMode == "off") return;
  if (WiFi.status() != WL_CONNECTED || !antClockOk() || now < 20000UL) return;
  bool needFetch = ant.wxLastOkMs == 0 || (uint32_t)(now - ant.wxLastOkMs) >= ANT_WX_REFRESH_MS;
  bool retryOk   = ant.wxLastTryMs == 0 || (uint32_t)(now - ant.wxLastTryMs) >= (ant.wxFail ? ANT_WX_RETRY_MS : 10000UL);
  if (!needFetch || !retryOk) return;
  ant.wxLastTryMs = now ? now : 1;
  if (antFetchWeather()) {
    ant.wxFail = 0;
    ant.wxLastOkMs = millis() ? millis() : 1;
  } else if (ant.wxFail < UINT8_MAX) {
    ant.wxFail++;
  }
}

/* ---------- State exposed to the interface ---------- */
static void antFillStatus(JsonDocument &d) {
  uint32_t ne = (uint32_t)time(nullptr);
  d["antMode"]       = cfg.antMode;
  d["antShade"]      = cfg.antShade;
  d["antValid"]      = ant.valid;
  d["antHeat"]       = ant.heat;
  d["antForce"]      = ant.force;
  d["antSkipHC"]     = (cfg.antMode == "on" && ant.valid && inHC(nowMin()) && !ant.heat);
  d["antReason"]     = ant.reason;
  d["antDeficitKWh"] = ant.dNowWh / 1000.0f;
  d["antCapKWh"]     = antCapWh() / 1000.0f;
  d["antNeedKWh"]    = ant.needDayWh / 1000.0f;
  d["antPlanKWh"]    = ant.hWh / 1000.0f;
  d["antSolarKWh"]   = ant.erechWh / 1000.0f;
  d["antShadowSkipKWh"] = ant.cycSkipWh / 1000.0f;
  d["antNeedDays"]   = ant.needN;
  d["antKDays"]      = ant.kN;
  d["antKValid"]     = ant.kValid;
  d["antFullRef"]    = ant.refValid;
  d["wxOk"]          = antWxFresh(ne);
  d["wxErr"]         = ant.wxErr;
  if (ant.wxEpoch && ne >= ant.wxEpoch) d["wxAgeMin"] = (ne - ant.wxEpoch) / 60UL;
  float tn = antTempAt(ne), rn = antRadAt(ne);
  if (!isnan(tn)) d["wxTemp"] = tn;
  if (!isnan(rn)) d["wxRad"] = rn;
}

void apiAnt() {
  JsonDocument d;
  antFillStatus(d);
  uint32_t ne = (uint32_t)time(nullptr);
  d["capKWh"]        = antCapWh() / 1000.0f;
  d["tankKWh"]       = cfg.tankKWh;
  d["needBaseKWh"]   = ant.needBase / 1000.0f;
  d["slopeWhPerC"]   = ant.slope;
  d["slopeValid"]    = ant.slopeValid;
  d["tRef"]          = isnan(ant.tBase) ? ant.tRef : ant.tBase; // reference actually used by the correction
  d["k"]             = ant.k;
  d["pvKwc"]         = cfg.pvKwc;
  d["kInit"]         = cfg.pvKwc * ANT_PV_RATIO; // starting coefficient derived from the kWp
  d["kLearn"]        = ant.kLearn;
  d["kW"]            = ant.kW;
  d["kSumRad"]       = ant.kSumRad;
  d["thresholdW"]    = antThresholdW();
  d["errRatio"]      = antErrRatio();
  d["tsEpoch"]       = ant.tsEpoch;
  d["tdEpoch"]       = ant.tdEpoch;
  d["nowEpoch"]      = ne;
  d["lastFullEpoch"] = ant.lastFullEpoch;
  d["deadlineMin"]   = cfg.deadlineMin;
  d["tankLiters"]    = cfg.tankLiters;
  d["reservePct"]    = cfg.reservePct;
  d["lat"]           = cfg.lat;
  d["lon"]           = cfg.lon;
  {
    // Shading by UTC hour of the day: factor (-1 = not enough samples yet) and number of samples.
    JsonObject sj = d["shade"].to<JsonObject>();
    sj["minN"] = ANT_SHADE_MIN_N;
    if (!isnan(antShadeRef)) sj["ref"] = antShadeRef;
    JsonArray sf = sj["f"].to<JsonArray>(), sn = sj["n"].to<JsonArray>();
    for (uint8_t h = 0; h < 24; ++h) { sf.add(antShadeOk[h] ? antShadeF[h] : -1.0f); sn.add(antShadeCnt[h]); }
  }
  JsonArray w = d["wx"].to<JsonArray>();
  for (uint32_t e = ne - ne % 3600UL; e < ne + 30UL * 3600UL; e += 3600UL) {
    int i = antIdx(e);
    if (i < 0) continue;
    JsonObject o = w.add<JsonObject>();
    o["e"] = e;
    if (ant.wxT10[i] != INT16_MIN) o["t"] = ant.wxT10[i] / 10.0f;
    o["r"] = ant.wxRad[i];
    o["u"] = antHourUseful(i);
  }
  // Forecast / actual per hour: the last 168 hours up to the current hour (-1 = unknown).
  // The current hour is the partial actual (>= 5 min of measurement).
  if (ne >= 946684800UL) {
    uint32_t hNow = ne / 3600UL, h0 = hNow - 167UL;
    JsonObject hj = d["hr"].to<JsonObject>();
    hj["h0"] = h0;
    JsonArray hp = hj["p"].to<JsonArray>(), hrl = hj["r"].to<JsonArray>();
    for (uint32_t hh = h0; hh <= hNow; ++hh) {
      const AntHr &x = antHr[hh % ANT_HR_N];
      bool ok = (x.h == hh);
      hp.add(ok && x.p != 0xFFFF ? (int)x.p : -1);
      int rv = ok && x.r != 0xFFFF ? (int)x.r : -1;
      if (hh == hNow && hh == antHrCur && antHrSec >= 300UL) rv = (int)antHrW(antHrWh * 3600.0f / (float)antHrSec);
      hrl.add(rv);
    }
  }
  JsonArray a = d["recs"].to<JsonArray>();
  for (uint8_t i = 0; i < antNRec; ++i) {
    const AntRec &r = antRecs[i];
    JsonObject o = a.add<JsonObject>();
    o["c"] = r.cycle; o["w"] = r.wday; o["need"] = antRecNeed(r) / 1000.0f; o["cov"] = r.sec / 3600.0f;
    o["rad"] = r.rad; o["pot"] = r.pot; o["sk"] = r.skipWh / 1000.0f;
    if (r.t10 != INT16_MIN) o["t"] = r.t10 / 10.0f;
    if (r.predWh >= 0.0f)    o["pred"] = r.predWh / 1000.0f;    // forecast need (kWh)
    if (r.predSolWh >= 0.0f) o["psol"] = r.predSolWh / 1000.0f; // forecast solar surplus (kWh); the actual = pot (Wh)
  }
  String s;
  serializeJson(d, s);
  server.send(200, "application/json", s);
}

// Raw export of the habits (re-importable via /api/import/ant).
// Heating log: segments [start epoch, duration s, Wh injected, Wh bought, reason 0=HC 1=sun 2=catch-up].
// "open" = the last segment is in progress.
void apiHeat() {
  JsonDocument d;
  JsonArray a = d["segs"].to<JsonArray>();
  for (uint16_t i = 0; i < heatNSeg; ++i) {
    JsonArray r = a.add<JsonArray>();
    r.add(heatSegs[i].t0); r.add(heatSegs[i].sec); r.add(heatSegs[i].wh); r.add(heatSegs[i].paidWh); r.add(heatSegs[i].reason);
  }
  bool open = heatCurOpen && heatCurWh >= 1.0f;
  if (open) {
    JsonArray r = a.add<JsonArray>();
    r.add(heatCur.t0); r.add(heatCur.sec);
    r.add((uint32_t)fminf(4000000000.0f, fmaxf(0.0f, heatCurWh))); r.add((uint32_t)fminf(4000000000.0f, fmaxf(0.0f, heatCurPaid))); r.add(heatCur.reason);
  }
  d["open"] = open;
  // "Relay" line: [start epoch, duration s]; "ropen" = the last interval is in progress.
  JsonArray rl = d["rel"].to<JsonArray>();
  for (uint16_t i = 0; i < relayNIv; ++i) {
    JsonArray r = rl.add<JsonArray>();
    r.add(relayIvs[i].t0); r.add(relayIvs[i].sec);
  }
  if (relayCurOpen) {
    JsonArray r = rl.add<JsonArray>();
    r.add(relayCur.t0); r.add(relayCur.sec);
  }
  d["ropen"] = relayCurOpen;
  String out;
  serializeJson(d, out);
  server.send(200, "application/json", out);
}

// Shelly internal temperature: {step, end, n, ot, t:[degC x10 | null], w:[average W]}, from oldest to most recent.
// Point i ends at end - (n-1-i)*step. Built by hand (no JsonDocument): ~6 KB for 48 h.
void apiStemp() {
  String out;
  out.reserve((size_t)stCount * 12 + 160);
  out += "{\"step\":"; out += STEMP_STEP_SEC;
  out += ",\"end\":"; out += stLastEpoch;
  out += ",\"n\":"; out += stCount;
  out += ",\"ot\":"; out += rt.shellyOtEpoch;
  out += ",\"t\":[";
  for (uint16_t i = 0; i < stCount; ++i) {
    const STempPt &p = stPts[(stHead + STEMP_N - stCount + i) % STEMP_N];
    if (i) out += ',';
    if (p.t10 == INT16_MIN) out += "null"; else out += (int)p.t10;
  }
  out += "],\"w\":[";
  for (uint16_t i = 0; i < stCount; ++i) {
    const STempPt &p = stPts[(stHead + STEMP_N - stCount + i) % STEMP_N];
    if (i) out += ',';
    out += (unsigned)p.w;
  }
  out += "]}";
  server.send(200, "application/json", out);
}

// Manual checkpoint ("Save now" button): writes the heating log WITH the heating in progress,
// and the cycle counters. To be used just before flashing an update.
void apiCheckpoint() {
  bool open = heatCurOpen && heatCurWh >= 1.0f;
  bool ok = heatSaveImpl(true);
  if (rt.cycleId != 0) saveHistory();
  bool antOk = antCycSave(); // forecast counters (day's measurement); false if forecast disabled
  Serial.printf("[Checkpoint] log=%s segments=%u heating_in_progress=%s forecast=%s\n", ok ? "OK" : "FAIL", (unsigned)heatNSeg, open ? "yes" : "no", antOk ? "OK" : "no");
  String out = String("{\"ok\":") + (ok ? "true" : "false") + ",\"open\":" + (open ? "true" : "false") +
               ",\"ant\":" + (antOk ? "true" : "false") +
               ",\"segs\":" + String((unsigned)(heatNSeg + (open ? 1 : 0))) + "}";
  server.send(ok ? 200 : 500, "application/json", out);
}

void apiAntRaw() {
  JsonDocument d;
  antFillRecsDoc(d);
  antHrFillDoc(d); // forecast / actual per hour (already stored: the export includes them)
  String s;
  serializeJson(d, s);
  server.send(200, "application/json", s);
}

/* Shelly thermal protection. Returns true as long as the relay must stay off.
   Trigger: internal temperature >= shellyTMaxC (if > 0) OR "overtemp" alarm of the Shelly (always taken into account).
   Resume: pause of at least shellyHoldMin minutes (re-armed while it is still too hot or the alarm lasts) AND Shelly reachable
   AND below shellyTResumeC. While it is active, tick() forces "request = OFF" in all modes: no switch-on command any more.
   If the Shelly stops answering, the pause is maintained (we do not know whether it has cooled down). "monitor" mode: the relay is not controlled. */
static bool thermalGuard() {
  bool tKnown = rt.shellyConnected && !isnan(rt.shellyT);
  // ALARM threshold: one line in the error log per episode (the buzzer itself is handled by the watchdog). Also active in "monitor" mode.
  bool crit = cfg.shellyTAlarmC > 0.0f && tKnown && rt.shellyT >= cfg.shellyTAlarmC;
  if (crit && !rt.thermCritLogged) {
    rt.thermCritLogged = true;
    reportError(String("OVERHEATING ALERT: Shelly at ") + String(rt.shellyT, 1) + " degC (alarm threshold " + String(cfg.shellyTAlarmC, 0) + " degC), buzzer on", ERROR_THERMAL);
  } else if (rt.thermCritLogged && (!tKnown || rt.shellyT < cfg.shellyTAlarmC - 5.0f)) {
    rt.thermCritLogged = false;
  }
  if (cfg.mode == "monitor") { rt.thermHold = false; return false; }
  bool hot = (cfg.shellyTMaxC > 0.0f && tKnown && rt.shellyT >= cfg.shellyTMaxC) || (rt.shellyConnected && rt.shellyOvertemp);
  uint32_t now = millis();
  if (!rt.thermHold) {
    if (hot) {
      rt.thermHold = true; rt.thermHoldStartMs = now; rt.thermHoldCount++;
      reportError(String("Thermal protection: Shelly at ") + (tKnown ? String(rt.shellyT, 1) + " degC" : String("? degC")) +
                  (rt.shellyOvertemp ? " + Shelly overtemp alarm" : "") + " : relay cut, no more switch-on commands (minimum pause " +
                  String((unsigned)cfg.shellyHoldMin) + " min)", ERROR_THERMAL);
      Serial.printf("[Protection] Shelly too hot (tC=%.1f, alarm=%s) : relay cut, no more switch-on commands (minimum pause %u min, resume below %.0f degC)\n",
                    tKnown ? rt.shellyT : -99.0f, rt.shellyOvertemp ? "yes" : "no", (unsigned)cfg.shellyHoldMin, cfg.shellyTResumeC);
    }
  } else if (hot) {
    rt.thermHoldStartMs = now;   // still too hot: the pause restarts from zero
  } else {
    bool minDone = (uint32_t)(now - rt.thermHoldStartMs) >= (uint32_t)cfg.shellyHoldMin * 60000UL;
    bool cooled  = rt.shellyConnected && (isnan(rt.shellyT) || rt.shellyT <= cfg.shellyTResumeC);
    if (minDone && cooled) {
      rt.thermHold = false;
      markErrorResolved(ERROR_THERMAL);
      Serial.printf("[Protection] Shelly cooled down (tC=%.1f) : normal operation resumed\n", isnan(rt.shellyT) ? -99.0f : rt.shellyT);
    }
  }
  return rt.thermHold;
}

void tick(uint32_t dt) {
  bool tHold = thermalGuard();
  uint32_t currentCycle = cycleForNow();
  if (currentCycle == 0) {
    // Unconditional thermal safety: we try to switch off even if rt.shellyOn is stale (last
    // Shelly read failed, rt.shellyConnected false) or already false, because an unconfirmed OFF creates no risk.
    if (tHold) shellySet(false);   // invalid clock: no decision, but we still cut if the Shelly overheats
    rt.state = "INVALID_CLOCK";
    Serial.println("[Decision] invalid clock: no automatic quota decision");
    return;
  }
  if (currentCycle != rt.cycleId) {
    if (rt.cycleId != 0) {
      accumulateMonth(rt.heatedSec, rt.relayOnSec, rt.tier1Sec, rt.tier2Sec, rt.nightSec,
                       rt.cycleWh, rt.paidWh, rt.nativeEUR, rt.paidEUR);
      antCycleClose(rt.cycleId); // records the need / solar / temperature of the cycle that is closing
    }
    heatSegClose(); relayIvClose(); if (heatDirty) heatSave(); // a heating run (or a relay on) in progress does not cross the cycle change
    rt.cycleId = currentCycle;
    rt.heatedSec = 0; rt.tier1Sec = 0; rt.tier2Sec = 0; rt.nightSec = 0; rt.relayOnSec = 0;
    rt.cycleWh = 0.0f; rt.paidWh = 0.0f; rt.nativeEUR = 0.0f; rt.paidEUR = 0.0f;
    restoreCurrentHistory(); // Resume after a restart in the same local quota day.
    Serial.printf("[Quota] new local cycle=%lu, counter restored=%lus\n",
                  (unsigned long)rt.cycleId, (unsigned long)rt.heatedSec);
    saveHistory();
  }

  antTick(dt); // tank model + plan; no effect on the command outside full mode + antMode "on"
  if (cfg.mode != "full") { if (heatCurOpen) heatSegClose(); relayIvClose(); } // the log only covers full mode

  if (cfg.mode == "manual") {
    bool timerActive = cfg.manualTimer && inTimeWindow(nowMin(), cfg.manualStart, cfg.manualEnd);
    bool desired = cfg.manualTimer ? timerActive : rt.manualOn;
    if (tHold) desired = false;   // Shelly thermal protection
    if (!rt.shellyConnected) {
      // Real relay state not confirmed (last Shelly read failed): rt.shellyOn may be stale,
      // so we take no decision based on it. Exception: thermal safety remains unconditional,
      // an OFF attempted without confirmation creates no risk.
      if (tHold) shellySet(false);
      Serial.println("[Manual] Shelly state not confirmed: decision postponed (no command this tick)");
    } else if (rt.shellyOn != desired) {
      shellySet(desired);
    }

    bool heating = rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW) && rt.shellyW >= cfg.thermostatW;
    Serial.printf("[Quota] time=%s dt=%lus before=%lus after=%lus requested=%s actual=%s heating=%s\n",
                  nowText().c_str(), (unsigned long)dt, (unsigned long)rt.heatedSec,
                  (unsigned long)(rt.heatedSec + (heating ? dt : 0)), desired ? "ON" : "OFF",
                  rt.shellyOn ? "ON" : "OFF", heating ? "YES" : "NO");
    if (heating) rt.heatedSec += dt;
    if (rt.shellyConnected && rt.shellyOn) rt.relayOnSec += dt;
    accumulateEnergyCost(dt, inHC(nowMin()));
    rt.state = heating ? "MANUAL" : "MANUAL_STOPPED";
    
    if (millis() - rt.lastHistorySave >= HISTORY_SAVE_PERIOD_MS) saveHistory();
    return;
  }

  if (cfg.mode == "monitor") {
    rt.state = "MONITOR";
    if (millis() - rt.lastHistorySave >= HISTORY_SAVE_PERIOD_MS) saveHistory();
    return;
  } 

  if (cfg.mode == "full") {
    uint16_t minute = nowMin();
    bool hc = inHC(minute);
    bool dataOK = !isnan(rt.restHouse) && !isnan(rt.shellyW);
    bool tier1 = dataOK && ((cfg.heaterW + rt.restHouse) * cfg.hp < cfg.heaterW * cfg.hc);
    // Forecast: only if antMode == "on" AND the plan is valid. Otherwise (off, shadow, missing data)
    // the behaviour is exactly that of classic full: complete HC + tier1.
    bool antActive = (cfg.antMode == "on") && ant.valid;
    bool antSkipHC = antActive && hc && !ant.heat;   // HC avoided: tomorrow's sun will cover the need
    bool antForce  = antActive && !hc && ant.force;  // catch-up in HP before the deadline
    bool desired = (hc && !antSkipHC) || tier1 || antForce;
    if (tHold) desired = false;   // Shelly thermal protection
    Serial.printf("[Full] time=%s HC=%s Ecojoko(t1=%s)->%s forecast(%s)=%s final=%s | actual=%s\n",
                  nowText().c_str(), hc ? "YES" : "NO", tier1 ? "YES" : "NO",
                  tier1 ? "ON" : "OFF", cfg.antMode.c_str(),
                  antSkipHC ? "HC_AVOIDED" : (antForce ? "CATCHUP" : "NONE"),
                  desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF");

    bool canTurnOn = (millis() - rt.lastStateChangeMs) >= MIN_OFF_TO_ON_MS;

    if (!rt.shellyConnected) {
      // Real relay state not confirmed: no decision based on rt.shellyOn (possibly stale).
      // Thermal safety remains unconditional (OFF attempted without confirmation = no risk).
      if (tHold) shellySet(false);
      Serial.println("[Full] Shelly state not confirmed: decision postponed (no command this tick)");
    } else if (!rt.shellyOn && desired && canTurnOn) {
      Serial.println("[Full] final=ON action=SHELLY_ON_COMMAND reason=relay_actual_OFF");
      if (!shellySet(true)) {
        Serial.println("[Full] final=ON result=NOT_SENT (backoff or missing IP); new attempt on the next tick");
      }
    } else if (!rt.shellyOn && desired && !canTurnOn) {
      Serial.println("[Full] final=ON_PENDING reason=debounce_off_to_on");
    } else if (rt.shellyOn && !desired) {
      Serial.println("[Full] final=OFF action=SHELLY_OFF_COMMAND reason=relay_actual_ON");
      if (shellySet(false)) {
        rt.lastStateChangeMs = millis();
      }
    } else if (desired && rt.shellyOn) {
      Serial.println("[Full] final=ON action=NO_COMMAND reason=relay_already_confirmed_ON");
    }

    bool heating = rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW) && rt.shellyW >= cfg.thermostatW;
    uint32_t heatedBefore = rt.heatedSec;
    if (heating) {
      rt.heatedSec += dt;
      if (hc) rt.nightSec += dt;
      else if (tier1 || antForce) rt.tier1Sec += dt;
    }
    if (rt.shellyConnected && rt.shellyOn) rt.relayOnSec += dt;
    relayLogTick(dt);
    accumulateEnergyCost(dt, hc);
    // Heating reason: HC first; otherwise profitable sun; otherwise plan catch-up
    heatLogTick(dt, heating, hc ? HR_HC : (tier1 ? HR_SOLAR : (antForce ? HR_FORCE : HR_SOLAR)));
    Serial.printf("[Full] time=%s dt=%lus before=%lus after=%lus requested=%s actual=%s heating=%s\n",
                  nowText().c_str(), (unsigned long)dt, (unsigned long)heatedBefore,
                  (unsigned long)rt.heatedSec, desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF",
                  heating ? "YES" : "NO");

    if (hc) rt.state = heating ? "FULL_HC_HEATING" : "FULL_HC_WAITING";
    else if (tier1 || antForce) rt.state = heating ? "FULL_OPTIMAL_HEATING" : "FULL_OPTIMAL_WAITING";
    else rt.state = "FULL_STOPPED";

    if (millis() - rt.lastHistorySave >= HISTORY_SAVE_PERIOD_MS) saveHistory();
    return;
  }

  uint16_t minute = nowMin();
  bool hc = inHC(minute);
  bool quota = rt.heatedSec >= (uint32_t)(cfg.quotaH * 3600.0f);
  bool dataOK = !isnan(rt.restHouse) && !isnan(rt.shellyW);
  uint32_t quotaTargetSec = (uint32_t)(cfg.quotaH * 3600.0f);
  uint32_t quotaRemainingSec = rt.heatedSec >= quotaTargetSec ? 0 : quotaTargetSec - rt.heatedSec;
  uint32_t hcRemainingSec = secondsUntilHcEnd(minute);

  bool tier1 = dataOK && ((cfg.heaterW + rt.restHouse) * cfg.hp < cfg.heaterW * cfg.hc);
  bool tier2 = dataOK && (rt.restHouse + cfg.heaterW <= 0);

  // The quota only overrides Ecojoko at the last possible moment in HC:
  // if we wait any longer, the remaining HC duration cannot meet the target.
  bool quotaDecision = hc && !quota && hcRemainingSec <= quotaRemainingSec;
  rt.forcedNight = quotaDecision;
  // Once the quota is met, Ecojoko may still use a real solar surplus at any
  // time. Before that, tier1 remains the normal HP optimisation rule.
  bool ecoDecision = quota ? tier2 : (!hc && tier1);
  bool desired = quotaDecision || ecoDecision;
  if (tHold) { desired = false; rt.forcedNight = false; }   // Shelly thermal protection (forcedNight cleared: otherwise the OFF command would be skipped)
  Serial.printf("[Decision] time=%s HC=%s HC_remaining=%lus quota=%lu/%lus remaining=%lus | Ecojoko(t1=%s,t2=%s)->%s quota->%s final=%s | actual=%s\n",
                nowText().c_str(), hc ? "YES" : "NO", (unsigned long)hcRemainingSec,
                (unsigned long)rt.heatedSec, (unsigned long)quotaTargetSec, (unsigned long)quotaRemainingSec,
                tier1 ? "YES" : "NO", tier2 ? "YES" : "NO", ecoDecision ? "ON" : "OFF",
                quotaDecision ? "ON" : "OFF", desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF");

    bool canTurnOn = (millis() - rt.lastStateChangeMs) >= MIN_OFF_TO_ON_MS;

  if (!rt.shellyConnected) {
    // Real relay state not confirmed: no decision based on rt.shellyOn (possibly stale).
    // Thermal safety remains unconditional (OFF attempted without confirmation = no risk).
    if (tHold) shellySet(false);
    Serial.println("[Decision] Shelly state not confirmed: decision postponed (no command this tick)");
  } else if (!rt.shellyOn && desired && canTurnOn) {
    Serial.println("[Decision] final=ON action=SHELLY_ON_COMMAND reason=relay_actual_OFF");
    if (!shellySet(true)) {
      Serial.println("[Decision] final=ON result=NOT_SENT (backoff or missing IP); new attempt on the next tick");
    }
  } else if (!rt.shellyOn && desired && !canTurnOn) {
    Serial.println("[Decision] final=ON_PENDING reason=debounce_off_to_on");
  } else if (rt.shellyOn && !desired && !rt.forcedNight) {
    Serial.println("[Decision] final=OFF action=SHELLY_OFF_COMMAND reason=relay_actual_ON");
    if (shellySet(false)) {
      rt.lastStateChangeMs = millis();
    }
  } else if (desired && rt.shellyOn) {
    Serial.println("[Decision] final=ON action=NO_COMMAND reason=relay_already_confirmed_ON");
  }
  bool heating = rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW) && rt.shellyW >= cfg.thermostatW;
  uint32_t quotaBefore = rt.heatedSec;
  if (heating) {
    rt.heatedSec += dt;
    if (rt.forcedNight) rt.nightSec += dt;
    else if (quota) rt.tier2Sec += dt;
    else rt.tier1Sec += dt;
  }
  if (rt.shellyConnected && rt.shellyOn) rt.relayOnSec += dt;
  accumulateEnergyCost(dt, hc);
  Serial.printf("[Quota] time=%s dt=%lus before=%lus after=%lus requested=%s actual=%s heating=%s\n",
                nowText().c_str(), (unsigned long)dt, (unsigned long)quotaBefore,
                (unsigned long)rt.heatedSec, desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF",
                heating ? "YES" : "NO");

    if (rt.forcedNight) rt.state = heating ? "HC_HEATING" : "HC_WAITING";
  else if (quota) rt.state = heating ? "SURPLUS_HEATING" : "SURPLUS_WAITING";
  else if (!hc && !quota) rt.state = heating ? "OPTIMAL_HEATING" : "OPTIMAL_WAITING";
  else rt.state = "STOPPED";

  if (millis() - rt.lastHistorySave >= HISTORY_SAVE_PERIOD_MS) saveHistory();
}

/* ============================================================
   7. WEB ROUTES / API
   ============================================================ */
static String statusJson() {
  JsonDocument d;
  uint32_t epoch = (uint32_t)time(nullptr);
  d["state"] = rt.state; 
  d["mode"] = cfg.mode;
  d["autoTime"] = cfg.autoTime;
  d["manualOn"] = rt.manualOn; 
  d["wifiConnected"] = (WiFi.status() == WL_CONNECTED); 
  d["wifiRssi"] = WiFi.RSSI();
  d["staticFallback"] = rt.staticFallback;
  {
    uint32_t apTot = (uint32_t)cfg.bootApSec * 1000UL, apEl = millis() - rt.bootApStartMs;
    d["bootApLeftSec"] = (rt.bootApOn && apEl < apTot) ? (int)((apTot - apEl) / 1000UL) : 0;
  }
  d["apRescue"] = rt.apRescue;
  d["ecoConnected"] = rt.ecoConnected;
  d["shellyConnected"] = rt.shellyConnected; 
  d["shellyLastMs"] = rt.shellyLastMs;
  d["shellyMaxMs"] = rt.shellyMaxMs;
  d["shellyRetryRecovered"] = rt.shellyRetryRecovered;
  d["nowEpoch"] = epoch; 
  d["inHC"] = inHC(nowMin());
  d["quotaReached"] = (rt.heatedSec >= (uint32_t)(cfg.quotaH * 3600.0f)); 
  
  bool dataOK = !isnan(rt.restHouse);
  d["tier1Eligible"] = dataOK && ((cfg.heaterW + rt.restHouse) * cfg.hp < cfg.heaterW * cfg.hc);
  d["tier2Eligible"] = dataOK && (rt.restHouse + cfg.heaterW <= 0); 
  
  if (dataOK) {
    d["ecoRaw"] = rt.ecoRaw;
    d["ecoSmooth"] = rt.ecoSmooth;
    d["restHouse"] = rt.restHouse;
    d["comparisonLeft"] = (cfg.heaterW + rt.restHouse) / 1000.0f * cfg.hp;
    d["solarBalanceW"] = rt.restHouse + cfg.heaterW;
  } else {
    d["ecoRaw"] = nullptr;
    d["ecoSmooth"] = nullptr;
    d["restHouse"] = nullptr;
    d["comparisonLeft"] = nullptr;
    d["solarBalanceW"] = nullptr;
  }
  
  d["comparisonRight"] = cfg.heaterW / 1000.0f * cfg.hc; 
  d["manualTimerEnabled"] = cfg.manualTimer; 
  d["manualTimerActive"] = cfg.manualTimer && inTimeWindow(nowMin(), cfg.manualStart, cfg.manualEnd);
  d["manualStart"] = cfg.manualStart; 
  d["manualEnd"] = cfg.manualEnd; 
  d["error"] = rt.error; 
  d["errorCountToday"] = rt.errorCountToday;
  d["ecoFailCount"] = rt.ecoFailCount;
  if (rt.ecoFailCount > 0) {
    // Countdown before the next Ecojoko attempt (real backoff, resolution = control period of 5 s).
    uint32_t nowMs = millis();
    uint32_t el = (uint32_t)(nowMs - rt.lastEcoPollMs), wait = networkBackoffMs(rt.ecoFailCount);
    d["ecoNextTryS"] = (rt.lastEcoPollMs == 0 || el >= wait) ? 0 : (int)((wait - el + 999UL) / 1000UL);
    uint32_t blocked = ecoBlockedAgeMs(nowMs);
    if (blocked > 0) {
      d["ecoBlockedMin"] = (int)(blocked / 60000UL);
      d["ecoRestartInMin"] = blocked >= ECO_BLOCKED_RESTART_MS ? 0 : (int)((ECO_BLOCKED_RESTART_MS - blocked + 59999UL) / 60000UL);
    }
  }
  d["shellyOn"] = rt.shellyOn;
  
  if (!isnan(rt.shellyW)) d["shellyW"] = rt.shellyW;
  else d["shellyW"] = nullptr;
  if (rt.shellyConnected && !isnan(rt.shellyT)) d["shellyT"] = roundf(rt.shellyT * 10.0f) / 10.0f;
  else d["shellyT"] = nullptr;
  d["thermHold"] = rt.thermHold;
  if (rt.thermHold) {
    uint32_t el = (uint32_t)(millis() - rt.thermHoldStartMs), tot = (uint32_t)cfg.shellyHoldMin * 60000UL;
    d["thermHoldMinLeft"] = el >= tot ? 0 : (int)((tot - el + 59999UL) / 60000UL);
  }
  d["thermHoldCount"] = rt.thermHoldCount;
  d["shellyOvertemp"] = rt.shellyConnected && rt.shellyOvertemp;   // no frozen alert if the Shelly stops answering

  d["heatedH"] = (float)rt.heatedSec / 3600.0f; 
  d["quotaH"] = cfg.quotaH; 
  d["tier1H"] = (float)rt.tier1Sec / 3600.0f;
  d["tier2H"] = (float)rt.tier2Sec / 3600.0f; 
  d["nightH"] = (float)rt.nightSec / 3600.0f;
  d["incoherent"] = (inHC(nowMin()) && rt.shellyW > 100.0f && !rt.shellyOn);
  wdFillStatus(d);
  antFillStatus(d);

  String s; 
  serializeJson(d, s); 
  return s;
}

void apiConfig() {
  if (server.method() == HTTP_GET) {
    JsonDocument d;
    d["wifiSsid"] = cfg.wifiSsid; d["wifiPassword"] = cfg.wifiPassword.length() ? PASS_MASK : "";
    d["ecoEmail"] = cfg.ecoEmail; d["ecoPassword"] = cfg.ecoPassword.length() ? PASS_MASK : "";
    d["shellyIp"] = cfg.shellyIp; d["mode"] = cfg.mode;
    d["manualTimer"] = cfg.manualTimer; d["manualStart"] = cfg.manualStart; d["manualEnd"] = cfg.manualEnd;
    d["hcStart"] = cfg.hcStart; d["hcEnd"] = cfg.hcEnd; d["heaterW"] = cfg.heaterW;
    d["thermostatW"] = cfg.thermostatW;
    d["hp"] = cfg.hp; d["hc"] = cfg.hc; d["quotaH"] = cfg.quotaH;
    d["smoothingMin"] = cfg.smoothingMin;
    d["ecoGraceSec"] = cfg.ecoGraceSec;
    d["wdHbPin"] = cfg.wdHbPin; d["wdErrPin"] = cfg.wdErrPin; d["wdProblemMin"] = cfg.wdProblemMin;
    d["wdTxPin"] = cfg.wdTxPin; d["muteOn"] = cfg.muteOn; d["muteStart"] = cfg.muteStart; d["muteEnd"] = cfg.muteEnd; d["snoozeMin"] = cfg.snoozeMin;
    d["manualEpoch"] = cfg.manualEpoch;
    d["autoTime"] = cfg.autoTime;
    d["antMode"] = cfg.antMode; d["antShade"] = cfg.antShade; d["tankLiters"] = cfg.tankLiters; d["tankKWh"] = cfg.tankKWh; d["reservePct"] = cfg.reservePct;
    d["deadlineMin"] = cfg.deadlineMin; d["lat"] = cfg.lat; d["lon"] = cfg.lon;
    d["pvKwc"] = cfg.pvKwc; d["staticIp"] = cfg.staticIp; d["ip"] = cfg.ip; d["gw"] = cfg.gw; d["mask"] = cfg.mask; d["dns"] = cfg.dns; d["bootApSec"] = cfg.bootApSec;
    d["shellyTMaxC"] = cfg.shellyTMaxC; d["shellyTResumeC"] = cfg.shellyTResumeC; d["shellyHoldMin"] = cfg.shellyHoldMin; d["shellyTAlarmC"] = cfg.shellyTAlarmC;
    String s; serializeJson(d, s); server.send(200, "application/json", s);
  } 
  else if (server.method() == HTTP_POST) {
    JsonDocument d;
    if (deserializeJson(d, server.arg("plain")) || !d.is<JsonObject>()) { server.send(400, "text/plain", "Invalid JSON"); return; }
    {
      int newHb  = d.containsKey("wdHbPin")  ? d["wdHbPin"].as<int>()  : (int)cfg.wdHbPin;
      int newErr = d.containsKey("wdErrPin") ? d["wdErrPin"].as<int>() : (int)cfg.wdErrPin;
      int newMin = d.containsKey("wdProblemMin") ? d["wdProblemMin"].as<int>() : (int)cfg.wdProblemMin;
      int newTx  = d.containsKey("wdTxPin")  ? d["wdTxPin"].as<int>()  : (int)cfg.wdTxPin;
      if (!wdPinAllowed(newHb) || !wdPinAllowed(newErr) || (newHb >= 0 && newHb == newErr)) {
        server.send(400, "text/plain", "Invalid pin (or same pin for HB and ERR)");
        return;
      }
      if (!wdPinAllowed(newTx) || (newTx >= 0 && (newTx == newHb || newTx == newErr))) {
        server.send(400, "text/plain", "Invalid TX pin (or same as HB or ERR)");
        return;
      }
      if (newMin < 5 || newMin > 720) {
        server.send(400, "text/plain", "Alarm delay: between 5 and 720 minutes");
        return;
      }
    }
    {
      if (d.containsKey("antMode")) {
        String m = d["antMode"].as<String>();
        if (m != "off" && m != "shadow" && m != "on") { server.send(400, "text/plain", "antMode: off, shadow or on"); return; }
      }
      if (d.containsKey("tankLiters") && (d["tankLiters"].as<int>() < 50 || d["tankLiters"].as<int>() > 1000)) {
        server.send(400, "text/plain", "Tank volume: between 50 and 1000 L"); return;
      }
      if (d.containsKey("tankKWh") && (d["tankKWh"].as<float>() < 0.0f || d["tankKWh"].as<float>() > 60.0f)) {
        server.send(400, "text/plain", "Capacity: 0 (automatic) or up to 60 kWh"); return;
      }
      if (d.containsKey("reservePct") && (d["reservePct"].as<int>() < 0 || d["reservePct"].as<int>() > 90)) {
        server.send(400, "text/plain", "Reserve: between 0 and 90 %"); return;
      }
      if (d.containsKey("deadlineMin") && (d["deadlineMin"].as<int>() < 0 || d["deadlineMin"].as<int>() > 1439)) {
        server.send(400, "text/plain", "Deadline: minute of the day (0-1439)"); return;
      }
      if (d.containsKey("lat") && fabsf(d["lat"].as<float>()) > 90.0f) { server.send(400, "text/plain", "Invalid latitude"); return; }
      if (d.containsKey("lon") && fabsf(d["lon"].as<float>()) > 180.0f) { server.send(400, "text/plain", "Invalid longitude"); return; }
      if (d.containsKey("pvKwc") && !(d["pvKwc"].as<float>() >= 0.0f && d["pvKwc"].as<float>() <= 50.0f)) {
        server.send(400, "text/plain", "Peak power: 0 (none) to 50 kWp"); return;
      }
      if (d.containsKey("bootApSec") && (d["bootApSec"].as<int>() < 0 || d["bootApSec"].as<int>() > 600)) {
        server.send(400, "text/plain", "Boot AP duration: 0 to 600 s"); return;
      }
      if (d.containsKey("shellyTMaxC") && !(d["shellyTMaxC"].as<float>() >= 0.0f && d["shellyTMaxC"].as<float>() <= 150.0f)) {
        server.send(400, "text/plain", "Shelly cut-off threshold: 0 (disabled) to 150 degC"); return;
      }
      if (d.containsKey("shellyTResumeC") && !(d["shellyTResumeC"].as<float>() >= 0.0f && d["shellyTResumeC"].as<float>() <= 150.0f)) {
        server.send(400, "text/plain", "Shelly resume temperature: 0 to 150 degC"); return;
      }
      if (d.containsKey("shellyTAlarmC") && !(d["shellyTAlarmC"].as<float>() >= 0.0f && d["shellyTAlarmC"].as<float>() <= 200.0f)) {
        server.send(400, "text/plain", "Shelly alarm threshold: 0 (disabled) to 200 degC"); return;
      }
      if (d.containsKey("shellyHoldMin") && (d["shellyHoldMin"].as<int>() < 1 || d["shellyHoldMin"].as<int>() > 240)) {
        server.send(400, "text/plain", "Minimum pause after overheating: 1 to 240 min"); return;
      }
      if (d.containsKey("muteStart") && (d["muteStart"].as<int>() < 0 || d["muteStart"].as<int>() > 1439)) {
        server.send(400, "text/plain", "Quiet window start: minute of the day (0-1439)"); return;
      }
      if (d.containsKey("muteEnd") && (d["muteEnd"].as<int>() < 0 || d["muteEnd"].as<int>() > 1439)) {
        server.send(400, "text/plain", "Quiet window end: minute of the day (0-1439)"); return;
      }
      if (d.containsKey("snoozeMin") && (d["snoozeMin"].as<int>() < 5 || d["snoozeMin"].as<int>() > 720)) {
        server.send(400, "text/plain", "Remote silence duration: between 5 and 720 minutes"); return;
      }
      {
        float nMax = d.containsKey("shellyTMaxC") ? d["shellyTMaxC"].as<float>() : cfg.shellyTMaxC;
        float nRes = d.containsKey("shellyTResumeC") ? d["shellyTResumeC"].as<float>() : cfg.shellyTResumeC;
        if (nMax > 0.0f && nRes >= nMax) { server.send(400, "text/plain", "The resume temperature must be lower than the cut-off threshold"); return; }
      }
      bool wantStatic = d.containsKey("staticIp") ? d["staticIp"].as<bool>() : cfg.staticIp;
      if (wantStatic) {
        String nip   = d.containsKey("ip")   ? d["ip"].as<String>()   : cfg.ip;
        String ngw   = d.containsKey("gw")   ? d["gw"].as<String>()   : cfg.gw;
        String nmask = d.containsKey("mask") ? d["mask"].as<String>() : cfg.mask;
        String ndns  = d.containsKey("dns")  ? d["dns"].as<String>()  : cfg.dns;
        IPAddress a;
        if (!parseIp(nip, a) || !parseIp(ngw, a) || !parseIp(nmask, a) || (ndns.length() && !parseIp(ndns, a))) {
          server.send(400, "text/plain", "Static IP: valid IP, gateway and mask required (DNS optional)"); return;
        }
      }
    }
    String oldSsid = cfg.wifiSsid; String oldPass = cfg.wifiPassword;
    String oldNet = String(cfg.staticIp ? "1" : "0") + cfg.ip + "|" + cfg.gw + "|" + cfg.mask + "|" + cfg.dns;
    bool oldAutoTime = cfg.autoTime;
    uint32_t oldManualEpoch = cfg.manualEpoch;
    
    if (d.containsKey("wifiSsid")) cfg.wifiSsid = d["wifiSsid"].as<String>();
    if (d.containsKey("wifiPassword") && d["wifiPassword"].as<String>() != PASS_MASK) cfg.wifiPassword = d["wifiPassword"].as<String>();
    if (d.containsKey("ecoEmail")) cfg.ecoEmail = d["ecoEmail"].as<String>();
    if (d.containsKey("ecoPassword") && d["ecoPassword"].as<String>() != PASS_MASK) cfg.ecoPassword = d["ecoPassword"].as<String>();
    if (d.containsKey("shellyIp")) cfg.shellyIp = d["shellyIp"].as<String>();
    
    String requestedMode = cfg.mode;
    if (d.containsKey("mode")) requestedMode = d["mode"].as<String>();
    if (requestedMode == "monitor" && cfg.mode != "monitor" && rt.shellyOn) { shellySet(false); }
    cfg.mode = requestedMode;
    
    if (d.containsKey("manualTimer")) cfg.manualTimer = d["manualTimer"].as<bool>();
    if (d.containsKey("manualStart")) cfg.manualStart = d["manualStart"].as<uint16_t>();
    if (d.containsKey("manualEnd")) cfg.manualEnd = d["manualEnd"].as<uint16_t>();
    if (d.containsKey("hcStart")) cfg.hcStart = d["hcStart"].as<uint16_t>();
    if (d.containsKey("hcEnd")) cfg.hcEnd = d["hcEnd"].as<uint16_t>();
    if (d.containsKey("heaterW")) cfg.heaterW = d["heaterW"].as<float>();
    if (d.containsKey("thermostatW")) cfg.thermostatW = d["thermostatW"].as<float>();
    if (d.containsKey("hp")) cfg.hp = d["hp"].as<float>();
    if (d.containsKey("hc")) cfg.hc = d["hc"].as<float>();
    if (d.containsKey("quotaH")) cfg.quotaH = d["quotaH"].as<float>();
    if (d.containsKey("smoothingMin")) cfg.smoothingMin = (uint8_t)constrain(d["smoothingMin"].as<int>(), 1, 30);
    if (d.containsKey("ecoGraceSec")) cfg.ecoGraceSec = d["ecoGraceSec"].as<uint16_t>();
    if (d.containsKey("wdHbPin")) cfg.wdHbPin = (int8_t)d["wdHbPin"].as<int>();
    if (d.containsKey("wdErrPin")) cfg.wdErrPin = (int8_t)d["wdErrPin"].as<int>();
    if (d.containsKey("wdProblemMin")) cfg.wdProblemMin = (uint16_t)d["wdProblemMin"].as<int>();
    if (d.containsKey("wdTxPin")) cfg.wdTxPin = (int8_t)d["wdTxPin"].as<int>();
    if (d.containsKey("muteOn")) cfg.muteOn = d["muteOn"].as<bool>();
    if (d.containsKey("muteStart")) cfg.muteStart = (uint16_t)d["muteStart"].as<int>();
    if (d.containsKey("muteEnd")) cfg.muteEnd = (uint16_t)d["muteEnd"].as<int>();
    if (d.containsKey("snoozeMin")) cfg.snoozeMin = (uint16_t)d["snoozeMin"].as<int>();
    if (d.containsKey("manualEpoch")) cfg.manualEpoch = d["manualEpoch"].as<uint32_t>();
    if (d.containsKey("autoTime")) cfg.autoTime = d["autoTime"].as<bool>();
    if (d.containsKey("antMode")) cfg.antMode = d["antMode"].as<String>();
    if (d.containsKey("antShade")) cfg.antShade = d["antShade"].as<bool>();
    if (d.containsKey("tankLiters")) cfg.tankLiters = (uint16_t)d["tankLiters"].as<int>();
    if (d.containsKey("tankKWh")) cfg.tankKWh = d["tankKWh"].as<float>();
    if (d.containsKey("reservePct")) cfg.reservePct = (uint8_t)d["reservePct"].as<int>();
    if (d.containsKey("deadlineMin")) cfg.deadlineMin = (uint16_t)d["deadlineMin"].as<int>();
    if (d.containsKey("lat")) cfg.lat = d["lat"].as<float>();
    if (d.containsKey("lon")) cfg.lon = d["lon"].as<float>();
    if (d.containsKey("pvKwc")) cfg.pvKwc = d["pvKwc"].as<float>();
    if (d.containsKey("staticIp")) cfg.staticIp = d["staticIp"].as<bool>();
    if (d.containsKey("ip")) cfg.ip = d["ip"].as<String>();
    if (d.containsKey("gw")) cfg.gw = d["gw"].as<String>();
    if (d.containsKey("mask")) cfg.mask = d["mask"].as<String>();
    if (d.containsKey("dns")) cfg.dns = d["dns"].as<String>();
    if (d.containsKey("bootApSec")) cfg.bootApSec = (uint16_t)d["bootApSec"].as<int>();
    if (d.containsKey("shellyTMaxC")) cfg.shellyTMaxC = d["shellyTMaxC"].as<float>();
    if (d.containsKey("shellyTResumeC")) cfg.shellyTResumeC = d["shellyTResumeC"].as<float>();
    if (d.containsKey("shellyHoldMin")) cfg.shellyHoldMin = (uint16_t)d["shellyHoldMin"].as<int>();
    if (d.containsKey("shellyTAlarmC")) cfg.shellyTAlarmC = d["shellyTAlarmC"].as<float>();
    sanitizeConfig();
    String newNet = String(cfg.staticIp ? "1" : "0") + cfg.ip + "|" + cfg.gw + "|" + cfg.mask + "|" + cfg.dns;
    if (d.containsKey("pvKwc")) antRefresh(); // the new starting coefficient applies immediately
    
    saveConfig();
    wdSetPins(cfg.wdHbPin, cfg.wdErrPin); // taken into account immediately, without a restart
    wdSetTxPin(cfg.wdTxPin);
    wdMuteChanged(); // window or duration changed: the C3 immediately receives the new silence duration
    server.send(200, "text/plain", "OK");
    bool clockChanged = cfg.autoTime != oldAutoTime ||
                        (!cfg.autoTime && cfg.manualEpoch != oldManualEpoch);
    if (cfg.wifiSsid != oldSsid || cfg.wifiPassword != oldPass || newNet != oldNet || clockChanged) {
      delay(500); ESP.restart();
    }
  }
}

// POST /api/watchdog {"cmd":"test"}: sets ERR to 1 for 8 s to check that the C3 really sounds the buzzer.
// (Acknowledgement is done with the button wired to the C3: there is no return link.)
void apiWatchdog() {
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain")) || d["cmd"].as<String>() != "test") {
    server.send(400, "text/plain", "Invalid command");
    return;
  }
  if (!wdStartTest()) {
    server.send(400, "text/plain", "Alarm pin (ERR) disabled");
    return;
  }
  server.send(200, "text/plain", "OK");
}

// POST /api/snooze {"cmd":"on"|"off"}: silences the C3 buzzer for cfg.snoozeMin minutes, or re-enables it.
// "on" is only accepted during an alarm, never for overheating (see wdSnoozeCmd).
void apiSnooze() {
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain"))) { server.send(400, "text/plain", "Invalid JSON"); return; }
  String cmd = d["cmd"].as<String>();
  if (cmd != "on" && cmd != "off") { server.send(400, "text/plain", "Invalid command"); return; }
  String msg;
  int code = wdSnoozeCmd(cmd == "on", msg);
  server.send(code, "text/plain", code == 200 ? String("OK") : msg);
}

void apiReboot() {
  server.send(200, "text/plain", "OK");
  delay(300);
  ESP.restart();
}

void apiManual() {
  if (cfg.mode != "manual") { server.send(400, "text/plain", "The system is not in manual mode"); return; }
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain")) || !d["on"].is<bool>()) { server.send(400, "text/plain", "Invalid JSON"); return; }
  rt.manualOn = d["on"].as<bool>();
  server.send(200, "text/plain", "OK");
}

void apiHistory() {
  File f = LittleFS.open("/history.json", "r");
  if (!f) { server.send(200, "application/json", "{\"days\":[]}"); return; }
  server.streamFile(f, "application/json");
  f.close();
}

// Serves /monthly.json, adding today's in-progress cycle on top of the
// current month's already-committed totals so the recap looks live -
// without persisting that partial day early.
void apiMonthly() {
  JsonDocument doc;
  JsonArray months = doc["months"].to<JsonArray>();
  File in = LittleFS.open("/monthly.json", "r");
  bool currentMonthFound = false;
  uint32_t todayKey = monthKeyForToday();
  if (in) {
    JsonDocument tmp;
    DeserializationError err = deserializeJson(tmp, in);
    in.close();
    if (!err && tmp.is<JsonObject>() && tmp["months"].is<JsonArray>()) {
      for (JsonObject src : tmp["months"].as<JsonArray>()) {
        if (src.isNull()) continue;
        uint32_t mk = src["monthKey"] | 0;
        JsonObject dst = months.add<JsonObject>();
        if (mk != 0 && mk == todayKey) {
          currentMonthFound = true;
          float mergedCycleWh   = (float)(src["energyKWh"] | 0.0f) * 1000.0f + rt.cycleWh;
          float mergedPaidWh    = (float)(src["paidKWh"]   | 0.0f) * 1000.0f + rt.paidWh;
          float mergedNativeEUR = (float)(src["nativeEUR"] | 0.0f) + rt.nativeEUR;
          float mergedPaidEUR   = (float)(src["paidEUR"]   | 0.0f) + rt.paidEUR;
          fillMonthlyEntry(dst, mk, (uint32_t)(src["heatedSec"] | 0) + rt.heatedSec,
                            (uint32_t)(src["relayOnSec"] | 0) + rt.relayOnSec,
                            (uint32_t)(src["tier1Sec"] | 0) + rt.tier1Sec,
                            (uint32_t)(src["tier2Sec"] | 0) + rt.tier2Sec,
                            (uint32_t)(src["nightSec"] | 0) + rt.nightSec,
                            mergedCycleWh, mergedPaidWh, mergedNativeEUR, mergedPaidEUR);
        } else {
          // Closed month: re-serialised as is, never recomputed with the current tariffs.
          dst["monthKey"]   = mk;
          dst["heatedSec"]  = src["heatedSec"]  | 0;
          dst["relayOnSec"] = src["relayOnSec"] | 0;
          dst["tier1Sec"]   = src["tier1Sec"]   | 0;
          dst["tier2Sec"]   = src["tier2Sec"]   | 0;
          dst["nightSec"]   = src["nightSec"]   | 0;
          dst["energyKWh"]  = src["energyKWh"]  | 0.0f;
          dst["nativeEUR"]  = src["nativeEUR"]  | 0.0f;
          dst["paidKWh"]    = src["paidKWh"]    | 0.0f;
          dst["paidEUR"]    = src["paidEUR"]    | 0.0f;
          dst["savedKWh"]   = src["savedKWh"]   | 0.0f;
          dst["savedEUR"]   = src["savedEUR"]   | 0.0f;
        }
      }
    }
  }
  if (!currentMonthFound && todayKey != 0) {
    // First day of the month: no closed month recorded yet, only today is shown.
    JsonObject dst = months.add<JsonObject>();
    fillMonthlyEntry(dst, todayKey, rt.heatedSec, rt.relayOnSec, rt.tier1Sec, rt.tier2Sec, rt.nightSec,
                      rt.cycleWh, rt.paidWh, rt.nativeEUR, rt.paidEUR);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Serves the yearly recap: sum of the months of /monthly.json per calendar year (closed months and the
// in-progress month), plus today's in-progress cycle - same "live" logic as apiMonthly().
// The 60 months kept cover 5 years; the oldest year may be partial and is dropped
// beyond YEARLY_HISTORY_MAX years.
void apiYearly() {
  JsonDocument doc;
  JsonArray years = doc["years"].to<JsonArray>();
  {
    JsonDocument tmp;
    File in = LittleFS.open("/monthly.json", "r");
    if (in) {
      DeserializationError err = deserializeJson(tmp, in);
      in.close();
      if (!err && tmp["months"].is<JsonArray>()) {
        for (JsonObject src : tmp["months"].as<JsonArray>()) {
          uint32_t mk = src["monthKey"] | 0;
          if (src.isNull() || mk == 0) continue;
          addTotals(yearEntry(years, mk / 100), src);
        }
      }
    }
  }
  uint32_t todayKey = monthKeyForToday();
  if (todayKey != 0) {
    JsonDocument live;
    JsonObject lo = live.to<JsonObject>();
    fillMonthlyEntry(lo, todayKey, rt.heatedSec, rt.relayOnSec, rt.tier1Sec, rt.tier2Sec, rt.nightSec,
                     rt.cycleWh, rt.paidWh, rt.nativeEUR, rt.paidEUR);
    addTotals(yearEntry(years, todayKey / 100), lo);
  }
  while (years.size() > YEARLY_HISTORY_MAX) years.remove(0);
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Memory state: LittleFS storage (files), RAM, program flash, history purges.
void apiStorage() {
  JsonDocument doc;
  size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
  doc["totalBytes"]   = (uint32_t)total;
  doc["usedBytes"]    = (uint32_t)used;
  doc["freeBytes"]    = (uint32_t)(total > used ? total - used : 0);
  doc["minFreeBytes"] = (uint32_t)storageMinFree();
  doc["low"]          = (total > used ? total - used : 0) < storageMinFree();
  JsonArray files = doc["files"].to<JsonArray>();
  File root = LittleFS.open("/");
  if (root) {
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      if (f.isDirectory()) continue;
      String nm = f.name();
      if (!nm.startsWith("/")) nm = "/" + nm;
      JsonObject o = files.add<JsonObject>();
      o["name"] = nm;
      o["size"] = (uint32_t)f.size();
    }
    root.close();
  }
  doc["heapFree"]     = ESP.getFreeHeap();
  doc["heapMin"]      = ESP.getMinFreeHeap();
  doc["heapMaxBlock"] = ESP.getMaxAllocHeap();
  doc["heapTotal"]    = ESP.getHeapSize();
  doc["fwVersion"]    = FW_VERSION;
  doc["fwBuild"]      = FW_BUILD;
  doc["sketchBytes"]  = ESP.getSketchSize();
  doc["sketchFree"]   = ESP.getFreeSketchSpace();
  doc["purgeCount"]   = storagePurgeCount;
  doc["lastPurgeEpoch"] = storageLastPurgeEpoch;
  doc["lastPurgeWhat"]  = storageLastPurgeWhat;
  JsonObject lim = doc["limits"].to<JsonObject>();
  lim["days"] = 31; lim["months"] = MONTHLY_HISTORY_MAX; lim["years"] = YEARLY_HISTORY_MAX;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Serves the rolling error log, filtered to the last 24h (epoch-based; entries
// logged while the clock was invalid have epoch=0 and are always included).
void apiErrors() {
  JsonDocument doc;
  JsonArray arr = doc["errors"].to<JsonArray>();
  time_t now = time(nullptr);
  bool clockOk = now >= 946684800;
  uint32_t cutoff = clockOk ? (uint32_t)now - 86400UL : 0;
  uint8_t start = (errorLogCount < ERROR_LOG_CAPACITY) ? 0 : errorLogHead;
  for (uint8_t i = 0; i < errorLogCount; i++) {
    uint8_t idx = (start + i) % ERROR_LOG_CAPACITY;
    ErrorLogEntry &e = errorLog[idx];
    if (clockOk && e.epoch != 0 && e.epoch < cutoff) continue;
    JsonObject o = arr.add<JsonObject>();
    o["epoch"]   = e.epoch;
    o["source"]  = errorSourceName(e.source);
    o["message"] = e.message;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

/* ============================================================
   7b. STALL DIAGNOSTIC
   ============================================================ */
static String diagFormatEpoch(uint32_t epoch) {
  if (epoch < 1600000000UL) return "unknown time";
  time_t t = (time_t)epoch;
  tm lt;
  localtime_r(&t, &lt);
  char buf[24];
  strftime(buf, sizeof(buf), "%d/%m %H:%M:%S", &lt);
  return String(buf);
}

static const char *diagResetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "crash (exception)";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "brownout (voltage drop)";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_EXT:       return "external reset";
    default:                return "unknown";
  }
}

// Appends a line to /boot_log.txt keeping the file under DIAG_LOG_MAX_BYTES (the oldest lines are lost).
static void diagAppendLog(const String &line) {
  String content;
  File in = LittleFS.open("/boot_log.txt", "r");
  if (in) { content = in.readString(); in.close(); }
  content += line + "\n";
  if (content.length() > DIAG_LOG_MAX_BYTES) {
    int cut = content.indexOf('\n', content.length() - (DIAG_LOG_MAX_BYTES * 2 / 3));
    content = (cut >= 0) ? content.substring(cut + 1) : line + "\n";
  }
  File out = LittleFS.open("/boot_log.txt", "w");
  if (!out) return;
  out.print(content);
  out.close();
}

// Called after every server.handleClient(): keeps the request in the RAM ring and logs slowness in /boot_log.txt.
static void diagWebAfter(uint32_t ms) {
  bool handled = (diagWebUri[0] != '(');
  if (handled) {
    WebReq &r = webRing[webRingHead];
    time_t t = time(nullptr);
    r.epoch = (t > 1600000000L) ? (uint32_t)t : 0;
    r.ms = ms;
    strncpy(r.uri, diagWebUri, sizeof(r.uri) - 1); r.uri[sizeof(r.uri) - 1] = '\0';
    strncpy(r.ip,  diagWebIp,  sizeof(r.ip)  - 1); r.ip[sizeof(r.ip) - 1]   = '\0';
    webRingHead = (webRingHead + 1) % WEB_RING;
    if (webRingCount < WEB_RING) webRingCount++;
  }
  if (ms >= WEB_SLOW_MS) {
    Serial.printf("[Web] slow: %s client=%s duration=%lums\n", diagWebUri, diagWebIp, (unsigned long)ms);
    uint32_t nowMs = millis();
    if (webLastSlowLogMs == 0 || (uint32_t)(nowMs - webLastSlowLogMs) > 30000UL) { // no more than one flash write per 30 s
      webLastSlowLogMs = nowMs ? nowMs : 1;
      String line = String("slow_web uri=") + diagWebUri + " client=" + diagWebIp + " duration=" + String(ms) + "ms" +
                    " heapFree=" + String(ESP.getFreeHeap()) + " rssi=" + String(WiFi.RSSI()) +
                    " time=" + diagFormatEpoch((uint32_t)time(nullptr));
      diagAppendLog(line);
    }
  }
}

// To be called once at boot, after applyClockConfig() (the time zone must be set).
static void diagBootReport() {
  esp_reset_reason_t reason = esp_reset_reason();
  bool rtcValid = (rtcMagic == DIAG_RTC_MAGIC);
  String line = String("reboot cause=") + diagResetReasonText(reason);

  if (rtcValid) {
    bool stalled = (rtcStallFlag == 1);
    line += stalled ? " STALL_DETECTED=yes" : " stall=no";
    line += String(" phase=") + rtcLastPhase;
    line += " last_sign_of_life=" + diagFormatEpoch(rtcLastEpoch);
    line += " uptime_before=" + String(rtcLastUptimeS) + "s";
    line += " heapFree=" + String(rtcLastFreeHeap) + " heapMin=" + String(rtcLastMinHeap);
    line += " rssi=" + String((int)rtcLastRssi);
    if (rtcLastWebAgeS > 0) {
      char uri[24], ip[16];
      strncpy(uri, rtcLastWebUri, sizeof(uri) - 1); uri[sizeof(uri) - 1] = '\0';
      strncpy(ip,  rtcLastWebIp,  sizeof(ip)  - 1); ip[sizeof(ip) - 1]   = '\0';
      line += String(" web_in_progress=") + uri + " client=" + ip + " since=" + String(rtcLastWebAgeS) + "s";
    }
  } else {
    line += " (no previous state: power cut or first boot)";
    rtcStallCount = 0;
    rtcLastWebUri[0] = '\0';
    rtcLastWebIp[0]  = '\0';
    rtcLastWebAgeS   = 0;
  }
  wdNoteBoot(reason, rtcValid, rtcValid && rtcStallFlag == 1); // before the stall flag is reset
  rtcMagic = DIAG_RTC_MAGIC;
  rtcStallFlag = 0;
  line += String(" fw=") + FW_VERSION + " build=" + FW_BUILD;
  Serial.println("[Diag] " + line);
  diagAppendLog(line);
}

static void stallMonitorTask(void *) {
  uint32_t prevLoopMs = 0;   // diagLoopMs seen at the previous check
  uint8_t  frozenChecks = 0; // consecutive checks where loop() has not advanced at all
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(STALL_CHECK_MS));

    // IMPORTANT ORDER: first read the markers written by loop() (on the other core), and ONLY THEN millis().
    // Before, millis() was read first: if loop() wrote its marker in the following millisecond, (nowMs - marker)
    // was negative, hence ~4.29 billion as unsigned (see "since=4294967s" in /boot_log.txt) -> false "stall" -> reboot.
    uint32_t loopMs   = diagLoopMs;
    uint32_t webStart = diagWebStartMs;
    uint32_t nowMs    = millis();
    int32_t  loopAgeMs = (int32_t)(nowMs - loopMs);
    if (loopAgeMs < 0) loopAgeMs = 0;
    int32_t  webAgeMs  = webStart ? (int32_t)(nowMs - webStart) : 0;
    if (webAgeMs < 0) webAgeMs = 0;

    // Snapshot kept in RTC memory: this is what will be read after a restart.
    rtcLastUptimeS  = nowMs / 1000UL;
    rtcLastFreeHeap = ESP.getFreeHeap();
    rtcLastMinHeap  = ESP.getMinFreeHeap();
    rtcLastRssi     = WiFi.RSSI();
    time_t t = time(nullptr);
    if (t > 1600000000L) rtcLastEpoch = (uint32_t)t;
    strncpy(rtcLastPhase, diagPhase, sizeof(rtcLastPhase) - 1);
    rtcLastPhase[sizeof(rtcLastPhase) - 1] = '\0';
    strncpy(rtcLastWebUri, diagWebUri, sizeof(rtcLastWebUri) - 1);
    rtcLastWebUri[sizeof(rtcLastWebUri) - 1] = '\0';
    strncpy(rtcLastWebIp, diagWebIp, sizeof(rtcLastWebIp) - 1);
    rtcLastWebIp[sizeof(rtcLastWebIp) - 1] = '\0';
    rtcLastWebAgeS = (uint32_t)webAgeMs / 1000UL;

    // Double confirmation: real stall = loop() has written nothing for STALL_RESTART_MS AND its marker has not moved
    // since the previous check (a live loop() always changes its marker: never a false positive).
    if (loopMs == prevLoopMs) { if (frozenChecks < 255) frozenChecks++; } else frozenChecks = 0;
    prevLoopMs = loopMs;

    if (loopAgeMs > (int32_t)STALL_RESTART_MS && frozenChecks >= 2) {
      rtcStallFlag = 1;
      rtcStallCount++;
      Serial.printf("[Diag] loop() blocked for %lus (phase=%s web=%s client=%s) -> restart\n",
                    (unsigned long)(loopAgeMs / 1000UL), rtcLastPhase, rtcLastWebUri, rtcLastWebIp);
      delay(100);
      ESP.restart();
    }
  }
}

static void apiDiag() {
  String out = String("fw=") + FW_VERSION + " build=" + FW_BUILD + "\n";
  out += "uptime=" + String(millis() / 1000UL) + "s heapFree=" + String(ESP.getFreeHeap()) +
               " heapMin=" + String(ESP.getMinFreeHeap()) + " rssi=" + String(WiFi.RSSI()) +
               " stalls_since_power=" + String(rtcStallCount) + "\n\n";
  File f = LittleFS.open("/boot_log.txt", "r");
  if (f) { out += f.readString(); f.close(); }
  else out += "(no history)\n";
  out += "\n--- latest web requests (most recent first) ---\n";
  if (webRingCount == 0) out += "(none)\n";
  for (uint8_t i = 0; i < webRingCount; i++) {
    const WebReq &r = webRing[(webRingHead + WEB_RING - 1 - i) % WEB_RING];
    out += (r.epoch ? diagFormatEpoch(r.epoch) : String("--/-- --:--:--")) + "  " + r.uri + "  " + String(r.ms) + "ms  " + r.ip + "\n";
  }
  server.send(200, "text/plain; charset=utf-8", out);
}

/* ---------- /ant page: forecast tracking + settings (standalone, without touching index.html) ---------- */
static const char ANT_PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tank forecast</title>
<style>
:root{color-scheme:light dark;--b:#8884;--ok:#2a8f4a;--ko:#c0392b;--w:#d98a00}
body{font-family:system-ui,sans-serif;max-width:760px;margin:0 auto;padding:12px}
h1{font-size:1.25rem;margin:.2em 0}h2{font-size:1rem;margin:1.2em 0 .4em}
.card{border:1px solid var(--b);border-radius:10px;padding:10px 12px;margin:8px 0}
.row{display:flex;justify-content:space-between;gap:8px;padding:2px 0}
.big{font-size:1.15rem;font-weight:600}.ok{color:var(--ok)}.ko{color:var(--ko)}.w{color:var(--w)}
label{display:block;margin:.5em 0 .1em;font-size:.9rem}input,select,button{font:inherit;padding:6px;width:100%;box-sizing:border-box}
button{margin-top:10px;cursor:pointer}small{opacity:.7}.chk{display:flex;align-items:center;gap:.5em}.chk input{width:auto;padding:0;margin:0}
table{border-collapse:collapse;width:100%;font-size:.8rem}td,th{border-bottom:1px solid var(--b);padding:3px 4px;text-align:right}
th:first-child,td:first-child{text-align:left}.bar{display:flex;gap:1px;align-items:flex-end;height:60px}
.bar div{flex:1;background:#e0a800;min-width:2px}.bar div.n{background:#8886}
</style></head><body>
<h1>Tank forecast <small><a href="/">← home</a></small></h1>
<div class="card" id="st">Loading…</div>
<h2>Forecast sun (30 h)</h2><div class="card"><div class="bar" id="bar"></div><small>Yellow bars = hours when solar makes heating clearly cheaper than off-peak (HC) hours.</small></div>
<h2>Shading by hour of the day</h2><div class="card"><div class="bar" id="shbar"></div><div id="shlab" style="display:flex;gap:1px;font-size:.65em;text-align:center"></div><p id="shtxt" style="margin:.6em 0 0"></p><small>Bar = factor learned for that hour (full height = ×1.2, grey = not enough samples yet). Ratio actual / forecast, median over the last days: a tree shadow comes back every day at the same hour, a cloud does not. Learned and displayed all the time; used by the decision only if the box below is ticked.</small></div>
<h2>Learning</h2><div class="card" id="learn"></div>
<h2>Settings</h2>
<div class="card">
<label>Mode</label><select id="antMode">
<option value="off">off: classic full mode, nothing is computed</option>
<option value="shadow">shadow: computes and displays, does not act</option>
<option value="on">on: the plan drives the off-peak hours</option></select>
<label>Tank volume (litres)</label><input id="tankLiters" type="number" min="50" max="1000">
<label>Useful capacity (kWh, 0 = estimated from the volume)</label><input id="tankKWh" type="number" min="0" max="60" step="0.1"><small>Energy to heat the tank from cold water to the max temperature: litres × 1.163 × temperature difference in °C ÷ 1000 (datasheet).</small>
<label>Minimum hot water reserve (% of capacity)</label><input id="reservePct" type="number" min="0" max="90"><small>Level to keep before the sun takes over heating again.</small>
<label>Check before the evening (time)</label><input id="deadline" type="time"><small>At that time the tank does not have to be full: it must keep the reserve plus enough to last the evening until the off-peak hours. Otherwise the system heats just the shortfall, at the last moment.</small>
<label>Latitude / longitude (weather)</label><div class="row"><input id="lat" type="number" step="0.001"><input id="lon" type="number" step="0.001"></div>
<label class="chk"><input id="antShade" type="checkbox"> Apply the shading factor to the decision</label><small>Leave unticked until the curve above looks consistent from one hour to the next (needs clear days: not before spring).</small>
<button id="save">Save</button><small id="msg"></small></div>
<h2>Latest days</h2><div class="card"><div style="overflow-x:auto"><table id="tb"></table></div><p id="sc" style="margin:.6em 0 0"></p><small>kWh, except Cov. (h), °C and Rad. (Wh/m²). Gap = forecast − actual (+: the model overestimates, on the comfort side). Forecast = taken at the start of the cycle (24 h need, usable solar surplus over 24 h); Actual sol. = measured usable surplus.</small></div>
<script>
const $=i=>document.getElementById(i);let first=true;
const hm=m=>String(Math.floor(m/60)).padStart(2,'0')+':'+String(m%60).padStart(2,'0');
const at=e=>e?new Date(e*1000).toLocaleString('en-GB',{weekday:'short',hour:'2-digit',minute:'2-digit'}):'—';
const f1=v=>v===undefined||v===null?'—':v.toFixed(1);
/* Forecast judgement: over days whose actual need is usable AND whose forecast was recorded, mean error of the model
   compared with a naive baseline (mean of the 5 previous usable days, at least 3 available). */
function score(rs){
 const rows=[];
 rs.forEach((r,i)=>{if(!(r.need>0)||r.pred===undefined)return;
  const prev=rs.slice(0,i).filter(x=>x.need>0).slice(-5);if(prev.length<3)return;
  rows.push({m:r.pred-r.need,b:prev.reduce((s,x)=>s+x.need,0)/prev.length-r.need,n:r.need,ps:r.psol,pr:r.pot/1000})});
 if(!rows.length)return'No comparable day yet: the forecast is recorded at the start of each cycle, the first gap appears at the end of the first following usable cycle.';
 const avg=x=>x.reduce((s,v)=>s+v,0)/x.length,mm=avg(rows.map(x=>Math.abs(x.m))),mb=avg(rows.map(x=>Math.abs(x.b))),bias=avg(rows.map(x=>x.m));
 const pm=avg(rows.map(x=>Math.abs(x.m)/x.n))*100,pb=avg(rows.map(x=>Math.abs(x.b)/x.n))*100;
 const v=rows.length<5?'too early to conclude (5 days minimum)':mm<mb?'<span class="ok">better</span> than the baseline':'<span class="w">not yet better</span> than the baseline';
 const so=rows.filter(x=>x.ps!==undefined),sol=so.length?` Usable solar over ${so.length} d: forecast ${so.reduce((s,x)=>s+x.ps,0).toFixed(1)} kWh, actual ${so.reduce((s,x)=>s+x.pr,0).toFixed(1)} kWh.`:'';
 return `${rows.length} comparable day(s): mean model error ${mm.toFixed(1)} kWh (${pm.toFixed(0)} %), bias ${bias>=0?'+':''}${bias.toFixed(1)} kWh (+ = overestimates). "Mean of the previous 5 days" baseline: ${mb.toFixed(1)} kWh (${pb.toFixed(0)} %) → ${v}.${sol}`}
async function load(){
 let a;try{a=await(await fetch('/api/ant')).json()}catch(e){$('st').textContent='ESP unreachable';return}
 const dec=a.antMode==='off'?'—':!a.antValid?'<span class="w">computation impossible</span>':a.antForce?'<span class="w">catch-up in progress</span>':a.antHeat?'heating in HC':'<span class="ok">HC avoided</span>';
 $('st').innerHTML=
 `<div class="row"><span>Mode</span><span class="big">${a.antMode}</span></div>
  <div class="row"><span>Decision</span><span class="big">${dec}</span></div>
  <div class="row"><span>Reason</span><span>${a.antReason||''}</span></div>
  <div class="row"><span>Estimated tank deficit</span><span>${(a.antDeficitKWh||0).toFixed(1)} kWh / ${a.capKWh.toFixed(1)}</span></div>
  <div class="row"><span>Expected 24 h need</span><span>${(a.antNeedKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>To heat (plan)</span><span>${(a.antPlanKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>Expected solar</span><span>${(a.antSolarKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>HC that would have been avoided (this period)</span><span>${(a.antShadowSkipKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>Last "tank full"</span><span>${at(a.lastFullEpoch)}</span></div>
  <div class="row"><span>Weather</span><span class="${a.wxOk?'ok':'ko'}">${a.wxOk?('ok, '+a.wxAgeMin+' min ago'):('missing '+(a.wxErr||''))}${a.wxTemp!==undefined?' · '+a.wxTemp.toFixed(1)+' °C':''}</span></div>`;
 $('learn').innerHTML=
 `<div class="row"><span>Usable need days</span><span>${a.antNeedDays} (4 minimum)</span></div>
  <div class="row"><span>Solar coefficient k</span><span class="${a.antKValid?'ok':'w'}">${a.antKValid?a.k.toFixed(3)+' W per W/m²'+(a.kW<1?' (start '+a.kInit.toFixed(1)+', '+Math.round(a.kW*100)+' % learned)':''):'in progress ('+a.antKDays+' d, '+Math.round(a.kSumRad)+'/1500 Wh/m²)'}</span></div>
  <div class="row"><span>Temperature correction</span><span>${a.slopeValid?a.slopeWhPerC.toFixed(0)+' Wh/°C/day (ref. '+a.tRef.toFixed(1)+' °C)':'not calibrated yet'}</span></div>
  <div class="row"><span>Model / actual gap</span><span>×${a.errRatio.toFixed(2)}</span></div>`;
 {const sh=a.shade||{f:[],n:[]},rows=[];
  for(let u=0;u<24;u++){const d=new Date();d.setUTCHours(u,0,0,0);rows.push({lh:d.getHours(),f:(sh.f||[])[u],n:(sh.n||[])[u]||0})}
  const day=rows.filter(x=>x.lh>=6&&x.lh<=21).sort((x,y)=>x.lh-y.lh);
  $('shbar').innerHTML=day.map(x=>`<div class="${x.f>=0?'':'n'}" style="height:${x.f>=0?Math.max(3,Math.min(60,x.f*50)):(x.n?6:3)}px" title="${x.lh} h: ${x.f>=0?'×'+x.f.toFixed(2):'not enough samples'} (${x.n}/${sh.minN||7})"></div>`).join('');
  $('shlab').innerHTML=day.map(x=>`<span style="flex:1">${x.lh}</span>`).join('');
  const nv=day.filter(x=>x.f>=0).length;
  $('shtxt').innerHTML=(nv?`${nv} hour(s) with a factor`+(sh.ref!==undefined?`, reference level ×${sh.ref.toFixed(2)}`:'')+'. ':'No factor yet: at least '+(sh.minN||7)+' usable days per hour are needed (forecast ≥ 35 % of the tank power). ')+`Applied to the decision: <b>${a.antShade?'yes':'no'}</b>.`;}
 $('bar').innerHTML=(a.wx||[]).map(h=>`<div class="${h.u?'':'n'}" style="height:${Math.max(3,Math.min(60,h.r/12))}px" title="${at(h.e)} ${h.r} W/m²"></div>`).join('');
 $('tb').innerHTML='<tr><th>Cycle</th><th>Need</th><th>Forecast</th><th>Gap</th><th>Cov. h</th><th>°C</th><th>Rad.</th><th>Sol. fcst</th><th>Sol. actual</th><th>HC avoidable</th></tr>'+(a.recs||[]).slice(-14).reverse().map(r=>{const ok=r.need>0,ec=ok&&r.pred!==undefined?r.pred-r.need:undefined;return `<tr><td>${r.c%1000}</td><td>${ok?r.need.toFixed(1):'—'}</td><td>${f1(r.pred)}</td><td>${ec===undefined?'—':(ec>0?'+':'')+ec.toFixed(1)}</td><td>${r.cov.toFixed(0)}</td><td>${f1(r.t)}</td><td>${Math.round(r.rad)}</td><td>${f1(r.psol)}</td><td>${(r.pot/1000).toFixed(1)}</td><td>${r.sk.toFixed(1)}</td></tr>`}).join('');
 $('sc').innerHTML=score(a.recs||[]);
 if(first){first=false;$('antMode').value=a.antMode;$('antShade').checked=!!a.antShade;$('tankLiters').value=a.tankLiters;$('tankKWh').value=a.tankKWh;$('reservePct').value=a.reservePct;$('deadline').value=hm(a.deadlineMin);$('lat').value=a.lat;$('lon').value=a.lon}
}
$('save').onclick=async()=>{
 const [h,m]=$('deadline').value.split(':').map(Number);
 const body={antMode:$('antMode').value,antShade:$('antShade').checked,tankLiters:+$('tankLiters').value,tankKWh:+$('tankKWh').value,reservePct:+$('reservePct').value,deadlineMin:h*60+m,lat:+$('lat').value,lon:+$('lon').value};
 const r=await fetch('/api/config',{method:'POST',body:JSON.stringify(body)});
 $('msg').textContent=r.ok?' saved':' rejected: '+await r.text();load();
};
load();setInterval(load,10000);
</script></body></html>)rawliteral";

/* ============================================================
   7b. BACKUP / RESTORE (import of the raw JSON exports)
   ============================================================
   The raw exports (/api/history, /api/monthly/raw, /api/ant/raw) can be re-imported, for example
   after the flash has been erased. The import MERGES: an entry already present is never overwritten, and
   the current period (day / month) is only restored if the RAM counter is still empty. */
static const char *const IMP_U[5] = {"heatedSec", "relayOnSec", "tier1Sec", "tier2Sec", "nightSec"};
static const char *const IMP_F[6] = {"energyKWh", "nativeEUR", "paidKWh", "paidEUR", "savedKWh", "savedEUR"};

// cycleId = year*1000 + day of year (0-365); monthKey = year*100 + month (1-12)
static bool impKeyOk(uint32_t k, bool monthly) {
  if (monthly) return k >= 200001UL && k <= 210012UL && (k % 100UL) >= 1 && (k % 100UL) <= 12;
  return k >= 2000000UL && k <= 2100365UL && (k % 1000UL) <= 365;
}

// strict = imported (validated) entry; otherwise lenient reading of the existing file (a row is never lost).
static bool impReadRow(ImpRow &r, JsonVariantConst o, const char *keyName, bool monthly, bool strict) {
  if (!o.is<JsonObjectConst>()) return false;
  r.key = (uint32_t)(o[keyName] | 0);
  if (strict ? !impKeyOk(r.key, monthly) : r.key == 0) return false;
  for (int i = 0; i < 5; ++i) {
    long v = o[IMP_U[i]] | 0;
    if (strict && (v < 0 || v > 3000000L)) return false;
    r.u[i] = v < 0 ? 0 : (uint32_t)v;
  }
  for (int i = 0; i < 6; ++i) {
    float v = (float)(o[IMP_F[i]] | 0.0f);
    if (!std::isfinite(v)) v = 0.0f;
    if (strict && fabsf(v) > 1.0e6f) return false;
    r.f[i] = v;
  }
  const char *m = o["mode"] | "auto";
  bool known = !strcmp(m, "auto") || !strcmp(m, "full") || !strcmp(m, "manual") || !strcmp(m, "monitor");
  strncpy(r.mode, known || !strict ? m : "auto", sizeof(r.mode) - 1);
  r.mode[sizeof(r.mode) - 1] = 0;
  return true;
}

static void impLoadRows(const char *path, const char *arrName, const char *keyName, bool monthly, std::vector<ImpRow> &rows) {
  File in = LittleFS.open(path, "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  if (err || !d[arrName].is<JsonArray>()) return;
  for (JsonVariantConst v : d[arrName].as<JsonArrayConst>()) {
    ImpRow r;
    if (impReadRow(r, v, keyName, monthly, false)) rows.push_back(r);
  }
}

static void impSendResult(int added, int skipped, int rejected, size_t total) {
  JsonDocument d;
  d["ok"] = true; d["added"] = added; d["skipped"] = skipped; d["rejected"] = rejected; d["total"] = (uint32_t)total;
  String s;
  serializeJson(d, s);
  server.send(200, "application/json", s);
}

// No fixed cap on the file size: the real limit is the memory available at the time of the
// import, which varies over time (purges, fragmentation). So it is checked HERE rather than refusing
// in advance a file that could have passed. server.arg("plain") has already copied the whole body into RAM before
// arriving here (WebServer library limit for a raw, non-multipart JSON body): an import of
// several MB may therefore remain out of reach as long as the client side (index.html) does not split the upload
// into a stream; but removing the arbitrary cap already allows importing much larger files than before,
// and adaptively rather than fixed at a number chosen in advance.
static bool impPrepare(String &body, JsonDocument &in, const char *arrName) {
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "POST expected"); return false; }
  body = server.arg("plain");
  size_t len = body.length();
  if (len < 10) { server.send(413, "text/plain", "Empty or unreadable file"); return false; }
  size_t freeHeap = (size_t)ESP.getMaxAllocHeap();
  // Margin: building the JSON tree (ArduinoJson) adds memory as parsing progresses,
  // on top of the body already in RAM; a fixed 20 KB margin + twice the file size covers the
  // real cases observed on this project without being an arbitrary number unrelated to the imported file.
  if (freeHeap < 50000 || freeHeap < len + 20000) {
    server.send(503, "text/plain",
      "Not enough memory for this file right now (" + String(len / 1024) + " Ko, " +
      String(freeHeap / 1024) + " KB free): try again after restarting the ESP, or outside a load peak");
    return false;
  }
  DeserializationError err = deserializeJson(in, body);
  if (err || !in[arrName].is<JsonArray>()) {
    server.send(400, "text/plain", String("Invalid file: array \"") + arrName + "\" expected");
    return false;
  }
  body = String(); // frees the text copy before building the result
  return true;
}

// Merge of days (history) or months (monthly recap).
static void apiImportRows(bool monthly) {
  const char *arrName = monthly ? "months" : "days";
  const char *keyName = monthly ? "monthKey" : "cycleId";
  const char *path    = monthly ? "/monthly.json" : "/history.json";
  const size_t maxN   = monthly ? (size_t)MONTHLY_HISTORY_MAX : 31;
  String body;
  JsonDocument in;
  if (!impPrepare(body, in, arrName)) return;

  std::vector<ImpRow> rows;
  impLoadRows(path, arrName, keyName, monthly, rows);

  uint32_t liveKey = monthly ? monthKeyForToday() : rt.cycleId;
  bool liveEmpty = monthly ? (rt.monthCycleWh <= 0.0f && rt.monthHeatedSec == 0)
                           : (rt.cycleWh <= 0.0f && rt.heatedSec == 0 && rt.relayOnSec == 0);
  int added = 0, skipped = 0, rejected = 0, fileAdded = 0;
  bool restoreLive = false;
  bool liveDay = false;   // current day to merge into the RAM counters (see below)
  ImpRow liveRow;
  for (JsonVariantConst v : in[arrName].as<JsonArrayConst>()) {
    ImpRow r;
    if (!impReadRow(r, v, keyName, monthly, true)) { rejected++; continue; }
    if (liveKey != 0 && r.key > liveKey) { rejected++; continue; }       // future date: different clock?
    /* CURRENT day: at boot the system immediately writes a row for today (counters at zero), so
       "already present" would always be true and the saved day could never come back after a flash erase.
       Rule: "never downwards" merge. The file is only taken if it is MORE COMPLETE than the current counters,
       and each counter then keeps the larger of the two values (no sum: no double counting if imported twice,
       or if the day had already been restored automatically at boot). Only one current day per file. */
    if (!monthly && liveKey != 0 && r.key == liveKey) {
      bool richer = r.u[0] > rt.heatedSec || r.u[1] > rt.relayOnSec || r.f[0] * 1000.0f > rt.cycleWh + 1.0f;
      if (!richer || liveDay) { skipped++; continue; }
      liveDay = true; liveRow = r; added++;
      continue;
    }
    bool exists = false;
    for (const ImpRow &x : rows) if (x.key == r.key) { exists = true; break; }
    if (exists || (r.key == liveKey && !liveEmpty)) { skipped++; continue; }
    rows.push_back(r);
    added++; fileAdded++;
    if (r.key == liveKey) restoreLive = true;
  }
  if (added == 0) { impSendResult(0, skipped, rejected, rows.size()); return; }

  if (fileAdded > 0) {
    std::sort(rows.begin(), rows.end(), [](const ImpRow &a, const ImpRow &b) { return a.key < b.key; });
    while (rows.size() > maxN) rows.erase(rows.begin());

    JsonDocument out;
    JsonArray arr = out[arrName].to<JsonArray>();
    for (const ImpRow &r : rows) {
      JsonObject o = arr.add<JsonObject>();
      o[keyName] = r.key;
      for (int i = 0; i < 5; ++i) o[IMP_U[i]] = r.u[i];
      for (int i = 0; i < 6; ++i) o[IMP_F[i]] = r.f[i];
      if (!monthly) o["mode"] = r.mode;
    }
    if (!writeJsonAtomic(path, out)) {
      reportError(String("Import: failed to write ") + path, ERROR_STORAGE);
      server.send(500, "text/plain", "Unable to write (memory full?)");
      return;
    }
    if (restoreLive) { if (monthly) restoreCurrentMonth(); else restoreCurrentHistory(); }
  }
  if (liveDay) {   // current day: RAM counters = max(current, imported), then write the day's row
    if (liveRow.u[0] > rt.heatedSec)   rt.heatedSec   = liveRow.u[0];
    if (liveRow.u[1] > rt.relayOnSec)  rt.relayOnSec  = liveRow.u[1];
    if (liveRow.u[2] > rt.tier1Sec)    rt.tier1Sec    = liveRow.u[2];
    if (liveRow.u[3] > rt.tier2Sec)    rt.tier2Sec    = liveRow.u[3];
    if (liveRow.u[4] > rt.nightSec)    rt.nightSec    = liveRow.u[4];
    if (liveRow.f[0] * 1000.0f > rt.cycleWh) rt.cycleWh = liveRow.f[0] * 1000.0f;
    if (liveRow.f[1] > rt.nativeEUR)         rt.nativeEUR = liveRow.f[1];
    if (liveRow.f[2] * 1000.0f > rt.paidWh)  rt.paidWh = liveRow.f[2] * 1000.0f;
    if (liveRow.f[3] > rt.paidEUR)           rt.paidEUR = liveRow.f[3];
    saveHistory();
  }
  Serial.printf("[Import] %s : +%d, %d ignored, %d rejected\n", path, added, skipped, rejected);
  impSendResult(added, skipped, rejected, rows.size());
}
void apiImportHistory() { apiImportRows(false); }
void apiImportMonthly() { apiImportRows(true); }

// Raw export of the monthly recap: only the recorded months (without the current day that /api/monthly
// adds for display), hence re-importable without duplicates.
void apiMonthlyRaw() {
  File f = LittleFS.open("/monthly.json", "r");
  if (!f) { server.send(200, "application/json", "{\"months\":[]}"); return; }
  server.streamFile(f, "application/json");
  f.close();
}

// Learned habits (daily need, solar, temperature per cycle).
void apiImportAnt() {
  String body;
  JsonDocument in;
  if (!impPrepare(body, in, "recs")) return;
  std::vector<AntRec> rows(antRecs, antRecs + antNRec);
  int added = 0, skipped = 0, rejected = 0;
  for (JsonVariantConst v : in["recs"].as<JsonArrayConst>()) {
    AntRec r;
    r.cycle  = (uint32_t)(v["c"] | 0);
    r.wday   = (uint8_t)(v["w"] | 0);
    r.sec    = (uint32_t)(v["s"] | 0);
    r.wh     = (float)(v["wh"] | 0.0f);
    r.rad    = (float)(v["r"] | 0.0f);
    r.pot    = (float)(v["p"] | 0.0f);
    r.skipWh = (float)(v["sk"] | 0.0f);
    r.t10    = v["t"].is<int>() ? (int16_t)v["t"].as<int>() : INT16_MIN;
    r.predWh    = (float)(v["pn"] | -1.0f);
    r.predSolWh = (float)(v["ps"] | -1.0f);
    if (!(r.predWh >= 0.0f))    r.predWh = -1.0f;    // optional fields (old exports); includes NaN
    if (!(r.predSolWh >= 0.0f)) r.predSolWh = -1.0f;
    bool ok = impKeyOk(r.cycle, false) && r.wday <= 6 && r.sec <= 200000UL &&
              std::isfinite(r.wh) && r.wh >= 0.0f && r.wh <= 1.0e6f &&
              std::isfinite(r.rad) && r.rad >= 0.0f && r.rad <= 50000.0f &&
              std::isfinite(r.pot) && r.pot >= 0.0f && r.pot <= 1.0e6f &&
              std::isfinite(r.skipWh) && r.skipWh >= 0.0f && r.skipWh <= 1.0e6f &&
              r.predWh <= 1.0e6f && r.predSolWh <= 1.0e6f &&
              (r.t10 == INT16_MIN || (r.t10 >= -600 && r.t10 <= 600));
    if (!ok) { rejected++; continue; }
    bool exists = false;
    for (const AntRec &x : rows) if (x.cycle == r.cycle) { exists = true; break; }
    if (exists) { skipped++; continue; }
    rows.push_back(r);
    added++;
  }
  int hrN = in["hr"].is<JsonObject>() ? antHrMerge(in["hr"]) : 0; // old exports: no "hr", no effect
  if (hrN > 0) antHrSave();
  if (added > 0) {
    std::sort(rows.begin(), rows.end(), [](const AntRec &a, const AntRec &b) { return a.cycle < b.cycle; });
    while (rows.size() > ANT_REC_MAX) rows.erase(rows.begin());
    antNRec = (uint8_t)rows.size();
    for (uint8_t i = 0; i < antNRec; ++i) antRecs[i] = rows[i];
    antSave();
    antRefresh();
  }
  Serial.printf("[Import] habits : +%d, %d ignored, %d rejected, forecast/actual hours touched=%d\n", added, skipped, rejected, hrN);
  impSendResult(added, skipped, rejected, antNRec);
}

// Heating log import (POST /api/import/heat, file exported by the page: {"segs":[[t0,sec,wh,paidWh,reason],...]}).
// Merge without overwriting: a block that overlaps in time with an existing block is ignored (so the same file can be
// imported twice without duplicates). The HEAT_SEG_MAX most recent are kept.
void apiImportHeat() {
  String body;
  JsonDocument in;
  if (!impPrepare(body, in, "segs")) return;
  time_t ne = time(nullptr);
  uint32_t maxT = ne >= 946684800 ? (uint32_t)ne + 86400UL : 4102444800UL;
  std::vector<HeatSeg> rows(heatSegs, heatSegs + heatNSeg);
  int added = 0, skipped = 0, rejected = 0;
  for (JsonVariantConst v : in["segs"].as<JsonArrayConst>()) {
    if (!v.is<JsonArrayConst>()) { rejected++; continue; }
    JsonArrayConst r = v.as<JsonArrayConst>();
    uint32_t t0 = r[0] | 0UL, sec = r[1] | 0UL, wh = r[2] | 0UL, pd = r[3] | 0UL, rs = r[4] | 0UL;
    // Widened limits (previously 65535, the uint16 limit lifted by this fix): anti-corrupted-data guard,
    // not a physical limit. 604800 s = 7 d, 1e6 Wh = 1 MWh.
    bool ok = t0 >= 946684800UL && t0 <= maxT && sec <= 604800UL && wh >= 1UL && wh <= 1000000UL && pd <= 1000000UL && rs <= 2UL;
    if (!ok) { rejected++; continue; }
    bool overlap = false;
    for (const HeatSeg &x : rows) {
      if (t0 <= x.t0 + x.sec && x.t0 <= t0 + sec) { overlap = true; break; }
    }
    if (overlap) { skipped++; continue; }
    HeatSeg h;
    h.t0 = t0; h.sec = sec; h.wh = wh; h.paidWh = pd; h.reason = (uint8_t)rs;
    rows.push_back(h);
    added++;
  }
  // "Relay" line (optional, absent from old exports): same merge without overwriting.
  std::vector<RelayIv> rrows(relayIvs, relayIvs + relayNIv);
  int relAdded = 0;
  if (in["rel"].is<JsonArrayConst>()) {
    for (JsonVariantConst v : in["rel"].as<JsonArrayConst>()) {
      if (!v.is<JsonArrayConst>()) continue;
      JsonArrayConst r = v.as<JsonArrayConst>();
      uint32_t t0 = r[0] | 0UL, sec = r[1] | 0UL;
      if (t0 < 946684800UL || t0 > maxT || sec == 0UL || sec > 604800UL) continue; // same guard as heatLoad()
      bool overlap = false;
      for (const RelayIv &x : rrows) {
        if (t0 <= x.t0 + x.sec && x.t0 <= t0 + sec) { overlap = true; break; }
      }
      if (overlap) continue;
      RelayIv n; n.t0 = t0; n.sec = sec;
      rrows.push_back(n);
      relAdded++;
    }
  }
  if (added > 0) {
    std::sort(rows.begin(), rows.end(), [](const HeatSeg &a, const HeatSeg &b) { return a.t0 < b.t0; });
    while (rows.size() > HEAT_SEG_MAX) rows.erase(rows.begin());
    heatNSeg = (uint16_t)rows.size();
    for (uint16_t i = 0; i < heatNSeg; ++i) heatSegs[i] = rows[i];
  }
  if (relAdded > 0) {
    std::sort(rrows.begin(), rrows.end(), [](const RelayIv &a, const RelayIv &b) { return a.t0 < b.t0; });
    while (rrows.size() > RELAY_IV_MAX) rrows.erase(rrows.begin());
    relayNIv = (uint16_t)rrows.size();
    for (uint16_t i = 0; i < relayNIv; ++i) relayIvs[i] = rrows[i];
  }
  if (added > 0 || relAdded > 0) heatSave();
  Serial.printf("[Import] heating log : +%d, %d ignored, %d rejected, relay +%d\n", added, skipped, rejected, relAdded);
  impSendResult(added, skipped, rejected, heatNSeg);
}

/* ============================================================
   8. NETWORK & INITIALISATION
   ============================================================ */
constexpr const char *AP_SSID = "Ballon-ESP32";
constexpr const char *AP_PASS = "Ballon_?";

static void wifiApplyRadio() {
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

constexpr uint32_t AP_RESCUE_AFTER_MS     = 60000;  // Wi-Fi lost for this long: the rescue AP opens
constexpr uint32_t AP_RESCUE_CLOSE_AFTER_MS = 120000; // Wi-Fi stable for this long: the rescue AP closes
constexpr uint32_t BOOT_AP_DELAY_MS       = 90000;  // no diagnostic AP before: boot is still initialising
constexpr uint32_t BOOT_AP_GIVEUP_MS      = 600000; // if Wi-Fi/Ecojoko/Shelly are not healthy before then, no diagnostic AP for this boot
constexpr uint32_t BOOT_AP_WIFI_STABLE_MS = 30000;  // Wi-Fi stable for this long before opening the diagnostic AP
constexpr uint32_t BOOT_AP_MIN_HEAP       = 70000;  // minimum free RAM to OPEN the diagnostic AP (the AP costs a few KB and the Ecojoko TLS must keep its margin)
constexpr uint32_t BOOT_AP_CLOSE_HEAP     = 50000;  // below this free RAM, the diagnostic AP is closed

static void monitorWifiConnection(uint32_t now) {
  if ((uint32_t)(now - rt.lastWifiCheckMs) < WIFI_CHECK_PERIOD_MS) return;
  rt.lastWifiCheckMs = now;

  if (WiFi.status() == WL_CONNECTED) {
    if (rt.wifiDisconnectedSinceMs != 0) {
      Serial.printf("[WiFi] connection recovered rssi=%d time=%s\n", WiFi.RSSI(), nowText().c_str());
    }
    rt.wifiDisconnectedSinceMs = 0;
    return;
  }

  if (!cfg.wifiSsid.length()) return;
  if (rt.wifiDisconnectedSinceMs == 0) {
    rt.wifiDisconnectedSinceMs = now;
    Serial.printf("[WiFi] connection lost status=%d rssi=%d time=%s\n",
                  (int)WiFi.status(), WiFi.RSSI(), nowText().c_str());
    ecoTlsReset(); // The persistent TLS socket died with the Wi-Fi link.
    return;
  }

  if ((uint32_t)(now - rt.wifiDisconnectedSinceMs) < WIFI_RECONNECT_AFTER_MS) return;
  Serial.printf("[WiFi] forced reconnection status=%d rssi=%d time=%s\n",
                (int)WiFi.status(), WiFi.RSSI(), nowText().c_str());
  ecoTlsReset();
  if (rt.apRescue || rt.bootApOn) {
    // An AP is open (rescue or diagnostic): it must stay reachable during the attempts.
    WiFi.mode(WIFI_AP_STA);
    if (WiFi.softAPIP() == IPAddress((uint32_t)0)) WiFi.softAP(AP_SSID, AP_PASS); // the AP is (re)configured explicitly
  } else if (WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_STA); // normal operation: no AP, the whole radio serves the link with the router
  }
  WiFi.disconnect();
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
  wifiApplyRadio();
  rt.wifiDisconnectedSinceMs = now;
}

// Does the gateway answer? An immediate connection refusal (RST) proves it is reachable;
// only a full timeout without answer signals a wrong IP/gateway/mask.
static bool gatewayReachable(const IPAddress &gw) {
  WiFiClient c;
  uint32_t t0 = millis();
  bool ok = c.connect(gw, 80, 2000);
  uint32_t el = millis() - t0;
  c.stop();
  return ok || el < 1500;
}

static bool waitWifi(uint8_t tries) {
  uint8_t count = 0;
  while (WiFi.status() != WL_CONNECTED && count < tries) { delay(500); count++; }
  return WiFi.status() == WL_CONNECTED;
}

static void apOpen() {
  // Same channel as the router link if it is active: the radio is shared, another channel could break the link.
  int ch = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 1;
  if (ch < 1 || ch > 13) ch = 1;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, ch);
  wifiApplyRadio();
}

static void apClose() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  wifiApplyRadio();
}

/* Two rules, in this order:
   1. Wi-Fi problem => rescue AP. Opened at boot if the connection fails (see connectNetwork), or after
      AP_RESCUE_AFTER_MS of lost Wi-Fi. Closed only when Wi-Fi has been stable for AP_RESCUE_CLOSE_AFTER_MS.
   2. Wi-Fi, Ecojoko and Shelly healthy, a few tens of seconds after boot => diagnostic AP for
      cfg.bootApSec. Closed at expiry, or as soon as Ecojoko/Shelly have a problem (the radio serves the business logic).
   In both cases: never closed while a device is connected to the AP, and a lost Wi-Fi turns the
   diagnostic AP into a rescue AP. */
static void serviceAp(uint32_t now) {
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn) { if (rt.wifiUpSinceMs == 0) rt.wifiUpSinceMs = now ? now : 1; rt.wifiLostSinceMs = 0; }
  else      { if (rt.wifiLostSinceMs == 0) rt.wifiLostSinceMs = now ? now : 1; rt.wifiUpSinceMs = 0; }

  // Rule 1: Wi-Fi problem.
  if (!conn && cfg.wifiSsid.length() > 0 && !rt.apRescue && rt.bootApOn) { // the diagnostic AP becomes a rescue AP
    rt.bootApOn = false; rt.apRescue = true;
  } else if (!conn && cfg.wifiSsid.length() > 0 && !rt.apRescue &&
             (uint32_t)(now - rt.wifiLostSinceMs) >= AP_RESCUE_AFTER_MS) {
    apOpen();
    rt.apRescue = true;
    Serial.println("[WiFi] Wi-Fi lost: rescue AP opened");
  }
  if (rt.apRescue) {
    if (conn && (uint32_t)(now - rt.wifiUpSinceMs) >= AP_RESCUE_CLOSE_AFTER_MS && WiFi.softAPgetStationNum() == 0) {
      apClose();
      rt.apRescue = false;
      Serial.println("[WiFi] Wi-Fi stable: rescue AP closed");
    }
    return;
  }

  // Rule 2: diagnostic AP.
  bool svcOk = rt.ecoConnected && rt.shellyConnected && rt.ecoFailCount == 0 && rt.shellyFailCount == 0;
  if (!rt.bootApOn) {
    if (rt.bootApDone || cfg.bootApSec == 0) return;
    if (now >= BOOT_AP_GIVEUP_MS) { rt.bootApDone = true; return; }
    if (now >= BOOT_AP_DELAY_MS && conn && (uint32_t)(now - rt.wifiUpSinceMs) >= BOOT_AP_WIFI_STABLE_MS && svcOk &&
        ESP.getFreeHeap() >= BOOT_AP_MIN_HEAP) {
      apOpen();
      rt.bootApOn = true; rt.bootApDone = true; rt.bootApStartMs = now;
      Serial.printf("[WiFi] Diagnostic AP opened for %us heapFree=%u\n", (unsigned)cfg.bootApSec, (unsigned)ESP.getFreeHeap());
    }
    return;
  }
  bool expired = (uint32_t)(now - rt.bootApStartMs) >= (uint32_t)cfg.bootApSec * 1000UL;
  bool svcErr = conn && (rt.shellyFailCount > 0 || rt.ecoFailCount > 0 || ESP.getFreeHeap() < BOOT_AP_CLOSE_HEAP);
  if ((expired || svcErr) && WiFi.softAPgetStationNum() == 0) {
    rt.bootApOn = false;
    apClose();
    Serial.printf("[WiFi] Diagnostic AP closed (%s)\n", expired ? "delay elapsed" : "Ecojoko/Shelly error");
  }
}

void connectNetwork() {
  WiFi.persistent(false); // credentials come from config.json: no copy in the Wi-Fi flash (wear, old "ghost" AP)
  rt.apRescue = false;
  if (cfg.wifiSsid.length() > 0) {
    WiFi.mode(WIFI_STA);
    IPAddress ip, gw, mask, dns;
    bool useStatic = false;
    if (cfg.staticIp && parseIp(cfg.ip, ip) && parseIp(cfg.gw, gw) && parseIp(cfg.mask, mask)) {
      if (!parseIp(cfg.dns, dns)) dns = gw;
      useStatic = WiFi.config(ip, gw, mask, dns);
    }
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
    wifiApplyRadio();
    waitWifi(20);
    if (useStatic && WiFi.status() == WL_CONNECTED && !gatewayReachable(gw)) {
      // static IP unusable (gateway unreachable): back to DHCP, otherwise the ESP32 would be unreachable.
      Serial.println("[WiFi] Static IP unusable: DHCP fallback");
      rt.staticFallback = true;
      WiFi.disconnect();
      WiFi.config(IPAddress((uint32_t)0), IPAddress((uint32_t)0), IPAddress((uint32_t)0));
      WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
      wifiApplyRadio();
      waitWifi(20);
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    // Rule 1: Wi-Fi missing or wrong (first setup, typo...) => rescue AP right away.
    rt.apRescue = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }
}

/* ============================================================
   9. LINK WITH THE "WATCHDOG" ESP32-C3 (watchdog_c3/watchdog_c3.ino)
   ============================================================
   Two wires to the C3 (3.3 V, common ground, C3 powered from VIN):
   - HB  (heartbeat): changes state every second as long as loop() runs AND the control loop
         (tick) is reached. Freeze, crash, restart loop: no more regular heartbeat,
         the C3 sounds on its own (it needs no information from this program).
   - ERR (alarm): goes to 1 when a problem that compromises heating is CONFIRMED (see wdEvaluate),
         falls back to 0 as soon as it disappears. The C3 sounds as long as ERR stays at 1 (unless acknowledged on the C3).
   No return link: this program does not know whether the C3 is sounding or even whether it is connected.
   - TX (serial 9600 baud, UART2, one-way): sends the C3 frames "$M<minutes>*<XX>" = "stay silent <minutes> minutes".
         The C3 counts this silence itself (see watchdog_c3.ino): even if this program freezes, it will sound at expiry.
         The silence is the larger of: the quiet window (cfg.muteOn/muteStart/muteEnd, rest of the window) and the remote
         silence (/api/snooze, cfg.snoozeMin). The remaining duration is resent every 30 s.
         NEVER any silence during overheating (WD_OVERHEAT) nor during the buzzer test: "M0" frame repeated every 2 s.
         Without a TX pin (-1), nothing is sent: the alarm always sounds.
   The "monitor" mode never alerts: the tank is deliberately not controlled in it. */
constexpr uint32_t WD_HB_PERIOD_MS      = 1000;   // HB toggle
constexpr uint32_t WD_TICK_MAX_AGE_MS   = 120000; // HB stops if tick() has not been reached for 2 min (legitimate worst case ~65 s)
constexpr uint32_t WD_FAST_DELAY_MS     = 120000; // relay open + power measured, memory almost empty: alert after 2 min
constexpr uint32_t WD_LOW_HEAP_BYTES    = 12000;
constexpr uint32_t WD_CMD_FAIL_GAP_MS   = 180000; // Shelly command failures considered continuous as long as a new failure arrives within 3 min
constexpr uint32_t WD_TEST_MS           = 8000;   // the C3 confirms ERR after 3 s: about 5 s of ringing remain
constexpr uint32_t WD_CRASH_WINDOW_S    = 3600;
constexpr uint32_t WD_CRASH_MAX         = 3;      // 3 crashes in less than 1 h = restart loop
constexpr uint32_t WD_MUTE_PERIOD_MS    = 30000;  // the remaining silence duration is resent to the C3 every 30 s
constexpr uint32_t WD_MUTE_BURST_MS     = 1000;   // frames sent in quick succession after an action (silence, "Re-enable", window change)
constexpr uint32_t WD_CANCEL_PERIOD_MS  = 2000;   // silence cancellation repeated during overheating or a test
constexpr uint16_t WD_MUTE_MAX_MIN      = 720;    // cap (the C3 applies the same one)

// WdCause is declared at the top of the file (see above)

static const char *wdCauseName(WdCause c) {
  switch (c) {
    case WD_CRASH_LOOP: return "crashes";
    case WD_RELAY:      return "relay";
    case WD_MEMORY:     return "memory";
    case WD_SHELLY:     return "shelly";
    case WD_COMMAND:    return "command";
    case WD_CLOCK:      return "clock";
    case WD_OVERHEAT:   return "overheat";
    default:            return "ok";
  }
}

struct WdState {
  int8_t   hbPin = -1, errPin = -1, txPin = -1;
  bool     hbLevel = false, errLevel = false;
  uint32_t lastHbMs = 0, lastEvalMs = 0, testUntilMs = 0;
  uint32_t relaySince = 0, memSince = 0, shellySince = 0, clockSince = 0; // 0 = condition absent
  uint32_t overheatSince = 0, lostHotSince = 0;
  uint32_t cmdFailSince = 0, cmdFailLast = 0;
  uint32_t snoozeUntilMs = 0, lastMuteTxMs = 0; // remote silence (RAM); last send to the C3
  uint8_t  sendBursts = 0;                     // remaining quick sends
  bool     crit = false;                       // an overheating cause is in alarm (never any silence), even if another cause is shown
  bool     alarm = false;
  WdCause  cause = WD_OK, pending = WD_OK;
  uint32_t pendingMs = 0, pendingLimitMs = 0;
};
static WdState wd;

bool wdPinAllowed(int p) {
  if (p == -1) return true;
  static const int8_t allowed[] = {4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
  for (int8_t v : allowed) if (v == p) return true;
  return false;
}

void wdSetPins(int8_t hb, int8_t err) {
  if (wd.hbPin != hb) {
    if (wd.hbPin >= 0) pinMode(wd.hbPin, INPUT);
    wd.hbPin = hb;
    wd.hbLevel = false;
    if (hb >= 0) { pinMode(hb, OUTPUT); digitalWrite(hb, LOW); }
  }
  if (wd.errPin != err) {
    if (wd.errPin >= 0) pinMode(wd.errPin, INPUT);
    wd.errPin = err;
    wd.errLevel = false;
    if (err >= 0) { pinMode(err, OUTPUT); digitalWrite(err, LOW); }
  }
}

// Silence serial link: UART2 transmit-only (RX = -1: no pin reserved for reception).
void wdSetTxPin(int8_t tx) {
  if (tx == wd.txPin) return;
  if (wd.txPin >= 0) { Serial2.end(); pinMode(wd.txPin, INPUT); }
  wd.txPin = tx;
  if (tx >= 0) Serial2.begin(9600, SERIAL_8N1, -1, tx);
}

void wdInit() { wdSetPins(cfg.wdHbPin, cfg.wdErrPin); wdSetTxPin(cfg.wdTxPin); }

// Duration (ms) for which `cond` has been true without interruption; since == 0 when it is false.
static uint32_t wdTrack(bool cond, uint32_t &since, uint32_t now) {
  if (!cond) { since = 0; return 0; }
  if (since == 0) since = now ? now : 1;
  return (uint32_t)(now - since);
}

// Called by shellySet(): final failure (after all attempts) or confirmed success.
void wdNoteCommand(bool ok) {
  uint32_t now = millis();
  if (ok) { wd.cmdFailSince = 0; wd.cmdFailLast = 0; return; }
  if (wd.cmdFailSince == 0 || (uint32_t)(now - wd.cmdFailLast) > WD_CMD_FAIL_GAP_MS) wd.cmdFailSince = now ? now : 1;
  wd.cmdFailLast = now;
}

// Called once by diagBootReport(), before the stall flag is reset.
// Only REAL crashes count (panic, watchdogs, detected stall, brownout):
// an intended restart (web button, Wi-Fi change, reflash) triggers nothing.
void wdNoteBoot(esp_reset_reason_t reason, bool rtcValid, bool stalled) {
  if (!rtcValid || rtcCrashCount > 1000) { rtcCrashCount = 0; rtcCrashSpanS = 0; return; }
  bool crashed = stalled || reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
                 reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT;
  if (!crashed) return;
  if (rtcCrashCount == 0) { rtcCrashCount = 1; rtcCrashSpanS = 0; return; }
  rtcCrashSpanS += rtcLastUptimeS + 20; // uptime of the previous session + approximate boot duration
  if (rtcCrashSpanS > WD_CRASH_WINDOW_S) { rtcCrashCount = 1; rtcCrashSpanS = 0; }
  else rtcCrashCount++;
}

static bool wdCrashLoopActive() {
  return rtcCrashCount >= WD_CRASH_MAX && (rtcCrashSpanS + millis() / 1000UL) <= WD_CRASH_WINDOW_S;
}

/* ---------- SOS log (/sos_log.json) ----------
   One entry per confirmed alarm (ERR goes to 1): time, cause, numeric explanation, and a snapshot of the system state
   (mode, Wi-Fi, memory, loop() phase, Shelly, Ecojoko, last error message, silence in progress).
   Written only on alarm state changes (not on every tick): very little flash wear.
   "dur" absent = alarm in progress; "reboot":true = the ESP restarted during the alarm (end unknown).
   Purge: an alarm that ended more than 72 h ago is deleted (on every write, on every read, then every hour).
   If the C3 sounds because the main ESP is frozen (no more heartbeat), nothing can be written at that moment: see /api/diag. */
static uint32_t sosOpenMs = 0;       // millis() at the start of the current alarm (0 = none): used to compute the duration
static uint32_t sosLastPruneMs = 0;
static uint16_t wdMuteLeftMin(uint32_t now); // defined further down

static void sosRead(JsonDocument &doc) {
  File f = LittleFS.open(SOS_PATH, "r");
  if (f) {
    DeserializationError e = deserializeJson(doc, f);
    f.close();
    if (e) doc.clear();
  }
  if (!doc["events"].is<JsonArray>()) doc["events"].to<JsonArray>();
}

// Deletes the alarms ended more than SOS_KEEP_S ago (valid clock required) and brings the list down to the cap. true = something changed.
static bool sosPrune(JsonArray arr) {
  bool changed = false;
  const time_t nowEp = time(nullptr);
  if (nowEp >= 946684800 && (uint32_t)nowEp > SOS_KEEP_S) {
    const uint32_t limit = (uint32_t)nowEp - SOS_KEEP_S;
    for (int i = (int)arr.size() - 1; i >= 0; i--) {
      JsonObject o = arr[i];
      const bool active = o["dur"].isNull() && !(o["reboot"] | false);   // alarm in progress: never deleted
      const uint32_t t = o["t"] | 0;
      if (active || t == 0) continue;
      if (t + (uint32_t)(o["dur"] | 0) < limit) { arr.remove(i); changed = true; }
    }
  }
  while (arr.size() > SOS_MAX_ENTRIES) { arr.remove(0); changed = true; }
  return changed;
}

static String sosDetail(WdCause c, uint32_t elapsedMs, bool tKnown) {
  const String mn = String((unsigned long)(elapsedMs / 60000UL));
  switch (c) {
    case WD_CRASH_LOOP: return String("crash loop: ") + String((unsigned)rtcCrashCount) + " crashes in under 1 h";
    case WD_OVERHEAT:
      if (tKnown && cfg.shellyTAlarmC > 0.0f && rt.shellyT >= cfg.shellyTAlarmC)
        return String("OVERHEATING: Shelly at ") + String(rt.shellyT, 1) + " degC (alarm threshold " + String(cfg.shellyTAlarmC, 0) + " degC)";
      return String("OVERHEATING: Shelly unreachable for ") + mn + " min while thermal protection was active";
    case WD_RELAY:   return String("relay open but the tank draws ") + (isnan(rt.shellyW) ? String("?") : String(rt.shellyW, 0)) + " W for " + mn + " min";
    case WD_MEMORY:  return String("memory almost full: ") + String((unsigned long)ESP.getFreeHeap()) + " bytes free (threshold " + String((unsigned long)WD_LOW_HEAP_BYTES) + ")";
    case WD_SHELLY:  return String("Shelly unreachable for ") + mn + " min (limit " + String((unsigned)cfg.wdProblemMin) + " min)";
    case WD_COMMAND: return String("Shelly commands not confirmed for ") + mn + " min (limit " + String((unsigned)cfg.wdProblemMin) + " min)";
    case WD_CLOCK:   return String("invalid clock (NTP) for ") + mn + " min (limit " + String((unsigned)cfg.wdProblemMin) + " min)";
    default:         return "unknown cause";
  }
}

// Called when the alarm state changes: closes the entry in progress (if any) then, if an alarm is active, opens one.
static void sosRecord(bool alarm, WdCause cause, uint32_t elapsedMs, bool tKnown, uint32_t now) {
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  const time_t ep = time(nullptr);
  const uint32_t epoch = (ep < 946684800) ? 0 : (uint32_t)ep;

  // 1. close the previous alarm
  for (int i = (int)arr.size() - 1; i >= 0; i--) {
    JsonObject o = arr[i];
    if (o["dur"].isNull() && !(o["reboot"] | false)) {
      o["dur"] = sosOpenMs ? (uint32_t)((now - sosOpenMs) / 1000UL) : 0U;
      break;
    }
  }
  sosOpenMs = 0;

  // 2. open the new one (or reopen the previous one if the same cause comes back right away)
  if (alarm) {
    sosOpenMs = now ? now : 1;
    bool reopened = false;
    if (epoch != 0 && arr.size() > 0) {
      JsonObject last = arr[arr.size() - 1];
      const uint32_t t = last["t"] | 0;
      if (t != 0 && !last["dur"].isNull() && String((const char *)(last["cause"] | "")) == wdCauseName(cause) &&
          epoch <= t + (uint32_t)(last["dur"] | 0) + SOS_REOPEN_S) {
        last.remove("dur");
        last["n"] = (uint32_t)(last["n"] | 1) + 1;
        last["msg"] = sosDetail(cause, elapsedMs, tKnown);
        reopened = true;
      }
    }
    if (!reopened) {
      JsonObject o = arr.add<JsonObject>();
      o["t"]     = epoch;                                  // 0 = clock invalid at that moment
      o["up"]    = (uint32_t)(millis() / 1000UL);          // ESP uptime (s)
      o["cause"] = wdCauseName(cause);
      o["msg"]   = sosDetail(cause, elapsedMs, tKnown);
      o["crit"]  = (cause == WD_OVERHEAT);
      o["mode"]  = cfg.mode;
      o["rssi"]  = WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0;
      o["wifi"]  = WiFi.status() == WL_CONNECTED;
      o["heap"]  = (uint32_t)ESP.getFreeHeap();
      o["heapMin"] = (uint32_t)ESP.getMinFreeHeap();
      o["phase"] = (const char *)diagPhase;
      o["sOk"]   = rt.shellyConnected;                     // Shelly reachable
      o["sOn"]   = rt.shellyOn;                            // relay closed
      if (!isnan(rt.shellyW)) o["sW"] = (int)lroundf(rt.shellyW);
      if (!isnan(rt.shellyT)) o["sT"] = (int)lroundf(rt.shellyT * 10.0f) / 10.0f;
      o["hold"]  = rt.thermHold;                           // thermal protection active
      o["eco"]   = rt.ecoConnected;
      o["ecoFail"] = (uint32_t)rt.ecoFailCount;
      o["shFail"]  = (uint32_t)rt.shellyFailCount;
      o["crash"] = (uint32_t)rtcCrashCount;
      o["mute"]  = (uint32_t)wdMuteLeftMin(now);           // minutes of silence remaining (quiet window / remote silence): > 0 = the buzzer was not supposed to sound
      if (rt.error.length()) o["err"] = rt.error.substring(0, 80);
    }
  }
  sosPrune(arr);
  writeJsonAtomic(SOS_PATH, doc);
  Serial.printf("[SOS] %s cause=%s time=%s\n", alarm ? "alarm recorded" : "alarm end recorded", wdCauseName(cause), nowText().c_str());
}

// Buzzer test requested from the interface: a "test" line so nobody wonders why it sounded.
static void sosRecordTest() {
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  const time_t ep = time(nullptr);
  JsonObject o = arr.add<JsonObject>();
  o["t"]     = (ep < 946684800) ? 0U : (uint32_t)ep;
  o["up"]    = (uint32_t)(millis() / 1000UL);
  o["cause"] = "test";
  o["msg"]   = "buzzer test requested from the interface";
  o["dur"]   = (uint32_t)(WD_TEST_MS / 1000UL);
  sosPrune(arr);
  writeJsonAtomic(SOS_PATH, doc);
}

// At boot: an alarm still "in progress" in the file was interrupted by a restart (end unknown).
static void sosBoot() {
  if (!LittleFS.exists(SOS_PATH)) return;
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  bool changed = false;
  for (JsonObject o : arr) {
    if (o["dur"].isNull() && !(o["reboot"] | false)) { o["reboot"] = true; changed = true; }
  }
  if (sosPrune(arr)) changed = true;
  if (changed) writeJsonAtomic(SOS_PATH, doc);
}

// Hourly cleanup: an alarm ended more than 72 h ago disappears even if nobody opens the page.
static void sosService(uint32_t now) {
  if ((uint32_t)(now - sosLastPruneMs) < SOS_PRUNE_EVERY_MS) return;
  sosLastPruneMs = now;
  if (!LittleFS.exists(SOS_PATH) || time(nullptr) < 946684800) return;
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  if (sosPrune(arr)) {
    if (arr.size() == 0) LittleFS.remove(SOS_PATH); else writeJsonAtomic(SOS_PATH, doc);
  }
}

// GET /api/sos: alarm log (oldest first), already purged of alarms older than 72 h.
void apiSos() {
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  if (sosPrune(arr)) {
    if (arr.size() == 0) LittleFS.remove(SOS_PATH); else writeJsonAtomic(SOS_PATH, doc);
  }
  doc["keepH"] = SOS_KEEP_S / 3600UL;
  doc["now"]   = (uint32_t)(time(nullptr) < 946684800 ? 0 : time(nullptr));
  doc["active"] = wd.alarm;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Decides whether ERR must be 1. A problem only alerts after lasting long enough:
//   - restart loop                            : immediately
//   - Shelly above the alarm threshold        : immediately (shellyTAlarmC); Shelly unreachable during thermal protection: 2 min
//   - relay open but power measured           : 2 min (the tank is heating uncontrolled)
//   - memory almost empty                     : 2 min (crash imminent)
//   - Shelly unreachable                      : wdProblemMin (the tank can no longer be controlled)
//   - Shelly commands not confirmed           : wdProblemMin (the relay no longer follows commands)
//   - invalid clock                           : wdProblemMin (tick() takes no decision)
// Ecojoko alone does not alert: without it, off-peak heating and the quota keep working.
static void wdEvaluate(uint32_t now) {
  const bool ctrl = (cfg.mode != "monitor");
  const uint32_t slowMs = (uint32_t)cfg.wdProblemMin * 60000UL;

  const bool relayBad = ctrl && !rt.shellyOn && !isnan(rt.shellyW) && rt.shellyW > max(cfg.thermostatW, 50.0f);
  const bool memBad   = ESP.getFreeHeap() < WD_LOW_HEAP_BYTES;
  const bool shellyBad = ctrl && !rt.shellyConnected;
  const bool cmdBad   = ctrl && wd.cmdFailSince != 0 && (uint32_t)(now - wd.cmdFailLast) <= WD_CMD_FAIL_GAP_MS;
  const bool clockBad = ctrl && time(nullptr) < 946684800;
  // Overheating: (1) Shelly above the alarm threshold, immediately; (2) Shelly became unreachable DURING thermal protection
  // (it was too hot and no longer answers: power cut, Wi-Fi down, or worse): 2 min. Also active in "monitor" mode.
  const bool tKnown   = rt.shellyConnected && !isnan(rt.shellyT);
  const bool overheat = cfg.shellyTAlarmC > 0.0f && tKnown && rt.shellyT >= cfg.shellyTAlarmC;
  const bool lostHot  = rt.thermHold && !rt.shellyConnected;
  if (!cmdBad) wd.cmdFailSince = 0;

  struct Item { WdCause cause; bool on; uint32_t elapsed; uint32_t limit; };
  const Item items[] = {
    { WD_CRASH_LOOP, wdCrashLoopActive(), 0, 0 },
    { WD_OVERHEAT,   overheat, wdTrack(overheat, wd.overheatSince, now), 0 },
    { WD_OVERHEAT,   lostHot,  wdTrack(lostHot,  wd.lostHotSince,  now), WD_FAST_DELAY_MS },
    { WD_RELAY,      relayBad,  wdTrack(relayBad,  wd.relaySince,  now), WD_FAST_DELAY_MS },
    { WD_MEMORY,     memBad,    wdTrack(memBad,    wd.memSince,    now), WD_FAST_DELAY_MS },
    { WD_SHELLY,     shellyBad, wdTrack(shellyBad, wd.shellySince, now), slowMs },
    { WD_COMMAND,    cmdBad,    cmdBad ? (uint32_t)(now - wd.cmdFailSince) : 0, slowMs },
    { WD_CLOCK,      clockBad,  wdTrack(clockBad,  wd.clockSince,  now), slowMs },
  };

  bool alarm = false, crit = false;
  WdCause cause = WD_OK, pending = WD_OK;
  uint32_t pendingMs = 0, pendingLimitMs = 0, causeElapsedMs = 0;
  for (const Item &it : items) {
    if (!it.on) continue;
    if (it.elapsed >= it.limit) {
      if (!alarm) { alarm = true; cause = it.cause; causeElapsedMs = it.elapsed; }
      if (it.cause == WD_OVERHEAT) crit = true;   // even if another cause (e.g. crashes) came first
    } else if (pending == WD_OK) {
      pending = it.cause; pendingMs = it.elapsed; pendingLimitMs = it.limit;
    }
  }

  if (alarm != wd.alarm || cause != wd.cause) {
    Serial.printf("[Watchdog] alarm=%s cause=%s time=%s\n", alarm ? "YES" : "NO", wdCauseName(cause), nowText().c_str());
    sosRecord(alarm, cause, causeElapsedMs, tKnown, now);   // SOS log (file): reason + system state
  }
  wd.alarm = alarm; wd.cause = cause; wd.crit = crit;
  wd.pending = pending; wd.pendingMs = pendingMs; wd.pendingLimitMs = pendingLimitMs;
}

// ---- Alarm silence (quiet window + remote silence) ----
// Frame sent to the C3: $M<minutes>*<XX>\n, XX = XOR of the characters between '$' and '*' (2 hex digits).
static void wdSendMute(uint16_t minutes) {
  if (wd.txPin < 0) return;
  if (minutes > WD_MUTE_MAX_MIN) minutes = WD_MUTE_MAX_MIN;
  char body[8];
  const int n = snprintf(body, sizeof(body), "M%u", (unsigned)minutes);
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)body[i];
  char out[16];
  snprintf(out, sizeof(out), "$%s*%02X\n", body, (unsigned)cs);
  Serial2.print(out);
}

// Inside the quiet window? (invalid clock: never)
static bool wdInQuietWindow() {
  if (!cfg.muteOn || cfg.muteStart == cfg.muteEnd) return false;
  tm lt;
  if (!localNow(lt)) return false;
  const uint16_t mm = (uint16_t)(lt.tm_hour * 60 + lt.tm_min);
  return cfg.muteStart < cfg.muteEnd
         ? (mm >= cfg.muteStart && mm < cfg.muteEnd)
         : (mm >= cfg.muteStart || mm < cfg.muteEnd);
}

// Minutes remaining in the quiet window (0 = outside the window).
static uint16_t wdQuietLeftMin() {
  if (!wdInQuietWindow()) return 0;
  const int left = ((int)cfg.muteEnd - (int)nowMin() + 1440) % 1440;
  return (uint16_t)(left > 0 ? left : 1);
}

// Minutes remaining of the remote silence (0 = none). Rounded up to the next minute.
static uint16_t wdSnoozeLeftMin(uint32_t now) {
  if (wd.snoozeUntilMs == 0) return 0;
  const int32_t left = (int32_t)(wd.snoozeUntilMs - now);
  if (left <= 0) { wd.snoozeUntilMs = 0; return 0; }
  const uint32_t mn = ((uint32_t)left + 59999UL) / 60000UL;
  return (uint16_t)(mn > WD_MUTE_MAX_MIN ? WD_MUTE_MAX_MIN : mn);
}

static uint16_t wdMuteLeftMin(uint32_t now) {
  const uint16_t a = wdSnoozeLeftMin(now), b = wdQuietLeftMin();
  return a > b ? a : b;
}

// An overheating alarm is NEVER silenced (fire risk in the enclosure).
static bool wdCritActive() { return wd.alarm && wd.crit; }

// Called on every pass through wdService().
static void wdMuteService(uint32_t now, bool testOn) {
  if (wd.txPin < 0) return;
  const bool force = wdCritActive() || testOn;   // silence forbidden: the C3 must be able to sound
  if (wdCritActive()) wd.snoozeUntilMs = 0;
  const uint16_t rem = force ? 0 : wdMuteLeftMin(now);
  uint32_t period;
  if (force) period = WD_CANCEL_PERIOD_MS;
  else if (wd.sendBursts > 0) period = WD_MUTE_BURST_MS;
  else if (rem > 0) period = WD_MUTE_PERIOD_MS;
  else return;                                   // nothing to say: the C3 keeps its own countdown
  if ((uint32_t)(now - wd.lastMuteTxMs) < period) return;
  wd.lastMuteTxMs = now;
  wdSendMute(rem);
  if (!force && wd.sendBursts > 0) wd.sendBursts--;
}

// Window or duration changed: immediately send the real state (including 0 if the silence no longer applies).
void wdMuteChanged() { wd.sendBursts = 3; }

// "on": silence the buzzer for cfg.snoozeMin minutes (during a non-critical alarm); "off": re-enable it.
int wdSnoozeCmd(bool on, String &msg) {
  const uint32_t now = millis();
  if (wd.txPin < 0) { msg = "Serial link to the C3 disabled (TX pin): remote silence impossible"; return 400; }
  if (!on) {
    wd.snoozeUntilMs = 0;
    wd.sendBursts = 3;
    Serial.printf("[Watchdog] remote silence cancelled time=%s\n", nowText().c_str());
    return 200;
  }
  if (wdCritActive()) { msg = "Overheating alarm: it cannot be silenced remotely"; return 409; }
  if (!wd.alarm) { msg = "No alarm in progress"; return 409; }
  wd.snoozeUntilMs = now + (uint32_t)cfg.snoozeMin * 60000UL;
  if (wd.snoozeUntilMs == 0) wd.snoozeUntilMs = 1;
  wd.sendBursts = 3;
  Serial.printf("[Watchdog] remote silence %u min cause=%s time=%s\n", (unsigned)cfg.snoozeMin, wdCauseName(wd.cause), nowText().c_str());
  return 200;
}

// To be called on every pass through loop().
void wdService(uint32_t now) {
  if (wd.hbPin >= 0 && (uint32_t)(now - wd.lastHbMs) >= WD_HB_PERIOD_MS) {
    wd.lastHbMs = now;
    if ((uint32_t)(now - rt.lastTick) < WD_TICK_MAX_AGE_MS) { // otherwise we stay silent: the C3 must notice
      wd.hbLevel = !wd.hbLevel;
      digitalWrite(wd.hbPin, wd.hbLevel ? HIGH : LOW);
    }
  }
  if ((uint32_t)(now - wd.lastEvalMs) >= 1000) {
    wd.lastEvalMs = now;
    wdEvaluate(now);
  }
  bool testOn = wd.testUntilMs != 0 && (int32_t)(wd.testUntilMs - now) > 0;
  if (wd.testUntilMs != 0 && !testOn) wd.testUntilMs = 0;
  wdMuteService(now, testOn);
  sosService(now);
  bool wantErr = wd.alarm || testOn;
  if (wd.errPin >= 0 && wantErr != wd.errLevel) {
    wd.errLevel = wantErr;
    digitalWrite(wd.errPin, wantErr ? HIGH : LOW);
  }
}

bool wdStartTest() {
  if (wd.errPin < 0) return false;
  wd.testUntilMs = millis() + WD_TEST_MS;
  if (wd.testUntilMs == 0) wd.testUntilMs = 1;
  sosRecordTest();
  return true;
}

void wdFillStatus(JsonDocument &d) {
  d["wdCause"]           = wdCauseName(wd.alarm ? wd.cause : WD_OK);
  d["wdPending"]         = wdCauseName(wd.pending);
  d["wdPendingMin"]      = wd.pendingMs / 60000UL;
  d["wdPendingLimitMin"] = wd.pendingLimitMs / 60000UL;
  d["wdTest"]            = wd.testUntilMs != 0;
  d["wdHbPin"]           = wd.hbPin;
  d["wdErrPin"]          = wd.errPin;
  const uint32_t now = millis();
  const uint16_t sn = wdSnoozeLeftMin(now), qu = wdQuietLeftMin(), mu = sn > qu ? sn : qu;
  d["wdTxPin"]           = wd.txPin;
  d["wdAlarm"]           = wd.alarm;
  d["wdCrit"]            = wdCritActive();
  d["wdSnoozeMin"]       = sn;   // remainder of the remote silence (min)
  d["wdQuietMin"]        = qu;   // remainder of the quiet window (min)
  d["wdMuteMin"]         = mu;   // effective silence = the larger of the two
  if (mu > 0 && time(nullptr) >= 946684800) d["wdMuteUntil"] = (uint32_t)time(nullptr) + (uint32_t)mu * 60U;
}

// Home page: replaces serveStatic("/", ..., "/index.html") so the request can be traced.
// Serves /index.html.gz (if present) first: smaller, hence less transfer time on a weak Wi-Fi.
// ETag + "no-cache": the browser revalidates on every load and gets a 304 WITHOUT a body as long as the file has not
// changed (a reload then costs only a few bytes of response instead of ~100 KB that keep loop() busy).
static char     indexEtag[24] = "";
static uint32_t indexEtagSize = 0;
static bool     indexEtagGz   = false;

static void serveIndex() {
  bool gz = LittleFS.exists("/index.html.gz");
  if (!gz && !LittleFS.exists("/index.html")) { server.send(404, "text/plain", "index.html missing"); return; }
  File f = LittleFS.open(gz ? "/index.html.gz" : "/index.html", "r");
  if (!f) { server.send(500, "text/plain", "unable to read index"); return; }

  // Fingerprint computed only once (re-read only if the file changes size or .gz variant).
  uint32_t sz = (uint32_t)f.size();
  if (indexEtag[0] == '\0' || indexEtagSize != sz || indexEtagGz != gz) {
    uint32_t h = 2166136261UL; // FNV-1a
    uint8_t buf[256];
    size_t n;
    while ((n = f.read(buf, sizeof(buf))) > 0) for (size_t i = 0; i < n; i++) { h ^= buf[i]; h *= 16777619UL; }
    f.seek(0);
    snprintf(indexEtag, sizeof(indexEtag), "\"%08lx-%lx\"", (unsigned long)h, (unsigned long)sz);
    indexEtagSize = sz;
    indexEtagGz   = gz;
  }
  server.sendHeader("ETag", indexEtag);
  server.sendHeader("Cache-Control", "no-cache");
  if (server.hasHeader("If-None-Match") && server.header("If-None-Match") == indexEtag) {
    f.close();
    server.send(304, "text/html", "");
    return;
  }
  // Manual send (not streamFile): streamFile() ALREADY adds "Content-Encoding: gzip" by itself when the file name ends in .gz
  // (recent ESP32 core); adding it here too produced a duplicate header, which the browser decodes twice (black page / endless loading).
  // Here the header is written once, whatever the core version.
  server.setContentLength(sz);
  if (gz) server.sendHeader("Content-Encoding", "gzip");
  server.send(200, "text/html; charset=utf-8", "");
  server.client().write(f);
  f.close();
}

void setup() { appSetup(); }
void loop()  { appLoop(); }

void appSetup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- [DEBUG] ESP32 COLD BOOT STARTUP ---\n");

  // Created before any web route and before controlTask(): neither may be able to start without it.
  ioMutex = xSemaphoreCreateMutex();
  if (!ioMutex) { // extreme case (not enough memory from the start): better to restart right away
    Serial.println("[Fatal] ioMutex creation failed: restarting");
    delay(1000);
    ESP.restart();
  }

  LittleFS.begin(true);
  readConfig();
  if (cfg.lastFwBuild != FW_BUILD) { // new firmware flashed (FW_BUILD changes on every compilation): back to the main mode
    Serial.printf("[Boot] new firmware (previous mode=%s): full mode\n", cfg.mode.c_str());
    cfg.mode = "full";
    cfg.lastFwBuild = FW_BUILD;
    saveConfig();
  }
  antLoad(); // forecast history (daily need, solar, temperature)
  antHrLoad(); // forecast / actual per hour
  antShadeLoad(); // shading statistic
  heatLoad(); // heating log
  wdInit(); // HB/ERR pins driven as early as possible (low level)
  sosBoot(); // alarm left "in progress" in the SOS log = interrupted by this restart
  restoreEcoGatewayCache();
  connectNetwork();

  applyClockConfig();
  diagBootReport();
  rt.cycleId = cycleForNow();
  if (rt.cycleId != 0) restoreCurrentHistory();
  restoreCurrentMonth();

  webRoute("/",             HTTP_GET,  serveIndex);
  webRoute("/api/status",   HTTP_GET,  []() { server.send(200, "application/json", statusJson()); });
  webRoute("/api/config",   HTTP_GET,  apiConfig);
  webRoute("/api/config",   HTTP_POST, apiConfig);
  webRoute("/api/manual",   HTTP_POST, apiManual);
  webRoute("/api/watchdog", HTTP_POST, apiWatchdog);
  webRoute("/api/snooze",   HTTP_POST, apiSnooze);
  webRoute("/api/reboot",   HTTP_POST, apiReboot);
  webRoute("/api/eco/retry", HTTP_POST, apiEcoRetry);
  webRoute("/api/history",  HTTP_GET,  apiHistory);
  webRoute("/api/monthly",  HTTP_GET,  apiMonthly);
  webRoute("/api/yearly",   HTTP_GET,  apiYearly);
  webRoute("/api/storage",  HTTP_GET,  apiStorage);
  webRoute("/api/errors",   HTTP_GET,  apiErrors);
  webRoute("/api/sos",      HTTP_GET,  apiSos);
  webRoute("/api/diag",     HTTP_GET,  apiDiag);
  webRoute("/api/ant",      HTTP_GET,  apiAnt);
  webRoute("/api/ant/raw",  HTTP_GET,  apiAntRaw);
  webRoute("/api/heat",     HTTP_GET,  apiHeat);
  webRoute("/api/stemp",    HTTP_GET,  apiStemp);
  webRoute("/api/checkpoint", HTTP_POST, apiCheckpoint);
  webRoute("/api/monthly/raw", HTTP_GET, apiMonthlyRaw);
  webRoute("/api/import/history", HTTP_POST, apiImportHistory);
  webRoute("/api/import/monthly", HTTP_POST, apiImportMonthly);
  webRoute("/api/import/ant", HTTP_POST, apiImportAnt);
  webRoute("/api/import/heat", HTTP_POST, apiImportHeat);
  webRoute("/ant",          HTTP_GET,  []() { server.send_P(200, "text/html; charset=utf-8", ANT_PAGE); });
  server.onNotFound([]() {  // e.g. /favicon.ico: answers 404 as before, but the URI is traced
    diagWebTag(server.uri().c_str());
    server.send(404, "text/plain", "Not found");
  });
  static const char *HDRS_COLLECT[] = {"If-None-Match"}; // required for the 304 revalidation of the home page
  server.collectHeaders(HDRS_COLLECT, 1);
  server.begin();
  
  rt.lastTick = millis();
  diagLoopMs = millis();
  diagPhase = "loop";
  xTaskCreatePinnedToCore(stallMonitorTask, "stallmon", 3072, nullptr, 1, nullptr, 0);
  // Core 1 (like loopTask/web), NOT core 0: the blocking Ecojoko/Shelly calls of controlTask()
  // thus run away from the internal Wi-Fi/lwIP tasks (core 0 by default), as before the web/control
  // separation. stallMonitorTask remains alone on core 0 (intentional cross-monitoring).
  // See the 15/11 fix in the comment near ioMutex for the details of the problem solved.
  xTaskCreatePinnedToCore(controlTask, "control", 10240, nullptr, 1, nullptr, 1);
}

// POST /api/eco/retry ("Retry now"): NO network call here (synchronous web server). We reset the backoff,
// the session (cookie) and Ecojoko's TLS socket; the next pass of sample() (5 s max) does login then read again.
// The known gatewayId / powerMeterId are kept (no extra /gateways call).
void apiEcoRetry() {
  rt.ecoFailCount = 0;
  rt.lastEcoPollMs = 0;
  rt.ecoCookie = "";
  ecoTlsReset();
  Serial.println("[Ecojoko] manual retry requested from the interface");
  server.send(200, "text/plain", "OK");
}

// Ecojoko unreachable for ECO_BLOCKED_RESTART_MS with Wi-Fi OK: we save what only lives in RAM (like the
// "Save now" button), note the cause in /boot_log.txt then restart. At worst one restart every 30 min
// if Ecojoko stays down (the count restarts from zero after each boot).
static void ecoBlockedService(uint32_t now) {
  uint32_t blocked = ecoBlockedAgeMs(now);
  if (blocked < ECO_BLOCKED_RESTART_MS) return;
  String line = String("automatic restart: Ecojoko unreachable for ") + String(blocked / 60000UL) + " min (Wi-Fi OK, consecutive failures=" +
                String(rt.ecoFailCount) + ") last_message=" + rt.error + " time=" + diagFormatEpoch((uint32_t)time(nullptr));
  Serial.println("[Ecojoko] " + line);
  diagAppendLog(line);
  heatSaveImpl(true);
  if (rt.cycleId != 0) saveHistory();
  antCycSave();
  delay(300);
  ESP.restart();
}

// A single pass of the control loop (C3 watchdog, Wi-Fi, Ecojoko/Shelly, relay decision, weather).
// Called in a loop by controlTask(), protected by ioMutex. Body unchanged compared to the former appLoop()
// (excluding the web server): only the place it is called from has changed.
static void controlPass() {
  uint32_t now = millis();
  wdService(now);
  refreshDailyErrorCount();
  clearResolvedError();
  serviceAp(now);
  monitorWifiConnection(now);
  ecoBlockedService(now);
  shellyCmdService(millis()); // confirmation / new relay attempt
  if (now - rt.lastTick >= CONTROL_PERIOD_MS) {
    uint32_t rawDt = (now - rt.lastTick) / 1000;
    uint32_t dt = min(rawDt, MAX_TICK_DT_SEC);
    if (rawDt != dt) {
      Serial.printf("[Quota] dt limited: raw=%lus, used=%lus\n", (unsigned long)rawDt, (unsigned long)dt);
    }
    rt.lastTick = now;
    sample();
    tick(dt);
    // Weather: only if this pass has not already been long (margin with the stall detector).
    if ((uint32_t)(millis() - now) < 20000UL) antService(millis());
  }
}

// FreeRTOS task dedicated to the control loop: independent of the web server (which stays in loop(), see
// appLoop() below). diagLoopMs (monitored by stallMonitorTask) is updated FIRST OF ALL, even
// before requesting ioMutex, so that this task remains seen as alive during a brief wait for the
// lock (otherwise a legitimately long web request could wrongly trigger a restart).
static void controlTask(void *) {
  for (;;) {
    diagLoopMs = millis();
    if (xSemaphoreTake(ioMutex, IO_MUTEX_CTRL_WAIT_TICKS) == pdTRUE) {
      controlPass();
      xSemaphoreGive(ioMutex);
    } else {
      // Only happens if a web request stayed abnormally long in its handler (JSON/LittleFS,
      // no network): we skip this pass rather than wait indefinitely, and retry quickly.
      Serial.println("[Control] ioMutex not obtained in time (abnormally long web request): pass skipped");
    }
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

// loop() (Arduino, web side): now only serves the HTTP server. All the control logic
// (controlPass()) now runs in its own task (controlTask()) and never again waits on a
// slow web request.
void appLoop() {
  // No DiagPhaseScope here: diagPhase now belongs exclusively to controlTask() (it is the
  // only task monitored by diagLoopMs/stallMonitorTask). Marking it from both tasks would recreate
  // a race (the web task would overwrite controlTask()'s "phase=eco_login" etc. at the worst moment: a
  // stall post-mortem). The web task keeps its own markers (diagWebUri/diagWebIp/diagWebStartMs).
  strcpy(diagWebUri, "(waiting/reading)"); // normal state when no request is being served; stays so if the stall happens before the handler (incomplete request)
  diagWebIp[0] = '\0';
  uint32_t web0 = millis();
  diagWebStartMs = web0;
  server.handleClient();
  diagWebStartMs = 0;
  diagWebAfter(millis() - web0);
}

