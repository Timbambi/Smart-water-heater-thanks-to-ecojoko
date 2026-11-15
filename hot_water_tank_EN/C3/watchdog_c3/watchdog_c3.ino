/* ============================================================
   watchdog_c3 — chien de garde externe du ballon d'eau chaude (ESP32-C3)
   ============================================================
   IMPORTANT : ce fichier doit etre dans SON PROPRE DOSSIER (watchdog_c3/watchdog_c3.ino).
   L'IDE Arduino compile ensemble tous les .ino d'un meme dossier : a cote de
   esp32-ballon-ecojoko.ino il serait melange au programme principal.

   Role : faire sonner un buzzer quand la chauffe est compromise, sans dependre du programme principal.
   Ce programme n'utilise ni Wi-Fi, ni fichiers, ni reseau, ni delay() bloquant : la boucle tourne
   en continu et un watchdog materiel redemarre le C3 si elle se figeait.

   Cablage (3,3 V partout ; le C3 est alimente par l'ESP32 principal) :
     ESP32 principal VIN  ---- 5V du C3
     ESP32 principal GND  ---- GND du C3
     Broche HB  du principal (Reglages) ---- GPIO4 du C3 (PIN_HB_IN)   battement de coeur
     Broche ERR du principal (Reglages) ---- GPIO3 du C3 (PIN_ERR_IN)  alarme
     GPIO6 (PIN_BUZZER) ---- buzzer PASSIF, autre borne du buzzer a GND. Le C3 genere lui-meme les tonalites (LEDC).
                     Buzzer piezo : direct. Buzzer magnetique (bobine, > ~10 mA) : transistor NPN + resistance + diode.
     GPIO10 (PIN_ACK)   ---- bouton on/off, autre borne du bouton a GND
     Broche TX (Reglages) du principal ---- GPIO1 du C3 (PIN_MUTE_RX)   liaison serie a sens unique : silence de l'alarme

   Le C3 sonne dans deux cas :
     1. Plus de battement de coeur depuis HB_TIMEOUT_MS : le programme principal est gele, plante en boucle ou absent.
        Son : grave 800 Hz / aigu 4100 Hz en alternance (x2), pause.
     2. ERR reste a 1 plus de ERR_CONFIRM_MS : le programme principal a detecte et confirme un probleme
        qui compromet la chauffe (Shelly injoignable, relais incoherent, etc.). Son : sirene 4100 Hz / 3200 Hz, pause.
        Le delai avant de lever ERR est regle dans la page web (Reglages > Chien de garde).

   Bouton ACK (interrupteur on/off ou poussoir) :
     - TOUT changement de position (on->off ou off->on), avec ou sans alarme, coupe la sonnerie pendant
       ACK_SNOOZE_MS (24 h) et fait un bip court (3200 Hz) de confirmation.
     - Ce silence est inconditionnel : il continue meme si le probleme disparait, ou si un AUTRE probleme apparait.
     - Chaque nouveau changement de position relance les 24 h a zero.
     - Seules fins possibles : les 24 h ecoulees, ou un redemarrage du C3 (coupure de courant, watchdog interne).

   Silence demande par le principal (plage silencieuse ou bouton « Desactiver pendant X h » de la page web) :
     le principal envoie sur GPIO1 des trames « $M<minutes>*<XX> » (9600 bauds, XX = XOR des caracteres entre $ et *, en hexa).
     Le C3 compte lui-meme le silence avec millis() : il n'a besoin ni de l'heure ni du principal pour savoir quand il se termine.
     Meme si le principal se fige pendant le silence, le C3 reste muet jusqu'a l'echeance puis sonne si le probleme dure.
     Plafond MUTE_MAX_MIN (une trame plus grande est ramenee a ce plafond). Une trame invalide est ignoree.
     « $M0*.. » annule le silence : le principal l'envoie pendant une surchauffe, un test du buzzer ou un « Reactiver ».
     Si le fil est coupe ou absent, le C3 ne recoit rien : l'alarme sonne normalement (defaut sans danger).
     Le bouton ACK du C3 reste independant et prioritaire (il fonctionne meme sans liaison).

   Pas de coupure de courant geree : le C3 est alimente par l'ESP32 principal, donc plus de courant = plus de ballon. */

#include <Arduino.h>
#include <esp_task_wdt.h>

/* ---------- Broches (ESP32-C3 SuperMini : eviter GPIO 2, 8, 9 = demarrage ; 18, 19 = USB ; 20, 21 = serie) ---------- */
constexpr uint8_t PIN_HB_IN  = 4;    // C3 GPIO4  <- HB  du principal
constexpr uint8_t PIN_ERR_IN = 3;    // C3 GPIO3  <- ERR du principal
constexpr uint8_t PIN_BUZZER = 6;    // C3 GPIO6  -> buzzer passif (autre borne a GND)
constexpr uint8_t PIN_ACK    = 10;   // C3 GPIO10 <- bouton on/off (autre borne a GND)
constexpr uint8_t PIN_MUTE_RX = 1;   // C3 GPIO1  <- TX du principal (trames de silence, sens unique)

/* ---------- Tonalites du buzzer (Hz), choisies pour les frequences ou ton buzzer sonne le mieux ---------- */
constexpr uint32_t FREQ_HIGH = 4100;
constexpr uint32_t FREQ_MID  = 3200;
constexpr uint32_t FREQ_LOW  = 800;

/* ---------- Delais ---------- */
constexpr uint32_t HB_TIMEOUT_MS      = 10UL * 60UL * 1000UL; // plus de battement depuis 10 min -> alarme
constexpr uint32_t HB_MIN_STABLE_MS   = 300;                  // un front n'est valide que si le niveau precedent a dure >= 300 ms
constexpr uint32_t ERR_CONFIRM_MS     = 3000;                 // ERR doit rester a 1 pendant 3 s (filtre les parasites)
constexpr uint32_t ACK_SNOOZE_MS = 24UL * 60UL * 60UL * 1000UL; // 24 heures
constexpr uint32_t ACK_DEBOUNCE_MS    = 60;
constexpr uint32_t BEEP_SHORT_MS      = 250;
constexpr uint32_t WDT_TIMEOUT_S      = 8;                    // watchdog materiel du C3 lui-meme
constexpr uint32_t MUTE_BAUD          = 9600;                 // liaison serie de silence (doit etre identique cote principal)
constexpr uint32_t MUTE_MAX_MIN       = 720;                  // plafond d'un silence demande par le principal : 12 h
constexpr uint8_t  MUTE_RX_MAX_BYTES  = 32;                   // octets lus au maximum a chaque tour de loop()

#define C3_DEBUG 0  // 1 = messages sur le port serie (utile uniquement branche a l'ordinateur)
#if C3_DEBUG
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...) do {} while (0)
#endif

enum : uint8_t { CAUSE_HB = 1, CAUSE_ERR = 2 };

/* ---------- Etat ---------- */
static bool     hbLast        = false;
static uint32_t hbChangeMs    = 0;   // dernier changement de niveau brut de HB
static uint32_t hbValidMs     = 0;   // dernier front valide
static uint32_t errHighSince  = 0;   // 0 = ERR bas
static bool     snoozed       = false;   // sonnerie coupee 24 h par le bouton (toutes causes confondues)
static uint32_t snoozeStartMs = 0;
static bool     ackRaw        = false, ackStable = false;
static uint32_t ackChangeMs   = 0;
static uint32_t beepUntilMs   = 0;
static uint8_t  lastMask      = 0;
static uint32_t curFreq       = 0;   // tonalite actuellement emise (0 = silence)
static uint32_t muteStartMs   = 0;   // silence demande par le principal : debut ...
static uint32_t muteDurMs     = 0;   // ... et duree (0 = aucun silence)
static char     rxBuf[16];
static uint8_t  rxLen         = 0;
static bool     rxInFrame     = false;

// Bip court en cours ? (beepUntilMs == 0 : aucun ; comparaison tolerante au debordement de millis())
static bool shortBeepOn(uint32_t now) {
  if (beepUntilMs == 0) return false;
  if ((int32_t)(beepUntilMs - now) > 0) return true;
  beepUntilMs = 0;
  return false;
}
static void startShortBeep(uint32_t now) {
  beepUntilMs = now + BEEP_SHORT_MS;
  if (beepUntilMs == 0) beepUntilMs = 1;
}

// Tonalite voulue selon les causes (t = ms depuis le debut de l'alarme) ; 0 = silence.
static uint32_t patternFreq(uint32_t t, uint8_t mask) {
  if (mask & CAUSE_HB) {            // grave / aigu / grave / aigu (250 ms chacun), puis pause : programme principal muet
    uint32_t p = t % 2400;
    if (p >= 1000) return 0;
    return ((p / 250) % 2 == 0) ? FREQ_LOW : FREQ_HIGH;
  }
  uint32_t p = t % 1200;            // sirene aigu / moyen (300 ms chacun), puis pause : probleme signale par le principal
  if (p >= 600) return 0;
  return ((p / 300) % 2 == 0) ? FREQ_HIGH : FREQ_MID;
}

// Sortie LEDC (compatible cartes Arduino-ESP32 2.x et 3.x). Ne reconfigure que si la tonalite change.
#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t BUZZER_LEDC_CH = 0;
#endif
static void buzzerBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, FREQ_MID, 8);
  ledcWriteTone(PIN_BUZZER, 0);
#else
  ledcSetup(BUZZER_LEDC_CH, FREQ_MID, 8);
  ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CH);
  ledcWriteTone(BUZZER_LEDC_CH, 0);
#endif
}
static void buzzerTone(uint32_t freq) {
  if (freq == curFreq) return;
  curFreq = freq;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIN_BUZZER, freq);   // 0 = coupe (rapport cyclique 0 : aucun courant continu dans le buzzer)
#else
  ledcWriteTone(BUZZER_LEDC_CH, freq);
#endif
}

/* ---------- Silence demande par le principal (liaison serie, sens unique) ---------- */
static void applyMute(uint32_t minutes, uint32_t now) {
  if (minutes > MUTE_MAX_MIN) minutes = MUTE_MAX_MIN;   // plafond
  if (minutes == 0) { muteDurMs = 0; DBG("[C3] silence annule\n"); return; }
  muteStartMs = now;
  muteDurMs = minutes * 60000UL;
  DBG("[C3] silence %lu min\n", (unsigned long)minutes);
}

// Silence en cours ? Efface l'etat a l'echeance (evite un faux silence apres le debordement de millis(), ~49 jours).
static bool muteActive(uint32_t now) {
  if (muteDurMs == 0) return false;
  if ((uint32_t)(now - muteStartMs) < muteDurMs) return true;
  muteDurMs = 0;
  return false;
}

// Trame (sans le '$' ni le '\n') : M<minutes>*<XX>, XX = XOR des caracteres avant '*', 2 chiffres hexa. Toute erreur : ignoree.
static void parseMuteFrame(const char *s, uint32_t now) {
  const char *star = strchr(s, '*');
  if (!star || s[0] != 'M' || star == s + 1 || (star - s) > 5) return;   // 'M' + 1 a 4 chiffres
  if (strlen(star + 1) != 2) return;
  uint8_t cs = 0;
  for (const char *p = s; p < star; p++) cs ^= (uint8_t)*p;
  const char hex[3] = { star[1], star[2], 0 };
  char *end = nullptr;
  const long v = strtol(hex, &end, 16);
  if (*end != 0 || v != (long)cs) return;
  uint32_t minutes = 0;
  for (const char *p = s + 1; p < star; p++) {
    if (*p < '0' || *p > '9') return;
    minutes = minutes * 10 + (uint32_t)(*p - '0');
  }
  applyMute(minutes, now);
}

static void muteRxByte(char c, uint32_t now) {
  if (c == '$') { rxLen = 0; rxInFrame = true; return; }           // debut de trame : on repart de zero
  if (!rxInFrame) return;
  if (c == '\n' || c == '\r') {
    rxInFrame = false;
    rxBuf[rxLen] = 0;
    parseMuteFrame(rxBuf, now);
    rxLen = 0;
    return;
  }
  if (rxLen >= sizeof(rxBuf) - 1) { rxInFrame = false; rxLen = 0; return; }   // trop long : poubelle
  rxBuf[rxLen++] = c;
}

static void serviceMuteRx(uint32_t now) {
  for (uint8_t i = 0; i < MUTE_RX_MAX_BYTES && Serial1.available() > 0; i++) muteRxByte((char)Serial1.read(), now);
}

static void setupWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const esp_task_wdt_config_t cfg = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL); // surveille la tache loop() : si elle se fige, le C3 redemarre
}

void setup() {
  buzzerBegin();
  pinMode(PIN_HB_IN, INPUT_PULLDOWN);   // ligne flottante (principal en reset) = niveau bas, jamais "alarme"
  pinMode(PIN_ERR_IN, INPUT_PULLDOWN);
  pinMode(PIN_ACK, INPUT_PULLUP);
  pinMode(PIN_MUTE_RX, INPUT_PULLUP);   // ligne au repos = niveau haut (repos d'une liaison serie)
  Serial1.begin(MUTE_BAUD, SERIAL_8N1, PIN_MUTE_RX, -1);   // reception seule
#if C3_DEBUG
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // ne jamais bloquer si aucun ordinateur n'est branche
#endif
  uint32_t now = millis();
  hbLast = digitalRead(PIN_HB_IN);
  hbChangeMs = hbValidMs = now;         // delai de grace complet au demarrage (le principal met ~30 s a demarrer)
  ackRaw = ackStable = digitalRead(PIN_ACK);
  ackChangeMs = now;
  startShortBeep(now);                  // bip court d'auto-test
  setupWatchdog();
  DBG("[C3] demarre\n");
}

void loop() {
  esp_task_wdt_reset();
  const uint32_t now = millis();

  /* --- HB : on ne compte que les fronts precedes d'un niveau stable (rejette parasites et sifflements) --- */
  bool hb = digitalRead(PIN_HB_IN);
  if (hb != hbLast) {
    if ((uint32_t)(now - hbChangeMs) >= HB_MIN_STABLE_MS) hbValidMs = now;
    hbLast = hb;
    hbChangeMs = now;
  }
  const bool hbLost = (uint32_t)(now - hbValidMs) >= HB_TIMEOUT_MS;

  /* --- ERR : doit rester a 1 ERR_CONFIRM_MS --- */
  if (digitalRead(PIN_ERR_IN) == HIGH) { if (errHighSince == 0) errHighSince = now ? now : 1; }
  else errHighSince = 0;
  const bool errOn = errHighSince != 0 && (uint32_t)(now - errHighSince) >= ERR_CONFIRM_MS;

  const uint8_t mask = (hbLost ? CAUSE_HB : 0) | (errOn ? CAUSE_ERR : 0);
  if (mask != lastMask) { DBG("[C3] causes=%u\n", mask); }
  static uint32_t alarmStartMs = 0;
  if (mask != 0 && lastMask == 0) alarmStartMs = now;
  lastMask = mask;

  /* --- Bouton ACK (poussoir ou interrupteur : tout changement de position compte) --- */
  bool a = digitalRead(PIN_ACK);
  if (a != ackRaw) { ackRaw = a; ackChangeMs = now; }
  if (ackRaw != ackStable && (uint32_t)(now - ackChangeMs) >= ACK_DEBOUNCE_MS) {
    ackStable = ackRaw;
    snoozed = true;                     // silence 24 h, avec ou sans alarme en cours
    snoozeStartMs = now;
    startShortBeep(now);                // bip = confirmation que le silence est arme
    DBG("[C3] silence 24h\n");
  }

  /* --- Silence bouton : expire uniquement apres ACK_SNOOZE_MS (ni la fin du probleme, ni un nouveau probleme ne l'annulent) --- */
  if (snoozed && (uint32_t)(now - snoozeStartMs) >= ACK_SNOOZE_MS) snoozed = false;

  serviceMuteRx(now);
  const bool silenced = snoozed || muteActive(now);   // ACK local OU silence demande par le principal
  uint32_t freq = (mask != 0 && !silenced) ? patternFreq(now - alarmStartMs, mask) : 0;
  if (freq == 0 && shortBeepOn(now)) freq = FREQ_MID;
  buzzerTone(freq);

  delay(2); // cede la main (FreeRTOS) ; la boucle reste a ~400 Hz, largement assez pour des signaux de 1 Hz
}
