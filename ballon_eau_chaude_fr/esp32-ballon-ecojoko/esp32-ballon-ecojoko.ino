
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
   ESP32 – Ballon d'eau chaude piloté par Ecojoko + Shelly 1PM Gen3
   ============================================================ */

// Version du programme : a incrementer a la main a chaque modification (format AAAA.MM.JJ-n, comparable en texte).
// FW_BUILD change tout seul a chaque compilation : si l'heure affichee dans Systeme n'a pas bouge, le flash n'est pas passe.
static const char FW_VERSION[] = "2026.11.18-1";
static const char FW_BUILD[]   = __DATE__ " " __TIME__;
constexpr uint32_t CONTROL_PERIOD_MS = 5000; // Fréquence de la boucle de contrôle (5s)
constexpr uint32_t MIN_OFF_TO_ON_MS = 60000; // 1 min mini entre un OFF et le ON suivant
// Shelly (local) : timeouts separes. 2,5 s etait trop court : une seule retransmission TCP (RTO ~1,5-3 s)
// apres un paquet perdu suffisait a produire un -11 alors que le Shelly repond en ~70 ms en temps normal.
constexpr uint16_t SHELLY_CONNECT_TIMEOUT_MS = 1500; // etablissement TCP (echec = -1, pas de retry immediat)
constexpr uint16_t SHELLY_READ_TIMEOUT_MS    = 5000; // attente de la reponse (echec = -11)
constexpr uint8_t  SHELLY_READ_MAX_TRIES     = 2;    // lecture de poll : 1 essai + 1 retry, sur -11 (immediat) et -1 (avec delai, voir SHELLY_RETRY_REFUSED_DELAY_MS)
constexpr uint16_t SHELLY_RETRY_REFUSED_DELAY_MS = 250; // -1 = Shelly sature/ferme le port : on lui laisse le temps de liberer son slot avant de retenter
constexpr uint32_t ECO_HTTP_TIMEOUT_MS = 8000; // Cloud Ecojoko (HTTPS distant, plus lent que Shelly en local)
constexpr uint32_t ECO_TLS_HANDSHAKE_TIMEOUT_S = 8; // Defaut du core = 120 s : un handshake TLS bloque loop() tant qu'il n'a pas abouti

/* ---- Certificats racine TLS (verification du serveur distant) ----
   Tant que ces constantes sont vides, le code bascule automatiquement sur setInsecure() (comportement
   actuel, identifiant/mot de passe Ecojoko envoyes sans verifier le serveur) et le journal Serial le
   rappelle. Pour activer la verification, coller ici le certificat RACINE (format PEM, le DERNIER
   certificat affiche par la commande ci-dessous, pas le premier) de chaque service :
     openssl s_client -connect service.ecojoko.com:443   -showcerts < /dev/null
     openssl s_client -connect api.open-meteo.com:443    -showcerts < /dev/null
   Un certificat racine change tres rarement (annees), contrairement au certificat du serveur lui-meme :
   pas de maintenance recurrente a prevoir une fois en place. */
static const char ECOJOKO_ROOT_CA[] = "";   // TODO: coller le certificat racine PEM de service.ecojoko.com
static const char OPENMETEO_ROOT_CA[] = ""; // TODO: coller le certificat racine PEM de api.open-meteo.com
constexpr int      ECO_BEGIN_FAILED = -100; // Code interne : https.begin() a echoue (distinct des codes HTTPClient, de -1 a -11)
constexpr uint32_t SHELLY_CONFIRM_DELAY_MS = 2000;
constexpr uint8_t  SHELLY_MAX_ATTEMPTS = 3;
constexpr uint32_t MAX_TICK_DT_SEC = 30; // Do not charge a long Wi-Fi/HTTP stall as heating time.
constexpr uint32_t WIFI_CHECK_PERIOD_MS = 10000;
constexpr uint32_t WIFI_RECONNECT_AFTER_MS = 20000;
constexpr uint8_t  NETWORK_BACKOFF_AFTER_FAILURES = 3;
constexpr uint32_t NETWORK_BACKOFF_MAX_MS = 60000;
constexpr uint32_t ERROR_CLEAR_DELAY_MS = 30000;
constexpr uint16_t MONTHLY_HISTORY_MAX = 60; // 5 ans de recap mensuel (~230 octets/entree, ~14 Ko au total) ; le recap annuel en est deduit
constexpr uint8_t  YEARLY_HISTORY_MAX = 5;   // nombre d'annees affichees dans le recap annuel (somme des mois de /monthly.json)
// Garde-fou memoire LittleFS : sous ce seuil libre, l'historique est purge (le plus ancien d'abord).
// Jamais touches : /config.json, /index.html, /eco_gateway.json, /boot_log.txt, /sos_log.json.
constexpr size_t   STORAGE_MIN_FREE_BYTES = 32768;
constexpr uint32_t STORAGE_PURGE_RETRY_MS = 600000UL; // si la purge n'a pas suffi, on ne reessaie pas avant 10 min
constexpr uint8_t  PURGE_KEEP_DAYS   = 7;
constexpr uint8_t  PURGE_KEEP_MONTHS_1 = 24;
constexpr uint8_t  PURGE_KEEP_MONTHS_2 = 3;
// Journal SOS : une entree par alarme du buzzer (raison + etat du systeme), purge automatique apres SOS_KEEP_S.
constexpr char     SOS_PATH[]        = "/sos_log.json";
constexpr uint32_t SOS_KEEP_S        = 72UL * 3600UL;   // une alarme terminee depuis plus de 72 h est supprimee
constexpr uint8_t  SOS_MAX_ENTRIES   = 30;              // plafond de securite (~350 octets par entree)
constexpr uint32_t SOS_PRUNE_EVERY_MS = 3600000UL;      // nettoyage horaire (en plus de celui fait a chaque ecriture / lecture)
constexpr uint32_t SOS_REOPEN_S      = 600;             // meme cause revenue < 10 min apres : on rouvre l'entree (compteur n) au lieu d'en creer une
constexpr char FRANCE_TZ[] = "CET-1CEST,M3.5.0,M10.5.0/3";

/* ---------- Prevision (mode full) : parametres internes, voir section 6a ---------- */
constexpr uint8_t  ANT_WX_MAX          = 72;   // heures de meteo gardees en RAM (hier + aujourd'hui + demain)
constexpr uint8_t  ANT_REC_MAX         = 35;   // cycles (jours) conserves dans /ant.json
constexpr uint16_t ANT_HR_N            = 192;  // heures prevu/reel conservees (8 j, anneau) dans /ant_hr.json
constexpr uint32_t ANT_HR_MIN_SEC      = 1800; // une heure n'est retenue que si >= 30 min de mesure
// Ombrage par heure de la journee (section "Ombrage") : appris depuis le prevu / reel horaire, applique seulement si cfg.antShade.
constexpr uint8_t  ANT_SHADE_N         = 20;   // echantillons gardes par heure de la journee (fenetre glissante d'environ 20 jours utilisables)
constexpr uint8_t  ANT_SHADE_MIN_N     = 7;    // minimum d'echantillons pour qu'une heure ait un facteur
constexpr uint8_t  ANT_SHADE_MIN_H     = 4;    // minimum d'heures avec facteur (le niveau de reference est calcule dessus)
constexpr float    ANT_SHADE_MIN_P     = 0.35f; // un echantillon n'est garde que si le prevu est >= 35 % de heaterW (sinon trop bruite)
constexpr float    ANT_SHADE_F_MIN     = 0.30f; // bornes du facteur
constexpr float    ANT_SHADE_F_MAX     = 1.20f;
constexpr uint32_t ANT_WX_REFRESH_MS   = 3UL * 3600UL * 1000UL; // nouvelle prevision toutes les 3 h
constexpr uint32_t ANT_WX_RETRY_MS     = 15UL * 60UL * 1000UL;  // apres un echec
constexpr uint32_t ANT_WX_TIMEOUT_MS   = 8000;
constexpr uint32_t ANT_WX_MAX_AGE_S    = 12UL * 3600UL;         // au-dela la prevision est jugee perimee
constexpr size_t   ANT_WX_MIN_HEAP     = 45000;                 // plus gros bloc libre mini pour un handshake TLS
constexpr float    ANT_TANK_WH_PER_L   = 58.15f; // 1,163 Wh/(L.K) x 50 K d'ecart utile
constexpr float    ANT_SAFETY          = 1.3f;  // marge sur la consommation future (soutirages en rafale)
constexpr float    ANT_DL_MIN_PCT      = 5.0f;  // plancher du deficit tolere a l'echeance (% de la capacite)
constexpr float    ANT_SOLAR_HAIRCUT   = 0.8f;  // on ne compte que 80 % du solaire prevu
constexpr float    ANT_HYST_WH         = 150.0f;
constexpr float    ANT_BENEFIT         = 0.6f;  // un soleil n'est "utile" que si l'energie coute <= 60 % du prix HC
constexpr uint32_t ANT_FORCE_MARGIN_SEC = 1800; // marge du rattrapage avant l'echeance
constexpr uint32_t ANT_GAP_MAX_SEC     = 300;   // trou de mesure max (relais ON sans puissance) avant d'ignorer l'intervalle
constexpr uint32_t ANT_HEAT_MIN_SEC    = 60;    // chauffe continue mini avant qu'une coupure compte comme "plein"
constexpr uint32_t ANT_FULL_IDLE_SEC   = 120;   // puissance < thermostatW pendant 2 min, relais ON => ballon plein
constexpr uint32_t ANT_REF_MAX_H       = 36;    // au-dela, le dernier "plein" est trop ancien
constexpr uint32_t ANT_MIN_COVER_SEC   = 14UL * 3600UL; // un cycle n'est exploitable que si >= 14 h ont ete mesurees
constexpr uint8_t  ANT_MIN_NEED_DAYS   = 4;
constexpr uint8_t  ANT_MIN_REG_DAYS    = 8;     // jours mini pour ajuster la pente temperature
constexpr float    ANT_MIN_REG_SPREAD  = 3.0f;  // ecart de temperature mini (degC) pour que la pente ait un sens
constexpr float    ANT_MAX_SLOPE       = 300.0f; // Wh/degC/jour
constexpr float    ANT_PV_RATIO        = 0.7f;   // coefficient de depart = kWc x 0,7 (W par W/m2) : rendement systeme ~0,8 diminue d'une marge pour la conso de la maison (prudent)
constexpr float    ANT_MIN_K_RAD       = 1500.0f; // Wh/m2 cumules mini pour estimer le coefficient solaire
// Journal de chauffe (mode full) : un segment = une chauffe continue (arrets < HEAT_SEG_GAP_SEC fusionnes) avec UNE raison.
constexpr uint16_t HEAT_SEG_MAX        = 120;     // segments gardes (~14 jours), 12 octets chacun en RAM
constexpr uint32_t HEAT_SEG_GAP_SEC    = 900;     // une pause plus courte (thermostat) ne coupe pas le segment
constexpr uint32_t HEAT_SAVE_MIN_MS    = 600000UL; // ecriture de /heat.json au plus toutes les 10 min (et a chaque fin de cycle)
constexpr uint32_t HISTORY_SAVE_PERIOD_MS = 600000UL; // ecriture de /history.json au plus toutes les 10 min (avant : 60 s) ; sauvegardes forcees conservees : fin de cycle, checkpoint, import, redemarrage auto Ecojoko
// Intervalles « relais allume » (etat CONFIRME par le Shelly), affiches sous la frise du journal : un par allumage, 8 octets chacun en RAM.
constexpr uint16_t RELAY_IV_MAX        = 160;     // ~14 jours ; stockes dans /heat.json avec les segments
// Temperature interne du Shelly (diagnostic de surchauffe) : un point toutes les 5 min sur 48 h, en RAM (4 octets par point).
constexpr uint16_t STEMP_N        = 576;
constexpr uint32_t STEMP_STEP_SEC = 300;
constexpr uint8_t  HR_HC = 0, HR_SOLAR = 1, HR_FORCE = 2; // raisons : heures creuses / soleil rentable / rattrapage du plan

enum ErrorSource : uint8_t { ERROR_NONE, ERROR_SHELLY, ERROR_ECOJOKO, ERROR_STORAGE, ERROR_THERMAL };

/* Types utilises dans des signatures de fonctions : declares ICI, avant toute fonction, car Arduino genere
   automatiquement des prototypes en haut du fichier, avant les definitions de types plus bas. */
enum WdCause : uint8_t { WD_OK, WD_CRASH_LOOP, WD_RELAY, WD_MEMORY, WD_SHELLY, WD_COMMAND, WD_CLOCK, WD_OVERHEAT };

// Un enregistrement par cycle (jour) pour la prevision : voir section 6a.
struct AntRec {
  uint32_t cycle  = 0;
  uint32_t sec    = 0;     // duree couverte par des mesures valides entre deux "ballon plein"
  float    wh     = 0.0f;  // energie injectee sur cette duree (= consommation du ballon)
  float    rad    = 0.0f;  // rayonnement cumule (Wh/m2) sur les ticks ou le surplus etait connu
  float    pot    = 0.0f;  // surplus solaire utilisable par le ballon (Wh) sur ces memes ticks
  float    skipWh = 0.0f;  // mode shadow : energie de HC que le plan aurait evitee
  int16_t  t10    = INT16_MIN; // temperature exterieure moyenne x10 (INT16_MIN = inconnue)
  uint8_t  wday   = 0;     // jour de semaine du cycle (0 = dimanche)
  float    predWh    = -1.0f; // besoin 24 h PREVU au debut du cycle (Wh ; < 0 = inconnu) : sert a juger la prevision
  float    predSolWh = -1.0f; // surplus solaire utilisable PREVU sur 24 h au debut du cycle (Wh ; < 0 = inconnu), a comparer a pot
};

// Un creneau horaire pour comparer le solaire PREVU et REEL (anneau indexe par heure epoch % ANT_HR_N).
// r = surplus solaire reel moyen de l'heure (W, plafonne a heaterW) ; p = prevision figee au debut du cycle (W). 0xFFFF = inconnu.
struct AntHr {
  uint32_t h = 0;       // heure epoch (epoch / 3600) ; 0 = creneau libre
  uint16_t r = 0xFFFF;
  uint16_t p = 0xFFFF;
};

// Ligne d'historique (jour ou mois) manipulee par l'import : voir section 7b.
struct ImpRow {
  uint32_t key = 0;
  uint32_t u[5] = {0, 0, 0, 0, 0};
  float    f[6] = {0, 0, 0, 0, 0, 0};
  char     mode[12] = "auto";
};

/* Liaison avec l'ESP32-C3 « chien de garde » (section 9 ci-dessous, et watchdog_c3/watchdog_c3.ino). */
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

/* ---------- Diagnostic de blocage (sans moniteur serie) ----------
   Une tache sur l'autre coeur surveille loop(). Si loop() ne tourne plus depuis STALL_RESTART_MS,
   l'ESP32 se redemarre seul et note la cause. Les infos survivent au redemarrage logiciel (memoire RTC)
   puis sont ecrites dans /boot_log.txt, consultable sur http://<ip-de-l-esp>/api/diag. */
constexpr uint32_t STALL_RESTART_MS   = 90000; // > pire cas legitime d'un passage de controlTask() (~45 s : poll Shelly 2 essais ~13 s + Ecojoko ~24 s + un Switch.Set ~6,5 s)
constexpr uint32_t STALL_CHECK_MS     = 5000;

/* ---- Separation web / controle (FreeRTOS) ----
   Le serveur web (loop() cote Arduino, appLoop()) et la boucle de controle (controlTask(), tache dediee)
   tournent desormais independamment : une requete web lente ou bloquee ne doit plus jamais retarder le
   pilotage du relais ni le battement de coeur du watchdog C3. ioMutex protege l'etat partage (cfg, rt,
   historiques en RAM, acces LittleFS) entre les deux : pris par chaque route web (webRoute(), un seul
   point de passage) et par chaque passage de controlTask(). Attentes bornees (jamais portMAX_DELAY) :
   si l'un des deux cote ne peut pas obtenir le verrou a temps, il renonce pour CE passage/CETTE requete
   plutot que de risquer un blocage mutuel ; le detecteur de blocage existant (diagLoopMs, mis a jour par
   controlTask() avant meme de prendre le verrou) reste le filet de securite ultime.
   Correctif (15/11) apres boucle de "watchdog tache" (ESP_RST_TASK_WDT) en Wi-Fi tres degrade (RSSI
   -80 a -93 dBm) : controlTask() fait les appels Ecojoko/Shelly (bloquants, jusqu'a 8 s chacun) EN TENANT
   ioMutex. Deux consequences observees ensemble dans ces conditions :
   1) toute route web (meme /api/status, un simple poll) attend alors le verrou jusqu'a IO_MUTEX_WEB_WAIT_TICKS
      avant de s'executer -> web_lent de plusieurs secondes sur des endpoints qui ne font pourtant rien de
      lourd (le "rien pendant ~1 min" ressenti a l'import n'etait pas l'import : c'etait l'attente du verrou) ;
      IO_MUTEX_WEB_WAIT_TICKS reduit a 8 s (~ECO_HTTP_TIMEOUT_MS, le temps d'un seul aller-retour Ecojoko) pour
      que l'UI echoue vite (503) plutot que de sembler figee.
   2) controlTask() etait epingle sur le MEME coeur (0) que les taches internes Wi-Fi/lwIP. Un appel Ecojoko
      bloquant sur ce coeur, pendant un episode Wi-Fi tres degrade ou le Wi-Fi consomme deja beaucoup de CPU
      sur ce coeur, peut suffire a affamer la tache idle du coeur 0 au-dela du delai du watchdog materiel ->
      reset ESP_RST_TASK_WDT (independant du detecteur de blocage maison, qui lui ne s'est jamais declenche).
      Avant la separation web/controle, ces memes appels bloquants tournaient dans la tache Arduino (loopTask,
      coeur 1), a l'ecart du Wi-Fi : controlTask() est donc repassee sur le coeur 1 (cf. setup()) pour retrouver
      cet isolement, stallMonitorTask restant seule sur le coeur 0 (surveillance croisee d'un coeur par l'autre,
      intentionnelle). */
static SemaphoreHandle_t ioMutex = nullptr;
constexpr TickType_t IO_MUTEX_WEB_WAIT_TICKS  = pdMS_TO_TICKS(8000);  // ~ECO_HTTP_TIMEOUT_MS : un seul aller-retour Ecojoko, pas 50 s d'attente silencieuse (voir correctif 15/11 ci-dessus)
constexpr TickType_t IO_MUTEX_CTRL_WAIT_TICKS = pdMS_TO_TICKS(20000); // tres genereux pour l'execution d'un seul handler web (pas de reseau dedans, juste JSON/LittleFS)
constexpr uint32_t   CONTROL_TASK_DELAY_MS    = 200; // frequence des verifications rapides (watchdog C3, confirmation Shelly...) ; sample()/tick() restent cadences par CONTROL_PERIOD_MS
constexpr uint32_t DIAG_RTC_MAGIC     = 0xB411A003; // change a chaque ajout de variables RTC : la 1re mise en route est vue comme "alimentation coupee"
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
RTC_NOINIT_ATTR static uint32_t rtcCrashCount; // plantages successifs (panic, watchdog, blocage) dans la fenetre courante
RTC_NOINIT_ATTR static uint32_t rtcCrashSpanS; // duree ecoulee depuis le 1er plantage de la serie (somme des uptimes)

static const char *volatile diagPhase = "demarrage";
static volatile uint32_t diagLoopMs = 0;

/* ---------- Instrumentation web : QUELLE requete bloque loop() ? ----------
   Chaque route est enregistree via webRoute() qui note l'URI et l'IP du client. Ces infos sont recopiees en memoire RTC
   par la tache de surveillance, donc lisibles apres un redemarrage (ligne "reboot" de /boot_log.txt, /api/diag).
   Toute requete plus lente que WEB_SLOW_MS est aussi ecrite dans /boot_log.txt, meme sans plantage. */
constexpr uint32_t WEB_SLOW_MS = 2000;  // seuil "requete lente"
constexpr uint8_t  WEB_RING    = 12;    // dernieres requetes gardees en RAM (visibles dans /api/diag)
RTC_NOINIT_ATTR static char     rtcLastWebUri[24];
RTC_NOINIT_ATTR static char     rtcLastWebIp[16];
RTC_NOINIT_ATTR static uint32_t rtcLastWebAgeS; // 0 = aucun traitement web en cours au dernier releve, sinon depuis combien de s

static char diagWebUri[24] = "";
static char diagWebIp[16]  = "";
static volatile uint32_t diagWebStartMs = 0; // debut du handleClient() en cours (0 = pas en cours)
struct WebReq { uint32_t epoch; uint32_t ms; char uri[24]; char ip[16]; };
static WebReq  webRing[WEB_RING];
static uint8_t webRingHead = 0, webRingCount = 0;
static uint32_t webLastSlowLogMs = 0;

// Marque la section de code en cours (restauree a la sortie) : indique ou loop() etait bloque.
struct DiagPhaseScope {
  const char *prev;
  explicit DiagPhaseScope(const char *p) : prev(diagPhase) { diagPhase = p; }
  ~DiagPhaseScope() { diagPhase = prev; }
};

/* --------------- Configuration persistée (LittleFS) ---------------- */
struct Config {
  String wifiSsid, wifiPassword, ecoEmail, ecoPassword, shellyIp;
  String mode = "full"; // auto | manual | monitor | full (full = mode principal : defaut, et remis apres chaque flash)
  String lastFwBuild;   // FW_BUILD du dernier demarrage : s'il change, un nouveau firmware vient d'etre flashe -> mode full
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
  int8_t   wdHbPin          = 13; // C3 : broche "battement de coeur" (-1 = desactive)
  int8_t   wdErrPin         = 14; // C3 : broche "alarme" (-1 = desactive)
  uint16_t wdProblemMin     = 60; // minutes de probleme tolerees avant d'alerter (Shelly injoignable, commandes KO, horloge)
  int8_t   wdTxPin          = 26; // C3 : liaison serie « silence » (TX de l'ESP32 vers RX du C3 ; -1 = desactive = l'alarme ne se tait jamais)
  bool     muteOn           = false; // plage silencieuse : le C3 ne sonne pas pendant la plage (sauf surchauffe)
  uint16_t muteStart        = 23 * 60, muteEnd = 7 * 60; // debut/fin de la plage (minutes apres minuit)
  uint16_t snoozeMin        = 480;   // duree du silence a distance (bouton du bandeau rouge), en minutes
  // Prevision (mode full) : voir section 6a
  String   antMode          = "shadow"; // off = full classique | shadow = calcule sans agir | on = pilote les HC
  bool     antShade         = false;    // applique le facteur d'ombrage appris a la decision (false = appris et affiche seulement)
  uint16_t tankLiters       = 200;      // volume du ballon (sert a estimer la capacite si tankKWh = 0)
  float    tankKWh          = 0.0f;     // capacite utile en kWh (energie "froid -> plein") ; 0 = estimee : litres x 52,3 Wh
  uint8_t  reservePct       = 40;       // reserve d'eau chaude a garder avant le premier soleil utile (% capacite)
  uint16_t deadlineMin      = 18 * 60 + 30; // controle avant la soiree : reserve + soiree a tenir (le ballon n'a pas a etre plein)
  float    lat              = 50.40f;   // coordonnees pour la meteo (approximatives, modifiables dans /ant)
  float    lon              = 3.15f;
  float    pvKwc            = 0.0f;     // puissance crete installee (kWc) : sert a deduire un coefficient solaire de depart ; 0 = aucun (apprentissage seul)
  // Reseau : IP fixe optionnelle (repli DHCP automatique) et AP de diagnostic apres le demarrage
  bool     staticIp         = false;
  String   ip, gw, mask = "255.255.255.0", dns;
  uint16_t bootApSec        = 120;      // duree de l'AP de diagnostic apres un demarrage reussi (0 = desactive)
  // Protection thermique du Shelly : au-dela de shellyTMaxC (0 = seuil desactive) ou si le Shelly signale « overtemp », le relais est
  // coupe et plus aucun ordre d'allumage n'est envoye. Reprise quand le Shelly est sous shellyTResumeC ET apres shellyHoldMin minutes.
  float    shellyTMaxC      = 85.0f;
  float    shellyTResumeC   = 65.0f;
  uint16_t shellyHoldMin    = 10;
  // Seuil d'ALARME (buzzer du watchdog, sans delai) si la temperature du Shelly le depasse (0 = desactive). Independant de la coupure.
  float    shellyTAlarmC    = 100.0f;
};

/* --------------- État runtime (RAM) ---------------- */
struct Runtime {
  float ecoRaw         = NAN, ecoSmooth   = NAN, shellyW   = NAN, restHouse   = NAN;
  float shellySmooth   = NAN;   // puissance Shelly lissee (meme filtre que ecoSmooth, evite le decalage d'inertie)
  bool  shellyOn       = false, forcedNight = false, probing   = false;
  bool  manualOn       = false;
  float shellyT        = NAN;   // temperature interne du Shelly (tC), lue avec Switch.GetStatus
  bool  shellyOvertemp = false; // le Shelly signale errors:["overtemp"] en ce moment
  uint32_t shellyOtEpoch = 0;   // derniere surchauffe vue (epoch, RAM : remis a 0 au redemarrage)
  bool     thermHold = false;   // protection thermique active : on n'envoie plus d'ordre d'allumage
  bool     thermCritLogged = false;   // alerte « seuil d'alarme depasse » deja ecrite pour cet episode
  uint32_t thermHoldStartMs = 0, thermHoldCount = 0;   // debut de la pause en cours (rearme tant qu'il fait trop chaud) / declenchements depuis le demarrage
  bool  ecoConnected   = false, shellyConnected = false; 
  uint32_t manualSetMs = 0;
  uint32_t cycleId              = 0, heatedSec      = 0, tier1Sec     = 0;
  uint32_t tier2Sec             = 0, nightSec       = 0, relayOnSec   = 0;
  uint32_t monthKey             = 0, monthHeatedSec = 0, monthTier1Sec = 0;
  uint32_t monthTier2Sec        = 0, monthNightSec  = 0, monthRelayOnSec = 0;
  // Energie et cout reels (mesures Shelly x surplus solaire reel), voir accumulateEnergyCost().
  // Remplacent l'ancienne estimation duree x puissance nominale (cfg.heaterW).
  float    cycleWh = 0.0f, paidWh = 0.0f, nativeEUR = 0.0f, paidEUR = 0.0f;
  float    monthCycleWh = 0.0f, monthPaidWh = 0.0f, monthNativeEUR = 0.0f, monthPaidEUR = 0.0f;
  uint32_t lastTick             = 0, lowPowerSince  = 0;
  uint32_t lastHistorySave      = 0;
  uint32_t lastStateChangeMs    = 0; // heure du dernier passage ON->OFF (pour le debounce)
  uint32_t lastWifiCheckMs      = 0, wifiDisconnectedSinceMs = 0;
  bool     apRescue = false, bootApOn = false, bootApDone = false, staticFallback = false; // AP de secours (probleme Wi-Fi) / AP de diagnostic 2 min / deja traite ce boot / IP fixe abandonnee
  uint32_t bootApStartMs        = 0, wifiUpSinceMs = 0, wifiLostSinceMs = 0;
  uint32_t lastShellyPollMs     = 0, lastShellyCmdMs = 0, lastEcoPollMs = 0;
  uint32_t lastShellySuccessMs  = 0, lastEcoSuccessMs = 0;
  uint32_t errorResolvedMs      = 0, errorDayKey = 0, errorCountToday = 0;
  uint32_t errorRepeatCount     = 0; // occurrences du meme episode d'erreur (meme errorKey) depuis la 1ere
  uint8_t  shellyFailCount      = 0, ecoFailCount = 0;
  uint32_t shellyLastMs         = 0, shellyMaxMs = 0, shellyRetryRecovered = 0; // latence GET Shelly (ms) et -11 recuperes par retry
  ErrorSource errorSource       = ERROR_NONE;
  String state    = "INIT", error     = "", errorKey = "", ecoCookie = "";
  String gatewayId = "", powerMeterId = ""; 
} rt;

/* --------------- Journal d'erreurs (RAM, glissant sur 24h) ---------------- */
constexpr uint8_t ERROR_LOG_CAPACITY = 40; // suffisant pour 24h d'erreurs intermittentes ; RAM uniquement, perdu au reboot
struct ErrorLogEntry {
  uint32_t epoch = 0; // 0 si horloge invalide au moment de l'evenement
  ErrorSource source = ERROR_NONE;
  char message[72] = {0};
};
ErrorLogEntry errorLog[ERROR_LOG_CAPACITY];
uint8_t errorLogHead  = 0; // prochain emplacement a ecrire
uint8_t errorLogCount = 0; // nombre d'entrees valides (jusqu'a ERROR_LOG_CAPACITY)

static const char* errorSourceName(ErrorSource s) {
  switch (s) {
    case ERROR_SHELLY:  return "SHELLY";
    case ERROR_ECOJOKO: return "ECOJOKO";
    case ERROR_STORAGE: return "STORAGE";
    case ERROR_THERMAL: return "THERMIQUE";
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

// Reecrit en place la DERNIERE entree du journal (celle juste avant errorLogHead), sans avancer le ring :
// utilise quand reportError() voit une repetition du meme episode (meme cle de deduplication), pour
// rafraichir le message/l'heure de cet episode au lieu d'empiler une ligne par tentative.
static void updateLastErrorLog(const String &message, ErrorSource source) {
  if (errorLogCount == 0) { pushErrorLog(message, source); return; } // ne devrait pas arriver ici, filet de securite
  uint8_t lastIdx = (errorLogHead + ERROR_LOG_CAPACITY - 1) % ERROR_LOG_CAPACITY;
  ErrorLogEntry &e = errorLog[lastIdx];
  time_t epoch = time(nullptr);
  e.epoch  = (epoch < 946684800) ? 0 : (uint32_t)epoch; // heure de la derniere occurrence
  e.source = source;
  strncpy(e.message, message.c_str(), sizeof(e.message) - 1);
  e.message[sizeof(e.message) - 1] = 0;
}

Config cfg;
WebServer server(80);

// IP valide ? (utilise par la configuration reseau)
static bool parseIp(const String &s, IPAddress &out) {
  return s.length() >= 7 && out.fromString(s.c_str());
}

// Note l'URI et l'IP de la requete en cours (appele au debut de chaque handler).
static void diagWebTag(const char *uri) {
  strncpy(diagWebUri, uri, sizeof(diagWebUri) - 1);
  diagWebUri[sizeof(diagWebUri) - 1] = '\0';
  String ip = server.client().remoteIP().toString();
  strncpy(diagWebIp, ip.c_str(), sizeof(diagWebIp) - 1);
  diagWebIp[sizeof(diagWebIp) - 1] = '\0';
}

// Remplace server.on(uri, methode, handler) : meme comportement, plus l'instrumentation.
static void webRoute(const char *uri, HTTPMethod method, std::function<void()> fn) {
  server.on(uri, method, [uri, fn]() {
    diagWebTag(uri);
    // Etat partage (cfg/rt/historiques/LittleFS) avec controlTask() : voir le commentaire pres de ioMutex.
    if (xSemaphoreTake(ioMutex, IO_MUTEX_WEB_WAIT_TICKS) == pdTRUE) {
      fn();
      xSemaphoreGive(ioMutex);
    } else {
      server.send(503, "text/plain", "Systeme occupe (cycle de controle en cours), reessayez dans quelques secondes");
    }
  });
}

/* --------------- Prevision : enregistrements par cycle + etat (RAM) ---------------- */
static AntRec  antRecs[ANT_REC_MAX];
static uint8_t antNRec = 0;
static AntHr   antHr[ANT_HR_N];            // anneau horaire prevu / reel (voir antHrIdx)
static uint32_t antHrCur = 0;              // heure epoch en cours d'accumulation (0 = aucune)
static float    antHrWh = 0.0f;            // surplus utilisable accumule sur l'heure en cours (Wh)
static uint32_t antHrSec = 0;              // secondes mesurees sur l'heure en cours
// Ombrage : rapport reel / prevu x100 par heure de la journee (heure UTC : la position du soleil ne suit pas le changement d'heure).
static uint8_t  antShadeS[24][ANT_SHADE_N]; // anneau d'echantillons par heure (seuls les antShadeCnt[h] premiers sont valides)
static uint8_t  antShadeHead[24], antShadeCnt[24];
static float    antShadeF[24];             // facteur appris (1.0 = neutre), valide seulement si antShadeOk[h]
static bool     antShadeOk[24];
static float    antShadeRef = NAN;         // niveau de reference : mediane des medianes horaires

// Journal de chauffe : voir heatLogTick(). lastTickWh/PaidWh sont renseignes par accumulateEnergyCost().
// sec/wh/paidWh en uint32 (et non uint16) : un segment de plus de 18 h (65535 s) ou de plus de 65535 Wh
// se tronquait silencieusement avant ce correctif ; un systeme prevu pour tourner plusieurs annees doit
// eviter ce cas, meme rare.
struct HeatSeg {
  uint32_t t0 = 0;       // debut (epoch)
  uint32_t sec = 0;      // duree du debut a la derniere seconde de chauffe
  uint32_t wh = 0;       // energie injectee dans le ballon
  uint32_t paidWh = 0;   // dont achetee au reseau (hors surplus solaire)
  uint8_t  reason = 0;   // HR_HC / HR_SOLAR / HR_FORCE
};
static HeatSeg  heatSegs[HEAT_SEG_MAX];
static uint16_t heatNSeg = 0;                 // le plus ancien est a l'indice 0
static HeatSeg  heatCur;                      // segment en cours
static bool     heatCurOpen = false;
static uint32_t heatCurIdle = 0;
static float    heatCurWh = 0.0f, heatCurPaid = 0.0f;
static bool     heatDirty = false;
static uint32_t heatLastSaveMs = 0;
static float    lastTickWh = 0.0f, lastTickPaidWh = 0.0f;

// Journal de chauffe, ligne « relais » : voir relayLogTick(). Un intervalle = relais confirme ON, de l'allumage a l'extinction.
struct RelayIv {
  uint32_t t0 = 0;       // debut (epoch)
  uint32_t sec = 0;      // duree jusqu'a la derniere mesure ON (uint32, meme raison que HeatSeg.sec)
};
static RelayIv  relayIvs[RELAY_IV_MAX];
static uint16_t relayNIv = 0;                 // le plus ancien est a l'indice 0
static RelayIv  relayCur;                     // intervalle en cours
static bool     relayCurOpen = false;
static uint32_t relayCurUnknown = 0;          // secondes sans etat Shelly connu pendant l'intervalle en cours

/* Temperature interne du Shelly : anneau de STEMP_N points (un par STEMP_STEP_SEC). t10 = maximum de l'intervalle en
   degC x10 (INT16_MIN = pas de mesure), w = puissance moyenne (W). Alimente par stempFeed() a chaque lecture Shelly reussie. */
struct STempPt { int16_t t10; uint16_t w; };
static STempPt  stPts[STEMP_N];
static uint16_t stCount = 0, stHead = 0;      // points valides / prochain indice d'ecriture
static uint32_t stLastEpoch = 0;              // fin du dernier intervalle clos (multiple de STEMP_STEP_SEC)
static int16_t  stMax10 = INT16_MIN;          // intervalle en cours
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
    stCount = 0; stHead = 0; stMax10 = INT16_MIN; stWSum = 0; stWN = 0;   // premiere mesure ou trou > 48 h : on repart de zero
    stLastEpoch = e - (e % STEMP_STEP_SEC);
  }
  while (e >= stLastEpoch + STEMP_STEP_SEC) {   // clot les intervalles termines (les trous sans mesure donnent des points vides)
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
  // Meteo
  uint32_t wxStart = 0, wxEpoch = 0, wxLastTryMs = 0, wxLastOkMs = 0;
  uint8_t  wxN = 0, wxFail = 0;
  int16_t  wxT10[ANT_WX_MAX];
  uint16_t wxRad[ANT_WX_MAX];
  char     wxErr[24] = {0};
  // Etat du ballon vu comme une batterie
  bool     refValid = false, refBad = false, wasHeating = false;
  uint32_t refEpoch = 0, lastFullEpoch = 0, gapSec = 0, heatRun = 0, idleRun = 0;
  float    whSince = 0.0f;                 // energie injectee depuis le dernier "plein"
  float    errAct = 0.0f, errPred = 0.0f;  // ecart modele / realite (sommes a poids decroissant)
  // Cumuls du cycle en cours
  float    cycMeasWh = 0.0f, cycRadWh = 0.0f, cycPotWh = 0.0f, cycSkipWh = 0.0f;
  float    cycPredWh = -1.0f, cycPredSolWh = -1.0f; // prevision figee au debut du cycle en cours (< 0 = pas encore prise)
  uint32_t cycMeasSec = 0, cycTSec = 0;
  int32_t  cycTSum10 = 0;
  // Apprentissage
  float    needBase = 0.0f, slope = 0.0f, tRef = 0.0f, k = 0.0f, kSumRad = 0.0f, kLearn = 0.0f, kW = 0.0f; // k = coefficient effectif ; kLearn = appris seul ; kW = part de l'appris (0..1)
  float    tBase = NAN;   // temperature moyenne des cycles qui ont servi a calculer needBase (reference reelle de la correction)
  bool     slopeValid = false, kValid = false;
  uint8_t  needN = 0, kN = 0;
  // Decision courante
  bool     valid = false, heat = true, force = false, heatPlan = false, forcing = false;
  float    dNowWh = 0.0f, hWh = 0.0f, erechWh = 0.0f, needDayWh = 0.0f;
  uint32_t tsEpoch = 0, tdEpoch = 0;
  char     reason[28] = "demarrage";
};
static AntState ant;

/* --------------- Utilitaires temps ---------------- */
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
  if (!localNow(local)) return "horloge-invalide";
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

// Variante avec une cle de deduplication distincte du message affiche : a utiliser quand le message contient
// un detail qui varie a chaque tentative (duree, etc.) et casserait sinon la deduplication a chaque appel,
// en noyant le journal d'une ligne par tentative au lieu d'une ligne par episode (avec compteur). Le message
// affiche (rt.error, /api/status) reste toujours a jour avec le dernier texte recu.
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
  reportError(message, message, source); // pas de cle distincte : deduplication identique a avant (texte exact)
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
  Serial.printf("[%s] echec reseau etape=%s http=%d wifiStatus=%d rssi=%d heure=%s\n",
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

// Redemarrage automatique : Ecojoko injoignable (Wi-Fi OK) depuis plus de 30 min. Le backoff n'est PAS modifie.
constexpr uint32_t ECO_BLOCKED_RESTART_MS = 30UL * 60UL * 1000UL;

// Duree (ms) depuis laquelle Ecojoko est injoignable alors que le Wi-Fi fonctionne. 0 si tout va bien, si Ecojoko n'est pas
// configure ou si le Wi-Fi est coupe (cas gere ailleurs). Le decompte repart de la reconnexion Wi-Fi : une coupure Wi-Fi
// ne declenche donc jamais de redemarrage des le retour du reseau.
static uint32_t ecoBlockedAgeMs(uint32_t now) {
  if (cfg.ecoEmail.length() == 0 || cfg.ecoPassword.length() == 0) return 0;
  if (rt.ecoFailCount == 0 || rt.wifiUpSinceMs == 0 || WiFi.status() != WL_CONNECTED) return 0;
  uint32_t sinceOk = (uint32_t)(now - rt.lastEcoSuccessMs); // lastEcoSuccessMs = 0 : jamais reussi depuis le boot
  uint32_t sinceWifi = (uint32_t)(now - rt.wifiUpSinceMs);
  return sinceOk < sinceWifi ? sinceOk : sinceWifi;
}

static void recordShellyResult(bool success) {
  rt.shellyFailCount = success ? 0 : (rt.shellyFailCount < UINT8_MAX ? rt.shellyFailCount + 1 : UINT8_MAX);
  if (!success) {
    Serial.printf("[Shelly] echec consecutif=%u prochain_poll_dans=%lums\n",
                  rt.shellyFailCount, (unsigned long)networkBackoffMs(rt.shellyFailCount));
  }
}

static void recordShellyPollResult(const char *origin, bool success) {
  if (strcmp(origin, "poll") == 0) recordShellyResult(success);
}

static void recordEcoResult(bool success) {
  rt.ecoFailCount = success ? 0 : (rt.ecoFailCount < UINT8_MAX ? rt.ecoFailCount + 1 : UINT8_MAX);
  if (!success) {
    Serial.printf("[Ecojoko] echec consecutif=%u prochain_poll_dans=%lums\n",
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
   1. GESTION DE CONFIGURATION (LittleFS)
   ============================================================ */
constexpr char PASS_MASK[] = "********"; // valeur renvoyee a la place des mots de passe par GET /api/config ; renvoyee telle quelle = "inchange"

// Bornes de securite : une valeur absurde (0, negative, hors plage, faute de frappe) revient a la valeur par defaut.
// Evite notamment smoothingMin = 0 (division par zero -> lissage NaN -> plus aucune decision solaire) et heaterW = 0.
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

  // Champs ajoutes apres coup (watchdog, plage silencieuse, prevision, reseau, protection thermique Shelly) :
  // memes bornes que la validation de /api/config (POST), mais appliquees ICI a chaque chargement (boot, POST,
  // et toute restauration de parametres), pour qu'un /config.json corrompu ou un import incomplet ne puisse
  // jamais faire tourner le systeme avec une valeur hors plage. Correction silencieuse (valeur par defaut),
  // comme le reste de cette fonction : pas de blocage du demarrage.
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
    if (!parseIp(cfg.shellyIp, a)) cfg.shellyIp = ""; // IP malformee : desactive plutot que de tenter des requetes vers une adresse invalide
  }
  if (cfg.staticIp) {
    IPAddress a;
    bool netOk = parseIp(cfg.ip, a) && parseIp(cfg.gw, a) && parseIp(cfg.mask, a) && (!cfg.dns.length() || parseIp(cfg.dns, a));
    if (!netOk) cfg.staticIp = false; // config IP fixe incoherente : repli DHCP plutot que de demarrer avec une IP invalide
  }
}

bool readConfig() {
  Serial.println("[DEBUG] Lecture du fichier de configuration...");
  File f = LittleFS.open("/config.json", "r"); 
  if (!f) {
    Serial.println("[DEBUG] Aucun fichier config.json trouvé.");
    return false; 
  }
  JsonDocument d; 
  DeserializationError err = deserializeJson(d, f);
  f.close();
  if (err || !d.is<JsonObject>()) {
    Serial.println("[DEBUG] Erreur de parsing du JSON de config.");
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
  // Toutes les bornes de securite (broches, plages, temperatures, IP...) sont desormais verifiees
  // UNIQUEMENT ici, par sanitizeConfig() : plus de double controle a maintenir en parallele (c'est cet
  // ecart entre readConfig() et sanitizeConfig() qui avait laisse passer muteStart/muteEnd/snoozeMin/
  // tankKWh/pvKwc/bootApSec/shellyT* au chargement direct de /config.json jusqu'ici).
  sanitizeConfig();
  
  Serial.println("[DEBUG] Configuration chargée avec succès.");
  return true;
}

void saveConfig() {
  Serial.println("[DEBUG] Sauvegarde de la configuration...");
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
    Serial.println("[DEBUG] Configuration sauvegardée.");
  }
}

/* ============================================================
   2. API SHELLY 1PM GEN3 (RPC)
   ============================================================ */
static bool readShelly(const char *origin = "poll");

/* Type d'erreur Shelly, lisible dans le journal d'erreurs web (sans moniteur serie).
   Message volontairement STABLE (pas de duree/RSSI) : reportError() ne pousse une nouvelle entree
   que si le message change, donc un meme type d'echec repete ne pollue pas le journal.
   Duree et RSSI restent dans le log Serial (logNetworkFailure + ligne DUREE_MS). */
static String shellyHttpDetail(int code) {
  if (WiFi.status() != WL_CONNECTED) return "Wi-Fi ESP32 coupe";
  if (code > 0) return "reponse HTTP " + String(code);
  const char *label;
  switch (code) {
    case HTTPC_ERROR_CONNECTION_REFUSED:  label = "connexion refusee";      break; // Shelly sature/ferme le port
    case HTTPC_ERROR_SEND_HEADER_FAILED:
    case HTTPC_ERROR_SEND_PAYLOAD_FAILED: label = "envoi requete echoue";   break;
    case HTTPC_ERROR_NOT_CONNECTED:       label = "socket non connectee";   break;
    case HTTPC_ERROR_CONNECTION_LOST:     label = "connexion perdue";       break; // coupee pendant l'echange
    case HTTPC_ERROR_NO_STREAM:
    case HTTPC_ERROR_NO_HTTP_SERVER:      label = "reponse non HTTP";       break;
    case HTTPC_ERROR_TOO_LESS_RAM:        label = "RAM ESP32 insuffisante"; break;
    case HTTPC_ERROR_READ_TIMEOUT:        label = "timeout reponse";        break; // Shelly n'a pas repondu a temps
    default:                              label = "erreur transport";       break;
  }
  return String(label) + " (" + String(code) + ")";
}

/* ---- Commande du relais Shelly : NON bloquante ----
   Avant : shellySet() restait dans loop() jusqu'a 3 x (Switch.Set + attente 2 s + relecture) ~ 45 s au pire, pendant lesquelles
   le serveur web ne repondait plus. Maintenant : shellySet() envoie UN Switch.Set puis rend la main. La confirmation (relecture
   de l'etat SHELLY_CONFIRM_DELAY_MS plus tard) et les tentatives suivantes sont faites par shellyCmdService(), appele a chaque
   passage de loop() : le web reste servi pendant l'attente. Resultat final (succes confirme / echec apres 3 tentatives) :
   recordShellyResult(), wdNoteCommand() et journal d'erreurs, exactement comme avant. */
struct ShellyCmd {
  bool     active         = false;
  bool     on             = false;
  uint8_t  attempt        = 0;     // tentative en cours (1..SHELLY_MAX_ATTEMPTS)
  uint32_t sentMs         = 0;     // instant d'envoi du Switch.Set en cours
  int      lastCode       = -1;    // code HTTP du dernier Switch.Set (-1 = non envoye)
  bool     lastPrepared   = true;  // false si h.begin() a echoue
  bool     lastStatusRead = false; // derniere lecture d'etat de confirmation reussie
} shellyCmd;

// Envoie le Switch.Set de la tentative en cours (bloque au plus connect + lecture, ~6,5 s). Ne confirme rien.
static void shellyCmdSend() {
  DiagPhaseScope phase("shelly_set");
  Serial.printf("[Shelly] tentative %u/%u ETAPE=DEBUT commande=%s\n",
                shellyCmd.attempt, SHELLY_MAX_ATTEMPTS, shellyCmd.on ? "ON" : "OFF");
  HTTPClient h;
  h.setConnectTimeout(SHELLY_CONNECT_TIMEOUT_MS);
  h.setTimeout(SHELLY_READ_TIMEOUT_MS);
  h.setReuse(false); // Connection: close -> libere le slot du Shelly des la reponse
  String url = "http://" + cfg.shellyIp + "/rpc/Switch.Set?id=0&on=" + (shellyCmd.on ? "true" : "false");
  int code = -1;
  shellyCmd.lastPrepared = h.begin(url);
  if (shellyCmd.lastPrepared) {
    Serial.printf("[Shelly] tentative %u ETAPE=ENVOI_HTTP commande=%s\n", shellyCmd.attempt, shellyCmd.on ? "ON" : "OFF");
    code = h.GET();
    h.end();
    Serial.printf("[Shelly] tentative %u ETAPE=REPONSE_HTTP code=%d\n", shellyCmd.attempt, code);
  } else {
    Serial.printf("[Shelly] tentative %u ETAPE=ECHEC_PREPARATION_HTTP (aucune requete envoyee)\n", shellyCmd.attempt);
  }
  shellyCmd.lastCode = code;
  shellyCmd.sentMs = millis();
}

// Demande l'etat « on » du relais. Retourne true si un ordre est envoye ou deja en cours (la confirmation arrive plus tard),
// false si rien n'a pu etre envoye (IP manquante, backoff reseau en cours).
static bool shellySet(bool on) {
  if (!cfg.shellyIp.length()) {
    reportError("IP Shelly manquante", ERROR_SHELLY);
    wdNoteCommand(false);
    return false;
  }
  uint32_t now = millis();
  if (shellyCmd.active) {
    if (shellyCmd.on == on) return true; // meme ordre deja en cours : on attend sa confirmation, pas de renvoi
    Serial.printf("[Shelly] ordre %s remplace l'ordre %s en cours\n", on ? "ON" : "OFF", shellyCmd.on ? "ON" : "OFF");
    // Ordre inverse : il remplace l'ordre en cours, sans passer par le backoff (le Shelly vient de repondre).
  } else if (!networkPollDue(rt.lastShellyCmdMs, rt.shellyFailCount, now)) {
    Serial.printf("[Shelly] commande=%s differee (backoff en cours, echecs consecutifs=%u, prochain_essai_dans=%lums)\n",
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

// Appele a chaque passage de loop() : confirme l'ordre en cours une fois le delai ecoule, ou relance une tentative.
static void shellyCmdService(uint32_t now) {
  if (!shellyCmd.active) return;
  if ((uint32_t)(now - shellyCmd.sentMs) < SHELLY_CONFIRM_DELAY_MS) return;
  if (cfg.mode == "monitor") { shellyCmd.active = false; return; } // monitor : ballon non pilote, plus rien a confirmer

  DiagPhaseScope phase("shelly_confirm");
  const bool on = shellyCmd.on;
  const uint8_t attempt = shellyCmd.attempt;
  Serial.printf("[Shelly] tentative %u ETAPE=LECTURE_ETAT\n", attempt);
  bool statusRead = readShelly("confirmation");
  shellyCmd.lastStatusRead = statusRead;
  Serial.printf("[Shelly] tentative %u ETAPE=ETAT_LU lecture=%s reel=%s puissance=%.1fW\n",
                attempt, statusRead ? "OK" : "ECHEC", rt.shellyOn ? "ON" : "OFF", rt.shellyW);
  // A HTTP 200 only acknowledges the RPC request; it is not a confirmation that the relay has actually switched.
  if (shellyCmd.lastCode == 200 && statusRead && rt.shellyOn == on) {
    shellyCmd.active = false;
    markErrorResolved(ERROR_SHELLY);
    recordShellyResult(true);
    wdNoteCommand(true);
    Serial.printf("[Shelly] RESULTAT_FINAL=SUCCES tentative=%u commande=%s confirmee\n", attempt, on ? "ON" : "OFF");
    return;
  }
  if (attempt < SHELLY_MAX_ATTEMPTS) {
    if (on && rt.thermHold) { // la protection thermique s'est declenchee entre-temps : on ne renvoie surtout pas un ordre d'allumage
      shellyCmd.active = false;
      Serial.println("[Shelly] ordre ON abandonne (protection thermique active)");
      return;
    }
    Serial.printf("[Shelly] tentative %u RESULTAT=ECHEC -> NOUVELLE_TENTATIVE\n", attempt);
    shellyCmd.attempt = attempt + 1;
    shellyCmdSend();
    return;
  }

  shellyCmd.active = false;
  String why;
  if (!shellyCmd.lastPrepared)         why = "requete non preparee";
  else if (shellyCmd.lastCode != 200)  why = shellyHttpDetail(shellyCmd.lastCode);
  else if (!shellyCmd.lastStatusRead)  why = "lecture etat KO";
  else                                 why = String("etat reel ") + (rt.shellyOn ? "ON" : "OFF");
  reportError("Shelly: cmd " + String(on ? "ON" : "OFF") + " non confirmee - " + why, ERROR_SHELLY);
  recordShellyResult(false);
  wdNoteCommand(false);
  Serial.printf("[Shelly] RESULTAT_FINAL=ECHEC commande=%s non confirmee apres %u tentatives\n", on ? "ON" : "OFF", SHELLY_MAX_ATTEMPTS);
}

static bool readShelly(const char *origin) {
  DiagPhaseScope phase("shelly_lecture");
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
  // Retry unique et immediat uniquement sur -11, et uniquement pour le poll periodique : les lectures de
  // confirmation de shellySet() sont deja couvertes par sa propre boucle de tentatives.
  const uint8_t maxTries = (strcmp(origin, "poll") == 0) ? SHELLY_READ_MAX_TRIES : 1;
  String payload;
  int code = -1;
  int retryCause = 0; // code ayant declenche le retry (-1 ou -11), pour le message de succes apres coup
  for (uint8_t tryNo = 1; tryNo <= maxTries; ++tryNo) {
    HTTPClient h;
    h.setConnectTimeout(SHELLY_CONNECT_TIMEOUT_MS);
    h.setTimeout(SHELLY_READ_TIMEOUT_MS);
    h.setReuse(false); // Connection: close -> pas de socket gardee ouverte cote Shelly
    if (!h.begin("http://" + cfg.shellyIp + "/rpc/Switch.GetStatus?id=0")) {
      Serial.printf("[Shelly] lecture origine=%s ETAPE=ECHEC_PREPARATION_HTTP\n", origin);
      logNetworkFailure("Shelly", "preparation_http", -1);
      reportError("URL Shelly invalide", ERROR_SHELLY);
      rt.shellyConnected = false;
      rt.shellyW = NAN;
      rt.shellyOn = false;
      recordShellyPollResult(origin, false);
      return false;
    }
    Serial.printf("[Shelly] lecture origine=%s ETAPE=ENVOI_HTTP essai=%u/%u\n", origin, tryNo, maxTries);
    uint32_t t0 = millis();
    code = h.GET();
    uint32_t durMs = millis() - t0;
    rt.shellyLastMs = durMs;
    if (durMs > rt.shellyMaxMs) rt.shellyMaxMs = durMs;
    Serial.printf("[Shelly] lecture origine=%s ETAPE=REPONSE_HTTP code=%d DUREE_MS=%lu essai=%u/%u rssi=%d\n",
                  origin, code, (unsigned long)durMs, tryNo, maxTries, WiFi.RSSI());
    if (code == 200) {
      payload = h.getString();
      h.end();
      if (tryNo > 1) {
        rt.shellyRetryRecovered++;
        Serial.printf("[Shelly] lecture origine=%s %s RECUPERE par retry (essai %u) total_recuperes=%lu\n",
                      origin, (retryCause == HTTPC_ERROR_CONNECTION_REFUSED ? "-1" : "-11"), tryNo, (unsigned long)rt.shellyRetryRecovered);
      }
      break;
    }
    h.end();
    bool retryable = (code == HTTPC_ERROR_READ_TIMEOUT || code == HTTPC_ERROR_CONNECTION_REFUSED);
    if (!retryable || tryNo == maxTries) break; // autres erreurs : pas de retry
    retryCause = code;
    if (code == HTTPC_ERROR_CONNECTION_REFUSED) {
      // -1 : le Shelly a refuse/ferme la connexion (sature) ; on lui laisse un court delai pour liberer
      // son slot avant de retenter, plutot qu'un retry immediat qui retomberait probablement dessus.
      Serial.printf("[Shelly] lecture origine=%s connexion refusee (-1) -> retry dans %ums\n", origin, SHELLY_RETRY_REFUSED_DELAY_MS);
      delay(SHELLY_RETRY_REFUSED_DELAY_MS);
    } else {
      Serial.printf("[Shelly] lecture origine=%s timeout (-11) -> retry immediat\n", origin);
    }
  }

  if (code != 200) {
    logNetworkFailure("Shelly", "lecture_http", code);
    reportError("Shelly: " + shellyHttpDetail(code), ERROR_SHELLY);
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false;
  }

  if (payload.length() == 0) { // 200 recu mais corps non lu (timeout pendant la lecture du corps)
    logNetworkFailure("Shelly", "corps_vide", code);
    reportError("Shelly: corps de reponse vide", ERROR_SHELLY);
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false;
  }

  JsonDocument d;
  DeserializationError err = deserializeJson(d, payload); 
  if (err || !d.is<JsonObject>()) { 
    reportError("Shelly: JSON invalide (" + String(err ? err.c_str() : "pas un objet") + ")", ERROR_SHELLY); 
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false; 
  }
  if (!d["output"].is<bool>() || !d["apower"].is<float>()) {
    reportError(String("Shelly: champ manquant (") + (!d["output"].is<bool>() ? "output" : "apower") + ")", ERROR_SHELLY);
    rt.shellyConnected = false;
    rt.shellyW = NAN;
    recordShellyPollResult(origin, false);
    return false;
  }
  rt.shellyConnected = true; 
  rt.shellyOn        = d["output"] | false; 
  rt.shellyW         = (float)(d["apower"] | 0.0);
  // Temperature interne du Shelly et erreur de surchauffe (champs facultatifs : leur absence n'est pas une erreur).
  JsonVariant tv = d["temperature"]["tC"];
  rt.shellyT = tv.is<float>() ? tv.as<float>() : NAN;
  bool ot = false;
  if (d["errors"].is<JsonArray>()) {
    for (JsonVariant ev : d["errors"].as<JsonArray>()) {
      if (ev.is<const char *>() && strcmp(ev.as<const char *>(), "overtemp") == 0) ot = true;
    }
  }
  if (ot && !rt.shellyOvertemp) Serial.println("[Shelly] SURCHAUFFE interne signalee par le Shelly (overtemp)");
  rt.shellyOvertemp = ot;
  if (ot) { time_t nowT = time(nullptr); if (nowT >= 946684800) rt.shellyOtEpoch = (uint32_t)nowT; }
  stempFeed(rt.shellyT, rt.shellyW);
  Serial.printf("[Shelly] lecture origine=%s ETAPE=ETAT_DECODE output=%s apower=%.1fW tC=%.1f\n",
                origin, rt.shellyOn ? "ON" : "OFF", rt.shellyW, isnan(rt.shellyT) ? -99.0f : rt.shellyT);
  recordShellyPollResult(origin, true);
  markErrorResolved(ERROR_SHELLY);
  return true;
}

/* ============================================================
   3. API ECOJOKO
   ============================================================ */

/* ---- Client TLS Ecojoko unique et persistant ----
   Un seul WiFiClientSecure + un seul HTTPClient pour login, gateways et realtime_conso.
   - setReuse(true) : si le serveur accepte le keep-alive, la socket TLS reste ouverte
     entre deux polls (plus de handshake ni d'allocation/liberation du contexte mbedtls a chaque appel).
   - Le HTTPClient doit etre persistant aussi : son destructeur ferme la socket.
   - Apres toute erreur on ferme explicitement la socket (jamais de socket a moitie morte). */
static WiFiClientSecure &ecoTlsClient() {
  static WiFiClientSecure client;
  static bool configured = false;
  if (!configured) {
    if (ECOJOKO_ROOT_CA[0] != '\0') {
      client.setCACert(ECOJOKO_ROOT_CA);
    } else {
      client.setInsecure();
      Serial.println("[Securite] ECOJOKO_ROOT_CA non renseigne : connexion Ecojoko SANS verification de certificat (setInsecure)");
    }
    client.setHandshakeTimeout(ECO_TLS_HANDSHAKE_TIMEOUT_S); // en secondes
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

// Ferme la socket TLS (Wi-Fi perdu, erreur, reponse invalide...). Le prochain appel refait un handshake propre.
static void ecoTlsReset() {
  ecoTlsClient().stop();
}

// Fin de requete. keepAlive=true uniquement apres un succes complet : la socket reste alors ouverte
// (si le serveur l'autorise). Sinon on la ferme, car une reponse partiellement lue la rendrait inutilisable.
static void ecoTlsEnd(bool keepAlive) {
  ecoHttpClient().end();
  if (!keepAlive) ecoTlsReset();
}

// Envoie une requete sur le client persistant et retourne le code HTTP (< 0 = erreur de transport,
// ECO_BEGIN_FAILED = begin() refuse). postBody != nullptr => POST (login), sinon GET avec cookie de session.
// Le corps de la reponse est lu par l'appelant, qui doit toujours terminer par ecoTlsEnd().
// Si une socket REUTILISEE etait morte (connexion inactive fermee par le serveur ou le Wi-Fi), l'echec est
// immediat : on retente UNE fois tout de suite sur une connexion neuve (pas de timeout de 8 s a attendre).
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
      Serial.printf("[Ecojoko] socket TLS reutilisee morte (code=%d), nouvelle connexion\n", code);
      ecoTlsEnd(false);
      continue;
    }
    Serial.printf("[Ecojoko] tls reutilise=%s heapLibre=%u plusGrosBloc=%u heapMin=%u\n",
                  reused && attempt == 0 ? "oui" : "non", (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getMaxAllocHeap(), (unsigned)ESP.getMinFreeHeap());
    return code;
  }
  return -1;
}

bool ecoLogin() {
  DiagPhaseScope phase("eco_login");
  Serial.println("[DEBUG] Début de ecoLogin()...");
  rt.ecoCookie = ""; // Never keep a potentially expired cookie after a failed login.
  if (cfg.ecoEmail.length() == 0 || cfg.ecoPassword.length() == 0) {
    Serial.println("[Ecojoko] Identifiants manquants dans la config.");
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
    Serial.println("[Ecojoko] Impossible de se connecter à l'API Login.");
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
    Serial.printf("[Ecojoko] Erreur d'authentification, code HTTP: %d\n", httpResponseCode);
    logNetworkFailure("Ecojoko", "login_http", httpResponseCode);
  }
  // Corps de la reponse de login non lu : on ferme la socket plutot que de risquer des octets residuels
  // sur la connexion persistante (le login est rare, un handshake de plus ne coute rien).
  ecoTlsEnd(false);
  // The session cookie is a credential: only log whether it was received.
  Serial.printf("[Ecojoko] Cookie de session: %s (%u caracteres)\n",
                rt.ecoCookie.length() ? "recu" : "absent", rt.ecoCookie.length());
  return (rt.ecoCookie.length() > 0);
}

static void saveEcoGatewayCache() {
  JsonDocument doc;
  doc["gatewayId"] = rt.gatewayId;
  doc["powerMeterId"] = rt.powerMeterId;

  File out = LittleFS.open("/eco_gateway.tmp", "w");
  if (!out) {
    reportError("Impossible d'ecrire /eco_gateway.json", ERROR_STORAGE);
    return;
  }

  size_t written = serializeJson(doc, out);
  out.close();
  if (!written || !LittleFS.rename("/eco_gateway.tmp", "/eco_gateway.json")) {
    LittleFS.remove("/eco_gateway.tmp");
    reportError("Echec d'ecriture /eco_gateway.json", ERROR_STORAGE);
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
    reportError("Lecture invalide de /eco_gateway.json", ERROR_STORAGE);
    return;
  }

  String gatewayId = doc["gatewayId"].as<String>();
  String powerMeterId = doc["powerMeterId"].as<String>();
  if (!gatewayId.length() || !powerMeterId.length()) return;

  rt.gatewayId = gatewayId;
  rt.powerMeterId = powerMeterId;
  Serial.println("[Ecojoko] Identifiants gateway restaure depuis le cache");
}

static bool ecoDiscover() {
  DiagPhaseScope phase("eco_gateways");
  rt.gatewayId = "";
  rt.powerMeterId = "";
  if (!rt.ecoCookie.length()) { 
    reportError("Pas de cookie Ecojoko pour la découverte", ERROR_ECOJOKO);
    return false; 
  }

  HTTPClient &https = ecoHttpClient();
  int httpResponseCode = ecoRequest("https://service.ecojoko.com/gateways", nullptr);
  if (httpResponseCode == ECO_BEGIN_FAILED) { 
    reportError("Impossible de joindre l'API gateways", ERROR_ECOJOKO);
    logNetworkFailure("Ecojoko", "gateways_preparation_http", -1);
    return false; 
  }
  Serial.printf("[Ecojoko] Gateways HTTP: %d\n", httpResponseCode);
  if (httpResponseCode == 200) { 
    String payload = https.getString();
    Serial.println("[Ecojoko] REPONSE /gateways:");
    Serial.println(payload);
    JsonDocument jsonDoc; 
    DeserializationError error = deserializeJson(jsonDoc, payload);

    if (!error && jsonDoc.is<JsonObject>() && jsonDoc["gateways"].is<JsonArray>() && jsonDoc["gateways"].size() > 0) { 
      Serial.printf("[Ecojoko] gateways trouve: %u\n", jsonDoc["gateways"].size());
      JsonObject gateway = jsonDoc["gateways"][0];
      JsonVariant gatewayId = gateway["gateway_id"];
      bool gatewayIdIsString = gatewayId.is<const char*>();
      bool gatewayIdIsNumber = gatewayId.is<uint32_t>();
      if (gateway.isNull() || (!gatewayIdIsString && !gatewayIdIsNumber) || !gateway["devices"].is<JsonArray>()) {
        reportError("Gateway Ecojoko incomplet", ERROR_ECOJOKO);
        Serial.println("[Ecojoko] gateway_id ou devices absent / type invalide");
      } else {
        // Ecojoko returns gateway_id as a JSON number (e.g. 141907), while
        // readEcojoko() needs it as text to build the request URL.
        rt.gatewayId = gatewayIdIsNumber ? String(gatewayId.as<uint32_t>()) : gatewayId.as<String>();
        Serial.printf("[Ecojoko] gateway_id: %s\n", rt.gatewayId.c_str());

        bool foundDevice = false;
        JsonArray devices = gateway["devices"].as<JsonArray>();
        Serial.printf("[Ecojoko] appareils dans gateway: %u\n", devices.size());
        for (JsonObject device : devices) { 
          const char* type = device["device_type"] | "<absent>";
          JsonVariant deviceId = device["device_id"];
          bool deviceIdIsString = deviceId.is<const char*>();
          bool deviceIdIsNumber = deviceId.is<uint32_t>();
          String id = (deviceIdIsString || deviceIdIsNumber) ? deviceId.as<String>() : "<absent>";
          Serial.printf("[Ecojoko] appareil: type=%s id=%s\n", type, id.c_str());
          if (!device.isNull() && device["device_type"].is<const char*>() &&
              (deviceIdIsString || deviceIdIsNumber) &&
              device["device_type"].as<String>() == "POWER_METER") {
            rt.powerMeterId = deviceId.as<String>();
            foundDevice = true;
            break;
          }
        }
        if (!foundDevice || !rt.gatewayId.length() || !rt.powerMeterId.length()) { 
           reportError("Compteur Ecojoko introuvable dans le gateway", ERROR_ECOJOKO);
           Serial.println("[Ecojoko] Aucun POWER_METER utilisable");
        } else {
           Serial.printf("[Ecojoko] power_meter_id: %s\n", rt.powerMeterId.c_str());
        }
      }
    } else {
       reportError("Réponse JSON gateways invalide ou vide", ERROR_ECOJOKO);
       Serial.printf("[Ecojoko] Parse gateways: %s; racine objet=%s; gateways tableau=%s; taille=%u\n",
                     error.c_str(), jsonDoc.is<JsonObject>() ? "oui" : "non",
                     jsonDoc["gateways"].is<JsonArray>() ? "oui" : "non",
                     jsonDoc["gateways"].is<JsonArray>() ? jsonDoc["gateways"].size() : 0);
    }
  } else if (httpResponseCode == 401 || httpResponseCode == 403) { 
     rt.ecoCookie = ""; 
     reportError("Session expirée ou refusée (Ecojoko)", ERROR_ECOJOKO);
  } else {
     reportError("Erreur API gateways : HTTP " + String(httpResponseCode), ERROR_ECOJOKO);
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
  // Login / decouverte : on rend la main (pas de valeur ce tick) mais seul un VRAI echec compte dans ecoFailCount,
  // sinon chaque reauthentification reussie gonflait le compteur (backoff, AP de diagnostic, redemarrage auto).
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
    reportError("Impossible de joindre l'API Ecojoko", ERROR_ECOJOKO);
    logNetworkFailure("Ecojoko", "realtime_preparation_http", -1);
    recordEcoResult(false);
    return false;
  }

  if (httpResponseCode == 401 || httpResponseCode == 403) {
     rt.ecoCookie = ""; rt.gatewayId = ""; rt.powerMeterId = "";
     reportError("Session expiree ou refusee sur realtime_conso (Ecojoko)", ERROR_ECOJOKO);
     Serial.printf("[Ecojoko] echec realtime_conso: session refusee (HTTP %d), reset cookie/gateway\n", httpResponseCode);
     ecoTlsEnd(false); recordEcoResult(false); return false;
  }

  if (httpResponseCode == 200) { 
    String payload = https.getString();
    JsonDocument jsonDoc; 
    DeserializationError error = deserializeJson(jsonDoc, payload);

    if (!error && jsonDoc.is<JsonObject>() && jsonDoc["real_time"].is<JsonObject>()) { 
      JsonObject rt_data = jsonDoc["real_time"];
      if (!rt_data["value"].is<float>()) {
        reportError("Réponse Ecojoko incomplète", ERROR_ECOJOKO);
      } else {
        watts = rt_data["value"].as<float>();
      }
      if (isnan(watts)) { ecoTlsEnd(false); recordEcoResult(false); return false; } 
    } else {
       reportError("JSON Ecojoko corrompu", ERROR_ECOJOKO);
    }
  } else {
     // Cle de deduplication STABLE (sans dtReq, qui varie a chaque tentative) : une panne prolongee se
     // journalise comme UN episode avec un compteur d'occurrences, pas une ligne par tentative.
     reportError("eco_realtime_http_" + String(httpResponseCode),
                  "Erreur lecture Ecojoko (HTTP: " + String(httpResponseCode) + ", " + String(dtReq) + " ms)", ERROR_ECOJOKO);
     logNetworkFailure("Ecojoko", "realtime_http", httpResponseCode);
     // On ne touche PAS au cookie/gatewayId ici : une erreur réseau ou un
     // timeout transitoire n'est pas une preuve que la session a expiré.
     // Seul un vrai 401/403 (géré plus haut) doit déclencher un relogin.
  }
  bool success = !isnan(watts);
  ecoTlsEnd(success);
  recordEcoResult(success);
  if (success) markErrorResolved(ERROR_ECOJOKO);
  return success;
}

/* ============================================================
   4. ÉCHANTILLONNAGE & CALCULS
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
        // Coupure brève : on garde la derniere puissance Shelly connue, pas de reset.
        rt.shellyW = prevShellyW;
        Serial.printf("[Shelly] donnee conservee (grace) age=%lums limite=%lums\n",
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
    // Coupure brève : on garde la dernière valeur lissée connue, pas de reset.
    Serial.printf("[Ecojoko] donnee conservee (grace) age=%lums limite=%lums\n",
                  (unsigned long)(now - rt.lastEcoSuccessMs), (unsigned long)graceMs);
  } else {
    // Coupure prolongée (ou grâce désactivée) : la donnée n'est plus fiable.
    rt.ecoRaw = NAN;
    rt.ecoSmooth = NAN;
  }

  // Lissage de la puissance Shelly avec le meme filtre que ecoSmooth : sans ca, le Shelly reagit
  // instantanement (ex. thermostat qui module en fin de chauffe) alors que ecoSmooth met plusieurs
  // minutes a suivre, ce qui fausse restHouse pendant la transition et provoque des coupures/relances
  // parasites du relais (tier1 bascule a tort) alors que le surplus solaire reste suffisant.
  float shSample = (cfg.mode != "monitor" && rt.shellyOn && !isnan(rt.shellyW)) ? rt.shellyW : 0.0f;
  if (!isnan(rt.ecoSmooth)) {
    float aSh = 1.0f / (cfg.smoothingMin * 12.0f);
    rt.shellySmooth = isnan(rt.shellySmooth) ? shSample : rt.shellySmooth + aSh * (shSample - rt.shellySmooth);
  } else {
    rt.shellySmooth = NAN; // pas de mesure Ecojoko valide : on resynchronise au prochain echantillon
  }
  float shellyP = isnan(rt.shellySmooth) ? 0.0f : rt.shellySmooth;
  rt.restHouse = isnan(rt.ecoSmooth) ? NAN : rt.ecoSmooth - shellyP;
}

/* ============================================================
   5. ARCHIVAGE & HISTORIQUE (JsonDocument v7)
   ============================================================ */
/* ---------- Etat / garde-fou de la memoire de stockage (LittleFS) ---------- */
static uint32_t storagePurgeCount = 0, storageLastPurgeEpoch = 0, storageLastPurgeTryMs = 0;
static char     storageLastPurgeWhat[48] = {0};
static void diagAppendLog(const String &line); // definie plus bas (section 7b)

// Seuil effectif : 32 Ko, ramene a 1/8 de la partition si celle-ci est petite.
static size_t storageMinFree() {
  size_t eighth = LittleFS.totalBytes() / 8;
  return eighth < STORAGE_MIN_FREE_BYTES ? eighth : STORAGE_MIN_FREE_BYTES;
}

static size_t fsFreeBytes() {
  size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
  return total > used ? total - used : 0;
}

// Ecriture JSON atomique (tmp + rename). Si le tmp ne tient pas (memoire quasi pleine),
// repli sur une ecriture directe : le document est deja entier en RAM.
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

// Supprime les n plus anciennes entrees de doc[key] pour n'en garder que keepNewest.
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

/* ---------- Recap annuel : somme des mois de /monthly.json (aucun fichier dedie) ---------- */
static const char *const TOTAL_KEYS_SEC[] = {"heatedSec", "relayOnSec", "tier1Sec", "tier2Sec", "nightSec"};
static const char *const TOTAL_KEYS_F[]   = {"energyKWh", "nativeEUR", "paidKWh", "paidEUR", "savedKWh", "savedEUR"};

// dst += src, champ par champ (champ absent = 0).
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

// Garde-fou : si la memoire libre passe sous STORAGE_MIN_FREE_BYTES, purge de l'historique
// par etapes (jours detailles, puis mois les plus anciens), en s'arretant des que l'espace est suffisant.
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
  // 0. journal de chauffe : simple outil de diagnostic, le premier a sacrifier
  if (!enough() && LittleFS.exists("/heat.json") && LittleFS.remove("/heat.json")) { heatNSeg = 0; relayNIv = 0; mark("journal"); }
  // 1. jours detailles : les mois les resument deja
  if (!enough() && trimJsonArrayFile("/history.json", "days", PURGE_KEEP_DAYS)) mark("jours");
  // 2. mois les plus anciens (le recap annuel s'en deduit : les annees les plus anciennes se raccourcissent)
  if (!enough() && trimJsonArrayFile("/monthly.json", "months", PURGE_KEEP_MONTHS_1)) mark("mois<=24");
  if (!enough() && trimJsonArrayFile("/monthly.json", "months", PURGE_KEEP_MONTHS_2)) mark("mois<=3");

  size_t freeAfter = fsFreeBytes();
  if (done.length()) {
    storagePurgeCount++;
    time_t ep = time(nullptr);
    storageLastPurgeEpoch = (ep < 946684800) ? 0 : (uint32_t)ep;
    strncpy(storageLastPurgeWhat, done.c_str(), sizeof(storageLastPurgeWhat) - 1);
    storageLastPurgeWhat[sizeof(storageLastPurgeWhat) - 1] = 0;
    pushErrorLog("Memoire basse: historique purge (" + done + ")", ERROR_STORAGE);
    diagAppendLog("purge historique (" + done + ") libre " + String((unsigned long)freeBefore) + "->" +
                  String((unsigned long)freeAfter) + " octets");
  } else {
    pushErrorLog("Memoire basse: rien a purger dans l'historique", ERROR_STORAGE);
  }
  Serial.printf("[Storage] purge=%s libre %lu -> %lu octets\n", done.length() ? done.c_str() : "aucune",
                (unsigned long)freeBefore, (unsigned long)freeAfter);
}

static void fillHistoryEntry(JsonObject o) {
  o["cycleId"]    = rt.cycleId;
  o["heatedSec"]  = rt.heatedSec;
  o["relayOnSec"] = rt.relayOnSec;
  o["tier1Sec"]   = rt.tier1Sec;
  o["tier2Sec"]   = rt.tier2Sec;
  o["nightSec"]   = rt.nightSec;
  // Energie et cout reels, accumules tick par tick dans accumulateEnergyCost()
  // a partir de la puissance mesuree du ballon et du surplus solaire reel -
  // plus fiable qu'une estimation duree x puissance nominale.
  o["energyKWh"] = rt.cycleWh / 1000.0f;                // energie totale reelle du ballon ce cycle
  o["nativeEUR"] = rt.nativeEUR;                         // "nativement" : cout si tout avait ete chauffe en HC
  o["paidKWh"]   = rt.paidWh / 1000.0f;                  // energie reellement achetee au reseau
  o["paidEUR"]   = rt.paidEUR;                           // cout reel
  o["savedKWh"]  = (rt.cycleWh - rt.paidWh) / 1000.0f;   // energie couverte par le solaire (grandeur physique)
  o["savedEUR"]  = rt.nativeEUR - rt.paidEUR;            // "economise" = nativement - paye reellement
  o["mode"]     = cfg.mode;
}

static void saveHistory() {
  purgeHistoryIfLow(); // garde-fou memoire : purge l'historique avant d'ecrire si l'espace libre est bas
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
      Serial.printf("[History] JSON invalide ignore: %s\n", err.c_str());
    } else {
      Serial.println("[History] Structure invalide ignoree");
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
      reportError("Echec d'ecriture /history.json", ERROR_STORAGE);
    }
  } else {
    reportError("Impossible d'écrire /history.json", ERROR_STORAGE);
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
        if ((uint32_t)(src["monthKey"] | 0) == rt.monthKey) continue; // remplace par la version a jour ci-dessous
        // Mois deja clos : reserialise tel quel, jamais recalcule avec les
        // tarifs courants (sinon un changement de tarif fausserait
        // retroactivement les mois passes).
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
      Serial.printf("[Monthly] JSON invalide ignore: %s\n", err.c_str());
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
      reportError("Echec d'ecriture /monthly.json", ERROR_STORAGE);
    }
  } else {
    reportError("Impossible d'écrire /monthly.json", ERROR_STORAGE);
  }
}

// Called once when a cycle (day) closes, with that day's final totals, right
// before the day counters reset to 0. Rolls the running month over if needed.
static void accumulateMonth(uint32_t heatedSec, uint32_t relayOnSec, uint32_t tier1Sec,
                             uint32_t tier2Sec, uint32_t nightSec,
                             float cycleWh, float paidWh, float nativeEUR, float paidEUR) {
  uint32_t mk = monthKeyForClosingCycle();
  if (mk == 0) return; // horloge invalide : on ne peut pas dater le mois, on saute ce cumul
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
   6. AUTOMATE DE CONTRÔLE (TICK)
   ============================================================ */
// Accumule l'energie et le cout reels de ce tick a partir de la puissance
// mesuree du ballon (Shelly apower) et du surplus solaire reel (restHouse) -
// jamais a partir de la puissance nominale supposee (cfg.heaterW). Reste donc
// correct quelle que soit la puissance instantanee reellement tiree par le
// ballon (ex: 150-400W en fin de cycle de chauffe, quand le thermostat module).
// - cycleWh/monthCycleWh : energie totale reelle du ballon (peu importe la fenetre tarifaire)
// - paidWh/monthPaidWh   : part de cette energie reellement achetee au reseau (hors surplus solaire)
// - nativeEUR            : cout "nativement" - comme si tout avait ete chauffe en HC
// - paidEUR              : cout reel, au tarif de la fenetre (HC/HP) en cours a chaque instant
static void accumulateEnergyCost(uint32_t dt, bool hc) {
  lastTickWh = 0.0f; lastTickPaidWh = 0.0f;
  if (!(rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW))) return;
  float ballonW = rt.shellyW;
  float freeW = 0.0f; // part du ballon couverte par un surplus solaire reel a cet instant
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

/* ---------- Journal de chauffe (mode full) ----------
   Un segment = chauffe continue avec UNE raison, ecrit quand elle s'arrete (> 15 min) ou change :
   HC (heures creuses), SOLAR (soleil rentable : tier1), FORCE (rattrapage du plan, donc chauffe qui n'aurait pas eu lieu sans lui).
   Gardes en RAM (HEAT_SEG_MAX) et recopies dans /heat.json (au plus toutes les 10 min, et a chaque fin de cycle).
   Le segment en cours au moment d'un redemarrage est perdu, sauf checkpoint manuel juste avant (POST /api/checkpoint). Lecture : /api/heat. */
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

/* Ligne « relais » du journal : etat CONFIRME par le Shelly (rt.shellyOn vient de Switch.GetStatus), pas l'ordre envoye.
   Seules les transitions sont gardees (un intervalle = debut + duree), pas un echantillon par tick.
   Etat inconnu (Shelly injoignable) : on n'etend pas l'intervalle et on ne le ferme pas tout de suite ; au-dela de ANT_GAP_MAX_SEC il est
   ferme a la derniere mesure ON, donc une coupure Wi-Fi ne fabrique ni faux ON ni faux OFF. L'intervalle est aussi ferme au changement de cycle (comme les segments de chauffe). */
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

/* withOpen = true (checkpoint manuel, /api/checkpoint) : ajoute aussi la chauffe EN COURS comme un segment ordinaire.
   Apres un redemarrage elle est relue comme un segment termine ; la chauffe qui reprend ensuite forme un nouveau segment.
   Dans le fonctionnement normal (withOpen = false) seuls les segments termines sont ecrits. */
static bool heatSaveImpl(bool withOpen) {
  heatDirty = false; heatLastSaveMs = millis();
  purgeHistoryIfLow();
  JsonDocument d;
  JsonArray a = d["segs"].to<JsonArray>();
  bool addOpen = withOpen && heatCurOpen && heatCurWh >= 1.0f;
  // heatLoad() s'arrete a HEAT_SEG_MAX segments : si le journal est plein, on laisse de cote le plus ancien pour ne pas perdre le plus recent.
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
  // Ligne « relais » : [debut epoch, duree s]. Avec withOpen, l'intervalle en cours est ecrit comme un intervalle termine.
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
  if (!ok) reportError("Echec d'ecriture /heat.json", ERROR_STORAGE);
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
      if (t0 < 946684800UL || sec == 0UL || sec > 604800UL) continue; // 7 j : garde-fou anti-corruption, pas une limite physique
      relayIvs[relayNIv].t0 = t0; relayIvs[relayNIv].sec = sec; relayNIv++;
    }
  }
}

// Appele a chaque tick du mode full, APRES accumulateEnergyCost() (dont il reprend l'energie du tick).
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
   6a. PREVISION (mode full) : besoin du jour appris + meteo + planification
   ============================================================
   Principe (page de suivi : /ant, details JSON : /api/ant) :
   - Le ballon est vu comme une batterie. Quand le thermostat coupe (relais ON mais puissance < thermostatW
     apres une chauffe), le ballon est PLEIN : c'est la seule mesure d'etat disponible sans sonde.
     L'energie injectee entre deux "plein" est exactement ce qui a ete consomme (soutirages + pertes) :
     c'est le besoin du jour, appris sans capteur supplementaire.
   - Le soleil "utile" des prochaines heures est estime a partir du rayonnement prevu (Open-Meteo) et d'un
     coefficient k appris (surplus reellement disponible pour le ballon / rayonnement mesure).
   - La temperature exterieure prevue corrige le besoin (pertes du ballon dans le garage) via une pente apprise.
   - Nuit (HC) : on ne chauffe que l'energie que le soleil du lendemain ne pourra pas apporter.
   - Jour : controle avant la soiree (heure reglable). A cette heure le ballon n'a PAS besoin d'etre plein : il doit
     garder la reserve plus de quoi tenir la soiree jusqu'aux HC. Sinon on chauffe juste le manque, au dernier moment (en HP).
   - Toute donnee manquante ou douteuse => comportement du mode full classique (HC complete). */

static bool antClockOk() { return time(nullptr) >= 946684800; }

// Capacite utile du ballon (Wh) : energie pour passer de "vide d'eau chaude" a "plein" (thermostat coupe).
// Saisie directe (cfg.tankKWh, par ex. litres x 1,163 x (Tmax - T eau froide) d'apres la fiche) ou, a defaut, estimee depuis le volume.
static float antCapWh() {
  return cfg.tankKWh > 0.0f ? cfg.tankKWh * 1000.0f : (float)cfg.tankLiters * ANT_TANK_WH_PER_L;
}

static void antSetReason(const char *r) {
  strncpy(ant.reason, r, sizeof(ant.reason) - 1);
  ant.reason[sizeof(ant.reason) - 1] = 0;
}

/* ---------- Acces a la meteo en RAM ---------- */
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

/* ---------- Soleil utile ----------
   Une heure est "utile" quand le surplus prevu rend la chauffe NETTEMENT moins chere que les HC :
   cout reel par kWh = (heaterW - surplus) / heaterW * hp, retenu si <= ANT_BENEFIT * hc.
   (Plus strict que tier1 : eviter les HC pour un soleil a peine rentable, puis devoir rattraper en HP, coute plus.) */
static float antThresholdW() {
  float r = (cfg.hp > 0.0f) ? 1.0f - ANT_BENEFIT * cfg.hc / cfg.hp : 0.5f;
  r = fminf(0.95f, fmaxf(0.05f, r));
  return cfg.heaterW * r;
}
// Facteur d'ombrage applique a la DECISION pour le creneau commencant a l'epoch e : 1.0 sauf si active ET appris pour cette heure.
static float antShadeApply(uint32_t e) {
  if (!cfg.antShade) return 1.0f;
  uint8_t h = (uint8_t)((e / 3600UL) % 24UL);
  return antShadeOk[h] ? antShadeF[h] : 1.0f;
}
static bool antHourUseful(int i) {
  return ant.kValid && (ant.k * antShadeApply(ant.wxStart + (uint32_t)i * 3600UL) * (float)ant.wxRad[i]) >= antThresholdW();
}
// Secondes de soleil utile dans [a, b[.
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
// Premier instant de soleil utile dans [a, limit[ (0 si aucun).
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

/* ---------- Apprentissage : besoin du jour, pente temperature, coefficient solaire ---------- */
static float antMedian(float *v, uint8_t n) {
  for (uint8_t i = 1; i < n; ++i) {
    float x = v[i];
    int j = (int)i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; --j; }
    v[j + 1] = x;
  }
  return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

// Besoin (Wh / 24 h) d'un cycle, ou < 0 s'il n'est pas exploitable (pas assez de duree mesuree).
static float antRecNeed(const AntRec &r) {
  return (r.sec >= ANT_MIN_COVER_SEC && r.wh > 0.0f) ? r.wh * 86400.0f / (float)r.sec : -1.0f;
}

static float antMean(const float *v, uint8_t n) {
  float s = 0.0f;
  for (uint8_t i = 0; i < n; ++i) s += v[i];
  return s / (float)n;
}

// Besoin de base (Wh / 24 h). Met dans *tBaseOut (peut etre nullptr) la temperature moyenne des cycles qui ont servi
// a calculer ce besoin (meme melange 14 derniers cycles / meme jour de semaine), ou NAN si trop peu de
// temperatures connues : la correction de temperature s'applique alors par rapport a CETTE reference.
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
    base = 0.5f * base + 0.5f * antMedian(same, ns); // jours de semaine / week-end
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
    // Apprentissage suffisant : coefficient appris seul (comportement d'origine).
    ant.kValid = true; ant.k = ant.kLearn; ant.kW = 1.0f;
  } else if (cfg.pvKwc > 0.0f) {
    // Coefficient de depart, remplace progressivement par l'appris : le poids de l'appris suit le
    // rayonnement cumule mesure (jusqu'a ANT_MIN_K_RAD) et le nombre de jours (jusqu'a 3). Continu a la bascule.
    float w = fminf(1.0f, sumRad / ANT_MIN_K_RAD) * fminf(1.0f, (float)nk / 3.0f);
    ant.kValid = true; ant.kW = w;
    ant.k = (1.0f - w) * (cfg.pvKwc * ANT_PV_RATIO) + w * ant.kLearn;
  } else {
    ant.kValid = false; ant.k = 0.0f; ant.kW = 0.0f; // pas de coefficient de depart : comportement d'origine
  }
}

static float antErrRatio() { return ant.errPred > 100.0f ? ant.errAct / ant.errPred : 1.0f; }

// Besoin attendu sur les 24 h a venir (Wh), corrige par la temperature prevue.
static float antNeedDayAt(uint32_t ne) {
  time_t t = (time_t)(ne + 12UL * 3600UL); // jour de semaine du milieu de la fenetre
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

/* ---------- Ombrage par heure de la journee (appris depuis le prevu / reel horaire) ----------
   Une ombre d'arbre revient a la meme position du soleil, pas un nuage. Pour chaque heure de la journee, rapport = reel / prevu
   sur les ANT_SHADE_N derniers jours utilisables : la mediane efface les nuages. Facteur = mediane de l'heure / reference
   (mediane des medianes horaires), pour ne pas compter deux fois le niveau global (deja porte par k) ; borne a [0.3, 1.2].
   Les echantillons sont pris contre le prevu BRUT (antHr.p), jamais contre le prevu ombre : sinon activer le facteur
   se nourrirait de son propre apprentissage. Appris et affiche en permanence ; utilise par la decision seulement si cfg.antShade.
   Fichier /ant_shade.json. */
static void antShadeRecompute() {
  float med[24], tmp[ANT_SHADE_N], lvl[24];
  uint8_t nv = 0;
  for (uint8_t h = 0; h < 24; ++h) {
    antShadeOk[h] = false;
    if (antShadeCnt[h] < ANT_SHADE_MIN_N) continue;
    for (uint8_t j = 0; j < antShadeCnt[h]; ++j) tmp[j] = (float)antShadeS[h][j];
    med[h] = antMedian(tmp, antShadeCnt[h]) / 100.0f;
    antShadeOk[h] = true; // provisoire : confirme plus bas
    lvl[nv++] = med[h];
  }
  antShadeRef = NAN;
  float ref = nv >= ANT_SHADE_MIN_H ? antMedian(lvl, nv) : 0.0f;
  if (!(ref > 0.05f)) { for (uint8_t h = 0; h < 24; ++h) antShadeOk[h] = false; return; }
  antShadeRef = ref;
  for (uint8_t h = 0; h < 24; ++h)
    if (antShadeOk[h]) antShadeF[h] = fminf(ANT_SHADE_F_MAX, fmaxf(ANT_SHADE_F_MIN, med[h] / ref));
}

// Appelee a la fermeture d'une heure : r = reel (W), p = prevu brut fige au debut du cycle (W).
static void antShadeAdd(uint32_t hh, uint16_t r, uint16_t p) {
  if (r == 0xFFFF || p == 0xFFFF || !((float)p >= ANT_SHADE_MIN_P * cfg.heaterW)) return;
  uint8_t h = (uint8_t)(hh % 24UL);
  float q = 100.0f * (float)r / (float)p;
  antShadeS[h][antShadeHead[h]] = (uint8_t)(q > 250.0f ? 250 : (int)(q + 0.5f));
  antShadeHead[h] = (uint8_t)((antShadeHead[h] + 1) % ANT_SHADE_N);
  if (antShadeCnt[h] < ANT_SHADE_N) antShadeCnt[h]++;
  antShadeRecompute();
}

// Fichier : {"s":[[rapports de l'heure UTC 0, du plus ancien au plus recent],[heure 1],...]}. Ecrit avec l'anneau horaire (changement de cycle, checkpoint manuel).
static void antShadeSave() {
  JsonDocument d;
  JsonArray a = d["s"].to<JsonArray>();
  for (uint8_t h = 0; h < 24; ++h) {
    JsonArray x = a.add<JsonArray>();
    for (uint8_t k = 0; k < antShadeCnt[h]; ++k)
      x.add(antShadeS[h][antShadeCnt[h] < ANT_SHADE_N ? k : (antShadeHead[h] + k) % ANT_SHADE_N]);
  }
  if (!writeJsonAtomic("/ant_shade.json", d)) reportError("Echec ecriture /ant_shade.json", ERROR_STORAGE);
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

/* ---------- Solaire prevu / reel PAR HEURE (affichage uniquement : aucun effet sur la decision) ----------
   Anneau de ANT_HR_N creneaux. Reel : surplus utilisable moyen (min(heaterW, -restHouse)), meme definition que "pot".
   Prevu : min(heaterW, k x rayonnement) calcule dans antCapturePrediction (prevision figee du debut de cycle).
   Persiste dans /ant_hr.json une fois par jour (changement de cycle) et au checkpoint manuel : pas d'usure flash en plus. */
static uint16_t antHrIdx(uint32_t hh) {
  uint16_t i = (uint16_t)(hh % ANT_HR_N);
  if (antHr[i].h != hh) { antHr[i].h = hh; antHr[i].r = 0xFFFF; antHr[i].p = 0xFFFF; } // creneau d'une heure plus ancienne : recycle
  return i;
}

static uint16_t antHrW(float w) { return w <= 0.0f ? 0 : (w >= 65000.0f ? 65000 : (uint16_t)(w + 0.5f)); }

// Ferme l'heure en cours : ecrit le reel moyen si l'heure a ete assez mesuree.
static void antHrClose() {
  if (antHrCur != 0 && antHrSec >= ANT_HR_MIN_SEC) {
    AntHr &x = antHr[antHrIdx(antHrCur)];
    x.r = antHrW(antHrWh * 3600.0f / (float)antHrSec);
    antShadeAdd(antHrCur, x.r, x.p); // statistique d'ombrage (contre le prevu brut)
  }
  antHrWh = 0.0f; antHrSec = 0;
}

// Appele a chaque tick avec le surplus utilisable instantane (W) ; hh = heure epoch courante.
static void antHrAccumulate(uint32_t hh, float usableW, uint32_t dt) {
  if (hh != antHrCur) { antHrClose(); antHrCur = hh; }
  antHrWh += usableW * (float)dt / 3600.0f;
  antHrSec += dt;
}

// doc["hr"] = {h:[heures epoch], r:[reel W], p:[prevu W]} (-1 = inconnu) : meme format pour le fichier, l'export brut et l'import.
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
  if (!writeJsonAtomic("/ant_hr.json", d)) reportError("Echec d'ecriture /ant_hr.json", ERROR_STORAGE);
}

// Fusion sans ecrasement (lecture du fichier au demarrage, import des habitudes) : un creneau plus recent est garde,
// une valeur deja connue n'est pas remplacee. Retourne le nombre de creneaux touches.
static int antHrMerge(JsonVariantConst hv) {
  if (!hv["h"].is<JsonArrayConst>() || !hv["r"].is<JsonArrayConst>() || !hv["p"].is<JsonArrayConst>()) return 0;
  JsonArrayConst ha = hv["h"].as<JsonArrayConst>(), ra = hv["r"].as<JsonArrayConst>(), pa = hv["p"].as<JsonArrayConst>();
  size_t n = min(ha.size(), min(ra.size(), pa.size()));
  int touched = 0;
  for (size_t j = 0; j < n; ++j) {
    uint32_t hh = (uint32_t)(ha[j] | 0UL);
    int r = ra[j] | -1, p = pa[j] | -1;
    if (hh < 262968UL) continue; // avant 2000 : invalide
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

/* ---------- Prevision figee au debut du cycle (pour la comparer au reel) ----------
   Prise une seule fois par cycle, dans les 3 h qui suivent la fin des HC (apres un redemarrage en milieu de cycle on ne
   prend rien : le cycle serait de toute facon incomplet). Deux valeurs, chacune sur 24 h :
   - besoin attendu (Wh), a comparer au besoin mesure du cycle ;
   - surplus solaire utilisable prevu (Wh) = somme horaire de min(heaterW, k x rayonnement prevu), a comparer a pot.
   Aucun effet sur la decision : needBase / tBase (ecrits par antNeedDayAt) sont restaures. */
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
          sol += fminf(cfg.heaterW, ant.k * antShadeApply(h0 + (uint32_t)i * 3600UL) * rd); // avec ombrage si active
          antHr[antHrIdx(h0 / 3600UL + i)].p = antHrW(w); // prevu horaire, fige comme la prevision du cycle (BRUT : base de la statistique d'ombrage)
        }
      }
      ant.cycPredSolWh = sol;
    }
  }
}

/* ---------- Persistance : /ant.json (un enregistrement par cycle, ecrit 1 fois par jour) ---------- */
static void antFillRecsDoc(JsonDocument &d) {
  JsonArray a = d["recs"].to<JsonArray>();
  for (uint8_t i = 0; i < antNRec; ++i) {
    const AntRec &r = antRecs[i];
    JsonObject o = a.add<JsonObject>();
    o["c"] = r.cycle; o["w"] = r.wday; o["s"] = r.sec;
    o["wh"] = r.wh; o["r"] = r.rad; o["p"] = r.pot; o["sk"] = r.skipWh;
    if (r.t10 != INT16_MIN) o["t"] = r.t10;
    if (r.predWh >= 0.0f)    o["pn"] = r.predWh;    // prevision de besoin (Wh), absente si inconnue
    if (r.predSolWh >= 0.0f) o["ps"] = r.predSolWh; // prevision de surplus solaire (Wh), absente si inconnue
  }
}

static void antSave() {
  JsonDocument d;
  antFillRecsDoc(d);
  if (!writeJsonAtomic("/ant.json", d)) reportError("Echec d'ecriture /ant.json", ERROR_STORAGE);
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
    if (!(r.predWh >= 0.0f))    r.predWh = -1.0f;    // inclut NaN
    if (!(r.predSolWh >= 0.0f)) r.predSolWh = -1.0f;
  }
  antRefresh();
}

// Appele au changement de cycle, avant la remise a zero des compteurs du jour.
static void antCycleClose(uint32_t closingCycle) {
  if (cfg.antMode == "off") return;
  tm lt;
  uint8_t wd = 0;
  if (localNow(lt)) wd = (uint8_t)((lt.tm_wday + 6) % 7); // le cycle qui se ferme est celui d'hier
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
  antHrSave(); // prevu / reel par heure (anneau de 8 jours)
  antShadeSave(); // statistique d'ombrage (par heure de la journee)
  antRefresh();
}

/* ---------- Checkpoint MANUEL des compteurs du cycle (/ant_cyc.json) ----------
   Ecrit uniquement par le bouton « Sauvegarder maintenant » (aucun checkpoint regulier : pas d'usure flash). Relu une seule
   fois au demarrage puis SUPPRIME. Ignore s'il date d'un autre cycle. Les compteurs (mesure, rayonnement, surplus,
   temperature, prevision figee, ecart modele/reel) sont repris tels quels ; l'etat « ballon plein » (reference de mesure)
   n'est repris que si l'arret a dure au plus ANT_GAP_MAX_SEC, sinon la mesure reprend au prochain « plein ». */
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
  antHrSave(); // l'anneau horaire suit le meme rythme (bouton manuel / redemarrage controle)
  antShadeSave();
  return ok;
}

static bool antCycRestored = false; // une seule tentative par demarrage

static void antCycRestore(uint32_t ne) {
  File in = LittleFS.open("/ant_cyc.json", "r");
  if (!in) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, in);
  in.close();
  LittleFS.remove("/ant_cyc.json"); // usage unique : le fichier est nettoye dans tous les cas
  if (err) return;
  uint32_t at = (uint32_t)(d["at"] | 0UL);
  if ((uint32_t)(d["cycle"] | 0UL) != rt.cycleId || at == 0 || at > ne) { Serial.println("[Prevision] checkpoint ignore (autre cycle)"); return; }
  float mw = d["mw"] | 0.0f, rw = d["rw"] | 0.0f, pw = d["pw"] | 0.0f, sk = d["sk"] | 0.0f;
  float pn = d["pn"] | -1.0f, ps = d["ps"] | -1.0f, ea = d["ea"] | 0.0f, ep = d["ep"] | 0.0f;
  uint32_t ms = (uint32_t)(d["ms"] | 0UL), tn = (uint32_t)(d["tn"] | 0UL);
  int32_t ts = (int32_t)(d["ts"] | 0);
  bool ok = std::isfinite(mw) && mw >= 0.0f && mw <= 1.0e6f && std::isfinite(rw) && rw >= 0.0f && rw <= 50000.0f &&
            std::isfinite(pw) && pw >= 0.0f && pw <= 1.0e6f && std::isfinite(sk) && sk >= 0.0f && sk <= 1.0e6f &&
            std::isfinite(ea) && ea >= 0.0f && ea <= 1.0e9f && std::isfinite(ep) && ep >= 0.0f && ep <= 1.0e9f &&
            ms <= 200000UL && tn <= 200000UL && (int64_t)labs((long)ts) <= (int64_t)600 * (int64_t)tn + 1;
  if (!ok) { Serial.println("[Prevision] checkpoint invalide, ignore"); return; }
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
  Serial.printf("[Prevision] checkpoint repris : mesure=%lus, arret=%lus, reference \"plein\"=%s\n",
                (unsigned long)ms, (unsigned long)(ne - at), refBack ? "reprise" : "a refaire");
}

/* ---------- Detection "ballon plein" ---------- */
static void antOnFull(uint32_t ne) {
  if (ant.refValid && !ant.refBad && ne > ant.refEpoch) {
    uint32_t sec = ne - ant.refEpoch;
    if (sec >= 60UL && sec <= ANT_REF_MAX_H * 3600UL) {
      ant.cycMeasWh += ant.whSince;
      ant.cycMeasSec += sec;
      if (ant.needN >= ANT_MIN_NEED_DAYS && sec >= 900UL) {
        // Ecart modele / realite : somme(reel) / somme(prevu) sur les derniers "plein" (poids decroissant).
        float pred = (ant.needBase / 24.0f) * (float)sec / 3600.0f;
        ant.errAct  = ant.errAct  * 0.95f + ant.whSince;
        ant.errPred = ant.errPred * 0.95f + pred;
      }
    }
  }
  ant.refEpoch = ne; ant.whSince = 0.0f; ant.gapSec = 0; ant.refBad = false; ant.refValid = true;
  ant.lastFullEpoch = ne;
  Serial.printf("[Prevision] ballon plein detecte heure=%s\n", nowText().c_str());
}

/* ---------- Planificateur ---------- */
static uint32_t antSecondsUntilMinute(uint16_t target) { // prochaine occurrence de cette minute du jour (1..86400 s)
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
  if (!antClockOk())         { antSetReason("horloge"); return; }
  if (!ant.refValid)         { antSetReason("ballon: etat inconnu"); ant.forcing = false; ant.heatPlan = false; return; }
  if (!antWxFresh(ne))       { antSetReason("meteo absente"); return; }
  if (ant.needN < ANT_MIN_NEED_DAYS) { antSetReason("apprentissage besoin"); return; }
  if (!ant.kValid)           { antSetReason("apprentissage solaire"); return; }
  if (!antDeadlineOk())      { antSetReason("echeance invalide"); return; }
  float capWh = antCapWh();
  if (capWh < 500.0f)        { antSetReason("capacite ballon?"); return; }
  if (ne < ant.refEpoch)     { antSetReason("horloge"); return; }

  float needDay = antNeedDayAt(ne);
  if (needDay <= 0.0f)       { antSetReason("besoin nul"); return; }
  ant.needDayWh = needDay;
  float rate = needDay / 24.0f; // Wh / h
  float hSince = (float)(ne - ant.refEpoch) / 3600.0f;
  if (hSince > (float)ANT_REF_MAX_H) { antSetReason("dernier plein trop ancien"); return; }
  float dNow = fminf(capWh, fmaxf(0.0f, rate * hSince - ant.whSince)); // deficit estime du ballon (Wh)
  ant.dNowWh = dNow;
  float safety = ANT_SAFETY * fminf(1.5f, fmaxf(1.0f, antErrRatio()));
  float dMaxA = capWh * (100.0f - (float)cfg.reservePct) / 100.0f; // deficit max tolere au premier soleil utile
  // Deficit max tolere a l'echeance : le ballon doit encore contenir la reserve PLUS de quoi tenir la soiree
  // jusqu'au debut des HC (ou la chauffe de nuit reprend la main).
  float eveningWh = rate * safety * (float)((int)cfg.hcStart - (int)cfg.deadlineMin) / 60.0f;
  float dMaxD = fmaxf(capWh * ANT_DL_MIN_PCT / 100.0f, dMaxA - eveningWh);
  uint16_t minute = nowMin();
  bool hc = inHC(minute);

  if (hc) {
    uint32_t tdE = ne + antSecondsUntilMinute(cfg.deadlineMin);
    uint32_t tsE = antFirstUseful(ne, ne + 30UL * 3600UL);
    ant.tdEpoch = tdE; ant.tsEpoch = tsE;
    if (!antCovers(ne, tdE)) { antSetReason("meteo incomplete"); return; }
    ant.valid = true;
    if (tsE == 0 || tsE >= tdE) { // pas de soleil utile avant l'echeance : HC complete, comme le full classique
      ant.heat = true; ant.heatPlan = true;
      antSetReason("pas de soleil utile");
      return;
    }
    float dTs   = dNow + rate * safety * (float)(tsE - ne) / 3600.0f;           // deficit au premier soleil utile, sans chauffer
    float erech = ANT_SOLAR_HAIRCUT * cfg.heaterW * (float)antUsefulSec(tsE, tdE) / 3600.0f;
    float cTd   = rate * safety * (float)(tdE - tsE) / 3600.0f;
    float xMax  = fmaxf(0.0f, fminf(dMaxA, dMaxD + erech - cTd));                // deficit admissible au premier soleil utile
    float H     = fmaxf(0.0f, dTs - xMax);                                        // energie a apporter cette nuit
    ant.erechWh = erech; ant.hWh = H;
    ant.heat = ant.heatPlan ? (H > 0.0f) : (H > ANT_HYST_WH);
    ant.heatPlan = ant.heat;
    antSetReason(ant.heat ? "HC: chauffe necessaire" : "HC evitee (soleil prevu)");
  } else if (antDeadlineOk() && inTimeWindow(minute, cfg.hcEnd, cfg.deadlineMin)) {
    uint32_t tdE = ne + antSecondsUntilMinute(cfg.deadlineMin);
    ant.tdEpoch = tdE;
    if (!antCovers(ne, tdE)) { antSetReason("meteo incomplete"); return; }
    ant.valid = true;
    uint32_t usefulSec = antUsefulSec(ne, tdE);
    float cTd     = rate * safety * (float)(tdE - ne) / 3600.0f;
    float solarWh = ANT_SOLAR_HAIRCUT * cfg.heaterW * (float)usefulSec / 3600.0f;
    float F       = fmaxf(0.0f, dNow + cTd - solarWh - dMaxD);   // energie a fournir en plus du solaire attendu
    float reqSec  = F / cfg.heaterW * 3600.0f;
    float availSec = (float)(tdE - ne) - (float)usefulSec;       // temps disponible hors heures solaires utiles
    ant.hWh = F; ant.erechWh = solarWh;
    bool force = false;
    if (F > 0.0f) force = ant.forcing ? true : (reqSec + (float)ANT_FORCE_MARGIN_SEC >= availSec);
    ant.forcing = force; ant.force = force;
    antSetReason(force ? "rattrapage avant echeance" : (F > 0.0f ? "attente du soleil" : "echeance ok"));
  } else {
    ant.valid = true;
    ant.forcing = false;
    antSetReason("hors plan");
  }
}

/* ---------- Appele a chaque tick (avant les branches de mode) ---------- */
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
      // Potentiel solaire reel pour le ballon (surplus de la maison, plafonne a la puissance du ballon)
      // et rayonnement prevu sur les MEMES ticks : leur rapport donne k.
      ant.cycRadWh += radNow * (float)dt / 3600.0f;
      ant.cycPotWh += fminf(cfg.heaterW, fmaxf(0.0f, -rt.restHouse)) * (float)dt / 3600.0f;
    }
    // Reel horaire (affichage prevu vs reel) : meme surplus utilisable, sans exiger la meteo.
    if (!isnan(rt.restHouse)) antHrAccumulate(ne / 3600UL, fminf(cfg.heaterW, fmaxf(0.0f, -rt.restHouse)), dt);
  }

  bool heating = false;
  if (cfg.mode == "monitor") {
    ant.refValid = false; ant.wasHeating = false; ant.heatRun = 0; ant.idleRun = 0;
  } else {
    bool relayOK = rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW);
    if (rt.shellyOn && !relayOK) {
      // Relais ON mais pas de mesure : l'energie injectee est inconnue sur ce tick.
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

  // Mode observation : energie de HC que le plan aurait evitee (indicateur pour valider avant de passer en "on").
  if (cfg.antMode == "shadow" && ant.valid && !ant.heat && heating && inHC(nowMin()) && !isnan(rt.shellyW)) {
    ant.cycSkipWh += rt.shellyW * (float)dt / 3600.0f;
  }

  static uint32_t lastLogMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastLogMs >= 60000UL) {
    lastLogMs = nowMs;
    Serial.printf("[Prevision] mode=%s valide=%s chauffeHC=%s rattrapage=%s raison=%s deficit=%.0fWh besoin24h=%.0fWh H=%.0fWh solaire=%.0fWh k=%.3f(%u) besoinJours=%u\n",
                  cfg.antMode.c_str(), ant.valid ? "oui" : "non", ant.heat ? "oui" : "non", ant.force ? "oui" : "non",
                  ant.reason, ant.dNowWh, ant.needDayWh, ant.hWh, ant.erechWh, ant.k, (unsigned)ant.kN, (unsigned)ant.needN);
  }
}

/* ---------- Meteo (Open-Meteo, gratuit, sans cle) ---------- */
static bool antFetchWeather() {
  DiagPhaseScope phase("meteo");
  if ((size_t)ESP.getMaxAllocHeap() < ANT_WX_MIN_HEAP) {
    strncpy(ant.wxErr, "memoire faible", sizeof(ant.wxErr) - 1);
    return false;
  }
  ecoTlsReset(); // un seul contexte TLS a la fois : on libere celui d'Ecojoko (il refera un handshake)
  WiFiClientSecure client;
  if (OPENMETEO_ROOT_CA[0] != '\0') {
    client.setCACert(OPENMETEO_ROOT_CA);
  } else {
    client.setInsecure();
    Serial.println("[Securite] OPENMETEO_ROOT_CA non renseigne : connexion meteo SANS verification de certificat (setInsecure)");
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
    strncpy(ant.wxErr, "begin refuse", sizeof(ant.wxErr) - 1);
    return false;
  }
  int code = http.GET();
  if (code != 200) {
    snprintf(ant.wxErr, sizeof(ant.wxErr), "HTTP %d", code);
    http.end();
    Serial.printf("[Prevision] meteo echec http=%d\n", code);
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
    strncpy(ant.wxErr, "JSON invalide", sizeof(ant.wxErr) - 1);
    return false;
  }
  size_t n = tms.size();
  if (n < 24 || n > ANT_WX_MAX || tp.size() < n || rd.size() < n) {
    strncpy(ant.wxErr, "taille inattendue", sizeof(ant.wxErr) - 1);
    return false;
  }
  uint32_t first = tms[0].as<uint32_t>();
  if (first < 946684800UL || (uint32_t)tms[1].as<uint32_t>() - first != 3600UL) {
    strncpy(ant.wxErr, "pas horaire?", sizeof(ant.wxErr) - 1);
    return false;
  }
  /* Open-Meteo: shortwave_radiation horodate T = moyenne de [T-1h, T] (heure PRECEDENTE). Les creneaux de
     antIdx() sont [wxStart+i*3600, +3600[ : on decale donc le debut d'1 h pour que le creneau i couvre bien
     la periode moyennee. (La temperature, instantanee, se decale aussi d'1 h : negligeable.) */
  ant.wxStart = first - 3600UL;
  ant.wxN = (uint8_t)n;
  for (size_t i = 0; i < n; ++i) {
    ant.wxT10[i] = tp[i].isNull() ? INT16_MIN : (int16_t)lroundf(tp[i].as<float>() * 10.0f);
    float r = rd[i].isNull() ? 0.0f : rd[i].as<float>();
    ant.wxRad[i] = (uint16_t)(r < 0.0f ? 0.0f : (r > 1500.0f ? 1500.0f : r));
  }
  ant.wxEpoch = (uint32_t)time(nullptr);
  ant.wxErr[0] = 0;
  Serial.printf("[Prevision] meteo ok: %u h depuis %lu, heapLibre=%u\n", (unsigned)ant.wxN,
                (unsigned long)ant.wxStart, (unsigned)ESP.getFreeHeap());
  return true;
}

// Appele dans loop() quand la boucle n'est pas deja longue. Premiere recuperation ~20 s apres le demarrage.
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

/* ---------- Etat expose a l'interface ---------- */
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
  d["tRef"]          = isnan(ant.tBase) ? ant.tRef : ant.tBase; // reference reellement utilisee par la correction
  d["k"]             = ant.k;
  d["pvKwc"]         = cfg.pvKwc;
  d["kInit"]         = cfg.pvKwc * ANT_PV_RATIO; // coefficient de depart deduit des kWc
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
    // Ombrage par heure UTC de la journee : facteur (-1 = pas assez d'echantillons) et nombre d'echantillons.
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
  // Prevu / reel par heure : les 168 dernieres heures jusqu'a l'heure en cours (-1 = inconnu).
  // L'heure en cours est le reel partiel (>= 5 min de mesure).
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
    if (r.predWh >= 0.0f)    o["pred"] = r.predWh / 1000.0f;    // besoin prevu (kWh)
    if (r.predSolWh >= 0.0f) o["psol"] = r.predSolWh / 1000.0f; // surplus solaire prevu (kWh) ; le reel = pot (Wh)
  }
  String s;
  serializeJson(d, s);
  server.send(200, "application/json", s);
}

// Export brut des habitudes (re-importable via /api/import/ant).
// Journal de chauffe : segments [debut epoch, duree s, Wh injectes, Wh achetes, raison 0=HC 1=soleil 2=rattrapage].
// "open" = le dernier segment est en cours.
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
  // Ligne « relais » : [debut epoch, duree s] ; "ropen" = le dernier intervalle est en cours.
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

// Temperature interne du Shelly : {step, end, n, ot, t:[degC x10 | null], w:[W moyens]}, du plus ancien au plus recent.
// Le point i se termine a end - (n-1-i)*step. Construit a la main (pas de JsonDocument) : ~6 Ko pour 48 h.
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

// Checkpoint manuel (bouton « Sauvegarder maintenant ») : ecrit le journal de chauffe AVEC la chauffe en cours,
// et les compteurs du cycle. A utiliser juste avant de flasher une mise a jour.
void apiCheckpoint() {
  bool open = heatCurOpen && heatCurWh >= 1.0f;
  bool ok = heatSaveImpl(true);
  if (rt.cycleId != 0) saveHistory();
  bool antOk = antCycSave(); // compteurs de la prevision (mesure du jour) ; false si prevision desactivee
  Serial.printf("[Checkpoint] journal=%s segments=%u chauffe_en_cours=%s prevision=%s\n", ok ? "OK" : "ECHEC", (unsigned)heatNSeg, open ? "oui" : "non", antOk ? "OK" : "non");
  String out = String("{\"ok\":") + (ok ? "true" : "false") + ",\"open\":" + (open ? "true" : "false") +
               ",\"ant\":" + (antOk ? "true" : "false") +
               ",\"segs\":" + String((unsigned)(heatNSeg + (open ? 1 : 0))) + "}";
  server.send(ok ? 200 : 500, "application/json", out);
}

void apiAntRaw() {
  JsonDocument d;
  antFillRecsDoc(d);
  antHrFillDoc(d); // prevu / reel par heure (deja stockes : l'export les inclut)
  String s;
  serializeJson(d, s);
  server.send(200, "application/json", s);
}

/* Protection thermique du Shelly. Retourne true tant que le relais doit rester coupe.
   Declenchement : temperature interne >= shellyTMaxC (si > 0) OU alarme « overtemp » du Shelly (toujours prise en compte).
   Reprise : pause d'au moins shellyHoldMin minutes (rearmee tant qu'il fait trop chaud ou que l'alarme dure) ET Shelly joignable
   ET sous shellyTResumeC. Tant qu'elle est active, tick() force « demande = OFF » dans tous les modes : plus aucun ordre d'allumage.
   Si le Shelly ne repond plus, la pause est maintenue (on ne sait pas s'il a refroidi). Mode « monitor » : le relais n'est pas pilote. */
static bool thermalGuard() {
  bool tKnown = rt.shellyConnected && !isnan(rt.shellyT);
  // Seuil d'ALARME : une ligne dans le journal d'erreurs par episode (le buzzer, lui, est gere par le watchdog). Actif aussi en mode « monitor ».
  bool crit = cfg.shellyTAlarmC > 0.0f && tKnown && rt.shellyT >= cfg.shellyTAlarmC;
  if (crit && !rt.thermCritLogged) {
    rt.thermCritLogged = true;
    reportError(String("ALERTE surchauffe : Shelly a ") + String(rt.shellyT, 1) + " degC (seuil d'alarme " + String(cfg.shellyTAlarmC, 0) + " degC), buzzer active", ERROR_THERMAL);
  } else if (rt.thermCritLogged && (!tKnown || rt.shellyT < cfg.shellyTAlarmC - 5.0f)) {
    rt.thermCritLogged = false;
  }
  if (cfg.mode == "monitor") { rt.thermHold = false; return false; }
  bool hot = (cfg.shellyTMaxC > 0.0f && tKnown && rt.shellyT >= cfg.shellyTMaxC) || (rt.shellyConnected && rt.shellyOvertemp);
  uint32_t now = millis();
  if (!rt.thermHold) {
    if (hot) {
      rt.thermHold = true; rt.thermHoldStartMs = now; rt.thermHoldCount++;
      reportError(String("Protection thermique : Shelly a ") + (tKnown ? String(rt.shellyT, 1) + " degC" : String("? degC")) +
                  (rt.shellyOvertemp ? " + alarme overtemp du Shelly" : "") + " : relais coupe, plus d'ordre d'allumage (pause mini " +
                  String((unsigned)cfg.shellyHoldMin) + " min)", ERROR_THERMAL);
      Serial.printf("[Protection] Shelly trop chaud (tC=%.1f, alarme=%s) : relais coupe, plus d'ordre d'allumage (pause mini %u min, reprise sous %.0f degC)\n",
                    tKnown ? rt.shellyT : -99.0f, rt.shellyOvertemp ? "oui" : "non", (unsigned)cfg.shellyHoldMin, cfg.shellyTResumeC);
    }
  } else if (hot) {
    rt.thermHoldStartMs = now;   // toujours trop chaud : la pause repart de zero
  } else {
    bool minDone = (uint32_t)(now - rt.thermHoldStartMs) >= (uint32_t)cfg.shellyHoldMin * 60000UL;
    bool cooled  = rt.shellyConnected && (isnan(rt.shellyT) || rt.shellyT <= cfg.shellyTResumeC);
    if (minDone && cooled) {
      rt.thermHold = false;
      markErrorResolved(ERROR_THERMAL);
      Serial.printf("[Protection] Shelly refroidi (tC=%.1f) : reprise normale\n", isnan(rt.shellyT) ? -99.0f : rt.shellyT);
    }
  }
  return rt.thermHold;
}

void tick(uint32_t dt) {
  bool tHold = thermalGuard();
  uint32_t currentCycle = cycleForNow();
  if (currentCycle == 0) {
    // Securite thermique inconditionnelle : on tente l'arret meme si rt.shellyOn est perime (derniere lecture
    // Shelly en echec, rt.shellyConnected faux) ou deja a false, car un OFF non confirme ne cree aucun risque.
    if (tHold) shellySet(false);   // horloge invalide : aucune decision, mais on coupe quand meme si le Shelly surchauffe
    rt.state = "HORLOGE_INVALIDE";
    Serial.println("[Decision] horloge invalide: aucune decision automatique de quota");
    return;
  }
  if (currentCycle != rt.cycleId) {
    if (rt.cycleId != 0) {
      accumulateMonth(rt.heatedSec, rt.relayOnSec, rt.tier1Sec, rt.tier2Sec, rt.nightSec,
                       rt.cycleWh, rt.paidWh, rt.nativeEUR, rt.paidEUR);
      antCycleClose(rt.cycleId); // enregistre le besoin / le solaire / la temperature du cycle qui se ferme
    }
    heatSegClose(); relayIvClose(); if (heatDirty) heatSave(); // une chauffe (ou un relais allume) en cours ne traverse pas le changement de cycle
    rt.cycleId = currentCycle;
    rt.heatedSec = 0; rt.tier1Sec = 0; rt.tier2Sec = 0; rt.nightSec = 0; rt.relayOnSec = 0;
    rt.cycleWh = 0.0f; rt.paidWh = 0.0f; rt.nativeEUR = 0.0f; rt.paidEUR = 0.0f;
    restoreCurrentHistory(); // Resume after a restart in the same local quota day.
    Serial.printf("[Quota] nouveau cycle local=%lu, compteur restaure=%lus\n",
                  (unsigned long)rt.cycleId, (unsigned long)rt.heatedSec);
    saveHistory();
  }

  antTick(dt); // modele du ballon + plan ; sans effet sur la commande hors mode full + antMode "on"
  if (cfg.mode != "full") { if (heatCurOpen) heatSegClose(); relayIvClose(); } // le journal ne couvre que le mode full

  if (cfg.mode == "manual") {
    bool timerActive = cfg.manualTimer && inTimeWindow(nowMin(), cfg.manualStart, cfg.manualEnd);
    bool desired = cfg.manualTimer ? timerActive : rt.manualOn;
    if (tHold) desired = false;   // protection thermique du Shelly
    if (!rt.shellyConnected) {
      // Etat reel du relais non confirme (derniere lecture Shelly en echec) : rt.shellyOn peut etre perime,
      // on ne prend donc pas de decision dessus. Exception : la securite thermique reste inconditionnelle,
      // un OFF tente sans confirmation ne cree aucun risque.
      if (tHold) shellySet(false);
      Serial.println("[Manuel] etat Shelly non confirme : decision reportee (pas de commande ce tick)");
    } else if (rt.shellyOn != desired) {
      shellySet(desired);
    }

    bool heating = rt.shellyConnected && rt.shellyOn && !isnan(rt.shellyW) && rt.shellyW >= cfg.thermostatW;
    Serial.printf("[Quota] heure=%s dt=%lus avant=%lus apres=%lus demande=%s reel=%s chauffage=%s\n",
                  nowText().c_str(), (unsigned long)dt, (unsigned long)rt.heatedSec,
                  (unsigned long)(rt.heatedSec + (heating ? dt : 0)), desired ? "ON" : "OFF",
                  rt.shellyOn ? "ON" : "OFF", heating ? "OUI" : "NON");
    if (heating) rt.heatedSec += dt;
    if (rt.shellyConnected && rt.shellyOn) rt.relayOnSec += dt;
    accumulateEnergyCost(dt, inHC(nowMin()));
    rt.state = heating ? "MANUEL" : "MANUEL_ARRET";
    
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
    // Prevision : seulement si antMode == "on" ET plan valide. Sinon (off, shadow, donnee manquante)
    // le comportement est exactement celui du full classique : HC complete + tier1.
    bool antActive = (cfg.antMode == "on") && ant.valid;
    bool antSkipHC = antActive && hc && !ant.heat;   // HC evitee : le soleil de demain couvrira le besoin
    bool antForce  = antActive && !hc && ant.force;  // rattrapage en HP avant l'echeance
    bool desired = (hc && !antSkipHC) || tier1 || antForce;
    if (tHold) desired = false;   // protection thermique du Shelly
    Serial.printf("[Full] heure=%s HC=%s Ecojoko(t1=%s)->%s prevision(%s)=%s final=%s | reel=%s\n",
                  nowText().c_str(), hc ? "OUI" : "NON", tier1 ? "OUI" : "NON",
                  tier1 ? "ON" : "OFF", cfg.antMode.c_str(),
                  antSkipHC ? "HC_EVITEE" : (antForce ? "RATTRAPAGE" : "RAS"),
                  desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF");

    bool canTurnOn = (millis() - rt.lastStateChangeMs) >= MIN_OFF_TO_ON_MS;

    if (!rt.shellyConnected) {
      // Etat reel du relais non confirme : pas de decision basee sur rt.shellyOn (potentiellement perime).
      // La securite thermique reste inconditionnelle (OFF tente sans confirmation = sans risque).
      if (tHold) shellySet(false);
      Serial.println("[Full] etat Shelly non confirme : decision reportee (pas de commande ce tick)");
    } else if (!rt.shellyOn && desired && canTurnOn) {
      Serial.println("[Full] final=ON action=COMMANDE_SHELLY_ON raison=relais_reel_OFF");
      if (!shellySet(true)) {
        Serial.println("[Full] final=ON resultat=NON_ENVOYEE (backoff ou IP manquante); nouvel essai au prochain tick");
      }
    } else if (!rt.shellyOn && desired && !canTurnOn) {
      Serial.println("[Full] final=ON_EN_ATTENTE raison=debounce_off_to_on");
    } else if (rt.shellyOn && !desired) {
      Serial.println("[Full] final=OFF action=COMMANDE_SHELLY_OFF raison=relais_reel_ON");
      if (shellySet(false)) {
        rt.lastStateChangeMs = millis();
      }
    } else if (desired && rt.shellyOn) {
      Serial.println("[Full] final=ON action=AUCUNE_COMMANDE raison=relais_deja_confirme_ON");
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
    // Raison de la chauffe : HC d'abord ; sinon soleil rentable ; sinon rattrapage du plan
    heatLogTick(dt, heating, hc ? HR_HC : (tier1 ? HR_SOLAR : (antForce ? HR_FORCE : HR_SOLAR)));
    Serial.printf("[Full] heure=%s dt=%lus avant=%lus apres=%lus demande=%s reel=%s chauffage=%s\n",
                  nowText().c_str(), (unsigned long)dt, (unsigned long)heatedBefore,
                  (unsigned long)rt.heatedSec, desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF",
                  heating ? "OUI" : "NON");

    if (hc) rt.state = heating ? "FULL_HC_CHAUFFE" : "FULL_HC_ATTENTE";
    else if (tier1 || antForce) rt.state = heating ? "FULL_OPTIMAL_CHAUFFE" : "FULL_OPTIMAL_ATTENTE";
    else rt.state = "FULL_ARRET";

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
  if (tHold) { desired = false; rt.forcedNight = false; }   // protection thermique du Shelly (forcedNight vide : sinon l'ordre OFF serait saute)
  Serial.printf("[Decision] heure=%s HC=%s restantHC=%lus quota=%lu/%lus restant=%lus | Ecojoko(t1=%s,t2=%s)->%s quota->%s final=%s | reel=%s\n",
                nowText().c_str(), hc ? "OUI" : "NON", (unsigned long)hcRemainingSec,
                (unsigned long)rt.heatedSec, (unsigned long)quotaTargetSec, (unsigned long)quotaRemainingSec,
                tier1 ? "OUI" : "NON", tier2 ? "OUI" : "NON", ecoDecision ? "ON" : "OFF",
                quotaDecision ? "ON" : "OFF", desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF");

    bool canTurnOn = (millis() - rt.lastStateChangeMs) >= MIN_OFF_TO_ON_MS;

  if (!rt.shellyConnected) {
    // Etat reel du relais non confirme : pas de decision basee sur rt.shellyOn (potentiellement perime).
    // La securite thermique reste inconditionnelle (OFF tente sans confirmation = sans risque).
    if (tHold) shellySet(false);
    Serial.println("[Decision] etat Shelly non confirme : decision reportee (pas de commande ce tick)");
  } else if (!rt.shellyOn && desired && canTurnOn) {
    Serial.println("[Decision] final=ON action=COMMANDE_SHELLY_ON raison=relais_reel_OFF");
    if (!shellySet(true)) {
      Serial.println("[Decision] final=ON resultat=NON_ENVOYEE (backoff ou IP manquante); nouvel essai au prochain tick");
    }
  } else if (!rt.shellyOn && desired && !canTurnOn) {
    Serial.println("[Decision] final=ON_EN_ATTENTE raison=debounce_off_to_on");
  } else if (rt.shellyOn && !desired && !rt.forcedNight) {
    Serial.println("[Decision] final=OFF action=COMMANDE_SHELLY_OFF raison=relais_reel_ON");
    if (shellySet(false)) {
      rt.lastStateChangeMs = millis();
    }
  } else if (desired && rt.shellyOn) {
    Serial.println("[Decision] final=ON action=AUCUNE_COMMANDE raison=relais_deja_confirme_ON");
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
  Serial.printf("[Quota] heure=%s dt=%lus avant=%lus apres=%lus demande=%s reel=%s chauffage=%s\n",
                nowText().c_str(), (unsigned long)dt, (unsigned long)quotaBefore,
                (unsigned long)rt.heatedSec, desired ? "ON" : "OFF", rt.shellyOn ? "ON" : "OFF",
                heating ? "OUI" : "NON");

    if (rt.forcedNight) rt.state = heating ? "HC_CHAUFFE" : "HC_ATTENTE";
  else if (quota) rt.state = heating ? "SURPLUS_CHAUFFE" : "SURPLUS_ATTENTE";
  else if (!hc && !quota) rt.state = heating ? "OPTIMAL_CHAUFFE" : "OPTIMAL_ATTENTE";
  else rt.state = "ARRET";

  if (millis() - rt.lastHistorySave >= HISTORY_SAVE_PERIOD_MS) saveHistory();
}

/* ============================================================
   7. ROUTES WEB / API
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
    // Compte a rebours avant le prochain essai Ecojoko (backoff reel, resolution = periode de controle de 5 s).
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
  d["shellyOvertemp"] = rt.shellyConnected && rt.shellyOvertemp;   // pas d'alerte figee si le Shelly ne repond plus

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
    if (deserializeJson(d, server.arg("plain")) || !d.is<JsonObject>()) { server.send(400, "text/plain", "JSON invalide"); return; }
    {
      int newHb  = d.containsKey("wdHbPin")  ? d["wdHbPin"].as<int>()  : (int)cfg.wdHbPin;
      int newErr = d.containsKey("wdErrPin") ? d["wdErrPin"].as<int>() : (int)cfg.wdErrPin;
      int newMin = d.containsKey("wdProblemMin") ? d["wdProblemMin"].as<int>() : (int)cfg.wdProblemMin;
      int newTx  = d.containsKey("wdTxPin")  ? d["wdTxPin"].as<int>()  : (int)cfg.wdTxPin;
      if (!wdPinAllowed(newHb) || !wdPinAllowed(newErr) || (newHb >= 0 && newHb == newErr)) {
        server.send(400, "text/plain", "Broche invalide (ou identique pour HB et ERR)");
        return;
      }
      if (!wdPinAllowed(newTx) || (newTx >= 0 && (newTx == newHb || newTx == newErr))) {
        server.send(400, "text/plain", "Broche TX invalide (ou identique a HB ou ERR)");
        return;
      }
      if (newMin < 5 || newMin > 720) {
        server.send(400, "text/plain", "Delai avant alarme : entre 5 et 720 minutes");
        return;
      }
    }
    {
      if (d.containsKey("antMode")) {
        String m = d["antMode"].as<String>();
        if (m != "off" && m != "shadow" && m != "on") { server.send(400, "text/plain", "antMode : off, shadow ou on"); return; }
      }
      if (d.containsKey("tankLiters") && (d["tankLiters"].as<int>() < 50 || d["tankLiters"].as<int>() > 1000)) {
        server.send(400, "text/plain", "Volume du ballon : entre 50 et 1000 L"); return;
      }
      if (d.containsKey("tankKWh") && (d["tankKWh"].as<float>() < 0.0f || d["tankKWh"].as<float>() > 60.0f)) {
        server.send(400, "text/plain", "Capacite : 0 (automatique) ou jusqu'a 60 kWh"); return;
      }
      if (d.containsKey("reservePct") && (d["reservePct"].as<int>() < 0 || d["reservePct"].as<int>() > 90)) {
        server.send(400, "text/plain", "Reserve : entre 0 et 90 %"); return;
      }
      if (d.containsKey("deadlineMin") && (d["deadlineMin"].as<int>() < 0 || d["deadlineMin"].as<int>() > 1439)) {
        server.send(400, "text/plain", "Echeance : minute du jour (0-1439)"); return;
      }
      if (d.containsKey("lat") && fabsf(d["lat"].as<float>()) > 90.0f) { server.send(400, "text/plain", "Latitude invalide"); return; }
      if (d.containsKey("lon") && fabsf(d["lon"].as<float>()) > 180.0f) { server.send(400, "text/plain", "Longitude invalide"); return; }
      if (d.containsKey("pvKwc") && !(d["pvKwc"].as<float>() >= 0.0f && d["pvKwc"].as<float>() <= 50.0f)) {
        server.send(400, "text/plain", "Puissance crete : 0 (aucune) a 50 kWc"); return;
      }
      if (d.containsKey("bootApSec") && (d["bootApSec"].as<int>() < 0 || d["bootApSec"].as<int>() > 600)) {
        server.send(400, "text/plain", "Duree de l'AP de demarrage : 0 a 600 s"); return;
      }
      if (d.containsKey("shellyTMaxC") && !(d["shellyTMaxC"].as<float>() >= 0.0f && d["shellyTMaxC"].as<float>() <= 150.0f)) {
        server.send(400, "text/plain", "Seuil de coupure du Shelly : 0 (desactive) a 150 degC"); return;
      }
      if (d.containsKey("shellyTResumeC") && !(d["shellyTResumeC"].as<float>() >= 0.0f && d["shellyTResumeC"].as<float>() <= 150.0f)) {
        server.send(400, "text/plain", "Temperature de reprise du Shelly : 0 a 150 degC"); return;
      }
      if (d.containsKey("shellyTAlarmC") && !(d["shellyTAlarmC"].as<float>() >= 0.0f && d["shellyTAlarmC"].as<float>() <= 200.0f)) {
        server.send(400, "text/plain", "Seuil d'alarme du Shelly : 0 (desactive) a 200 degC"); return;
      }
      if (d.containsKey("shellyHoldMin") && (d["shellyHoldMin"].as<int>() < 1 || d["shellyHoldMin"].as<int>() > 240)) {
        server.send(400, "text/plain", "Pause minimale apres surchauffe : 1 a 240 min"); return;
      }
      if (d.containsKey("muteStart") && (d["muteStart"].as<int>() < 0 || d["muteStart"].as<int>() > 1439)) {
        server.send(400, "text/plain", "Debut de la plage silencieuse : minute du jour (0-1439)"); return;
      }
      if (d.containsKey("muteEnd") && (d["muteEnd"].as<int>() < 0 || d["muteEnd"].as<int>() > 1439)) {
        server.send(400, "text/plain", "Fin de la plage silencieuse : minute du jour (0-1439)"); return;
      }
      if (d.containsKey("snoozeMin") && (d["snoozeMin"].as<int>() < 5 || d["snoozeMin"].as<int>() > 720)) {
        server.send(400, "text/plain", "Duree du silence a distance : entre 5 et 720 minutes"); return;
      }
      {
        float nMax = d.containsKey("shellyTMaxC") ? d["shellyTMaxC"].as<float>() : cfg.shellyTMaxC;
        float nRes = d.containsKey("shellyTResumeC") ? d["shellyTResumeC"].as<float>() : cfg.shellyTResumeC;
        if (nMax > 0.0f && nRes >= nMax) { server.send(400, "text/plain", "La temperature de reprise doit etre inferieure au seuil de coupure"); return; }
      }
      bool wantStatic = d.containsKey("staticIp") ? d["staticIp"].as<bool>() : cfg.staticIp;
      if (wantStatic) {
        String nip   = d.containsKey("ip")   ? d["ip"].as<String>()   : cfg.ip;
        String ngw   = d.containsKey("gw")   ? d["gw"].as<String>()   : cfg.gw;
        String nmask = d.containsKey("mask") ? d["mask"].as<String>() : cfg.mask;
        String ndns  = d.containsKey("dns")  ? d["dns"].as<String>()  : cfg.dns;
        IPAddress a;
        if (!parseIp(nip, a) || !parseIp(ngw, a) || !parseIp(nmask, a) || (ndns.length() && !parseIp(ndns, a))) {
          server.send(400, "text/plain", "IP fixe : IP, passerelle et masque valides requis (DNS optionnel)"); return;
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
    if (d.containsKey("pvKwc")) antRefresh(); // le nouveau coefficient de depart s'applique tout de suite
    
    saveConfig();
    wdSetPins(cfg.wdHbPin, cfg.wdErrPin); // pris en compte tout de suite, sans redemarrage
    wdSetTxPin(cfg.wdTxPin);
    wdMuteChanged(); // plage ou duree modifiee : le C3 recoit tout de suite la nouvelle duree de silence
    server.send(200, "text/plain", "OK");
    bool clockChanged = cfg.autoTime != oldAutoTime ||
                        (!cfg.autoTime && cfg.manualEpoch != oldManualEpoch);
    if (cfg.wifiSsid != oldSsid || cfg.wifiPassword != oldPass || newNet != oldNet || clockChanged) {
      delay(500); ESP.restart();
    }
  }
}

// POST /api/watchdog {"cmd":"test"} : met ERR a 1 pendant 8 s pour verifier que le C3 fait bien sonner le buzzer.
// (L'acquittement se fait avec le bouton branche sur le C3 : il n'y a pas de liaison retour.)
void apiWatchdog() {
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain")) || d["cmd"].as<String>() != "test") {
    server.send(400, "text/plain", "Commande invalide");
    return;
  }
  if (!wdStartTest()) {
    server.send(400, "text/plain", "Broche d'alarme (ERR) desactivee");
    return;
  }
  server.send(200, "text/plain", "OK");
}

// POST /api/snooze {"cmd":"on"|"off"} : coupe la sonnerie du C3 pendant cfg.snoozeMin minutes, ou la reactive.
// "on" n'est accepte que pendant une alarme, jamais pour une surchauffe (voir wdSnoozeCmd).
void apiSnooze() {
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain"))) { server.send(400, "text/plain", "JSON invalide"); return; }
  String cmd = d["cmd"].as<String>();
  if (cmd != "on" && cmd != "off") { server.send(400, "text/plain", "Commande invalide"); return; }
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
  if (cfg.mode != "manual") { server.send(400, "text/plain", "Le systeme n'est pas en mode manuel"); return; }
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain")) || !d["on"].is<bool>()) { server.send(400, "text/plain", "JSON invalide"); return; }
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
          // Mois clos : reserialise tel quel, jamais recalcule avec les tarifs courants.
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
    // Premier jour du mois : aucun mois clos encore enregistre, on affiche juste aujourd'hui.
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
// Les 60 mois conserves couvrent 5 ans ; l'annee la plus ancienne peut etre partielle et est ecartee
// au-dela de YEARLY_HISTORY_MAX annees.
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

// Etat de la memoire : stockage LittleFS (fichiers), RAM, flash programme, purges de l'historique.
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
   7b. DIAGNOSTIC DE BLOCAGE
   ============================================================ */
static String diagFormatEpoch(uint32_t epoch) {
  if (epoch < 1600000000UL) return "heure inconnue";
  time_t t = (time_t)epoch;
  tm lt;
  localtime_r(&t, &lt);
  char buf[24];
  strftime(buf, sizeof(buf), "%d/%m %H:%M:%S", &lt);
  return String(buf);
}

static const char *diagResetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "mise sous tension";
    case ESP_RST_SW:        return "redemarrage logiciel";
    case ESP_RST_PANIC:     return "crash (exception)";
    case ESP_RST_INT_WDT:   return "watchdog interruption";
    case ESP_RST_TASK_WDT:  return "watchdog tache";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "chute de tension (brownout)";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_EXT:       return "reset externe";
    default:                return "inconnue";
  }
}

// Ajoute une ligne a /boot_log.txt en gardant le fichier sous DIAG_LOG_MAX_BYTES (on perd les plus anciennes lignes).
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

// Appele apres chaque server.handleClient() : garde la requete dans l'anneau RAM et trace les lenteurs dans /boot_log.txt.
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
    Serial.printf("[Web] lent: %s client=%s duree=%lums\n", diagWebUri, diagWebIp, (unsigned long)ms);
    uint32_t nowMs = millis();
    if (webLastSlowLogMs == 0 || (uint32_t)(nowMs - webLastSlowLogMs) > 30000UL) { // pas plus d'une ecriture flash / 30 s
      webLastSlowLogMs = nowMs ? nowMs : 1;
      String line = String("web_lent uri=") + diagWebUri + " client=" + diagWebIp + " duree=" + String(ms) + "ms" +
                    " heapLibre=" + String(ESP.getFreeHeap()) + " rssi=" + String(WiFi.RSSI()) +
                    " heure=" + diagFormatEpoch((uint32_t)time(nullptr));
      diagAppendLog(line);
    }
  }
}

// A appeler une fois au demarrage, apres applyClockConfig() (le fuseau horaire doit etre regle).
static void diagBootReport() {
  esp_reset_reason_t reason = esp_reset_reason();
  bool rtcValid = (rtcMagic == DIAG_RTC_MAGIC);
  String line = String("reboot cause=") + diagResetReasonText(reason);

  if (rtcValid) {
    bool stalled = (rtcStallFlag == 1);
    line += stalled ? " BLOCAGE_DETECTE=oui" : " blocage=non";
    line += String(" phase=") + rtcLastPhase;
    line += " dernier_signe_de_vie=" + diagFormatEpoch(rtcLastEpoch);
    line += " uptime_avant=" + String(rtcLastUptimeS) + "s";
    line += " heapLibre=" + String(rtcLastFreeHeap) + " heapMin=" + String(rtcLastMinHeap);
    line += " rssi=" + String((int)rtcLastRssi);
    if (rtcLastWebAgeS > 0) {
      char uri[24], ip[16];
      strncpy(uri, rtcLastWebUri, sizeof(uri) - 1); uri[sizeof(uri) - 1] = '\0';
      strncpy(ip,  rtcLastWebIp,  sizeof(ip)  - 1); ip[sizeof(ip) - 1]   = '\0';
      line += String(" web_en_cours=") + uri + " client=" + ip + " depuis=" + String(rtcLastWebAgeS) + "s";
    }
  } else {
    line += " (pas d'etat precedent : coupure d'alimentation ou premier demarrage)";
    rtcStallCount = 0;
    rtcLastWebUri[0] = '\0';
    rtcLastWebIp[0]  = '\0';
    rtcLastWebAgeS   = 0;
  }
  wdNoteBoot(reason, rtcValid, rtcValid && rtcStallFlag == 1); // avant la remise a zero du drapeau de blocage
  rtcMagic = DIAG_RTC_MAGIC;
  rtcStallFlag = 0;
  line += String(" fw=") + FW_VERSION + " build=" + FW_BUILD;
  Serial.println("[Diag] " + line);
  diagAppendLog(line);
}

static void stallMonitorTask(void *) {
  uint32_t prevLoopMs = 0;   // diagLoopMs vu au controle precedent
  uint8_t  frozenChecks = 0; // controles consecutifs ou loop() n'a pas avance du tout
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(STALL_CHECK_MS));

    // ORDRE IMPORTANT : on lit d'abord les marqueurs ecrits par loop() (sur l'autre coeur), et SEULEMENT ENSUITE millis().
    // Avant, millis() etait lu en premier : si loop() ecrivait son marqueur dans la milliseconde suivante, (nowMs - marqueur)
    // etait negatif, donc ~4,29 milliards en non signe (cf. "depuis=4294967s" dans /boot_log.txt) -> faux "blocage" -> reboot.
    uint32_t loopMs   = diagLoopMs;
    uint32_t webStart = diagWebStartMs;
    uint32_t nowMs    = millis();
    int32_t  loopAgeMs = (int32_t)(nowMs - loopMs);
    if (loopAgeMs < 0) loopAgeMs = 0;
    int32_t  webAgeMs  = webStart ? (int32_t)(nowMs - webStart) : 0;
    if (webAgeMs < 0) webAgeMs = 0;

    // Instantane conserve en memoire RTC : c'est ce qu'on lira apres un redemarrage.
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

    // Double confirmation : blocage reel = loop() n'a rien ecrit depuis STALL_RESTART_MS ET son marqueur n'a pas bouge
    // depuis le controle precedent (un loop() vivant change toujours de marqueur : jamais de faux positif).
    if (loopMs == prevLoopMs) { if (frozenChecks < 255) frozenChecks++; } else frozenChecks = 0;
    prevLoopMs = loopMs;

    if (loopAgeMs > (int32_t)STALL_RESTART_MS && frozenChecks >= 2) {
      rtcStallFlag = 1;
      rtcStallCount++;
      Serial.printf("[Diag] loop() bloque depuis %lus (phase=%s web=%s client=%s) -> redemarrage\n",
                    (unsigned long)(loopAgeMs / 1000UL), rtcLastPhase, rtcLastWebUri, rtcLastWebIp);
      delay(100);
      ESP.restart();
    }
  }
}

static void apiDiag() {
  String out = String("fw=") + FW_VERSION + " build=" + FW_BUILD + "\n";
  out += "uptime=" + String(millis() / 1000UL) + "s heapLibre=" + String(ESP.getFreeHeap()) +
               " heapMin=" + String(ESP.getMinFreeHeap()) + " rssi=" + String(WiFi.RSSI()) +
               " blocages_depuis_alim=" + String(rtcStallCount) + "\n\n";
  File f = LittleFS.open("/boot_log.txt", "r");
  if (f) { out += f.readString(); f.close(); }
  else out += "(aucun historique)\n";
  out += "\n--- dernieres requetes web (la plus recente en premier) ---\n";
  if (webRingCount == 0) out += "(aucune)\n";
  for (uint8_t i = 0; i < webRingCount; i++) {
    const WebReq &r = webRing[(webRingHead + WEB_RING - 1 - i) % WEB_RING];
    out += (r.epoch ? diagFormatEpoch(r.epoch) : String("--/-- --:--:--")) + "  " + r.uri + "  " + String(r.ms) + "ms  " + r.ip + "\n";
  }
  server.send(200, "text/plain; charset=utf-8", out);
}

/* ---------- Page /ant : suivi de la prevision + reglages (autonome, sans toucher a index.html) ---------- */
static const char ANT_PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Prévision ballon</title>
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
<h1>Prévision du ballon <small><a href="/">← accueil</a></small></h1>
<div class="card" id="st">Chargement…</div>
<h2>Soleil prévu (30 h)</h2><div class="card"><div class="bar" id="bar"></div><small>Barres jaunes = heures où le solaire rend la chauffe nettement moins chère que les heures creuses.</small></div>
<h2>Ombrage par heure de la journée</h2><div class="card"><div class="bar" id="shbar"></div><div id="shlab" style="display:flex;gap:1px;font-size:.65em;text-align:center"></div><p id="shtxt" style="margin:.6em 0 0"></p><small>Barre = facteur appris pour cette heure (pleine hauteur = ×1,2, grise = pas encore assez d'échantillons). Rapport réel / prévu, médiane sur les derniers jours : une ombre d'arbre revient chaque jour à la même heure, pas un nuage. Appris et affiché en permanence ; utilisé par la décision seulement si la case ci-dessous est cochée.</small></div>
<h2>Apprentissage</h2><div class="card" id="learn"></div>
<h2>Réglages</h2>
<div class="card">
<label>Mode</label><select id="antMode">
<option value="off">off : mode full classique, rien n'est calculé</option>
<option value="shadow">shadow : calcule et affiche, n'agit pas</option>
<option value="on">on : le plan pilote les heures creuses</option></select>
<label>Volume du ballon (litres)</label><input id="tankLiters" type="number" min="50" max="1000">
<label>Capacité utile (kWh, 0 = estimée depuis le volume)</label><input id="tankKWh" type="number" min="0" max="60" step="0.1"><small>Énergie pour chauffer le ballon de l'eau froide à la température max : litres × 1,163 × écart en °C ÷ 1000 (fiche technique).</small>
<label>Réserve minimale d'eau chaude (% de la capacité)</label><input id="reservePct" type="number" min="0" max="90"><small>Niveau à garder avant que le soleil reprenne la chauffe.</small>
<label>Contrôle avant la soirée (heure)</label><input id="deadline" type="time"><small>Le ballon n'a pas à être plein : il doit garder la réserve plus de quoi tenir la soirée jusqu'aux heures creuses. Sinon le système chauffe juste le manque, au dernier moment.</small>
<label>Latitude / longitude (météo)</label><div class="row"><input id="lat" type="number" step="0.001"><input id="lon" type="number" step="0.001"></div>
<label class="chk"><input id="antShade" type="checkbox"> Appliquer le facteur d'ombrage à la décision</label><small>Laisser décoché tant que la courbe ci-dessus n'est pas cohérente d'une heure à l'autre (il faut des jours dégagés : pas avant le printemps).</small>
<button id="save">Enregistrer</button><small id="msg"></small></div>
<h2>Derniers jours</h2><div class="card"><div style="overflow-x:auto"><table id="tb"></table></div><p id="sc" style="margin:.6em 0 0"></p><small>kWh, sauf Couv. (h), °C et Rayon. (Wh/m²). Écart = prévu − réel (+ : le modèle surestime, côté confort). Prévu = pris au début du cycle (besoin sur 24 h, surplus solaire utilisable sur 24 h) ; Sol. réel = surplus utilisable mesuré.</small></div>
<script>
const $=i=>document.getElementById(i);let first=true;
const hm=m=>String(Math.floor(m/60)).padStart(2,'0')+':'+String(m%60).padStart(2,'0');
const at=e=>e?new Date(e*1000).toLocaleString('fr-FR',{weekday:'short',hour:'2-digit',minute:'2-digit'}):'—';
const f1=v=>v===undefined||v===null?'—':v.toFixed(1);
/* Jugement de la prévision : sur les jours dont le besoin réel est exploitable ET dont la prévision a été enregistrée, erreur moyenne du modèle
   comparée à une baseline bête (moyenne des 5 jours exploitables précédents, au moins 3 disponibles). */
function score(rs){
 const rows=[];
 rs.forEach((r,i)=>{if(!(r.need>0)||r.pred===undefined)return;
  const prev=rs.slice(0,i).filter(x=>x.need>0).slice(-5);if(prev.length<3)return;
  rows.push({m:r.pred-r.need,b:prev.reduce((s,x)=>s+x.need,0)/prev.length-r.need,n:r.need,ps:r.psol,pr:r.pot/1000})});
 if(!rows.length)return'Pas encore de jour comparable : la prévision est enregistrée au début de chaque cycle, le premier écart apparaît à la fin du premier cycle exploitable suivant.';
 const avg=x=>x.reduce((s,v)=>s+v,0)/x.length,mm=avg(rows.map(x=>Math.abs(x.m))),mb=avg(rows.map(x=>Math.abs(x.b))),bias=avg(rows.map(x=>x.m));
 const pm=avg(rows.map(x=>Math.abs(x.m)/x.n))*100,pb=avg(rows.map(x=>Math.abs(x.b)/x.n))*100;
 const v=rows.length<5?'trop tôt pour conclure (5 jours mini)':mm<mb?'<span class="ok">meilleur</span> que la baseline':'<span class="w">pas encore meilleur</span> que la baseline';
 const so=rows.filter(x=>x.ps!==undefined),sol=so.length?` Solaire utilisable sur ${so.length} j : prévu ${so.reduce((s,x)=>s+x.ps,0).toFixed(1)} kWh, réel ${so.reduce((s,x)=>s+x.pr,0).toFixed(1)} kWh.`:'';
 return `${rows.length} jour(s) comparable(s) : erreur moyenne du modèle ${mm.toFixed(1)} kWh (${pm.toFixed(0)} %), biais ${bias>=0?'+':''}${bias.toFixed(1)} kWh (+ = surestime). Baseline « moyenne des 5 jours précédents » : ${mb.toFixed(1)} kWh (${pb.toFixed(0)} %) → ${v}.${sol}`}
async function load(){
 let a;try{a=await(await fetch('/api/ant')).json()}catch(e){$('st').textContent='ESP injoignable';return}
 const dec=a.antMode==='off'?'—':!a.antValid?'<span class="w">calcul impossible</span>':a.antForce?'<span class="w">rattrapage en cours</span>':a.antHeat?'chauffe en HC':'<span class="ok">HC évitée</span>';
 $('st').innerHTML=
 `<div class="row"><span>Mode</span><span class="big">${a.antMode}</span></div>
  <div class="row"><span>Décision</span><span class="big">${dec}</span></div>
  <div class="row"><span>Raison</span><span>${a.antReason||''}</span></div>
  <div class="row"><span>Déficit estimé du ballon</span><span>${(a.antDeficitKWh||0).toFixed(1)} kWh / ${a.capKWh.toFixed(1)}</span></div>
  <div class="row"><span>Besoin attendu 24 h</span><span>${(a.antNeedKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>À chauffer (plan)</span><span>${(a.antPlanKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>Solaire attendu</span><span>${(a.antSolarKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>HC qui aurait été évitée (cette période)</span><span>${(a.antShadowSkipKWh||0).toFixed(1)} kWh</span></div>
  <div class="row"><span>Dernier « ballon plein »</span><span>${at(a.lastFullEpoch)}</span></div>
  <div class="row"><span>Météo</span><span class="${a.wxOk?'ok':'ko'}">${a.wxOk?('ok, il y a '+a.wxAgeMin+' min'):('absente '+(a.wxErr||''))}${a.wxTemp!==undefined?' · '+a.wxTemp.toFixed(1)+' °C':''}</span></div>`;
 $('learn').innerHTML=
 `<div class="row"><span>Jours de besoin exploitables</span><span>${a.antNeedDays} (4 mini)</span></div>
  <div class="row"><span>Coefficient solaire k</span><span class="${a.antKValid?'ok':'w'}">${a.antKValid?a.k.toFixed(3)+' W par W/m²'+(a.kW<1?' (départ '+a.kInit.toFixed(1)+', '+Math.round(a.kW*100)+' % appris)':''):'en cours ('+a.antKDays+' j, '+Math.round(a.kSumRad)+'/1500 Wh/m²)'}</span></div>
  <div class="row"><span>Correction température</span><span>${a.slopeValid?a.slopeWhPerC.toFixed(0)+' Wh/°C/jour (réf. '+a.tRef.toFixed(1)+' °C)':'pas encore calibrée'}</span></div>
  <div class="row"><span>Écart modèle / réel</span><span>×${a.errRatio.toFixed(2)}</span></div>`;
 {const sh=a.shade||{f:[],n:[]},rows=[];
  for(let u=0;u<24;u++){const d=new Date();d.setUTCHours(u,0,0,0);rows.push({lh:d.getHours(),f:(sh.f||[])[u],n:(sh.n||[])[u]||0})}
  const day=rows.filter(x=>x.lh>=6&&x.lh<=21).sort((x,y)=>x.lh-y.lh);
  $('shbar').innerHTML=day.map(x=>`<div class="${x.f>=0?'':'n'}" style="height:${x.f>=0?Math.max(3,Math.min(60,x.f*50)):(x.n?6:3)}px" title="${x.lh} h : ${x.f>=0?'×'+x.f.toFixed(2):'pas assez d\'échantillons'} (${x.n}/${sh.minN||7})"></div>`).join('');
  $('shlab').innerHTML=day.map(x=>`<span style="flex:1">${x.lh}</span>`).join('');
  const nv=day.filter(x=>x.f>=0).length;
  $('shtxt').innerHTML=(nv?`${nv} heure(s) avec facteur`+(sh.ref!==undefined?`, niveau de référence ×${sh.ref.toFixed(2)}`:'')+'. ':'Pas encore de facteur : il faut au moins '+(sh.minN||7)+' jours utilisables par heure (prévu ≥ 35 % de la puissance du ballon). ')+`Appliqué à la décision : <b>${a.antShade?'oui':'non'}</b>.`;}
 $('bar').innerHTML=(a.wx||[]).map(h=>`<div class="${h.u?'':'n'}" style="height:${Math.max(3,Math.min(60,h.r/12))}px" title="${at(h.e)} ${h.r} W/m²"></div>`).join('');
 $('tb').innerHTML='<tr><th>Cycle</th><th>Besoin</th><th>Prévu</th><th>Écart</th><th>Couv. h</th><th>°C</th><th>Rayon.</th><th>Sol. prévu</th><th>Sol. réel</th><th>HC évitable</th></tr>'+(a.recs||[]).slice(-14).reverse().map(r=>{const ok=r.need>0,ec=ok&&r.pred!==undefined?r.pred-r.need:undefined;return `<tr><td>${r.c%1000}</td><td>${ok?r.need.toFixed(1):'—'}</td><td>${f1(r.pred)}</td><td>${ec===undefined?'—':(ec>0?'+':'')+ec.toFixed(1)}</td><td>${r.cov.toFixed(0)}</td><td>${f1(r.t)}</td><td>${Math.round(r.rad)}</td><td>${f1(r.psol)}</td><td>${(r.pot/1000).toFixed(1)}</td><td>${r.sk.toFixed(1)}</td></tr>`}).join('');
 $('sc').innerHTML=score(a.recs||[]);
 if(first){first=false;$('antMode').value=a.antMode;$('antShade').checked=!!a.antShade;$('tankLiters').value=a.tankLiters;$('tankKWh').value=a.tankKWh;$('reservePct').value=a.reservePct;$('deadline').value=hm(a.deadlineMin);$('lat').value=a.lat;$('lon').value=a.lon}
}
$('save').onclick=async()=>{
 const [h,m]=$('deadline').value.split(':').map(Number);
 const body={antMode:$('antMode').value,antShade:$('antShade').checked,tankLiters:+$('tankLiters').value,tankKWh:+$('tankKWh').value,reservePct:+$('reservePct').value,deadlineMin:h*60+m,lat:+$('lat').value,lon:+$('lon').value};
 const r=await fetch('/api/config',{method:'POST',body:JSON.stringify(body)});
 $('msg').textContent=r.ok?' enregistré':' refusé : '+await r.text();load();
};
load();setInterval(load,10000);
</script></body></html>)rawliteral";

/* ============================================================
   7b. SAUVEGARDE / RESTAURATION (import des exports JSON bruts)
   ============================================================
   Les exports bruts (/api/history, /api/monthly/raw, /api/ant/raw) peuvent etre re-importes, par exemple
   apres un effacement de la flash. L'import FUSIONNE : une entree deja presente n'est jamais ecrasee, et
   la periode en cours (jour / mois) n'est reprise que si le compteur en RAM est encore vide. */
static const char *const IMP_U[5] = {"heatedSec", "relayOnSec", "tier1Sec", "tier2Sec", "nightSec"};
static const char *const IMP_F[6] = {"energyKWh", "nativeEUR", "paidKWh", "paidEUR", "savedKWh", "savedEUR"};

// cycleId = annee*1000 + jour de l'annee (0-365) ; monthKey = annee*100 + mois (1-12)
static bool impKeyOk(uint32_t k, bool monthly) {
  if (monthly) return k >= 200001UL && k <= 210012UL && (k % 100UL) >= 1 && (k % 100UL) <= 12;
  return k >= 2000000UL && k <= 2100365UL && (k % 1000UL) <= 365;
}

// strict = entree importee (validee) ; sinon lecture indulgente du fichier existant (on ne perd jamais une ligne).
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

// Pas de plafond fixe sur la taille du fichier : la vraie limite est la memoire disponible au moment de
// l'import, qui varie dans le temps (purges, fragmentation). On la verifie donc ICI plutot que de refuser
// par avance un fichier qui aurait pu passer. server.arg("plain") a deja copie tout le corps en RAM avant
// d'arriver ici (limite de la bibliotheque WebServer pour un corps JSON brut, non multipart) : un import de
// plusieurs Mo peut donc rester hors de portee tant que cote client (index.html) l'envoi n'est pas decoupe
// en flux ; mais retirer le plafond arbitraire permet deja d'importer des fichiers bien plus gros qu'avant,
// et de maniere adaptative plutot que figee a un nombre choisi a l'avance.
static bool impPrepare(String &body, JsonDocument &in, const char *arrName) {
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "POST attendu"); return false; }
  body = server.arg("plain");
  size_t len = body.length();
  if (len < 10) { server.send(413, "text/plain", "Fichier vide ou illisible"); return false; }
  size_t freeHeap = (size_t)ESP.getMaxAllocHeap();
  // Marge : la construction de l'arbre JSON (ArduinoJson) ajoute de la memoire au fur et a mesure du parsing,
  // en plus du corps deja en RAM ; 20 Ko de marge fixe + le double de la taille du fichier couvre les cas
  // reels observes sur ce projet sans etre un nombre arbitraire sans rapport avec le fichier importe.
  if (freeHeap < 50000 || freeHeap < len + 20000) {
    server.send(503, "text/plain",
      "Memoire insuffisante pour ce fichier maintenant (" + String(len / 1024) + " Ko, " +
      String(freeHeap / 1024) + " Ko libres) : reessayez apres un redemarrage de l'ESP, ou en dehors d'un pic de charge");
    return false;
  }
  DeserializationError err = deserializeJson(in, body);
  if (err || !in[arrName].is<JsonArray>()) {
    server.send(400, "text/plain", String("Fichier invalide : tableau \"") + arrName + "\" attendu");
    return false;
  }
  body = String(); // libere la copie du texte avant de construire le resultat
  return true;
}

// Fusion des jours (historique) ou des mois (recap mensuel).
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
  bool liveDay = false;   // jour en cours a fusionner dans les compteurs en RAM (voir plus bas)
  ImpRow liveRow;
  for (JsonVariantConst v : in[arrName].as<JsonArrayConst>()) {
    ImpRow r;
    if (!impReadRow(r, v, keyName, monthly, true)) { rejected++; continue; }
    if (liveKey != 0 && r.key > liveKey) { rejected++; continue; }       // date future : horloge differente ?
    /* Jour EN COURS : au demarrage le systeme ecrit tout de suite une ligne pour aujourd'hui (compteurs a zero), donc
       « deja presente » serait toujours vrai et la journee sauvegardee ne pourrait jamais revenir apres un effacement de la flash.
       Regle : fusion « jamais vers le bas ». Le fichier n'est repris que s'il est PLUS COMPLET que les compteurs actuels,
       et chaque compteur garde alors la plus grande des deux valeurs (pas de somme : pas de double comptage si on importe deux fois,
       ou si le jour avait deja ete restaure automatiquement au demarrage). Un seul jour en cours par fichier. */
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
      reportError(String("Import : echec d'ecriture ") + path, ERROR_STORAGE);
      server.send(500, "text/plain", "Ecriture impossible (memoire pleine ?)");
      return;
    }
    if (restoreLive) { if (monthly) restoreCurrentMonth(); else restoreCurrentHistory(); }
  }
  if (liveDay) {   // jour en cours : compteurs en RAM = maximum(actuel, importe), puis ecriture de la ligne du jour
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
  Serial.printf("[Import] %s : +%d, %d ignorees, %d refusees\n", path, added, skipped, rejected);
  impSendResult(added, skipped, rejected, rows.size());
}
void apiImportHistory() { apiImportRows(false); }
void apiImportMonthly() { apiImportRows(true); }

// Export brut du recap mensuel : uniquement les mois enregistres (sans le jour en cours que /api/monthly
// ajoute a l'affichage), donc re-importable sans doublon.
void apiMonthlyRaw() {
  File f = LittleFS.open("/monthly.json", "r");
  if (!f) { server.send(200, "application/json", "{\"months\":[]}"); return; }
  server.streamFile(f, "application/json");
  f.close();
}

// Habitudes apprises (besoin du jour, solaire, temperature par cycle).
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
    if (!(r.predWh >= 0.0f))    r.predWh = -1.0f;    // champs optionnels (anciens exports) ; inclut NaN
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
  int hrN = in["hr"].is<JsonObject>() ? antHrMerge(in["hr"]) : 0; // exports anciens : pas de "hr", sans effet
  if (hrN > 0) antHrSave();
  if (added > 0) {
    std::sort(rows.begin(), rows.end(), [](const AntRec &a, const AntRec &b) { return a.cycle < b.cycle; });
    while (rows.size() > ANT_REC_MAX) rows.erase(rows.begin());
    antNRec = (uint8_t)rows.size();
    for (uint8_t i = 0; i < antNRec; ++i) antRecs[i] = rows[i];
    antSave();
    antRefresh();
  }
  Serial.printf("[Import] habitudes : +%d, %d ignorees, %d refusees, heures prevu/reel touchees=%d\n", added, skipped, rejected, hrN);
  impSendResult(added, skipped, rejected, antNRec);
}

// Import du journal de chauffe (POST /api/import/heat, fichier exporte par la page : {"segs":[[t0,sec,wh,paidWh,raison],...]}).
// Fusion sans ecrasement : un bloc qui recouvre dans le temps un bloc deja present est ignore (donc un meme fichier peut etre
// importe deux fois sans doublon). On garde les HEAT_SEG_MAX plus recents.
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
    // Bornes elargies (anciennement 65535, limite uint16 levee par ce correctif) : garde-fou anti-donnees
    // corrompues, pas une limite physique. 604800 s = 7 j, 1e6 Wh = 1 MWh.
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
  // Ligne « relais » (facultative, absente des anciens exports) : meme fusion sans ecrasement.
  std::vector<RelayIv> rrows(relayIvs, relayIvs + relayNIv);
  int relAdded = 0;
  if (in["rel"].is<JsonArrayConst>()) {
    for (JsonVariantConst v : in["rel"].as<JsonArrayConst>()) {
      if (!v.is<JsonArrayConst>()) continue;
      JsonArrayConst r = v.as<JsonArrayConst>();
      uint32_t t0 = r[0] | 0UL, sec = r[1] | 0UL;
      if (t0 < 946684800UL || t0 > maxT || sec == 0UL || sec > 604800UL) continue; // meme garde-fou que heatLoad()
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
  Serial.printf("[Import] journal de chauffe : +%d, %d ignores, %d refuses, relais +%d\n", added, skipped, rejected, relAdded);
  impSendResult(added, skipped, rejected, heatNSeg);
}

/* ============================================================
   8. RÉSEAU & INITIALISATION
   ============================================================ */
constexpr const char *AP_SSID = "Ballon-ESP32";
constexpr const char *AP_PASS = "Ballon_?";

static void wifiApplyRadio() {
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

constexpr uint32_t AP_RESCUE_AFTER_MS     = 60000;  // Wi-Fi perdu depuis ce delai : l'AP de secours s'ouvre
constexpr uint32_t AP_RESCUE_CLOSE_AFTER_MS = 120000; // Wi-Fi stable depuis ce delai : l'AP de secours se ferme
constexpr uint32_t BOOT_AP_DELAY_MS       = 90000;  // pas d'AP de diagnostic avant : le demarrage s'initialise
constexpr uint32_t BOOT_AP_GIVEUP_MS      = 600000; // si Wi-Fi/Ecojoko/Shelly ne sont pas sains avant, pas d'AP de diagnostic pour ce demarrage
constexpr uint32_t BOOT_AP_WIFI_STABLE_MS = 30000;  // Wi-Fi stable depuis ce delai avant d'ouvrir l'AP de diagnostic
constexpr uint32_t BOOT_AP_MIN_HEAP       = 70000;  // RAM libre mini pour OUVRIR l'AP de diagnostic (l'AP coute quelques Ko et le TLS Ecojoko doit garder sa marge)
constexpr uint32_t BOOT_AP_CLOSE_HEAP     = 50000;  // sous cette RAM libre, l'AP de diagnostic est ferme

static void monitorWifiConnection(uint32_t now) {
  if ((uint32_t)(now - rt.lastWifiCheckMs) < WIFI_CHECK_PERIOD_MS) return;
  rt.lastWifiCheckMs = now;

  if (WiFi.status() == WL_CONNECTED) {
    if (rt.wifiDisconnectedSinceMs != 0) {
      Serial.printf("[WiFi] connexion recuperee rssi=%d heure=%s\n", WiFi.RSSI(), nowText().c_str());
    }
    rt.wifiDisconnectedSinceMs = 0;
    return;
  }

  if (!cfg.wifiSsid.length()) return;
  if (rt.wifiDisconnectedSinceMs == 0) {
    rt.wifiDisconnectedSinceMs = now;
    Serial.printf("[WiFi] connexion perdue status=%d rssi=%d heure=%s\n",
                  (int)WiFi.status(), WiFi.RSSI(), nowText().c_str());
    ecoTlsReset(); // La socket TLS persistante est morte avec le lien Wi-Fi.
    return;
  }

  if ((uint32_t)(now - rt.wifiDisconnectedSinceMs) < WIFI_RECONNECT_AFTER_MS) return;
  Serial.printf("[WiFi] reconnexion forcee status=%d rssi=%d heure=%s\n",
                (int)WiFi.status(), WiFi.RSSI(), nowText().c_str());
  ecoTlsReset();
  if (rt.apRescue || rt.bootApOn) {
    // Un AP est ouvert (secours ou diagnostic) : il doit rester joignable pendant les tentatives.
    WiFi.mode(WIFI_AP_STA);
    if (WiFi.softAPIP() == IPAddress((uint32_t)0)) WiFi.softAP(AP_SSID, AP_PASS); // l'AP est (re)configure explicitement
  } else if (WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_STA); // fonctionnement normal : pas d'AP, toute la radio sert la liaison avec la box
  }
  WiFi.disconnect();
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
  wifiApplyRadio();
  rt.wifiDisconnectedSinceMs = now;
}

// La passerelle repond-elle ? Un refus immediat de connexion (RST) prouve qu'elle est joignable ;
// seul un delai complet sans reponse signale une IP/passerelle/masque faux.
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
  // Meme canal que la liaison box si elle est active : la radio est partagee, un autre canal risquerait de couper la liaison.
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

/* Deux regles, dans cet ordre :
   1. Probleme Wi-Fi => AP de secours. Ouvert au demarrage si la connexion echoue (voir connectNetwork), ou apres
      AP_RESCUE_AFTER_MS de Wi-Fi perdu. Ferme seulement quand le Wi-Fi est stable depuis AP_RESCUE_CLOSE_AFTER_MS.
   2. Wi-Fi, Ecojoko et Shelly sains, quelques dizaines de secondes apres le demarrage => AP de diagnostic pendant
      cfg.bootApSec. Ferme a l'echeance, ou des qu'Ecojoko/Shelly ont un probleme (la radio sert le metier).
   Dans les deux cas : jamais de fermeture tant qu'un appareil est connecte a l'AP, et un Wi-Fi perdu transforme l'AP de
   diagnostic en AP de secours. */
static void serviceAp(uint32_t now) {
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn) { if (rt.wifiUpSinceMs == 0) rt.wifiUpSinceMs = now ? now : 1; rt.wifiLostSinceMs = 0; }
  else      { if (rt.wifiLostSinceMs == 0) rt.wifiLostSinceMs = now ? now : 1; rt.wifiUpSinceMs = 0; }

  // Regle 1 : probleme Wi-Fi.
  if (!conn && cfg.wifiSsid.length() > 0 && !rt.apRescue && rt.bootApOn) { // l'AP de diagnostic devient AP de secours
    rt.bootApOn = false; rt.apRescue = true;
  } else if (!conn && cfg.wifiSsid.length() > 0 && !rt.apRescue &&
             (uint32_t)(now - rt.wifiLostSinceMs) >= AP_RESCUE_AFTER_MS) {
    apOpen();
    rt.apRescue = true;
    Serial.println("[WiFi] Wi-Fi perdu : AP de secours ouvert");
  }
  if (rt.apRescue) {
    if (conn && (uint32_t)(now - rt.wifiUpSinceMs) >= AP_RESCUE_CLOSE_AFTER_MS && WiFi.softAPgetStationNum() == 0) {
      apClose();
      rt.apRescue = false;
      Serial.println("[WiFi] Wi-Fi stable : AP de secours ferme");
    }
    return;
  }

  // Regle 2 : AP de diagnostic.
  bool svcOk = rt.ecoConnected && rt.shellyConnected && rt.ecoFailCount == 0 && rt.shellyFailCount == 0;
  if (!rt.bootApOn) {
    if (rt.bootApDone || cfg.bootApSec == 0) return;
    if (now >= BOOT_AP_GIVEUP_MS) { rt.bootApDone = true; return; }
    if (now >= BOOT_AP_DELAY_MS && conn && (uint32_t)(now - rt.wifiUpSinceMs) >= BOOT_AP_WIFI_STABLE_MS && svcOk &&
        ESP.getFreeHeap() >= BOOT_AP_MIN_HEAP) {
      apOpen();
      rt.bootApOn = true; rt.bootApDone = true; rt.bootApStartMs = now;
      Serial.printf("[WiFi] AP de diagnostic ouvert pour %us heapLibre=%u\n", (unsigned)cfg.bootApSec, (unsigned)ESP.getFreeHeap());
    }
    return;
  }
  bool expired = (uint32_t)(now - rt.bootApStartMs) >= (uint32_t)cfg.bootApSec * 1000UL;
  bool svcErr = conn && (rt.shellyFailCount > 0 || rt.ecoFailCount > 0 || ESP.getFreeHeap() < BOOT_AP_CLOSE_HEAP);
  if ((expired || svcErr) && WiFi.softAPgetStationNum() == 0) {
    rt.bootApOn = false;
    apClose();
    Serial.printf("[WiFi] AP de diagnostic ferme (%s)\n", expired ? "delai ecoule" : "erreur Ecojoko/Shelly");
  }
}

void connectNetwork() {
  WiFi.persistent(false); // les identifiants viennent de config.json : pas de copie dans la flash Wi-Fi (usure, ancien AP "fantome")
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
      // IP fixe inutilisable (passerelle injoignable) : retour en DHCP, sans quoi l'ESP32 serait injoignable.
      Serial.println("[WiFi] IP fixe inutilisable : repli DHCP");
      rt.staticFallback = true;
      WiFi.disconnect();
      WiFi.config(IPAddress((uint32_t)0), IPAddress((uint32_t)0), IPAddress((uint32_t)0));
      WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
      wifiApplyRadio();
      waitWifi(20);
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    // Regle 1 : Wi-Fi absent ou faux (premier reglage, faute de frappe...) => AP de secours tout de suite.
    rt.apRescue = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }
}

/* ============================================================
   9. LIAISON AVEC L'ESP32-C3 « CHIEN DE GARDE » (watchdog_c3/watchdog_c3.ino)
   ============================================================
   Deux fils vers le C3 (3,3 V, masse commune, C3 alimente depuis VIN) :
   - HB  (battement de coeur) : change d'etat chaque seconde tant que loop() tourne ET que la boucle de
         controle (tick) est atteinte. Gel, plantage, boucle de redemarrages : plus de battement
         regulier, le C3 sonne seul (il n'a besoin d'aucune information de ce programme).
   - ERR (alarme) : passe a 1 quand un probleme qui compromet la chauffe est CONFIRME (voir wdEvaluate),
         retombe a 0 des qu'il disparait. Le C3 sonne tant qu'ERR reste a 1 (sauf acquittement sur le C3).
   Pas de liaison retour : ce programme ne sait pas si le C3 sonne ni s'il est branche.
   - TX (serie 9600 bauds, UART2, sens unique) : envoie au C3 des trames « $M<minutes>*<XX> » = « reste muet <minutes> minutes ».
         Le C3 compte lui-meme ce silence (voir watchdog_c3.ino) : meme si ce programme se fige, il sonnera a l'echeance.
         Le silence est le plus grand de : la plage silencieuse (cfg.muteOn/muteStart/muteEnd, reste de la plage) et le silence
         a distance (/api/snooze, cfg.snoozeMin). La duree restante est renvoyee toutes les 30 s.
         JAMAIS de silence pendant une surchauffe (WD_OVERHEAT) ni pendant le test du buzzer : trame « M0 » repetee toutes les 2 s.
         Sans broche TX (-1), rien n'est envoye : l'alarme sonne toujours.
   Le mode "monitor" n'alerte jamais : le ballon n'y est volontairement pas pilote. */
constexpr uint32_t WD_HB_PERIOD_MS      = 1000;   // bascule de HB
constexpr uint32_t WD_TICK_MAX_AGE_MS   = 120000; // HB s'arrete si tick() n'a pas ete atteint depuis 2 min (pire cas legitime ~65 s)
constexpr uint32_t WD_FAST_DELAY_MS     = 120000; // relais ouvert + puissance mesuree, memoire presque vide : alerte apres 2 min
constexpr uint32_t WD_LOW_HEAP_BYTES    = 12000;
constexpr uint32_t WD_CMD_FAIL_GAP_MS   = 180000; // echecs de commande Shelly consideres continus tant qu'un nouvel echec arrive sous 3 min
constexpr uint32_t WD_TEST_MS           = 8000;   // le C3 confirme ERR apres 3 s : il reste ~5 s de sonnerie
constexpr uint32_t WD_CRASH_WINDOW_S    = 3600;
constexpr uint32_t WD_CRASH_MAX         = 3;      // 3 plantages en moins d'1 h = boucle de redemarrages
constexpr uint32_t WD_MUTE_PERIOD_MS    = 30000;  // la duree de silence restante est renvoyee au C3 toutes les 30 s
constexpr uint32_t WD_MUTE_BURST_MS     = 1000;   // trames rapprochees apres une action (silence, « Reactiver », changement de plage)
constexpr uint32_t WD_CANCEL_PERIOD_MS  = 2000;   // annulation du silence repetee pendant une surchauffe ou un test
constexpr uint16_t WD_MUTE_MAX_MIN      = 720;    // plafond (le C3 applique le meme)

// WdCause est declare en tete de fichier (voir plus haut)

static const char *wdCauseName(WdCause c) {
  switch (c) {
    case WD_CRASH_LOOP: return "plantages";
    case WD_RELAY:      return "relais";
    case WD_MEMORY:     return "memoire";
    case WD_SHELLY:     return "shelly";
    case WD_COMMAND:    return "commande";
    case WD_CLOCK:      return "horloge";
    case WD_OVERHEAT:   return "surchauffe";
    default:            return "ok";
  }
}

struct WdState {
  int8_t   hbPin = -1, errPin = -1, txPin = -1;
  bool     hbLevel = false, errLevel = false;
  uint32_t lastHbMs = 0, lastEvalMs = 0, testUntilMs = 0;
  uint32_t relaySince = 0, memSince = 0, shellySince = 0, clockSince = 0; // 0 = condition absente
  uint32_t overheatSince = 0, lostHotSince = 0;
  uint32_t cmdFailSince = 0, cmdFailLast = 0;
  uint32_t snoozeUntilMs = 0, lastMuteTxMs = 0; // silence a distance (RAM) ; dernier envoi au C3
  uint8_t  sendBursts = 0;                     // envois rapproches restants
  bool     crit = false;                       // une cause de surchauffe est en alarme (jamais de silence), meme si une autre cause est affichee
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

// Liaison serie de silence : UART2 en emission seule (RX = -1 : aucune broche reservee a la reception).
void wdSetTxPin(int8_t tx) {
  if (tx == wd.txPin) return;
  if (wd.txPin >= 0) { Serial2.end(); pinMode(wd.txPin, INPUT); }
  wd.txPin = tx;
  if (tx >= 0) Serial2.begin(9600, SERIAL_8N1, -1, tx);
}

void wdInit() { wdSetPins(cfg.wdHbPin, cfg.wdErrPin); wdSetTxPin(cfg.wdTxPin); }

// Duree (ms) depuis laquelle `cond` est vraie sans interruption ; since == 0 quand elle est fausse.
static uint32_t wdTrack(bool cond, uint32_t &since, uint32_t now) {
  if (!cond) { since = 0; return 0; }
  if (since == 0) since = now ? now : 1;
  return (uint32_t)(now - since);
}

// Appele par shellySet() : echec definitif (apres toutes les tentatives) ou succes confirme.
void wdNoteCommand(bool ok) {
  uint32_t now = millis();
  if (ok) { wd.cmdFailSince = 0; wd.cmdFailLast = 0; return; }
  if (wd.cmdFailSince == 0 || (uint32_t)(now - wd.cmdFailLast) > WD_CMD_FAIL_GAP_MS) wd.cmdFailSince = now ? now : 1;
  wd.cmdFailLast = now;
}

// Appele une fois par diagBootReport(), avant la remise a zero du drapeau de blocage.
// Seuls les VRAIS plantages comptent (panic, watchdogs, blocage detecte, baisse de tension) :
// un redemarrage voulu (bouton web, changement de Wi-Fi, reflashage) ne declenche rien.
void wdNoteBoot(esp_reset_reason_t reason, bool rtcValid, bool stalled) {
  if (!rtcValid || rtcCrashCount > 1000) { rtcCrashCount = 0; rtcCrashSpanS = 0; return; }
  bool crashed = stalled || reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
                 reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT;
  if (!crashed) return;
  if (rtcCrashCount == 0) { rtcCrashCount = 1; rtcCrashSpanS = 0; return; }
  rtcCrashSpanS += rtcLastUptimeS + 20; // uptime de la session precedente + duree approximative du demarrage
  if (rtcCrashSpanS > WD_CRASH_WINDOW_S) { rtcCrashCount = 1; rtcCrashSpanS = 0; }
  else rtcCrashCount++;
}

static bool wdCrashLoopActive() {
  return rtcCrashCount >= WD_CRASH_MAX && (rtcCrashSpanS + millis() / 1000UL) <= WD_CRASH_WINDOW_S;
}

/* ---------- Journal SOS (/sos_log.json) ----------
   Une entree par alarme confirmee (ERR passe a 1) : heure, cause, explication chiffree, et un instantane de l'etat du systeme
   (mode, Wi-Fi, memoire, phase de loop(), Shelly, Ecojoko, dernier message d'erreur, silence en cours).
   Ecriture uniquement aux changements d'etat de l'alarme (pas a chaque tick) : tres peu d'usure de la flash.
   "dur" absent = alarme en cours ; "reboot":true = l'ESP a redemarre pendant l'alarme (fin inconnue).
   Purge : une alarme terminee depuis plus de 72 h est supprimee (a chaque ecriture, a chaque lecture, puis toutes les heures).
   Si le C3 sonne parce que l'ESP principal est gele (plus de battement), rien ne peut etre ecrit sur le moment : voir /api/diag. */
static uint32_t sosOpenMs = 0;       // millis() au debut de l'alarme en cours (0 = aucune) : sert a calculer la duree
static uint32_t sosLastPruneMs = 0;
static uint16_t wdMuteLeftMin(uint32_t now); // defini plus bas

static void sosRead(JsonDocument &doc) {
  File f = LittleFS.open(SOS_PATH, "r");
  if (f) {
    DeserializationError e = deserializeJson(doc, f);
    f.close();
    if (e) doc.clear();
  }
  if (!doc["events"].is<JsonArray>()) doc["events"].to<JsonArray>();
}

// Supprime les alarmes terminees depuis plus de SOS_KEEP_S (horloge valide requise) et ramene la liste au plafond. true = quelque chose a change.
static bool sosPrune(JsonArray arr) {
  bool changed = false;
  const time_t nowEp = time(nullptr);
  if (nowEp >= 946684800 && (uint32_t)nowEp > SOS_KEEP_S) {
    const uint32_t limit = (uint32_t)nowEp - SOS_KEEP_S;
    for (int i = (int)arr.size() - 1; i >= 0; i--) {
      JsonObject o = arr[i];
      const bool active = o["dur"].isNull() && !(o["reboot"] | false);   // alarme en cours : jamais supprimee
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
    case WD_CRASH_LOOP: return String("boucle de plantages : ") + String((unsigned)rtcCrashCount) + " plantages en moins d'1 h";
    case WD_OVERHEAT:
      if (tKnown && cfg.shellyTAlarmC > 0.0f && rt.shellyT >= cfg.shellyTAlarmC)
        return String("SURCHAUFFE : Shelly a ") + String(rt.shellyT, 1) + " degC (seuil d'alarme " + String(cfg.shellyTAlarmC, 0) + " degC)";
      return String("SURCHAUFFE : Shelly injoignable depuis ") + mn + " min alors que la protection thermique etait active";
    case WD_RELAY:   return String("relais ouvert mais le ballon consomme ") + (isnan(rt.shellyW) ? String("?") : String(rt.shellyW, 0)) + " W depuis " + mn + " min";
    case WD_MEMORY:  return String("memoire presque pleine : ") + String((unsigned long)ESP.getFreeHeap()) + " octets libres (seuil " + String((unsigned long)WD_LOW_HEAP_BYTES) + ")";
    case WD_SHELLY:  return String("Shelly injoignable depuis ") + mn + " min (limite " + String((unsigned)cfg.wdProblemMin) + " min)";
    case WD_COMMAND: return String("commandes Shelly non confirmees depuis ") + mn + " min (limite " + String((unsigned)cfg.wdProblemMin) + " min)";
    case WD_CLOCK:   return String("horloge invalide (NTP) depuis ") + mn + " min (limite " + String((unsigned)cfg.wdProblemMin) + " min)";
    default:         return "cause inconnue";
  }
}

// Appele quand l'etat de l'alarme change : ferme l'entree en cours (si elle existe) puis, si une alarme est active, en ouvre une.
static void sosRecord(bool alarm, WdCause cause, uint32_t elapsedMs, bool tKnown, uint32_t now) {
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  const time_t ep = time(nullptr);
  const uint32_t epoch = (ep < 946684800) ? 0 : (uint32_t)ep;

  // 1. fermer l'alarme precedente
  for (int i = (int)arr.size() - 1; i >= 0; i--) {
    JsonObject o = arr[i];
    if (o["dur"].isNull() && !(o["reboot"] | false)) {
      o["dur"] = sosOpenMs ? (uint32_t)((now - sosOpenMs) / 1000UL) : 0U;
      break;
    }
  }
  sosOpenMs = 0;

  // 2. ouvrir la nouvelle (ou rouvrir la precedente si la meme cause revient tout de suite)
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
      o["t"]     = epoch;                                  // 0 = horloge invalide a ce moment-la
      o["up"]    = (uint32_t)(millis() / 1000UL);          // uptime de l'ESP (s)
      o["cause"] = wdCauseName(cause);
      o["msg"]   = sosDetail(cause, elapsedMs, tKnown);
      o["crit"]  = (cause == WD_OVERHEAT);
      o["mode"]  = cfg.mode;
      o["rssi"]  = WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0;
      o["wifi"]  = WiFi.status() == WL_CONNECTED;
      o["heap"]  = (uint32_t)ESP.getFreeHeap();
      o["heapMin"] = (uint32_t)ESP.getMinFreeHeap();
      o["phase"] = (const char *)diagPhase;
      o["sOk"]   = rt.shellyConnected;                     // Shelly joignable
      o["sOn"]   = rt.shellyOn;                            // relais ferme
      if (!isnan(rt.shellyW)) o["sW"] = (int)lroundf(rt.shellyW);
      if (!isnan(rt.shellyT)) o["sT"] = (int)lroundf(rt.shellyT * 10.0f) / 10.0f;
      o["hold"]  = rt.thermHold;                           // protection thermique active
      o["eco"]   = rt.ecoConnected;
      o["ecoFail"] = (uint32_t)rt.ecoFailCount;
      o["shFail"]  = (uint32_t)rt.shellyFailCount;
      o["crash"] = (uint32_t)rtcCrashCount;
      o["mute"]  = (uint32_t)wdMuteLeftMin(now);           // minutes de silence restantes (plage silencieuse / silence a distance) : > 0 = le buzzer ne devait pas sonner
      if (rt.error.length()) o["err"] = rt.error.substring(0, 80);
    }
  }
  sosPrune(arr);
  writeJsonAtomic(SOS_PATH, doc);
  Serial.printf("[SOS] %s cause=%s heure=%s\n", alarm ? "alarme enregistree" : "fin d'alarme enregistree", wdCauseName(cause), nowText().c_str());
}

// Test du buzzer demande depuis l'interface : une ligne "test" pour ne pas se demander pourquoi ca a sonne.
static void sosRecordTest() {
  JsonDocument doc;
  sosRead(doc);
  JsonArray arr = doc["events"].as<JsonArray>();
  const time_t ep = time(nullptr);
  JsonObject o = arr.add<JsonObject>();
  o["t"]     = (ep < 946684800) ? 0U : (uint32_t)ep;
  o["up"]    = (uint32_t)(millis() / 1000UL);
  o["cause"] = "test";
  o["msg"]   = "test du buzzer demande depuis l'interface";
  o["dur"]   = (uint32_t)(WD_TEST_MS / 1000UL);
  sosPrune(arr);
  writeJsonAtomic(SOS_PATH, doc);
}

// Au demarrage : une alarme encore "en cours" dans le fichier a ete interrompue par un redemarrage (fin inconnue).
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

// Nettoyage horaire : une alarme terminee depuis plus de 72 h disparait meme si personne n'ouvre la page.
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

// GET /api/sos : journal des alarmes (les plus anciennes d'abord), deja purge des alarmes de plus de 72 h.
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

// Decide si ERR doit etre a 1. Un probleme n'alerte qu'apres avoir dure assez longtemps :
//   - boucle de plantages                     : tout de suite
//   - Shelly au-dessus du seuil d'alarme      : tout de suite (shellyTAlarmC) ; Shelly injoignable pendant la protection thermique : 2 min
//   - relais ouvert mais puissance mesuree    : 2 min (le ballon chauffe sans controle)
//   - memoire presque vide                    : 2 min (plantage imminent)
//   - Shelly injoignable                      : wdProblemMin (le ballon ne peut plus etre pilote)
//   - commandes Shelly non confirmees         : wdProblemMin (le relais ne suit plus les ordres)
//   - horloge invalide                        : wdProblemMin (tick() ne prend aucune decision)
// Ecojoko seul n'alerte pas : sans lui la chauffe en heures creuses et le quota continuent de fonctionner.
static void wdEvaluate(uint32_t now) {
  const bool ctrl = (cfg.mode != "monitor");
  const uint32_t slowMs = (uint32_t)cfg.wdProblemMin * 60000UL;

  const bool relayBad = ctrl && !rt.shellyOn && !isnan(rt.shellyW) && rt.shellyW > max(cfg.thermostatW, 50.0f);
  const bool memBad   = ESP.getFreeHeap() < WD_LOW_HEAP_BYTES;
  const bool shellyBad = ctrl && !rt.shellyConnected;
  const bool cmdBad   = ctrl && wd.cmdFailSince != 0 && (uint32_t)(now - wd.cmdFailLast) <= WD_CMD_FAIL_GAP_MS;
  const bool clockBad = ctrl && time(nullptr) < 946684800;
  // Surchauffe : (1) Shelly au-dessus du seuil d'alarme, tout de suite ; (2) Shelly devenu injoignable PENDANT la protection thermique
  // (il etait trop chaud et ne repond plus : coupure d'alimentation, wifi tombe, ou pire) : 2 min. Actifs aussi en mode « monitor ».
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
      if (it.cause == WD_OVERHEAT) crit = true;   // meme si une autre cause (ex. plantages) est passee en premier
    } else if (pending == WD_OK) {
      pending = it.cause; pendingMs = it.elapsed; pendingLimitMs = it.limit;
    }
  }

  if (alarm != wd.alarm || cause != wd.cause) {
    Serial.printf("[Watchdog] alarme=%s cause=%s heure=%s\n", alarm ? "OUI" : "NON", wdCauseName(cause), nowText().c_str());
    sosRecord(alarm, cause, causeElapsedMs, tKnown, now);   // journal SOS (fichier) : raison + etat du systeme
  }
  wd.alarm = alarm; wd.cause = cause; wd.crit = crit;
  wd.pending = pending; wd.pendingMs = pendingMs; wd.pendingLimitMs = pendingLimitMs;
}

// ---- Silence de l'alarme (plage silencieuse + silence a distance) ----
// Trame envoyee au C3 : $M<minutes>*<XX>\n, XX = XOR des caracteres entre '$' et '*' (2 chiffres hexa).
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

// Dans la plage silencieuse ? (horloge invalide : jamais)
static bool wdInQuietWindow() {
  if (!cfg.muteOn || cfg.muteStart == cfg.muteEnd) return false;
  tm lt;
  if (!localNow(lt)) return false;
  const uint16_t mm = (uint16_t)(lt.tm_hour * 60 + lt.tm_min);
  return cfg.muteStart < cfg.muteEnd
         ? (mm >= cfg.muteStart && mm < cfg.muteEnd)
         : (mm >= cfg.muteStart || mm < cfg.muteEnd);
}

// Minutes restantes de la plage silencieuse (0 = hors plage).
static uint16_t wdQuietLeftMin() {
  if (!wdInQuietWindow()) return 0;
  const int left = ((int)cfg.muteEnd - (int)nowMin() + 1440) % 1440;
  return (uint16_t)(left > 0 ? left : 1);
}

// Minutes restantes du silence a distance (0 = aucun). Arrondi a la minute superieure.
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

// Une alarme de surchauffe n'est JAMAIS coupee (risque de feu dans le coffret).
static bool wdCritActive() { return wd.alarm && wd.crit; }

// Appele a chaque passage dans wdService().
static void wdMuteService(uint32_t now, bool testOn) {
  if (wd.txPin < 0) return;
  const bool force = wdCritActive() || testOn;   // silence interdit : le C3 doit pouvoir sonner
  if (wdCritActive()) wd.snoozeUntilMs = 0;
  const uint16_t rem = force ? 0 : wdMuteLeftMin(now);
  uint32_t period;
  if (force) period = WD_CANCEL_PERIOD_MS;
  else if (wd.sendBursts > 0) period = WD_MUTE_BURST_MS;
  else if (rem > 0) period = WD_MUTE_PERIOD_MS;
  else return;                                   // rien a dire : le C3 garde son propre compte a rebours
  if ((uint32_t)(now - wd.lastMuteTxMs) < period) return;
  wd.lastMuteTxMs = now;
  wdSendMute(rem);
  if (!force && wd.sendBursts > 0) wd.sendBursts--;
}

// Plage ou duree modifiee : envoyer tout de suite l'etat reel (y compris 0 si le silence n'a plus lieu d'etre).
void wdMuteChanged() { wd.sendBursts = 3; }

// "on" : couper la sonnerie pendant cfg.snoozeMin minutes (pendant une alarme non critique) ; "off" : la reactiver.
int wdSnoozeCmd(bool on, String &msg) {
  const uint32_t now = millis();
  if (wd.txPin < 0) { msg = "Liaison serie vers le C3 desactivee (broche TX) : coupure a distance impossible"; return 400; }
  if (!on) {
    wd.snoozeUntilMs = 0;
    wd.sendBursts = 3;
    Serial.printf("[Watchdog] silence a distance annule heure=%s\n", nowText().c_str());
    return 200;
  }
  if (wdCritActive()) { msg = "Alarme de surchauffe : elle ne peut pas etre coupee a distance"; return 409; }
  if (!wd.alarm) { msg = "Aucune alarme en cours"; return 409; }
  wd.snoozeUntilMs = now + (uint32_t)cfg.snoozeMin * 60000UL;
  if (wd.snoozeUntilMs == 0) wd.snoozeUntilMs = 1;
  wd.sendBursts = 3;
  Serial.printf("[Watchdog] silence a distance %u min cause=%s heure=%s\n", (unsigned)cfg.snoozeMin, wdCauseName(wd.cause), nowText().c_str());
  return 200;
}

// A appeler a chaque passage dans loop().
void wdService(uint32_t now) {
  if (wd.hbPin >= 0 && (uint32_t)(now - wd.lastHbMs) >= WD_HB_PERIOD_MS) {
    wd.lastHbMs = now;
    if ((uint32_t)(now - rt.lastTick) < WD_TICK_MAX_AGE_MS) { // sinon on se tait : le C3 doit s'en apercevoir
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
  d["wdSnoozeMin"]       = sn;   // reste du silence a distance (min)
  d["wdQuietMin"]        = qu;   // reste de la plage silencieuse (min)
  d["wdMuteMin"]         = mu;   // silence effectif = le plus grand des deux
  if (mu > 0 && time(nullptr) >= 946684800) d["wdMuteUntil"] = (uint32_t)time(nullptr) + (uint32_t)mu * 60U;
}

// Page d'accueil : remplace serveStatic("/", ..., "/index.html") pour pouvoir tracer la requete.
// Sert /index.html.gz (si present) en priorite : plus petit, donc moins de temps de transfert sur un Wi-Fi faible.
// ETag + "no-cache" : le navigateur revalide a chaque chargement et recoit un 304 SANS corps tant que le fichier n'a pas
// change (un rechargement ne coute plus qu'une reponse de quelques octets au lieu de ~100 Ko qui occupent loop()).
static char     indexEtag[24] = "";
static uint32_t indexEtagSize = 0;
static bool     indexEtagGz   = false;

static void serveIndex() {
  bool gz = LittleFS.exists("/index.html.gz");
  if (!gz && !LittleFS.exists("/index.html")) { server.send(404, "text/plain", "index.html absent"); return; }
  File f = LittleFS.open(gz ? "/index.html.gz" : "/index.html", "r");
  if (!f) { server.send(500, "text/plain", "lecture index impossible"); return; }

  // Empreinte calculee une seule fois (relue seulement si le fichier change de taille ou de variante .gz).
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
  // Envoi manuel (et non streamFile) : streamFile() ajoute DEJA « Content-Encoding: gzip » tout seul quand le nom du fichier finit par .gz
  // (core ESP32 recent) ; l'ajouter aussi ici donnait un en-tete en double, que le navigateur decode deux fois (page noire / chargement infini).
  // Ici l'en-tete est ecrit une seule fois, quelle que soit la version du core.
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
  Serial.println("\n--- [DEBUG] ESP32 DÉMARRAGE COLD BOOT ---");

  // Cree avant toute route web et avant controlTask() : aucune des deux ne doit pouvoir demarrer sans lui.
  ioMutex = xSemaphoreCreateMutex();
  if (!ioMutex) { // cas extreme (memoire insuffisante des le demarrage) : mieux vaut redemarrer tout de suite
    Serial.println("[Fatal] creation du mutex ioMutex echouee : redemarrage");
    delay(1000);
    ESP.restart();
  }

  LittleFS.begin(true);
  readConfig();
  if (cfg.lastFwBuild != FW_BUILD) { // nouveau firmware flashe (FW_BUILD change a chaque compilation) : retour au mode principal
    Serial.printf("[Boot] nouveau firmware (mode precedent=%s) : mode full\n", cfg.mode.c_str());
    cfg.mode = "full";
    cfg.lastFwBuild = FW_BUILD;
    saveConfig();
  }
  antLoad(); // historique de la prevision (besoin du jour, solaire, temperature)
  antHrLoad(); // prevu / reel par heure
  antShadeLoad(); // statistique d'ombrage
  heatLoad(); // journal de chauffe
  wdInit(); // broches HB/ERR pilotees le plus tot possible (niveau bas)
  sosBoot(); // alarme restee "en cours" dans le journal SOS = interrompue par ce redemarrage
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
  server.onNotFound([]() {  // ex. /favicon.ico : repond 404 comme avant, mais l'URI est tracee
    diagWebTag(server.uri().c_str());
    server.send(404, "text/plain", "Not found");
  });
  static const char *HDRS_COLLECT[] = {"If-None-Match"}; // necessaire a la revalidation 304 de la page d'accueil
  server.collectHeaders(HDRS_COLLECT, 1);
  server.begin();
  
  rt.lastTick = millis();
  diagLoopMs = millis();
  diagPhase = "loop";
  xTaskCreatePinnedToCore(stallMonitorTask, "stallmon", 3072, nullptr, 1, nullptr, 0);
  // Coeur 1 (comme loopTask/web), PAS le coeur 0 : les appels Ecojoko/Shelly bloquants de controlTask()
  // tournent ainsi a l'ecart des taches internes Wi-Fi/lwIP (coeur 0 par defaut), comme avant la separation
  // web/controle. stallMonitorTask reste seule sur le coeur 0 (surveillance croisee volontaire).
  // Voir le correctif du 15/11 dans le commentaire pres de ioMutex pour le detail du probleme resolu.
  xTaskCreatePinnedToCore(controlTask, "control", 10240, nullptr, 1, nullptr, 1);
}

// POST /api/eco/retry (« Reessayer maintenant ») : AUCUN appel reseau ici (serveur web synchrone). On remet a zero le backoff,
// la session (cookie) et la socket TLS d'Ecojoko ; le prochain passage de sample() (5 s max) refait login puis lecture.
// Le gatewayId / powerMeterId connus sont conserves (pas d'appel /gateways en plus).
void apiEcoRetry() {
  rt.ecoFailCount = 0;
  rt.lastEcoPollMs = 0;
  rt.ecoCookie = "";
  ecoTlsReset();
  Serial.println("[Ecojoko] reessai manuel demande depuis l'interface");
  server.send(200, "text/plain", "OK");
}

// Ecojoko injoignable depuis ECO_BLOCKED_RESTART_MS avec le Wi-Fi OK : on sauvegarde ce qui n'est qu'en RAM (comme le bouton
// « Sauvegarder maintenant »), on note la cause dans /boot_log.txt puis on redemarre. Au pire un redemarrage toutes les 30 min
// si Ecojoko reste en panne (le decompte repart de zero apres chaque boot).
static void ecoBlockedService(uint32_t now) {
  uint32_t blocked = ecoBlockedAgeMs(now);
  if (blocked < ECO_BLOCKED_RESTART_MS) return;
  String line = String("redemarrage auto: Ecojoko injoignable depuis ") + String(blocked / 60000UL) + " min (Wi-Fi OK, echecs consecutifs=" +
                String(rt.ecoFailCount) + ") dernier_message=" + rt.error + " heure=" + diagFormatEpoch((uint32_t)time(nullptr));
  Serial.println("[Ecojoko] " + line);
  diagAppendLog(line);
  heatSaveImpl(true);
  if (rt.cycleId != 0) saveHistory();
  antCycSave();
  delay(300);
  ESP.restart();
}

// Un seul passage de la boucle de controle (watchdog C3, Wi-Fi, Ecojoko/Shelly, decision du relais, meteo).
// Appele en boucle par controlTask(), protege par ioMutex. Corps inchange par rapport a l'ancien appLoop()
// (hors serveur web) : seul l'endroit d'ou il est appele a change.
static void controlPass() {
  uint32_t now = millis();
  wdService(now);
  refreshDailyErrorCount();
  clearResolvedError();
  serviceAp(now);
  monitorWifiConnection(now);
  ecoBlockedService(now);
  shellyCmdService(millis()); // confirmation / nouvelle tentative du relais
  if (now - rt.lastTick >= CONTROL_PERIOD_MS) {
    uint32_t rawDt = (now - rt.lastTick) / 1000;
    uint32_t dt = min(rawDt, MAX_TICK_DT_SEC);
    if (rawDt != dt) {
      Serial.printf("[Quota] dt limite: brut=%lus, utilise=%lus\n", (unsigned long)rawDt, (unsigned long)dt);
    }
    rt.lastTick = now;
    sample();
    tick(dt);
    // Meteo : seulement si ce passage n'a pas deja ete long (marge avec le detecteur de blocage).
    if ((uint32_t)(millis() - now) < 20000UL) antService(millis());
  }
}

// Tache FreeRTOS dediee a la boucle de controle : independante du serveur web (reste dans loop(), voir
// appLoop() ci-dessous). diagLoopMs (surveille par stallMonitorTask) est mis a jour EN TOUT PREMIER, avant
// meme de demander ioMutex, pour que cette tache reste vue comme vivante pendant une attente breve du
// verrou (sinon une requete web legitimement longue pourrait a tort declencher un redemarrage).
static void controlTask(void *) {
  for (;;) {
    diagLoopMs = millis();
    if (xSemaphoreTake(ioMutex, IO_MUTEX_CTRL_WAIT_TICKS) == pdTRUE) {
      controlPass();
      xSemaphoreGive(ioMutex);
    } else {
      // N'arrive que si une requete web est restee anormalement longue dans son handler (JSON/LittleFS,
      // pas de reseau) : on saute ce passage plutot que d'attendre indefiniment, et on reessaie vite.
      Serial.println("[Controle] ioMutex non obtenu a temps (requete web anormalement longue) : passage saute");
    }
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

// loop() (Arduino, cote web) : ne fait plus que servir le serveur HTTP. Toute la logique de pilotage
// (controlPass()) tourne desormais dans sa propre tache (controlTask()) et n'attend plus jamais apres une
// requete web lente.
void appLoop() {
  // Pas de DiagPhaseScope ici : diagPhase appartient desormais exclusivement a controlTask() (c'est la
  // seule tache surveillee par diagLoopMs/stallMonitorTask). Le marquer depuis les deux taches re-creerait
  // une course (la tache web ecraserait le "phase=eco_login" etc. de controlTask() au pire moment : un
  // postmortem de blocage). La tache web garde ses propres marqueurs (diagWebUri/diagWebIp/diagWebStartMs).
  strcpy(diagWebUri, "(attente/lecture)"); // etat normal quand aucune requete n'est servie ; reste ainsi si le blocage est avant le handler (requete incomplete)
  diagWebIp[0] = '\0';
  uint32_t web0 = millis();
  diagWebStartMs = web0;
  server.handleClient();
  diagWebStartMs = 0;
  diagWebAfter(millis() - web0);
}

