# Ballon d'eau chaude intelligent (ESP32 + Ecojoko + Shelly/Ogemray)

Contrôleur de chauffe-eau électrique piloté par un ESP32, qui optimise le déclenchement de la chauffe en fonction :
- du **surplus solaire** (remonté par l'API interne d'**Ecojoko**),
- des **heures creuses / heures pleines** (tarif Linky),
- et d'une **logique tarifaire** (rentabilité HC vs HP, prévision des besoins).

Le relais de puissance (**Shelly 1PM Gen3**, remplacé depuis par un module **Ogemray 25A** compatible API Shelly) coupe/active le chauffe-eau, le contacteur Hager ET221 d'origine restant forcé en position permanente. Un second ESP32-C3 fait office de **chien de garde** externe (watchdog), indépendant du système principal, avec buzzer d'alarme.

> *English version of the firmware and documentation available in [`hot_water_tank_EN/`](./hot_water_tank_EN).*

---

## Structure du dépôt

```
ballon_eau_chaude_fr/                          Version française (principale)
├── esp32-ballon-ecojoko/                      Firmware principal (ESP32)
│   ├── esp32-ballon-ecojoko.ino                Programme Arduino
│   ├── data/                                   Fichiers servis par l'ESP (LittleFS)
│   │   ├── index.html                          Interface web
│   │   └── index.html.gz                       Version compressée (gzip) servie en prod
│   └── tests/
│       └── index.test.mjs                      Tests des fonctions JS de l'interface
├── C3/
│   └── watchdog_c3/
│       └── watchdog_c3.ino                     Firmware du chien de garde (ESP32-C3)
├── Guide_de_demarrage_Ballon_eau_chaude.docx/.pdf   Installation et mise en route
├── Manuel_utilisation_Ballon.docx/.pdf              Utilisation au quotidien (interface, réglages, glossaire)
├── Reference_technique_Ballon.docx/.pdf             Fonctionnement interne (Wi-Fi, Ecojoko/Shelly, diagnostic, dépannage)
└── montage_boitier_esp.docx/.pdf                    Montage physique du boîtier (mono ou double ESP)

hot_water_tank_EN/                             English version
├── EN_esp32_ballon_ecojoko/
│   ├── EN_esp32_ballon_ecojoko.ino
│   └── data/
│       ├── index.html
│       └── index.html.gz
├── C3/
│   └── watchdog_c3/
│       └── watchdog_c3.ino
├── ESP_Enclosure_Assembly.docx/.pdf
├── Getting_Started_Guide_Hot_Water_Tank.docx/.pdf
├── Hot_Water_Tank_Technical_Reference.docx/.pdf
└── Hot_Water_Tank_User_Manual.docx/.pdf
```

---

## Matériel

| Composant | Rôle |
|---|---|
| ESP32 (dual-core classique) | Carte principale : logique de pilotage, serveur web, API Ecojoko/Shelly |
| ESP32-C3 (SuperMini) | Chien de garde indépendant : surveille l'ESP principal, sonne un buzzer passif en cas de panne prolongée |
| Shelly 1PM Gen3 *(remplacé par un module Ogemray 25A, API RPC compatible Shelly)* | Relais de puissance qui commande le chauffe-eau |
| Contacteur Hager ET221 | Forcé en position permanente (circuit « Pilote EDF » coupé) ; le relais fait désormais tout le travail |
| Compteur Linky | Fournit les horaires HC/HP (lus via NTP, pas de câblage direct) |
| Ecojoko | Fournit la production solaire en temps réel (API interne, type intégration *Little Monkey*) |
| Buzzer passif + bouton on/off | Sur le C3, pour l'alarme et son acquittement |

Aucune dépendance à Home Assistant ou à un cloud pour le fonctionnement de base : l'ESP32 parle directement à Ecojoko (HTTPS) et au relais (HTTP local).

---

## Logiciel

- **Firmware** : Arduino C++ (`.ino`), stockage **LittleFS** pour l'interface web et l'historique
- **Interface web** : page unique (`index.html`), servie en gzip, avec onglets Accueil / Historique / Prévision / Réglages / Système
- **Modes de fonctionnement** : `auto` (quota, historique), `manual`, `monitor`, **`full`** (mode recommandé par défaut : chauffe dès que c'est rentable en HP, et systématiquement pendant les heures creuses)
- **Prévision** : estimation du besoin et de la production solaire à partir de l'historique appris (sans sonde de température pour l'instant)
- **Chien de garde C3** : liaison GPIO (battement de cœur / alarme) + liaison série pour la mise en silence à distance de l'alarme

---

## Démarrage rapide

1. Flasher `esp32-ballon-ecojoko.ino` (ou `EN_esp32_ballon_ecojoko.ino`) sur l'ESP32 principal, avec le contenu de `data/` en LittleFS.
2. Flasher `watchdog_c3.ino` sur l'ESP32-C3.
3. Se connecter au point d'accès Wi-Fi émis par l'ESP32 et y renseigner les identifiants du Wi-Fi de la maison.
4. Dans l'interface web : renseigner les identifiants Ecojoko, l'IP du relais, les tarifs et les horaires HC/HP.
5. Laisser le mode `full` (mode par défaut conseillé) ; la prévision peut rester en observation tant qu'elle n'est pas jugée concluante.

Pour le détail complet, voir le **Guide de démarrage** (`Guide_de_demarrage_Ballon_eau_chaude.docx` / `.pdf`).

---

## Documentation

| Document | Contenu |
|---|---|
| `montage_boitier_esp` | Montage physique du boîtier (mono-ESP ou double ESP avec buzzer/bouton) |
| `Guide_de_demarrage_Ballon_eau_chaude` | Installation et mise en route (réseau de secours, mise à jour du programme et de la page, vérification finale) |
| `Manuel_utilisation_Ballon` | Utilisation au quotidien : découvrir le système, l'interface, les réglages, glossaire |
| `Reference_technique_Ballon` | Fonctionnement interne : séquence de démarrage, Wi-Fi, échanges Ecojoko/Shelly, rôle du chien de garde C3, diagnostic et dépannage |

*(Versions anglaises équivalentes dans `hot_water_tank_EN/` : `ESP_Enclosure_Assembly`, `Getting_Started_Guide_Hot_Water_Tank`, `Hot_Water_Tank_User_Manual`, `Hot_Water_Tank_Technical_Reference`.)*

---

## État actuel

- Résilience réseau (reprise Ecojoko/Shelly après coupure), redémarrage auto si Ecojoko reste en échec
- Historique détaillé + récap mensuel/annuel, export/import (CSV/JSON)
- Suivi « prévu vs réel » du solaire et du besoin (mode shadow)
- Interface web entièrement remaniée (thème sombre, responsive mobile/PC)
- Chien de garde C3 opérationnel (alarme, plage de silence programmable, mise en silence à distance)

Travaux en cours / pistes : passage éventuel à `ESPAsyncWebServer`, séparation de `index.html` en HTML/CSS/JS, protection thermique avancée du relais.
