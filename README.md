# Alba — CO2 Display (Pico 2W + MH-Z19B + GC9A01)

Capteur CO2 connecté « **alba** » : affichage animé (anneaux concentriques qui « respirent ») sur écran rond 1.28" (240x240), anneau LED WS2812 synchronisé, provisioning WiFi **depuis l'app AirCarto (Bluetooth)** ou par portail captif, et envoi des mesures vers l'API AirCarto.

## Schematic

![Schéma Alba](schematic%20Alba.png)

## Hardware

- **Board** : Raspberry Pi Pico 2W
- **Display** : GC9A01 240x240 round LCD
- **Sensor** : MH-Z19B CO2 sensor
- **LEDs** : anneau WS2812 (64 LEDs)

## Wiring

| Pico | GC9A01 Display |
|------|----------------|
| GP10 | SCK |
| GP11 | MOSI |
| GP13 | CS |
| GP14 | DC |
| GP15 | RST |
| GP12 | BL |

| Pico | LED ring / WS2812 |
|------|-------------------|
| GP22 | DIN |
| 5V   | 5V |
| GND  | GND |

| Pico | MH-Z19B |
|------|---------|
| GP6 (TX) | RX |
| GP7 (RX) | TX |
| 5V   | VIN |
| GND  | GND |

> Le MH-Z19B est câblé **croisé** : TX du capteur → GP7 (RX du Pico), RX du capteur → GP6 (TX du Pico). GP6/GP7 ne sont pas sur un UART matériel, le firmware utilise un port série émulé par PIO (`SerialPIO`).

> L'anneau LED est piloté par **WS2812FX** (mode custom « breathing » synchronisé avec la respiration de l'écran). Ajuster `LED_COUNT` dans `src/main.cpp` si l'anneau n'a pas 64 LEDs.

> L'affichage est rendu avec **LVGL v9** : une « cible » d'anneaux concentriques dessinée en un seul dégradé radial (couleur cœur → bord → fondu noir) qui respire à ~60 fps. `Arduino_GFX` ne sert que de driver de flush bas niveau pour le GC9A01. La config LVGL est dans `include/lv_conf.h` (activée via `-DLV_CONF_INCLUDE_SIMPLE`).

## Flasher sans rien installer (fichier .uf2)

Le firmware compilé est fourni dans **`firmware/alba-firmware-1.1.0.uf2`** :

1. Débrancher le Pico 2W, **maintenir BOOTSEL** et rebrancher l'USB.
2. Relâcher : une clé USB **`RP2350`** apparaît.
3. Y **glisser-déposer** le fichier `.uf2` — le Pico redémarre tout seul.

Le WiFi déjà enregistré est conservé (le .uf2 ne remplace que le programme).

## Build & Flash (PlatformIO)

Le firmware est en **C++ / Arduino** (core earlephilhower, **épinglé en 5.6.0** dans `platformio.ini` — l'AP+STA concurrent fiable nécessite arduino-pico ≥ 5.5.1) géré par **PlatformIO**.

```bash
# Compiler
pio run

# Compiler + flasher le Pico 2W (en BOOTSEL, ou via USB si déjà flashé)
pio run --target upload

# Moniteur série (115200 baud)
pio device monitor
```

Pour le tout premier flash : maintenir **BOOTSEL**, brancher l'USB, relâcher quand le lecteur **RPI-RP2** apparaît, puis `pio run -t upload`.

> Les anciens fichiers MicroPython (`micropython/main.py`, `micropython/code.py`, etc.) sont obsolètes : le firmware actif est `src/main.cpp`.

## Configuration WiFi depuis l'app AirCarto (Bluetooth)

Tant qu'il n'est pas connecté (mode configuration), alba s'annonce en Bluetooth LE sous le nom **`Alba-<TOKEN>`** (ex : `Alba-88A29E0CEFF3`), avec le service **Improv WiFi** — exactement le protocole des ModuleAir / ModuleAir mini (`src/ble_improv.cpp`, mêmes UUID, mêmes octets).

1. Dans l'app : **Ajouter un capteur** → alba apparaît dans « Capteurs détectés » (icône CO₂, « Alba · WiFi à configurer »).
2. Choisir le réseau (liste scannée par alba), saisir le mot de passe, **Connecter**. L'écran d'alba affiche « Connexion a … » puis la coche verte.
3. alba renvoie son **token** à l'app, qui retrouve le capteur côté serveur (`getPublicName.php` → `alba-001`) et l'ajoute au tableau de bord.

Les identifiants ne sont écrits en flash **qu'après une connexion réussie** : un mot de passe erroné laisse le réseau enregistré intact, et l'app peut réessayer sans rien couper (le hotspot reste actif pendant la tentative, en AP+STA). Une fois en ligne, alba cesse d'annoncer (après ~8 s, le temps que l'app lise le verdict). S'il perd son réseau et retombe en mode configuration, il réapparaît dans l'app, qui propose « Reconfigurer le WiFi ».

> Le capteur doit être **enregistré côté serveur** (`capteurs.capteurs`, type `Alba`, `token` = MAC affichée au boot — touche `i` dans le moniteur série) pour que l'app récupère son nom et ses données.

## Configuration WiFi (portail captif)

Alternative sans l'app : à la première mise sous tension (ou après un reset), si aucun identifiant WiFi n'est enregistré, l'appareil démarre aussi un **point d'accès ouvert** et un **portail captif** :

1. L'écran affiche `Config WiFi`, le nom du réseau (`alba-XXXXXX`) et l'IP (`192.168.4.1`).
2. Sur un téléphone/PC, rejoindre le réseau WiFi **`alba-XXXXXX`** (ouvert, sans mot de passe).
3. La page de configuration s'ouvre automatiquement (sinon aller sur `http://192.168.4.1`).
4. Choisir son réseau dans la liste scannée, saisir le mot de passe, **Connecter**.
5. L'appareil enregistre les identifiants (LittleFS) et redémarre connecté.

Une fois connecté, l'IP locale de l'appareil sert une **page de statut** (CO2 en direct, réseau, signal, bouton « Oublier le WiFi »).

Si la connexion échoue ou tombe, une machine à états (inspirée de ModuleAir_V4) gère le repli : hotspot de config pendant 3 min, puis affichage ppm avec **reconnexion en arrière-plan toutes les 10 min** (AP+STA concurrent — le hotspot reste actif pendant les tentatives).

### Réinitialiser le WiFi

- Depuis la page de statut : bouton **« Oublier le WiFi »**, ou
- Via le moniteur série : envoyer la touche **`r`**.

L'appareil efface la config et redémarre en mode configuration.

## Envoi des données (AirCarto)

Une fois connecté, l'appareil envoie **toutes les 60 s** la moyenne des mesures CO2 de la fenêtre écoulée (pas la dernière valeur instantanée) vers :

```
POST https://api.aircarto.fr/capteurs/alba.php
```

L'identifiant de l'appareil (`device_id`) est dérivé de l'adresse MAC — c'est le même token qui apparaît dans le nom du hotspot `alba-XXXXXX`.

## CO2 Levels

| Level | Color | Action |
|-------|-------|--------|
| < 600 ppm | 🟢 Green | Excellent |
| 600-800 ppm | 🔵 Cyan | Good |
| 800-1200 ppm | 🟣 Purple | Open windows |
| > 1200 ppm | 🔴 Red | Ventilate! |
