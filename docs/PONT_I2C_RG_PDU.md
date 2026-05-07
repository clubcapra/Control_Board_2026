# Pont I²C RoboGuard ↔ PDU — Mise en service et dépannage

Ce document décrit comment mettre en service, vérifier et dépanner le pont I²C
entre la carte **RoboGuard** (STM32F446RE, maître) et le **PDU** sur Blue Pill
(STM32F103C8T6, esclave).

> Pour la spécification du protocole (registres, structures, commandes), voir
> [`README.md`](../README.md) à la racine du projet `code_lm5066h1`. Le présent
> document couvre l'**intégration matérielle**, la **procédure de mise en
> service** et la **chasse aux bugs** réalisée durant la mise au point.

---

## 1. Architecture du bus

| Rôle             | MCU            | Pile logicielle    | Projet           |
|------------------|----------------|--------------------|------------------|
| **Maître**       | STM32F446RE    | I²C bit-bangé      | `RoboGuard`      |
| **Esclave**      | STM32F103C8T6  | I²C2 (matériel)    | `code_lm5066h1`  |

| Paramètre                  | Valeur          |
|----------------------------|-----------------|
| Adresse esclave (7 bits)   | `0x31`          |
| Horloge SCL                | **50 kHz**      |
| Endianness des structures  | Little-endian   |
| Encapsulation              | Packed, pas de padding |

### Brochage

| Signal | RoboGuard (F446RE) | PDU (Blue Pill) |
|--------|---------------------|-----------------|
| SCL    | `PB10`              | `PB10`          |
| SDA    | `PC12`              | `PB11`          |
| GND    | `GND` (commun)      | `GND` (commun)  |

> **Important :** RoboGuard utilise un I²C bit-bangé en logiciel parce que ses
> broches I²C2 matérielles sont occupées ailleurs sur la carte. Cela signifie
> que **les pull-ups externes (4,7 kΩ – 10 kΩ vers 3,3 V) sont fortement
> recommandés** sur SCL et SDA. Les pull-ups internes du STM32 (~30–50 kΩ) sont
> à la limite de la spécification I²C même à 50 kHz.

---

## 2. Configurations de build

### Côté esclave (`code_lm5066h1`, Blue Pill)

| Environnement              | Quand l'utiliser                                        |
|----------------------------|---------------------------------------------------------|
| `bluepill_flight`          | **Production.** Initialise tout (LM5066H1, LEDs, winch, E-stop, fault log, API I²C2). |
| `bluepill_api_debug`       | **Mise au point du protocole.** Initialise *seulement* l'API I²C2. Les LEDs, les rails et le winch ne sont pas montés, donc les commandes correspondantes renvoient `kStatusNotInit (=8)` mais le bus reste pleinement fonctionnel. Active aussi les traces `[API RX] / [API TX]` sur SWO. |

```bash
# Production
pio run -e bluepill_flight -t upload

# Débogage du protocole I²C
pio run -e bluepill_api_debug -t upload
```

### Côté maître (`RoboGuard`, F446RE)

| Environnement                  | Quand l'utiliser                                           |
|--------------------------------|------------------------------------------------------------|
| `light`                        | **Mise au point.** Pas de micro-ROS, expose la boucle de test PDU sur `Serial1`. |
| `with_micro_ros[_precompiled]` | Production avec ROS.                                       |

```bash
pio run -e light -t upload
```

---

## 3. Vérification du bon fonctionnement

### 3.1 Trace SWO de l'esclave

Au démarrage du PDU, vous devez voir sur SWO (PB3) :

```
[API INIT] I2C2 slave addr=0x31 clock=50000Hz pins SCL=PB10 SDA=PB11
```

Si cette ligne **n'apparaît pas**, c'est que l'esclave n'atteint jamais
`control_api::init()`. Vérifiez :
- L'horloge HSE (8 MHz) est bien câblée et `HSE_VALUE=8000000U` est défini ;
- Le watchdog ne réinitialise pas le MCU avant la fin de `setup()` ;
- L'environnement compilé est bien `bluepill_flight` ou `bluepill_api_debug`.

### 3.2 Trace SWO en mode `bluepill_api_debug`

Quand le maître interroge l'esclave, vous voyez les événements I²C en direct :

```
[API RX] select reg=0x00 bytes=1
[API TX] reg=0x00 len=16
[API RX] command reg=0x20 len=5 cmd=0x00 arg0=0x00 arg1=0x00
[API RX] select reg=0x21 bytes=1
[API TX] reg=0x21 len=8
```

- `[API RX]` = octets reçus depuis le maître (sélection de registre ou commande).
- `[API TX]` = octets envoyés au maître (réponse à une lecture).
- `[API DROP]` = file de trace pleine, événements perdus (cosmétique).

### 3.3 Console série du maître (`Serial1`, PC6/PC7, 9600 bauds)

```
[INFO] OK magic=PDU1 proto=1.9 fw=1.0.0 addr=0x31 rails=3
[T0.0] noop -> seq=1 rc=0 cmd=0x00 arg0=0 arg1=0
[INFO] OK magic=PDU1 proto=1.9 fw=1.0.0 addr=0x31 rails=3
[T0.1] setLedDuty(BRAS,25%) -> seq=2 rc=0 cmd=0x02 arg0=0 arg1=25
... etc, 10 commandes par cycle ...
--- end of test cycle ---
```

Ce que les champs signifient :
- `magic=PDU1` : entête confirmant qu'on parle bien à un PDU.
- `seq=N` : compteur monotone d'exécution côté esclave. **Doit augmenter à
  chaque commande.** S'il stagne, l'esclave ne traite plus rien.
- `rc=` : code de retour `kStatusXxx` (0 = OK, 8 = NotInit, etc.).
- `cmd=0xNN arg0=… arg1=…` : écho des octets de la commande, byte-pour-byte.

> En mode `bluepill_api_debug`, les commandes qui touchent les sous-systèmes
> non initialisés (LEDs, winch) renvoient `rc=8 (kStatusNotInit)`. Ce
> **n'est pas un bug** — le bus fonctionne, le PDU rapporte fidèlement que le
> sous-système n'est pas en service dans cette image. Pour tester avec le vrai
> matériel, basculer en `bluepill_flight`.

---

## 4. Bugs corrigés durant la mise en service

Trois vrais bugs ont été corrigés côté esclave et un manquement côté maître.
Documentés ici pour éviter qu'ils se réintroduisent.

### Bug n° 1 — `setClock()` avant `begin()`

**Plateforme :** `code_lm5066h1/src/control_api.cpp`

**Symptôme :** Aucune communication. Le maître envoyait, l'esclave NACKait
toutes les adresses.

**Cause :** Dans la librairie stm32duino,
`TwoWire::begin(uint8_t address)` force l'horloge I²C à **100 kHz en dur**
dans `i2c_init()` :

```cpp
i2c_init(&_i2c, 100000, ownAddress);
```

L'appel à `setClock(50000)` *avant* `begin()` était silencieusement écrasé.
Et placé *après* `begin()`, il repassait par `HAL_I2C_Init()` qui efface les
bits `I2C_IT_EVT | I2C_IT_ERR` — l'esclave sortait du mode LISTEN.

**Correctif :** Bon ordre + ré-armer manuellement le mode Listen.

```cpp
s_api_wire.setSCL(cfg::kPin_API_I2C2_SCL);
s_api_wire.setSDA(cfg::kPin_API_I2C2_SDA);
s_api_wire.begin(cfg::kApiI2cAddress);                     // 1) init à 100 kHz
s_api_wire.setClock(cfg::kApiI2cClock_Hz);                 // 2) réajuste à 50 kHz
(void)HAL_I2C_EnableListen_IT(s_api_wire.getHandle());     // 3) réarme l'écoute
s_api_wire.onReceive(onReceive);
s_api_wire.onRequest(onRequest);
```

### Bug n° 2 — `publishSnapshot()` saturait les interruptions

**Plateforme :** `code_lm5066h1/src/control_api.cpp`

**Symptôme :** Le bus marchait pour **un seul cycle** de test, puis l'esclave
devenait sourd pour toujours. Seul un redémarrage du Blue Pill remettait les
choses en marche.

**Cause :** `publishSnapshot()` était appelé à chaque itération de `loop()` et
contenait une copie atomique de `s_snapshot` (944 octets) sous `noInterrupts()`.
En `bluepill_api_debug`, la boucle ne fait rien d'autre, donc cette section
critique de ~50 µs s'exécutait des milliers de fois par seconde. L'ISR de
l'I²C2 finissait par manquer une fenêtre `STOPF`/`AF` et l'état HAL se
retrouvait coincé : `CR1.ACK` à zéro mais `slaveMode` toujours LISTEN. À
partir de là, plus aucune adresse n'était acquittée.

**Correctif :** Limiter la publication à 20 Hz (au plus une fois toutes les
50 ms).

```cpp
static uint32_t s_last_publish_ms = 0U;
const uint32_t now = millis();
if ((now - s_last_publish_ms) < 50U) {
    return;
}
s_last_publish_ms = now;
```

### Bug n° 3 — Aucune récupération si l'esclave restait coincé

**Plateforme :** `code_lm5066h1/src/control_api.cpp`

**Symptôme :** Si le périphérique I²C2 sortait du mode LISTEN pour une autre
raison (bruit sur le bus, ISR longue ailleurs, etc.), il y restait jusqu'au
prochain reset matériel.

**Correctif :** Ajout de `rearmListenIfStuck()`, appelée à chaque `tick()`.
Le chemin rapide se résume à deux lectures de registre + une comparaison
booléenne. Si elle détecte un état dégradé (`PE` ou `ACK` à zéro, état
différent de LISTEN, et aucun transfert en cours), elle exécute un
`SWRST → HAL_I2C_Init → HAL_I2C_EnableListen_IT` pour remettre le périphérique
en marche en moins d'1 ms, sans rebooter le MCU.

### Bug n° 4 — Boucle de test du maître trop minimaliste

**Plateforme :** `RoboGuard/src/main.cpp`

**Symptôme :** La boucle ne testait qu'un seul `noop` aux 2 secondes —
impossible de vérifier que le reste de l'API (LEDs, E-stop, log de fautes…)
répondait correctement.

**Correctif :** Remplacement par une **machine à états à 10 étapes** qui,
toutes les secondes :

1. Lit `/info` (registre `0x00`, 16 octets) — sonde de vivacité du bus.
2. Envoie une commande différente à chaque cycle (rotation modulo 10) :
   `noop` → 4× `setLedDuty` → `setAllLeds` → 2× `setLedPattern` →
   `setEstopVtx` → `clearFaultLog`.
3. Lit `ApiCommandResult` (registre `0x21`, 8 octets) et imprime le résultat
   sur `Serial1`.

---

## 5. Dépannage rapide

| Symptôme observé                                              | Diagnostic probable                                                | À vérifier                                                                                  |
|---------------------------------------------------------------|--------------------------------------------------------------------|---------------------------------------------------------------------------------------------|
| Pas de `[API INIT]` sur SWO                                   | L'esclave ne démarre pas / reset par watchdog                      | Câble SWD, alimentation 3,3 V, présence de l'horloge HSE 8 MHz.                              |
| `[API INIT]` présent, mais maître affiche `[INFO] FAIL` partout | Problème électrique sur le bus                                     | Pull-ups externes 4,7–10 kΩ, masse commune entre RG et PDU, longueur de câble (< 30 cm idéalement). |
| Premier cycle OK puis `[INFO] FAIL` à répétition              | Ce devrait être impossible avec les correctifs en place            | Vérifier que `publishSnapshot()` a bien le rate-limit 50 ms et que `rearmListenIfStuck()` est appelée à chaque tick. |
| `seq=` n'augmente plus alors que `[INFO] OK` continue          | L'esclave répond aux lectures mais ne traite plus les écritures de commande | `tick()` est-il bien appelé dans `loop()` ? `s_initialised` est-il à `true` ?              |
| `rc=8 (kStatusNotInit)` sur des commandes LED ou winch         | **Comportement normal** en `bluepill_api_debug`                    | Recompiler en `bluepill_flight` pour exercer le vrai matériel.                              |
| `rc=2 (kStatusTimeout)` ou `rc=1 (kStatusBusError)` sur des commandes PMBus | Le LM5066H1 du rail concerné ne répond pas                | Tension d'entrée présente sur le rail, broche `ENABLE` non bloquée bas, adresse correcte (0x52 / 0x43 / 0x41). |
| `[INFO] OK` mais magic ≠ `"PDU1"`                              | Corruption en lecture (probablement timing)                        | Réduire la fréquence à 25 kHz : `pdu_client.begin(25000)` côté RG, `kApiI2cClock_Hz = 25000UL` côté PDU. |

---

## 6. Limites connues

- **`kTelemetryAll` (registre `0x10`, 944 octets) ne peut pas être lu en une
  seule transaction** parce que le tampon matériel de l'esclave stm32duino
  (`I2C_TXRX_BUFFER_SIZE`) est de 32 octets. Si vous avez besoin de toute la
  télémétrie, il faut soit la chunker en plusieurs registres, soit augmenter
  le tampon avec `-DI2C_TXRX_BUFFER_SIZE=1024` dans le `build_flags` du PDU.
- **Le bit-bangé du maître bloque le coeur du F446RE pendant chaque
  transaction.** À 50 kHz, lire `/info` (16 octets) prend ~3,5 ms ; lire
  `ApiCommandResult` (8 octets) ~2 ms. Pour des taux de mise à jour plus
  élevés, basculer sur l'I²C2 matériel (`Client(Wire, 0x31)`) en re-câblant
  SDA sur PB11 plutôt que PC12.
- **L'esclave n'incrémente `seq=` que dans `processCommand()`, pas dans
  `processBridge()`.** Si vous mélangez les deux types de requêtes, comparer
  les `seq` entre commandes et bridges n'a pas de sens : ce sont des
  séquenceurs distincts (`s_command_result.sequence` vs
  `s_bridge_result.sequence`).

---

## 7. Références fichiers

| Sujet                                  | Fichier                                                          |
|----------------------------------------|------------------------------------------------------------------|
| Spécification du protocole I²C         | [`code_lm5066h1/README.md`](../README.md)                        |
| Init et callbacks de l'esclave I²C     | `code_lm5066h1/src/control_api.cpp`                              |
| Adresse, broches, horloge esclave      | `code_lm5066h1/include/avionics_config.h`                        |
| Définitions des structures et codes    | `RoboGuard/lib/pdu_i2c_api/pdu_i2c_api.h`                        |
| Implémentation du client (bit-bang)    | `RoboGuard/lib/pdu_i2c_api/pdu_i2c_api.cpp`                      |
| Boucle de test côté maître             | `RoboGuard/src/main.cpp` (`run_pdu_test_cycle`)                  |
