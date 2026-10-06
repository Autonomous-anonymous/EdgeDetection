# CARLA ADAS Testing – Edge Detection, Path Prediction and Automatic Emergency Braking

Projektissa kehitetään ja testataan autonomisen ajoneuvon ADAS-toimintoja CARLA-simulaatioympäristössä.

Projektin ensimmäinen vaihe toteutetaan Software-in-the-Loop (SIL) -testauksena. Python-ohjelmat kommunikoivat CARLA-simulaattorin kanssa, käsittelevät virtuaalista sensoridataa, suorittavat ADAS-controller-logiikkaa ja tallentavat testituloksia analysointia varten.

Projektissa on tällä hetkellä kaksi pääkokonaisuutta:

1. kameraperusteinen edge detection ja path prediction
2. Automatic Emergency Braking (AEB)

AEB-osuus on edennyt yksinkertaisesta kiinteään etäisyysrajaan perustuvasta prototyypistä erilliseksi SIL-controlleriksi, joka kommunikoi CARLA-testin kanssa ZeroMQ:n avulla. Controllerista on toteutettu myös Time To Collision (TTC) -pohjainen versio sekä suhteellista nopeutta käyttävä moving-obstacle-versio.

Projektin myöhemmässä vaiheessa tavoitteena on siirtää sama controller-ajatus sulautetulle järjestelmälle ja edetä Hardware-in-the-Loop (HIL) -testaukseen.

---

# Projektin tavoite

Projektin tavoitteena on rakentaa helposti toistettava testausympäristö, jossa autonomisen ajoneuvon toimintoja voidaan kehittää ja validoida vaiheittain.

Nykyinen kokonaisuus:

```text
CARLA-simulaatio
        ↓
Virtuaalinen ajoneuvo
        ↓
Virtuaalinen sensoridata
        ↓
ZeroMQ
        ↓
Erillinen Python-controller
        ↓
ADAS-päätös
        ↓
ZeroMQ
        ↓
CARLA-ajoneuvon ohjaus
        ↓
Automaattiset SIL-testit
        ↓
CSV-tulokset
        ↓
Analysointi ja controllerin kehitys
        ↓
Mahdollinen HIL-validointi
```

Keskeinen tavoite on erottaa ADAS-controller simulaattorista niin, että controllerin logiikka ei ole suoraan riippuvainen CARLAsta. Tämä helpottaa saman logiikan siirtämistä myöhemmin esimerkiksi Raspberry Pi -laitteelle.

---

# Projektin nykyinen tila

Projektissa on toteutettu:

- OpenCV Proof of Concept valmiilla ajovideolla
- CARLA Python API -yhteys
- virtuaalisen ajoneuvon luominen
- CARLA autopilot
- RGB-etukamera
- reaaliaikainen kameradatan käsittely Pythonilla
- Canny Edge Detection
- Region of Interest
- Hough Line Transform
- kaistaviivojen tunnistus
- path prediction
- ohjaussuunnan arviointi
- obstacle sensor -prototyyppi
- collision sensor
- Automatic Emergency Braking
- automaattinen PASS/FAIL-testaus
- CSV-testitulosten tallennus
- CARLA synchronous mode
- kiinteä 0.05 s simulaatioaskel
- automatisoitu AEB-testisarja
- kontrolloidut 20 / 30 / 40 / 50 km/h testit
- vähintään 1.0 m turvamarginaali PASS/FAIL-kriteerissä
- simulatorin ja controllerin erottaminen omiksi prosesseiksi
- ZeroMQ-kommunikaatio simulatorin ja controllerin välillä
- kiinteän 10 m AEB-controllerin baseline
- TTC-pohjainen AEB
- TTC-rajojen 1.5 s ja 1.7 s vertailu
- suhteelliseen nopeuteen perustuva TTC
- liikkuvan esteen testit nopeuksilla 0 / 20 / 30 / 40 km/h
- controllerin käsittelyajan ja ZeroMQ round-trip -ajan mittaus

---

# Projektin rakenne

```text
EdgeDetection/
│
├── README.md
│
└── edge-path-project/
    │
    ├── main.py
    ├── carla_connect.py
    ├── carla_vehicle.py
    ├── carla_camera.py
    ├── carla_path_prediction.py
    ├── carla_emergency_brake.py
    ├── carla_aeb_test.py
    ├── carla_aeb_batch_test.py
    │
    ├── environment.yml
    ├── .gitignore
    │
    ├── aeb_sil/
    │   ├── simulator.py
    │   ├── controller.py
    │   ├── target_speed_test.py
    │   ├── controller_ttc.py
    │   ├── target_speed_test_ttc.py
    │   ├── controller_ttc_relative.py
    │   └── moving_obstacle_test.py
    │
    ├── videos/
    ├── output/
    │
    └── results/
        ├── aeb_results.csv
        ├── aeb_batch_results.csv
        ├── aeb_sil_target_speed_results.csv
        ├── aeb_sil_ttc_results.csv
        └── aeb_relative_ttc_results.csv
```

---

# Tiedostojen tarkoitus

## `main.py`

Projektin ensimmäinen OpenCV Proof of Concept.

Ohjelma käsittelee valmista ajovideota ja suorittaa:

```text
Ajovideo
    ↓
Harmaasävymuunnos
    ↓
Gaussian Blur
    ↓
Canny Edge Detection
    ↓
Region of Interest
    ↓
Hough Line Transform
    ↓
Vasemman ja oikean kaistaviivan arviointi
    ↓
Path Prediction
    ↓
Ohjauskulman arviointi
    ↓
LEFT / STRAIGHT / RIGHT
```

Tämän vaiheen tarkoituksena oli varmistaa konenäköalgoritmin toiminta ennen sen liittämistä simulaatioon.

Käsitelty video voidaan tallentaa esimerkiksi:

```text
output/path_prediction.mp4
```

## `carla_connect.py`

Ensimmäinen CARLA-yhteystesti.

Tiedosto:

- muodostaa yhteyden käynnissä olevaan CARLA-palvelimeen
- käyttää oletuksena `localhost:2000`
- hakee nykyisen CARLA-maailman
- tulostaa käytössä olevan kartan
- tulostaa maailmassa olevien actorien määrän

## `carla_vehicle.py`

Luo CARLA-simulaatioon virtuaalisen ajoneuvon, käynnistää autopilotin ja lukee ajoneuvon nopeutta.

## `carla_camera.py`

Lisää CARLA-ajoneuvoon RGB-etukameran.

```text
CARLA
  ↓
Virtuaalinen auto
  ↓
RGB-kamera
  ↓
Python callback
  ↓
NumPy
  ↓
OpenCV
  ↓
Live-kamerakuva
```

## `carla_path_prediction.py`

Yhdistää OpenCV-prototyypin CARLAn live-kameradataan.

```text
CARLA RGB Camera
        ↓
OpenCV
        ↓
Edge Detection
        ↓
Lane Detection
        ↓
Path Prediction
        ↓
Direction Estimate
```

Tässä vaiheessa CARLAn autopilot ohjaa edelleen ajoneuvoa. Path prediction toimii perception-prototyyppinä eikä vielä varsinaisena lateral controllerina.

## `carla_emergency_brake.py`

Projektin ensimmäinen yksinkertainen AEB-prototyyppi.

```text
Este kauempana kuin 8 m
        ↓
ajo jatkuu

Este enintään 8 m päässä
        ↓
throttle = 0
brake = 1.0
```

## `carla_aeb_test.py`

Ensimmäinen kontrolloitu AEB SIL -testi.

Ohjelma luo ego-ajoneuvon ja paikallaan olevan esteajoneuvon, mittaa AEB:n laukaisun ja pysähtymisen sekä tallentaa tuloksen CSV-tiedostoon.

Tulokset:

```text
results/aeb_results.csv
```

## `carla_aeb_batch_test.py`

Ensimmäinen automatisoitu baseline-testisarja.

Tässä versiossa otettiin käyttöön CARLAn synchronous mode:

```python
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
```

Tämä tarkoittaa:

```text
20 simulation steps / second
1 simulation tick = 0.05 s
```

Ensimmäinen batch-versio testasi eri throttle-arvoja. Tämä osoitti, että throttle ei ole hyvä tapa määrittää vertailukelpoisia testinopeuksia, minkä jälkeen siirryttiin kontrolloituihin tavoitenopeuksiin.

---

# `aeb_sil/` – erotettu SIL-arkkitehtuuri

Projektin tärkeä kehitysvaihe oli erottaa simulaattori ja ADAS-controller toisistaan.

Aikaisemmin:

```text
CARLA + sensorit + AEB-päätös + jarrutus
samassa Python-ohjelmassa
```

Nykyinen arkkitehtuuri:

```text
┌─────────────────────────────┐
│ CARLA simulator / test      │
│ speed + obstacle distance   │
└──────────────┬──────────────┘
               │ ZeroMQ
               ▼
┌─────────────────────────────┐
│ AEB controller              │
│ fixed-distance / TTC        │
└──────────────┬──────────────┘
               │ ZeroMQ
               ▼
┌─────────────────────────────┐
│ CARLA simulator / test      │
│ brake command               │
└─────────────────────────────┘
```

Controller ei importoi CARLAa. Se vastaanottaa vain ajoneuvon tilaa kuvaavaa dataa ja palauttaa ohjauskomennon.

Tämä tukee SIL → HIL -kehityspolkua.

## `aeb_sil/simulator.py`

Ensimmäinen erillinen CARLA-simulator-prosessi.

Se:

- yhdistää CARLAan
- luo ego-ajoneuvon ja esteen
- käyttää synchronous modea
- muodostaa virtuaalisen obstacle distance -mittauksen CARLAn ground truth -paikkatiedoista
- lähettää nopeuden ja etäisyyden ZeroMQ:lla controllerille
- vastaanottaa brake-komennon
- käyttää brake-komennon CARLA-ajoneuvoon
- tarkkailee collision sensoria

Virtuaalinen etäisyys lasketaan bumper-to-bumper-muodossa ajoneuvojen sijaintien ja bounding boxien avulla.

## `aeb_sil/controller.py`

Fixed-distance baseline-controller.

```text
distance > 10 m
→ brake = 0

distance <= 10 m
→ brake = 1.0
```

Controller ei sisällä CARLA-riippuvuutta.

## `aeb_sil/target_speed_test.py`

Kontrolloitu fixed-distance AEB -testisarja nopeuksille:

```text
20 km/h
30 km/h
40 km/h
50 km/h
```

Ennen AEB:n aktivoitumista test harness pitää ajoneuvon kontrolloidussa testinopeudessa `set_target_velocity()`-toiminnolla.

PASS-kriteeri:

```text
collision == False
JA
final_gap >= 1.0 m
```

Tulokset:

```text
results/aeb_sil_target_speed_results.csv
```

---

# Fixed-distance baseline -tulokset

| Tavoitenopeus | Trigger speed | Trigger gap | Pysähtymismatka | Loppuetäisyys | Collision | Result |
|---:|---:|---:|---:|---:|---|---|
| 20 km/h | 19.99 km/h | 9.95 m | 1.30 m | 8.65 m | Ei | PASS |
| 30 km/h | 29.35 km/h | 9.80 m | 5.20 m | 4.61 m | Ei | PASS |
| 40 km/h | 40.40 km/h | 9.66 m | 9.69 m | 0.00 m | Kyllä | FAIL |
| 50 km/h | 49.86 km/h | 9.81 m | 9.78 m | 0.02 m | Kyllä | FAIL |

Tulokset osoittavat, että kiinteä 10 m AEB-raja ei skaalaudu nopeuden mukana.

```text
20 km/h → PASS
30 km/h → PASS
40 km/h → FAIL
50 km/h → FAIL
```

---

# TTC-pohjainen AEB

Time To Collision:

```text
TTC = distance / relative_speed
```

AEB aktivoituu, kun:

```text
TTC <= threshold
```

## `aeb_sil/controller_ttc.py`

Ensimmäinen TTC-controller paikallaan olevalle esteelle.

Paikallaan olevan esteen tapauksessa:

```text
relative_speed = ego_speed
```

## `aeb_sil/target_speed_test_ttc.py`

Testaa TTC-controlleria samoilla nopeuksilla 20 / 30 / 40 / 50 km/h.

Tulokset:

```text
results/aeb_sil_ttc_results.csv
```

---

# TTC = 1.5 s -tulokset

| Nopeus | Trigger gap | Trigger TTC | Loppuetäisyys | Result |
|---:|---:|---:|---:|---|
| 20 km/h | 8.28 m | 1.491 s | 6.98 m | PASS |
| 30 km/h | 12.71 m | 1.494 s | 7.17 m | PASS |
| 40 km/h | 16.33 m | 1.455 s | 3.95 m | PASS |
| 50 km/h | 20.23 m | 1.453 s | 0.00 m | FAIL |

TTC 1.5 s paransi tulosta selvästi, mutta ei jättänyt riittävää pysähtymismarginaalia 50 km/h nopeudella.

---

# TTC = 1.7 s -tulokset

| Nopeus | Trigger speed | Trigger gap | Trigger TTC | Pysähtymismatka | Loppuetäisyys | Result |
|---:|---:|---:|---:|---:|---:|---|
| 20 km/h | 19.99 km/h | 9.39 m | 1.691 s | 1.30 m | 8.09 m | PASS |
| 30 km/h | 30.64 km/h | 14.38 m | 1.690 s | 5.54 m | 8.84 m | PASS |
| 40 km/h | 40.40 km/h | 18.55 m | 1.653 s | 12.38 m | 6.17 m | PASS |
| 50 km/h | 50.11 km/h | 23.00 m | 1.653 s | 20.93 m | 2.07 m | PASS |

TTC 1.7 s saavutti kaikki neljä tavoitetta ilman törmäystä ja vähintään 1 metrin turvamarginaalilla.

---

# Fixed-distance vs TTC

| Nopeus | Fixed 10 m | TTC 1.5 s | TTC 1.7 s |
|---:|---|---|---|
| 20 km/h | PASS | PASS | PASS |
| 30 km/h | PASS | PASS | PASS |
| 40 km/h | FAIL | PASS | PASS |
| 50 km/h | FAIL | FAIL | PASS |

Tulokset osoittavat, että nopeuteen sidottu TTC-päätös toimii tässä testiskenaariossa paremmin kuin kaikille nopeuksille sama 10 m etäisyysraja.

---

# Suhteelliseen nopeuteen perustuva TTC

Liikkuvaa estettä varten TTC lasketaan:

```text
relative_speed = ego_speed - obstacle_speed
TTC = distance / relative_speed
```

TTC lasketaan vain, jos ego todella lähestyy edellä olevaa ajoneuvoa.

## `aeb_sil/controller_ttc_relative.py`

Relative-speed TTC-controller vastaanottaa esimerkiksi:

```json
{
  "ego_speed_kmh": 50.0,
  "obstacle_speed_kmh": 30.0,
  "obstacle_distance_m": 10.0
}
```

ja laskee:

```text
relative speed = 50 - 30
               = 20 km/h
```

## `aeb_sil/moving_obstacle_test.py`

Ego-ajoneuvon tavoitenopeus:

```text
50 km/h
```

Esteajoneuvon tavoitenopeudet:

```text
0 km/h
20 km/h
30 km/h
40 km/h
```

Jokaiselle testille valitaan suhteelliseen nopeuteen perustuva sopiva lähtöetäisyys.

PASS-kriteeri:

```text
collision == False
JA
minimum_gap >= 1.0 m
```

Liikkuvan esteen testissä tärkein turvamittari on `minimum_gap`, koska este jatkaa liikkumista myös ego-ajoneuvon jarrutuksen aikana.

Tulokset:

```text
results/aeb_relative_ttc_results.csv
```

---

# Relative-speed TTC -tulokset

Validoidussa testiajossa:

| Ego | Este | Relative speed | Trigger gap | Trigger TTC | Minimum gap | Result |
|---:|---:|---:|---:|---:|---:|---|
| 50 km/h | 0 km/h | 50.02 km/h | 23.28 m | 1.675 s | 2.35 m | PASS |
| 50 km/h | 20 km/h | 30.93 km/h | 14.35 m | 1.670 s | 5.19 m | PASS |
| 50 km/h | 30 km/h | 20.51 km/h | 9.38 m | 1.647 s | 5.21 m | PASS |
| 50 km/h | 40 km/h | 10.28 km/h | 4.80 m | 1.679 s | 3.66 m | PASS |

Kaikki neljä moving-obstacle-skenaariota saivat tuloksen `PASS`.

Tulos havainnollistaa TTC:n perusidean:

```text
suuri relative speed
→ AEB reagoi kauempana

pieni relative speed
→ AEB voi reagoida lähempänä
```

Esimerkiksi:

```text
Ego 50 km/h, este 0 km/h
→ relative speed ≈ 50 km/h
→ trigger gap ≈ 23.3 m

Ego 50 km/h, este 40 km/h
→ relative speed ≈ 10 km/h
→ trigger gap ≈ 4.8 m
```

---

# Huomio moving-obstacle-testin kehityksestä

Ensimmäisessä moving-obstacle-testissä 40 km/h este ei pysynyt tavoitenopeudessa. Laukaisuhetkellä sen todellinen nopeus oli vain noin 10.6 km/h, joten testi ei vastannut suunniteltua skenaariota.

Tämä korjattiin muuttamalla testien alkuetäisyyttä suhteellisen nopeuden perusteella.

Korjatun testin 40 km/h tapauksessa:

```text
Ego speed:       49.86 km/h
Obstacle speed:  39.58 km/h
Relative speed:  10.28 km/h
Trigger gap:      4.80 m
Trigger TTC:      1.679 s
Minimum gap:      3.66 m
Result:           PASS
```

`aeb_relative_ttc_results.csv` voi sisältää myös aikaisempien kehitystestien rivejä, koska tulokset lisätään tiedoston loppuun. Lopulliseen analyysiin tulee käyttää validoitua uusinta testiajoa tai tehdä erillinen puhdas tulostiedosto.

---

# Synchronous mode ja toistettavuus

Testisarjoissa käytetään:

```python
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
```

Tämä tarkoittaa:

```text
1 simulation tick = 0.05 s
simulation frequency = 20 Hz
```

20 Hz näytteenotto vaikuttaa myös AEB:n todelliseen laukaisuhetkeen.

Esimerkiksi 50 km/h:

```text
50 km/h ≈ 13.89 m/s
13.89 m/s × 0.05 s ≈ 0.69 m / simulation tick
```

Tästä syystä TTC-controller ei yleensä laukea täsmälleen arvossa 1.700 s, vaan esimerkiksi noin 1.65–1.69 s välillä.

---

# Controller processing ja ZeroMQ latency

Erotetussa SIL-arkkitehtuurissa mitataan:

```text
controller_processing_ms
```

ja:

```text
controller_round_trip_ms
```

Tyypillisesti nykyisissä testeissä:

```text
controller processing ≈ 0.01–0.03 ms
ZeroMQ round trip     ≈ 0.2–0.4 ms
```

CARLAn simulation tick on:

```text
50 ms
```

Nykyisessä testissä Python-controllerin ja paikallisen ZeroMQ-kommunikaation viive on siis pieni verrattuna simulaation aikasteppiin.

Nämä arvot eivät kuitenkaan ole koko todellisen ajoneuvojärjestelmän sensor-to-actuator latency -mittauksia.

---

# Virtuaalinen sensoridata

Ensimmäisissä AEB-versioissa käytettiin CARLAn `sensor.other.obstacle` -sensoria.

Kontrolloiduissa SIL-testeissä obstacle distance muodostetaan tällä hetkellä CARLAn ground truth -tiedosta:

```text
ego transform
+
obstacle transform
+
vehicle bounding boxes
        ↓
longitudinal bumper-to-bumper gap
```

Tämän tarkoituksena on erottaa controller-logiikan validointi sensorin omasta epävarmuudesta.

Jatkossa virtuaalisen sensoridatan lähteeksi voidaan vaihtaa esimerkiksi:

```text
CARLA radar
CARLA LiDAR
kamera + perception
oikean ajoneuvon sensori
CAN-signaali
```

---

# OpenCV-perception

Projektin alkuperäinen osa käsittelee kamerakuvaa klassisilla konenäkömenetelmillä.

## Harmaasävymuunnos

RGB/BGR-värikuva muutetaan harmaasävyksi.

## Gaussian Blur

Nykyinen kernel:

```text
5 × 5
```

## Canny Edge Detection

Nykyiset raja-arvot:

```text
50
150
```

## Region of Interest

Käsittely rajataan kuvan alaosassa olevaan tiealueeseen.

## Hough Line Transform

Projektissa käytetään:

```python
cv2.HoughLinesP()
```

## Path Prediction

Kaistaviivojen perusteella arvioidaan kaistan keskikohta ja tavoiteltu ajolinja.

```text
angle < -3°
→ LEFT

-3° ... +3°
→ STRAIGHT

angle > +3°
→ RIGHT
```

Kyseessä on kamerageometriasta laskettu suunta-arvio, ei fyysinen ohjauspyörän kulma.

---

# Tulostiedostot

## `results/aeb_results.csv`

Yksittäisen AEB-prototyypin tuloksia.

## `results/aeb_batch_results.csv`

Ensimmäisen throttle-pohjaisen automatisoidun testisarjan tuloksia.

## `results/aeb_sil_target_speed_results.csv`

Fixed-distance controllerin kontrolloidut 20 / 30 / 40 / 50 km/h testit.

## `results/aeb_sil_ttc_results.csv`

TTC-controllerin target-speed-testit. Tiedosto voi sisältää sekä 1.5 s että 1.7 s TTC-testiajoja.

## `results/aeb_relative_ttc_results.csv`

Relative-speed TTC moving-obstacle -testitulokset.

---

# Käytetyt teknologiat

Projektissa käytetään:

- Python 3.12
- OpenCV
- NumPy
- CARLA Simulator
- CARLA Python API
- ZeroMQ / pyzmq
- Miniforge / Conda
- Git
- GitHub

---

# Python-ympäristö

Luo ympäristö:

```bash
conda env create -f environment.yml
```

Aktivoi:

```bash
conda activate edgepath
```

Tarkista CARLA:

```bash
python -c "import carla; print('CARLA Python API OK')"
```

Tarkista ZeroMQ:

```bash
python -c "import zmq; print(zmq.zmq_version())"
```

Tarvittaessa:

```bash
pip install carla==0.9.16
pip install pyzmq
```

---

# CARLAn käynnistäminen

CARLA-simulaattori täytyy käynnistää ennen testejä.

Python-client käyttää oletuksena:

```text
localhost:2000
```

---

# Suositeltu ohjelmien suoritusjärjestys

Perusteet:

```text
1. carla_connect.py
2. carla_vehicle.py
3. carla_camera.py
4. carla_path_prediction.py
5. carla_emergency_brake.py
6. carla_aeb_test.py
7. carla_aeb_batch_test.py
```

Erotettu SIL-arkkitehtuuri:

```text
8. aeb_sil/controller.py
   + aeb_sil/simulator.py

9. aeb_sil/controller.py
   + aeb_sil/target_speed_test.py

10. aeb_sil/controller_ttc.py
    + aeb_sil/target_speed_test_ttc.py

11. aeb_sil/controller_ttc_relative.py
    + aeb_sil/moving_obstacle_test.py
```

---

# Ajaminen – fixed-distance SIL

Terminaali 1:

```bash
python aeb_sil/controller.py
```

Terminaali 2:

```bash
python aeb_sil/target_speed_test.py
```

# Ajaminen – TTC SIL

Terminaali 1:

```bash
python aeb_sil/controller_ttc.py
```

Terminaali 2:

```bash
python aeb_sil/target_speed_test_ttc.py
```

# Ajaminen – relative-speed TTC

Terminaali 1:

```bash
python aeb_sil/controller_ttc_relative.py
```

Terminaali 2:

```bash
python aeb_sil/moving_obstacle_test.py
```

Vain yksi controller/testipari tulee ajaa kerrallaan, koska ne käyttävät samoja ZeroMQ-portteja:

```text
5555
5556
```

---

# Git-versionhallinta

Lähdekoodi, asetukset ja pienet testitulokset tallennetaan GitHubiin.

Versionhallintaan kuuluvat esimerkiksi:

```text
*.py
README.md
environment.yml
.gitignore
results/*.csv
```

Suuria videoita ei tallenneta Git-repositorioon:

```text
videos/*.mp4
output/*.mp4
```

---

# Nykyiset rajoitukset

## Path prediction

- perustuu suoriin Hough-viivoihin
- käyttää kiinteää ROI-aluetta
- voi toimia huonosti voimakkaissa mutkissa
- ei vielä ohjaa ajoneuvoa

## AEB / SIL

Nykyinen testijärjestelmä:

- käyttää täyttä `brake = 1.0` -hätäjarrutusta
- käyttää kontrolloiduissa testeissä CARLAn ground truth -etäisyyttä
- ei vielä sisällä sensorikohinaa tai havaintovirhettä
- ei vielä mallinna CAN-väylän viivettä
- ZeroMQ toimii tällä hetkellä samalla tietokoneella
- target-speed test harness käyttää `set_target_velocity()`-toimintoa
- moving-obstacle-testissä obstacle-nopeus pakotetaan test harnessilla
- testit on tehty tällä hetkellä CARLAn Town10HD_Opt-kartalla
- Ruskotunturin ympäristöä ei ole vielä tuotu CARLAan
- Jarcrac-ajoneuvoa ei ole vielä tuotu CARLAan
- HIL-vaihetta ei ole vielä toteutettu

Nykyiset tulokset ovat ennen kaikkea controller-logiikan SIL-validointia, eivät täydellinen malli oikean ajoneuvon pysähtymiskäyttäytymisestä.

---

# Seuraavat kehitysvaiheet

## 1. Testitulosten analyysi ja visualisointi

Tuloksista voidaan tehdä kuvaajat:

- trigger distance vs speed
- stopping distance vs speed
- minimum safety gap
- controller processing latency
- ZeroMQ round-trip latency

## 2. Realistisempi virtuaalisensori

Ground truth -etäisyyden tilalle voidaan kokeilla CARLA-radaria tai LiDARia.

## 3. Moving-obstacle-testien laajennus

Mahdollisia uusia skenaarioita:

```text
ego 30 / 40 / 50 km/h
obstacle 0 / 10 / 20 / 30 / 40 km/h
```

Lisäksi voidaan testata:

```text
obstacle decelerates
obstacle emergency brakes
```

## 4. Ruskotunturi CARLAan

NUVE-LAB / Ruskotunturi -materiaalissa olevia ympäristömeshejä voidaan myöhemmin käyttää CARLA-kartan rakentamisen lähtökohtana.

Kartta vaatii erillisen CARLA/Unreal-tuonnin ja tieverkon määrittelyn. Nykyisiä Mevea-malleja ei voi käyttää CARLAssa sellaisenaan.

## 5. Jarcrac-ajoneuvo

Jarcracin geometriaa ja Mevea-mallia voidaan käyttää myöhemmin CARLA-ajoneuvon lähtötietona.

Ajoneuvon tuonti vaatii erillisen CARLA/Unreal-yhteensopivan ajoneuvomallin ja fysiikka-asetukset.

## 6. HIL – Hardware-in-the-Loop

Tavoiteltu rakenne:

```text
CARLA
  ↓
Virtual sensor data
  ↓
Network / CAN interface
  ↓
Raspberry Pi
  ↓
ADAS controller
  ↓
Brake command
  ↓
CARLA / virtual actuator
```

Nykyinen ZeroMQ-erottelu tukee tätä kehityssuuntaa, koska controller ei ole suoraan sidottu CARLA Python API:in.

---

# Projektin kehityspolku

```text
OpenCV Proof of Concept
        ↓
CARLA Python API
        ↓
Virtual Vehicle
        ↓
RGB Camera
        ↓
Live Edge Detection
        ↓
Path Prediction
        ↓
AEB Prototype
        ↓
Controlled SIL Test
        ↓
Synchronous Batch Testing
        ↓
Target-Speed Baseline
        ↓
Simulator / Controller Separation
        ↓
ZeroMQ
        ↓
Fixed 10 m Baseline
        ↓
TTC 1.5 s
        ↓
TTC 1.7 s
        ↓
Relative-Speed TTC
        ↓
Moving Obstacle Tests
        ↓
Ruskotunturi / Sensor Development
        ↓
HIL Validation
```

---

# Projektin tavoiteltu lopputulos

Lopullisen projektin tavoitteena on muodostaa dokumentoitu ja toistettava ADAS-testauksen työnkulku, jossa:

1. ADAS-logiikka kehitetään erillisenä controllerina
2. controller testataan CARLA-simulaatiossa
3. testit suoritetaan automaattisesti ja deterministisesti
4. tulokset tallennetaan CSV-muodossa
5. controllerin toimintaa verrataan eri skenaarioissa
6. sensorin ja controllerin rajapinta pidetään mahdollisimman yleisenä
7. sama controller-ajatus voidaan myöhemmin siirtää sulautetulle laitteelle
8. SIL-vaiheesta voidaan edetä HIL-validointiin

Tavoitteena ei ole rakentaa täydellistä autonomista ajoneuvoa, vaan luoda vaiheittainen, mitattava ja helposti toistettava menetelmä ADAS-toimintojen kehittämiseen ja validointiin.
