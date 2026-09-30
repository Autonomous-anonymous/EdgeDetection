# CARLA ADAS Testing – Edge Detection, Path Prediction and Automatic Emergency Braking

Projektissa kehitetään ja testataan autonomisen ajoneuvon ADAS-toimintoja
CARLA-simulaatioympäristössä.

Projektin ensimmäinen vaihe toteutetaan Software-in-the-Loop (SIL)
-testauksena. Python-ohjelmat kommunikoivat CARLA-simulaattorin kanssa,
lukevat virtuaalisten sensoreiden dataa ja toteuttavat yksinkertaisia
ADAS-toimintoja.

Projektissa on tällä hetkellä kaksi pääkokonaisuutta:

1. kameraperusteinen edge detection ja path prediction
2. Automatic Emergency Braking (AEB)

Projektin myöhemmässä vaiheessa tavoitteena on tarkastella controllerin
siirtämistä sulautetulle järjestelmälle ja siirtymistä
Hardware-in-the-Loop (HIL) -testaukseen.

---

# Projektin tavoite

Projektin tavoitteena on rakentaa helposti toistettava testausympäristö,
jossa autonomisen ajoneuvon toimintoja voidaan kehittää ja validoida
vaiheittain.

Nykyinen kokonaisuus:

```text
CARLA-simulaatio
        ↓
Virtuaalinen ajoneuvo
        ↓
Virtuaalisensorit
        ↓
Python-controllerit
        ↓
ADAS-toiminnot
        ↓
Automaattiset SIL-testit
        ↓
Tulosten tallennus
        ↓
Analysointi ja optimointi
        ↓
Mahdollinen HIL-validointi
```

---

# Projektin nykyinen tila

Projektissa on toteutettu seuraavat toiminnot:

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
- obstacle sensor
- collision sensor
- Automatic Emergency Braking
- automaattinen PASS/FAIL-testaus
- CSV-testitulosten tallennus
- CARLA synchronous mode
- kiinteä simulaatioaskel
- automatisoitu AEB-testisarja

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
    ├── videos/
    ├── output/
    │
    └── results/
        ├── aeb_results.csv
        └── aeb_batch_results.csv
```

---

# Tiedostojen tarkoitus

## `main.py`

Projektin ensimmäinen OpenCV Proof of Concept.

Ohjelma käsittelee valmista ajovideota ja suorittaa seuraavat vaiheet:

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

Tämän tiedoston tarkoituksena oli ensin testata konenäköalgoritmin
toimintaa ilman simulaatioympäristöä.

Ohjelma pystyy myös tallentamaan käsitellyn videon esimerkiksi:

```text
output/path_prediction.mp4
```

---

## `carla_connect.py`

Ensimmäinen CARLA-yhteystesti.

Tiedosto:

- muodostaa yhteyden käynnissä olevaan CARLA-palvelimeen
- käyttää oletuksena osoitetta `localhost`
- käyttää CARLAn oletusporttia `2000`
- hakee nykyisen CARLA-maailman
- tulostaa käytössä olevan kartan
- tulostaa maailmassa olevien actorien määrän

Tämän tiedoston tarkoituksena on varmistaa, että Python API ja
CARLA-simulaattori kommunikoivat oikein.

Käsittely:

```text
Python
   ↓
CARLA Client
   ↓
localhost:2000
   ↓
CARLA Server
```

---

## `carla_vehicle.py`

Luo CARLA-simulaatioon virtuaalisen ajoneuvon.

Tiedosto:

- muodostaa yhteyden CARLAan
- hakee saatavilla olevat ajoneuvot
- valitsee nelipyöräisen ajoneuvon
- etsii vapaan spawn-pisteen
- luo ajoneuvon simulaatioon
- käynnistää CARLAn autopilotin
- lukee ajoneuvon nopeuden
- näyttää nopeuden terminaalissa
- poistaa ajoneuvon ohjelman lopuksi

Tämän avulla saatiin ensimmäinen varsinainen simulaatioajo toimimaan.

---

## `carla_camera.py`

Lisää CARLA-ajoneuvoon virtuaalisen RGB-etukameran.

Tiedosto:

- luo Tesla Model 3 -ajoneuvon
- luo `sensor.camera.rgb`-sensorin
- kiinnittää kameran ajoneuvoon
- vastaanottaa kamerakuvat CARLAsta
- muuntaa CARLAn BGRA-kuvan OpenCV:n BGR-muotoon
- näyttää kamerakuvan reaaliajassa
- näyttää ajoneuvon nopeuden kamerakuvan päällä

Käsittelyketju:

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

Tämä osoittaa, että CARLAn virtuaalisensorista saatavaa dataa voidaan
käsitellä reaaliaikaisesti Pythonissa.

---

## `carla_path_prediction.py`

Yhdistää alkuperäisen OpenCV-prototyypin CARLAn live-kameradataan.

Tiedosto käyttää CARLAn RGB-etukameraa ja suorittaa kamerakuvalle:

- harmaasävymuunnoksen
- Gaussian Blur -suodatuksen
- Canny Edge Detectionin
- Region of Interest -rajauksen
- Hough Line Transformin
- vasemman ja oikean kaistaviivan arvioinnin
- ajolinjan arvioinnin
- ohjauskulman laskennan
- LEFT / STRAIGHT / RIGHT -luokittelun

Käsittely:

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

Kaistaviivat piirretään vihreällä ja arvioitu ajolinja punaisella.

Tässä vaiheessa CARLAn autopilot vastaa edelleen ajoneuvon varsinaisesta
ohjaamisesta. Path prediction toimii perception-prototyyppinä eikä
vielä ohjaa ajoneuvoa.

---

## `carla_emergency_brake.py`

Projektin ensimmäinen oma ADAS-controller.

Toteuttaa yksinkertaisen Automatic Emergency Braking -toiminnon.

Ajoneuvoon lisätään CARLAn:

```text
sensor.other.obstacle
```

Sensorin avulla mitataan ajoneuvon edessä olevan esteen etäisyyttä.

Ensimmäisen controllerin logiikka on:

```text
Este kauempana kuin 8 m
        ↓
Autopilot jatkaa ajoa

Este enintään 8 m päässä
        ↓
Autopilot pois päältä
        ↓
Throttle = 0
Brake = 1.0
        ↓
Täysi hätäjarrutus
```

Tämä oli ensimmäinen yksinkertainen collision avoidance /
emergency braking -prototyyppi.

---

## `carla_aeb_test.py`

Ensimmäinen kontrolloitu AEB SIL -testi.

Ohjelma luo simulaatioon kaksi ajoneuvoa:

```text
EGO VEHICLE  → → → → →      OBSTACLE VEHICLE
                               paikallaan
```

Tiedosto:

- etsii suoran tieosuuden
- luo ego-ajoneuvon
- luo pysähtyneen esteajoneuvon
- lisää obstacle sensorin
- lisää collision sensorin
- ajaa ego-ajoneuvoa kohti estettä
- aktivoi AEB:n määritetyllä etäisyydellä
- mittaa ajoneuvon pysähtymisen
- tarkistaa tapahtuiko törmäys
- muodostaa PASS/FAIL-tuloksen
- tallentaa tulokset CSV-tiedostoon

Mitattavia arvoja ovat:

- nopeus AEB:n aktivoituessa
- AEB:n laukaisuetäisyys
- Python-controllerin käsittelyviive
- pysähtymisaika
- pysähtymismatka
- törmäys / ei törmäystä

Tulokset tallennetaan:

```text
results/aeb_results.csv
```

---

## `carla_aeb_batch_test.py`

Projektin tämänhetkinen kehittynein AEB-testiohjelma.

Ohjelma suorittaa useita AEB-testejä automaattisesti.

Tärkeä ero aikaisempaan versioon on CARLAn synchronous mode.

Simulaatiossa käytetään:

```text
fixed_delta_seconds = 0.05 s
```

Tämä tarkoittaa:

```text
20 simulation steps / second
```

Jokainen `world.tick()` vie simulaatiota täsmälleen 0,05 sekuntia
eteenpäin.

Tämän tavoitteena on parantaa testien:

- toistettavuutta
- vertailukelpoisuutta
- ajallista determinismiä

Nykyinen testisarja käyttää:

```text
AEB trigger distance = 10 m
```

ja suorittaa viisi testiä eri throttle-arvoilla.

Testit:

```text
test_01 → throttle 0.25
test_02 → throttle 0.35
test_03 → throttle 0.45
test_04 → throttle 0.55
test_05 → throttle 0.65
```

Jokaisessa testissä tallennetaan:

- testin nimi
- throttle
- AEB:n määritetty laukaisuetäisyys
- todellinen laukaisuetäisyys
- nopeus laukaisuhetkellä
- controller processing latency
- pysähtymisaika
- pysähtymismatka
- loppuetäisyys esteeseen
- collision-event
- PASS/FAIL-tulos

Tulokset tallennetaan:

```text
results/aeb_batch_results.csv
```

---

# OpenCV-perception

Projektin alkuperäinen osa käsittelee kamerakuvaa klassisilla
konenäkömenetelmillä.

## Harmaasävymuunnos

RGB/BGR-värikuva muutetaan harmaasävyksi.

Tämä yksinkertaistaa reunantunnistusta ja vähentää käsiteltävän datan
määrää.

---

## Gaussian Blur

Kuvaa pehmennetään ennen reunantunnistusta.

Tarkoituksena on vähentää:

- kohinaa
- pieniä yksityiskohtia
- epäolennaisia reunoja

Nykyinen kernel-koko on:

```text
5 × 5
```

---

## Canny Edge Detection

Canny-algoritmi tunnistaa kuvasta voimakkaita kirkkausmuutoksia.

Nykyiset raja-arvot:

```text
50
150
```

Tuloksena saadaan mustavalkoinen reunakuva.

---

## Region of Interest

Koko kuvaa ei analysoida.

Käsittely rajataan kuvan alaosassa olevaan alueeseen, jossa tien
oletetaan olevan.

Näin esimerkiksi:

- taivas
- rakennukset
- puut
- muut epäolennaiset alueet

voidaan jättää analyysin ulkopuolelle.

---

## Hough Line Transform

Canny Edge Detection tuottaa reunapisteitä.

Hough Line Transform muodostaa näistä viivasegmenttejä.

Projektissa käytetään:

```python
cv2.HoughLinesP()
```

Tunnistetut viivat jaetaan vasemman ja oikean kaistan viivoihin
kulmakertoimen perusteella.

---

## Path Prediction

Vasemman ja oikean kaistaviivan perusteella lasketaan kaistan
keskikohta.

Kameran oletetaan sijaitsevan auton keskellä.

Näiden pisteiden perusteella muodostetaan arvioitu ajolinja.

```text
          target
            ●
           /
          /
         /
        ●
      vehicle
```

Tämän perusteella arvioidaan myös suunta:

```text
angle < -3°
→ LEFT

-3° ... +3°
→ STRAIGHT

angle > +3°
→ RIGHT
```

Tämä on geometrinen suunta-arvio eikä ajoneuvon fyysinen
ohjauspyörän kulma.

---

# Automatic Emergency Braking

Projektin toinen pääkokonaisuus on AEB.

Nykyinen baseline-controller käyttää kiinteää etäisyysrajaa:

```text
distance > 10 m
        ↓
normal driving

distance <= 10 m
        ↓
throttle = 0
brake = 1.0
```

Tämä toimii baseline-ratkaisuna, johon myöhempiä controller-versioita
voidaan verrata.

---

# Ensimmäinen automatisoitu AEB-testisarja

Ensimmäisessä synchronous mode -testisarjassa saatiin seuraavat tulokset:

| Testi | Nopeus AEB:n laukaisuhetkellä | Todellinen laukaisuetäisyys | Pysähtymismatka | Loppuetäisyys | Collision |
|---|---:|---:|---:|---:|---|
| test_01 | 15.93 km/h | 9.81 m | 2.08 m | 7.51 m | Ei |
| test_02 | 18.64 km/h | 9.91 m | 1.58 m | 8.07 m | Ei |
| test_03 | 25.53 km/h | 9.84 m | 2.86 m | 6.62 m | Ei |
| test_04 | 32.82 km/h | 9.70 m | 5.84 m | 3.41 m | Ei |
| test_05 | 39.83 km/h | 9.46 m | 9.04 m | 0.00 m | Ei |

Kaikki testit saivat nykyisellä collision-eventtiin perustuvalla
logiikalla tuloksen `PASS`.

Viides testi osoitti kuitenkin nykyisen PASS/FAIL-kriteerin
rajoituksen.

Vaikka CARLA ei ilmoittanut collision-eventtiä:

```text
final obstacle distance = 0.00 m
```

Turvallisuusmarginaalia ei käytännössä jäänyt.

Tämän vuoksi seuraavassa testiversiossa PASS-kriteeriin lisätään
vähimmäisturvallisuusetäisyys.

---

# Baseline-testien havaintoja

Kiinteä 10 metrin AEB-raja toimii eri tavalla eri nopeuksilla.

Pienellä nopeudella:

```text
10 m
→ suuri turvallisuusmarginaali
```

Suuremmalla nopeudella:

```text
10 m
→ pysähtymismatka kasvaa
→ turvallisuusmarginaali pienenee
```

Suurimmassa nykyisessä testissä:

```text
Trigger speed:
39.83 km/h

Stopping distance:
9.04 m

Final obstacle distance:
0.00 m
```

Tämä osoittaa, että pelkkä kiinteä etäisyysraja ei ole riittävä
ratkaisu kaikille ajonopeuksille.

---

# Synchronous Mode ja toistettavuus

CARLAn normaalissa asynchronous mode -tilassa simulaation eteneminen
voi riippua tietokoneen suorituskyvystä.

Testisarjassa käytetään tämän vuoksi synchronous modea:

```python
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
```

Tämän avulla:

```text
1 simulation tick = 0.05 s
```

Simulaation taajuus on:

```text
20 Hz
```

Tämä tekee testien ajallisesta etenemisestä paremmin toistettavaa.

---

# Controller processing latency

AEB-testissä mitataan myös Python-controllerin käsittelyviivettä.

Ensimmäisessä testisarjassa arvot olivat noin:

```text
6.8–7.2 ms
```

Tämä mittaus kuvaa sensorin callback-tapahtuman ja controllerin
päätöksenteon välistä Python-prosessin käsittelyaikaa.

Se ei tarkoita koko järjestelmän todellista sensor-to-brake-viivettä.

---

# Tulostiedostot

## `results/aeb_results.csv`

Sisältää yksittäisellä AEB-testiohjelmalla tuotettuja tuloksia.

---

## `results/aeb_batch_results.csv`

Sisältää automatisoidun testisarjan tulokset.

Tallennettavia kenttiä ovat esimerkiksi:

```text
timestamp
test_name
result
throttle
aeb_trigger_distance_m
actual_trigger_distance_m
speed_at_trigger_kmh
controller_processing_latency_ms
stopping_time_s
stopping_distance_m
final_obstacle_distance_m
collision
```

CSV-muoto mahdollistaa tulosten myöhemmän:

- analysoinnin
- visualisoinnin
- vertailun
- tilastollisen käsittelyn

---

# Käytetyt teknologiat

Projektissa käytetään tällä hetkellä:

- Python 3.12
- OpenCV
- NumPy
- CARLA Simulator
- CARLA Python API
- Miniforge / Conda
- Git
- GitHub

---

# Python-ympäristö

Projektissa käytetään Miniforgea Python-ympäristöjen hallintaan.

Luo ympäristö:

```bash
conda env create -f environment.yml
```

Aktivoi ympäristö:

```bash
conda activate edgepath
```

Tarkista Python:

```bash
python --version
```

CARLA Python API:n toiminnan voi tarkistaa:

```bash
python -c "import carla; print('CARLA Python API OK')"
```

Jos CARLA-pakettia ei ole ympäristössä:

```bash
pip install carla==0.9.16
```

---

# CARLAn käynnistäminen

CARLA-simulaattori täytyy käynnistää ennen Python-ohjelmia.

Windowsissa esimerkiksi:

```text
CarlaUE4.exe
```

Python-client yhdistää CARLAan:

```text
localhost:2000
```

---

# Suositeltu ohjelmien suoritusjärjestys

Projektin toimintaan voi tutustua vaiheittain:

```text
1. carla_connect.py
        ↓
2. carla_vehicle.py
        ↓
3. carla_camera.py
        ↓
4. carla_path_prediction.py
        ↓
5. carla_emergency_brake.py
        ↓
6. carla_aeb_test.py
        ↓
7. carla_aeb_batch_test.py
```

## 1. Testaa CARLA-yhteys

```bash
python carla_connect.py
```

## 2. Testaa virtuaaliauto

```bash
python carla_vehicle.py
```

## 3. Testaa RGB-kamera

```bash
python carla_camera.py
```

## 4. Testaa live path prediction

```bash
python carla_path_prediction.py
```

## 5. Testaa yksinkertainen emergency braking

```bash
python carla_emergency_brake.py
```

## 6. Suorita yksi kontrolloitu AEB-testi

```bash
python carla_aeb_test.py
```

## 7. Suorita automatisoitu AEB-testisarja

```bash
python carla_aeb_batch_test.py
```

---

# Git-versionhallinta

Projektin lähdekoodi, asetukset ja pienet testitulokset tallennetaan
GitHubiin.

Git-versionhallintaan kuuluvat esimerkiksi:

```text
*.py
README.md
environment.yml
.gitignore
results/*.csv
```

Suuria videoita ei tallenneta Git-repositorioon.

Esimerkiksi:

```text
videos/*.mp4
output/*.mp4
```

pidetään paikallisina tiedostoina.

---

# Nykyiset rajoitukset

Projektissa on vielä useita kehityskohteita.

## Path Prediction

Nykyinen algoritmi:

- perustuu suoriin Hough-viivoihin
- käyttää kiinteää ROI-aluetta
- voi toimia huonosti voimakkaissa mutkissa
- voi tulkita muita tien rakenteita kaistaviivoiksi
- ei vielä ohjaa ajoneuvoa

---

## AEB

Nykyinen baseline-controller:

- käyttää kiinteää 10 metrin etäisyysrajaa
- ei huomioi nopeutta päätöksenteossa
- ei vielä huomioi suhteellista nopeutta
- käyttää täyttä `brake = 1.0` -jarrutusta
- ei vielä käytä realistista turvallisuusmarginaalia PASS/FAIL-kriteerissä

---

## Testinopeudet

Nykyinen batch-testi käyttää throttle-arvoja:

```text
0.25
0.35
0.45
0.55
0.65
```

Throttle ei kuitenkaan vastaa suoraan tiettyä ajonopeutta.

Tämän vuoksi seuraavissa testeissä käytetään kontrolloituja
tavoitenopeuksia.

---

# Seuraavat kehitysvaiheet

## 1. Kontrolloidut tavoitenopeudet

Seuraava AEB-testiversio käyttää esimerkiksi:

```text
20 km/h
30 km/h
40 km/h
50 km/h
```

Tällöin testejä voidaan vertailla luotettavammin.

---

## 2. Parempi PASS/FAIL-kriteeri

Pelkkä collision-event ei riitä.

Testiin lisätään vähimmäisturvallisuusetäisyys.

Esimerkiksi:

```text
collision == False

JA

final_distance >= safety_margin
```

---

## 3. TTC-pohjainen AEB

Nykyinen:

```text
distance <= 10 m
→ brake
```

korvataan myöhemmin dynaamisemmalla päätöksellä.

Time To Collision:

```text
TTC = distance / relative_speed
```

Tällöin AEB ottaa huomioon sekä etäisyyden että lähestymisnopeuden.

Tavoitteena on verrata:

```text
Fixed-distance AEB
        VS
TTC-based AEB
```

---

## 4. Suorituskykymittaukset

Myöhemmissä vaiheissa voidaan mitata esimerkiksi:

- Python-controllerin käsittelyaikaa
- sensorien päivitystaajuutta
- FPS-arvoa
- OpenCV-kuvankäsittelyn aikaa
- CARLA-clientin kuormitusta

---

## 5. HIL – Hardware-in-the-Loop

Projektin myöhemmässä vaiheessa controller voidaan siirtää
sulautetulle järjestelmälle.

Mahdollinen kokonaisuus:

```text
CARLA
  ↓
Virtual sensors
  ↓
CAN / communication interface
  ↓
Raspberry Pi
  ↓
ADAS Controller
  ↓
Brake command
  ↓
CARLA
```

Tavoitteena on käyttää mahdollisimman pitkälle samaa controller-logiikkaa
sekä SIL- että HIL-vaiheessa.

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
Obstacle Sensor
        ↓
Collision Sensor
        ↓
AEB Controller
        ↓
Controlled SIL Test
        ↓
Synchronous Batch Testing
        ↓
Target-Speed Tests
        ↓
TTC-Based AEB
        ↓
Performance Analysis
        ↓
HIL Validation
```

---

# Projektin tavoiteltu lopputulos

Lopullisen projektin tavoitteena on muodostaa dokumentoitu ja
toistettava ADAS-testauksen työnkulku, jossa:

1. ADAS-logiikka kehitetään Pythonilla
2. controller testataan CARLA-simulaatiossa
3. testit suoritetaan automaattisesti
4. tulokset tallennetaan ja analysoidaan
5. controlleria kehitetään tulosten perusteella
6. samaa logiikkaa voidaan myöhemmin testata sulautetulla laitteella

Tavoitteena ei ole rakentaa täydellistä autonomista ajoneuvoa, vaan
luoda vaiheittainen, mitattava ja helposti toistettava menetelmä
ADAS-toimintojen kehittämiseen ja validointiin.