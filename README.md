# Edge Detection + Path Prediction

Konenäköprojekti ajo- ja peruutuskameran kuvan analysointiin.

Projektin tavoitteena on tunnistaa kamerakuvasta ajettavan reitin tai kaistan reunat ja arvioida niiden perusteella ajoneuvon tavoiteltu ajosuunta.

Projektissa käytetään Pythonia ja OpenCV:tä. Python-ympäristöä hallitaan Miniforgella ja projektin versionhallintaan käytetään Gitiä.

---

## Projektin tavoite

Projektissa toteutetaan kamerakuvaa hyödyntävä järjestelmä, joka:

1. lukee ajoneuvon kamerakuvaa tai videotallennetta
2. tunnistaa kuvasta reunoja
3. rajaa käsiteltäväksi vain tien kannalta kiinnostavan alueen
4. tunnistaa vasemman ja oikean kaista- tai reunaviivan
5. arvioi niiden perusteella ajettavan reitin keskikohdan
6. muodostaa ennustetun ajolinjan
7. arvioi tarvittavan ohjaussuunnan ja ohjauskulman
8. visualisoi tulokset videon päälle
9. tallentaa käsitellyn videon myöhempää tarkastelua varten

Projektin myöhemmissä vaiheissa tarkoituksena on käyttää simulaatioympäristöstä kerättyä kameradataa, suorittaa erilaisia simulaatioajoja sekä mitata ja optimoida toteutuksen suorituskykyä.

---

# Version 0.1 – Proof of Concept

Version 0.1 tarkoituksena on toimia ensimmäisenä toimivana prototyyppinä.

Tässä vaiheessa järjestelmää testataan valmiilla ajovideolla. Tarkoituksena on ensin varmistaa, että käytetyt konenäkömenetelmät toimivat ennen niiden liittämistä simulaatioympäristöön.

Nykyinen käsittelyketju on:

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
Ajolinjan arviointi
    ↓
Ohjauskulman laskenta
    ↓
LEFT / STRAIGHT / RIGHT
    ↓
Tuloksen visualisointi ja tallennus
```

---

# Toteutetut ominaisuudet

## 1. Videon lukeminen OpenCV:llä

Ohjelma käyttää OpenCV:n `VideoCapture`-toimintoa videon avaamiseen.

Video käsitellään yksi kuva eli frame kerrallaan.

Tämä mahdollistaa saman algoritmin käyttämisen myöhemmin myös live-kamerakuvan tai simulaatiosta saatavan videovirran kanssa.

---

## 2. Harmaasävymuunnos

Alkuperäinen värikuva muutetaan harmaasävykuvaksi OpenCV:n `cvtColor`-toiminnolla.

Värikuva sisältää kolme värikanavaa:

- sininen
- vihreä
- punainen

Reunantunnistuksessa kaikkia värikanavia ei tarvita, joten harmaasävy vähentää käsiteltävän datan määrää.

Käsittely:

```text
Värikuva
   ↓
Harmaasävykuva
```

---

## 3. Gaussian Blur

Harmaasävykuvaa pehmennetään Gaussian Blur -suodatuksella.

Pehmennyksen tarkoituksena on vähentää:

- kuvakohinaa
- pieniä yksityiskohtia
- epäolennaisia reunoja

Tämä auttaa reunantunnistusta löytämään tärkeämmät ja selkeämmät rakenteet kuvasta.

Projektissa käytetään tällä hetkellä 5 × 5 -kokoista Gaussian-suodatinta.

---

## 4. Canny Edge Detection

Canny Edge Detection -algoritmilla etsitään kuvasta kohtia, joissa kuvan kirkkaus muuttuu nopeasti.

Tällaisia kohtia ovat esimerkiksi:

- kaistaviivojen reunat
- tien reunat
- ajoneuvojen reunat
- kaiteet
- rakennusten reunat

Tuloksena syntyy mustavalkoinen kuva, jossa tunnistetut reunat näkyvät valkoisina viivoina.

```text
Alkuperäinen kuva

      tie
   \       /
    \     /
     \   /

        ↓

Edge Detection

   \       /
    \     /
     \   /
```

Nykyiset Canny-raja-arvot ovat:

```text
50 ja 150
```

Näitä voidaan myöhemmin säätää eri ympäristöihin sopiviksi.

---

## 5. Region of Interest (ROI)

Edge Detection löytää myös paljon reunoja, jotka eivät ole ajamisen kannalta kiinnostavia.

Tällaisia voivat olla esimerkiksi:

- taivas
- puut
- rakennukset
- vastaantulevat autot
- liikennemerkit

Tämän vuoksi kuvasta rajataan vain kiinnostava alue eli Region of Interest.

Tässä projektissa ROI on kuvan alaosassa oleva trapetsin muotoinen alue, jossa ajettava tie oletetaan olevan.

```text
+-----------------------+
|                       |
|                       |
|          /\           |
|         /  \          |
|        /    \         |
|       /      \        |
|______/________\_______|
```

Kaikki tämän alueen ulkopuolella olevat reunat jätetään käsittelemättä.

---

## 6. Hough Line Transform

Canny Edge Detection tuottaa yksittäisiä reunapisteitä.

Hough Line Transform yrittää muodostaa näistä pisteistä suoria viivoja.

Tämän avulla voidaan löytää esimerkiksi tien tai kaistamerkintöjen suuntaisia viivoja.

Projektissa käytetään OpenCV:n:

```python
cv2.HoughLinesP()
```

-toimintoa.

Se palauttaa joukon lyhyitä viivasegmenttejä.

---

## 7. Vasemman ja oikean kaistaviivan arviointi

Hough Transform löytää yleensä useita pieniä viivasegmenttejä.

Ohjelma jakaa nämä kahteen ryhmään niiden kulmakertoimen perusteella:

```text
negatiivinen kulmakerroin
        ↓
vasen kaistaviiva

positiivinen kulmakerroin
        ↓
oikea kaistaviiva
```

Lähes vaakasuorat viivat jätetään huomiotta.

Vasemman puolen viivoista lasketaan keskimääräinen vasen kaistaviiva ja oikean puolen viivoista keskimääräinen oikea kaistaviiva.

Tuloksena saadaan kaksi yhtenäistä viivaa:

```text
\             /
 \           /
  \         /
   \       /
```

Nämä piirretään videolle vihreällä.

---

## 8. Ajolinjan arviointi

Kun vasen ja oikea kaistaviiva on tunnistettu, niiden välistä voidaan laskea kaistan keskikohta.

Kaistan keskikohtaa käytetään tavoitepisteenä.

Ohjelma olettaa kameran ja ajoneuvon sijaitsevan kuvan alareunan keskellä.

Näiden pisteiden välille piirretään ennustettu ajolinja.

```text
          tavoitepiste
               ●
              /
             /
            /
           /
          ●
      ajoneuvo
```

Ajolinja piirretään videolle punaisella.

Tämä on yksinkertainen geometriaan perustuva path prediction -menetelmä.

---

## 9. Ohjauskulman laskenta

Ohjelma vertaa tavoitepisteen sijaintia kameran keskikohtaan.

Lasketaan:

```text
dx = tavoitepisteen vaakasuuntainen ero

dy = tavoitepisteen pystysuuntainen etäisyys
```

Näiden perusteella arvioidaan ohjauskulma käyttäen `atan2`-funktiota.

Tuloksena saadaan kulma asteina.

Esimerkiksi:

```text
-8.4 astetta
```

tarkoittaa, että tavoitepiste sijaitsee vasemmalla.

```text
+6.2 astetta
```

tarkoittaa, että tavoitepiste sijaitsee oikealla.

Tämä ei vielä ole ajoneuvon fyysinen ohjauspyörän kulma, vaan kamerakuvan geometriasta laskettu suunta-arvio.

---

## 10. LEFT / STRAIGHT / RIGHT -luokittelu

Ohjauskulman perusteella ohjelma muodostaa yksinkertaisen ajosuuntaluokituksen.

Nykyiset rajat ovat:

```text
kulma < -3°
    → LEFT

-3° ... +3°
    → STRAIGHT

kulma > +3°
    → RIGHT
```

Videolla voidaan tämän vuoksi näyttää esimerkiksi:

```text
Direction: LEFT
Steering angle: -7.2 deg
```

tai:

```text
Direction: STRAIGHT
Steering angle: 1.4 deg
```

Raja-arvoja voidaan myöhemmin testata ja säätää simulaatioajojen perusteella.

---

## 11. Ohjauskulman vakautus

Yksittäisten videoframien välillä tunnistetut kaistaviivat voivat hieman vaihdella.

Tämä aiheuttaa sen, että laskettu ohjauskulma voi hypellä nopeasti esimerkiksi:

```text
2°
5°
-1°
3°
```

Tämän vähentämiseksi ohjelma tallentaa viimeiset 10 ohjauskulmaa.

Niistä lasketaan keskiarvo:

```text
viimeiset 10 kulmaa
        ↓
keskiarvo
        ↓
vakautettu ohjauskulma
```

Tämä tekee videolla näkyvästä suunnasta ja ohjauskulmasta tasaisemman.

---

## 12. Tulosten visualisointi

Tunnistetut tiedot piirretään alkuperäisen videon päälle.

Nykyisessä toteutuksessa:

```text
vihreä
→ tunnistetut kaistaviivat

punainen
→ ennustettu ajolinja

keltainen teksti
→ ajosuunta ja ohjauskulma
```

Näin algoritmin toimintaa voidaan tarkastella visuaalisesti.

---

## 13. Käsitellyn videon tallentaminen

Ohjelma tallentaa käsitellyn videon MP4-muodossa.

Nykyinen tulostiedosto on:

```text
output/path_prediction.mp4
```

Tallennettu video sisältää:

- alkuperäisen kamerakuvan
- tunnistetut kaistaviivat
- ennustetun ajolinjan
- ajosuunnan
- ohjauskulman

Tätä voidaan käyttää myöhemmin algoritmin toiminnan arviointiin ja projektin esittelyyn.

---

# Projektin rakenne

```text
edge-path-project/
│
├── main.py
├── README.md
├── environment.yml
├── .gitignore
│
├── videos/
│   └── test_video.mp4
│
├── output/
│   └── path_prediction.mp4
│
└── results/
```

## main.py

Sisältää nykyisen konenäkö- ja path prediction -toteutuksen.

## environment.yml

Sisältää projektissa käytettävän Python-version ja tarvittavat Python-kirjastot.

Tämän avulla sama ympäristö voidaan muodostaa toiselle tietokoneelle.

## videos/

Sisältää ohjelman käsittelemät testivideot.

Videot eivät kuulu Git-versionhallintaan niiden suuren tiedostokoon vuoksi.

## output/

Sisältää ohjelman tuottamat käsitellyt videot.

## results/

Tarkoitettu myöhemmin suorituskykymittausten, testitulosten ja muiden tulosten tallentamiseen.

---

# Käytetyt teknologiat

Projektissa käytetään tällä hetkellä:

- Python 3.12
- OpenCV
- NumPy
- Miniforge / Conda
- Git

Myöhemmissä vaiheissa mukaan tulee simulaatioympäristö, esimerkiksi:

- Mevea

tai mahdollisesti:

- CARLA

---

# Python-ympäristön asentaminen

Projektissa käytetään Miniforgea Python-ympäristön hallintaan.

Luo ympäristö tiedostosta:

```bash
conda env create -f environment.yml
```

Aktivoi ympäristö:

```bash
conda activate edgepath
```

Tarkista Python-versio:

```bash
python --version
```

---

# Ohjelman käynnistäminen

Lisää testivideo `videos`-kansioon.

Nykyinen `main.py` käyttää esimerkiksi tiedostoa:

```text
videos/solidWhiteRight.mp4
```

Käynnistä ohjelma:

```bash
python main.py
```

Ohjelma näyttää käsitellyn videon ruudulla.

Ohjelma voidaan lopettaa painamalla:

```text
Q
```

Käsitelty video tallennetaan:

```text
output/path_prediction.mp4
```

---

# Toistettavuus

Projektin yksi tavoite on, että toteutus voidaan suorittaa myös toisella tietokoneella ilman vaikeita riippuvuuksia.

Tarvittavat Python-riippuvuudet määritellään `environment.yml`-tiedostossa.

Uudella tietokoneella projektin pitäisi olla mahdollista ottaa käyttöön seuraavasti:

```bash
conda env create -f environment.yml
conda activate edgepath
python main.py
```

---

# Nykyisen version rajoitukset

Version 0.1 toteutus on tarkoituksella yksinkertainen ensimmäinen prototyyppi.

Nykyisiä rajoituksia ovat esimerkiksi:

- algoritmi olettaa tien reunojen tai kaistaviivojen olevan melko selkeitä
- algoritmi perustuu suoriin Hough-viivoihin
- voimakkaasti kaartuvat tiet voivat aiheuttaa ongelmia
- valaistusolosuhteiden muutoksia ei vielä käsitellä erityisesti
- varjot voivat aiheuttaa ylimääräisiä reunoja
- ROI on tällä hetkellä kiinteä
- path prediction perustuu yksinkertaiseen geometriaan
- todellista ajoneuvon ohjausmallia ei vielä käytetä
- algoritmia ei vielä testata simulaatiodatalla
- suorituskykyä ei vielä mitata systemaattisesti

Näitä kohtia voidaan käyttää myöhemmin kehitys- ja optimointikohteina.

---

# Seuraavat vaiheet

## Version 0.2 – Simulaatiodata

Seuraavaksi tarkoituksena on siirtyä valmiista testivideoista simulaatioympäristöön.

Tavoitteet:

- tutustua valittuun simulaatioympäristöön
- käynnistää simulaatioajo
- käyttää ajoneuvon ajo- tai peruutuskameraa
- tallentaa kameradataa
- käyttää simulaatiosta kerättyä videota nykyisen algoritmin syötteenä

---

## Version 0.3 – Algoritmin kehitys

Tavoitteet:

- testata useita erilaisia ajoja
- testata suoria ja mutkaisia osuuksia
- säätää ROI-aluetta
- säätää Canny-parametreja
- säätää Hough-parametreja
- parantaa kaistaviivojen vakautta
- parantaa path prediction -menetelmää

---

## Version 0.4 – Suorituskykymittaukset

Tavoitteet:

- mitata FPS
- mitata yhden framen käsittelyaika
- selvittää ohjelman hitaimmat vaiheet
- tallentaa mittaustulokset `results`-kansioon

Esimerkiksi:

```text
FPS: 42.5

Keskimääräinen käsittelyaika:
23.5 ms / frame
```

---

## Version 0.5 – Optimointi

Tavoitteet:

- vähentää käsittelyaikaa
- vertailla eri resoluutioita
- optimoida ROI:n kokoa
- optimoida kuvankäsittelyvaiheita
- vertailla suorituskykyä ennen ja jälkeen optimoinnin

Mahdollisesti voidaan myöhemmin tutkia myös GPU- tai NPU-kiihdytystä.

---

## Version 1.0 – Lopullinen toteutus

Lopullisen version tavoitteena on sisältää:

- simulaatioajo
- simulaatiosta kerätty kameradata
- edge detection
- reuna- tai kaistaviivojen tunnistus
- path prediction
- ohjaussuunnan arviointi
- suorituskykymittaukset
- testaus erilaisissa tilanteissa
- optimointi
- dokumentointi
- Git-versionhallinta
- toistettava Python-ympäristö

Mahdollisuuksien mukaan toteutus validoidaan myös sulautetulla järjestelmällä.

---

# Projektin alustava kokonaisuus

```text
OpenCV Proof of Concept
        ↓
Simulaatioympäristö
        ↓
Simulaatioajo
        ↓
Kameradatan kerääminen
        ↓
Edge Detection
        ↓
Reuna-/kaistaviivojen tunnistus
        ↓
Path Prediction
        ↓
Ohjaussuunnan arviointi
        ↓
Testiajot
        ↓
FPS ja käsittelyajan mittaus
        ↓
Optimointi
        ↓
Mahdollinen sulautetun järjestelmän validointi
```
