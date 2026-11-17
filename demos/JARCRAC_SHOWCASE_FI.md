# CAMINO: kahden ajoneuvon yhteistyö Gazebo-simulaatiossa

**Esittelydemo 30.11.2026 — Software-in-the-Loop (SIL)**

## Lyhyesti: mitä demossa tapahtuu?

Demon tavoitteena on näyttää, miten kaksi autonomista työkoneen mallia voivat liikkua samalla työmaa-alueella, sovittaa ajonsa yhteen ja reagoida esteeseen. Simulaatio toteutetaan **Gazebo Harmonic** -ympäristössä Ruskotunturin maastomallilla.

- **Ajoneuvo A (JARCRAC)** on telaketjullisen maastoajoneuvon simulaatiomalli.
- **Ajoneuvo B** on toinen Gazebon ajoneuvomalli, joka käyttää Ackermann-tyyppistä ohjausta.
- **A lähtee pääreitiltä liittymän läheltä** ja etenee liittymän läpi ensin. **B lähtee sivuhaaran puolelta**, antaa A:lle tietä ja liittyy sen jälkeen samalle reitille A:n taakse. Ajoneuvot eivät siis aja toisiaan vastaan.
- B seuraa A:ta yhteisellä reitillä. Edessä oleva este saa A:n jarruttamaan ja B:n pysähtymään sen taakse.
- A pysähtyy, varmistaa että este ei liiku ja aloittaa sen kiertämisen. B voi edetä hitaasti turvalliseksi arvioituun odotuskohtaan, mutta aloittaa oman väistönsä vasta A:n vapautettua väistöalueen. Molemmat palaavat reitille ja pysähtyvät omille kuormauspaikoilleen.

Kyseessä on ohjelmiston ja simuloitujen ajoneuvojen **SIL-testi**, ei kahden oikean koneen kenttäkoe tai turvallisuussertifioitu autonominen järjestelmä.

## Mitä katsojan kannattaa seurata?

| Vaihe | Mitä ruudulla näkyy? | Miksi se on tärkeää? |
| --- | --- | --- |
| 1. Liittymä | A ajaa pääreitillä liittymän läpi ensin. B odottaa sivuhaaralla ja liittyy pääreitille A:n jälkeen. | Yhteensovitus ja kulkujärjestys |
| 2. Seuraaminen | B liikkuu A:n takana. | Ajoneuvojen välinen koordinointi |
| 3. Esteen havaitseminen | Ajoneuvot hidastavat, jarruttavat ja pysähtyvät esteen vuoksi. | Virtuaalisten anturien käyttö ja turvallinen pysähtyminen |
| 4. A:n väistö | A pysähtyy ja odottaa vähintään viisi simuloitua sekuntia ennen esteen kiertämistä. B pysyy poissa A:n väistöalueelta; A:n edetessä se voi siirtyä hitaasti lähemmäs omaa odotuskohtaansa. | Esteenväistö ja liikealueen varaaminen |
| 5. B:n väistö | Kun A on vapauttanut väistöalueen ja B:n oma pysähtymisjakso on täyttynyt, B kiertää esteen ja palaa reitille. | Turvalliseksi tarkistetun reitin suunnittelu ja uudelleen liittyminen |
| 6. Saapuminen | Molemmat ajoneuvot pysähtyvät omille merkityille kuormauspaikoilleen. | Tehtävän onnistumisen arviointi |

Yhden testiajon sisällä on myös pitkiä odotusjaksoja: ne kuuluvat demon toimintaan, eivät välttämättä tarkoita ohjelman jumiutumista. **Jos väistöreittiä ei voida varmistaa turvalliseksi, ohjain pysäyttää ajoneuvon.**

## Mitä antureita ja ohjausta käytetään?

- JARCRAC käyttää eteenpäin katsovaa noin **180° LiDAR** -mittausta.
- Ajoneuvo B käyttää **360° LiDAR** -mittausta.
- Python-ohjaimet käsittelevät simuloituja mittauksia sekä ajoneuvojen tilatietoja ja muodostavat nopeus- ja ohjauskomentoja.
- Ajoneuvojen keskinäinen järjestys ja väistöliikkeen käyttöoikeus sovitetaan yhteen: molemmat eivät saa ajaa samaan varattuun väistöalueeseen.
- Testissä paikannustiedon lähde on **Gazebon odometria**. Riippumattomassa arvioinnissa käytetään simulaation pose-tietoja, joten tulosta ei pidä sekoittaa tosimaailman GNSS-/paikannusvalidointiin.

Gazebon **Visualize Lidar** -näkymästä voi halutessaan valita aiheet `/camino/cooperative/a/lidar` ja `/camino/cooperative/b/lidar`. LiDAR-säteiden näkyminen on visualisointi; simuloitu anturi toimii myös ilman säteiden näyttämistä.

## Miten onnistuminen tarkistetaan?

Simulaatio tuottaa jokaiselta kierrokselta lokit ja erillisen arviointiraportin:

- `result.json`: testin lopputulos (`passed`), virhesyy, simulaatioaika ja kulunut todellinen aika.
- `evaluation.json`: esimerkiksi liittymisjärjestys, ajoneuvojen ja esteen väliset etäisyydet, reitille palaaminen sekä pysähtyminen tavoitteisiin.
- `control.jsonl`: ohjainten vaiheet, nopeuskomennot, pysähtymisen syyt ja väistövuorot.

Arvioinnissa tarkastetaan muun muassa, että A kulkee liittymäalueen läpi ennen B:tä, ajoneuvot eivät varaa liittymää yhtä aikaa, molempien väistöt ja reitille paluut toteutuvat, ajoneuvot saavuttavat pysähtyneinä tavoitteensa ja mitatuissa jalanjälkietäisyyksissä säilyy vähintään **1 metrin** vaadittu väli. Pysähtymis- ja reitillepaluuvaatimukset tarkistetaan myös mittausten perusteella.

### Esityskoneella mitatut testiajot (17.11.2026)

| Asetukset | Kierros | Todellinen ajoaika | Simulaatioaika | Tulos |
| --- | ---: | ---: | ---: | --- |
| 2 ms fysiikka-askel, tavoite-RTF 1,0 | 1 | 222,93 s | 168,09 s | PASS |
| 2 ms fysiikka-askel, tavoite-RTF 1,0 | 2 | 226,17 s | 170,06 s | PASS |
| **2 ms fysiikka-askel, tavoite-RTF 1,5** | **1** | **183,50 s** | **164,14 s** | **PASS** |
| **2 ms fysiikka-askel, tavoite-RTF 1,5** | **2** | **182,58 s** | **165,27 s** | **PASS** |
| 2 ms fysiikka-askel, tavoite-RTF 2,0 | 1 | 188,79 s | 172,08 s | FAIL |

**Esittelyyn valittu testattu asetus on fysiikka-askel 0,002 s ja tavoite-RTF 1,5.** Sen kaksi peräkkäistä kierrosta onnistuivat noin **3 minuutissa 3 sekunnissa per kierros** (todellinen aika). Kahden onnistuneen kierroksen perusteella ei kuitenkaan voi luvata täydellistä toimintavarmuutta.

RTF (**real-time factor**) vertaa simulaatioaikaa ja todellista aikaa. Tavoite-RTF 1,5 ei tarkoita, että simulaatio olisi käytännössä 1,5 kertaa reaaliaikaa nopeampi: laskentateho, anturisimulointi, grafiikka ja käynnistyskulut rajoittavat nopeutta.

Nopeammassa RTF 2,0 -kokeessa B:n reitille paluu ei varmistunut ja se pysähtyi lopulta syyllä `no_safe_detour`. Testi päättyi simulaatioajan aikarajaan. Mittauksissa ei raportoitu törmäys- tai käytävärajojen rikkomista, mutta **tehtävä jäi kesken**. Siksi tätä asetusta ei valittu esittelyyn.

## Demon käynnistys esityspisteellä

Aja WSL Ubuntu -terminaalissa projektin juuresta:

```bash
cd ~/camino-integrated-showcase
python3 -u edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py loop \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900 \
  --out /tmp/camino-exhibition-live
```

Komento toistaa `showcase`-skenaariota peräkkäin. **Onnistuneen kierroksen jälkeen Gazebo suljetaan ja käynnistetään uudelleen**; kyseessä ei siis ole saumaton videolooppi. Keskeytä `Ctrl+C`:llä. Silmukka pysähtyy myös epäonnistuneeseen kierrokseen. Käytä jokaisessa erillisessä käynnistyksessä uutta `--out`-hakemistoa.

Käynnistyskomento edellyttää projektin Gazebo-riippuvuuksia ja showcase-skriptejä. Pelkkä tämä Markdown-tiedosto ei sisällä simulaation toteutusta.

## Muuta

Tässä käytetään kahden ajoneuvon virtuaalisia malleja ja ohjelmallisia ohjaimia Gazebossa. Menetelmän tavoite on kokeilla käyttäytymistä ennen oikeiden laitteiden käyttöä.

Ajoneuvot jakavat kapean reitin ja esteen ohitusalueen. Järjestelmä sallii väistön yhdelle ajoneuvolle kerrallaan ja varmistaa pysähtymisen sekä etenemistilan ennen seuraavaa vaihetta.

Ohjain ei perusta päätöstä vain näytöllä näkyvään vapaaseen tilaan. Jos anturimittaus, ajoura tai paluu reitille ei ole riittävän varmasti tarkistettu, ajoneuvo jarruttaa ja odottaa.

*Software-in-the-Loop*: ohjausohjelmistoa testataan simulaatiossa virtuaalisilla ajoneuvoilla, antureilla ja ympäristöllä.

PASS tarkoittaa, että tämän skenaarion määritetyt simulaatiokriteerit täyttyivät kyseisellä ajokerralla. Todellisen koneen käyttö vaatii erillistä validointia ja turvallisuusarviointia.
  
RTF 2,0 -kokeessa B:n väistö jäi kesken. Esitykseen valittiin nopeampi **kahdesti onnistunut** RTF 1,5 -asetus. Lisäksi fysiikka-askeleen kasvattaminen vaikuttaisi simuloinnin tarkkuuteen ja pitäisi testata erikseen.

## Keskeiset projektitiedostot

- [`jarcrac_convoy_trial.py`](../edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py) — demon käynnistys, kierrosten hallinta ja raportointi.
- [`jarcrac_showcase_mission.py`](../edge-path-project/gazebo/scripts/jarcrac_showcase_mission.py) — ajoneuvojen yhteistyö ja esittelytehtävä.
- [`jarcrac_showcase_scene.py`](../edge-path-project/gazebo/scripts/jarcrac_showcase_scene.py) — esittelyalueen näkymä ja merkinnät.
- [`jarcrac_convoy_v2.py`](../edge-path-project/gazebo/scripts/jarcrac_convoy_v2.py) — väistö- ja ajosuunnittelun logiikka.
- [`jarcrac_convoy_score.py`](../edge-path-project/gazebo/scripts/jarcrac_convoy_score.py) — mittauksiin perustuva arviointi.
- [`JARCRAC_SHOWCASE_SIL.md`](../edge-path-project/gazebo/docs/JARCRAC_SHOWCASE_SIL.md) — teknisempi kuvaus ja testausohjeet.

Lisätietoja: [CAMINO-projektin pää-README](../README.md) sekä [Gazebo-dokumentaatio](../edge-path-project/gazebo/README.md).
