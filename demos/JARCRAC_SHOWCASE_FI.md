# CAMINO: kahden ajoneuvon yhteistyö Gazebo-simulaatiossa

**Esittelydemo 30.11.2026 — Software-in-the-Loop (SIL)**

## Lyhyesti: mitä demossa tapahtuu?

Demon tavoitteena on näyttää, miten kaksi autonomista työkoneen mallia voivat liikkua samalla työmaa-alueella, sovittaa ajonsa yhteen ja reagoida esteeseen. Simulaatio toteutetaan **Gazebo Harmonic** -ympäristössä Ruskotunturin maastomallilla.

- **Ajoneuvo A (JARCRAC)** on telaketjullisen maastoajoneuvon simulaatiomalli.
- **Ajoneuvo B** on toinen Gazebon ajoneuvomalli, joka käyttää Ackermann-tyyppistä ohjausta.
- Ajoneuvot lähtevät eri suunnista, kohtaavat liittymässä ja jatkavat yhteisellä reitillä.
- B väistää A:ta liittymässä ja seuraa tätä. Reitillä oleva este aiheuttaa jarrutuksen ja odottamisen.
- Ajoneuvot kiertävät esteen **vuorotellen**, palaavat suunnitellulle reitille ja pysähtyvät omille kuormauspaikoilleen.

Kyseessä on ohjelmiston ja simuloitujen ajoneuvojen **SIL-testi**, ei kahden oikean koneen kenttäkoe tai turvallisuussertifioitu autonominen järjestelmä.

## Mitä katsojan kannattaa seurata?

| Vaihe | Mitä ruudulla näkyy? | Miksi se on tärkeää? |
| --- | --- | --- |
| 1. Liittymä | A etenee ensin, B antaa tietä ja liittyy sen jälkeen samalle reitille. | Yhteensovitus ja kulkujärjestys |
| 2. Seuraaminen | B liikkuu A:n takana. | Ajoneuvojen välinen koordinointi |
| 3. Esteen havaitseminen | Ajoneuvot hidastavat, jarruttavat ja pysähtyvät esteen vuoksi. | Virtuaalisten anturien käyttö ja turvallinen pysähtyminen |
| 4. A:n väistö | A odottaa pysähtyneenä ja kiertää esteen. B odottaa sille varatun liikealueen vapautumista. | Esteenväistö ja liikealueen varaaminen |
| 5. B:n väistö | B tekee oman väistönsä ja palaa reitille. | Turvalliseksi tarkistetun reitin suunnittelu ja uudelleen liittyminen |
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

Arvioinnissa tarkastetaan muun muassa, että A liittyy ensin, ajoneuvot eivät varaa liittymää yhtä aikaa, molempien väistöt ja reitille paluut toteutuvat, ajoneuvot saavuttavat pysähtyneinä tavoitteensa ja mitatuissa jalanjälkietäisyyksissä säilyy vähintään **1 metrin** vaadittu väli. Pysähtymis- ja reitillepaluuvaatimukset tarkistetaan myös mittausten perusteella.

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

Ennen esittelyä kannattaa tehdä vielä muutaman peräkkäisen kierroksen harjoitus **samalla tietokoneella ja samoilla GUI-asetuksilla**. Varmuuskopioksi kannattaa tallentaa onnistuneesta kierroksesta video.

Tämä käynnistyskomento edellyttää, että kokeellisen showcase-version skriptit ja niiden riippuvuudet on lisätty paikalliseen työpuuhun. Pelkkä tämä Markdown-tiedosto ei sisällä simulaation toteutusta.

## Usein kysytyt kysymykset

**Onko tämä oikeasti kaksi autonomista konetta?**  
Ei. Tässä käytetään kahden ajoneuvon virtuaalisia malleja ja ohjelmallisia ohjaimia Gazebossa. Menetelmän tavoite on kokeilla käyttäytymistä ennen oikeiden laitteiden käyttöä.

**Miksi toinen ajoneuvo odottaa niin kauan?**  
Ajoneuvot jakavat kapean reitin ja esteen ohitusalueen. Järjestelmä sallii väistön yhdelle ajoneuvolle kerrallaan ja varmistaa pysähtymisen sekä etenemistilan ennen seuraavaa vaihetta.

**Miksi ajoneuvo pysähtyy, vaikka se näyttäisi voivan ajaa eteenpäin?**  
Ohjain ei perusta päätöstä vain näytöllä näkyvään vapaaseen tilaan. Jos anturimittaus, ajoura tai paluu reitille ei ole riittävän varmasti tarkistettu, ajoneuvo jarruttaa ja odottaa.

**Mitä SIL tarkoittaa?**  
*Software-in-the-Loop*: ohjausohjelmistoa testataan simulaatiossa virtuaalisilla ajoneuvoilla, antureilla ja ympäristöllä.

**Onko Raspberry Pi / CAN / HIL mukana tässä demossa?**  
Ei tässä nimenomaisessa showcase-ajossa. Kurssin laajempi tavoite kattaa myös SIL–HIL-siirtymän, sulautetun ohjaimen ja CAN-signaalit. Projektin muut HIL-testit dokumentoidaan erikseen; tätä SIL-demoa ei pidä esittää HIL-todisteena.

**Todistaako PASS, että ratkaisu on turvallinen oikealla työmaalla?**  
Ei. PASS tarkoittaa, että tämän skenaarion määritetyt simulaatiokriteerit täyttyivät kyseisellä ajokerralla. Todellisen koneen käyttö vaatii erillistä validointia ja turvallisuusarviointia.

**Miksi ei ajeta suuremmalla nopeudella?**  
RTF 2,0 -kokeessa B:n väistö jäi kesken. Esitykseen valittiin nopeampi **kahdesti onnistunut** RTF 1,5 -asetus. Lisäksi fysiikka-askeleen kasvattaminen vaikuttaisi simuloinnin tarkkuuteen ja pitäisi testata erikseen.

## Keskeiset projektitiedostot

- [`jarcrac_convoy_trial.py`](../edge-path-project/gazebo/scripts/jarcrac_convoy_trial.py) — demon käynnistys, kierrosten hallinta ja raportointi.
- [`jarcrac_showcase_mission.py`](../edge-path-project/gazebo/scripts/jarcrac_showcase_mission.py) — ajoneuvojen yhteistyö ja esittelytehtävä.
- [`jarcrac_showcase_scene.py`](../edge-path-project/gazebo/scripts/jarcrac_showcase_scene.py) — esittelyalueen näkymä ja merkinnät.
- [`jarcrac_convoy_v2.py`](../edge-path-project/gazebo/scripts/jarcrac_convoy_v2.py) — väistö- ja ajosuunnittelun logiikka.
- [`jarcrac_convoy_score.py`](../edge-path-project/gazebo/scripts/jarcrac_convoy_score.py) — mittauksiin perustuva arviointi.
- [`JARCRAC_SHOWCASE_SIL.md`](../edge-path-project/gazebo/docs/JARCRAC_SHOWCASE_SIL.md) — teknisempi kuvaus ja testausohjeet.

Lisätietoja: [CAMINO-projektin pää-README](../README.md) sekä [Gazebo-dokumentaatio](../edge-path-project/gazebo/README.md).