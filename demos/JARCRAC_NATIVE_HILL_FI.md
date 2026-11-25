# CAMINO: JARCRACin nousu Ruskotunturin luonnollisella rinteellä

**Kolmas esittelydemo 30.11.2026 — Software-in-the-Loop (SIL)**

Tämä demo täydentää kahta muuta CAMINO-esittelyä: [kahden ajoneuvon yhteistyödemoa](JARCRAC_SHOWCASE_FI.md) sekä [loivemman, kaartuvan summit-reitin demoa](JARCRAC_SUMMIT_FI.md). Tässä kolmannessa kokeessa **yksi telaketjullinen JARCRAC nousee alkuperäistä, muokkaamatonta Ruskotunturin rinnettä**, väistää kannon ja pysähtyy ylempänä olevaan tavoitepaikkaan.

**Huomautus:** Kyseessä on Gazebo Harmonic -ympäristössä ajettava ohjelmisto-simulaatio, ei oikean koneen rinnekoe eikä osoitus fyysisen JARCRACin turvallisista kaltevuusrajoista.

## Lyhyesti: mitä demossa tapahtuu?

- Yksityiskohtaisen telamallin JARCRAC aloittaa Ruskotunturin rinteen alaosasta. Ajoneuvo pysyy lyhyesti paikallaan, kun simuloitu alusta asettuu maastoon.
- Ajoneuvo seuraa noin **22,44 m** pitkää, merkittyä nousureittiä. Punaiset reittimerkit helpottavat etenemisen seuraamista.
- Reitin varrella on puunkantoa kuvaava sylinteri (**halkaisija 0,64 m, korkeus 0,58 m**).
- Ajoneuvo havaitsee kannon eteenpäin katsovalla LiDAR-anturilla, jarruttaa ja odottaa vähintään **viisi simuloitua sekuntia**.
- Sen jälkeen JARCRAC kiertää kannon, palaa nousureitille ja jatkaa ylämäkeen. Ylemmässä osassa ohjain voi kulkea hieman merkatun linjan sivussa jyrkän maastokohdan välttämiseksi.
- Ajoneuvo saavuttaa vihreällä renkaalla merkityn tavoitteen ja pysyy paikallaan vähintään **kolme simuloitua sekuntia**.

**Maasto on alkuperäinen Ruskotunturin malli:** tämä versio ei lisää maastoon keinotekoista mäkeä, tasoitettua ajouraa tai uutta maaston törmäysverkkoa. Kanto ja värilliset reittimerkit ovat demoalueen erillisiä lisäyksiä.

## Mitä katsojan kannattaa seurata?

| Vaihe | Mitä näkyy Gazebossa? | Miksi se on tärkeää? |
| --- | --- | --- |
| 1. Lähtö | JARCRAC asettuu rinteeseen ja aloittaa nousun. | Tela-alustan ja maaston yhteistoiminta |
| 2. Kannon havaitseminen | Ajoneuvo jarruttaa ennen estealuetta. | LiDAR-havaintojen ja nopeusohjauksen hyödyntäminen |
| 3. Pysähtyminen | JARCRAC odottaa paikallaan vähintään viisi sekuntia. | Hallittu pysähdys ja odotusehto |
| 4. Esteen kierto | Ajoneuvo kiertää kannon rinteen poikki. | Esteen, kallistuman ja ajouran huomioon ottaminen |
| 5. Reitille paluu | JARCRAC palaa nousulinjan suuntaan ja jatkaa eteenpäin. | Kaarresäteen, havaintojen ja reittipoikkeaman hallinta |
| 6. Nousun loppu | Ajoneuvo nousee ylemmälle rinteelle ja pysähtyy vihreän tavoiterenkaan sisälle. | Tavoitteen saavuttaminen ja mitattu paikallaan pysyminen |

Pitkät pysähtymiset voivat kuulua esteenväistön logiikkaan. Sen sijaan **pysyvä pysähdys, jonka jälkeen ajoneuvo ei saavuta tavoitetta, ei ole onnistunut kierros**.

## Mitä antureita ja ohjausta käytetään?

- JARCRACin virtuaalinen noin **180° eteenpäin katsova LiDAR** tuottaa havaintoja esteistä ja ajettavasta tilasta.
- Python-ohjain käyttää Gazebon **odometriaa**, LiDAR-havaintoja sekä alkuperäisestä maastoverkosta johdettua korkeusmallia.
- Reittiehdokkaille tehdään ajoneuvon jalanjäljen ja paikallisen maastoprofiilin tarkistuksia. Maastoa arvioidaan **5 × 5 näytepisteen** alueella ajoneuvon alla.
- Ohjain tarkastaa myös väistöliikkeen kaarevuutta, pysähtymistilaa sekä nykyisiä ja aikaisemmin havaittuja vapaita alueita. Mittaamattoman alueen turvallisuutta ei ole tarkoitus olettaa.
- Jyrkällä osuudella ajonopeutta pienennetään, ja ylempi reittiosuus kiertää lyhyen maastoharjanteen lievästi sivusta.
- Ohjaus ja arviointi käyttävät tässä **kokeellisia kallistusrajoja: pituuskallistuma (*pitch*) ±24° ja poikittaiskallistuma (*roll*) ±16°**. Rajat on valittu tätä simulaatiokoetta varten; ne **eivät ole todennettuja valmistajan sallimia kaltevuuksia** eivätkä oikean ajoneuvon turvallisuusrajoja.

Haluttaessa Gazebon **Visualize Lidar** -näkymässä voi tarkastella aihetta `/camino/cooperative/a/lidar`. Sädevisualisointi ei ole välttämätön ohjauksen toiminnalle.

## Miten onnistuminen tarkistetaan?

Ohjaimen tilaviestien lisäksi erillinen arvioija käyttää Gazebon ajoneuvon **pose-mittauksia**. Se tarkistaa muun muassa:

| Hyväksymiskriteeri | Vaatimus |
| --- | --- |
| Kannon väistäminen | Mitattu jalanjälkietäisyys vähintään 1,0 m |
| Odotus ennen väistöä | Vähintään 5 simuloitua sekuntia paikallaan |
| Reitille paluu | Vähintään 1,5 m linjaan kohdistettua etenemistä vähintään 5 sekunnin ajan |
| Nousukorkeus | Vähintään 1,2 m |
| Pituuskallistuma | Enintään 24° itse simuloidun ajoneuvon mittauksissa |
| Poikittaiskallistuma | Enintään 16° itse simuloidun ajoneuvon mittauksissa |
| Taaksepäin liukuminen | Enintään 0,30 m reitin suuntaista taaksepäin etenemää |
| Tavoite | Ajoneuvo saapuu perille ja pysyy paikallaan vähintään 3 sekuntia |
| Aikaraja | Enintään 150 s simulaatioaikaa; lisäksi 300 s esittelyn todellista aikaa |

Simulaatio tallentaa jokaiselta kierrokselta esimerkiksi seuraavat tiedostot:

- `result.json` — kokonaisarvio (`"passed": true/false`), virhesyy, kesto ja yhteenvetomittarit.
- `evaluation.json` — ajoneuvon simuloitujen asentojen ja sijaintien perusteella arvioidut kallistumat, este-etäisyydet, nousu, pysähtyminen ja reitille paluu.
- `control.jsonl` — ohjaimen vaiheet, ajokomennot sekä pysähtymisen syyt.
- `summit-reports.zip` — tärkeimmät raportit yhdessä paketissa.

**PASS tarkoittaa vain, että määritetyt simulaation hyväksymisehdot toteutuivat kyseisellä kierroksella.** Se ei osoita fyysisen ajoneuvon maastokelpoisuutta tai turvallisuutta.

## Työpöytäkoneen live-Gazebo-tulokset (24.11.2026)

Yksi erillinen ajo ja sen jälkeen kaksi peräkkäistä `loop`-kierrosta tehtiin WSL Ubuntu 24.04 -ympäristössä. Molemmissa ajotavoissa käytettiin **fysiikka-askelta 0,002 s** ja **tavoite-RTF-arvoa 1,5**.

| Mittari | Erillinen ajo | Loop 1 | Loop 2 |
| --- | ---: | ---: | ---: |
| Lopputulos | **PASS** | **PASS** | **PASS** |
| Simulaatioaika | 130,606 s | 130,576 s | 130,478 s |
| Nousukorkeus | 5,883 m | 5,883 m | 5,882 m |
| Pienin kannon jalanjälkietäisyys | 1,850 m | 1,849 m | 1,851 m |
| Suurin pitch | 23,097° | 23,071° | 23,139° |
| Suurin roll | 11,588° | 11,515° | 11,578° |
| Todennettu pysähdys ennen väistöä | 5,316 s | 5,362 s | 5,236 s |
| Todennettu reitille paluun etenemä | 1,512 m | 1,509 m | 1,507 m |
| Reitille paluu todennettu | Kyllä | Kyllä | Kyllä |
| Pysähtyminen tavoitteessa | 3,070 s | 3,050 s | 3,002 s |
| Etäisyys lopputavoitteesta | 0,404 m | 0,406 m | 0,406 m |
| Ylämäen tavoite saavutettu | Kyllä | Kyllä | Kyllä |

**Kolme live-kierrosta onnistui (3/3 PASS).** Tämä tukee demon esittelykäyttöä, mutta ei takaa täydellistä toistettavuutta. Viimeisen kierroksen suurin pitch, 23,139°, jäi noin **0,86°** alle 24° kokeellisen rajan. Reitille paluun etenemä jäi **1,507 metriin**, eli alle sentin yli 1,5 m minimivaatimuksen. Näitä marginaaleja kannattaa seurata myös myöhemmissä ajoissa.

Tavoite-RTF kuvaa haluttua simulaation nopeutta, ei mitattua seinäkelloaikaa. Näille kolmelle kierrokselle ei ole tässä taulukossa vahvistettu todellisia ajoaikoja.

## Demon käynnistys WSL Ubuntu -ympäristössä

[Native Hill -paketti](camino-native-hill-fixed.zip) käyttää projektin Gazebo-malleja ja yhteisiä Python-moduuleja. Aja yksi kierros projektin juurihakemistosta.

```bash
python3 demos/camino-native-hill-fixed.zip demo \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

### Kahden peräkkäisen kierroksen testaus

```bash
python3 demos/camino-native-hill-fixed.zip loop \
  --laps 2 \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Ohjelma tekee uuden tuloshakemiston `/tmp/camino-native-hill-.../` ja tulostaa raporttien sijainnit. `--wall-timeout 900` on prosessin enimmäisodotus, **ei** esittelysuorituksen 300 sekunnin PASS-aikarajan korotus. `loop` sulkee ja käynnistää Gazebon kierrosten välillä; ohjelma ei ole saumaton videotoisto. `Ctrl+C` keskeyttää ajon.

Voit myös avata pelkän esikatselun komennolla `python3 demos/camino-native-hill-fixed.zip preview`.

## Kolmen demon erot ja turvallisuusrajaukset

| Demo | Tarkoitus | Keskeinen piirre |
| --- | --- | --- |
| [JARCRAC SHOWCASE](JARCRAC_SHOWCASE_FI.md) | Kahden ajoneuvon yhteistyö | Liittyminen, perässä ajo, esteenväistö ja kuormauspaikoille pysähtyminen |
| [JARCRAC SUMMIT](JARCRAC_SUMMIT_FI.md) | Yhden JARCRACin loivempi nousu | Noin 27 m kaartuva nousureitti, kanto ja ±8° simulaatiorajat |
| **JARCRAC NATIVE HILL** (tämä dokumentti) | Alkuperäisen rinteen jyrkempi nousu | Noin 22,44 m nousureitti, noin 5,88 m mitattu nousu sekä kokeelliset 24°/16° kallistusrajat |

Native-hill-demossa ei ole kahden työkoneen yhteistyötä, CAN-väylän fyysistä HIL-testausta eikä oikean koneen automaattista mäkiajoa. Kaikki ajoneuvot, anturit ja ympäristö ovat simuloituja. Väistön suunnittelu voi pysäyttää ajoneuvon, jos käytettävissä oleva mittaustieto ei riitä.

**Tärkeää:** Tämä ZIP-versio on erillinen kokeellinen demon toteutus. Älä korvaa sillä aiempaa kolmen PASS-kierroksen summit ZIP -demoa tai kahden ajoneuvon showcasea. Tallenna tiedostot erillisillä nimillä ja säilytä raportit.

## Keskeiset tiedostot ja linkit

- [camino-native-hill-fixed.zip](camino-native-hill-fixed.zip) — suoritettava Native Hill -demo.
- `src/jarcrac_summit_knots.py` (ZIP:n sisällä) — Ruskotunturin alkuperäiseen maastoon sidotut reittipisteet.
- `src/jarcrac_summit_route.py` (ZIP:n sisällä) — reitin geometria, maastoprofiili ja kokeelliset kallistusrajat.
- `src/jarcrac_summit_mission.py` (ZIP:n sisällä) — nousu, jarrutus, kannon kierto ja reitille palaaminen.
- `src/jarcrac_summit_scene.py` (ZIP:n sisällä) — kanto, punaiset reittimerkit, vihreä tavoite ja kamera.
- `src/jarcrac_summit_score.py` (ZIP:n sisällä) — mitattuihin Gazebo-poseihin perustuva arviointi.
- `src/jarcrac_summit_trial.py` (ZIP:n sisällä) — Gazebo-kierroksen ajaminen ja raportointi.
- `README.md`, `native-hill-route.png` ja `verification/` (ZIP:n sisällä) — tekninen kuvaus, reittikuva sekä offline-toistokokeiden tulokset.

Lisätietoja: [CAMINO README](../README.md) ja [Ruskotunturin Gazebo-projekti](../edge-path-project/gazebo/README.md).

*Software-in-the-Loop tarkoittaa, että ohjausohjelmistoa testataan simuloidussa ympäristössä ennen mahdollista oikean kaluston testausvaihetta.*
