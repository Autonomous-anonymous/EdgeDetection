# CAMINO: JARCRACin nousu ja kannon väistö Gazebo-simulaatiossa

**Varademo 30.11.2026 — Software-in-the-Loop (SIL)**

Tämä on **erillinen yhden ajoneuvon esittelydemo**, joka täydentää [kahden ajoneuvon yhteistyödemoa](JARCRAC_SHOWCASE_FI.md). Päädemo näyttää ajoneuvojen keskinäistä koordinointia; tässä demossa painopiste on **maaston kaltevuudessa, esteen havaitsemisessa, väistössä ja reitille palaamisessa**.

## Lyhyesti: mitä demossa tapahtuu?

Gazebo Harmonic -simulaatiossa yksi telaketjullisen **JARCRACin yksityiskohtaisella telamallilla** toteutettu virtuaaliajoneuvo kulkee Ruskotunturin maastossa noin **27 metrin mittaista, kaartuvaa ylämäkireittiä**.

- Ajoneuvo käynnistyy reitin alkupäästä ja odottaa aluksi hetken, jotta simuloidun ajoneuvon asento ehtii tasaantua.
- Reitillä on puunkantoa esittävä **sylinterimäinen este**, jonka halkaisija on **0,64 m** ja korkeus **0,58 m**.
- JARCRAC havaitsee esteen eteenpäin katsovalla LiDAR-anturilla, jarruttaa ja pysähtyy. Se odottaa vähintään **viisi simuloitua sekuntia** ennen väistön aloittamista.
- Ohjain etsii ajettavaksi arvioidun väistöreitin, kiertää kannon, palaa suunnitellulle kaartuvalle reitille ja jatkaa ylämäkeen.
- Lopuksi JARCRAC pysähtyy **vihreällä renkaalla merkittyyn huipputavoitteeseen**.

Kyseessä on **SIL-simulaatio**, ei oikean JARCRAC-ajoneuvon maastokoe, eikä tulos yksin osoita järjestelmän soveltuvuutta turvallisuuskriittiseen käyttöön.

## Mitä katsojan kannattaa seurata?

| Vaihe | Mitä ruudulla näkyy? | Mitä se havainnollistaa? |
| --- | --- | --- |
| 1. Lähtö ja tasaantuminen | JARCRAC odottaa lyhyesti lähtöpaikallaan ja lähtee sitten liikkeelle. | Simuloidun alustan asennon tarkkailu |
| 2. Esteen havaitseminen | Ajoneuvo lähestyy kantoa, hidastaa ja pysähtyy. | LiDAR-havainnot ja jarrutuspäätös |
| 3. Odotus | Ajoneuvo pysyy paikallaan vähintään viisi simuloitua sekuntia. | Pysähtymisen todentaminen ennen väistöä |
| 4. Kannon kierto | JARCRAC ohittaa kannon turvalliseksi arvioitua reittiä. | Esteenväistö ja ajouran tarkistaminen |
| 5. Reitille paluu | Ajoneuvo palaa kaartuvan pääreitin tuntumaan. | Ohjauksen kaarevuusrajat ja reitille palaaminen |
| 6. Nousu ja saapuminen | JARCRAC jatkaa rinnettä ja pysähtyy vihreän tavoiterenkaan kohdalle. | Maaston huomioiva navigointi ja tavoitteen saavuttaminen |

**Huomio:** Jos ohjain ei kykene vahvistamaan kulkureitin tai pysähtymistilan turvallisuutta käytettävissä olevilla havainnoilla, ajoneuvo pysähtyy. Pitkä odotus tai päättymätön reitille paluu ei ole onnistunut suoritus.

## Mitä antureita ja ohjausta käytetään?

- Simuloidun JARCRACin eteenpäin katsova noin **180° LiDAR** tuottaa havaintoja esteistä ja ajettavasta tilasta.
- Python-ohjain hyödyntää aikaleimattua **Gazebo-odometriaa**, LiDAR-havaintoja ja Ruskotunturin maastomallia.
- Väistöehdokkaiden arvioinnissa tarkistetaan muun muassa ajoneuvon jalanjälki, maastoprofiili, reittikäytävän rajat, kaarevuus sekä havaitun kannon sijainti.
- Ohjain käyttää nykyisiä ja aikaisempia havaintoja liikkeen tarkistamiseen. Havaitsematonta tilaa ei tule tulkita automaattisesti vapaaksi.
- Simulaation hyväksymisrajana on **enintään 8° kallistuma** sekä pituus- että poikittaissuunnassa (*pitch* ja *roll*). Tämä on ohjelmiston testiraja, **ei vahvistettu oikean koneen valmistajaraja**.
- Riippumaton arviointi käyttää Gazebon simuloituja ajoneuvon asentoja; kyseessä ei ole riippumaton tosimaailman paikannusvalidointi.

Haluttaessa Gazebon **Visualize Lidar** -näkymässä voi valita aiheen `/camino/cooperative/a/lidar`. LiDAR-säteiden näyttäminen ruudulla on valinnainen visualisointi.

## Miten onnistuminen tarkistetaan?

Kukin kierros tuottaa tulostiedostot:

- `result.json` — koko testin tulos (`passed`), mahdollinen epäonnistumisen syy, simulaatio- ja seinäkelloaika sekä yhteenvetomittarit.
- `evaluation.json` — riippumattomasta simuloidusta pose-tiedosta johdetut etäisyydet, kallistumat, reitille paluu, nousu ja tavoitepysähdys.
- `control.jsonl` — ohjaimen vaiheet, komennot ja pysähtymisen syyt.
- `summit-reports.zip` — keskeiset raportit samassa siirrettävässä arkistossa.

PASS-tuloksen edellytyksiä ovat muun muassa:

| Vaatimus | Hyväksymisraja |
| --- | --- |
| Etäisyys kannosta | Vähintään 1,0 m mitattua ajoneuvon jalanjälkiväliä |
| Paikallaan odottaminen | Vähintään 5 simuloitua sekuntia |
| Reitille paluu | Vähintään 1,5 m reittiin kohdistettua etenemistä vähintään 5 sekunnissa |
| Nousukorkeus | Vähintään 1,2 m |
| Pituus- ja poikittaiskallistuma | Enintään 8° kumpaankin suuntaan |
| Tavoite | Ajoneuvo saavuttaa huipun ja pysähtyy |
| Aikarajat | Enintään 150 s simulaatioaikaa ja 300 s todellista aikaa esittelykriteerin mukaan |

### Työpöytäkoneella mitattu Gazebo-koe (23.11.2026)

| Mittari | Toteutunut tulos |
| --- | ---: |
| Koko kierros | **PASS** |
| Simulaatioaika | 102,212 s |
| Nousukorkeus | 1,981 m |
| Pienin etäisyys kannosta | 1,965 m |
| Suurin pituuskallistuma (pitch) | 7,264° |
| Suurin poikittaiskallistuma (roll) | 5,646° |
| Todennettu pysähtyminen | 5,378 s |
| Todennettu reitille paluun etenemä | 1,524 m / 5,572 s |
| Huipputavoite saavutettu | Kyllä |

Koe tehtiin asetuksilla **fysiikka-askel 0,002 s** ja **tavoite-RTF 1,5**. Ajoneuvo suoritti kannon väistön, palasi reitille ja pysähtyi huipputavoitteeseen.

**Tämä on yksi raportoitu onnistunut live-Gazebo-kierros, ei vielä toistettavuustakuu.** Erityisesti suurin pitch (7,264°) on melko lähellä 8° rajaa, ja reitille paluun etenemä (1,524 m) on lähellä 1,5 m hyväksymisrajaa. Lisäkierrokset sekä alkuperäisen päädemon regressiotesti ovat tarpeen ennen näyttelykäyttöä.

## Demon käynnistys WSL Ubuntu -ympäristössä

Summit toteutetaan **itsenäisenä Pythonilla ajettavana ZIP-pakettina** nimeltä `camino-summit-climb.zip`. Se hyödyntää saman projektin Gazebo-malleja ja ohjelmistoriippuvuuksia mutta ei edellytä päädemon skriptien muokkaamista.

Kun ZIP on lisätty tämän dokumentin rinnalle hakemistoon `demos/`, käynnistä yksi kierros projektin juuresta:

```bash
cd ~/camino-desktop
python3 demos/camino-summit-climb.zip demo \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Jos ZIP on toistaiseksi vain paikallisessa kotihakemistossa, käytä jo testattua muotoa:

```bash
cd ~/camino-desktop
python3 ~/camino-summit-climb.zip demo \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Paketti luo oletusarvoisesti uuden tuloshakemiston `/tmp/camino-summit-climb-.../lap` ja tulostaa raporttien polut konsoliin. **RTF on tavoite**, ei lupaus todellisesta suoritusnopeudesta.

### Kahden kierroksen harjoitus

```bash
cd ~/camino-desktop
python3 demos/camino-summit-climb.zip loop \
  --laps 2 \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Kierrosten välissä Gazebo suljetaan ja käynnistetään uudelleen. Sarja pysähtyy ensimmäiseen epäonnistumiseen; `Ctrl+C` keskeyttää ajon. Ilman `--laps`-valintaa silmukka jatkuu onnistuneiden kierrosten jälkeen.

**Päädemo säilyy erillisenä:** sen käynnistyskomento löytyy tiedostosta [JARCRAC_SHOWCASE_FI.md](JARCRAC_SHOWCASE_FI.md). Summit-paketti ei muuta kahden ajoneuvon ohjainta tai esittelyskenaariota.

## Kokeilun rajaukset ja seuraavat testit

- Tämän version **offline-replay** käyttää yksinkertaistettua ajoneuvoliikettä. Varsinainen 23.11.2026 PASS on tärkeämpi osoitus Gazebossa tapahtuvasta käyttäytymisestä, mutta edelleen vain yksi ajo.
- Toista summit useita kertoja samalla esitystietokoneella ja asetuksilla. Tarkista jokaisesta `result.json`-tiedostosta `passed`, kallistumat, kannon etäisyys ja reitille paluu.
- Aja tämän jälkeen kahden ajoneuvon showcase uudelleen ja varmista, että se tuottaa PASS-tuloksen.
- Tämän SIL-demon ajossa ei käytetä oikeaa R2100-ohjainta, CAN-väylää eikä HIL-kytkentää. Ne kuuluvat projektin erillisiin kokeisiin.
- Tallenna onnistuneen esittelyajon videotallenne varalle.

## Keskeiset tiedostot

- `demos/camino-summit-climb.zip` — erillinen suoritettava Python ZIP -demo, **kun paketti on toimitettu GitHubiin**.
- `src/jarcrac_summit_mission.py` (ZIP-paketissa) — JARCRACin esteenväistö, nousu ja reitille paluu.
- `src/jarcrac_summit_route.py` ja `src/jarcrac_summit_knots.py` (ZIP-paketissa) — mitoitettu kaartuva reitti ja maastogeometria.
- `src/jarcrac_summit_scene.py` (ZIP-paketissa) — kanto, maalimerkinnät ja kameranäkymä.
- `src/jarcrac_summit_score.py` (ZIP-paketissa) — riippumaton mittaus ja hyväksymiskriteerit.
- `src/jarcrac_summit_trial.py` (ZIP-paketissa) — simulaation käynnistys ja raportointi.
- [Kahden ajoneuvon päädemon kuvaus](JARCRAC_SHOWCASE_FI.md) — alkuperäinen näyttelydemo.
