# CAMINO: kahden ajoneuvon kohtaaminen yksikaistaisella metsätiellä

**Viides esittelydemo 30.11.2026 — Software-in-the-Loop (SIL)**

Tämä demo täydentää [kahden ajoneuvon yhteistyö- ja perässäajodemoa](JARCRAC_SHOWCASE_FI.md), [kaartuvaa summit-nousua](JARCRAC_SUMMIT_FI.md), [jyrkempää Ruskotunturin nousua](JARCRAC_NATIVE_HILL_FI.md) ja [liikkuvan esteen turvavalvontademoa](JARCRAC_SAFETY_SUPERVISOR_FI.md).

**Keskeinen ajatus:** kaksi konetta saapuu kapealle metsäautotielle vastakkaisista suunnista. Tie on niin kapea, että vain yksi ajoneuvo saa käyttää sitä kerrallaan. Järjestelmä antaa ajovuoron ensin saapuneelle ja pitää toisen ajoneuvon odotusalueella, kunnes ensimmäisen **koko törmäysjalanjälki** on poistunut yksikaistaiselta osuudelta.

Kyse on Gazebo Harmonic -ympäristössä ajettavasta **SIL-simulaatiosta**, ei oikeiden työkoneiden maastokokeesta eikä turvallisuussertifioidusta liikenteenohjauksesta.

## Lyhyesti: mitä demossa tapahtuu?

- **Ajoneuvo A** on telaketjullisen JARCRACin simulaatiomalli.
- **Ajoneuvo B** on Ackermann-ohjattu, pyörillä liikkuva ajoneuvomalli.
- Ajoneuvot lähtevät yksikaistaisen osuuden eri puolilta ja ajavat kohti toisiaan. Lähdön viiveitä vaihtamalla voidaan esittää kaksi tilannetta: **JARCRAC ensin** tai **ajoneuvo B ensin**.
- Keltainen saapumisviiva osoittaa, milloin ajoneuvon keula on saapunut etuajo-oikeuden ratkaisualueelle. **Ensimmäisenä oman saapumisviivansa saavuttanut ajoneuvo saa vuoron.** Täsmällisessä aikaleimatasatilanteessa etusija annetaan JARCRACille.
- Vuoroaan odottava ajoneuvo pysähtyy omalle merkitylle odotusalueelleen.
- Ensimmäinen ajoneuvo ajaa noin **22 metrin** pituisen, kahden porttiviivan rajaaman yksikaistaisen osuuden läpi.
- Vasta kun sen **takaosa** on ylittänyt vastakkaisen poistumisportin, toinen ajoneuvo saa ajaa kapealle osuudelle.
- Lopuksi molemmat ajoneuvot pysähtyvät omille värimerkityille pysäköintialueilleen vähintään **kolmeksi simuloiduksi sekunniksi**.

Demossa käytetään **alkuperäistä Ruskotunturin maastomallia**. Paketin mukana toimitetaan tarvittavat mallit ja maailmaresurssit. Tie-, portti-, odotus- ja maalimerkinnät ovat tätä koetta varten lisättyjä visualisointeja.

## Mitä katsojan kannattaa seurata?

| Vaihe | Gazebossa näkyvä tapahtuma | Miksi sillä on merkitystä? |
| --- | --- | --- |
| 1. Vastakkaiset lähestymiset | JARCRAC ja ajoneuvo B ajavat kohti kapeaa tieosuutta eri suunnista. | Yhteinen työmaaväylä tarvitsee ajovuorojen koordinointia. |
| 2. Saapumisjärjestys | Ajoneuvon keula ylittää keltaisen saapumisviivan. | Etuajo-oikeus päätetään mitattuun saapumisaikaan perustuen. |
| 3. Odotusalue | Toinen ajoneuvo pysähtyy omaan sivutaskuunsa. | Ajoneuvot eivät siirry yhtä aikaa yksikaistaiselle osuudelle. |
| 4. Ensimmäinen läpiajo | Vuoron saanut ajoneuvo kulkee kapean osuuden läpi. | Yksikaistaisen alueen käyttöoikeus on vain yhdellä ajoneuvolla. |
| 5. Vuoron vapauttaminen | Ensimmäisen ajoneuvon takaosa poistuu vastakkaisen portin yli. | Pelkkä keulan ulostulo ei riitä: koko törmäysjalanjäljen on poistuttava. |
| 6. Toinen läpiajo | Odottanut ajoneuvo saa luvan edetä. | Kulkujärjestystä noudatetaan molemmille ajoneuvoille. |
| 7. Pysäköinti | JARCRAC pysähtyy vihreään ja B siniseen pysäköintialueeseen. | Koko kuljetustehtävän onnistuminen varmistetaan. |

**Huomio näyttelyssä:** Pysähtynyt ajoneuvo ei välttämättä ole jumissa. Testatuissa ajoissa odottaminen kesti jopa noin **88 simuloitua sekuntia**, koska odotus jatkuu, kunnes ensimmäinen ajoneuvo on poistunut kokonaan.

## Miten ajovuorojen koordinointi toimii?

Paketin `lane_course.py` sisältää molempien ajoneuvojen reitit ja tilan **Meeting**. Se käyttää paritettuja Gazebon odometriamittauksia sekä ajoneuvojen törmäysjalanjälkien geometriaa.

1. Ajoneuvon **etummaisen törmäysjalanjälkipisteen** saapumisaika määritetään saapumisviivan ylityksestä, tarvittaessa kahden peräkkäisen mittauksen välillä interpoloiden.
2. Kun ensimmäinen saapuminen vahvistuu, ohjaus varaa yksikaistaisen osuuden kyseiselle ajoneuvolle.
3. Toisen ajoneuvon reittinohjain ohjaa vain odotusalueelle (`WAIT`), ei vapaasti yksikaistaiselle tielle.
4. Vuoro vapautetaan vasta, kun ensimmäisen ajoneuvon **takaosan** mitattu reittiasema ylittää poistumisportin.
5. Sen jälkeen toinen ajoneuvo voi saada tieosuuden käyttöoikeuden ja edetä omaan maaliinsa.

Jos paritetut tilamittaukset puuttuvat, ovat vanhentuneita tai ristiriitaisia, ohjain komentaa pysähdystä. Samoin ohjaus keskeytyy, jos ajoneuvo on kapealla osuudella ilman sille kuuluvaa vuoroa tai mitattu kahden ajoneuvon etäisyys alittaa minimivälin.

**Tämä ei ole ajoneuvojen välinen todellisen radio- tai CAN-viestinnän validointi.** Vuoroja koordinoi yhteinen simulaatio-ohjain, ja sijainnit perustuvat Gazebon odometriaan.

## Miten onnistuminen tarkistetaan?

Ohjelman `lane_score.py` käyttää **Gazebon maailman malliasentoja**, jotka arvioidaan erillään ajovuorojen ohjauspäätöksistä. Jokaiselle kierrokselle tallennetaan:

- `result.json` — PASS/FAIL, syy, saapumis- ja portinylitysajat, odotuksen kesto sekä ajoneuvojen pienin etäisyys.
- `poses.csv` — kahden ajoneuvon tallennetut sijainnit ja asennot.
- `watch.html` — selaimessa avattava, tallennetuista poseista muodostettu katseluympäristö.

PASS-tulokseen vaaditaan muun muassa:

| Ehto | Hyväksymiskriteeri |
| --- | --- |
| Oikea saapumisjärjestys | Saapumisviivojen mitattu ylitysjärjestys vastaa kierroksen tavoitetta. |
| Yksikaistaisen osuuden käyttö | **Enintään yhden ajoneuvon törmäysjalanjälki** saa olla 22 m osuudella kerrallaan. |
| Ajoneuvojen välinen etäisyys | Vähintään **1,0 m** mitattua tasoprojektion jalanjälkietäisyyttä. |
| Odotus | Odottaja on pysähtynyt omalla alueellaan vähintään **3 sekuntia** viimeistään, kun ensimmäinen saavuttaa yksikaistaisen osuuden keskikohdan. |
| Vuoron vapauttaminen | Toinen ajoneuvo ei saa tulla tieosuudelle ennen ensimmäisen **takaosan poistumista**. |
| Reitti ja liike | Ajoneuvot pysyvät merkityllä reitillä ja simuloitujen asentojen liike on jatkuvaa. |
| Kallistuma | Ohjelman kokeellinen **8°** raja sekä pitch- että roll-kallistumalle. |
| Pysäköinti | Molemmat ajoneuvot pysyvät pysähtyneinä omissa maaleissaan **3 sekuntia**. |
| Aikaraja | Enintään **450 simuloitua sekuntia** kumpaakin kierrosta kohti. |

8° on tämän simulaation ohjelmallinen hyväksymisraja, **ei vahvistettu fyysisen JARCRACin turvallinen enimmäiskallistuma**. PASS tarkoittaa määriteltyjen SIL-testiehtojen täyttymistä, ei fyysisten koneiden turvallisuustodistusta.

## Esityskoneella suoritetut live-Gazebo-testit (25.11.2026)

Käytössä oli ZIP-paketin **`camino-single-lane-physics-rtf.zip`** -versio. Siinä voi valita Gazebon fysiikka-askeleen ja tavoite-RTF:n käynnistysparametreilla. Näissä live-ajoissa käytettiin samoja asetuksia kuin muissa CAMINO-demonstraatioissa:

- Fysiikka-askel **0,002 s** (2 ms).
- Tavoite-RTF **1,5**.
- Gazebo-prosessin seinäkelloaikaraja **1200 s / kierros**.

Yksi erillinen **JARCRAC ensin** -ajo sekä sen jälkeen molempien ajovuorojärjestysten `--first both` -ajo päättyivät onnistuneesti.

| Mittari | Erillinen A ensin | Toisto: A ensin | B ensin |
| --- | ---: | ---: | ---: |
| Tulos | **PASS** | **PASS** | **PASS** |
| Molemmat ajoneuvot pysäköity | Kyllä | Kyllä | Kyllä |
| Pienin mitattu ajoneuvojen väli | **2,92 m** | **2,92 m** | **2,94 m** |
| Paikallaan odottaminen | **87,9 s** | **87,8 s** | **63,8 s** |
| Arviointitiedosto | `first-a/result.json` | `first-a/result.json` | `first-b/result.json` |

**Yhteensä 3/3 live-Gazebo-kierrosta PASS.** A ensin -järjestys onnistui kahdesti; B ensin -järjestys kerran. Lyhyt toistomäärä ei osoita täydellistä luotettavuutta, mutta molemmat ajovuorojärjestykset ovat nyt toteutuneet onnistuneesti oikeassa Gazebo-simulaatiossa.

**Tietolähde:** Taulukon live-mittaukset on kirjattu esityskoneen konsolitulosteista. Tämä Markdown ja yhteenveto-JSON eivät ole alkuperäisiä `result.json`- tai `poses.csv`-tiedostoja. Säilytä myös alkuperäiset lokit ja pose-tallenteet.

### Alkuperäisen ZIP:n offline-toistot

Paketti sisältää lisäksi ennalta tallennetut **kinemaattiset offline-replayt**, jotka ovat eri asia kuin live-Gazebo-kierrokset:

| Replay | Minimi-etäisyys | Odotus | Tulos |
| --- | ---: | ---: | --- |
| JARCRAC ensin | 2,9601 m | 92,7 s | PASS |
| Ajoneuvo B ensin | 2,9522 m | 64,4 s | PASS |

Offline-replay käyttää yksinkertaistettua liike- ja maastomallia. Näitä lukuja **ei tule sekoittaa** yllä oleviin kolmeen Gazebon live-tulokseen.

## Demon käynnistys WSL Ubuntu -ympäristössä

Tämä demo on muista ohjelmista erillinen suoritettava Python ZIP. Se sisältää Ruskotunturin mallin, molemmat ajoneuvomallit, ohjaimen, arvioijan ja selainkatselun. Käynnistys ei edellytä toimivien showcase-, summit-, native-hill- tai safety-supervisor-ohjaimien muokkaamista.

Kun ZIP on lisätty tämän dokumentin rinnalle GitHubin `demos/`-hakemistoon, voit ajaa yhden **JARCRAC ensin** -kierroksen:

```bash
cd ~/camino-desktop
python3 demos/camino-single-lane-physics-rtf.zip demo \
  --first a \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 1200
```

Koko kahden ajovuorojärjestyksen esittely:

```bash
cd ~/camino-desktop
python3 demos/camino-single-lane-physics-rtf.zip demo \
  --first both \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 1200
```

Jos ZIP on toistaiseksi WSL-kotihakemistossa, **jo live-testattu** komento toimii kokeilu-worktreestä:

```bash
cd ~/camino-summit-test
python3 ~/camino-single-lane-physics-rtf.zip demo \
  --first both \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 1200
```

Voit testata vain B ensin -tilanteen vaihtamalla `--first both` → `--first b`.

Paketin toiminta kirjoittaa erillisen hakemiston `/tmp/camino-single-lane-.../` ja luo siellä `first-a/`, `first-b/` ja `watch.html` -tiedostot. **Aja katselu** avaamalla esimerkiksi:

```bash
explorer.exe /tmp/camino-single-lane-e6ci6u9h/watch.html
```

Tuo polku on yhden onnistuneen testikierroksen tallenteeseen; uuden kierroksen hakemistopolku vaihtuu. `--first both` käynnistää **tuoreen Gazebo-simulaation kumpaakin vuorojärjestystä varten**. Kierrosten vaihto ei ole saumaton animaatiolooppi. Pysäytä ajaminen `Ctrl+C`:llä.

Pelkkä tämä Markdown ei ole suoritettava demo. Testattu ZIP on tallennettava sen rinnalle polkuun `demos/camino-single-lane-physics-rtf.zip`.

### Testatun paketin tarkistussumma

```text
SHA-256: d24191dc62aad239db25d5a399ff707bb41ede0e1c16d1d0c285708a1879ef2e
```

Tarkista tiedosto WSL:ssä esimerkiksi komennolla `sha256sum ~/camino-single-lane-physics-rtf.zip`. ZIP-paketin oma `README.md` voi edelleen mainita, ettei live-Gazebo-ajokertoja ollut suoritettu **paketin tekohetkellä**. Tämä dokumentti kirjaa sen jälkeen tehdyt kolme onnistunutta live-koetta muuttamatta testatun ZIP:n sisältöä.

## Miten tämä eroaa muista CAMINO-demoista?

| Demo | Painopiste |
| --- | --- |
| [SHOWCASE](JARCRAC_SHOWCASE_FI.md) | Kaksi ajoneuvoa liittyy samalle reitille ja ajaa peräkkäin, väistää esteen ja saapuu kuormausalueelle. |
| [SUMMIT](JARCRAC_SUMMIT_FI.md) | Yksi JARCRAC kiertää kannon ja nousee loivempaa kaartuvaa reittiä. |
| [NATIVE HILL](JARCRAC_NATIVE_HILL_FI.md) | Yksi JARCRAC nousee jyrkempää alkuperäistä Ruskotunturin rinnettä. |
| [SAFETY SUPERVISOR](JARCRAC_SAFETY_SUPERVISOR_FI.md) | Erillinen turvavalvoja keskeyttää normaalin ohjauksen liikkuvan vaaran vuoksi. |
| **SINGLE LANE (tämä)** | Kaksi vastakkaisista suunnista tulevaa konetta neuvottelee yhteisen kapean tieosuuden ajovuorosta. |

Demon kannalta tärkeintä on, että **oikeus käyttää jaettua työmaa-aluetta perustuu mitattuihin saapumis- ja poistumistapahtumiin**, ei vain aikaan tai näytöllä näkyviin ajoneuvojen keskikohtiin.

## Keskeiset paketin tiedostot

- `demos/camino-single-lane-physics-rtf.zip` — itsenäinen live-Gazebo- ja offline-replay-paketti **kun ZIP on lisätty GitHubiin**.
- `__main__.py` (ZIP:n sisällä) — komentorivivalinnat, ajovuorojärjestykset ja kierrosten käynnistys.
- `lane_course.py` — portit, odotusalueet, saapumisen tulkinta, reitinohjain ja ajovuorojen jako.
- `lane_gazebo.py` — Gazebo-maailma, ajoneuvot, fysiikkaparametrit, ajokierrokset ja pose-tallennus.
- `lane_score.py` — riippumaton arviointi Gazebon mitatuista asennoista.
- `lane_replay.py` — erillinen kinemaattinen offline-toisto.
- `lane_view.py`, `watch.html` — simulaation tapahtumien selainkatselu.
- `test_single_lane.py` — paketissa olevat offline-testit.
- `recorded/first-a/` ja `recorded/first-b/` — pakettiin sisältyvät offline-tallenteet (ei live-Gazebo-tuloksia).
- `demos/camino-single-lane-physics-rtf-live-results.json` — kolmen live-ajon konsolitulosteista koottu mittaustaulukko.

Lisätietoja: [CAMINO-projektin pää-README](../README.md), [Gazebo-dokumentaatio](../edge-path-project/gazebo/README.md) ja muut `demos/`-hakemiston esittelyt.

*Software-in-the-Loop (SIL) tarkoittaa ohjauslogiikan testaamista simuloiduilla ajoneuvoilla, antureilla ja ympäristöllä. Tulokset eivät sellaisinaan ole osoitus oikean JARCRACin tai toisen työkoneen turvallisesta käytöstä.*
