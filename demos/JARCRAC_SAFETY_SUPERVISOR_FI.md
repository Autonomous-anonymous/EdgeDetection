# CAMINO: JARCRACin itsenäinen turvavalvoja ja liikkuva este Gazebo-simulaatiossa

**Neljäs esittelydemo 30.11.2026 — Software-in-the-Loop (SIL)**

Tämä demo täydentää [kahden ajoneuvon yhteistyödemoa](JARCRAC_SHOWCASE_FI.md), [loivempaa summit-nousua](JARCRAC_SUMMIT_FI.md) ja [jyrkempää Ruskotunturin nousua](JARCRAC_NATIVE_HILL_FI.md). Aiemmissa esittelyissä korostuvat navigointi, yhteistyö ja esteen kiertäminen. **Tässä demossa pääasia on ajoneuvon normaalista navigointiohjaimesta erotettu turvallisuusvalvoja, joka voi keskeyttää liikkeen vaaran ilmaantuessa.**

**Tärkeä rajaus:** Kyseessä on Gazebo Harmonic -ympäristössä toteutettu SIL-koe. Tätä testiä ei ole tehty oikealla JARCRACilla, R2100-ohjaimella tai fyysisellä CAN-väylällä. Ohjelmallinen turvavalvoja ei ole turvallisuussertifioitu järjestelmä.

## Lyhyesti: mitä demossa tapahtuu?

- Yksi **yksityiskohtaisella telamallilla toteutettu JARCRAC** ajaa Ruskotunturin alkuperäisessä maastomallissa noin **27 metrin kaartuvaa nousureittiä**.
- **Normaali reitinohjain** antaa nopeus- ja ohjauskomentoja. Se ei tunne liikkuvan esteen aikataulua eikä saa esteen tarkkaa sijaintia.
- Oranssivalkoinen laatikkomainen **liikkuva vaarakohde** siirtyy reitin poikki noin 11 metrin reittiaseman kohdalla. Esteen koko on **0,95 × 0,95 × 1,45 m**, ja sen ohjelmoitu sivuttaisnopeus on noin **0,36 m/s**.
- **Itsenäinen turvavalvoja** tarkkailee JARCRACin simuloitua LiDARia ja ajoneuvon tilatietoja. Kun se havaitsee esteen reitin edessä, se korvaa normaalin ajokäskyn pysäytyskäskyllä.
- Valvoja pitää ajoneuvon paikallaan, kunnes havainnot osoittavat ajokäytävän olleen esteetön vähintään **1,2 simuloitua sekuntia**.
- Valvoja vapauttaa ohjauksen normaalille reitinohjaimelle. JARCRAC jatkaa ja pysähtyy vihreään maaliin.

**Tässä kokeessa JARCRAC ei kierrä estettä.** Se osoittaa erillisen turvallisuustoiminnon **pysäytä → odota → jatka**. Liikkuva este on Gazebon erillisen ohjelmaprosessin siirtämä simuloitu laatikko, ei oikea jalankulkija tai toinen autonominen ajoneuvo.

## Mitä katsojan kannattaa seurata?

| Vaihe | Näytöllä tai lokissa näkyvä tapahtuma | Havainnollistettava ominaisuus |
| --- | --- | --- |
| 1. Lähtö | JARCRAC aloittaa reitin seurannan. Tilana on `SAFETY NAVIGATION`. | Normaali tehtäväohjaus |
| 2. Vaaran ilmaantuminen | Oranssivalkoinen este liikkuu ajoreitin poikki. | Muuttuva työmaaympäristö |
| 3. Turvavalvojan ohitus | Lokissa `SAFETY EMERGENCY_BRAKE`. Normaali pyyntö **0,27 m/s**, mutta toteutettava käsky **0,00 m/s**. | Turvavalvojan etusija normaaliin ohjaimeen nähden |
| 4. Turvallisuuden varmistus | `SAFETY CLEARANCE_HOLD` jatkaa pysäytystä estehavainnon poistuttua. | Uudelleenliikkeellelähdön viive ja hystereesi |
| 5. Ohjauksen palautus | `SAFETY NAVIGATION` ja toteutettava nopeuspyyntö jälleen **0,27 m/s**. | Hallittu tehtävän jatkaminen |
| 6. Maali | JARCRAC pysähtyy merkitylle tavoitealueelle. | Valvotun tehtävän loppuun saattaminen |

Alussa voi näkyä tiloja `SENSOR_FAIL_SAFE` ja `STATE_FAIL_SAFE`, kun kellon, anturin tai ajoneuvon mittausvirta ei ole vielä valmis. Niissä pysäytyskäsky on tarkoituksellinen. Vastaavasti mittauksen vanhentuessa ajoneuvo ei saa jatkaa sokkona.

## Ohjelmistoarkkitehtuuri

Kolme toimintoa pidetään tarkoituksella erillisinä:

1. **Reitinohjain (RouteFollower)**: muodostaa nimellisen telakomennon ajoneuvon odometrian ja reitin perusteella. Se ei näe esteen komentoreittiä.
2. **Turvavalvoja (SafetySupervisor)**: tarkistaa ajoneuvon asennon ja mittausten tuoreuden sekä maastosta suodatetut LiDAR-estehavainnot. Se voi korvata normaalin komennon STOP-komennolla. Suojatarkasteluun kuuluu noin **5,6 m** eteenpäin reitin suunnassa ja **1,85 m** sivuttaissuuntainen käytävä.
3. **Liikkuvan esteen ohjain ja erillinen arvioija**: esteen kulkua ohjataan Gazebon `set_pose_vector`- tai `set_pose`-palvelun avulla. Ohjain vahvistaa palveluvastauksen **ja** tarkistaa siirretyn esteen havaitun pose-arvon Gazebon mittausaiheesta. Riippumaton arvioija käyttää ajoneuvon ja esteen mitattuja Gazebo-maailma-asentoja, ei pelkästään turvavalvojan omia tilailmoituksia.

Turvavalvoja ei saa arvioijan käyttämää esteen tarkkaa paikkaa. Arvioija puolestaan mittaa todellisen simulaatiossa toteutuneen etäisyyden ajoneuvon jalanjäljen ja esteen välillä.

**Tämä toteutus on LiDAR-havaintoon ja reittikäytävään perustuva suojapysäytys, ei validoitu TTC-pohjainen automaattinen hätäjarrutus.** Fyysistä turvatoimintoa tai suorituskyvyn turvallisuusluokitusta ei tässä todenneta.

## Miten onnistuminen arvioidaan?

Jokaiselle kierrokselle kirjoitetaan:

- `result.json` — yhdistetty PASS/FAIL, virhesyy, ajoajat, mitatut rajat ja turvavalvojan jälki.
- `evaluation.json` — Gazebon pose-tietoihin perustuva ajoneuvo–este-etäisyys, mitatut kallistumat, pysähtyminen, jatkaminen ja maali.
- `hazard.json` — esteen ohjauksen palvelupyynnöt, Gazebon pose-vahvistukset ja mahdolliset virheet.
- `control.jsonl` — jokaisen ohjausjakson **pyydetty** ja **toteutettava** nopeus sekä turvavalvojan toimintatila.
- `safety-reports.zip` — koottu raporttiarkisto.

PASS edellyttää seuraavien ehtojen täyttymistä:

| Tarkistus | Ehto |
| --- | --- |
| Liikkuvan esteen läpikulku | Esteen liikkeen alku ja valmistuminen havaittu myös Gazebon pose-tiedoista |
| Todellinen turvaohitus | Vähintään kaksi lokihavaintoa, joissa turvavalvoja pysäyttää positiivisen normaalin ajopyynnön |
| Pysähtyminen | Ajoneuvon todellinen nopeus putoaa ≤ 0,065 m/s vähintään 0,45 sekunniksi esteen lähestyessä |
| Ajon jatkaminen | Esteen ohituksen jälkeen ajoneuvo liikkuu eteenpäin vähintään 0,8 m ja nopeus on ≥ 0,10 m/s |
| Jalanjälkietäisyys | Esteen ja ajoneuvon välillä koko ajan vähintään **1,0 m** |
| Kallistuma | Pituus- ja poikittaiskallistuma enintään **8°** kumpaankin suuntaan |
| Maali | Ajoneuvo saavuttaa tavoitteen ja pysähtyy siinä vähintään 0,7 sekunniksi |
| Mittausten kattavuus | Ei arvioinnin liian pitkiä pose-aikaleimavälejä eikä taaksepäin siirtyvää simulaatioaikaa |
| Kesto | Enintään 150 s simulaatioaikaa ja 300 s seinäkelloaikaa |

**8° on tämän ohjelmistokokeen hyväksymisraja, ei vahvistettu fyysisen JARCRACin turvallinen kallistusraja.**

## Esityskoneella mitatut live-Gazebo-ajot (marraskuu 2026)

Testattu paketti on **`camino-safety-supervisor-v2.zip`**. Ensimmäinen versio epäonnistui liikkuvan esteen Gazebo-pose-palvelun kuittauksessa; V2 lisäsi rajatut uudelleenyritykset ja vaihtoehtoisen `set_pose`-palvelun. Vahvistusvaatimuksia tai varsinaisia turvarajoja ei höllennetty.

V2:lla tehtiin yksi yksittäinen live-ajo ja kaksi peräkkäistä `loop`-kierrosta. **Kaikki kolme päättyivät PASS-tulokseen.** Käytettiin **0,002 s** fysiikka-askelta ja **tavoite-RTF-arvoa 1,5**.

| Mittari | Yksittäinen ajo | Loop 1 | Loop 2 |
| --- | ---: | ---: | ---: |
| Lopputulos | **PASS** | **PASS** | **PASS** |
| Simulaatioaika | 122,518 s | 118,710 s | 120,198 s |
| Pienin ajoneuvo–este-jalanjälkietäisyys | 1,996 m | 1,611 m | 1,962 m |
| Suurin pituuskallistuma (pitch) | 7,499° | 7,500° | 7,499° |
| Suurin poikittaiskallistuma (roll) | 5,646° | 5,646° | 5,646° |
| Turvavalvojan `EMERGENCY_BRAKE`-ohitushavainnot | 357 | 270 | 304 |
| Gazebon vahvistamat esteen asennot | 39 | 41 | 41 |
| Pysähdys todennettu | Kyllä | Kyllä | Kyllä |
| Jatkaminen todennettu | Kyllä | Kyllä | Kyllä |
| Maali saavutettu pysähtyneenä | Kyllä | Kyllä | Kyllä |

### Mitä aikajanalla tapahtui?

| Tapahtuma | Yksittäinen ajo | Loop 1 | Loop 2 |
| --- | ---: | ---: | ---: |
| `EMERGENCY_BRAKE` alkoi | 25,724 s | 27,058 s | 25,746 s |
| `CLEARANCE_HOLD` alkoi | 42,740 s | 40,624 s | 41,030 s |
| `NAVIGATION` palautui | 43,972 s | 41,828 s | 42,234 s |

Tulokset on kirjattu **käyttäjän Gazebosta saaduista konsolilokeista**. Tässä GitHub-dokumentissa esitetyt luvut eivät korvaa alkuperäisiä `result.json`-, `evaluation.json`-, `control.jsonl`- ja `hazard.json`-tiedostoja; säilytä myös jokaisen kierroksen `safety-reports.zip`.

**Tulosten tulkinta:** Turvavalvoja keskeytti ajon todellisesti kolmessa toistossa, ja ajoneuvon pienin mitattu väli liikkuvaan esteeseen oli alimmillaankin 1,611 m. Suurin pitch oli noin 7,50°, eli vain noin 0,50° alle kokeellisen 8° rajan. Kolme onnistunutta ajoa lisää luottamusta demon esittelykelpoisuuteen, mutta ei takaa onnistumista kaikissa käynnistyksissä tai osoita todellisen koneen turvallisuutta.

Tavoite-RTF on pyydetty simulaation nopeus, **ei** mitattu todellinen suoritusaika.

## Demon käynnistys WSL Ubuntu -ympäristössä

Kun testattu ZIP on tallennettu projektin `demos/`-hakemistoon:

```bash
cd ~/camino-desktop
python3 demos/camino-safety-supervisor-v2.zip test
python3 demos/camino-safety-supervisor-v2.zip demo \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Jos ZIP on toistaiseksi vain WSL-kotihakemistossa, voit käyttää samaa jo live-testattua ajotapaa:

```bash
cd ~/camino-summit-test
python3 ~/camino-safety-supervisor-v2.zip demo \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Kahden peräkkäisen kierroksen toisto:

```bash
cd ~/camino-desktop
python3 demos/camino-safety-supervisor-v2.zip loop \
  --laps 2 \
  --physics-step 0.002 \
  --target-rtf 1.5 \
  --wall-timeout 900
```

Ohjelma luo tuoreen `/tmp/camino-safety-.../`-hakemiston. Jokaisella `loop`-kierroksella Gazebo suljetaan ja käynnistetään uudelleen: kyseessä ei ole saumaton visuaalinen silmukka. `Ctrl+C` keskeyttää ajon. Älä käytä samaa `--out`-hakemistoa uudelleen.

Paketti tarvitsee saman CAMINO-projektin Python-moduulit, Gazebo Harmonic -asennuksen ja JARCRAC-mallit kuin aiemmat demot. `prepare`-toiminnolla voi luoda erillisen maailmatiedoston tarkastelua varten, mutta se ei käynnistä liikkuvan esteen ohjainta.

**Kokeen julkaiseminen:** Tämän dokumentin lisäksi GitHubin `main`-haaraan on lisättävä juuri testattu suoritettava ZIP, `demos/camino-safety-supervisor-v2.zip`. Testatun paketin SHA-256-tarkistussumma on:

```text
e3eb4db942a22357840b59aabfd6e81bb9840e3cbf72a51ef9d4a82067cc4cba
```

Koska kyse on jo mitatusta ZIP-versiosta, sen sisäistä `README.md`-tekstiä ei päivitetä jälkikäteen: se kuvaa paketin luontihetken alustavaa validointitilaa. Tämä dokumentti sisältää sen jälkeen tehdyt live-Gazebo-tulokset.

## Demojen erot

| Demo | Pääasia |
| --- | --- |
| [SHOWCASE](JARCRAC_SHOWCASE_FI.md) | Kaksi ajoneuvoa: liittymä, perässä ajo, yhteinen este ja kuormauspaikat |
| [SUMMIT](JARCRAC_SUMMIT_FI.md) | Yksi JARCRAC: loivempi ylämäki, kiinteän kannon väistö ja reitille paluu |
| [NATIVE HILL](JARCRAC_NATIVE_HILL_FI.md) | Yksi JARCRAC: jyrkempi nousu alkuperäistä Ruskotunturin rinnettä |
| **SAFETY SUPERVISOR (tämä)** | Yksi JARCRAC: liikkuvan vaaran vuoksi normaalin ohjauksen ohittava pysäytys ja turvallisuuden toteamisen jälkeinen jatkaminen |

## Keskeiset tiedostot

- `demos/camino-safety-supervisor-v2.zip` — erillinen Python ZIP -sovellus, **kun tiedosto on lisätty GitHubiin**.
- `src/camino_safety_logic.py` (ZIP:n sisällä) — normaali reitinohjain ja erillinen turvavalvoja.
- `src/camino_safety_hazard.py` (ZIP:n sisällä) — liikkuvan esteen Gazebo-palvelukomennot ja vahvistukset.
- `src/camino_safety_scene.py` (ZIP:n sisällä) — Ruskotunturin reitti, liikkuva este ja näkymä.
- `src/camino_safety_score.py` (ZIP:n sisällä) — riippumaton, Gazebon poseihin perustuva arvioija.
- `src/camino_safety_trial.py` (ZIP:n sisällä) — prosessien käynnistäminen, hallinta ja raportointi.
- `tests/test_safety_logic.py` (ZIP:n sisällä) — turvavalvojan offline-yksikkötestit.

Lisätietoja: [CAMINO-projektin pää-README](../README.md) ja [Gazebon projektidokumentaatio](../edge-path-project/gazebo/README.md).

*Software-in-the-Loop (SIL) tarkoittaa ohjausohjelmiston kokeilemista virtuaalisilla antureilla, ajoneuvoilla ja ympäristöllä ennen mahdollista laitteistoon tai oikeaan ajoneuvoon liittyvää validointia.*
