# HIL-10: mittaustiedon tuoreus ja katkostestit

Päiväys: 29.10.2026. Pohja: `hil-development`, commit
`52331fde756991988cbb94f01e6d33d615c8d7df`.

Tämä paketti lisää HIL-10-testit nykyisen HIL-9-toteutuksen rinnalle.
HIL-9:n skriptejä, maailmoja, R2100-ohjelmia ja vanhoja tuloksia ei korvata.
Korjattu toiminta on käytössä, kun ajat tässä ohjeessa mainittuja **HIL-10-skriptejä**.
Vanhojen HIL-9-skriptien ajaminen käyttää edelleen niiden vanhaa toimintaa.

## 1. Mitä on korjattu

- B:n odometrian ja jarruviestin suurin sallittu paikallinen ikä on 0,50 s.
  Puuttuva tai vanhentunut tieto estää B:n ajokomennon.
- Yksi nopeusnäyte lähetetään R2100:lle korkeintaan kerran. Vanhaa nopeutta ei
  toisteta uutena CAN-nopeuspäivityksenä. R2100:n nykyinen anturikatkon valvonta
  pääsee näin toimimaan myös nopeusmittauksen katketessa.
- Mittauksen Gazebo-aikaleiman on edettävä, jos viesti sisältää aikaleiman.
  Tuoreuden mittaus käyttää PC:n monotonista kelloa. Eri koneiden kelloja ei verrata.
- Virheelliset, toistuvat ja väärässä järjestyksessä saapuvat jarruvastaukset
  eivät päivitä jarrutiedon tuoreutta. Laskurin 255 -> 0 -vaihtuminen sallitaan.
- Liikkeellelähdön jälkeen havaittu tiedonsiirtovika lukitsee pysäytyksen kyseisen
  testiajon loppuun. Hetkellinen palautuminen ei vapauta jarrua automaattisesti.
- Pysähtyminen vaatii vähintään kolme uutta peräkkäistä odometriaviestiä, joiden
  nopeus on enintään 0,05 m/s ja jotka saapuvat pysäytyskomennon jälkeen.
  Saman välimuistiarvon lukeminen monta kertaa ei täytä ehtoa. Nopeutena käytetään
  lineaarisen nopeusvektorin suuruutta, myös sivuttais- ja pystysuunnassa.
- B:n reittitesti vahvistaa pysähtymisen ennen PASS-tulosta. Hybriditesti säilyttää
  Phase 6:n hyväksymisehdot ja lisää B:n pysähtymisen varmistuksen. HIL-vian
  keskeyttäessä hybridiajon molemmille ajoneuvoille lähetetään nollakomento.
- Jokainen ajo tallentaa oman CSV-jäljen ja JSON-yhteenvedon hakemistoon
  `edge-path-project/gazebo/results/hil10/`. Vanhaan SIL-tulos-CSV:hen ei kirjoiteta.

1,00 m:n AEB-turvaetäisyyttä tai R2100:n jarrutuskynnyksiä ei muuteta.

## 2. Asennus ja ohjelmistotarkistus WSL:ssä

Sulje käynnissä olevat testiohjaimet ja Gazebo. Pura ZIP projektin juureen
`~/edge-path-project-git`, niin että paketin `edge-path-project` ja `tests`
yhdistyvät olemassa oleviin samannimisiin hakemistoihin. Tiedostot ovat uusia;
paketti ei sisällä HIL-9:n lähdetiedostojen korvaavia kopioita.

Suorita projektin juuressa:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/hil10_preflight.py
python3 -m unittest discover -s tests -p 'test_hil10*.py' -v
```

Odotettu tulos: lähtöversion riippuvuudet `OK` ja **42 ohjelmistotestiä / OK**.
Tarkistus lukee tiedostoja, eikä muuta niitä. `DIFFERS` tai `MISSING` tarkoittaa,
että paikallinen lähtöversio pitää tarkistaa ennen laitteistoajoa. Älä poista
omia muutoksiasi tai tuloksiasi tarkistuksen vuoksi.

Ohjelmistotestit eivät tarvitse Gazeboa eivätkä CAN-laitetta. Niissä käytetään
oikeaa tuoreusvalvontaa ja testien ajosilmukkaa, mutta ajan kulku, Gazebo-viestien
tuonti ja ajoneuvon vaste korvataan hallituilla testivastineilla. R2100:n nykyistä
AEB-päätöslogiikkaa testataan suoraan ilman CAN-liitäntää. Nämä testit eivät ole
uusia Gazebo–R2100-laitteistotuloksia.

## 3. Ajojärjestys ja maailmat

Tee jokainen taulukon rivi omana ajonaan. Käynnistä **Gazebo ja kaikki kolme
R2100-prosessia uudelleen jokaisen ajon välissä**. R2100:n AEB-lukitus ja
testin vikamuisti nollautuvat vain uudessa ajossa. Älä käytä Gazebon reset-painiketta
kesken testin; aikaleiman palaaminen taakse tulkitaan vanhaksi mittaukseksi.

| Järjestys | Koe | Gazebo-maailma | Testi / valinta |
|---|---|---|---|
| 1 | B:n normaali reitti ja lopullinen pysähtyminen | Phase 6 | B-testi, `--mode route` |
| 2 | Kahden ajoneuvon hybridiajo (HIL-9D:n uusinta) | Phase 6 | hybriditesti |
| 3 | Esteen aiheuttama AEB (HIL-9E:n uusinta) | HIL-9E:n estemaailma | B-testi, `--mode aeb` |
| 4 | B:n ohjaimelle tulevan odometrian katko | Phase 6 | B-testi, `--mode fault-odometry` |
| 5 | Palautuvien jarruviestien katko | Phase 6 | B-testi, `--mode fault-brake` |
| 6 | R2100:lle lähetettävän LiDAR-datan katko | Phase 6 | B-testi, `--mode fault-lidar` |

HIL-10 lähettää B:n raakamuotoisen LiDAR-datan itse. **Älä käynnistä erillistä
`gazebo_lidar_sender.py`-ohjelmaa.** Myöskään vanha HIL-9-ohjain tai toinen
B:n komentoja lähettävä prosessi ei saa olla käynnissä samanaikaisesti.

### Terminaali 1: WSL / Gazebo

Phase 6 -maailma reitti-, hybridi- ja katkostesteille:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_phase6_two_vehicle_world.sdf
```

Vain AEB-kokeessa (`--mode aeb`) käynnistä sen sijasta:

```bash
cd ~/edge-path-project-git
export GZ_SIM_RESOURCE_PATH="$PWD/edge-path-project/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf
```

### Terminaali 2: R2100

Käytä HIL-9:ssä toimineita ohjelmia ja fyysistä CAN0–CAN1-yhteyttä. Molempien
liitäntöjen nopeus on 500 kbit/s. Tarkista tila:

```bash
ip -details link show can0
ip -details link show can1
```

Jos liitännät eivät ole valmiina, tee nykyisen `edge-path-project/hil/README.md`:n
CAN-asetukset. Paketti ei edellytä R2100:n lähdekoodin muuttamista.

Pysäytä edellisen testin kolme R2100-prosessia ennen uutta käynnistystä.
Jos ne käynnistettiin saman terminaalin taustatöinä, jäljempänä tallennetut PID:t
mahdollistavat juuri niiden pysäyttämisen. Ensimmäisellä kerralla pysäytä vanhat
HIL-9-prosessit niistä terminaaleista tai istunnoista, joissa ne käynnistit.

```bash
cd ~/EdgeDetection/edge-path-project/hil
HIL10_LOG_DIR="$HOME/hil_test_logs/hil10_$(date -u +%Y%m%dT%H%M%S)"
mkdir -p "$HIL10_LOG_DIR"

python3 -u r2100_lidar_perception.py > "$HIL10_LOG_DIR/perception.log" 2>&1 &
HIL10_PERCEPTION_PID=$!
python3 -u r2100_can_gateway.py --external-perception > "$HIL10_LOG_DIR/gateway.log" 2>&1 &
HIL10_GATEWAY_PID=$!
python3 -u r2100_aeb_controller.py > "$HIL10_LOG_DIR/aeb.log" 2>&1 &
HIL10_AEB_PID=$!
```

`--external-perception` on tarpeellinen: LiDAR-havaintoprosessi omistaa CAN-signaalin
`0x101`. Gateway välittää B:n nopeuden `0x100`-signaalina ja palauttaa `0x200`-jarrutiedon.
Ennen PC-testin käynnistymistä puuttuva anturidata ja `sensor_timeout` ovat odotettu tila.

### Terminaali 3: WSL / HIL-10-testi

Kysy nykyinen IP-osoite kerran tässä terminaalissa:

```bash
cd ~/edge-path-project-git
read -r -p "R2100:n IP-osoite: " R2100_IP
```

Ensimmäinen normaali B:n reittiajo:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode route --r2100-host "$R2100_IP"
```

Toinen koe, kahden ajoneuvon hybridiajo:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_hybrid_test.py --r2100-host "$R2100_IP"
```

Kolmas koe, AEB estemaailmassa:

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode aeb --r2100-host "$R2100_IP"
```

Katkostestien kolme erillistä käynnistyskomentoa ovat alla. Aja ne **yksi kerrallaan**;
tee jokaisen välissä Gazebon ja R2100-prosessien uudelleenkäynnistys.

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode fault-odometry --r2100-host "$R2100_IP"
```

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode fault-brake --r2100-host "$R2100_IP"
```

```bash
python3 -u edge-path-project/gazebo/scripts/ruskotunturi_hil10_b_test.py --mode fault-lidar --r2100-host "$R2100_IP"
```

Testi odottaa alussa enintään 12 sekuntia tuoretta asentoa, odometriaa ja R2100:n
DRIVE-tilaa. Auto ei saa lupaa ajaa ennen näitä. Väärä lähtöpaikka tuottaa
`startup_not_ready`-tuloksen ja pyynnön käynnistää maailma uudestaan.

### Ajon jälkeen ja ennen seuraavaa koetta

PC-testi lopettaa oman LiDAR-lähettimensä ja lähettää pysäytyskomennot.
Sulje Gazebo terminaalissa 1. Pysäytä juuri käynnistetyt R2100-prosessit
terminaalissa 2 ja odota niiden sulkeutumista:

```bash
kill -INT "$HIL10_PERCEPTION_PID" "$HIL10_GATEWAY_PID" "$HIL10_AEB_PID"
wait "$HIL10_PERCEPTION_PID" "$HIL10_GATEWAY_PID" "$HIL10_AEB_PID"
```

Säilytä kunkin ajon R2100-lokihakemisto. Seuraavaa koetta varten käynnistä oikea
maailma, aloita R2100:n kolme prosessia uudestaan ja käynnistä sitten PC-testi.

## 4. Katkostestin PASS-ehdot

Katko injektoidaan oletuksena 2 sekuntia ensimmäisen havaitun liikkeen jälkeen,
kun B:n mittaus näyttää edelleen vähintään 0,20 m/s ja ajokomento on positiivinen.
Viivettä voi muuttaa valinnalla `--fault-after`, mutta ensimmäiset kokeet tehdään
oletusarvolla vertailukelpoisuuden vuoksi.

| Katko | Mitä katkaistaan | Odotettu tunnistus |
|---|---|---|
| `fault-odometry` | Odometriapäivitykset HIL-komentoportille ja nopeuden lähetykselle | Paikallinen `odometry_timeout`; R2100:n `sensor_timeout` voidaan havaita ensin |
| `fault-brake` | PC:llä vastaanotettavien jarruvastausten hyväksyminen | Paikallinen `brake_timeout` |
| `fault-lidar` | B:n raakamuotoisen LiDAR-datan lähettäminen R2100:lle | R2100:lta tuore jarruvastaus, syy `sensor_timeout` |

Kaikissa kolmessa kokeessa vaaditaan:

1. B on ensin liikkunut ja vasta sitten on injektoitu valittu katko.
2. Oikea vika havaitaan viimeistään 1,0 sekunnissa injektiosta.
3. Havaitsemisen jälkeen sekä nopeus- että kääntökomento pysyvät nollassa.
4. Pysähtyminen vahvistetaan kolmesta uudesta mittauksesta viimeistään
   3,0 sekunnissa injektiosta.
5. Vahvistettu pysähtyminen säilyy vähintään 1,0 sekunnin ajan ja mittaukset
   pysyvät tuoreina.

Odotettu onnistumisen syy on `fault_stop_and_hold_verified`. Normaali reitti
hyväksytään syyllä `route_complete`, hybridi syyllä `hybrid_complete` ja AEB
syyllä `aeb_stop_verified`. Pelkkä vikailmoitus tai saapuminen maaliin ei riitä
katkostestin PASS-tulokseen. Ennen injektiota ilmenevä vika on FAIL.

Odometriakatkossa erillinen varmennuslukija lukee edelleen saman Gazebo-ajoneuvon
odometriaa. Se ei anna nopeustietoa ohjaimelle. Näin ohjaimen mittauskatko ja
ajoneuvon todellinen simuloitu liike voidaan tarkistaa erikseen. Jos itse Gazebon
odometrialähde katkeaa kokonaan, pysähtymistä ei voi vahvistaa ja koe on FAIL.

Nämä ovat hallittuja ohjelmallisia katkoksia. Ne eivät sellaisenaan testaa
fyysisen CAN-kaapelin irrotusta, CAN bus-off -tilaa, kaikkia pakettien viiveitä,
prosessin pakkotappoa tai PC:n virrankatkoa. Prosessin ulkopuolinen komentojen
watchdog on erillinen jatkokehitysasia. Tämä paketti ei ota sitä käyttöön.

## 5. Tuloslokit ja vianrajaus

Jokaisesta ajosta muodostuu pari:

- `hil10_<mode>_<aikaleima>_<tunniste>.csv`: mittausikä, nopeus, pyydetty ja
  käytetty komento, jarrun syy/laskuri, vikamuisti ja pysähtymisnäytteet.
- Samanniminen `.json`: PASS/FAIL, päättymissyy, pysähtymisen vahvistus sekä
  testikohtaiset tulokset. Katkokokeissa mukana ovat tunnistus- ja pysähtymisviive;
  AEB:ssä turvaetäisyys; hybridissä Phase 6:n hyväksymisehdot.

CSV:n monotoniset sekunnit ovat PC:n paikallista aikaa eivätkä kalenteriaikaa.
JSONin `baseline_commit` kertoo pohjaversion; HIL-10-muutokset ovat tämän
erikseen jaetun paketin paikallisia tiedostoja. Paketti ei tee Git-committia tai pushia.

```bash
cd ~/edge-path-project-git/edge-path-project/gazebo/results/hil10
ls -lt
explorer.exe .
```

Jos koe epäonnistuu, säilytä JSON ja saman ajon CSV sekä R2100:n kolme lokia.
`startup_not_ready`-tilanteessa tarkista ensin, että maailma käy, IP-osoite on
oikea, R2100:n prosessit ovat käynnissä ja toinen LiDAR-lähetin tai ohjain ei ole
päällä. Tarkastele R2100:lla esimerkiksi:

```bash
tail -n 20 "$HIL10_LOG_DIR/perception.log" "$HIL10_LOG_DIR/gateway.log" "$HIL10_LOG_DIR/aeb.log"
```

LiDAR-lokissa pitäisi näkyä kokonaisia 721-säteen skannauksia ennen vian injektiota.
`stop_not_verified` ja `verification_odometry_timeout` eivät ole hyväksyttäviä
pysähtymisiä, vaikka auto näyttäisi kuvassa paikallaan olevalta.

## 6. Valmistumiskriteeri

Ohjelmistotarkistukset: **42/42 PASS** tämän paketin valmistuksessa.
Gazebo + R2100 + fyysinen CAN: **ei vielä ajettu tällä HIL-10-versiolla**.

Seuraava välitavoite on kuuden taulukossa nimetyn kokeen onnistunut ajo omalla
laitteistolla, jokaisesta CSV/JSON-pari ja R2100-lokit. Vasta niiden perusteella
HIL-10 voidaan kirjata laitteistolla varmennetuksi vaiheeksi ja viedä hyväksytyt
tiedostot sekä tulokset `hil-development`-haaraan.
