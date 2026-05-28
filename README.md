# Moodul 03: Roboti simulatsioon Webotsis

> **Järeleaitamine** — see repo on mõeldud tudengitele, kes teevad ülesande hiljem.
> Täpsem õppematerjal on Moodle'is: **Moodul 03: Roboti simulatsioon Webotsis**.

---

## Samm 1 — Forki see repo

1. Mine selle repo lehele GitHubis
2. Kliki paremal ülal nuppu **Fork**
3. Vali oma GitHub konto
4. Kliki **Create fork**

Sinu isiklik koopia tekib:
```
https://github.com/SINU-KASUTAJANIMI/TRO013-03-Webots-jarel
```

---

## Samm 2 — Klooni konteinerisse

Ava noVNC terminal (`http://SERVER_IP:33000+N`) ja käivita:

```bash
cd /workspace/ros2_ws/src
git clone https://github.com/SINU-KASUTAJANIMI/TRO013-03-Webots-jarel.git
cd TRO013-03-Webots-jarel
```

---

## Samm 3 — Tee ülesanne

Sinu ülesanne on kirjutada `folkrace_driver.py` — reaktiivne sõlm, mis loeb lidarit ja juhib robotit.

**Faili asukoht repos on sinu valida**, nt:
```
folkrace_driver/folkrace_driver/folkrace_driver.py   (ROS2 paketina)
folkrace_driver.py                                    (lihtne skript)
```

**Lidar indeksid (720 kiirt):**
```
ranges[0]   = -180° = otse TAGA
ranges[180] =  -90° = PAREMALE
ranges[360] =    0° = otse ETTE
ranges[540] =  +90° = VASAKULE
```

**Käivitamine:**
```bash
# Terminal 1 — simulatsioon
source /opt/mobros_ws/install/setup.bash
ros2 launch yahboom_webots webots.launch.py

# Terminal 2 — sinu sõlm
python3 folkrace_driver.py
# või
ros2 run folkrace_driver folkrace_driver
```

**Nõuded:**
- Robot teeb vähemalt **ühe täisringi**
- Robot **ei põrka seintesse**
- Robot suudab üle **silla** minna

---

## Samm 4 — Commit ja push

```bash
cd /workspace/ros2_ws/src/TRO013-03-Webots-jarel
git add .
git commit -m "Moodul 03: folkrace_driver implementeeritud"
git push
```

> Git küsib parool — kasuta GitHubi **Personal Access Token**:
> GitHub → Settings → Developer settings → Personal access tokens → Generate new token (scopes: `repo`)

---

## Samm 5 — Vaata hindamistulemusi

Pärast push-i käivitub automaatne hindamine (~1-2 min).

**Vaata tulemusi:** oma repo → **Actions** → viimane töö

Hindamine otsib `folkrace_driver.py` faili **kõikidest kaustadest** (nimi loeb, tee ei loe) ja kontrollib:
| Test | Punktid |
|------|---------|
| `folkrace_driver.py` eksisteerib | 5 p |
| Python süntaks korrektne | 10 p |
| Impordib `LaserScan` ja `Twist` | 5 p |
| Lidar lugemine implementeeritud (mitte stub) | 15 p |
| Vasak/parem lugemine implementeeritud | 10 p |
| Reaktiivne loogika olemas (`linear.x`, `angular.z`) | 15 p |
| Tingimuse loogika olemas (takistuse vältimine) | 10 p |
| `package.xml` ja `setup.py` olemas | 5 p |
| TODO-d lahendatud | 10 p |

---

## Küsimused ja lisainfo

Kui midagi on ebaselge või vajad täiendavat selgitust, vaata Moodle kursuse **Moodul 03: Roboti simulatsioon Webotsis** peatükki — seal on üksikasjalikud juhised, näidiskood ja selgitused.
