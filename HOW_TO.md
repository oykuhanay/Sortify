# Sortify — Yeni Bir Mac'te Çalıştırma Rehberi

Bu döküman, repoyu yeni bir makinede (veya aynı makinede yeniden) çalıştırmak için tüm adımları içerir. Sıralı takip et.

## 0) Donanım Hazırlığı

Çalıştırmadan önce şunlar hazır olmalı:
- **Robot açık ve menzilde** (BT05 / HM-10 modülü görünür)
- **Üstten kamera** (USB) Mac'e bağlı, kartona bakıyor
- **Karton oyun alanı** düz, 100x70 cm
- **Renkli alan(lar)** ve **küp(ler)** karton üzerinde
- **Marker robotun üstünde** (ArUco DICT_4X4_50, ID 0)

## 1) Repoyu Klonla / Çek

```bash
git clone <repo-url> sortify-comm
cd sortify-comm
git pull          # zaten klonladıysan
```

## 2) Python Ortamı

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

İlk kurulumda `ultralytics` ve `bleak` paketleri kurulurken biraz bekler.

## 3) Mac BLE İzni (sadece ilk sefer)

System Settings → Privacy & Security → Bluetooth → Terminal/iTerm'e izin ver.

## 4) Homografi Kalibrasyonu (kamera her taşındığında tekrarla)

Bu **bir kez** yapılır, sonuç `homography.npy` olarak kaydedilir. Eğer kamera, karton veya zoom değişirse tekrar yap.

```bash
python3 calibrate_homography.py
```

- Bir frame çekilir, pencere açılır
- **4 köşeyi sırayla tıkla**: sol-üst → sağ-üst → sağ-alt → sol-alt
- `S` ile kaydet (`R` ile reset, `Q` ile çıkış)
- Konsolda her köşenin world cm karşılığı yazar — (0,0), (100,0), (100,70), (0,70) bekleniyor

## 5) Ana Akışı Çalıştır

```bash
python3 main.py
```

Beklenen konsol çıktısı:
```
[WP4] Loaded homography from homography.npy.
[WP4] Robot connected (BT05). Trims set R=75.00 L=25.00.
Real-time detection started (camera 0). Press Q or ESC to quit.
```

Robot bağlanamazsa: `[WP4] Robot connect failed: BT05 not found.` → robotu kontrol et, tekrar başlat.

### Web Dashboard

`main.py` başlatıldıktan sonra tarayıcıdan **`http://127.0.0.1:8080`** aç:

- Canlı MJPEG video akışı (OpenCV penceresiyle aynı görüntü — overlay'lerle birlikte)
- Bridge state / target colour / son komut / robot bağlantı durumu / FPS
- START / RESET / STOP butonları (SPACE / R / acil stop karşılığı)
- Renk seçici (red / blue / green / auto)
- Macro butonları ("Move red cube to red field" gibi — target + START)
- Live tune slider'ları (gripper FWD / RIGHT, parallax, nadir x,y)
- "Save tunables.json" / "Reset to defaults"
- Manuel komut kutusu (`GRIP O`, `MOVE +5.00`, `TURN +010` vs. — direkt BLE'ye)

Dashboard opt-in: bağlanılmazsa veya port 8080 başkası tarafından tutuluyorsa `main.py` çökmeden devam eder.

## 6) Operatör Tuşları

Pencere üstünde:
- **SPACE** veya **S** → START (AWAITING_START'tan çıkıp akışı başlatır)
- **R** → RESET (akışı durdurur, AWAITING_START'a döner, robotu yeniden hizalamak için)
- **Q** veya **ESC** → çıkış

## 7) Tipik Bir Demo Akışı

1. `python3 main.py` çalıştır
2. State: `AWAITING_START` görünür
3. **Robotu fiziksel olarak küpe yaklaştır** (manuel hizala):
   - Magenta nokta (gripper) küpe yakın olmalı
   - Kıskaç küpe **bakıyor** olmalı
4. **SPACE** bas
5. State değişimleri:
   - `INIT_OPEN` (GRIP O gönderir) → kıskaç açılır
   - `SEEKING_BLOCK` → robot küpe yaklaşır (TURN + MOVE)
   - `GRABBING` → kıskaç kapanır (GRIP C)
   - `SEEKING_FIELD` → field'a doğru gider
   - `RELEASING` → kıskaç açılır (GRIP O), küp bırakılır
   - `BACKING_OFF` → 10cm geri çekilir
   - Loop: `SEEKING_BLOCK` (sonraki küp)

## 8) Sorun Giderme

### Cyan/magenta halka kıskaçta değil
1. Robotu kartonun ortasına koy. Halka jaw'ların arasında mı?
2. Eğer kayıyorsa, robotu **kartonun farklı köşelerine** götürüp test et
3. Köşelerde de kayıyorsa: `PARALLAX_FACTOR`'ı (`main.py` ~satır 60) ayarla
   - Halka markeri yönünde kaymışsa → arttır (0.18 → 0.22)
   - Halka kameraya doğru kaymışsa → azalt (0.18 → 0.12)
4. Sadece bir yönde kalıcı kayma varsa: `GRIPPER_FORWARD_CM` veya `GRIPPER_RIGHT_CM` ayarla

### Robot ters yöne dönüyor
- `bridge.py`'da `_tick_seeking_block` artık A*'ı bypass ediyor → direkt küpe gider
- Marker'ın robotun **önüne** bakacak şekilde yapıştırıldığını kontrol et
- Robot REPL'de (`python3 robot_repl.py`) `TURN +010` sağa, `TURN -010` sola dönüyor mu?

### Robot bir yöne kayıyor (düz gitmiyor)
- REPL aç: `python3 robot_repl.py`
- `r=75` ve `l=25` yaz (trim'i set et)
- `f` (5cm forward) → düz gidiyor mu?
- Sağa kayarsa: `l+` ile sol motor trim'ini arttır (1'er)
- Sola kayarsa: `l-` ile azalt
- Düz gidene kadar bul, doğru değeri `main.py` → `STARTUP_TRIM_LEFT`'e yaz

### Halka köşelerde kayıyor (paralaks)
- Kamera ne kadar yüksekse paralaks o kadar az
- Kamerayı yükseltebilirsen yükselt, sonra:
  - `calibrate_homography.py` tekrar çalıştır (kamera taştı çünkü)
  - `PARALLAX_FACTOR`'ı düşür (yüksek kamera = az paralaks)

### Konsola sürekli aynı TURN spam'i geliyor ama robot dönmüyor
- Motor stiction problemi. `TURN_DEG_MIN`'i (`bridge.py` ~satır 103) arttır: 10 → 15

### Halka yerinde duruyor ama robot karton dışında
- Robot karton dışında → homografi mantıksız koordinat verir
- Robotu kartonun içine koy ve **R** ile reset

## 9) Önemli Sabitler (Tüning İçin)

### `main.py`
```python
COMMAND_INTERVAL_SEC = 2.0    # iki komut arası min süre (saniye)
STARTUP_TRIM_RIGHT = 75.00    # sağ motor PWM yüzdesi
STARTUP_TRIM_LEFT  = 25.00    # sol motor PWM yüzdesi
GRIPPER_FORWARD_CM = 20.0     # marker merkezi → kıskaç jaw'larına mesafe (cm)
GRIPPER_RIGHT_CM   = 0.0      # marker merkezi → kıskaç sağa ofset (cm)
MARKER_EMA_ALPHA = 0.25       # marker okuma filtresi (düşük = daha kararlı)
CAMERA_NADIR_CM   = (50.0, 35.0)   # kameranın yere indiği nokta (cm)
PARALLAX_FACTOR   = 0.18      # marker yüksekliği düzeltmesi
```

### `bridge.py`
```python
ANGLE_TOLERANCE_DEG = 10.0    # bu hatanın altında MOVE'a geçer
WAYPOINT_REACHED_CM = 2.5     # path waypoint'i bu kadar yakınsa geç
BLOCK_GRAB_RADIUS_CM = 3.0    # küpe bu kadar yaklaşınca GRIP C
FIELD_RELEASE_RADIUS_CM = 8.0 # field'a bu kadar yaklaşınca GRIP O
MOVE_CM_MAX_BLOCK = 5.00      # küpe yaklaşırken max MOVE
MOVE_CM_MAX_FIELD = 7.00      # field'a giderken max MOVE
TURN_DEG_MIN = 10             # min TURN (motor stiction için)
TURN_DEG_MAX = 10             # max TURN
BACKOFF_CM = 10.00            # küpü bıraktıktan sonra geri çekilme
COLOR_PRIORITY = ("red", "blue", "green")
```

## 10) Dosya Yapısı

```
sortify-comm/
├── main.py                       # ana akış, kamera + vision + bridge
├── bridge.py                     # state machine (SEEKING_BLOCK → ... → SEEKING_BLOCK)
├── robot.py                      # BLE driver (Robot sınıfı)
├── robot_repl.py                 # manuel test REPL'i (trim ayarı için)
├── calibrate_homography.py       # 4-köşe kalibrasyon scripti
├── homography.npy                # piksel → cm dönüşüm matrisi
├── sortify_path_finding.py       # A* (şu an SEEKING_BLOCK'ta kullanılmıyor)
├── camera/                       # kamera modülü
├── esp_firmware/command_echo/    # ESP8266 firmware (Arduino)
├── best_finetuned.pt             # YOLO model (küp + alan detection)
└── requirements.txt
```

## 11) Bilinen Eksikler / Yapılacaklar

- **A* path takibi** SEEKING_BLOCK'ta devre dışı (path_to_block ilk waypoint'i robotun gerisinde kalabilir, robotu ters yöne çevirir). Yoldaki engeller hesaba katılmıyor; demo'da kartonun üzerinde sadece hedef küp + alan olmalı.
- **IDLE durumu**: küp confidence ~0.55 altına düşerse bridge IDLE'a geçer ve durur. Aydınlatma değişirse fark edilir.
- **Paralaks düzeltmesi** birinci dereceden lineer model; tam tepedeki bir kamera için yeter, kenarlarda 1-2cm hata kalır.
- Marker'ın yere bakan açısı sabit varsayılıyor (robot eğimli yüzeyde işe yaramaz).

## 12) İletişim Notu

Bu repo Sortify (CENG424) projesi için. WP4 (Communication & Control Infrastructure) sorumluluğu altında tüm entegrasyon — vision (YOLO), ArUco, A*, BLE driver, state machine, ESP firmware burada birleştirilmiş halde.
