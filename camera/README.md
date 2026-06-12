# Sortify — Kamera Modülü (WP4 → WP3 arayüzü)

**Sahip:** Serhat ULUDAĞ (WP4 — Communication & Control)
**Hedef kitle:** Vision ekibi (WP3 — Öykü, Serkan)
**Amaç:** Telefon kamerasından gelen frame'leri Python'da kolayca okumanız için temiz bir arayüz.

---

## Bu klasörde ne var?

| Dosya | Ne işe yarar |
|---|---|
| `camera.py` | **Asıl modül.** `Camera` sınıfı — `get_frame()` ile her çağrıda en güncel kare verir. |
| `demo_vision_consumer.py` | Modülün nasıl kullanılacağını gösteren örnek. YOLO çağrınızın yerine bakacağınız şablon. |
| `requirements.txt` | Gerekli paketler (`numpy`, `opencv-python`). |

---

## Kurulum (ilk seferlik)

### 1. Telefon → Mac/PC kamera köprüsü: Iriun Webcam

Telefonun kamerasını bilgisayara sanal webcam olarak veriyoruz. Bu olmazsa hiçbir şey çalışmaz.

- **Telefona:** App Store / Play Store'dan **Iriun Webcam** uygulamasını kurun.
- **Bilgisayara:** https://iriun.com adresinden Mac/Windows için Iriun Webcam masaüstü uygulamasını kurun.
- Her iki uygulamayı da açın. Aynı Wi-Fi'da olun **veya** telefonu USB ile bağlayın (geliştirme sırasında USB önerilir — eduroam gibi ağlar Wi-Fi keşfini engelliyor).
- Masaüstü Iriun penceresinde telefonun canlı görüntüsünü görüyorsanız köprü hazır.

> macOS'ta ilk Python çalıştırmasında "Terminal kamerayı kullansın mı?" izni çıkar → **Allow**.

### 2. Python ortamı

Python 3.12 önerilir (3.14 OpenCV ile sorunlu).

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

---

## Kullanım (vision ekibinin yazacağı kod)

```python
from camera import Camera

with Camera() as cam:
    print(f"Kamera hazır: {cam.resolution[0]}x{cam.resolution[1]}")

    while True:
        frame = cam.get_frame()    # numpy array, BGR, shape (H, W, 3)

        # Buraya YOLO / OpenCV detection kodunuz gelecek:
        # detections = detector.run(frame)
        # ...
```

Tek bilmen gereken şey **`Camera()` ve `cam.get_frame()`**. Iriun'u, OpenCV index'lerini, thread'leri merak etmene gerek yok — `camera.py` halletti.

Çalışan tam örnek için: `python3 demo_vision_consumer.py` (çıkmak için video penceresinde **q**).

---

## API özeti

```python
Camera(index: int = 1)
```
Kamerayı açar, arka planda frame okumaya başlar. Iriun varsayılan olarak index 1'e oturur; farklıysa `Camera(index=0)` veya `Camera(index=2)` deneyin.

```python
cam.get_frame() -> np.ndarray
```
**Her zaman en güncel kareyi** döndürür (BGR, shape `(H, W, 3)`). Bayat veri yok — yavaş detection yapsanız bile bir sonraki çağrıda gerçek "şu an"ı alırsınız. Bir `.copy()` döner; üstüne çizim yapmak güvenli.

```python
cam.resolution -> tuple[int, int]
```
`(width, height)` döner. Test ortamında **1920×1080** alıyoruz.

```python
cam.close()
```
Context manager kullanırsanız (`with Camera() as cam:`) otomatik çağrılır, manuel çağırmanıza gerek yok.

---

## Neden bu tasarım? (önemli — robotik için kritik)

OpenCV'nin ham `cv2.VideoCapture.read()`'i bir **buffer** tutar. Detection yavaşsa buffer eskimiş karelerle dolar ve `read()` her seferinde **eski** bir kare verir. Robotik için felaket: topun *eskiden* olduğu yere göre komut göndermiş olursunuz.

`camera.py`'nin çözümü: bir arka plan thread'i kamerayı tam hızda okur, sadece **tek bir "en son kare" slotunu** sürekli üstüne yazar. `get_frame()` ne zaman çağrılırsa o anki en güncel kareyi verir — her zaman <33 ms tazelikte (30 FPS'te).

**Pratik kural:** Bu modülde `get_frame()`'i istediğin sıklıkta çağırabilirsin. Yavaş çağırırsan eski kareler **atılır** (kuyrukta birikmez). Robotikte doğru olan davranış budur.

---

## Performans (ölçülen)

- Çözünürlük: 1920×1080
- Consumer tarafı throughput: **~50 FPS** sürekli
- Bağlantı: iPhone ↔ MacBook Air (Wi-Fi veya USB, Iriun üzerinden)

---

## Sık karşılaşılan sorunlar

| Belirti | Sebep | Çözüm |
|---|---|---|
| `CameraError: could not open camera at index 1` | Iriun masaüstü uygulaması kapalı veya başka bir uygulama (FaceTime, Zoom, Photo Booth) kamerayı tutuyor | Iriun'u açın, diğer kamera kullananları kapatın |
| Açılıyor ama `no frame available yet` | Telefondaki Iriun arka planda; iOS arka planda kamerayı kısar | Telefonda Iriun'u ön plana alın, ekran açık kalsın |
| `import cv2` çok yavaş veya takılı | `.venv` bozulmuş olabilir | `.venv`'i silip yeniden kurun: `rm -rf .venv && python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt` |
| Iriun masaüstü penceresinde görüntü kaybolur Python script çalışırken | **Normal davranış** — macOS aynı anda tek uygulamaya kamera verir. Script'i kapatınca Iriun'a geri döner. | Bir şey yapmaya gerek yok |
| Python kamerayı açıyor ama görüntü siyah / bozuk | Iriun yanlış index'te oturmuş olabilir | `Camera(index=0)` veya `Camera(index=2)` deneyin |
| `q` ile çıkamıyorum | OpenCV penceresi focused değil, Terminal focused | Önce video penceresine tıklayın, sonra `q` |

---

## Bağlantı / soru

Modülde eksik gördüğünüz, eklemenizi istediğiniz bir şey olursa (örn. frame timestamp, otomatik reconnect, çözünürlük ayarı) bana söyleyin — `camera.py` sizin için yazıldı, ihtiyaca göre genişletilebilir.

— Serhat (WP4)
