# acisalrobotdinamik

## Masaüstü Arayüz (PyQt6)

Bu repo, Arduino tarafındaki seri komut arayüzünü kullanarak step motorları test etmek için PyQt6 tabanlı modern bir masaüstü uygulaması içerir.

### Gereksinimler

- Python 3.9+
- `pip install -r requirements.txt`

### Çalıştırma

1. Arduino kartına `acisaldinamik.ino`'yu yükleyin ve seri hızı `115200` olarak ayarlı olduğundan emin olun.
2. Bağımlılıkları kurun:

```bash
pip install -r requirements.txt
```

3. PyQt6 arayüzünü başlatın:

```bash
python ui_qt.py
```

Özellikler:
- Karanlık tema, ölçeklenebilir layout
- Parametreler için `QSpinBox/QDoubleSpinBox` doğrulaması
- Konsol penceresinde canlı Arduino çıktısı
- Acil Durdur (ABORT) ve ENA ON/OFF kontrol butonları
- Açı ve Hız için slider + spinbox; bırakınca ve sürüklerken (debounce) otomatik gönderim
- Çoklu motor seçimi (M1..M5): yeşil=seçili, kırmızı=seçili değil; seçililere toplu G/Q/MOVE/TEST
- Otomatik durum: firmware `STATUS` ile ENA/MOVING yayınlar; arayüz buton renk/ipuçlarını anlık günceller

### Net Kontroller

- BAŞLAT: Seçili motorlarda mevcut açı/hız/rampa ile ileri yönde hareket başlatır (G).
- DURDUR: Seçili motorlarda hareketi hemen keser (`STOP m`).
- SOL (−): Seçili motorlarda girilen açı kadar negatif yönde tek hamle (`MOVE m -|deg| dps`).
- SAĞ (+): Seçili motorlarda girilen açı kadar pozitif yönde tek hamle (`MOVE m +|deg| dps`).

### Kullanım

- Motor seçimi ve hareket parametrelerini (Açı, Hız, Rampa, Tekrar, Pause) girin.
- Hızlı komutlar:
  - **Motor Seç (M)**: Etkin motoru 1-5 arası seçer.
  - **Açı Ayarla (A)** ve **Hız Ayarla (V)**: Arduino'daki `setAngleDeg` ve `setSpeedDPS` değerlerini günceller.
  - **Rampa / Pause / Tekrar**: `RAMP`, `PAUSE`, `REP` komutlarını gönderir.
  - **G**: Sadece ileri hareket.
  - **Q**: İleri-geri test (REP ve PAUSE'a göre).
  - **MOVE**: Tek satırda `MOVE m deg dps` gönderir (rampa ayrı ayarlanır).
  - **TEST**: `TEST m deg dps rep pause_ms` gönderir (rampa ayrı ayarlanır).

Konsol bölümünde Arduino'nun döndürdüğü `SHOW` ve diğer çıktılar izlenebilir.

### Notlar

- Arduino kodu varsayılan olarak ENA pinlerini LOW ile aktif eder. Donanımınız ters ise `enaOn/enaOff` lojiklerini uyarlayın.
- Eğer port listesi boş görünürse sürücünüzü ve kabloyu kontrol edin; Windows'ta Aygıt Yöneticisi'nden COM numarasını doğrulayın.