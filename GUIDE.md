# 📚 ME48B Kütüphane Robotu - Ekip Geliştirici Kılavuzu

Bu kılavuz, projeyi devralacak ekip arkadaşları için hazırlanmıştır. ROS veya bu proje hakkında az bilginiz olsa bile, bu dokümanı takip ederek sistemi çalıştırabilir ve geliştirebilirsiniz.

---

## 🎯 Proje Ne Yapıyor?

Bir kütüphanede çalışan otonom forklift robotu simülasyonu:

1. **Masalarda kitap bekler** (3 farklı istasyon)
2. **Kitabı algılar ve yaklaşır** (QR kod ile)
3. **Forklift ile kitabı alır**
4. **Doğru rafa götürür** (QR koduna göre)
5. **Rafa bırakır ve döner**
6. **Sonsuz döngüde çalışır** (Demo Mode)

---

## 🚀 Sistemi Çalıştırma (En Kolay Yol)

### Adım 1: Kontrol Panelini Aç
```bash
cd ~/my_ros_project
python3 scripts/control_panel.py
```

Bu bir GUI penceresi açar. Tüm işlemleri buradan yapabilirsiniz.

### Adım 2: GUI'deki Butonları Sırayla Tıkla

| Sıra | Buton | Ne Yapar |
|------|-------|----------|
| 1 | **🌍 LAUNCH WORLD** | Gazebo + Robot + Navigasyon başlatır |
| 2 | **🛠️ Setup Stations** | 3 adet kitap bırakma istasyonu koyar |
| 3 | **📚 Spawn S1/S2/S3** | Seçtiğin renkte kitap koyar |
| 4 | **🚀 START MISSION** | Robotu çalıştırır! |

### Adım 3: İzle!
Robot otomatik olarak:
- Kitabı bulur
- Yaklaşır (Visual Servoing)
- Forklift ile alır
- Rafa götürür
- Bırakır ve geri döner

---

## 📁 Önemli Dosyalar ve Ne İşe Yaradıkları

### Scripts (Python Kodları)

| Dosya | Görevi | Ne Zaman Değiştirirsiniz |
|-------|--------|--------------------------|
| `control_panel.py` | GUI kontrol paneli | Yeni buton eklemek için |
| `library_mission.py` | **ANA GÖREV KODU** - Robot mantığı | Davranış değiştirmek için |
| `qr_reader.py` | Kameradan QR kod okur | Yeni QR formatı eklemek için |
| `setup_environment.py` | Kitap/istasyon spawn eder | Yeni obje eklemek için |
| `clean_world.py` | Dünyayı temizler | - |

### Launch Dosyaları

| Dosya | Ne Başlatır |
|-------|-------------|
| `full_system.launch` | **HER ŞEY** (Gazebo+Robot+Nav+RViz) |
| `navigation.launch` | Sadece AMCL + move_base |
| `library_with_robot.launch` | Gazebo + Robot |

### Config Dosyaları

| Dosya | Ayarladığı Şey |
|-------|----------------|
| `dwa_local_planner.yaml` | Robot hızı, dönüş hızı |
| `costmap_common.yaml` | Engel algılama mesafesi |
| `amcl.yaml` | Lokalizasyon hassasiyeti |

### Model Dosyaları

| Klasör | İçerik |
|--------|--------|
| `models/book_red/` | Kırmızı kitap (SHELF_A) |
| `models/book_green/` | Yeşil kitap (SHELF_C) |
| `models/book_blue/` | Mavi kitap (SHELF_B) |
| `models/dropoff_station/` | Kitap bırakma istasyonu |
| `urdf/turtlebot3_forklift.urdf.xacro` | **ROBOT TANIMI** |

---

## 🔧 Sık Karşılaşılan Sorunlar ve Çözümleri

### ❌ "roscore bulunamadı" / "Master online değil"
```bash
# Önce roscore'u başlat
roscore &
# Sonra paneli aç
python3 scripts/control_panel.py
```

### ❌ Gazebo açılmıyor / Donuyor
```bash
# Her şeyi öldür ve yeniden başla
killall -9 gzserver gzclient rviz rosmaster roscore xterm python3
# Tekrar başlat
python3 scripts/control_panel.py
```

### ❌ Robot hareket etmiyor
- RViz'de "2D Nav Goal" vermeyi deneyin
- Terminal'de hata mesajlarını kontrol edin
- `rostopic echo /cmd_vel` ile komut gidip gitmediğine bakın

### ❌ QR kod algılanmıyor
- Kamera görüntüsünü kontrol edin: `rqt_image_view /custom_camera/image_raw`
- Kitap spawn edildi mi kontrol edin
- QR kod kitabın ön yüzünde mi?

### ❌ Forklift çalışmıyor
```bash
# Manuel test
rostopic pub /fork_joint_position_controller/command std_msgs/Float64 "data: 1.0"
```

---

## 📐 Önemli Koordinatlar

Robotun gittiği yerler `library_mission.py` içinde tanımlı:

```python
STATIONS = {
    1: {'pickup': (-0.1, 0.8, 0.0, 1.0), 'book_model': 'book_s1'},
    2: {'pickup': (4.8, 0.4, 0.0, 1.0), 'book_model': 'book_s2'},
    3: {'pickup': (-3.2, 2.8, 0.7, 0.7), 'book_model': 'book_s3'},
}

SHELVES = {
    'SHELF_A': (1.35, 1.45, 0.7, 0.7),
    'SHELF_B': (3.2, -0.55, 0.0, 1.0),
    'SHELF_C': (1.35, 2.6, 0.7, 0.7),
}

LOC_IDLE = (0.75, 0.85, 0.0, 1.0)  # Bekleme noktası
```

Koordinat formatı: `(x, y, orientation_z, orientation_w)`

---

## 🛠️ Nasıl Değişiklik Yapılır?

### Yeni Kitap Rengi Eklemek
1. `models/` altında yeni klasör oluştur (örn: `book_yellow/`)
2. `model.sdf` ve `model.config` dosyalarını kopyala
3. SDF'te rengi değiştir: `<name>Gazebo/Yellow</name>`
4. `setup_environment.py`'deki `model_name_map`'e ekle
5. `control_panel.py`'deki Combobox'a ekle

### Robot Hızını Değiştirmek
`config/dwa_local_planner.yaml`:
```yaml
max_vel_x: 0.3      # İleri hız (m/s)
max_vel_theta: 1.5  # Dönüş hızı (rad/s)
```

### Yeni İstasyon Eklemek
1. `library_mission.py`'de `STATIONS` dict'ine ekle
2. `setup_environment.py`'de koordinatları ekle
3. `control_panel.py`'ye yeni buton ekle

---

## 🧪 Test Etme

### Manuel Navigasyon Testi
```bash
# Terminalde
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# WASD tuşlarıyla robotu kontrol et
```

### QR Kod Testi
```bash
# QR okuyucuyu başlat
python3 scripts/qr_reader.py

# Başka terminalde dinle
rostopic echo /detected_qr
```

### Forklift Testi
```bash
# Yukarı
rostopic pub /fork_joint_position_controller/command std_msgs/Float64 "data: 1.0"

# Aşağı
rostopic pub /fork_joint_position_controller/command std_msgs/Float64 "data: 0.05"
```

---

## 📊 ROS Topic'leri

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/cmd_vel` | Twist | Robot hareket komutu |
| `/odom` | Odometry | Robot pozisyonu |
| `/scan` | LaserScan | LiDAR verisi |
| `/custom_camera/image_raw` | Image | Kamera görüntüsü |
| `/detected_qr` | String | Algılanan QR kod değeri |
| `/fork_joint_position_controller/command` | Float64 | Forklift yüksekliği |
| `/move_base/goal` | MoveBaseActionGoal | Navigasyon hedefi |

---

## 🏗️ Mimari Özet

```
┌─────────────────────────────────────────────────────────────┐
│                     Control Panel (GUI)                      │
│  [Launch] [Setup] [Spawn] [Mission] [Stop]                  │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    library_mission.py                        │
│  ┌──────┐   ┌──────────┐   ┌────────┐   ┌─────────┐        │
│  │ IDLE │──►│ PICKUP   │──►│ TRAVEL │──►│ DROPOFF │───┐    │
│  └──────┘   └──────────┘   └────────┘   └─────────┘   │    │
│      ▲                                                 │    │
│      └─────────────────────────────────────────────────┘    │
└─────────────────────────┬───────────────────────────────────┘
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
    ┌──────────┐   ┌──────────┐   ┌──────────────┐
    │ move_base│   │qr_reader │   │ Forklift Pub │
    │(Nav Stack)│   │(OpenCV)  │   │ (Joint Ctrl) │
    └──────────┘   └──────────┘   └──────────────┘
          │               │               │
          └───────────────┼───────────────┘
                          ▼
                ┌─────────────────┐
                │     Gazebo      │
                │  (Simulation)   │
                └─────────────────┘
```

---

## 📝 Geliştirme Notları

### Visual Servoing Nasıl Çalışıyor?
`library_mission.py` → `visual_servoing_approach()`:
- QR kodu kamerada merkezlemek için robot döner (`angular.z`)
- QR kodu dikey merkezlemek için forklift yükselir/alçalır
- QR yeterince büyük görününce durur (yaklaştı demek)

### Magnet (Kitap Tutma) Nasıl Çalışıyor?
Gazebo'da fiziksel tutma zor, bu yüzden "sihirli" yöntem:
- `self.carrying_book = True` olunca timer başlar
- Timer her 0.1 saniyede kitabı robotun önüne "ışınlar"
- Bu sayede kitap forklift ile birlikte hareket eder

### Neden Forklift Hitbox'ı Kapalı?
Gazebo fiziği bazen çıldırıyor. Hitbox açıkken:
- Kitapla çarpışıp "patlıyor"
- Robot titriyor
Bu yüzden `urdf/turtlebot3_forklift.urdf.xacro`'da collision kapalı.

---

## ✅ Proje Durumu

| Özellik | Durum |
|---------|-------|
| Otonom Navigasyon | ✅ Çalışıyor |
| QR Kod Okuma | ✅ Çalışıyor |
| Visual Servoing | ✅ Çalışıyor |
| Forklift Kontrolü | ✅ Çalışıyor |
| Kitap Alma/Bırakma | ✅ Çalışıyor |
| Sürekli Döngü | ✅ Çalışıyor |
| Renk Seçimi (GUI) | ✅ Çalışıyor |
| Dinamik Engel | ❌ Yapılmadı |

---

## 🆘 Yardıma İhtiyacınız Olursa

1. **Terminal'deki hata mesajlarını okuyun** - Genelde çözüm orada
2. **`rostopic list`** ile aktif topic'leri görün
3. **`rosnode list`** ile çalışan node'ları görün
4. **RViz açın** ve robotun haritadaki konumunu kontrol edin
5. **Google'da hata mesajını arayın** - ROS topluluğu çok aktif

---

*Son güncelleme: 2026-01-04*
*Hazırlayan: ME48B Proje Ekibi*
