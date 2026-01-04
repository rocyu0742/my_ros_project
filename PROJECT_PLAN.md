# ME48B - Kütüphane Kitap Organizasyon Robotu

## 📋 Proje Özeti

**Ders:** ME48B - Introduction to Mobile Robotics  
**Proje Adı:** Library Book Organization Robot  
**Ortam:** Kütüphane Simülasyon Ortamı (Gazebo)

### Görev Tanımı
Bir kütüphanede çalışan otonom mobil robot:
1. Sabit bir "kitap bırakma noktasına" gider
2. Kullanıcının bıraktığı kitabı forklift mekanizmasıyla alır
3. Kitabın üzerindeki QR kodu okur
4. QR koduna göre hangi rafa gideceğini belirler
5. İlgili rafa gidip kitabı yerleştirir
6. Başlangıç noktasına döner veya yeni kitap bekler

### Kapsam (MVP - Minimum Viable Product)
- [x] Tek robot
- [x] 3-5 farklı kitap türü (farklı QR kodları)
- [x] Her kitap farklı bir rafa gidecek
- [x] Sabit kitap bırakma noktası
- [x] Aynı anda 1 kitap taşıma
- [x] Statik ve dinamik engelden kaçınma
- [x] Önceden oluşturulmuş harita ile navigasyon

---

## 🤖 Sistem Mimarisi

### Robot: TurtleBot3 Burger/Waffle + Forklift

```
┌─────────────────────────────────────────────────────────┐
│                    TurtleBot3 + Forklift                │
├─────────────────────────────────────────────────────────┤
│  Sensörler:                                             │
│  ├── LiDAR (360°) ──────────► Haritalama & Navigasyon  │
│  ├── Kamera (Ön) ───────────► QR Kod Okuma            │
│  ├── Ultrasonik (Ön) ───────► Yakın Mesafe Algılama   │
│  └── IMU ───────────────────► Odometri Düzeltme       │
│                                                         │
│  Aktüatörler:                                           │
│  ├── 2x Tekerlek Motoru ────► Hareket                  │
│  └── Forklift Mekanizması ──► Kitap Alma/Bırakma      │
└─────────────────────────────────────────────────────────┘
```

### Yazılım Stack

| Katman | Teknoloji | Açıklama |
|--------|-----------|----------|
| Simülasyon | Gazebo 11 | Fizik motoru ve 3D görselleştirme |
| Middleware | ROS Noetic | Robot Operating System |
| Haritalama | GMapping | SLAM ile harita oluşturma |
| Lokalizasyon | AMCL | Adaptive Monte Carlo Localization |
| Navigasyon | move_base | Global + Local Planner |
| QR Okuma | OpenCV + pyzbar | Görüntü işleme |
| State Machine | smach veya custom | Görev yönetimi |

---

## 📁 Proje Dosya Yapısı

```
me48b_library_robot/
├── PROJECT_PLAN.md              # Bu dosya
├── TASKS.md                     # Görev takip listesi
├── README.md                    # Proje açıklaması
├── launch/
│   ├── bookstore.launch         # Mevcut world launch
│   ├── robot_spawn.launch       # Robot spawn
│   ├── mapping.launch           # GMapping için
│   ├── navigation.launch        # AMCL + move_base
│   └── full_system.launch       # Tüm sistem
├── models/
│   ├── turtlebot3_forklift/     # Özelleştirilmiş robot modeli
│   ├── book_red/                # QR kodlu kitap modeli
│   ├── book_blue/
│   ├── book_green/
│   └── dropoff_station/         # Kitap bırakma istasyonu
├── maps/
│   ├── bookstore_map.yaml       # Oluşturulacak harita
│   └── bookstore_map.pgm
├── config/
│   ├── costmap_common.yaml      # Costmap ayarları
│   ├── global_costmap.yaml
│   ├── local_costmap.yaml
│   ├── base_local_planner.yaml  # DWA Planner
│   └── amcl.yaml                # AMCL parametreleri
├── scripts/
│   ├── qr_reader.py             # QR kod okuma node
│   ├── book_manager.py          # Kitap alma/bırakma logic
│   ├── mission_controller.py    # Ana görev yönetimi
│   └── forklift_controller.py   # Forklift kontrolü
├── src/
│   └── (C++ nodes if needed)
├── urdf/
│   └── turtlebot3_forklift.urdf.xacro  # Robot tanımı
├── worlds/
│   └── bookstore.world          # Kütüphane ortamı
├── rviz/
│   └── navigation.rviz          # Görselleştirme config
├── msg/
│   └── BookInfo.msg             # Özel mesaj tipleri
└── srv/
    └── PickupBook.srv           # Özel servisler
```

---

## 🗺️ Proje Aşamaları (Roadmap)

### Faz 1: Ortam Hazırlığı ✅ (Tamamlandı)
- [x] ROS Noetic kurulumu
- [x] Gazebo 11 kurulumu
- [x] Kütüphane ortamının çalıştırılması
- [x] DISPLAY=:0 ayarı (WSLg)

### Faz 2: Robot Modeli ✅ (Tamamlandı)
- [x] TurtleBot3 paketlerinin kurulumu
- [x] TurtleBot3'ün kütüphane ortamına spawn edilmesi
- [x] Forklift mekanizmasının URDF'e eklenmesi
- [x] Forklift joint controller'ının ayarlanması
- [x] Sensörlerin (LiDAR, kamera, ultrasonik, IMU) eklenmesi/doğrulanması

### Faz 3: Haritalama - GMapping ✅ (Tamamlandı)
- [x] GMapping paketinin kurulumu
- [x] Teleop ile robotun manuel kontrolü
- [x] Kütüphanenin haritasının çıkarılması
- [x] Haritanın kaydedilmesi (map_saver)

### Faz 4: Lokalizasyon - AMCL ✅ (Tamamlandı)
- [x] AMCL paketinin yapılandırılması
- [x] Kaydedilen harita üzerinde lokalizasyon testi
- [x] Particle filter parametrelerinin ayarlanması

### Faz 5: Navigasyon - move_base ✅ (Tamamlandı)
- [x] move_base paketinin yapılandırılması
- [x] Global planner (navfn veya global_planner) ayarları
- [x] Local planner (DWA) ayarları
- [x] Costmap parametrelerinin ayarlanması
- [x] Hedef noktaya otonom gitme testi
- [x] Engelden kaçınma testi

### Faz 6: QR Kod Sistemi ✅ (Tamamlandı)
- [x] Gazebo'da QR kodlu kitap modellerinin oluşturulması
- [x] Kamera feed'inden QR kod okuma (OpenCV + pyzbar)
- [x] QR kod → Raf pozisyonu eşleştirmesi
- [x] ROS topic/service ile bilgi paylaşımı

### Faz 7: Kitap Alma/Bırakma ✅ (Tamamlandı)
- [x] Kitap bırakma istasyonunun world'e eklenmesi
- [x] Forklift yukarı/aşağı hareketi
- [x] Gazebo link attachment (kitabı robota bağlama)
- [x] Kitap bırakma (detach) mantığı
- [x] Raf pozisyonlarının tanımlanması

### Faz 8: Görev Yönetimi - State Machine ✅ (Tamamlandı)
- [x] State machine tasarımı (library_mission.py)
- [x] States: IDLE → GO_TO_PICKUP → SCAN_QR → PICKUP → GO_TO_SHELF → DROPOFF → RETURN
- [x] Hata durumları ve recovery behavior
- [x] Tam görev döngüsü testi

### Faz 9: Dinamik Engeller (Opsiyonel) (Tahmini: 1 gün)
- [ ] Hareketli kutu/insan modellerinin eklenmesi
- [ ] Gazebo actor plugin ile hareket
- [ ] Dinamik engelden kaçınma testi

### Faz 10: Test ve İyileştirme (Tahmini: 2-3 gün)
- [ ] Uç durumların test edilmesi
- [ ] Parametrelerin fine-tuning'i
- [ ] Video kaydı ve demo hazırlığı
- [ ] Dokümantasyon

---

## 🔧 Teknik Detaylar

### Sensör Konfigürasyonu

| Sensör | Topic | Mesaj Tipi | Kullanım |
|--------|-------|------------|----------|
| LiDAR | `/scan` | sensor_msgs/LaserScan | SLAM, Navigasyon |
| Kamera | `/camera/image_raw` | sensor_msgs/Image | QR Okuma |
| Ultrasonik | `/ultrasonic` | sensor_msgs/Range | Yakın engel |
| IMU | `/imu` | sensor_msgs/Imu | Odometri |
| Odometri | `/odom` | nav_msgs/Odometry | Pozisyon |

### Koordinat Sistemi ve Önemli Pozisyonlar

```yaml
# Pozisyonlar (harita çıkarıldıktan sonra güncellenecek)
positions:
  home: {x: 0.0, y: 0.0, yaw: 0.0}           # Başlangıç/bekleme noktası
  dropoff: {x: 2.0, y: 1.0, yaw: 0.0}        # Kitap bırakma istasyonu
  shelf_A: {x: 5.0, y: 2.0, yaw: 1.57}       # Raf A (QR: "SHELF_A")
  shelf_B: {x: 5.0, y: 0.0, yaw: 1.57}       # Raf B (QR: "SHELF_B")
  shelf_C: {x: 5.0, y: -2.0, yaw: 1.57}      # Raf C (QR: "SHELF_C")
  shelf_D: {x: 7.0, y: 1.0, yaw: 0.0}        # Raf D (QR: "SHELF_D")
  shelf_E: {x: 7.0, y: -1.0, yaw: 0.0}       # Raf E (QR: "SHELF_E")
```

### State Machine Diyagramı

```
                    ┌──────────────┐
                    │     IDLE     │◄─────────────────────┐
                    └──────┬───────┘                      │
                           │ Kitap algılandı              │
                           ▼                              │
                    ┌──────────────┐                      │
                    │ GO_TO_PICKUP │                      │
                    └──────┬───────┘                      │
                           │ Hedefe ulaşıldı              │
                           ▼                              │
                    ┌──────────────┐                      │
                    │   SCAN_QR    │                      │
                    └──────┬───────┘                      │
                           │ QR okundu                    │
                           ▼                              │
                    ┌──────────────┐                      │
                    │    PICKUP    │                      │
                    └──────┬───────┘                      │
                           │ Kitap alındı                 │
                           ▼                              │
                    ┌──────────────┐                      │
                    │ GO_TO_SHELF  │                      │
                    └──────┬───────┘                      │
                           │ Rafa ulaşıldı                │
                           ▼                              │
                    ┌──────────────┐                      │
                    │   DROPOFF    │                      │
                    └──────┬───────┘                      │
                           │ Kitap bırakıldı              │
                           ▼                              │
                    ┌──────────────┐                      │
                    │    RETURN    │──────────────────────┘
                    └──────────────┘
```

---

## 📦 Gerekli ROS Paketleri

```bash
# TurtleBot3
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations

# Navigasyon
sudo apt install ros-noetic-navigation ros-noetic-map-server ros-noetic-amcl

# SLAM
sudo apt install ros-noetic-gmapping ros-noetic-slam-toolbox

# Görüntü İşleme
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
pip3 install opencv-python pyzbar

# Teleop
sudo apt install ros-noetic-teleop-twist-keyboard

# Gazebo Plugins
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# SMACH (State Machine)
sudo apt install ros-noetic-smach ros-noetic-smach-ros
```

---

## 🎯 Başarı Kriterleri

### Minimum (MVP)
- [x] Robot haritada başarıyla lokalize olabiliyor
- [x] Robot verilen hedefe otonom gidebiliyor
- [x] Robot statik engellerden kaçınabiliyor
- [x] QR kod başarıyla okunabiliyor
- [x] Kitap forklift ile alınıp bırakılabiliyor
- [x] Tam görev döngüsü (pickup → scan → go → dropoff → return) çalışıyor

### Bonus (Zaman kalırsa)
- [ ] Dinamik engellerden (hareketli kutular) kaçınma
- [ ] Birden fazla kitap sırası (queue)
- [ ] RViz'de görev durumu görselleştirmesi
- [ ] Sesli/görsel geri bildirim

---

## 📝 Notlar ve Kararlar

| Tarih | Karar |
|-------|-------|
| 2026-01-02 | Proje planı oluşturuldu |
| 2026-01-02 | Robot: TurtleBot3 seçildi |
| 2026-01-02 | Sensörler: LiDAR, Kamera, Ultrasonik, IMU |
| 2026-01-02 | Haritalama: GMapping, Lokalizasyon: AMCL |
| 2026-01-02 | MVP ile başlanacak, zamana göre genişletilecek |
| 2026-01-02 | DISPLAY=:0 ile Gazebo GUI çalıştırıldı (WSLg) |
| 2026-01-02 | Proje dosyaları özelleştirildi (me48b_library_robot) |
| 2026-01-03 | Forklift URDF tasarlandı ve entegre edildi |
| 2026-01-04 | Navigasyon ve SLAM başarıyla tamamlandı |
| 2026-01-04 | QR kod okuma ve kitap eşleştirmesi yapıldı |
| 2026-01-04 | Tam otonom görev döngüsü (library_mission.py) tamamlandı |
| 2026-01-04 | 3D Visual Servoing eklendi (X, Y, Area) |
| 2026-01-04 | GUI Control Panel geliştirildi (Renk seçimi, Spawn) |
| 2026-01-04 | Forklift hitbox kapatıldı (Fizik bug'ı önleme) |
| 2026-01-04 | Ekip Kılavuzu (GUIDE.md) oluşturuldu |

---

## 🚀 Çalıştırma Komutları

```bash
# Gerekli path ayarı (Her yeni terminalde)
export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/home/rocyu/my_ros_project

# 1. Ana Sistem (Gazebo + Robot + Navigasyon)
roslaunch me48b_library_robot full_system.launch

# 2. Ortam Kurulumu (İstasyonlar ve Kitaplar)
python3 ~/my_ros_project/scripts/setup_environment.py

# 3. QR Okuyucu
python3 ~/my_ros_project/scripts/qr_reader.py

# 4. Görevi Başlat
python3 ~/my_ros_project/scripts/library_mission.py
```
