# ROS Mimarisi Sunum Metni
Bu diyagram, robotunuzun **nasıl düşündüğünü ve hareket ettiğini** gösterir. Sunumda bu görseli "Sistem Mimarisi" veya "Veri Akış Şeması" başlığı altında anlatabilirsiniz.

Konuşmanızı yaparken diyagramı **Yukarıdan Aşağıya (Sensörlerden -> Karar Mekanizmasına -> Harekete)** doğru okuyarak anlatmak en anlaşılır yoldur.

---

## 1. Giriş: Büyük Resim
*"Projemizin yazılım mimarisi, ROS (Robot Operating System) üzerinde modüler node'lardan oluşmaktadır. Sistemimiz temelde üç katmandan oluşur: **Algılama (Sensörler)**, **İşleme (Navigation & Vision)** ve **Karar Verme (Ana Görev Düğümü)**."*

---

## 2. Adım Adım Anlatım (Yukarıdan Aşağıya)

### A. Üst Katman: Sensörler (Gözler ve kulaklar)
*(Diyagramın en üstündeki 3 kutuyu gösterin)*
*"Robotumuz dış dünyayı algılamak için üç temel sensör kullanır:"*
1.  **LiDAR (Lazer Tarayıcı):** Etraftaki engelleri ve duvarları görür. `/scan` topiği üzerinden mesafe verisi gönderir.
2.  **Kamera:** Kitapların üzerindeki QR kodları okumak için kullanılır. Ham görüntü verisi (`/image_raw`) sağlar.
3.  **Tekerlek Enkoderleri (Odometry):** Tekerleklerin ne kadar döndüğünü ölçerek robotun anlık hızını ve tahmini konumunu (`/odom`) bildirir.

### B. Orta Katman: Algı ve Navigasyon
*(Ortadaki `qr_reader`, `amcl` ve `move_base` kutularını gösterin)*
*"Sensörlerden gelen veriler bu katmanda işlenir:"*
*   **QR Reader Node:** Kameradan gelen görüntüyü işler (OpenCV kullanarak). Bir QR kod bulduğunda bunu çözer ve konumunu (`metric`) hesaplayıp ana sisteme bildirir.
*   **AMCL (Adaptive Monte Carlo Localization):** *"Ben haritanın neresindeyim?"* sorusunun cevabıdır. LiDAR verisini ve haritayı karşılaştırarak robotun tam konumunu (`amcl_pose`) günceller.
*   **Move Base (Sürücü):** Navigasyon paketidir. Hedef noktaya gitmek için engellere çarpmadan bir rota (path planning) oluşturur ve tekerleklere hız komutları (`cmd_vel`) gönderir.

### C. Alt Katman: Patron (Karar Mekanizması)
*(En alttaki `library_mission.py` kutusunu gösterin)*
*"Burası sistemin beynidir. **`library_mission.py`** isimli bu düğüm, bir orkestra şefi gibi çalışır (diagramda 'Patron' olarak belirttik)."*

*"Şöyle çalışır:"*
1.  **Görev Yönetimi:** Kullanıcıdan gelen isteklere veya otonom döngüye göre nereye gidileceğine karar verir.
2.  **Koordinasyon:** `move_base`'e "Şu rafa git" der.
3.  **Görsel Servolama (Visual Servoing):** Rafa yaklaştığında navigasyonu devreden çıkarıp, kameradan gelen anlık veriye göre milimetrik yanaşma (docking) yapar.
4.  **Manipülasyon:** Forklift mekanizmasını kontrol ederek (`/fork_joint...`) kitabı alır veya bırakır.

---

## 3. Kapanış Cümlesi (Özet)
*"Özetle; sensör verileri işlenerek anlamlı bilgiye (konum ve görsel hedef) dönüştürülüyor, ana kontrolcümüz bu bilgileri kullanarak navigasyon ve forklift motorlarını senkronize bir şekilde yönetiyor."*

---

## Gelebilecek Soru-Cevap İpuçları

*   **Soru:** "Neden `cmd_vel` hem `move_base`'den hem `library_mission`'dan çıkıyor?"
    *   **Cevap:** *"Normal sürüşte (uzun mesafe) `move_base` engellerden kaçarak sürer. Ancak kitabı alırken (hassas yanaşma) kontrolü ana yazılım devralır ve kameraya bakarak robotu milimetrik hareket ettirir."*

*   **Soru:** "TF Tree (Dönüşüm Ağacı) nerede işin içine giriyor?"
    *   **Cevap:** *"Kameranın algıladığı QR kodun robotun merkezine, oradan da haritadaki konuma dönüştürülmesi için arka planda TF ağacını kullanıyoruz."*
