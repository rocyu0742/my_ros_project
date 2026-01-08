# Robot Karar Algoritması (Akış Şeması) Sunum Metni
Bu metin, hazırladığımız **Akış Diyagramını (Flowchart)** sunumda anlatırken kullanabileceğiniz bir rehberdir.

Bu şema, robotun "Şimdi ne yapmalıyım?" sorusuna nasıl cevap verdiğini anlatır.

---

## 1. Giriş
*"Sistem mimarisinden bahsettik, şimdi de robotun davranış mantığına, yani **Karar Algoritmasına** bakalım. Robotumuz **Sürekli Durum Döngüsü (State Machine)** mantığıyla çalışır."*

---

## 2. Ana Döngü (Main Loop)
*(Diyagramın sol başındaki `Main Loop` ve `CheckCarry` kısmını gösterin)*

*"Her şey robotun elinde (forkliftinde) bir kitap olup olmadığını kontrol etmesiyle başlar:"*
*   **Taşıma Modu (Carrying? = Yes):** Eğer mıknatıs aktifse ve yük varsa, robot doğrudan **Bırakma (Dropoff)** moduna geçer.
*   **Arama Modu (Carrying? = No):** Eğer yükü yoksa, tanımlı istasyonlarda kitap olup olmadığını kontrol eder.

---

## 3. Akıllı Alma Operasyonu (Pickup Logic)
*(Ortadaki `Pickup Logic` kutusunu detaylandırın)*

*"Eğer bir kitap bulunursa, robotumuz gelişmiş bir **Alma Prosedürü** başlatır. Bu sıradan bir 'git-al' işlemi değildir, görüntü işleme desteklidir:"*

1.  **Yaklaşma:** Robot önce kitabın olduğu masaya kaba navigasyonla gider.
2.  **Görsel Arama:** Çatalı aşağı indirir ve kamerasıyla yukarı doğru tarama yaparak kitabın üzerindeki QR kodu arar.
3.  **Visual Servoing (Kilitlenme):** QR kodu gördüğü anda kontrolü 'Görsel Servo' algoritması devralır. Robot, QR kodu kameranın tam ortasına getirecek şekilde milimetrik hareketler yapar.
4.  **Manyetik Kenetlenme:** Hedefe kilitlendiğinde mıknatısı açar (`Mag ON`) ve kitabı alır.

---

## 4. Akıllı Bırakma Operasyonu (Dropoff Logic)
*(Sağ alttaki `Dropoff Logic` kutusunu anlatın)*

*"Robot kitabı aldıktan sonra nereye götüreceğini bilir:"*
1.  **Raf Seçimi:** Okuduğu QR koda bağlı olarak (Örn: "Roman", "Bilim") doğru rafı veritabanından seçer. Eğer kodu tanıyamazsa "Varsayılan Raf"a götürür.
2.  **Bırakma:** Rafa yanaşır, çatalı en üst seviyeye kaldırır ve mıknatısı kapatarak (`Mag OFF`) kitabı rafa bırakır.

---

## 5. Özet
*"Sonuç olarak robotumuz; körlemesine hareket etmek yerine, sürekli çevresini kontrol eden, hata durumunda (QR göremezse) işlemi iptal edip güvenli moda geçen **dinamik bir karar mekanizmasına** sahiptir."* 
