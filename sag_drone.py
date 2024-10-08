from dronekit import connect, VehicleMode, Command
import time
from pymavlink import mavutil
import cv2
from ultralytics import YOLO
import numpy as np
import os
import shutil



# YOLO modelini yükle
try:
    modello = YOLO("last.pt")
    print("YOLO modeli başarıyla yüklendi.")
except Exception as e:
    print(f"YOLO model yükleme hatası: {e}")
    exit()

# Drone bağlantısını yap
dronebaglanti = "127.0.0.1:14550"  # IP ve port bilgisi
try:
    sagdrone = connect(dronebaglanti, wait_ready=True)
    print("Drone'a başarıyla bağlandı.")
except Exception as e:
    print(f"Drone bağlantı hatası: {e}")
    exit()

def takeoff(irtifa):
    while not sagdrone.is_armable:
        print("İHA arm edilebilir durumda değil.")
        time.sleep(1)

    print("İHA arm edilebilir.")
    sagdrone.mode = VehicleMode("GUIDED")
    sagdrone.armed = True

    while not sagdrone.armed:
        print("İHA arm ediliyor...")
        time.sleep(0.5)

    print("İHA arm edildi.")
    sagdrone.simple_takeoff(irtifa)
    
    while sagdrone.location.global_relative_frame.alt < irtifa * 0.9:
        print("İHA hedefe yükseliyor.")
        time.sleep(1)


def clean_log_directory():
    folder = "log"
    
    # Klasör varsa içeriğini sil
    if os.path.exists(folder):
        shutil.rmtree(folder)
        print(f"{folder} klasörü temizlendi.")
    
    # Klasörü yeniden oluştur
    os.makedirs(folder)
    print(f"{folder} klasörü oluşturuldu.")

clean_log_directory()

def yaz(file, results, drone):
    coords = results[0].boxes.xyxy
    if coords.size(0) == 0:
        return

    labels = results[0].boxes.cls
    lbl_np = np.array(labels.cpu())
    a_np = np.array(coords.cpu())
    file.write("[\n")
    for lbl, arr in zip(lbl_np, a_np):
        arr_str = ' '.join(map(str, arr))  # Koordinatları string'e çevir
        file.write(f"{int(lbl)} {arr_str}\n")  # Etiketi ve koordinatları yaz
    file.write("]\n")

    # Drone konumunu al
    lat = drone.location.global_frame.lat  # Latitude
    lon = drone.location.global_frame.lon  # Longitude
    alt = drone.location.global_relative_frame.alt  # Altitude (relative to the takeoff point)
    
    # Koordinatları dosya adı için kullan
    file_name = f"log/img_lat_{lat:.6f}_lon_{lon:.6f}_alt_{alt:.2f}.jpg"
    
    # Eğer ilgili nesne varsa (örneğin, label 3)
    count = 0
    for lbl in labels:    
        if lbl == 3:
            count += 1

    if count >= 1:
        # Resmi kaydet
        cv2.imwrite(file_name, results[0].plot())
        print(f"Resim kaydedildi: {file_name}")


# İHA hız ayarları
sagdrone.airspeed = 2
sagdrone.groundspeed = 2
sagdrone.velocity[0] = 2
sagdrone._yawspeed = 2
sagdrone._pitchspeed = 2

def tara(x1, y1, x2, y2, x3, y3, x4, y4):
    x_orta1 = (x1 + x4) / 2
    x_orta2 = (x1 + x_orta1) / 2
    x_orta3 = (x4 + x_orta1) / 2
    y_orta1 = (y1 + y2) / 2
    
    global komut
    komut = sagdrone.commands
    komut.clear()
    time.sleep(1)
    
    # Takeoff komutu
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 6))
    
    # Diğer komutlar
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x1, y1, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x2, y2, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x_orta2, y2, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x_orta2, y1, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x_orta1, y1, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x_orta1, y2, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x_orta3, y2, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x_orta3, y1, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x4, y1, 6))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, x3, y3, 6))
    # RTL komutu
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    komut.upload()
    print("Komutlar yükleniyor.\n")

# Kalkış yap
takeoff(6)

# Taramayı başlat

# Sol Alt Köşe, Sağ Alt Köşe, Sağ Üst Köşe, Sol Üst Köşe nin koordinatlarını sırasıyla girin

#tara(-35.3632019, 149.1649845, -35.3631713, 149.1659769, -35.3621192, 149.1658482, -35.3621257, 149.1648799)

tara(-35.36312265,149.16498214,-35.36312265,149.16537541,-35.36282953,149.16537541,-35.36282953,149.16498214)


# İHA'yı otomatik moda al
komut.next = 0
sagdrone.mode = VehicleMode("AUTO")
time.sleep(1)
print(f"İHA : {str(sagdrone.mode)} moduna alındı.\n")

# Kamera bağlantısını yap
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Kamera açılamadı! Lütfen kamerayı kontrol edin.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

with open("data.txt", "w") as file:
    counter = 0

    while True:
        # İHA hız ayarları
        sagdrone.airspeed = 2
        sagdrone.groundspeed = 2
        sagdrone.velocity[0] = 2

        # Uçuş tamamlandıysa döngüyü kır
        if komut.next >= komut.count:
            print("Uçuş tamamlandı.")
            break

        ret, frame = cap.read()
        if not ret:
            print("Görüntü alınamadı! Kamerayı kontrol edin.")
            break
        
        # Görüntüyü işleyin
        try:
            results = modello(frame, imgsz=(256))
            result = results[0]  # İlk sonuç
            frame = result.plot()
        except Exception as e:
            print(f"YOLO görüntü işleme hatası: {e}")
            continue

        # Görüntüyü ekranda göster
        cv2.imshow('fra', frame)
        
        # Çıkış için 'q' tuşunu kontrol et
        if cv2.waitKey(10) & 0xFF == ord("q"):
            break
        
        counter += 1
        if counter == 10:
            counter = 0
            try:
                yaz(file, results,sagdrone)
            except Exception as e:
                print(f"Görüntü kaydetme hatası: {e}")

# Kaynakları serbest bırak
cap.release()
cv2.destroyAllWindows()