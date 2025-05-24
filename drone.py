import airsim
import numpy as np
import cv2
import time
import math

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Kalkış ve sabit yüksekliğe çık
client.takeoffAsync().join()
client.moveToZAsync(-5, 2).join()
print("Drone havalandı.")

def get_yaw():
    orientation = client.getMultirotorState().kinematics_estimated.orientation
    _, _, yaw = airsim.to_eularian_angles(orientation)
    return yaw

def get_z():
    return client.getMultirotorState().kinematics_estimated.position.z_val

def ileri_git_surekli(hiz=2.0, sure=1.0):
    yaw = get_yaw()
    vx = math.cos(yaw) * hiz
    vy = math.sin(yaw) * hiz
    client.moveByVelocityAsync(vx, vy, 0, sure)

def get_camera_image():
    raw = client.simGetImage("front_center", airsim.ImageType.Scene)
    if raw is None:
        return None
    img1d = np.frombuffer(raw, dtype=np.uint8)
    return cv2.imdecode(img1d, cv2.IMREAD_COLOR)

def engel_puani(img):
    h, w, _ = img.shape

    # Orta bölge (genel engeller için)
    orta = img[h//3:h*4//5, w//3:w*2//3]
    gri_orta = cv2.cvtColor(orta, cv2.COLOR_BGR2GRAY)
    _, maske_orta = cv2.threshold(gri_orta, 60, 255, cv2.THRESH_BINARY_INV)
    puan_orta = cv2.countNonZero(maske_orta)

    # Alt bölge (zemin yükselmesi, kaya için)
    alt = img[h*4//5:h, w//3:w*2//3]
    gri_alt = cv2.cvtColor(alt, cv2.COLOR_BGR2GRAY)
    _, maske_alt = cv2.threshold(gri_alt, 60, 255, cv2.THRESH_BINARY_INV)
    puan_alt = cv2.countNonZero(maske_alt)

    # Görüntü gösterimi
    cv2.imshow("Orta Görüş", orta)
    cv2.imshow("Alt Görüş", alt)
    cv2.waitKey(1)

    return puan_orta, puan_alt

def uygun_yon_bul():
    print("Çevre taranıyor...")
    en_az_engel = float("inf")
    en_iyi_yaw = None
    mevcut_yaw = math.degrees(get_yaw())

    for aci in range(-90, 91, 30):  # -90° ile +90° arası
        hedef_yaw = mevcut_yaw + aci
        client.rotateToYawAsync(hedef_yaw).join()
        time.sleep(0.5)

        img = get_camera_image()
        if img is None:
            continue

        puan_orta, _ = engel_puani(img)
        print(f"Açı {aci:+}° → Engel puanı: {puan_orta}")

        if puan_orta < en_az_engel:
            en_az_engel = puan_orta
            en_iyi_yaw = hedef_yaw

    return en_iyi_yaw, en_az_engel

# Ana döngü
while True:
    img = get_camera_image()
    if img is None:
        print("Kamera alınamadı")
        continue

    puan_orta, puan_alt = engel_puani(img)

    if puan_alt > 7000 and puan_orta < 3000:
        print("Zemin/kaya yükseliyor gibi, yukarı çıkılıyor")
        client.moveByVelocityAsync(0, 0, -2, 1).join()
        continue

    if puan_orta > 5000:
        print("Engel algılandı! Alternatif yön aranıyor...")
        en_iyi_yaw, en_iyi_puan = uygun_yon_bul()

        if en_iyi_puan < 5000:
            print(f"Yeni yön bulundu! Yaw: {en_iyi_yaw:.1f}")
            client.rotateToYawAsync(en_iyi_yaw).join()
        else:
            z = get_z()
            if z > -20:
                print("⬆️ Yukarı çıkılıyor (başka yön yok)")
                client.moveByVelocityAsync(0, 0, -2, 1).join()
            elif z < -3:
                print("⬇️ Aşağı iniliyor (başka yön yok)")
                client.moveByVelocityAsync(0, 0, 2, 1).join()
            else:
                print("Hiçbir yön uygun değil, bekleniyor")
            time.sleep(1)
    else:
        print("Yol açık, akıcı ilerleniyor")
        ileri_git_surekli()

    time.sleep(0.1)
