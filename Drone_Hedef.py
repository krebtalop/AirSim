import airsim
import numpy as np
import cv2
import time
import math

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
client.moveToZAsync(-5, 2).join()
time.sleep(2)
print("🚁 Drone havalandı.")

# 🎯 Sabit hedef koordinatları
hedef_x = 20
hedef_y = 50

def get_yaw():
    orientation = client.getMultirotorState().kinematics_estimated.orientation
    _, _, yaw = airsim.to_eularian_angles(orientation)
    return yaw

def get_z():
    return client.getMultirotorState().kinematics_estimated.position.z_val

def hedefe_uzaklik():
    pos = client.getMultirotorState().kinematics_estimated.position
    dx = hedef_x - pos.x_val
    dy = hedef_y - pos.y_val
    return math.sqrt(dx*2 + dy*2)

def hedefe_don_ve_git(hiz=1.0, sure=0.5):
    pos = client.getMultirotorState().kinematics_estimated.position
    dx = hedef_x - pos.x_val
    dy = hedef_y - pos.y_val
    uzaklik = math.sqrt(dx*2 + dy*2)
    if uzaklik == 0:
        return
    yaw = math.degrees(math.atan2(dy, dx))
    client.rotateToYawAsync(yaw).join()
    vx = (dx / uzaklik) * hiz
    vy = (dy / uzaklik) * hiz
    client.moveByVelocityAsync(vx, vy, 0, sure)

def get_camera_image():
    raw = client.simGetImage("front_center", airsim.ImageType.Scene)
    if raw is None:
        return None
    img1d = np.frombuffer(raw, dtype=np.uint8)
    return cv2.imdecode(img1d, cv2.IMREAD_COLOR)

def get_depth_image():
    responses = client.simGetImages([
        airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, pixels_as_float=True, compress=False)
    ])
    if not responses or responses[0].image_data_float == []:
        return None
    img1d = np.array(responses[0].image_data_float, dtype=np.float32)
    return img1d.reshape(responses[0].height, responses[0].width)

def engel_var_mi_depth(threshold_metre=2.0):
    depth = get_depth_image()
    if depth is None:
        print("⚠ Derinlik verisi alınamadı!")
        return False
    h, w = depth.shape
    orta_bolge = depth[h//3:h*2//3, w//3:w*2//3]
    yakin_nokta_sayisi = np.sum(orta_bolge < threshold_metre)
    oran = yakin_nokta_sayisi / orta_bolge.size
    print(f"📏 Derinlik oranı < {threshold_metre}m: %{oran*100:.1f} ➤ {'🚨 TEHLİKE!' if oran > 0.15 else '✔ Güvenli'}")
    return oran > 0.15

def engel_puanlari(img):
    h, w, _ = img.shape
    orta = img[h//3:h*4//5, w//3:w*2//3]
    alt = img[h*4//5:h, w//3:w*2//3]
    sag = img[h//3:h*4//5, w*2//3:]
    sol = img[h//3:h*4//5, :w//3]

    gri_orta = cv2.cvtColor(orta, cv2.COLOR_BGR2GRAY)
    gri_alt = cv2.cvtColor(alt, cv2.COLOR_BGR2GRAY)
    gri_sag = cv2.cvtColor(sag, cv2.COLOR_BGR2GRAY)
    gri_sol = cv2.cvtColor(sol, cv2.COLOR_BGR2GRAY)

    def dinamik_esik(gri):
        ort = np.mean(gri)
        return 255 - max(ort, 80)

    _, maske_orta = cv2.threshold(gri_orta, dinamik_esik(gri_orta), 255, cv2.THRESH_BINARY_INV)
    _, maske_sag = cv2.threshold(gri_sag, dinamik_esik(gri_sag), 255, cv2.THRESH_BINARY_INV)
    _, maske_sol = cv2.threshold(gri_sol, dinamik_esik(gri_sol), 255, cv2.THRESH_BINARY_INV)

    kenarlar = cv2.Canny(gri_alt, 50, 150)

    puan_orta = cv2.countNonZero(maske_orta)
    puan_sag = cv2.countNonZero(maske_sag)
    puan_sol = cv2.countNonZero(maske_sol)
    kaya_puani = cv2.countNonZero(kenarlar)

    return puan_orta, puan_sag, puan_sol, kaya_puani

def uygun_yon_bul():
    print("🛡 Çevre taranıyor...")
    en_az_engel = float("inf")
    en_iyi_yaw = None
    mevcut_yaw = math.degrees(get_yaw())

    for aci in range(-60, 61, 30):
        hedef_yaw = mevcut_yaw + aci
        client.rotateToYawAsync(hedef_yaw).join()
        time.sleep(0.3)

        img = get_camera_image()
        if img is None:
            continue

        puan_orta, _, _, _ = engel_puanlari(img)
        print(f"Açı {aci:+}° → Engel puanı: {puan_orta}")

        if puan_orta < en_az_engel:
            en_az_engel = puan_orta
            en_iyi_yaw = hedef_yaw

    return en_iyi_yaw, en_az_engel

# 🔁 ANA DÖNGÜ
while True:
    uzaklik = hedefe_uzaklik()
    print(f"🎯 Hedefe uzaklık: {uzaklik:.2f} metre")

    if uzaklik < 2.0:
        print("🛬 Hedefe ulaşıldı → Hızlı iniş başlatılıyor!")
        client.moveByVelocityAsync(0, 0,  -5, 2).join()  # Hızlıca in
        client.landAsync().join()  # Son güvenli iniş
        break

    img = get_camera_image()
    if img is None:
        print("Kamera alınamadı")
        continue

    puan_orta, puan_sag, puan_sol, kaya_puani = engel_puanlari(img)
    derinlik_engel = engel_var_mi_depth(threshold_metre=2.0)

    if kaya_puani > 3500:
        print("🪨 Kaya algılandı → Yukarı kaçınılıyor!")
        client.moveByVelocityAsync(0, 0, -3, 1).join()
        continue

    if derinlik_engel:
        print("🚩 Derinlikte ciddi engel var → Yukarı çıkılıyor!")
        client.moveByVelocityAsync(0, 0, -2.5, 1).join()
        continue

    if puan_orta > 8000:
        print("🚧 Görsel yoğunluk yüksek → Yön aranıyor...")
        en_iyi_yaw, en_iyi_puan = uygun_yon_bul()

        if en_iyi_puan < 8000:
            print(f"✅ Yeni yön: {en_iyi_yaw:.1f}°")
            client.rotateToYawAsync(en_iyi_yaw).join()
        else:
            z = get_z()
            if z > -20:
                print("🔼 Yukarı çıkılıyor (yön yok)")
                client.moveByVelocityAsync(0, 0, -2.5, 1).join()
            elif z < -3:
                print("🔽 Aşağı iniliyor (yön yok)")
                client.moveByVelocityAsync(0, 0, 2, 1).join()
            else:
                print("⏸ Bekleniyor")
            time.sleep(1)
    else:
        print("➡ Hedef yönünde ilerleniyor")
        hedefe_don_ve_git(hiz=1.0, sure=0.5)

    time.sleep(0.1)