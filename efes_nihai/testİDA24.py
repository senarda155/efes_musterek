import pyzed.sl as sl
import cv2
import numpy as np
import math
import time
import os
import select, sys
import tty
import termios
import threading
import queue
import datetime
from colorama import Fore, Back, Style, init
import serial
from ultralytics import YOLO
import supervision as sv
import json

import utilities as utils
import config as cfg
import navigasyon as nav
import kamera
from kamera import TimestampHandler
import telem
from telem import TelemetrySender, CommandReceiver
from headingFilter import KalmanFilter  # todo: heading filtresi testi yap
from MainSystem2 import USVController

import torch 

import socket, struct,pickle 

# Sadece Görüntü Aktarma icin (yarısmada komut satırına alınacak)
if cfg.STREAM == True:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(("192.168.1.108", 5000))

# Görüntü İşleme
#torch.backends.cudnn.benchmark = True  # sabit input boyutları için hız
model = YOLO("/home/yarkin/PycharmProjects/teknofestIDA/600m_fp16_gpu.engine")
#model.to('cuda').half()
#model.fuse()  # varsa Conv+BN birleştirir
bounding_box_annotator = sv.RoundBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# OrangeCube
controller = USVController("/dev/ttyACM0", baud=57600)
print("Arming vehicle...")
#controller.arm_vehicle()
print("Vehicle armed!")
print("Setting mode...")
controller.set_mode("MANUAL")
print("Mode set!")

# Telemetry
telemetry = TelemetrySender(cfg.SERIAL_PORT, cfg.SERIAL_BAUD)
cmd_queue = queue.Queue()
cmd_rx = CommandReceiver(telemetry, cmd_queue)
cmd_rx.start()
#Asenkron TX'i bir kez oluştur
tx = telem.TelemetryTx(telemetry, max_hz=10)
# Bazı implementasyonlar start() ister, varsa aç:
if hasattr(tx, "start"):
    tx.start()


# Güvenli başlangıç değerleri (NameError önlemek için)
magnetic_heading = None
magnetic_heading_state = None

def main():
    global width, manual_mode, magnetic_heading, mission_started

    print("Initializing Camera...")
    zed = kamera.initialize_camera()
    print("Camera initialized!")
    
    camera_info = zed.get_camera_information()
    width = camera_info.camera_configuration.resolution.width
    height = camera_info.camera_configuration.resolution.height
    print(width,height)
    # Görüntüde merkez noktasını hesapla
    center_x = width // 2
    center_y = height // 2

    ts_handler = TimestampHandler()     # Used to store the sensors timestamp to know if the sensors_data is a new one or not

    sensors_data = sl.SensorsData()    # Sensör verisi al
    image = sl.Mat()# Görüntü ve derinlik verilerini almak için Mat nesneleri oluştur
    depth = sl.Mat()

    adviced_course = 0
    aci_farki = 0
    hedefe_mesafe = 1000
    TOPLAM_HATA= 0.0

    _waypoints = []       # global waypoint listesi (kullanılıyor)
    _active_wp_index = 0  # hangi waypointteyiz
    _waypoints_created = False

    
    #video kayıt
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    video_path = f"idaKayit/kayit_{ts}.mp4"
    # VideoWriter thread
    writer = utils.AsyncVideoWriter(video_path, fps=20.0, max_queue=120)
    writer.start()
    print(f"[INFO] Video kaydı başladı: {video_path}")
    
    mevcut_gorev = "GPS4"

    manual_mode = False
    mission_started = True
    
    try:
        while True:
            
            # ---- Komut kuyruğunu boşalt ve gelen komutları uygula ----
            try:
                while True:
                    cmd = cmd_queue.get_nowait()
                    print("[CMD RX]", cmd)
                    # EMERGENCY_STOP yakala
                    if isinstance(cmd, dict) and cmd.get("cmd") == "emergency_stop":
                        print("\n[INFO] Yer kontrolden ACİL KAPATMA alındı, kapanıyor...")
                        raise utils.EmergencyShutdown()
                        raise KeyboardInterrupt  # mevcut except bloğunla aynı yolu kullanır
                    manual_mode, mission_started = telem.handle_command(cmd, controller, cfg, manual_mode, mission_started)
            except queue.Empty:
                pass
            

            ida_enlem, ida_boylam = controller.get_current_position()
            
            if zed.grab() == sl.ERROR_CODE.SUCCESS:

                # Görüntü ve derinlik verilerini al
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # OpenCV formatına dönüştür
                frame = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)  # BGRA -> BGR
                results = model(frame, conf=0.6, verbose=False)[0]

                # yolo sonuçlarının sv.Detections formatına dönüştürülmesi
                detections = sv.Detections.from_ultralytics(results)

                # tespitlerin sınırlarının ve etiketlerinin oluşturulması
                frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
                frame = label_annotator.annotate(scene=frame, detections=detections)

                # tespitlerin koordinatlarının sınıflarının alınması
                coordinates = detections.xyxy.tolist()
                class_ids = detections.class_id.tolist()
                # retrieve the current sensors sensors_data
                if zed.get_sensors_data(sensors_data,
                                        sl.TIME_REFERENCE.CURRENT):  # time_reference.image for synchorinzed timestamps
                    # Check if the data has been updated since the last time
                    # IMU is the sensor with the highest rate
                    if ts_handler.is_new(sensors_data.get_imu_data()):
                        magnetometer_data = sensors_data.get_magnetometer_data()
                        magnetic_heading_state = magnetometer_data.magnetic_heading_state
                        
                        # Get the raw magnetic heading  # Apply low-pass filter
                        magnetic_filter = KalmanFilter(process_variance=1e-3, measurement_variance=1e-1)
                        magnetic_heading = magnetic_filter.update(sensors_data.get_magnetometer_data().magnetic_heading)
                        magnetic_heading = (magnetic_heading + 6 )% 360
                        #magnetic_heading = sensors_data.get_magnetometer_data().magnetic_heading
                        heading_dogruluk = sensors_data.get_magnetometer_data().magnetic_heading_accuracy

                            # Her tespit kutusunun sağ üst köşesine derinlik değerini yazdırmak için:
                
                for box in coordinates:
                    x1, y1, x2, y2 = map(int, box)  # tamsayıya çeviriyoruz
                    # Sağ üst köşe koordinatları: (x2, y1)
                    depth_val = depth.get_value(int((x2 + x1) / 2), int((y1 + y2) / 2))[
                        1]  # Eğer depth değeri geçerliyse (NaN değilse) yazdır
                    if not np.isnan(depth_val):
                        text = f"{depth_val:.2f} m"
                        # Yazıyı kutunun sağ üst köşesine ekleyelim; konum ayarını isteğinize göre değiştirebilirsiniz
                        cv2.putText(frame, text, (x2 - 60, y1 + 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,0,255), 2)

                orange_detected = False
                yellow_detected = False

                orange_positions = []
                yellow_positions = []

                # while not (şekiller detected and şekil tespitinden 7 saniye geçmişse)
                for i, class_id in enumerate(class_ids):
                    if class_id == 3 : 
                        orange_detected = True
                        orange_positions.append(coordinates[i])
                    elif class_id == 1 :  
                        yellow_detected = True
                        yellow_positions.append(coordinates[i])

#----------------------------- GÖREV GPS 1 --------------------------------------------------------------------------------------------------------------------
                if mevcut_gorev == "GPS1" and mission_started and not manual_mode:
                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS1_enlem, cfg.GPS1_boylam)
                    aci_farki = nav.signed_angle_difference(magnetic_heading, adviced_course)
                    # Calculate the error using: #negatif deger tavsiye rotanın iskelede, pozitif deger tavsiye rotanın sancakta kaldıgı anlamına gelir
                    hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS1_enlem, cfg.GPS1_boylam)
                    
                    kp_aci = (cfg.KpACI)*aci_farki

                    if abs(aci_farki) > cfg.ACI_THRESHOLD_AZ:
                        print("aci düzeliyor")
                        if abs(kp_aci) < 100:
                            if kp_aci <0:
                                kp_aci = -100
                            else:
                                kp_aci = 100
                        
                        toplam_sol = cfg.BASE_PWM  + kp_aci
                        toplam_sag = cfg.BASE_PWM  - kp_aci
                    
                    else:
                        print("ileri")
                        TOPLAM_HATA = TOPLAM_HATA + hedefe_mesafe
                        ki_mesafe= TOPLAM_HATA* cfg.KiMESAFE
                        kp_mesafe= cfg.KpMESAFE*hedefe_mesafe
                        duzeltme = ki_mesafe + kp_mesafe
                        print(duzeltme)
                    
                        if duzeltme < 100:
                            duzeltme = 100
                        toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                        toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                    if toplam_sol > 1980:
                        toplam_sol = 1980
                    if toplam_sol < 1100:
                        toplam_sol = 1110
                    if toplam_sag > 1980:
                        toplam_sag = 1980
                    if toplam_sag < 1100:
                        toplam_sag = 1110

                    controller.set_servo(cfg.SOL_MOTOR, toplam_sol)
                    controller.set_servo(cfg.SAG_MOTOR, toplam_sag)

                    if hedefe_mesafe < cfg.THRESHOLD:
                        TOPLAM_HATA= 0.0
                        mevcut_gorev = "GPS2"

#----------------------------- GÖREV GPS 1 --------------------------------------------------------------------------------------------------------------------


#----------------------------- GÖREV GPS 2 --------------------------------------------------------------------------------------------------------------------
               
                elif mevcut_gorev == "GPS2" and mission_started and not manual_mode:
                    
                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS2_enlem, cfg.GPS2_boylam)
                    aci_farki = nav.signed_angle_difference(magnetic_heading, adviced_course)
                    # Calculate the error using: #negatif deger tavsiye rotanın iskelede, pozitif deger tavsiye rotanın sancakta kaldıgı anlamına gelir
                    hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS2_enlem, cfg.GPS2_boylam)

                    kp_aci = (cfg.KpACI)*aci_farki

                    if abs(aci_farki) > cfg.ACI_THRESHOLD_AZ:
                        if abs(kp_aci) < 100:
                            if kp_aci <0:
                                kp_aci = -100
                            else:
                                kp_aci = 100
                        
                        toplam_sol = cfg.BASE_PWM  + kp_aci
                        toplam_sag = cfg.BASE_PWM  - kp_aci
                    
                    else:
                        TOPLAM_HATA = TOPLAM_HATA + hedefe_mesafe
                        ki_mesafe= TOPLAM_HATA* cfg.KiMESAFE
                        kp_mesafe= cfg.KpMESAFE*hedefe_mesafe
                        duzeltme = ki_mesafe + kp_mesafe
                    
                        if duzeltme < 100:
                            duzeltme = 100
                        toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                        toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                    if toplam_sol > 1980:
                        toplam_sol = 1980
                    if toplam_sol < 1100:
                        toplam_sol = 1110
                    if toplam_sag > 1980:
                        toplam_sag = 1980
                    if toplam_sag < 1100:
                        toplam_sag = 1110

                    controller.set_servo(cfg.SOL_MOTOR, toplam_sol)
                    controller.set_servo(cfg.SAG_MOTOR, toplam_sag)


                    if hedefe_mesafe < cfg.THRESHOLD:
                        TOPLAM_HATA= 0.0
                        mevcut_gorev = "GPS3"

                elif mevcut_gorev == "GPS3" and mission_started and not manual_mode:
                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS3_enlem, cfg.GPS3_boylam)
                    aci_farki = nav.signed_angle_difference(magnetic_heading, adviced_course)
                    # Calculate the error using: #negatif deger tavsiye rotanın iskelede, pozitif deger tavsiye rotanın sancakta kaldıgı anlamına gelir
                    hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS3_enlem, cfg.GPS3_boylam)

                    
                    kp_aci = (cfg.KpACI)*aci_farki

                    if abs(aci_farki) > cfg.ACI_THRESHOLD_AZ:
                        if abs(kp_aci) < 100:
                            if kp_aci <0:
                                kp_aci = -100
                            else:
                                kp_aci = 100
                   
                        toplam_sol = cfg.BASE_PWM  + kp_aci
                        toplam_sag = cfg.BASE_PWM  - kp_aci
                    
                    else:
                        TOPLAM_HATA = TOPLAM_HATA + hedefe_mesafe
                        ki_mesafe= TOPLAM_HATA* cfg.KiMESAFE
                        kp_mesafe= cfg.KpMESAFE*hedefe_mesafe
                        duzeltme = ki_mesafe + kp_mesafe
                    
                        if duzeltme < 125:
                            duzeltme = 125
                        toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                        toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                    if toplam_sol > 1980:
                        toplam_sol = 1980
                    if toplam_sol < 1100:
                        toplam_sol = 1110
                    if toplam_sag > 1980:
                        toplam_sag = 1980
                    if toplam_sag < 1100:
                        toplam_sag = 1110

                    controller.set_servo(cfg.SOL_MOTOR, toplam_sol)
                    controller.set_servo(cfg.SAG_MOTOR, toplam_sag)

                    if hedefe_mesafe < cfg.THRESHOLD:
                        TOPLAM_HATA= 0.0
                        mevcut_gorev = "GPS4"
                elif mevcut_gorev == "GPS4" and mission_started and not manual_mode:
                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS4_enlem, cfg.GPS4_boylam)
                    aci_farki = nav.signed_angle_difference(magnetic_heading, adviced_course)
                    # Calculate the error using: #negatif deger tavsiye rotanın iskelede, pozitif deger tavsiye rotanın sancakta kaldıgı anlamına gelir
                    hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS4_enlem, cfg.GPS4_boylam)

                    
                    kp_aci = (cfg.KpACI)*aci_farki

                    if abs(aci_farki) > cfg.ACI_THRESHOLD_AZ:
                        if abs(kp_aci) < 100:
                            if kp_aci <0:
                                kp_aci = -100
                            else:
                                kp_aci = 100
                        
                        toplam_sol = cfg.BASE_PWM  + kp_aci
                        toplam_sag = cfg.BASE_PWM  - kp_aci
                    
                    else:
                        TOPLAM_HATA = TOPLAM_HATA + hedefe_mesafe
                        ki_mesafe= TOPLAM_HATA* cfg.KiMESAFE
                        kp_mesafe= cfg.KpMESAFE*hedefe_mesafe
                        duzeltme = ki_mesafe + kp_mesafe
                    
                        if duzeltme < 125:
                            duzeltme = 125
                        toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                        toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                    if toplam_sol > 1980:
                        toplam_sol = 1980
                    if toplam_sol < 1100:
                        toplam_sol = 1110
                    if toplam_sag > 1980:
                        toplam_sag = 1980
                    if toplam_sag < 1100:
                        toplam_sag = 1110

                    controller.set_servo(cfg.SOL_MOTOR, toplam_sol)
                    controller.set_servo(cfg.SAG_MOTOR, toplam_sag)

                    if hedefe_mesafe < cfg.THRESHOLD:
                        TOPLAM_HATA= 0.0
                        mevcut_gorev = "GPS5"









#-------------------------------------------------------- 2.GÖREV --------------------------------------------------------------------------------------------------------------
                
                elif mevcut_gorev == "GPS5" and mission_started and not manual_mode:
                    # 0) Hedef/Path ve temel seyir hesapları
                    hedef_enlem_wp, hedef_boylam_wp = cfg.GPS5_enlem, cfg.GPS5_boylam
                    hedef_enlem, hedef_boylam = hedef_enlem_wp, hedef_boylam_wp

                    try:
                        path_dir = nav.calculate_bearing(cfg.GPS4_enlem, cfg.GPS4_boylam, cfg.GPS5_enlem, cfg.GPS5_boylam)
                    except Exception:
                        path_dir = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS5_enlem, cfg.GPS5_boylam)

                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, hedef_enlem_wp, hedef_boylam_wp)
                    aci_farki      = nav.signed_angle_difference(magnetic_heading, adviced_course)  # (+) hedef sağda
                    hedefe_mesafe  = nav.haversine(ida_enlem, ida_boylam, hedef_enlem_wp, hedef_boylam_wp)

                    if aci_farki > 0:
                        print(f"[HEDEF] Ana hedef SAĞDA {abs(aci_farki):.1f}° (heading→hedef)")
                    elif aci_farki < 0:
                        print(f"[HEDEF] Ana hedef SOLDA {abs(aci_farki):.1f}° (heading→hedef)")
                    else:
                        print(f"[HEDEF] Ana hedef TAM ÖNDE 0.0° (heading→hedef)")

                    # Sarı tespiti (en yakını)
                    min_yellow_dist = float('inf')
                    min_yellow_box  = None
                    for i, class_id in enumerate(class_ids):
                        if class_id != 1:
                            continue
                        x1, y1, x2, y2 = map(int, coordinates[i])
                        cx = int((x1 + x2) / 2); cy = int((y1 + y2) / 2)

                        depths = []
                        for dx in (-1, 0, 1):
                            for dy in (-1, 0, 1):
                                px = int(np.clip(cx + dx, 0, width - 1))
                                py = int(np.clip(cy + dy, 0, height - 1))
                                try:
                                    d = depth.get_value(px, py)[1]
                                    if not np.isnan(d) and d > 0:
                                        depths.append(d)
                                except Exception:
                                    pass
                        if not depths:
                            continue

                        dist_m = float(np.median(depths))
                        if dist_m < min_yellow_dist:
                            min_yellow_dist = dist_m
                            min_yellow_box  = (x1, y1, x2, y2)

                    angle_offset_deg = None
                    if min_yellow_box is not None:
                        x1, y1, x2, y2 = min_yellow_box
                        cx_box = int((x1 + x2) / 2)
                        angle_offset_deg = (cx_box - center_x) / float(width) * cfg.CAM_HFOV  # (+) sağ, (−) sol

                    # 1) Sarı yoksa veya >5 m ise → normal ilerleme
                    if (min_yellow_box is None) or (min_yellow_dist > 5.0):
                        print("sari yok veya 5 metreden uzak")
                        kp_aci = cfg.KpACI * aci_farki
                        if abs(aci_farki) > cfg.ACI_THRESHOLD_AZ:
                            if abs(kp_aci) < 100:
                                kp_aci = -100 if kp_aci < 0 else 100
                            toplam_sol = cfg.BASE_PWM + kp_aci
                            toplam_sag = cfg.BASE_PWM - kp_aci
                        else:
                            TOPLAM_HATA  = TOPLAM_HATA + hedefe_mesafe
                            ki_mesafe    = TOPLAM_HATA * cfg.KiMESAFE
                            kp_mesafe    = cfg.KpMESAFE2 * hedefe_mesafe
                            duzeltme     = ki_mesafe + kp_mesafe
                            if duzeltme < 125: duzeltme = 125
                            toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                            toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                    # 2) Sarı tespit edilmişse
                    else:
                        # 2.1) Dönme işlemi yapıyorsam → sadece dön
                        if abs(aci_farki) > cfg.ACI_THRESHOLD_AZ:
                            kp_aci = cfg.KpACI * aci_farki
                            if abs(kp_aci) < 100:
                                kp_aci = -100 if kp_aci < 0 else 100
                            toplam_sol = cfg.BASE_PWM + kp_aci
                            toplam_sag = cfg.BASE_PWM - kp_aci
                            print("sari var ama dönülüyor")

                        else:
                            # 2.2) Düz ilerliyorsam (açı düzgün)
                            if min_yellow_dist > 4.0:
                                print("sari var ama 4 metreden uzak düz gidiliyor")
                                # 2.2.a) 4 m'den uzak → normal ileri
                                kp_aci = cfg.KpACI * aci_farki
                                TOPLAM_HATA  = TOPLAM_HATA + hedefe_mesafe
                                ki_mesafe    = TOPLAM_HATA * cfg.KiMESAFE
                                kp_mesafe    = cfg.KpMESAFE2 * hedefe_mesafe
                                duzeltme     = ki_mesafe + kp_mesafe
                                if duzeltme < 125: duzeltme = 125
                                toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                                toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                            elif (min_yellow_dist <= 4.0) and (min_yellow_dist >= 1.0) and (angle_offset_deg is not None):
                                # 2.2.b) 4–1 m arası:
                                if abs(angle_offset_deg) <= 12.0:
                                    print("1-4 metre aralıkta ve tam önümde sarı var")
                                    # ±12° ön koridorda → nudge ile sakın
                                    A = abs(angle_offset_deg)
                                    D = float(min_yellow_dist)
                                    ANGLE_MAX_DEG = 60.0
                                    HDG_GATE_DEG  = 15.0
                                    NUDGE_MAX     = 160
                                    NUDGE_MIN     = 8

                                    angle_factor = max(0.0, 1.0 - (A / ANGLE_MAX_DEG))
                                    if D <= 1.0:
                                        dist_factor = 1.0
                                    elif D >= 4.0:
                                        dist_factor = 0.0
                                    else:
                                        dist_factor = (4.0 - D) / (4.0 - 1.0)

                                    hf = max(0.0, 1.0 - max(0.0, abs(aci_farki) - HDG_GATE_DEG) / HDG_GATE_DEG)
                                    nudge_raw = int(NUDGE_MAX * angle_factor * dist_factor * hf)
                                    nudge = nudge_raw if nudge_raw >= NUDGE_MIN else 0
                                    nudge = nudge*1.2

                                    # Taban ileri itiş (PI) + açı düzeltmesi
                                    kp_aci = cfg.KpACI * aci_farki
                                    TOPLAM_HATA  = TOPLAM_HATA + hedefe_mesafe
                                    ki_mesafe    = TOPLAM_HATA * cfg.KiMESAFE
                                    kp_mesafe    = cfg.KpMESAFE2 * hedefe_mesafe
                                    duzeltme     = ki_mesafe + kp_mesafe
                                    if duzeltme < 125: duzeltme = 125
                                    toplam_sol = cfg.BASE_PWM+50
                                    toplam_sag = cfg.BASE_PWM+50
                                    #toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                                    #toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                                    # --- NUDGE yön & mod seçimi ---
                                    if nudge > 0 and (angle_offset_deg is not None):
                                        # aynı tarafta mı? (ikisi de + ya da ikisi de −)
                                        same_side = (angle_offset_deg * aci_farki) > 0
                                        # "sarı hedefle IDA'nın arasında mı?" ölçütü
                                        buoy_between_target = abs(angle_offset_deg) < abs(aci_farki)

                                        if same_side and buoy_between_target:
                                            # >>> Yerinde yavaş dön: karşı tarafa +n/2, şamandıra tarafına -n/2
                                            half = int(max(1, nudge // 2))  # en az 1 PWM
                                            if angle_offset_deg > 0:
                                                # Sarı SAĞDA → SOLA dön: sol +half, sağ -half
                                                toplam_sol += half
                                                toplam_sag -= half
                                                print(f"[AVOID] same_side&between → turn-in-place  SOL+{half} SAG-{half} "
                                                    f"(A_buoy={abs(angle_offset_deg):.1f}°, A_target={abs(aci_farki):.1f}°)")
                                            elif angle_offset_deg < 0:
                                                # Sarı SOLDA → SAĞA dön: sol -half, sağ +half
                                                toplam_sol -= half
                                                toplam_sag += half
                                                print(f"[AVOID] same_side&between → turn-in-place  SOL-{half} SAG+{half} "
                                                    f"(A_buoy={abs(angle_offset_deg):.1f}°, A_target={abs(aci_farki):.1f}°)")
                                        else:
                                            # >>> Klasik: şamandıradan UZAĞA tek-taraf nudge
                                            if angle_offset_deg > 0:
                                                # Sarı SAĞDA → sağ motoru artır → SOLA kaç
                                                toplam_sag += nudge
                                                print(f"[AVOID] away-from-buoy → SAG+{nudge} (same_side={same_side}, between={buoy_between_target})")
                                            elif angle_offset_deg < 0:
                                                # Sarı SOLDA → sol motoru artır → SAĞA kaç
                                                toplam_sol += nudge
                                                print(f"[AVOID] away-from-buoy → SOL+{nudge} (same_side={same_side}, between={buoy_between_target})")
                                else:
                                    # ±12° dışındaysa → düz PI ilerleme (sakınma yok)
                                    print("1-4 metre aralıkta ama köşede sarı var düz gidiliyor")
                                    kp_aci = cfg.KpACI * aci_farki
                                    TOPLAM_HATA  = TOPLAM_HATA + hedefe_mesafe
                                    ki_mesafe    = TOPLAM_HATA * cfg.KiMESAFE
                                    kp_mesafe    = cfg.KpMESAFE2 * hedefe_mesafe
                                    duzeltme     = ki_mesafe + kp_mesafe
                                    if duzeltme < 125: duzeltme = 125
                                    toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                                    toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                            elif (min_yellow_dist < 1.0) and (angle_offset_deg is not None) and (abs(angle_offset_deg) <= 12.0):
                                # 2.2.c) <1 m ve ±12° → Geri geri gel (heading hold ile, +1 m artana kadar)
                                print(f"[GERI] Sarı {min_yellow_dist:.2f} m ve tam önde; geri kaçış başlıyor.")
                                ref_mesafe = hedefe_mesafe

                                # ---- geri sürüş ayarları (cfg'de yoksa varsayılanlar) ----
                                REV_PWM_DELTA      = getattr(cfg, "REV_PWM_DELTA", 220)   # geri taban
                                REVERSE_TURN_SIGN  = getattr(cfg, "REVERSE_TURN_SIGN", -1) # çoğu teknede ters işaret İYİDİR
                                REV_KP_SCALE       = getattr(cfg, "REV_KP_SCALE", 0.35)    # geri Kp küçültme
                                REV_KP_MAX         = getattr(cfg, "REV_KP_MAX", 80)        # geri diferans tavanı (PWM)
                                REV_DIFF_SLEW_MAX  = getattr(cfg, "REV_DIFF_SLEW_MAX", 40) # kare başına diff değişim sınırı (PWM)
                                REV_STEP_SEC       = getattr(cfg, "REV_STEP_SEC", 0.05)    # 50 ms adım
                                REV_MAX_STEPS      = getattr(cfg, "REV_MAX_STEPS", 600)    # ~30 sn

                                # ---- heading hold: geri başlarkenki baş açısını koru ----
                                heading_ref = magnetic_heading
                                prev_diff = 0  # slew-limit için önceki (sol-sağ) diferans

                                for _ in range(REV_MAX_STEPS):
                                    # güncel hedef ve mesafe
                                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, hedef_enlem_wp, hedef_boylam_wp)
                                    hedefe_mesafe_now = nav.haversine(ida_enlem, ida_boylam, hedef_enlem_wp, hedef_boylam_wp)
                                    if hedefe_mesafe_now >= (ref_mesafe + 1.0):
                                        print("[GERI] +1.0 m şartı sağlandı; geri kaçış tamamlandı.")
                                        break

                                    # heading hold hatası (hedef açısını kovalamıyoruz!)
                                    err_heading = nav.signed_angle_difference(magnetic_heading, heading_ref)

                                    # geri için küçültülmüş ve ters işaretli Kp
                                    kp_aci = cfg.KpACI * REV_KP_SCALE * err_heading * REVERSE_TURN_SIGN
                                    # min/max sınır
                                    if kp_aci > REV_KP_MAX:   kp_aci =  REV_KP_MAX
                                    if kp_aci < -REV_KP_MAX:  kp_aci = -REV_KP_MAX
                                    # küçük değerler statik sürtünmeyi kıramayabilir — ama aşırı dönmesin diye geri modda min zorlamıyoruz

                                    # geri PWM
                                    toplam_sol = cfg.BASE_PWM - REV_PWM_DELTA + kp_aci
                                    toplam_sag = cfg.BASE_PWM - REV_PWM_DELTA - kp_aci

                                    # --- isteğe bağlı: diferans slew-limit (pirueti keser) ---
                                    diff_now = toplam_sol - toplam_sag
                                    delta_diff = diff_now - prev_diff
                                    if   delta_diff >  REV_DIFF_SLEW_MAX: diff_now = prev_diff + REV_DIFF_SLEW_MAX
                                    elif delta_diff < -REV_DIFF_SLEW_MAX: diff_now = prev_diff - REV_DIFF_SLEW_MAX
                                    # diff_now = (sol - sağ) sabit kalsın, ortalamayı geri tabanda tut
                                    mean_pwm = (toplam_sol + toplam_sag) / 2.0
                                    toplam_sol = mean_pwm + diff_now/2.0
                                    toplam_sag = mean_pwm - diff_now/2.0
                                    prev_diff = diff_now

                                    # PWM sınırları
                                    if toplam_sol > 1980: toplam_sol = 1980
                                    if toplam_sol < 1100: toplam_sol = 1110
                                    if toplam_sag > 1980: toplam_sag = 1980
                                    if toplam_sag < 1100: toplam_sag = 1110

                                    controller.set_servo(cfg.SOL_MOTOR, toplam_sol)
                                    controller.set_servo(cfg.SAG_MOTOR, toplam_sag)
                                    time.sleep(REV_STEP_SEC)

                                # geri mod bittiyse bu iterasyonu kapat
                                continue


                            else:
                                # Sarı var ama ön koridorda değil → normal ileri
                                kp_aci = cfg.KpACI * aci_farki
                                TOPLAM_HATA  = TOPLAM_HATA + hedefe_mesafe
                                ki_mesafe    = TOPLAM_HATA * cfg.KiMESAFE
                                kp_mesafe    = cfg.KpMESAFE2 * hedefe_mesafe
                                duzeltme     = ki_mesafe + kp_mesafe
                                if duzeltme < 125: duzeltme = 125
                                toplam_sol = cfg.BASE_PWM + duzeltme + kp_aci
                                toplam_sag = cfg.BASE_PWM + duzeltme - kp_aci

                    # Ortak: PWM limitleri + yaz
                    if toplam_sol > 1980: toplam_sol = 1980
                    if toplam_sol < 1100: toplam_sol = 1110
                    if toplam_sag > 1980: toplam_sag = 1980
                    if toplam_sag < 1100: toplam_sag = 1110

                    controller.set_servo(cfg.SOL_MOTOR, toplam_sol)
                    controller.set_servo(cfg.SAG_MOTOR, toplam_sag)

                    if hedefe_mesafe < cfg.THRESHOLD:
                        mevcut_gorev = "IHA"
                        TOPLAM_HATA = 0.0
                        print("[INFO] GPS5 hedefine ulaşıldı.")

                elif mevcut_gorev == "IHA" and mission_started and not manual_mode:
                    print(mevcut_gorev)









                # Görüntüyü göster
                frame_resized = cv2.resize(frame, (960, 540))  # Resize the frame to desired dimensions960, 540
                
                # cfg içindeki GPS noktalarını sözlükte topla
                gps_points = {
                    "GPS1": (cfg.GPS1_enlem, cfg.GPS1_boylam),
                    "GPS2": (cfg.GPS2_enlem, cfg.GPS2_boylam),
                    "GPS3": (cfg.GPS3_enlem, cfg.GPS3_boylam),
                    "GPS4": (cfg.GPS4_enlem, cfg.GPS4_boylam),
                    "GPS5": (cfg.GPS5_enlem, cfg.GPS5_boylam),
                }

                # mevcut_gorev sözlükte varsa hedef koordinatı al
                hedef = gps_points.get(mevcut_gorev, (None, None))
                hedef_enlem, hedef_boylam = hedef
                
                # görüntüye zaman damgası ekle
                cv2.putText(frame_resized, datetime.datetime.now().strftime('%H:%M:%S'), (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                cv2.putText(frame_resized, f"FPS: {utils.nint(zed.get_current_fps())}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                cv2.putText(frame_resized, f"Görev: {mevcut_gorev}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                cv2.putText(frame_resized, f"sol:{utils.nint(controller.get_servo_pwm(cfg.SOL_MOTOR))}" , (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                cv2.putText(frame_resized, f"sag:{utils.nint(controller.get_servo_pwm(cfg.SAG_MOTOR))}" , (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

                cv2.putText(frame_resized, f"Hedefe mesafe: {hedefe_mesafe:.2f}", (400, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                cv2.putText(frame_resized, f"Rota tavsiyesi:{adviced_course:.0f}" , (400, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,100,255), 2)
                cv2.putText(frame_resized, f"aci farki:{aci_farki:.0f}" , (400, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,150,255), 2)
                cv2.putText(frame_resized, f"Heading: {magnetic_heading:.0f}", (400, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)
                cv2.putText(frame_resized, f"manyetometre durumu: {magnetic_heading_state}", (400, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)
                cv2.putText(frame_resized, f"Heading dogrulugu: {heading_dogruluk:.1f}", (400, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)



                cv2.putText(frame_resized, f"GPS DURUMU: {controller.get_gps_fix_type_verbose()}" , (400, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                
                if ida_enlem is not None and ida_boylam is not None:
                    cv2.putText(frame_resized, f"ida enlem: {ida_enlem:.6f}" , (400, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,100,255), 2)
                    cv2.putText(frame_resized, f"ida boylam: {ida_boylam:.6f}" , (400, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,100,255), 2)

                else:
                    cv2.putText(frame_resized,"ida konum: N/A",(400, 170),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,100,255),2)
                
                if hedef_enlem is not None and hedef_boylam is not None:
                    cv2.putText(frame_resized, f"hedef enlem: {hedef_enlem:.6f}" , (400, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,180,255), 2)
                    cv2.putText(frame_resized, f"hedef boylam: {hedef_boylam:.6f}" , (400, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,180,255), 2)

                else:
                    cv2.putText(frame_resized,"hedef konum: N/A",(400, 210),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,100,255),2)


                #video kaydet
                writer.enqueue(frame_resized.copy())

                if os.environ.get("DISPLAY"):
                    cv2.imshow("DHO KEMALREIS", frame_resized)
                    cv2.waitKey(1)
                
                #frame'i karşıya gönder sadece ssh için
                if cfg.STREAM == True:
                    _, buffer = cv2.imencode('.jpg', frame_resized)
                    data = pickle.dumps(buffer)
                    client_socket.sendall(struct.pack("!Q", len(data)) + data)

                
                # --- Görev noktalarını topla (her karede güncel) ---
                gps_points = {
                    "GPS1": {"lat": float(getattr(cfg, "GPS1_enlem", 0.0)), 
                            "lon": float(getattr(cfg, "GPS1_boylam", 0.0))},
                    "GPS2": {"lat": float(getattr(cfg, "GPS2_enlem", 0.0)), 
                            "lon": float(getattr(cfg, "GPS2_boylam", 0.0))},
                    "GPS3": {"lat": float(getattr(cfg, "GPS3_enlem", 0.0)), 
                            "lon": float(getattr(cfg, "GPS3_boylam", 0.0))},
                    "GPS4": {"lat": float(getattr(cfg, "GPS4_enlem", 0.0)), 
                            "lon": float(getattr(cfg, "GPS4_boylam", 0.0))},
                    "GPS5": {"lat": float(getattr(cfg, "GPS5_enlem", 0.0)), 
                            "lon": float(getattr(cfg, "GPS5_boylam", 0.0))},
                }

                hs = controller.get_horizontal_speed()
                payload = {
                    "t_ms": datetime.datetime.now().strftime('%H:%M:%S'),

                    # 2) SOL İTİCİ İTKİ İSTEĞİ (PWM)
                    "SOL_İTİCİ_İTKİ_İSTEĞİ_PWM": utils.nint(controller.get_servo_pwm(cfg.SOL_MOTOR)),

                    # 3) SAĞ İTİCİ İTKİ İSTEĞİ (PWM)
                    "SAĞ_İTİCİ_İTKİ_İSTEĞİ_PWM": utils.nint(controller.get_servo_pwm(cfg.SAG_MOTOR)),

                    # 4) İDA GERÇEK HIZ (m/s)
                    "İDA_GERÇEK_HIZ_mps": utils.nfloat(hs),

                    # 5) GERÇEK HEADING (°)
                    "GERÇEK_HEADING_deg": (f"{magnetic_heading:.0f}" if magnetic_heading is not None else None),

                    # 6) HEDEF HEADING (°)
                    "HEDEF_HEADING_deg": utils.nint(adviced_course),

                    # 7) HEADING SAĞLIĞI
                    "HEADING_SAĞLIĞI": magnetic_heading_state,

                    # 8) SONRAKİ GÖREV NOKTASI
                    "SONRAKİ_GÖREV_NOKTASI": mevcut_gorev,

                    # 9) MEVCUT KONUM (lat/lon ayrı anahtarlarla)
                    "MEVCUT_KONUM": {"lat": utils.nfloat(ida_enlem), "lon": utils.nfloat(ida_boylam)},


                    # 10) KALAN MESAFE (m)
                    "KALAN_MESAFE_m": utils.nint(hedefe_mesafe),

                    # 11) MANUEL MOD
                    "MANUEL_MOD": bool(manual_mode),

                    "FPS": utils.nfloat(round(zed.get_current_fps())),
                }
                payload["GÖREV_NOKTALARI"] = gps_points

                tx.send(payload)  #asenkron çalışıyor
                controller.set_servo(1,1500)

                
                
    except (KeyboardInterrupt, utils.EmergencyShutdown):
        print("\n[INFO] Ctrl+C alındı, kapanıyor...")
    finally:
        # --- kaynakları daima kapat ---
        try:
            writer.stop()   # dosya başlıkları doğru finalize edilir
        except Exception as e:
            print(f"[WARN] writer stop: {e}")

        try:
            cmd_rx.stop()
        except Exception:
            pass

        try:
            telemetry.close()
        except Exception:
            pass

        try:
            controller.disarm_vehicle()
        except Exception:
            pass

        try:
            if os.environ.get("DISPLAY"):
                cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            zed.close()
        except Exception:
            pass

        print("[INFO] Temiz çıkış tamamlandı.")


if __name__ == "__main__":
    main()