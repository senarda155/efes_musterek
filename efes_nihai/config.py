# config.py
import os 
SERIAL_PORT = os.getenv("TELEM_PORT", "/dev/ttyUSB1")  # kendi portun
SERIAL_BAUD = int(os.getenv("TELEM_BAUD", "57600"))
# telemetri açılmazsa terminalde çalıştır: sudo chmod a+rw /dev/ttyUSB0

MEVCUT_GOREV = "GPS1"       #TODO: yarisma gunu GPS1 olacak
MANUAL_MODE = True          #TODO: yarisma gunu True olacak
MISSION_STARTED = False     #TODO: yarisma gunu False olacak
STREAM = False            #TODO: yarisma gunu False olacak

OTONOM_IHA = True           #TODO: iha görevi otonom yapılacaksa true yap
angajman_renk = "kirmizi"   #TODO: iha görevi manuel yapılacaksa kabloyla yüklenecek

# Görüntü işleme
YOLO_CONFIDENCE = 0.4

MIN_DUZELTME = 125 #ilk görevde ida hedefi tam pruvasına aldıysa her iki motora eklenecek minimum düzeltme pwm'i
BASE_PWM = 1500
BASE_PWM_ACI = 1500
BASE_PWM_MESAFE = 1500
KiMESAFE = 0.001
KpMESAFE = 14
KpMESAFE2 = 8
KpACI = 2.5

#gorev3 veri gelmezse
IHA_HOLD_RADIUS_M = 1.5 # bunun içinde "tutuş" moduna geç
IHA_HOLD_DEADBAND_M = 0.5 # çok küçük gps salınımları görmezden gel
IHA_HOLD_KP_DIST = 60.0 # PWM/metre (tutuş modundaki ileri/geri itiş)
IHA_HOLD_KI_DIST = 6.0 # küçük integral, akıntıyı kompanze eder
IHA_HOLD_KP_YAW = KpACI * 0.8 # baş tutarken yumuşak yaw Kp
IHA_HOLD_YAW_CLAMP = 90.0 # ± maks yaw PWM
IHA_HOLD_THR_CLAMP = 120.0 # ± maks ileri/geri PWM
# Costmap
COSTMAP_SIZE_PX = (2000, 2000)
COSTMAP_RES_M_PER_PX = 0.1 # çözünürlük (m/px) # Boyut ve çözünürlük: 0.2 m/px -> 2000x2000 piksel (400x400 m alan)
COSTMAP_BG_BGR = (200, 230, 255) # açık mavi tuval rengi (BGR)
COSTMAP_DET_RADIUS_M = 0.5 # tespit daire yarıçapı (metre)
COSTMAP_PATH_DIR = "costmap" # çıktı klasörü

# Motor pinleri
SOL_MOTOR = 5
SAG_MOTOR = 6



waypoint_width=4

THRESHOLD = 1.5
ACI_THRESHOLD_COK = 30
ACI_THRESHOLD_AZ = 8

CAM_HFOV = 110.0 # kamera yatay görüş açısı
CLEARANCE = 2.5 # waypoint kaydırma miktarı

# GPS koordinatları (enlem, boylam)
GPS1_enlem = 40.8630501 #TODO: SCRİPT ÇALIŞTIRILMADAN ÖNCE DÜZENLENİR.
GPS1_boylam = 29.2599517  # video için: dikdörtgenin 1.köşe koordinatları girilir

GPS2_enlem = 40.8629223
GPS2_boylam = 29.2599123  # video için: dikdörtgenin 2.köşe koordinatları girilir

GPS3_enlem = 40.8628156
GPS3_boylam = 29.2597343  # video için: dikdörtgenin 3.köşe koordinatları girilir

GPS4_enlem = 40.8626815
GPS4_boylam = 29.2594698 # video için: dikdörtgenin 4.köşe koordinatları girilir

GPS5_enlem = 40.8632559
GPS5_boylam =  29.2594437 # video için: İDA'nın eve dönüş,rıhtım, koordinatları girilir


