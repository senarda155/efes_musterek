#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DHO KEMALREİS — İHA Konfigürasyon Parametreleri
EFES-2026 | Milli Savunma Üniversitesi
"""

import math


class IHAConfig:
    # ── Uçuş Kontrolcü ──────────────────────────────
    FC_PORT      : str   = "/dev/ttyTHS1"   # Jetson Orin Nano ↔ Orange Cube+
    FC_BAUD_RATE : int   = 921600
    SIMULASYON   : bool  = True

    # ── Görev Parametreleri ──────────────────────────
    GOREV_ID         : str   = "EFES2026_001"
    DEVRIYE_IRTIFA   : float = 30.0     # metre (AGL)
    HAVA_HIZI        : float = 8.0      # m/s  (~29 km/h)
    WP_KABUL_RADIUS  : float = 4.0      # metre

    # Devriye merkez koordinatı (yarışma bölgesi)
    DEVRIYE_MERKEZ = {"lat": 38.4192, "lon": 27.1287}

    # Devriye poligon genişliği / yüksekliği (metre)
    DEVRIYE_EN     : float = 200.0
    DEVRIYE_BOY    : float = 150.0
    DEVRIYE_HATTI  : float = 20.0   # paralel hat aralığı (örter alan tarama)

    # ── Kamera / Görüntü İşleme ──────────────────────
    KAMERA_GENISLIK : int   = 1920
    KAMERA_YUKSEKLIK: int   = 1080
    KAMERA_FPS      : int   = 30
    KAMERA_HFOV_DEG : float = 84.0    # Sony IMX477 yatay FOV
    KAMERA_VFOV_DEG : float = 53.0

    # Pre-compute tan yarı-açılar (koordinat dönüşümü için)
    FOV_TAN_H = math.tan(math.radians(KAMERA_HFOV_DEG / 2.0))
    FOV_TAN_V = math.tan(math.radians(KAMERA_VFOV_DEG / 2.0))

    # ── YOLO / Tespit ────────────────────────────────
    YOLO_MODEL_PATH   : str   = r"C:\Users\MONSTER\PycharmProjects\PythonProject\goruntu\efes.yolov8\runs\detect\train-3\weights\best.pt"
    YOLO_CONF_ESIK    : float = 0.45    # Güven skoru eşiği
    YOLO_NMS_ESIK     : float = 0.40    # Non-maximum suppression
    YOLO_SINIFLAR     = [               # Tespit edilecek sınıflar
        "kacakcilik_botu",
        "gocmen_botu",
        "dusman_iha",
        "dusman_ida",
        "deniz_kazazedesi",
        "sahil_guvenligi_botu",   # dost — angajman yapılmaz
    ]
    DUSMAN_SINIFLAR   = [
        "kacakcilik_botu",
        "gocmen_botu",
        "dusman_iha",
        "dusman_ida",
    ]

    # ── İDA Haberleşme (RFD900x 915 MHz) ─────────────
    IDA_COMM_PORT     : str   = "/dev/ttyUSB0"
    IDA_COMM_BAUD     : int   = 57600
    IDA_COMM_PROTOKOL : str   = "mavlink2"   # MAVLink v2
    IDA_SYSTEM_ID     : int   = 2            # İDA MAVLink system ID
    IDA_COMP_ID       : int   = 1
    IDA_TIMEOUT_S     : float = 5.0          # ACK bekleme süresi

    # Yedek: UDP üzerinden simülasyon haberleşmesi
    IDA_SIM_IP        : str   = "127.0.0.1"
    IDA_SIM_PORT      : int   = 14552

    # ── Telemetri / Loglama ──────────────────────────
    LOG_DIZINI        : str   = "logs/"
    TELEMETRI_HZ      : int   = 2           # log kayıt frekansı
    KOORDINAT_KUYRUĞU : int   = 50          # bağlantı kesilince tampon

    # ── Güvenlik Limitleri ───────────────────────────
    MAX_IRTIFA        : float = 120.0       # metre (yasal limit)
    MIN_BATARYA_PCT   : int   = 20          # %  — RTL tetik eşiği
    GEOFENCE_RADIUS   : float = 500.0       # metre — görev sınırı
