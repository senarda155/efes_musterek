#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DHO KEMALREİS — Hedef Tespit Modülü
YOLO v11 + TensorRT | Sony IMX477 @ 30 FPS
EFES-2026 | Milli Savunma Üniversitesi
"""

import logging
import time
import numpy as np
from typing import List, Dict, Optional

logger = logging.getLogger("DETECTOR")


class HedefTespitModulu:
    """
    NVIDIA TensorRT ile hızlandırılmış YOLO v11 tabanlı
    gerçek zamanlı hedef tespit ve sınıflandırma modülü.

    Desteklenen Sınıflar:
        0 - kacakcilik_botu
        1 - gocmen_botu
        2 - dusman_iha
        3 - dusman_ida
        4 - deniz_kazazedesi
        5 - sahil_guvenligi_botu
    """

    RENK_HARITASI = {
        "kacakcilik_botu"     : (0,   0, 255),   # kırmızı
        "gocmen_botu"         : (0, 165, 255),   # turuncu
        "dusman_iha"          : (0,   0, 200),   # koyu kırmızı
        "dusman_ida"          : (128,  0, 128),  # mor
        "deniz_kazazedesi"    : (0, 255, 255),   # sarı
        "sahil_guvenligi_botu": (0, 255,   0),   # yeşil (dost)
    }

    def __init__(self, cfg):
        self.cfg   = cfg
        self.model = None
        self.kamera = None
        self._kare_sayaci = 0
        self._fps_sayaci  = 0
        self._fps_zaman   = time.time()

    # ─────────────────────────────────────────
    # MODEL YÜKLEMESİ
    # ─────────────────────────────────────────
    def yukle(self) -> bool:
        logger.info("YOLO v11 TensorRT motoru yükleniyor...")
        try:
            if self.cfg.SIMULASYON:
                # SITL / gösterim modu — sahte çıkarım motoru
                self.model = _SahteTRTMotor(self.cfg)
                logger.info("  [SIM] Sahte TRT motoru aktif (SITL modu)")
            else:
                # Gerçek donanım — ultralytics + TensorRT
                from ultralytics import YOLO
                self.model = YOLO(self.cfg.YOLO_MODEL_PATH, task="detect")
                # self.model.to("cuda")
                logger.info(f"  Model   : {self.cfg.YOLO_MODEL_PATH}")

            logger.info(f"  Güven ε : {self.cfg.YOLO_CONF_ESIK}")
            logger.info(f"  NMS   ε : {self.cfg.YOLO_NMS_ESIK}")
            logger.info(f"  Cihaz   : {'SIM' if self.cfg.SIMULASYON else 'CUDA:0 (Jetson Orin Nano)'}")
            logger.info(f"  Sınıflar: {len(self.cfg.YOLO_SINIFLAR)} adet")
            logger.info("YOLO motoru hazır ✓")

            # Kamera akışını başlat
            self.kamera = _KameraAkis(
                self.cfg.KAMERA_GENISLIK,
                self.cfg.KAMERA_YUKSEKLIK,
                self.cfg.KAMERA_FPS
            )
            self.kamera.baslat()
            return True

        except Exception as e:
            logger.error(f"Model yükleme hatası: {e}")
            return False

    # ─────────────────────────────────────────
    # KARE İŞLEME — ANA FONKSİYON
    # ─────────────────────────────────────────
    def kare_isle(self) -> List[Dict]:
        """
        Kameradan bir kare alır, YOLO ile işler,
        güven eşiğini geçen tespitleri döndürür.

        Returns:
            List[Dict]: Her elemanı şu anahtarları içerir:
                sinif, confidence, bbox [x1,y1,x2,y2], kare_id
        """
        kare = self.kamera.kare_al()
        if kare is None:
            return []

        self._kare_sayaci += 1

        # Çıkarım
        baslangic = time.perf_counter()
        ham_tespitler = self.model.cikari(kare)
        sure_ms = (time.perf_counter() - baslangic) * 1000

        # FPS hesapla
        self._fps_sayaci += 1
        gecen = time.time() - self._fps_zaman
        if gecen >= 5.0:
            fps = self._fps_sayaci / gecen
            logger.debug(f"[DETECTOR] FPS: {fps:.1f} | Çıkarım: {sure_ms:.1f}ms")
            self._fps_sayaci = 0
            self._fps_zaman  = time.time()

        # Filtrele ve formatla
        tespitler = []
        for det in ham_tespitler:
            if det['confidence'] < self.cfg.YOLO_CONF_ESIK:
                continue
            if det['sinif'] not in self.cfg.YOLO_SINIFLAR:
                continue

            tespitler.append({
                'kare_id'    : self._kare_sayaci,
                'sinif'      : det['sinif'],
                'confidence' : det['confidence'],
                'bbox'       : det['bbox'],     # [x1, y1, x2, y2]
                'merkez'     : self._merkez_hesapla(det['bbox']),
                'alan_px2'   : self._alan_hesapla(det['bbox']),
                'sure_ms'    : round(sure_ms, 2),
            })

        return tespitler

    # ─────────────────────────────────────────
    # YARDIMCI
    # ─────────────────────────────────────────
    @staticmethod
    def _merkez_hesapla(bbox) -> tuple:
        return (
            int((bbox[0] + bbox[2]) / 2),
            int((bbox[1] + bbox[3]) / 2)
        )

    @staticmethod
    def _alan_hesapla(bbox) -> int:
        return abs(bbox[2] - bbox[0]) * abs(bbox[3] - bbox[1])


# ─────────────────────────────────────────────────────────
# YARDIMCI SINIFLAR (simülasyon / kamera soyutlama katmanı)
# ─────────────────────────────────────────────────────────

class _SahteTRTMotor:
    """
    Gerçek sistemde ultralytics.YOLO veya tensorrt engine kullanılır.
    Bu sınıf entegrasyon testleri ve SITL simülasyonu içindir.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._rng = np.random.default_rng(seed=42)
        self._tespit_olasiligi = 0.05     # kare başına ~%5 tespit olasılığı
        self._son_tespit = 0

    def cikari(self, kare: np.ndarray) -> List[Dict]:
        # Her 5 saniyede bir tespit şansı ver (gerçekçi bekleme süresi)
        if time.time() - self._son_tespit < 5.0:
            return []

        # %30 ihtimalle tespit etsin
        if self._rng.random() > 0.30:
            return []

        self._son_tespit = time.time()
        H, W = kare.shape[:2]

        # Rastgele bbox üret
        w = int(self._rng.integers(80, 150))
        h = int(self._rng.integers(60, 100))
        x1 = int(self._rng.integers(W // 4, W // 2))
        y1 = int(self._rng.integers(H // 4, H // 2))
        x2 = x1 + w
        y2 = y1 + h

        sinif = self._rng.choice(
            [s for s in self.cfg.YOLO_SINIFLAR if s in self.cfg.DUSMAN_SINIFLAR]
        )
        conf = float(self._rng.uniform(0.85, 0.96))  # Yüksek güven skoru

        return [{'sinif': sinif, 'confidence': conf, 'bbox': [x1, y1, x2, y2]}]


class _KameraAkis:
    """
    CSI kamera soyutlama katmanı.
    Gerçek sistemde GStreamer pipeline ile açılır:
      nvarguscamerasrc sensor-id=0 !
      video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 !
      nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! appsink
    """

    def __init__(self, w: int, h: int, fps: int):
        self.w, self.h, self.fps = w, h, fps
        self._cap = None

    def baslat(self):
        logger.info(f"Kamera açılıyor: {self.w}×{self.h} @ {self.fps}fps")
        try:
            import cv2
            gst_pipeline = (
                f"nvarguscamerasrc sensor-id=0 ! "
                f"video/x-raw(memory:NVMM),width={self.w},height={self.h},"
                f"framerate={self.fps}/1 ! nvvidconv ! "
                f"video/x-raw,format=BGRx ! videoconvert ! "
                f"video/x-raw,format=BGR ! appsink"
            )
            self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if self._cap.isOpened():
                logger.info("Kamera akışı (GStreamer/NVMM) başarıyla açıldı ✓")
            else:
                logger.warning("GStreamer başarısız, USB kameraya geçiliyor...")
                self._cap = cv2.VideoCapture(0)
        except ImportError:
            logger.warning("OpenCV yok — sahte kare üretimi aktif")
            self._cap = None

    def kare_al(self) -> Optional[np.ndarray]:
        if self._cap and self._cap.isOpened():
            ret, kare = self._cap.read()
            return kare if ret else None
        # Sahte kare (simülasyon)
        return np.zeros((self.h, self.w, 3), dtype=np.uint8)

    def serbest_birak(self):
        if self._cap:
            self._cap.release()
