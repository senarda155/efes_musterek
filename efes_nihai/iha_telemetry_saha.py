#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DHO KEMALREİS — Telemetri Kaydedici
Uçuş verisi, tespit olayları ve haberleşme loglama modülü.
EFES-2026 | Milli Savunma Üniversitesi
"""

import csv
import json
import time
import logging
import threading
from pathlib import Path
from datetime import datetime

logger = logging.getLogger("TELEMETRY")


class TelemetriKaydedici:
    """
    İHA'nın anlık uçuş verisini ve tespit olaylarını CSV + JSON
    formatında kaydeder. Bağlantı kesildiğinde koordinat paketlerini
    tampon kuyruğunda bekletir, bağlantı kurulunca gönderir.
    """

    def __init__(self, arac, cfg):
        self.arac = arac
        self.cfg  = cfg
        self._aktif   = threading.Event()
        self._thread  = None
        self._tampon  = []
        self._lock    = threading.Lock()

        # Log klasörü
        Path(cfg.LOG_DIZINI).mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        self._csv_dosya  = Path(cfg.LOG_DIZINI) / f"telemetri_{ts}.csv"
        self._json_dosya = Path(cfg.LOG_DIZINI) / f"tespitler_{ts}.json"
        self._tespitler  = []

        self._csv_basliklari = [
            "zaman_unix", "timestamp",
            "lat", "lon", "alt_m",
            "yer_hizi_ms", "hava_hizi_ms",
            "yaw_deg", "pitch_deg", "roll_deg",
            "batarya_pct", "uydu_sayisi",
            "mod", "armed"
        ]

    def baslat(self):
        logger.info(f"Telemetri kaydedici başlatılıyor: {self._csv_dosya}")
        with open(self._csv_dosya, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._csv_basliklari)
            writer.writeheader()

        self._aktif.set()
        self._thread = threading.Thread(
            target=self._kayit_dongusu, daemon=True
        )
        self._thread.start()

    def _kayit_dongusu(self):
        while self._aktif.is_set():
            try:
                self._satir_kaydet()
            except Exception as e:
                logger.debug(f"Telemetri kayıt hatası: {e}")
            time.sleep(1.0 / self.cfg.TELEMETRI_HZ)

    def _satir_kaydet(self):
        if self.cfg.SIMULASYON:
            import random
            _konum_sim = self.arac.location.global_relative_frame
            satir = {
                "zaman_unix": round(time.time(), 3),
                "timestamp": datetime.now().isoformat(),
                "lat": round(_konum_sim.lat + random.uniform(-0.0001, 0.0001), 7),
                "lon": round(_konum_sim.lon + random.uniform(-0.0001, 0.0001), 7),
                "alt_m": round(_konum_sim.alt, 2),
                "yer_hizi_ms": round(8.0, 2),
                "hava_hizi_ms": round(8.0, 2),
                "yaw_deg": round(0.0, 1),
                "pitch_deg": round(0.0, 1),
                "roll_deg": round(0.0, 1),
                "batarya_pct": self.arac.battery.level,
                "uydu_sayisi": self.arac.gps_0.satellites_visible,
                "mod": self.arac.mode.name,
                "armed": int(self.arac.armed),
            }
            with open(self._csv_dosya, 'a', newline='') as f:
                csv.DictWriter(f, fieldnames=self._csv_basliklari).writerow(satir)
            return
        try:
            konum  = self.arac.location.global_relative_frame
            tutum  = self.arac.attitude
            batarya = self.arac.battery
            gps    = self.arac.gps_0
            import math
        except Exception:
            return

        satir = {
            "zaman_unix"   : round(time.time(), 3),
            "timestamp"    : datetime.now().isoformat(),
            "lat"          : round(konum.lat, 7),
            "lon"          : round(konum.lon, 7),
            "alt_m"        : round(konum.alt, 2),
            "yer_hizi_ms"  : round(
                (self.arac.velocity[0]**2 + self.arac.velocity[1]**2)**0.5, 2
            ),
            "hava_hizi_ms" : round(self.arac.airspeed, 2),
            "yaw_deg"      : round(math.degrees(tutum.yaw),   1),
            "pitch_deg"    : round(math.degrees(tutum.pitch), 1),
            "roll_deg"     : round(math.degrees(tutum.roll),  1),
            "batarya_pct"  : batarya.level,
            "uydu_sayisi"  : gps.satellites_visible,
            "mod"          : str(self.arac.mode.name),
            "armed"        : int(self.arac.armed),
        }

        with open(self._csv_dosya, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=self._csv_basliklari).writerow(satir)

    def tespit_kaydet(self, tespit: dict):
        with self._lock:
            self._tespitler.append(tespit)
        with open(self._json_dosya, 'w') as f:
            json.dump(self._tespitler, f, ensure_ascii=False, indent=2)

    def kuyruğa_kaydet(self, paket: dict):
        with self._lock:
            if len(self._tampon) < self.cfg.KOORDINAT_KUYRUĞU:
                self._tampon.append(paket)
                logger.debug(
                    f"Tampon: {len(self._tampon)}/{self.cfg.KOORDINAT_KUYRUĞU}"
                )

    def tampon_bos_mu(self) -> bool:
        return len(self._tampon) == 0

    def tampon_al(self) -> list:
        with self._lock:
            temp = list(self._tampon)
            self._tampon.clear()
        return temp

    def durdur(self):
        self._aktif.clear()
        if self._thread:
            self._thread.join(timeout=3)
        logger.info(f"Telemetri kaydedici durduruldu: {self._csv_dosya}")
