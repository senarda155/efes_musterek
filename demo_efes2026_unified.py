#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
EFES-2026 Unified Simulation Script
Müşterek Harekat: İHA ve İDA Otonom Sistem Entegrasyonu

Bu script donanım bağımlılıklarını (Jetson, ZED, OrangeCube vb.) bypass ederek
Windows/Linux üzerinde masaüstünde çalışan bir "Sistemlerin Sistemi" demosu sunar.
Orijinal repo dosyaları (efes_nihai) olduğu gibi import edilerek kullanılır.
"""

import sys
import os
import time
import math
import struct
import threading
import queue
from collections import namedtuple

# ==========================================
# 1. DONANIM BYPASS (MOCKING)
# ==========================================
class DummyModule:
    """Sahte modül sınıfı. Olmayan özellikleri çağırırken de kendini döndürür/yok sayar."""
    def __getattr__(self, name):
        if name in ('__path__', '__file__'):
            return None
        return DummyModule()
    def __call__(self, *args, **kwargs):
        return DummyModule()

# Kritik donanım kütüphanelerini mock'la
sys.modules.setdefault('pyzed.sl', DummyModule())
sys.modules.setdefault('pyzed', DummyModule())
sys.modules.setdefault('dronekit', DummyModule())
sys.modules.setdefault('pymavlink', DummyModule())
sys.modules.setdefault('pymavlink.mavutil', DummyModule())
sys.modules.setdefault('serial', DummyModule())
sys.modules.setdefault('ultralytics', DummyModule())
sys.modules.setdefault('supervision', DummyModule())
sys.modules.setdefault('torch', DummyModule())
sys.modules.setdefault('cv2', DummyModule())
sys.modules.setdefault('numpy', DummyModule())

# Orijinal modüllerin dizinini PYTHONPATH'e ekle
sys.path.insert(0, os.path.abspath("efes_nihai"))

import config
import navigasyon

# ==========================================
# 2. ORKESTRASYON VE LOGLAMA
# ==========================================
log_q = queue.Queue()

# Terminal renk kodları
COLOR_IHA = '\033[96m'  # Cyan / Mavi
COLOR_IDA = '\033[95m'  # Magenta / Pembe
COLOR_RESET = '\033[0m'

def logger_thread():
    """Tüm logların birbirine karışmadan sırayla basılmasını sağlar."""
    while True:
        msg = log_q.get()
        if msg is None:
            break
        print(msg)
        log_q.task_done()

def log_iha(text):
    log_q.put(f"{COLOR_IHA}[İHA] {text}{COLOR_RESET}")

def log_ida(text):
    log_q.put(f"{COLOR_IDA}[İDA] {text}{COLOR_RESET}")

# İHA ve İDA arası haberleşme kuyruğu (RFD900x simülasyonu)
bridge_q = queue.Queue()

# ==========================================
# 3. İHA SİMÜLASYONU
# ==========================================
class IHASim(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.waypoints = [
            (config.GPS1_enlem, config.GPS1_boylam),
            (config.GPS2_enlem, config.GPS2_boylam),
            (config.GPS3_enlem, config.GPS3_boylam), # Bu noktada tespit yapılacak
            (config.GPS4_enlem, config.GPS4_boylam)
        ]
        self.current_lat = self.waypoints[0][0]
        self.current_lon = self.waypoints[0][1]
        self.alt = 30.0

    def run(self):
        log_iha("Sistem başlatılıyor...")
        time.sleep(1)
        log_iha(f"Kalkış yapıldı. Devriye irtifası: {self.alt}m")
        time.sleep(1)

        for i, wp in enumerate(self.waypoints):
            log_iha(f"Waypoint {i+1} hedefine ilerleniyor...")

            # WP'ye gidiş simülasyonu (2 saniye uçuş)
            time.sleep(2)
            self.current_lat = wp[0]
            self.current_lon = wp[1]
            log_iha(f"Waypoint {i+1} ulaşıldı: ({self.current_lat:.6f}, {self.current_lon:.6f})")

            # 3. Waypoint'te tespiti tetikle (index 2)
            if i == 2:
                log_iha("!!! YAPAY TESPİT TETİKLENDİ !!!")
                log_iha("Sınıf: dusman_ida, Güven: 0.91")

                # İDA'ya yollanacak paket: MAVLink ID 229 (<BddfBHH)
                # B: msg_id(229)
                # d: lat (İHA'nın o anki konumu)
                # d: lon (İHA'nın o anki konumu)
                # f: alt
                # B: class_id (3 = dusman_ida, config'e göre)
                # H: confidence (0.91 * 10000 = 9100)
                # H: accuracy (varsayılan 500)

                msg_id = 229
                class_id = 3
                confidence = int(0.91 * 10000)
                accuracy = 500

                paket = struct.pack(
                    "<BddfBHH",
                    msg_id,
                    self.current_lat,
                    self.current_lon,
                    self.alt,
                    class_id,
                    confidence,
                    accuracy
                )

                log_iha("Tespit koordinatları RFD900x üzerinden İDA'ya gönderiliyor...")
                bridge_q.put(paket)
                break # Görevi kesip RTL yapalım

        log_iha("Devriye / Tespit tamamlandı. RTL (Eve Dönüş) başlatılıyor...")
        time.sleep(2)
        log_iha("İniş tamamlandı. DISARM.")

# ==========================================
# 4. İDA SİMÜLASYONU
# ==========================================
class IDASim(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.current_lat = config.GPS1_enlem
        self.current_lon = config.GPS1_boylam

        # İDA otonom olarak GPS5 noktasına gidiyor varsayıyoruz
        self.target_lat = config.GPS5_enlem
        self.target_lon = config.GPS5_boylam

        self.mode = "OTONOM"

    def run(self):
        log_ida("Sistem başlatılıyor...")
        time.sleep(1.5)
        log_ida(f"Otonom modda ({self.target_lat:.6f}, {self.target_lon:.6f}) hedefine ilerleniyor...")

        while True:
            # bridge_q'da mesaj var mı kontrol et
            try:
                paket = bridge_q.get_nowait()
                self.process_mavlink_message(paket)
            except queue.Empty:
                pass

            # Hedefe doğru ilerleme simülasyonu
            dist = navigasyon.haversine(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            bearing = navigasyon.calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)

            if dist < 1.0:
                if self.mode == "TRANSIT (Kamikaze)":
                    log_ida("KAMİKAZE ANGAJMANI TAMAMLANDI!")
                    break
                else:
                    log_ida("Hedefe ulaşıldı.")
                    break

            if self.mode == "TRANSIT (Kamikaze)" and 3.0 <= dist <= 4.0:
                log_ida(f"Mesafe: {dist:.1f}m -> Nudge (Sakınma) manevrası yapılıyor...")

            # Dinamik adım: Uzaksa hızlı, yakınsa yavaş git
            step_size = 5.0 if dist > 10 else 1.0
            step_size = min(step_size, dist) # Hedefi geçmemek için

            # Yeni konumu hesapla
            self.current_lat, self.current_lon = navigasyon.destination_point(
                self.current_lat, self.current_lon, bearing, step_size
            )

            if self.mode == "TRANSIT (Kamikaze)":
                log_ida(f"Kamikaze hedefi takip ediliyor. Mesafe: {dist:.1f}m")

            time.sleep(1) # Simülasyon hızı

    def process_mavlink_message(self, paket):
        # Paketi unpack et (<BddfBHH)
        try:
            unpacked = struct.unpack("<BddfBHH", paket)
            msg_id = unpacked[0]
            lat = unpacked[1]
            lon = unpacked[2]

            if msg_id == 229:
                log_ida(f"RFD900x'ten hedef paketi alındı: MAVLink ID {msg_id}")
                log_ida(f"Yeni Hedef: ({lat:.6f}, {lon:.6f})")

                self.mode = "TRANSIT (Kamikaze)"
                self.target_lat = lat
                self.target_lon = lon
                log_ida("Görev modu TRANSIT (Kamikaze) olarak güncellendi!")
        except Exception as e:
            log_ida(f"Paket çözme hatası: {e}")


# ==========================================
# ANA ÇALIŞTIRMA FONKSİYONU
# ==========================================
def main():
    print("==================================================")
    print("EFES-2026 Müşterek Harekat Masaüstü Simülasyonu")
    print("==================================================")

    # Log thread başlat
    log_t = threading.Thread(target=logger_thread)
    log_t.daemon = True
    log_t.start()

    iha = IHASim()
    ida = IDASim()

    ida.start()
    time.sleep(0.5)
    iha.start()

    # Her iki threadin bitmesini bekle
    iha.join()
    ida.join()

    # Log kuyruğunun boşalmasını bekle ve sonlandır
    log_q.join()
    log_q.put(None)
    log_t.join()

    print("Simülasyon başarıyla tamamlandı.")

if __name__ == "__main__":
    main()
