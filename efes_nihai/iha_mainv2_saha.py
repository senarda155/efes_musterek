#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DHO KEMALREİS - EFES-2026
İnsansız Hava Aracı (İHA) Ana Görev Kontrolcüsü
Otonom Keşif, Hedef Tespit ve İDA Koordinasyon Modülü

Platform     : NVIDIA Jetson Orin Nano
Uçuş Kontrol : ArduPilot (Orange Cube+)
Protokol     : MAVLink v2 over RFD900x (915 MHz)
Görüntü İşl. : YOLO v11 + TensorRT
"""

import time
import threading
import logging
import argparse
from datetime import datetime
import os
os.makedirs("logs", exist_ok=True)

import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative

from iha_config_saha    import IHAConfig
from iha_detector_saha  import HedefTespitModulu
from iha_comms_saha     import IDAHaberlesme
from iha_patrol_saha    import DevriyeRotalayici
from iha_telemetry_saha import TelemetriKaydedici

# ── SİMÜLASYON İÇİN MOCK SINIFLAR (DRONEKIT API UYUMLU) ─────────────
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class MockGlobalRelativeFrame:
    """DroneKit'teki location.global_relative_frame yapısını taklit eder"""
    lat: float = 38.4192
    lon: float = 27.1287
    alt: float = 0.0  # metre AGL

@dataclass
class MockLocation:
    """DroneKit'teki vehicle.location yapısını taklit eder"""
    global_relative_frame: MockGlobalRelativeFrame = field(default_factory=MockGlobalRelativeFrame)

@dataclass
class MockAttitude:
    yaw: float = 0.0      # radyan
    pitch: float = 0.0    # radyan
    roll: float = 0.0     # radyan

@dataclass
class MockGPS:
    fix_type: int = 3
    satellites_visible: int = 14

@dataclass
class MockBattery:
    level: int = 98

class MockVehicle:
    def __init__(self):
        self.location = MockLocation()  # ← İç içe yapı
        self.attitude = MockAttitude()
        self.gps_0 = MockGPS()
        self.battery = MockBattery()
        self.ekf_ok = True
        self.is_armable = True
        self.armed = False
        self.mode = type("Mode", (), {"name": "STABILIZE"})()
        self.velocity = [0.0, 0.0, 0.0]  # NED frame [vx, vy, vz]
        self.airspeed = 8.0
        self.version = "MockFirmware-v1.0"

    def simple_takeoff(self, alt):
        self.armed = True
        self.mode.name = "GUIDED"
        self.location.global_relative_frame.alt = alt  # ← Doğru erişim
        logger.info(f"   [MOCK] Kalkış tamamlandı: {alt}m")

    def simple_goto(self, loc, airspeed=None):
        # loc: LocationGlobalRelative objesi veya dict
        if hasattr(loc, 'lat'):
            self.location.global_relative_frame.lat = loc.lat
            self.location.global_relative_frame.lon = loc.lon
            self.location.global_relative_frame.alt = loc.alt
        else:
            self.location.global_relative_frame.lat = loc['lat']
            self.location.global_relative_frame.lon = loc['lon']
            self.location.global_relative_frame.alt = loc.get('alt', 30.0)
        self.mode.name = "GUIDED"
        logger.debug(f"   [MOCK] WP'ye ilerle: {self.location.global_relative_frame.lat:.6f}, {self.location.global_relative_frame.lon:.6f}")

    def close(self):
        """DroneKit uyumluluğu için sahte close metodu"""
        self.armed = False
        self.mode.name = "STABILIZE"
        logger.debug("[MOCK] Araç kapatıldı (simülasyon)")
# ─────────────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────
# Loglama ayarları
# ─────────────────────────────────────────────
# ─────────────────────────────────────────────
# Loglama ayarları (UTF-8 encoding ile Windows uyumlu)
# ─────────────────────────────────────────────
import sys
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[
        logging.FileHandler(
            f"logs/iha_gorev_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
            encoding='utf-8',  # ← BU SATIRI EKLE
            mode='a'
        ),
        # Windows terminalinde UTF-8 desteği yoksa ASCII fallback
        logging.StreamHandler(sys.stdout if sys.platform != 'win32' else sys.stderr)
    ]
)
logger = logging.getLogger("IHA_MAIN")


class IHAGorevKontrolcusu:
    """
    İHA'nın tüm otonom görev döngüsünü yöneten ana sınıf.
    Kalkış → Devriye → Tespit → Raporlama → İniş
    """

    def __init__(self, config: IHAConfig):
        self.cfg = config
        self.arac    = None          # DroneKit araç nesnesi
        self.detector = None
        self.komlink  = None
        self.rotalayici = None
        self.telemetri  = None

        self._calisma_bayragi = threading.Event()
        self._tespit_kuyrugu  = []
        self._lock = threading.Lock()
        # SİMÜLASYON KONTROLÜ
        if self.cfg.SIMULASYON:
            self.arac = MockVehicle()
            logger.warning(">>> SİMÜLASYON MODU: Mock Uçuş Kontrolcüsü Aktif <<<")

        logger.info("IHA Görev Kontrolcüsü başlatılıyor...")
        logger.info(f"Görev Bölgesi  : {self.cfg.DEVRIYE_MERKEZ}")
        logger.info(f"Devriye İrtifa : {self.cfg.DEVRIYE_IRTIFA} m")
        logger.info(f"Haberleşme     : {self.cfg.IDA_COMM_PORT}")

    # ─────────────────────────────────────────
    # 1) SİSTEM BAŞLATMA
    # ─────────────────────────────────────────
    def sistemi_baslat(self) -> bool:
        logger.info("── Sistem başlatma sekansı ──")

        # ── SİMÜLASYON MODU KONTROLÜ ─────────────
        if self.cfg.SIMULASYON:
            # MockVehicle zaten __init__'te atandı, sadece log düş
            logger.info(f"Bağlantı OK — Firmware: {self.arac.version} (SIM)")
        else:
            # ── GERÇEK DONANIM BAĞLANTISI ────────
            logger.info(f"ArduPilot bağlantısı: {self.cfg.FC_BAUD_RATE} bps @ {self.cfg.FC_PORT}")
            try:
                self.arac = connect(
                    self.cfg.FC_PORT,
                    baud=self.cfg.FC_BAUD_RATE,
                    wait_ready=True,
                    heartbeat_timeout=30
                )
                logger.info(f"Bağlantı OK — Firmware: {self.arac.version}")
            except Exception as e:
                logger.error(f"FC bağlantısı başarısız: {e}")
                return False
        # ─────────────────────────────────────────

        # Alt sistemleri başlat (her iki modda da çalışır)
        self.detector = HedefTespitModulu(self.cfg)
        self.komlink = IDAHaberlesme(self.cfg)
        self.rotalayici = DevriyeRotalayici(self.cfg)
        self.telemetri = TelemetriKaydedici(self.arac, self.cfg)

        if not self.detector.yukle():
            logger.error("Hedef tespit modülü yüklenemedi!")
            return False
        if not self.komlink.baglan():
            logger.error("İDA haberleşme bağlantısı kurulamadı!")
            return False
        self.telemetri.baslat()
        logger.info("Tüm alt sistemler hazır ✓")
        return True

    # ─────────────────────────────────────────
    # 2) PRE-FLIGHT KONTROLLER
    # ─────────────────────────────────────────
    def preflight_kontrol(self) -> bool:
        logger.info("── Pre-flight kontroller ──")

        kontroller = {
            "GPS Fix (3D)"   : lambda: self.arac.gps_0.fix_type >= 3,
            "Uydu Sayısı>8"  : lambda: self.arac.gps_0.satellites_visible >= 8,
            "Batarya>%80"    : lambda: self.arac.battery.level >= 80,
            "EKF Sağlıklı"   : lambda: self.arac.ekf_ok,
            "Armable"        : lambda: self.arac.is_armable,
        }

        tum_gecti = True
        for ad, kontrol in kontroller.items():
            try:
                sonuc = kontrol()
                durum = "✓" if sonuc else "✗"
                logger.info(f"  [{durum}] {ad}")
                if not sonuc:
                    tum_gecti = False
            except Exception as e:
                logger.warning(f"  [?] {ad} — kontrol hatası: {e}")
                tum_gecti = False

        return tum_gecti

    # ─────────────────────────────────────────
    # 3) OTONOM KALKIŞ
    # ─────────────────────────────────────────
    def otonom_kalkis(self, hedef_irtifa: float) -> bool:
        logger.info(f"── Otonom kalkış: hedef={hedef_irtifa}m ──")

        self.arac.mode = VehicleMode("GUIDED")
        time.sleep(1)

        logger.info("ARM komutu gönderiliyor...")
        self.arac.armed = True

        for _ in range(15):
            if self.arac.armed:
                break
            logger.debug("ARM bekleniyor...")
            time.sleep(1)

        if not self.arac.armed:
            logger.error("ARM başarısız!")
            return False

        logger.info(f"TAKEOFF → {hedef_irtifa}m")
        self.arac.simple_takeoff(hedef_irtifa)

        # İrtifaya ulaşılana kadar bekle
        if self.cfg.SIMULASYON:
            self.arac.location.alt = hedef_irtifa
            logger.info(f"Hedef irtifaya ulaşıldı: {hedef_irtifa:.1f}m ✓")
        else:
            while True:
                mevcut = self.arac.location.global_relative_frame.alt
                logger.debug(f"  İrtifa: {mevcut:.1f} / {hedef_irtifa:.1f} m")
                if mevcut >= hedef_irtifa * 0.95:
                    logger.info(f"Hedef irtifaya ulaşıldı: {mevcut:.1f}m ✓")
                    break
                time.sleep(0.5)

        return True

    # ─────────────────────────────────────────
    # 4) DEVRİYE DÖNGÜSÜ
    # ─────────────────────────────────────────
    def devriye_dongusu(self):
        logger.info("── Devriye döngüsü başladı ──")
        rota = self.rotalayici.rota_hesapla()
        logger.info(f"Toplam {len(rota)} waypoint üretildi")

        self._calisma_bayragi.set()
        tespit_thread = threading.Thread(
            target=self._tespit_dongusu, daemon=True
        )
        tespit_thread.start()

        wp_indeks = 0
        while self._calisma_bayragi.is_set():

            # ── GÜVENLİK KONTROLLERİ ─────────────────────────────────
            # Batarya düşük → devriyeyi kes, RTL tetiklensin
            if self.arac.battery.level < self.cfg.MIN_BATARYA_PCT:
                logger.warning(
                    f"[GÜVENLİK] Batarya %{self.arac.battery.level} — "
                    f"eşik %{self.cfg.MIN_BATARYA_PCT} altında! RTL tetikleniyor."
                )
                break
            # ─────────────────────────────────────────────────────────

            hedef_konum = rota[wp_indeks % len(rota)]
            self._waypointte_ilerle(hedef_konum)

            # Waypointe yakın mı?
            if self._mesafe_kontrol(hedef_konum, esik=self.cfg.WP_KABUL_RADIUS):
                logger.info(f"WP-{wp_indeks+1} ulaşıldı → sonraki noktaya")
                wp_indeks += 1

            # Tespit kuyruğunu işle
            with self._lock:
                if self._tespit_kuyrugu:
                    tespit = self._tespit_kuyrugu.pop(0)
                    self._ida_ye_rapor_et(tespit)

            time.sleep(0.2)

    # ─────────────────────────────────────────
    # 5) GÖRÜNTÜ TEBLİT DÖNGÜSÜ (ayrı thread)
    # ─────────────────────────────────────────
    def _tespit_dongusu(self):
        logger.info("[DETECTOR] Görüntü işleme thread'i aktif")
        while self._calisma_bayragi.is_set():
            tespitler = self.detector.kare_isle()
            for t in tespitler:
                gps = self._piksel_koordinat_donustur(
                    t['bbox'], t['confidence']
                )
                if gps:
                    t['gps'] = gps
                    with self._lock:
                        self._tespit_kuyrugu.append(t)
                    logger.warning(
                        f"[TESPIT] {t['sinif'].upper():15s} | "
                        f"Güven:{t['confidence']:.2f} | "
                        f"GPS: {gps['lat']:.6f}, {gps['lon']:.6f} | "
                        f"İrtifa: {gps['alt']:.1f}m"
                    )
            time.sleep(1.0 / self.cfg.KAMERA_FPS)

    # ─────────────────────────────────────────
    # 6) PIKSELden GPS KOORDİNAT DÖNÜŞÜMÜ
    # ─────────────────────────────────────────
    def _piksel_koordinat_donustur(self, bbox, confidence) -> dict:
        """
        Kamera görüntüsündeki tespit kutusunun merkezini,
        uçuş irtifası ve kamera FOV parametreleriyle
        gerçek GPS koordinatına dönüştürür.
        """
        iha_konum = self.arac.location.global_relative_frame
        iha_irtifa = iha_konum.alt
        iha_yaw   = self.arac.attitude.yaw   # radyan

        # Bounding box merkez pikseli
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0

        # Piksel → zemin offset (metre)
        # Yatay: 2 * irtifa * tan(HFOV/2) / genislik
        yer_per_px_x = (2 * iha_irtifa * self.cfg.FOV_TAN_H) / self.cfg.KAMERA_GENISLIK
        yer_per_px_y = (2 * iha_irtifa * self.cfg.FOV_TAN_V) / self.cfg.KAMERA_YUKSEKLIK

        # Merkeze göre offset
        dx_m = (cx - self.cfg.KAMERA_GENISLIK  / 2.0) * yer_per_px_x
        dy_m = (cy - self.cfg.KAMERA_YUKSEKLIK / 2.0) * yer_per_px_y

        # Yaw rotasyonu uygula
        import math
        cos_y, sin_y = math.cos(iha_yaw), math.sin(iha_yaw)
        north_m =  dx_m * sin_y + dy_m * cos_y
        east_m  =  dx_m * cos_y - dy_m * sin_y

        # Metre → derece dönüşümü (WGS84 yaklaşımı)
        dlat = north_m / 111_320.0
        dlon = east_m  / (111_320.0 * math.cos(math.radians(iha_konum.lat)))

        return {
            'lat': iha_konum.lat + dlat,
            'lon': iha_konum.lon + dlon,
            'alt': 0.0,                   # deniz seviyesi hedef
            'accuracy_m': iha_irtifa * 0.05,   # tahmini ~%5 irtifa hatası
            'confidence': confidence
        }

    # ─────────────────────────────────────────
    # 7) İDA'ya RAPOR GÖNDER
    # ─────────────────────────────────────────
    def _ida_ye_rapor_et(self, tespit: dict):
        # ── DOST UNSUR FİLTRESİ ──────────────────────────────────────
        # Sadece düşman sınıfları İDA'ya iletilir; dost unsurlar atlanır
        if tespit['sinif'] not in self.cfg.DUSMAN_SINIFLAR:
            logger.info(
                f"[DOST UNSUR] {tespit['sinif'].upper()} tespit edildi — "
                f"angajman dışı, İDA'ya iletilmedi."
            )
            return
        # ─────────────────────────────────────────────────────────────

        gps = tespit.get('gps')
        if not gps:
            return

        paket = {
            'zaman'        : time.time(),
            'hedef_sinifi' : tespit['sinif'],
            'confidence'   : round(tespit['confidence'], 4),
            'lat'          : round(gps['lat'], 7),
            'lon'          : round(gps['lon'], 7),
            'alt'          : round(gps['alt'], 2),
            'accuracy_m'   : round(gps['accuracy_m'], 2),
            'iha_irtifa'   : round(
                self.arac.location.global_relative_frame.alt, 2
            ),
            'gorev_id'     : self.cfg.GOREV_ID,
        }

        if self.komlink.koordinat_gonder(paket):
            logger.info(
                f"[KOMLINK] ✉ İDA'ya koordinat iletildi → "
                f"({paket['lat']:.6f}, {paket['lon']:.6f}) "
                f"| Hedef: {paket['hedef_sinifi']}"
            )
        else:
            logger.error("[KOMLINK] ✗ İDA'ya ulaşılamadı! Koordinat kayıt altına alındı.")
            self.telemetri.kuyruğa_kaydet(paket)

    # ─────────────────────────────────────────
    # 8) OTONOM İNİŞ
    # ─────────────────────────────────────────
    def otonom_inis(self):
        logger.info("── RTL (Return-to-Launch) başlatılıyor ──")
        self._calisma_bayragi.clear()
        time.sleep(0.5)
        if self.cfg.SIMULASYON:
            # Simülasyonda anında iniş simüle et
            self.arac.location.global_relative_frame.alt = 0.0
            self.arac.armed = False
            logger.info("   [MOCK] İniş tamamlandı, DISARM ✓")
            return

        self.arac.mode = VehicleMode("RTL")
        logger.info("Mod: RTL ✓ — Ana üsse dönüş başladı")

        while self.arac.armed:
            if self.cfg.SIMULASYON:
                irtifa = self.arac.location.global_relative_frame.alt
            else:
                irtifa = self.arac.location.global_relative_frame.alt
            logger.debug(f"  İniş irtifa: {irtifa:.1f}m")
            time.sleep(1)

        logger.info("İniş tamamlandı. DISARM ✓")

    # ─────────────────────────────────────────
    # YARDIMCI: waypoint ilerle
    # ─────────────────────────────────────────
    def _waypointte_ilerle(self, konum):
        hedef = LocationGlobalRelative(
            konum['lat'], konum['lon'], self.cfg.DEVRIYE_IRTIFA
        )
        self.arac.simple_goto(hedef, airspeed=self.cfg.HAVA_HIZI)

    def _mesafe_kontrol(self, hedef, esik=3.0) -> bool:
        import math
        # Mock modunda doğrudan global_relative_frame'e eriş
        if self.cfg.SIMULASYON:
            mevcut = self.arac.location.global_relative_frame
        else:
            mevcut = self.arac.location.global_relative_frame

        dlat = (hedef['lat'] - mevcut.lat) * 111_320.0
        dlon = (hedef['lon'] - mevcut.lon) * 111_320.0 * \
               math.cos(math.radians(mevcut.lat))
        mesafe = math.sqrt(dlat ** 2 + dlon ** 2)

        if self.cfg.SIMULASYON and mesafe < esik * 2:  # Simülasyonda daha toleranslı
            # Waypoint'e ulaştıysa konumu tam hedefe ışınla (akışı hızlandır)
            self.arac.location.global_relative_frame.lat = hedef['lat']
            self.arac.location.global_relative_frame.lon = hedef['lon']
            return True
        return mesafe < esik

    # ─────────────────────────────────────────
    # ANA GÖREV AKIŞI
    # ─────────────────────────────────────────
    def gorevi_calistir(self):
        logger.info("╔══════════════════════════════════════╗")
        logger.info("║  DHO KEMALREİS — İHA GÖREV BAŞLIYOR ║")
        logger.info("╚══════════════════════════════════════╝")

        try:
            if not self.sistemi_baslat():
                logger.critical("Sistem başlatma başarısız. Görev iptal.")
                return

            if not self.preflight_kontrol():
                logger.critical("Pre-flight kontrol başarısız. Görev iptal.")
                return

            if not self.otonom_kalkis(self.cfg.DEVRIYE_IRTIFA):
                logger.critical("Kalkış başarısız!")
                return

            logger.info("Devriye moduna geçiliyor...")
            self.devriye_dongusu()

        except KeyboardInterrupt:
            logger.warning("Kullanıcı tarafından durduruldu.")
        except Exception as e:
            logger.exception(f"Beklenmeyen hata: {e}")
        finally:
            self.otonom_inis()
            if self.arac:
                self.arac.close()
            if self.komlink:
                self.komlink.baglantiyi_kes()
            logger.info("Görev sonlandırıldı. İyi uçuşlar.")


# ─────────────────────────────────────────────
# GİRİŞ NOKTASI
# ─────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DHO KEMALREİS İHA Görev Yazılımı")
    parser.add_argument("--fc-port",    default="/dev/ttyTHS1",  help="Uçuş kontrolcü seri portu")
    parser.add_argument("--sim",        action="store_true",      help="Simülasyon modu (SITL)")
    parser.add_argument("--gorev-id",   default="EFES2026_001",  help="Görev tanımlayıcısı")
    args = parser.parse_args()

    cfg = IHAConfig()
    if args.sim:
        cfg.FC_PORT    = "tcp:127.0.0.1:5762"
        cfg.SIMULASYON = True
        logger.info("*** SİMÜLASYON MODU AKTİF (SITL/Gazebo) ***")
    else:
        cfg.FC_PORT = args.fc_port

    cfg.GOREV_ID = args.gorev_id

    kontrolcu = IHAGorevKontrolcusu(cfg)
    kontrolcu.gorevi_calistir()
