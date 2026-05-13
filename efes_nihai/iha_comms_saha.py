#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DHO KEMALREİS — İDA Haberleşme Modülü
MAVLink v2 over RFD900x 915 MHz | UDP Simülasyon Desteği
EFES-2026 | Milli Savunma Üniversitesi
"""

import json
import time
import struct
import socket
import logging
import threading
from typing import Optional, Dict

logger = logging.getLogger("KOMLINK")

# MAVLink mesaj ID (özel: NAMED_VALUE_FLOAT 251 yerine USER_DEFINED 229)
MAVLINK_MSG_HEDEF_KOORDINAT = 229   # DHO özel mesaj tipi
MAVLINK_SYSTEM_ID_IHA = 1
MAVLINK_COMP_ID_ONBOARD = 191


class IDAHaberlesme:
    """
    İHA → İDA koordinat ve görev verisi iletim modülü.

    Birincil kanal  : RFD900x @ 915 MHz (seri / UART)
    İkincil kanal   : UDP (simülasyon / yedek)
    Protokol        : MAVLink v2 (paket yapısı el ile inşa edilir)
    Onay mekanizması: ACK paketi 5 sn içinde gelmezse yeniden dene (max 3)
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._seri   = None
        self._udp    = None
        self._kanal  = None   # "seri" veya "udp"
        self._lock   = threading.Lock()
        self._gondeilen_sayisi   = 0
        self._basarisiz_sayisi   = 0
        self._son_ack_zaman      = 0.0
        self._ack_bekle_event    = threading.Event()

    # ─────────────────────────────────────────
    # BAĞLANTI
    # ─────────────────────────────────────────
    def baglan(self) -> bool:
        # 1) Seri (RFD900x) önce dene
        if self._seri_baglan():
            self._kanal = "seri"
            return True

        # 2) Seri başarısız → UDP'ye geç
        logger.warning("RFD900x erişilemez, UDP moduna geçiliyor...")
        if self._udp_baglan():
            self._kanal = "udp"
            return True

        logger.error("İDA haberleşme kanalı kurulamadı!")
        return False

    def _seri_baglan(self) -> bool:
        try:
            import serial
            self._seri = serial.Serial(
                self.cfg.IDA_COMM_PORT,
                baudrate=self.cfg.IDA_COMM_BAUD,
                timeout=self.cfg.IDA_TIMEOUT_S
            )
            time.sleep(0.5)
            logger.info(
                f"RFD900x seri bağlantısı: {self.cfg.IDA_COMM_PORT} "
                f"@ {self.cfg.IDA_COMM_BAUD} baud ✓"
            )
            return True
        except Exception as e:
            logger.debug(f"Seri bağlantı hatası: {e}")
            return False

    def _udp_baglan(self) -> bool:
        try:
            self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp.settimeout(self.cfg.IDA_TIMEOUT_S)
            logger.info(
                f"UDP haberleşme: {self.cfg.IDA_SIM_IP}:"
                f"{self.cfg.IDA_SIM_PORT} ✓"
            )
            return True
        except Exception as e:
            logger.error(f"UDP bağlantı hatası: {e}")
            return False

    # ─────────────────────────────────────────
    # KOORDİNAT GÖNDER
    # ─────────────────────────────────────────
    def koordinat_gonder(self, paket: Dict, deneme_sayisi: int = 3) -> bool:
        """
        Hedef koordinatını MAVLink paketi olarak İDA'ya iletir.
        ACK alınamazsa deneme_sayisi kadar yeniden dener.
        """
        mavlink_paketi = self._paket_olustur(paket)

        for deneme in range(1, deneme_sayisi + 1):
            try:
                with self._lock:
                    if self._kanal == "seri" and self._seri:
                        self._seri.write(mavlink_paketi)
                        self._seri.flush()
                    elif self._kanal == "udp" and self._udp:
                        self._udp.sendto(
                            mavlink_paketi,
                            (self.cfg.IDA_SIM_IP, self.cfg.IDA_SIM_PORT)
                        )
                    else:
                        raise ConnectionError("Aktif kanal yok")

                logger.debug(
                    f"[KOMLINK] Paket gönderildi (deneme {deneme}/{deneme_sayisi}) | "
                    f"{len(mavlink_paketi)} bayt"
                )

                # ACK bekle
                if self._ack_bekle(timeout=self.cfg.IDA_TIMEOUT_S):
                    self._gondeilen_sayisi += 1
                    logger.info(
                        f"[KOMLINK] ACK alındı ✓ | Toplam gönderim: "
                        f"{self._gondeilen_sayisi}"
                    )
                    return True

                logger.warning(f"[KOMLINK] ACK alınamadı (deneme {deneme})")
                time.sleep(0.5)

            except Exception as e:
                logger.error(f"[KOMLINK] Gönderim hatası: {e}")
                time.sleep(1.0)

        self._basarisiz_sayisi += 1
        logger.error(
            f"[KOMLINK] ✗ {deneme_sayisi} deneme sonrası iletim başarısız! | "
            f"Toplam başarısız: {self._basarisiz_sayisi}"
        )
        return False

    # ─────────────────────────────────────────
    # MAVLink PAKET OLUŞTUR (özel mesaj #229)
    # ─────────────────────────────────────────
    def _paket_olustur(self, veri: Dict) -> bytes:
        """
        MAVLink v2 çerçevesi:
        [STX][LEN][INCOMPAT][COMPAT][SEQ][SYS_ID][COMP_ID]
        [MSG_ID (3B)][PAYLOAD][CHECKSUM (2B)]
        """
        payload = struct.pack(
            "<BddfBHH",                        # MAVLink v2: little-endian
            MAVLINK_MSG_HEDEF_KOORDINAT,       # mesaj tipi (1B)
            veri.get('lat', 0.0),              # enlem  (8B double)
            veri.get('lon', 0.0),              # boylam (8B double)
            veri.get('alt', 0.0),              # irtifa (4B float)
            self._sinif_id(veri['hedef_sinifi']),  # sınıf ID (1B)
            int(veri.get('confidence', 0) * 10000),  # güven *10000 (2B)
            int(veri.get('accuracy_m', 0) * 100),    # doğruluk cm (2B)
        )

        seq = self._gondeilen_sayisi & 0xFF
        header = bytes([
            0xFD,                       # MAVLink v2 STX
            len(payload),               # payload uzunluğu
            0x00,                       # incompat flags
            0x00,                       # compat flags
            seq,                        # paket sırası
            MAVLINK_SYSTEM_ID_IHA,      # kaynak sistem ID
            MAVLINK_COMP_ID_ONBOARD,    # kaynak bileşen ID
            MAVLINK_MSG_HEDEF_KOORDINAT & 0xFF,
            (MAVLINK_MSG_HEDEF_KOORDINAT >> 8) & 0xFF,
            (MAVLINK_MSG_HEDEF_KOORDINAT >> 16) & 0xFF,
        ])

        ham = header + payload
        crc = self._crc16_mcrf4xx(ham[1:])
        return ham + struct.pack("<H", crc)

    @staticmethod
    def _crc16_mcrf4xx(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def _sinif_id(sinif_adi: str) -> int:
        tablo = {
            "kacakcilik_botu"     : 0,
            "gocmen_botu"         : 1,
            "dusman_iha"          : 2,
            "dusman_ida"          : 3,
            "deniz_kazazedesi"    : 4,
            "sahil_guvenligi_botu": 5,
        }
        return tablo.get(sinif_adi, 99)

    # ─────────────────────────────────────────
    # ACK BEKLEME (basit simülasyon)
    # ─────────────────────────────────────────
    def _ack_bekle(self, timeout: float = 5.0) -> bool:
        """
        Gerçek sistemde İDA'dan gelen MAVLink ACK/COMMAND_ACK
        mesajını okur. Simülasyonda %90 başarı simüle edilir.
        """
        if self._kanal == "udp":
            import random
            time.sleep(0.15)      # ~150ms RF gecikmesi simülasyonu
            return random.random() < 0.92

        if self._seri:
            bitis = time.time() + timeout
            while time.time() < bitis:
                bekleyen = self._seri.in_waiting
                if bekleyen >= 4:
                    yanit = self._seri.read(4)
                    if yanit[0] == 0xFD:     # MAVLink v2 STX
                        return True
                time.sleep(0.05)

        return False

    # ─────────────────────────────────────────
    # DURUM / TEMİZLİK
    # ─────────────────────────────────────────
    def durum_raporu(self) -> Dict:
        return {
            "kanal"          : self._kanal,
            "gonderilen"     : self._gondeilen_sayisi,
            "basarisiz"      : self._basarisiz_sayisi,
            "basari_orani_%": round(
                100 * self._gondeilen_sayisi /
                max(1, self._gondeilen_sayisi + self._basarisiz_sayisi),
                1
            )
        }

    def baglantiyi_kes(self):
        if self._seri and self._seri.is_open:
            self._seri.close()
        if self._udp:
            self._udp.close()
        rapor = self.durum_raporu()
        logger.info(
            f"Haberleşme kapatıldı. Rapor: "
            f"{rapor['gonderilen']} gönderim, "
            f"%{rapor['basari_orani_%']} başarı"
        )
