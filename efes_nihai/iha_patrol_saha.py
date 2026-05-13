#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DHO KEMALREİS — Devriye Rota Hesaplayıcı
Paralel Hat Tarama (Lawnmower Pattern) | EFES-2026
"""

import math
import logging
from typing import List, Dict

logger = logging.getLogger("PATROL")


class DevriyeRotalayici:
    """
    İHA'nın görev bölgesini sistematik olarak taraması için
    paralel hat (lawnmower / boustrophedon) devriye rotası üretir.

    Koordinatlar WGS-84 formatında üretilir.
    Hat aralığı, kamera FOV ve devriye irtifasına göre otomatik
    örtüşme payı (%20) hesaplanarak ayarlanır.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._metre_per_lat = 111_320.0
        self._metre_per_lon = 111_320.0 * math.cos(
            math.radians(cfg.DEVRIYE_MERKEZ['lat'])
        )

    # ─────────────────────────────────────────
    # ANA FONKSİYON
    # ─────────────────────────────────────────
    def rota_hesapla(self) -> List[Dict]:
        """
        Görev bölgesi için lawnmower devriye waypoint listesi üretir.

        Kamera yatay görüş alanı (yer iz genişliği):
            swath = 2 × irtifa × tan(HFOV/2)

        Hat aralığı = swath × (1 − overlap)
        """
        irtifa  = self.cfg.DEVRIYE_IRTIFA
        en      = self.cfg.DEVRIYE_EN
        boy     = self.cfg.DEVRIYE_BOY
        merkez  = self.cfg.DEVRIYE_MERKEZ

        # Kamera yayılım genişliği (metre)
        swath   = 2.0 * irtifa * self.cfg.FOV_TAN_H
        overlap = 0.20     # %20 örtüşme
        hat_araliği = swath * (1.0 - overlap)

        if self.cfg.DEVRIYE_HATTI > 0:
            hat_araliği = self.cfg.DEVRIYE_HATTI

        logger.info(f"Devriye bölgesi: {en}m × {boy}m")
        logger.info(f"Kamera swath   : {swath:.1f}m  Hat aralığı: {hat_araliği:.1f}m")

        # Başlangıç noktası (sol-alt köşe)
        baslangic_lat = merkez['lat'] - self._m2lat(boy / 2)
        baslangic_lon = merkez['lon'] - self._m2lon(en  / 2)

        waypoints = []
        hat_sayisi = int(math.ceil(en / hat_araliği)) + 1
        soldan_saga = True

        for i in range(hat_sayisi):
            x_m = i * hat_araliği
            if x_m > en:
                x_m = en

            lon_wp = baslangic_lon + self._m2lon(x_m)

            if soldan_saga:
                lat_a = baslangic_lat
                lat_b = baslangic_lat + self._m2lat(boy)
            else:
                lat_a = baslangic_lat + self._m2lat(boy)
                lat_b = baslangic_lat

            waypoints.append({"lat": lat_a, "lon": lon_wp})
            waypoints.append({"lat": lat_b, "lon": lon_wp})

            soldan_saga = not soldan_saga

        logger.info(f"Üretilen waypoint: {len(waypoints)} adet ({hat_sayisi} hat)")
        self._rota_logla(waypoints)
        return waypoints

    # ─────────────────────────────────────────
    # YARDIMCI
    # ─────────────────────────────────────────
    def _m2lat(self, metre: float) -> float:
        return metre / self._metre_per_lat

    def _m2lon(self, metre: float) -> float:
        return metre / self._metre_per_lon

    def _rota_logla(self, waypoints: List[Dict]):
        logger.debug("── Devriye Rotası ──")
        for i, wp in enumerate(waypoints):
            logger.debug(f"  WP-{i+1:02d}: {wp['lat']:.6f}, {wp['lon']:.6f}")

    def gorsel_yazdir(self) -> str:
        """ASCII harita çıktısı (görev brifing / debug)"""
        wps = self.rota_hesapla()
        lats = [w['lat'] for w in wps]
        lons = [w['lon'] for w in wps]

        genislik, yukseklik = 60, 20
        satirlar = [['·'] * genislik for _ in range(yukseklik)]

        for i, wp in enumerate(wps):
            x = int((wp['lon'] - min(lons)) / (max(lons) - min(lons) + 1e-9)
                    * (genislik - 1))
            y = int((1 - (wp['lat'] - min(lats)) / (max(lats) - min(lats) + 1e-9))
                    * (yukseklik - 1))
            satirlar[y][x] = str(i % 10)

        cikti = ["╔" + "═" * genislik + "╗"]
        cikti += ["║" + "".join(s) + "║" for s in satirlar]
        cikti += ["╚" + "═" * genislik + "╝"]
        return "\n".join(cikti)
