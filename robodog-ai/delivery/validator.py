# delivery/validator.py
"""
Modul PackageValidator untuk memeriksa kelayakan paket (dimensi dan berat).
Sesuai dengan panduan arsitektur, batasan diambil dari config/settings.py.
"""

# Mencoba mengimpor konstanta dari file pengaturan pusat
try:
    from config.settings import (
        MAX_PACKAGE_LENGTH_CM,
        MAX_PACKAGE_WIDTH_CM,
        MAX_PACKAGE_HEIGHT_CM,
        MAX_PACKAGE_WEIGHT_KG
    )
except ImportError:
    # Fallback sementara jika Team Lead belum selesai menyusun config/settings.py
    MAX_PACKAGE_LENGTH_CM = 15.0
    MAX_PACKAGE_WIDTH_CM = 15.0
    MAX_PACKAGE_HEIGHT_CM = 15.0
    MAX_PACKAGE_WEIGHT_KG = 2.0


class PackageValidator:
    """
    Kelas yang memvalidasi ukuran fisik dan berat paket yang akan dikirim
    oleh robot agar tidak melebihi kapasitas bawaan.
    """

    @staticmethod
    def validate(length: float, width: float, height: float, weight: float) -> tuple[bool, list[str]]:
        """
        Memeriksa apakah paket memenuhi syarat pengiriman.

        Args:
            length (float): Panjang paket dalam cm.
            width (float): Lebar paket dalam cm.
            height (float): Tinggi paket dalam cm.
            weight (float): Berat paket dalam kg.

        Returns:
            tuple[bool, list[str]]: (Status kelayakan, Daftar pesan error jika ada)
        """
        errors = []

        # Validasi Panjang
        if length > MAX_PACKAGE_LENGTH_CM:
            errors.append(f"Panjang paket ({length} cm) melebihi batas maksimal ({MAX_PACKAGE_LENGTH_CM} cm).")
        
        # Validasi Lebar
        if width > MAX_PACKAGE_WIDTH_CM:
            errors.append(f"Lebar paket ({width} cm) melebihi batas maksimal ({MAX_PACKAGE_WIDTH_CM} cm).")
            
        # Validasi Tinggi
        if height > MAX_PACKAGE_HEIGHT_CM:
            errors.append(f"Tinggi paket ({height} cm) melebihi batas maksimal ({MAX_PACKAGE_HEIGHT_CM} cm).")
            
        # Validasi Berat
        if weight > MAX_PACKAGE_WEIGHT_KG:
            errors.append(f"Berat paket ({weight} kg) melebihi batas maksimal ({MAX_PACKAGE_WEIGHT_KG} kg).")
            
        # Jika array errors kosong, berarti paket valid (True)
        is_valid = len(errors) == 0
        
        return is_valid, errors