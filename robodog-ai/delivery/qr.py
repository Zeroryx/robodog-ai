# delivery/qr.py
"""
Modul QRSystem untuk pengelolaan pembuatan dan pemindaian QR Code.
Menggunakan pustaka 'qrcode' untuk enkripsi data ke gambar,
dan 'pyzbar' + 'cv2' (OpenCV) untuk dekripsi/pemindaian lewat kamera.
"""


import os
import qrcode
import cv2
import numpy as np
from pyzbar.pyzbar import decode


class QRSystem:
    """
    Sistem otentikasi berbasis QR Code untuk memvalidasi identitas pengirim
    dan penerima barang pada AI Robotic Dog.
    """

    @staticmethod
    def generate_qr(data: str, output_path: str) -> bool:
        """
        Membuat file gambar QR Code berdasarkan teks/ID yang diberikan.

        Args:
            data (str): Data teks unik (misal: ID Pengiriman atau Passcode).
            output_path (str): Jalur folder dan nama file untuk menyimpan gambar (contoh: 'assets/qr_code.png').

        Returns:
            bool: True jika berhasil dibuat dan disimpan, False jika gagal.
        """
        try:
            # Memastikan folder tujuan sudah dibuat sebelum menyimpan file
            directory = os.path.dirname(output_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)

            # Konfigurasi standar QR Code
            qr = qrcode.QRCode(
                version=1,
                error_correction=qrcode.constants.ERROR_CORRECT_L,
                box_size=10,
                border=4,
            )
            qr.add_data(data)
            qr.make(fit=True)

            # Render matriks QR ke dalam format gambar Pillow (PIL)
            img = qr.make_image(fill_color="black", back_color="white")
            img.save(output_path)
            return True
            
        except Exception as e:
            print(f"[QR Error] Gagal membuat QR Code: {e}")
            return False

    @staticmethod
    def scan_frame(frame: np.ndarray) -> list[str]:
        """
        Memindai dan membaca teks dari QR Code langsung melalui frame kamera (OpenCV).
        Fungsi ini akan dipanggil berulang kali di dalam loop kamera utama robot.

        Args:
            frame (np.ndarray): Gambar matriks (BGR) yang ditangkap oleh kamera robot.

        Returns:
            list[str]: Daftar string teks yang berhasil didekode dari semua QR yang terlihat di frame.
        """
        if frame is None:
            return []

        try:
            # pyzbar melakukan scanning otomatis pada frame gambar
            decoded_objects = decode(frame)
            results = []
            
            for obj in decoded_objects:
                # Data dari pyzbar bertipe bytes, kita ubah (decode) ke string utf-8
                qr_text = obj.data.decode("utf-8")
                results.append(qr_text)
                
            return results
            
        except Exception as e:
            print(f"[QR Error] Terjadi kegagalan saat memindai frame: {e}")
            return []

    @staticmethod
    def verify_passcode(scanned_data: str, expected_passcode: str) -> bool:
        """
        Membandingkan teks hasil scan QR dengan kode rahasia pengiriman yang valid.

        Args:
            scanned_data (str): Teks yang terbaca dari kamera robot.
            expected_passcode (str): Teks kode rahasia yang terdaftar di sistem pusat.

        Returns:
            bool: True jika kode cocok (akses diterima), False jika salah.
        """
        if not scanned_data or not expected_passcode:
            return False
            
        # .strip() digunakan untuk menghapus spasi atau enter yang tidak sengaja terbawa
        return scanned_data.strip() == expected_passcode.strip()