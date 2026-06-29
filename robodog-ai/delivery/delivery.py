# delivery/delivery.py
"""
Modul DeliveryManager untuk mengatur status dan siklus hidup pengiriman barang.
Menghubungkan validasi paket (PackageValidator) dan sistem QR (QRSystem) 
ke dalam workflow robot.
"""

from .validator import PackageValidator
from .qr import QRSystem


class DeliveryManager:
    """
    Mengelola seluruh status pengiriman paket, mulai dari inisiasi,
    validasi fisik, perjalanan, hingga verifikasi penerima di tujuan.
    """

    def __init__(self):
        # Status awal pengiriman
        self.delivery_id = None
        self.status = "IDLE"  # Pilihan status: IDLE, VALIDATING, EN_ROUTE, ARRIVED, VERIFYING, COMPLETED, FAILED
        self.sender_passcode = None
        self.receiver_passcode = None
        self.package_details = {}
        self.error_message = ""

    def create_delivery_order(self, delivery_id: str, sender_code: str, receiver_code: str, 
                              length: float, width: float, height: float, weight: float) -> bool:
        """
        Inisiasi order pengiriman baru dan langsung melakukan validasi paket.
        """
        self.delivery_id = delivery_id
        self.sender_passcode = sender_code
        self.receiver_passcode = receiver_code
        self.package_details = {
            "length": length,
            "width": width,
            "height": height,
            "weight": weight
        }
        
        self.status = "VALIDATING"
        print(f"[Delivery {self.delivery_id}] Memulai validasi paket...")
        
        # Memanggil validator yang sudah kita buat sebelumnya
        is_valid, errors = PackageValidator.validate(length, width, height, weight)
        
        if not is_valid:
            self.status = "FAILED"
            self.error_message = " | ".join(errors)
            print(f"[Delivery {self.delivery_id}] Validasi GAGAL: {self.error_message}")
            return False
            
        print(f"[Delivery {self.delivery_id}] Validasi BERHASIL. Siap berangkat.")
        self.status = "READY_TO_DEPART"
        return True

    def start_transit(self) -> None:
        """Mengubah status menjadi sedang dalam perjalanan (En Route)."""
        if self.status == "READY_TO_DEPART":
            self.status = "EN_ROUTE"
            print(f"[Delivery {self.delivery_id}] Robot sedang berjalan menuju lokasi penerima.")

    def arrive_at_destination(self) -> None:
        """Dipanggil ketika modul Navigation mendeteksi robot sudah sampai di titik tujuan."""
        if self.status == "EN_ROUTE":
            self.status = "ARRIVED"
            print(f"[Delivery {self.delivery_id}] Robot telah sampai di lokasi. Menunggu verifikasi QR.")

    def process_qr_verification(self, scanned_text: str) -> bool:
        """
        Memproses text hasil scan QR dari kamera untuk mencocokkan passcode penerima.
        """
        if self.status != "ARRIVED" and self.status != "VERIFYING":
            print(f"[Delivery {self.delivery_id}] Peringatan: Robot tidak dalam posisi siap serah terima.")
            return False
            
        self.status = "VERIFYING"
        
        # Memanggil verifikator QR yang sudah kita buat sebelumnya
        is_authenticated = QRSystem.verify_passcode(scanned_text, self.receiver_passcode)
        
        if is_authenticated:
            self.status = "COMPLETED"
            print(f"[Delivery {self.delivery_id}] Verifikasi sukses! Paket berhasil diserahkan.")
            return True
        else:
            print(f"[Delivery {self.delivery_id}] QR Code salah/tidak cocok. Akses ditolak.")
            return False

    def reset_manager(self) -> None:
        """Mengembalikan status ke IDLE agar siap menerima order pengiriman berikutnya."""
        self.__init__()