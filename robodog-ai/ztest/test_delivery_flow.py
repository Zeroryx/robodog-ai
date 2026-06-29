# ztest/test_delivery_flow.py
"""
Simulasi pengujian untuk modul Delivery.
Menguji PackageValidator, QRSystem, dan DeliveryManager.
"""

import os
from delivery import PackageValidator, QRSystem, DeliveryManager

def test_package_validation_success():
    """Simulasi paket yang memenuhi syarat (di bawah 15cm & 2kg)."""
    is_valid, errors = PackageValidator.validate(length=10.0, width=10.0, height=5.0, weight=1.2)
    
    assert is_valid == True
    assert len(errors) == 0

def test_package_validation_failed():
    """Simulasi paket yang ukurannya atau beratnya berlebih."""
    is_valid, errors = PackageValidator.validate(length=20.0, width=10.0, height=5.0, weight=2.5)
    
    assert is_valid == False
    assert len(errors) > 0  # Harus memunculkan pesan error

def test_qr_generation(tmp_path):
    """Simulasi membuat gambar QR Code untuk pengetesan sistem."""
    test_qr_path = os.path.join(tmp_path, "sim_qr.png")
    
    # Generate QR dengan data ID unik
    success = QRSystem.generate_qr(data="ORDER_9999", output_path=test_qr_path)
    
    assert success == True
    assert os.path.exists(test_qr_path)

def test_delivery_workflow_fsm():
    """Simulasi Finite State Machine (FSM) pada DeliveryManager."""
    manager = DeliveryManager()
    
    # 1. Inisiasi order dengan paket valid
    success = manager.create_delivery_order(
        delivery_id="D001", 
        sender_code="SEND_123", 
        receiver_code="REC_456",
        length=10.0, 
        width=10.0, 
        height=10.0, 
        weight=1.0
    )
    
    assert success == True
    assert manager.status == "READY_TO_DEPART"
    
    # 2. Simulasi transit dan sampai tujuan
    manager.start_transit()
    assert manager.status == "EN_ROUTE"
    
    manager.arrive_at_destination()
    assert manager.status == "ARRIVED"
    
    # 3. Simulasi scan QR penerima yang sesuai
    qr_verified = manager.process_qr_verification(scanned_text="REC_456")
    
    assert qr_verified == True
    assert manager.status == "COMPLETED"