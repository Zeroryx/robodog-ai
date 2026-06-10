from roboflow import Roboflow

def fetch_unified_dataset():
    # Inisialisasi koneksi dengan kunci API Anda
    rf = Roboflow(api_key="ZNzPBgOpWSDPHo2kDdIK")
    
    # Targetkan proyek gabungan terbaru Anda di cloud
    project = rf.workspace("mannuelmoses-gmail-com").project("unified_vision_system-fotth")
    
    # Unduh menggunakan format arsitektur YOLO standar agar tidak ditolak SDK
    version = project.version(1)
    dataset = version.download("yolov8")
    
    print("Pengunduhan dataset gabungan berhasil. Silakan ekstrak file data.yaml.")

if __name__ == '__main__':
    fetch_unified_dataset()