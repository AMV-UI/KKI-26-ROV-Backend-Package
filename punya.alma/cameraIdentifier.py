
from pygrabber.dshow_graph import FilterGraph

# Mengambil daftar nama kamera sesuai urutan indeks OpenCV
devices = FilterGraph().get_input_devices()

print("--- DAFTAR INDEKS KAMERA OPENCV ---")
for index, name in enumerate(devices):
    print(f"Index {index} : {name}")