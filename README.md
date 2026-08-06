# SETUP
## Windows
1. Jalan instalasi khusus python versi 3.10.8 https://www.python.org/ftp/python/3.10.8/python-3.10.8-amd64.exe
1. Buka command prompt lalu jalanin `pip install git+https://github.com/AMV-UI/KKI-26-ROV-Backend-Package` untuk instalasi backendnya
1. Instalasi docker (docker desktop yang dipake windows)
1. Unduh config mediamtx dari https://github.com/AMV-UI/KKI-26-ROV-Backend-Package/blob/main/mediamtx.yml
1. Buka command prompt baru lalu jalankan `docker run --rm -it --network=host -v "<PATH_TO_MTX_YAML>:/mediamtx.yml:ro" bluenviron/mediamtx:1`
1. Jalankan `python -m rov26backend.main` untuk menyalakan backend
1. Unduh repo dari https://github.com/AMV-UI/KKI-26-ROV-gcs (download as zip saja)
1. Pastikan sudah install node.js atau unduh dari https://nodejs.org/dist/v24.19.0/node-v24.19.0-x64.msi kalau belum
1. Buka command prompt di folder repo dan jalankan `npm run dev`
