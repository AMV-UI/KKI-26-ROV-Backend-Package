@echo off
docker run --rm -it ^
  -p 8554:8554 -p 1935:1935 -p 8888:8888 -p 8889:8889 ^
  -p 8000:8000/udp -p 8001:8001/udp -p 8189:8189/udp ^
  -p 9997:9997 -p 9996:9996 ^
  -v "%cd%\mediamtx.yml:/mediamtx.yml:ro" ^
  bluenviron/mediamtx:1
