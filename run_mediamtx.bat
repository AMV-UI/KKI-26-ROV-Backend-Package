@echo off
docker run --rm -it --network=host -v "%cd%\mediamtx.yml:/mediamtx.yml:ro" bluenviron/mediamtx:1
