#!/bin/bash
docker run --rm -it \
  -p 8889:8889 \
  -p 8554:8554 \
  -p 8189:8189/udp \
  -p 8189:8189/tcp \
  -v "$PWD/mediamtx.yml:/mediamtx.yml:ro" \
  bluenviron/mediamtx:1
