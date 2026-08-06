#!/bin/sh
python -m grpc_tools.protoc -I./src/protos --python_out=./src/generated --pyi_out=./src/generated --grpc_python_out=./src/generated ./src/protos/server.proto
