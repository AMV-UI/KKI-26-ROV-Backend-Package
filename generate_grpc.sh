#!/bin/sh
python -m grpc_tools.protoc -I./src/rov26backend/protos --python_out=./src/rov26backend/generated --pyi_out=./src/rov26backend/generated --grpc_python_out=./src/rov26backend/generated ./src/rov26backend/protos/server.proto
