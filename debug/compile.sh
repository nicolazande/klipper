#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <source_file.c>"
    exit 1
fi

SRC_FILE="$1"
SRC_DIR="$(dirname "$(readlink -f "$0")")"
OUT_DIR="./out"
# create object directory
if [ ! -d "$OUT_DIR" ]; then
    mkdir -p "$OUT_DIR"
fi

# include paths from the original script
INCLUDE_PATHS=(
    "-I$SRC_DIR"
)

# compile the specified source file
BASENAME="$(basename "$SRC_FILE")"
OBJ_FILE="${BASENAME%.c}.o"

gcc -Wall -g -O2 -c -fPIC "${INCLUDE_PATHS[@]}" "$SRC_FILE" -o "$OUT_DIR/$OBJ_FILE"

if [ $? -eq 0 ]; then
    echo "Compilation successful. Object file: $OBJ_FILE"
else
    echo "Error compiling $SRC_FILE"
    exit 1
fi



