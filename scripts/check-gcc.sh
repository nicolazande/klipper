#!/bin/sh
# This script checks for a broken Ubuntu 18.04 arm-none-eabi-gcc compile

f1="$1"
f2="$2"

readelf_cmd="${READELF:-readelf}"
if ! command -v "$readelf_cmd" >/dev/null 2>&1; then
    if command -v arm-none-eabi-readelf >/dev/null 2>&1; then
        readelf_cmd=arm-none-eabi-readelf
    else
        echo "ERROR: Unable to find readelf or arm-none-eabi-readelf"
        exit 1
    fi
fi

s1=$("$readelf_cmd" -A "$f1" | grep "Tag_ARM_ISA_use")
s2=$("$readelf_cmd" -A "$f2" | grep "Tag_ARM_ISA_use")

if [ "$s1" != "$s2" ]; then
    echo ""
    echo "ERROR: The compiler failed to correctly compile Klipper"
    echo "It will be necessary to upgrade the compiler"
    echo "See: https://bugs.launchpad.net/ubuntu/+source/newlib/+bug/1767223"
    echo ""
    rm -f "$f1"
    exit 99
fi
