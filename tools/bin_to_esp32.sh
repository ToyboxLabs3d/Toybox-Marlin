#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"/..

source ~/.platformio/penv/bin/activate

if [ -z "${ESP32_DIR:-}" ]; then
    echo "need to set ESP32_DIR environment variable to the path of the ESP32 firmware"
    exit 1
fi

if [ -z "${1:-}" ]; then
    echo "need to provide Marlin PIO environment as the first argument"
    exit 1
fi

MARLIN_PIO_ENV="$1"
echo "Marlin PIO environment: ${MARLIN_PIO_ENV}"


case "$MARLIN_PIO_ENV" in
    Alpha3)
        DEST_DIR="${ESP32_DIR}/data_common/alpha_3"
        ;;
    Alpha4)
        DEST_DIR="${ESP32_DIR}/app1_other_data/alpha_4"
        ;;
    *)
        echo "unknown env: $MARLIN_PIO_ENV"
        exit 1
        ;;
esac

pio run -e "${MARLIN_PIO_ENV}" 

MARLIN_BIN=".pio/build/${MARLIN_PIO_ENV}/firmware.bin"
MARLIN_BUILD=$(strings "${MARLIN_BIN}" | grep "Toybox-marlin BUILD:" | awk -F'[ ,]' '{print $3}')

echo "Marlin bin: ${MARLIN_BIN}"
echo "Marlin build: ${MARLIN_BUILD}"


echo "copying ${MARLIN_BIN} to ${DEST_DIR}"
mkdir -p "${DEST_DIR}"
cp "${MARLIN_BIN}" "${DEST_DIR}/marlin.bin"

echo "writing ${MARLIN_BUILD} to ${DEST_DIR}/marlin_ver"
printf "%s" "${MARLIN_BUILD}" > "${DEST_DIR}/marlin_ver"