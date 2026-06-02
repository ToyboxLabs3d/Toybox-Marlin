#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"/..

source ./tools/lib/common.sh

if [ -z "${ESP32_DIR:-}" ]; then
    err "need to set ESP32_DIR environment variable to the path of the ESP32 firmware"
    exit 1
fi

if [ -z "${1:-}" ]; then
    err "need to provide Marlin PIO environment as the first argument"
    exit 1
fi

MARLIN_PIO_ENV="$1"
info "Marlin PIO environment: ${MARLIN_PIO_ENV}"


case "$MARLIN_PIO_ENV" in
    Alpha3)
        DEST_DIR="${ESP32_DIR}/data_common/alpha_3"
        ;;
    Alpha4)
        DEST_DIR="${ESP32_DIR}/app1_other_data/alpha_4"
        ;;
    *)
        err "unknown env: $MARLIN_PIO_ENV"
        exit 1
        ;;
esac

if [ "${2:-}" = "--risky-mode" ]; then
    warn "RISKY MODE ENABLED: Not doing full rebuild. THIS IS INAPPROPRIATE FOR PRODUCTION ESP32 BUILDS."
else
    step "pio fullclean"
    pio run -e "${MARLIN_PIO_ENV}" -t fullclean
fi

step "pio build"
pio run -e "${MARLIN_PIO_ENV}" 

MARLIN_BIN=".pio/build/${MARLIN_PIO_ENV}/firmware.bin"
MARLIN_BUILD=$(strings "${MARLIN_BIN}" | grep "Toybox-marlin BUILD:" | awk -F'[ ,]' '{print $3}')

info "Marlin bin: ${MARLIN_BIN}"
info "Marlin build: ${MARLIN_BUILD}"


step "copying ${MARLIN_BIN} to ${DEST_DIR}"
mkdir -p "${DEST_DIR}"
cp "${MARLIN_BIN}" "${DEST_DIR}/marlin.bin"

step "writing ${MARLIN_BUILD} to ${DEST_DIR}/marlin_ver"
printf "%s" "${MARLIN_BUILD}" > "${DEST_DIR}/marlin_ver"

success "Marlin firmware for ${MARLIN_PIO_ENV} written to ${DEST_DIR}"