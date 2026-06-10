#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"/..

source ./tools/lib/common.sh

if [ -z "${1:-}" ]; then
    err "Need to provide Marlin PIO environment as the first argument"
    exit 1
fi


case "$1" in
    Alpha4|Alpha3)
        ;;
    *)
        err "unknown env: $1"
        exit 1 ;;
esac


MARLIN_PIO_ENV="$1"
info "Marlin PIO environment: ${MARLIN_PIO_ENV}"


OUTPUT_DIR="./releases/${MARLIN_PIO_ENV}"
rm -rf "${OUTPUT_DIR}" 2>/dev/null || true
mkdir -p "${OUTPUT_DIR}"


if [ -n "$(git status --porcelain)" ]; then
    err "WORKING DIRECTORY IS DIRTY. COMMIT CHANGES BEFORE CREATING A RELEASE!!!!!!"
    exit 1
fi

LOCAL_TAGS=$(git tag --points-at HEAD)
REMOTE_TAGS=$(git ls-remote --tags --refs toybox | awk '{print $2}' | sed 's#refs/tags/##')
COMMON_TAGS=$(comm -12 <(echo "$LOCAL_TAGS" | sort) <(echo "$REMOTE_TAGS" | sort))

info "Local tags: ${LOCAL_TAGS}"
info "Local / Remote common tags: ${COMMON_TAGS}"

if [ -z "${COMMON_TAGS}" ]; then
    err "CURRENT COMMIT NOT TAGGED ON REMOTE. PUSH A TAG BEFORE CREATING A RELEASE!!!!!!"
    exit 1
fi

BUILD_TAG=$(echo "$COMMON_TAGS" | grep -E '^[0-9]+$' || true)
if [[ $(echo "$BUILD_TAG" | wc -l) -gt 1 ]]; then
    err "Multiple common tags that look like build tags found: $BUILD_TAG. Please ensure only one tag that looks like a build tag (all numbers) is on the current commit."
    # exit 1

fi
if [ -z "$BUILD_TAG" ]; then
    err "No common tags that look like build tags (all numbers) found. Please ensure a tag that looks like a build tag is on the current commit."
    exit 1
fi



GIT_COMMIT_HASH="$(git rev-parse --short HEAD)"

BIN_FILE="./.pio/build/${MARLIN_PIO_ENV}/firmware.bin"
HASH_FILE="./.pio/build/${MARLIN_PIO_ENV}/hash.txt"
OUTPUT_BIN_FILE="HC32-Marlin-${MARLIN_PIO_ENV}_${BUILD_TAG}-${GIT_COMMIT_HASH}-$(date +%Y.%m.%d).bin"
HASH_OUTPUT_FILE="HC32-Marlin-${MARLIN_PIO_ENV}_${BUILD_TAG}-${GIT_COMMIT_HASH}-$(date +%Y.%m.%d).sha256"


step "pio fullclean"
pio run -e "${MARLIN_PIO_ENV}" -t fullclean
step "pio build"
pio run -e "${MARLIN_PIO_ENV}"



BIN_BUILD_NUMBER=$(strings "${BIN_FILE}" | grep "Toybox-marlin BUILD:" | awk -F'[ ,]' '{print $3}')
info "Marlin build: ${BIN_BUILD_NUMBER}, build from tag: ${BUILD_TAG}" 1>&2
if [ -z "$BIN_BUILD_NUMBER" ]; then
    err "Could not find build string in ${BIN_FILE}. Did you change src/gcode/toybox/M10002.cpp? Is the path correct and is the file a valid Marlin binary?"
    exit 1
fi
if [ "$BIN_BUILD_NUMBER" != "$BUILD_TAG" ]; then
    err "MARLIN BUILD MISMATCH: bin=${BIN_BUILD_NUMBER} from tag=${BUILD_TAG} !!!!!!"
    exit 1
fi



step "Copying ${BIN_FILE} to ${OUTPUT_DIR}/${OUTPUT_BIN_FILE}"
cp "${BIN_FILE}" "${OUTPUT_DIR}/${OUTPUT_BIN_FILE}"

step "Copying ${HASH_FILE} to ${OUTPUT_DIR}/${HASH_OUTPUT_FILE}"
cp "${HASH_FILE}" "${OUTPUT_DIR}/${HASH_OUTPUT_FILE}"


success "Release artifacts written to ${OUTPUT_DIR}"

case "$(uname -s)" in
    Darwin) open "${OUTPUT_DIR}" ;;
    Linux)  xdg-open "${OUTPUT_DIR}" ;;
    *) ;;
esac