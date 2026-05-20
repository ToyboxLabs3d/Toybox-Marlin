#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"/..

source ~/.platformio/penv/bin/activate

if [ -z "${1:-}" ]; then
    echo "need to provide Marlin PIO environment as the first argument"
    exit 1
fi

MARLIN_PIO_ENV="$1"
echo "Marlin PIO environment: ${MARLIN_PIO_ENV}"

case "$MARLIN_PIO_ENV" in
    Alpha3)
        ;;
    Alpha4)
        ;;
    *)
        echo "unknown env: $MARLIN_PIO_ENV"
        exit 1
        ;;
esac

pio run -e "${MARLIN_PIO_ENV}"

pushd .pio/build/${MARLIN_PIO_ENV}/
python3 -m http.server 8080
popd