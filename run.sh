#!/bin/bash

set -e

SKIP_SYNC=true
for arg in "$@"; do
    if [ "$arg" = "--sync" ]; then
        SKIP_SYNC=false
    fi
done

if [ "$SKIP_SYNC" = false ]; then
    python3 -m robotpy sync
fi
python3 -m robotpy run
