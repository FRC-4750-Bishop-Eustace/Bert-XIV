#!/bin/bash

set -e

SKIP_SYNC=false
for arg in "$@"; do
    if [ "$arg" = "--skip-sync" ]; then
        SKIP_SYNC=true
    fi
done

if [ "$SKIP_SYNC" = false ]; then
    python3 -m robotpy sync
fi
python3 -m robotpy sim
