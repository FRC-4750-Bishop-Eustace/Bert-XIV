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
python3 -m robotpy create-physics
python3 -m robotpy test --builtin

rm physics.py
