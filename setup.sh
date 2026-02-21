#!/bin/bash

set -e

pip install --upgrade pip
pip install robotpy --upgrade

SKIP_SYNC=true
for arg in "$@"; do
    if [ "$arg" = "--sync" ]; then
        SKIP_SYNC=false
    fi
done
