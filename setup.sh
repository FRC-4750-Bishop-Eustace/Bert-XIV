#!/bin/bash

set -e

pip install --upgrade pip
pip install -r requirements.txt --upgrade

SKIP_SYNC=true
for arg in "$@"; do
    if [ "$arg" = "--sync" ]; then
        SKIP_SYNC=false
    fi
done
