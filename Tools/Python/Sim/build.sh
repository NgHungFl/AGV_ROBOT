#!/bin/bash
# Vi du: ./build.sh build/sim/chapter5 [-DLINE_STATE_K2=0.10f]
exec "${PYTHON:-python3}" "$(dirname "$0")/build.py" "$@"
