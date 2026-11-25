#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# The test uses the deployed vcan interface names only inside this new namespace.
exec unshare --net sh -ec '
    for i in 0 1; do
        ip link add "vcan$i" type vxcan peer name "o6peer$i"
        ip link set "vcan$i" up
        ip link set "o6peer$i" up
    done
    PYTHONPATH="$PWD/src" BELLO_HAND_VXCAN=1 python3 -m unittest discover -s tests -v
'
