#!/usr/bin/env bash
# Download nuXmv 2.1.0 (Linux x86-64) from FBK and put the binary at ./nuXmv in the repo root.
# Adapted from the BehaVerify README. By running this you accept FBK's nuXmv licence (academic, non-commercial use
# only, no redistribution): https://nuxmv.fbk.eu/. The binary is git-ignored; never commit it or any part of its
# distribution.
#
#   ./install_nuxmv.sh          # does nothing if ./nuXmv already exists
set -euo pipefail
cd "$(dirname "$0")"

if [[ -x ./nuXmv ]]; then
    echo "nuXmv already installed: $(./nuXmv -h 2>&1 | head -1)"
    exit 0
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
wget -q --show-progress "https://nuxmv.fbk.eu/theme/download.php?file=nuXmv-2.1.0-linux64.tar.xz" -O "$tmp/nuXmv.tar.xz"
mkdir "$tmp/nuXmv_DL"
tar -xf "$tmp/nuXmv.tar.xz" -C "$tmp/nuXmv_DL" --strip-components 1
mv "$tmp/nuXmv_DL/bin/nuXmv" ./nuXmv
chmod +x ./nuXmv
echo "installed ./nuXmv"
