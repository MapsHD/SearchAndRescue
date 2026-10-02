#!/usr/bin/env bash

set -e

cd "$(dirname "$0")"

echo "=== Git update ==="

git pull --ff-only

echo "=== Sync submodules ==="

git submodule sync --recursive
git submodule update --init --recursive

echo "=== Configure CMake ==="

cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

echo "=== Build ==="

cmake --build build --config Release -j

echo "=== Done ==="
