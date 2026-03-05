#!/usr/bin/env sh
set -eu

echo "[1/6] Pull main repo..."
git pull --rebase

echo "[2/6] Sync submodule URLs..."
git submodule sync --recursive

echo "[3/6] Init/update submodules..."
git submodule update --init --recursive

echo "[4/6] Pull latest commits in each submodule..."
git submodule foreach --recursive git pull origin HEAD

echo "[5/6] Submodule status:"
git submodule status

if [ "${1:-}" = "--remote" ]; then
  echo "[6/6] Updating submodules to latest remote branch..."
  git submodule update --remote --merge --recursive
fi

echo "Done."