#!/usr/bin/env bash
set -eu

echo "[1/6] Pull main repo..."
git pull --rebase

echo "[2/6] Sync submodule URLs..."
git submodule sync --recursive

echo "[3/6] Init/update submodules..."
git submodule update --init --recursive

if [[ "${1:-}" == "--remote" ]]; then
  echo "[4/6] Updating submodules to latest remote branch..."
  git submodule update --remote --merge --recursive
  echo "[5/6] Submodule status:"
  git status --porcelain
fi

echo "Done."