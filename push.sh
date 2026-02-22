#!/usr/bin/env bash
set -e

if [ $# -eq 0 ]; then
  echo "Usage: ./push.sh \"commit message\""
  exit 1
fi

COMMIT_MSG="$*"

echo "=== Staging all changes ==="
git add .
echo ""

echo "=== Committing with message: $COMMIT_MSG ==="
git commit -m "$COMMIT_MSG"
echo ""

echo "=== Pushing to remote ==="
git push
echo ""