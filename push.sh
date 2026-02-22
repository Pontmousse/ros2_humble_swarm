#!/usr/bin/env bash
set -e

if [ $# -eq 0 ]; then
  echo "Usage: ./push.sh \"commit message\""
  exit 1
fi

COMMIT_MSG="$*"

git add .
git commit -m "$COMMIT_MSG"
git push
