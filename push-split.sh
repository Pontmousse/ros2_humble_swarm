#!/usr/bin/env bash
# Stage everything, commit, push to origin/split-containers. Refuses to run from any other branch,
# so split-containers work never lands on the wrong branch (push-hil.sh / push-split.sh).
set -e
BRANCH=split-containers

if [ $# -eq 0 ]; then echo "Usage: ./push-split.sh \"commit message\""; exit 1; fi
CURRENT=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT" != "$BRANCH" ]; then
  echo "Refusing: on branch '$CURRENT', this script pushes '$BRANCH'. Run 'git switch $BRANCH' first."; exit 1
fi

echo "=== Staging all changes ==="
git add .
echo ""

echo "=== Committing with message: $* ==="
git diff --cached --quiet && echo "(nothing to commit)" || git commit -m "$*"
echo ""

echo "=== Pushing to origin/$BRANCH ==="
git push -u origin "$BRANCH"
echo ""
