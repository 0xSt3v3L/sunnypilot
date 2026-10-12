#!/usr/bin/env bash
# Copy a route's rlogs from the comma four for drive_report.py and mads_replay.py.
# usage: pull_route.sh --list | pull_route.sh <route, e.g. 000000a2--57fc5a375a> [dest]
set -euo pipefail

DEVICE=${OP_MADS_DEVICE:-comma_4}
REALDATA=/data/media/0/realdata
DEST=${2:-${OP_MADS_LOGS:-}}

if [ "${1:-}" = "--list" ]; then
  ssh "$DEVICE" "ls $REALDATA | sed -n 's/--[0-9]*\$//p' | sort -u"
  exit 0
fi

route=${1:?usage: pull_route.sh --list | pull_route.sh <route> [dest]}
[ -n "$DEST" ] || { echo "pass a destination directory or set OP_MADS_LOGS"; exit 1; }
for seg in $(ssh "$DEVICE" "ls -d $REALDATA/${route}--* | sed 's/.*--//' | sort -n"); do
  mkdir -p "$DEST/$route/$seg"
  # copy to a .part file first so an interrupted copy is never mistaken for a complete segment
  if [ ! -f "$DEST/$route/$seg/rlog.zst" ]; then
    if scp -q -o ConnectTimeout=8 "$DEVICE:$REALDATA/${route}--$seg/rlog.zst" "$DEST/$route/$seg/rlog.zst.part"; then
      mv "$DEST/$route/$seg/rlog.zst.part" "$DEST/$route/$seg/rlog.zst"
    else
      echo "segment $seg: copy failed, rerun to retry"
    fi
  fi
done
echo "$DEST/$route"
