#!/usr/bin/env bash
# a51.sh -- reach the flight box (Taylor's Alienware) from this laptop, and keep a copy of everything it prints.
#
#   hardware/a51.sh               a shell on the Alienware, in ~/vu-autonomy-compositional
#   hardware/a51.sh ground        the hover runner's ground check (motors never start)
#   hardware/a51.sh hover         the hover flight (you type "hover" to take off)
#   hardware/a51.sh run 'CMD'     any one command there, e.g.  hardware/a51.sh run 'ls ~/tello-runs'
#   hardware/a51.sh sync          copy this laptop's whole repo to the Alienware (not nuXmv; never into the kit)
#   hardware/a51.sh last          print the newest saved log here, to copy and paste
#
# Every session is also saved on THIS laptop in hardware/logs/<date>_<time>_<what>.log (git-ignored).
#
# Flying from the laptop is fine when you stand by the drone: Ctrl-C here reaches the flight host, which lands
# (ONE Ctrl-C). If the connection drops, the host gets SIGHUP and lands too. Needs your key on the Alienware
# (done 2026-10-13) and the lab network.
set -euo pipefail
HOST="${A51_HOST:-verivital@10.2.219.242}"
REMOTE_DIR="vu-autonomy-compositional"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="$HERE/logs"
mkdir -p "$LOGDIR"
SSH=(ssh -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)

logged() {  # logged NAME CMD...   run CMD, show it live, and keep a copy in hardware/logs/
  local log="$LOGDIR/$(date +%F_%H%M%S)_$1.log"; shift
  echo "(saving to $log)"
  # -t gives the remote command a terminal, so its questions and Ctrl-C work as if typed there
  "$@" 2>&1 | tee "$log"
  local rc=${PIPESTATUS[0]}
  echo "(saved: $log, exit $rc)"
  return "$rc"
}

remote() {  # remote NAME 'COMMAND'
  logged "$1" "${SSH[@]}" -t "$HOST" "cd ~/$REMOTE_DIR && $2"
}

case "${1:-shell}" in
  shell)  exec "${SSH[@]}" -t "$HOST" "cd ~/$REMOTE_DIR && exec bash -l" ;;
  ground) remote ground "python3 hardware/neo2_hover.py --ground" ;;
  hover)  remote hover "python3 hardware/neo2_hover.py" ;;
  run)    [ -n "${2:-}" ] || { echo "usage: $0 run 'COMMAND'"; exit 2; }
          remote run "$2" ;;
  sync)   # the whole working tree, .git included, so the Alienware's copy is the same checkout as this one.
          # Never nuXmv (its licence forbids redistribution), nor this laptop's logs and caches. Deletes nothing there.
          rsync -a --exclude='/nuXmv' --exclude='hardware/logs/' --exclude='__pycache__/' --exclude='.pytest_cache/' \
                -e "${SSH[*]} -o BatchMode=yes" "$HERE/../" "$HOST:$REMOTE_DIR/"
          echo "synced to $HOST:~/$REMOTE_DIR/ :"
          "${SSH[@]}" -o BatchMode=yes "$HOST" "cd ~/$REMOTE_DIR && git log --oneline -1 && git status --short" ;;
  last)   f="$(ls -t "$LOGDIR"/*.log 2>/dev/null | head -1)"; [ -n "$f" ] || { echo "no logs yet"; exit 1; }
          echo "== $f"; cat "$f" ;;
  -h|--help|help) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//' ;;
  *) echo "unknown: $1   (try: $0 help)"; exit 2 ;;
esac
