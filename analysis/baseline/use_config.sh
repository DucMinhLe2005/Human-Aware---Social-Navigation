#!/bin/bash
# Switch the INSTALLED navigation.yaml between PS-VSP and the DWB baseline.
# navigation.launch.py always loads the installed file, so this is the only way
# to change the controller without editing src_humanaware.
#
#   use_config.sh dwb         DWB, social layer removed (human-unaware baseline)
#   use_config.sh dwb_social  DWB on the same social costmap as PS-VSP (ablation)
#   use_config.sh psvsp    restore the original PS-VSP config
#   use_config.sh status   show which one is active
#
# Nothing under src/ is written: with a symlink install the link itself is
# moved aside and put back, never the file it points to.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(cd "$HERE/../../src_humanaware" && pwd)}"
SRC="$WS/src/linorobot2/linorobot2_navigation/config/navigation.yaml"
INST="$WS/install/linorobot2_navigation/share/linorobot2_navigation/config/navigation.yaml"
BACKUP="$INST.psvsp_backup"
[ -e "$INST" ] || [ -e "$BACKUP" ] || { echo "ERROR: $INST not found -- build the workspace first"; exit 1; }

busy() {
  # Match process names, not command lines, so a shell that merely mentions
  # these words is not mistaken for a running stack.
  if pgrep -x 'component_conta|controller_serv|planner_server|bt_navigator|run_batch.sh' >/dev/null; then
    pgrep -ax 'component_conta|controller_serv|planner_server|bt_navigator|run_batch.sh'
    echo "ERROR: Nav2 or a batch is running (listed above) -- stop it first"; exit 1
  fi
}

status() {
  local plugin
  plugin=$(grep -m1 -A1 '^ *FollowPath:' "$INST" | grep plugin | sed 's/.*plugin: *//; s/["'\'']//g')
  echo "active controller : $plugin"
  echo "social layer      : $(grep -c aghpm_social_layer "$INST" | sed 's/^0$/removed/; s/^[1-9].*/present/')"
  echo "installed file md5: $(md5sum "$(readlink -f "$INST")" | cut -c1-12)"
  [ -e "$BACKUP" ] && echo "state             : BASELINE (PS-VSP config kept at $BACKUP)" \
                   || echo "state             : PS-VSP (original)"
}

case "${1:-status}" in
  dwb|dwb_social)
    busy
    [ -e "$BACKUP" ] && { echo "a baseline config is already installed -- run 'use_config.sh psvsp' first"; status; exit 1; }
    TMP="$(mktemp)"
    FLAG=""; [ "$1" = "dwb_social" ] && FLAG="--keep-social-layer"
    python3 "$HERE/make_dwb_yaml.py" "$SRC" "$TMP" $FLAG
    mv "$INST" "$BACKUP"          # moves the symlink itself, not its target
    cp "$TMP" "$INST"; rm -f "$TMP"
    status ;;
  psvsp)
    busy
    [ -e "$BACKUP" ] || { echo "already on the PS-VSP config"; status; exit 0; }
    rm -f "$INST"; mv "$BACKUP" "$INST"
    cmp -s "$(readlink -f "$INST")" "$SRC" || { echo "ERROR: restored file differs from src"; exit 1; }
    status ;;
  status) status ;;
  *) echo "usage: $0 dwb|dwb_social|psvsp|status"; exit 2 ;;
esac
