#!/usr/bin/env bash
set -euo pipefail

DEEP_CLEAN=0
BACKUP_USER_CONFIG=0

for arg in "$@"; do
  case "$arg" in
    --deep-clean) DEEP_CLEAN=1 ;;
    --backup-user-config) BACKUP_USER_CONFIG=1 ;;
    *) echo "unknown arg: $arg" >&2; exit 64 ;;
  esac
done

CACHE_ROOT="${XDG_CACHE_HOME:-$HOME/.cache}"
BACKUP_ROOT="${CACHE_ROOT}/isaacsim-recovery/$(date +%Y%m%d-%H%M%S)"
MOVED_ANY=0

move_if_exists() {
  local src="$1"
  local rel_dest="$2"

  if [[ ! -e "$src" ]]; then
    printf 'skip %s (not present)\n' "$src"
    return
  fi

  mkdir -p "$BACKUP_ROOT/$(dirname "$rel_dest")"
  mv "$src" "$BACKUP_ROOT/$rel_dest"
  printf 'moved %s -> %s\n' "$src" "$BACKUP_ROOT/$rel_dest"
  MOVED_ANY=1
}

move_if_exists "$CACHE_ROOT/ov/shaders" "cache/ov/shaders"
move_if_exists "$CACHE_ROOT/nvidia/GLCache" "cache/nvidia/GLCache"
move_if_exists "$CACHE_ROOT/mesa_shader_cache" "cache/mesa_shader_cache"

if [[ "$DEEP_CLEAN" == "1" ]]; then
  move_if_exists "$CACHE_ROOT/ov/DerivedDataCache" "cache/ov/DerivedDataCache"
fi

if [[ "$BACKUP_USER_CONFIG" == "1" ]]; then
  move_if_exists \
    "$HOME/.local/share/ov/data/Kit/Isaac-Sim Full/5.0/user.config.json" \
    "data/Kit/Isaac-Sim Full/5.0/user.config.json"
fi

if [[ "$MOVED_ANY" == "0" ]]; then
  rmdir "$BACKUP_ROOT" 2>/dev/null || true
  echo "nothing moved"
  exit 0
fi

printf 'backup root: %s\n' "$BACKUP_ROOT"
