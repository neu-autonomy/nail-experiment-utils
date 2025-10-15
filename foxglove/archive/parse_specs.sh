#!/usr/bin/env bash
# Parsing helpers for run_foxglove_robot_flex.sh
# Safe to source from other scripts.

trim_spec() {                         # strip leading/trailing spaces + CR
  local s="${1//$'\r'/}"
  s="${s#"${s%%[![:space:]]*}"}"
  s="${s%"${s##*[![:space:]]}"}"
  printf '%s' "$s"
}

parse_spec() {
  # Input: TYPE:TOPIC[@HZ][,k=v,...]
  local raw="$(trim_spec "$1")"
  [[ -z "$raw" ]] && { echo "::::"; return; }

  # Validate prefix TYPE:
  local type="${raw%%:*}"
  [[ "$type" != "img" && "$type" != "pcl" && "$type" != "any" ]] && { echo "::::"; return; }

  local rest="${raw#*:}"

  # topic = up to '@' or ',' or end
  local topic="${rest%%[@,]*}"
  [[ -z "$topic" || "$topic" != /* ]] && { echo "::::"; return; }

  # hz = number after '@' if present, until ',' or end
  local hz=""
  if [[ "$rest" == *"@"* ]]; then
    local after_at="${rest#*@}"
    hz="${after_at%%,*}"
    # allow ints/floats
    [[ "$hz" =~ ^[0-9]+([.][0-9]+)?$ ]] || hz=""
  fi

  # opts = stuff after the first ',' (if any)
  local opts=""
  if [[ "$rest" == *,* ]]; then
    opts="${rest#*,}"
    opts="$(trim_spec "$opts")"
  fi

  printf '%s
%s
%s
%s
' "$type" "$topic" "$hz" "$opts"
}

opt_get() {
  # get key from "k1=v1,k2=v2" (no spaces)
  local kvs="$1" key="$2" default="${3:-}"
  IFS=',' read -ra parts <<< "$kvs"
  for kv in "${parts[@]}"; do
    [[ -z "$kv" ]] && continue
    local k="${kv%%=*}"
    local v="${kv#*=}"
    if [[ "$k" == "$key" ]]; then
      printf '%s' "$v"; return 0
    fi
  done
  printf '%s' "$default"
}
