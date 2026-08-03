#!/bin/zsh
set -euo pipefail

if (( $# < 1 )); then
    print -u2 "usage: $0 /path/to/Parapenting-Mac-Shipping [warmup-seconds]"
    exit 64
fi

shipping_binary="$1"
warmup_seconds="${2:-6}"

if [[ ! -x "$shipping_binary" ]]; then
    print -u2 "Shipping executable is not executable: $shipping_binary"
    exit 66
fi

for visual_qa_case in morning:8 midday:13 evening:19; do
    capture_name="${visual_qa_case%%:*}"
    local_hour="${visual_qa_case##*:}"
    "$shipping_binary" \
        -windowed -ResX=1600 -ResY=900 -NoSplash \
        -VisualQACapture="$capture_name" \
        -VisualQAHour="$local_hour" \
        -VisualQAWarmup="$warmup_seconds"
done

print "Level 1 captures are under the packaged game's Saved/VisualQA directory."
