#!/usr/bin/env bash
# One repeatable frame capture, for the performance plan's L1.
#
#   Tools/frame-capture.sh <label> [cvar=value ...]
#
# Prints one row: frame, game thread, GPU, and the GPU's largest pass.
#
# WHY THIS EXISTS RATHER THAN A COMMAND LINE IN A DOCUMENT. Level 0 took a
# frame profile it could stand behind and two comparisons it could not, and
# both failures were the harness:
#
#   * THE SCENE MOVES. The aircraft flies, so two captures compare different
#     parts of the route - and the route's own spread is 7.8 ms in open air
#     against 15-28 near the launch slope, three times the saving the next
#     level is chasing.
#
#     TWO ATTEMPTS TO FREEZE IT FAILED FOR ONE STUPID REASON, AND THE FIRST
#     EXPLANATION OF IT WAS ALSO WRONG. `pause` and `slomo 0.001` both appeared
#     to be ignored, and this comment used to say that -ExecCmds runs before a
#     world exists so world commands are dropped. That was a plausible story
#     and it was not the reason. -ExecCmds SEPARATES ON COMMAS, NOT SEMICOLONS:
#     the whole semicolon-joined string was executed as ONE command, so the
#     leading cvar swallowed the rest as its argument and `atoi` stopped at the
#     first `;`. That is also why `t.MaxFPS 120; slomo ...` still capped at 120
#     - the number parsed - and nothing downstream ran. Commas below.
#
#     Kept because the failure was invisible in every direction: the cap worked,
#     so the string looked fine; the freeze did not, so the blame went to the
#     engine lifecycle. What caught it was checking the log for the command
#     rather than reasoning about it.
#
#     So the scene is not frozen - it is ALIGNED. The simulation is
#     deterministic and locked to real time, so the same number of seconds
#     after the flight starts is the same aircraft in the same place at any
#     frame rate. What defeated wall-clock alignment in Level 0 was that the
#     capture starts at ENGINE start and level load time varies between runs,
#     sliding the flight underneath the window. So this finds the start of the
#     flight in the capture itself, and measures from there.
#   * THE WINDOW LOSES FOREGROUND AND THE ENGINE THROTTLES. CPU swinging 7% to
#     137%, one frame at 1013 ms against a 7.5 ms median. `t.IdleWhenNotForeground 0`
#     is the switch for that.
#
# It also settles before capturing, so shader compilation and level load are
# not in the average, and it reads the CSV BY COLUMN NAME - the column layout
# changes between captures, which silently moved FrameTime by 27 columns
# between two of Level 0's runs.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENGINE="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
EDITOR="$ENGINE/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"
CSV_DIR="$HOME/Library/Application Support/Epic/UnrealEngine/5.8/Saved/Profiling/CSV"

LABEL="${1:-reference}"; shift || true
# The capture starts at engine start, so it has to span the level load AND
# the settle AND the window. 6000 frames is ~50 s at 120 fps, leaving ~15 s
# of window after a 35 s settle. A slower row simply takes longer in wall
# time; the scene is frozen, so a long settle costs nothing but patience.
SETTLE="${FRAME_CAPTURE_SETTLE:-20}"   # seconds after the flight starts
WINDOW="${FRAME_CAPTURE_WINDOW:-12}"   # seconds averaged
FRAMES="${FRAME_CAPTURE_FRAMES:-6000}"

# The frozen scene, the throttle defeat, then whatever the caller is testing -
# last, so a caller can override any of it deliberately.
CMDS="t.IdleWhenNotForeground 0,t.MaxFPS 120"
for kv in "$@"; do CMDS="$CMDS,${kv/=/ }"; done

[[ -x "$EDITOR" ]] || { echo "error: no engine at $ENGINE" >&2; exit 1; }
# A plain path with a .log suffix: -abslog silently wrote nothing
# to the mktemp path the first version used, so the "Capture Ended"
# poll never fired and every run waited out its whole timeout.
LOG="/tmp/frame-capture-$$.log"
BEFORE="$(ls -t "$CSV_DIR"/*.csv 2>/dev/null | head -1 || true)"

"$EDITOR" "$ROOT/Parapenting.uproject" -game -windowed -resx=1280 -resy=720 \
    -csvCaptureFrames="$FRAMES" -csvGpuStats -ExecCmds="$CMDS" \
    -abslog="$LOG" >/dev/null 2>&1 &
ENGINE_PID=$!
trap 'kill "$ENGINE_PID" 2>/dev/null || true' EXIT

for _ in $(seq 1 120); do
    grep -qi "Capture Ended" "$LOG" 2>/dev/null && break
    kill -0 "$ENGINE_PID" 2>/dev/null || break
    sleep 5
done
kill "$ENGINE_PID" 2>/dev/null || true
sleep 2

CSV="$(ls -t "$CSV_DIR"/*.csv 2>/dev/null | head -1 || true)"
[[ -n "$CSV" && "$CSV" != "$BEFORE" ]] || { echo "error: no new capture" >&2; exit 1; }

# Settle frames are dropped by TIME rather than by count, so the same number of
# seconds is discarded whatever frame rate the row runs at.
awk -F, -v label="$LABEL" -v settle="$SETTLE" -v window="$WINDOW" '
NR==1 { cols = NF; for (i=1;i<=NF;i++) { gsub(/^ +| +$/,"",$i); c[$i]=i; nm[i]=$i } next }
$c["FrameTime"] ~ /^[0-9.]+$/ { rows++; ft[rows]=$c["FrameTime"]+0; line[rows]=$0 }
END {
  if (!rows) { printf "%-22s NO ROWS\n", label; exit 1 }

  # WHERE THE FLIGHT STARTS. Level load and shader compilation produce long,
  # irregular frames; flight produces short even ones. So the start is the
  # first frame after which two continuous seconds pass with nothing over
  # 50 ms. Stated as a rule rather than a constant chosen per run, and printed
  # so a row whose start was misdetected is visible rather than silent.
  t = 0; cand = -1; run = 0
  for (i = 1; i <= rows; i++) {
    if (ft[i] > 50) { cand = -1; run = 0 }
    else { if (cand < 0) { cand = t; run = 0 }; run += ft[i]/1000.0 }
    t += ft[i]/1000.0
    if (cand >= 0 && run >= 2.0) { start = cand; found = 1; break }
  }
  if (!found) { printf "%-22s FLIGHT START NOT FOUND\n", label; exit 1 }

  from = start + settle; to = from + window
  t = 0
  for (i = 1; i <= rows; i++) {
    t += ft[i]/1000.0
    if (t < from || t > to) continue
    split(line[i], v, ",")
    n++; f += ft[i]; g += v[c["GameThreadTime"]]; gpu += v[c["GPUTime"]]
    tick += v[c["Exclusive/GameThread/TickActors"]]
    # cols, not NF: in END, NF holds the field count of the LAST record,
    # and a trailing blank line makes it zero - which silently reported no
    # GPU pass at all on the first run of this.
    for (j = 1; j <= cols; j++) {
      if (nm[j] ~ /^GPU\// && v[j]+0 == v[j]) pass[nm[j]] += v[j]
      # L3: game-thread attribution, from CSV_DEFINE_CATEGORY(Parapenting).
      if (nm[j] ~ /^Parapenting\// && v[j]+0 == v[j]) { own[nm[j]] += v[j]; seen[nm[j]]++ }
    }
    x = v[c["GPUTime"]] + 0
    if (x > hi) hi = x
    if (lo == 0 || x < lo) lo = x
  }
  if (!n) { printf "%-22s window %.0f-%.0f s not covered (capture spans %.0f s, flight began %.0f s)\n",
            label, from, to, t, start; exit 1 }

  top = ""; topv = 0
  for (k in pass) if (pass[k]/n > topv) { topv = pass[k]/n; top = k }
  sub(/^GPU\//, "", top)
  spread = 100*(hi-lo)/(gpu/n)
  printf "%-22s %7.2f ms %6.1f fps | game %5.2f (tick %5.2f) | GPU %6.2f | %-24s %5.2f | flight+%.0fs spread %3.0f%%\n",
         label, f/n, 1000/(f/n), g/n, tick/n, gpu/n, top, topv, settle, spread
  # Printed only when the module carries the L3 stats, so a stock build gives
  # the same single row it always did.
  if (length(own)) {
    acc = 0
    for (k in own) { sub(/^Parapenting\//, "", k) }
    for (k in own) { kk = k; sub(/^Parapenting\//, "", kk); acc += own[k]/n
      printf "    %-26s %6.3f ms\n", kk, own[k]/n }
    printf "    %-26s %6.3f ms  (game thread %.2f, so %.2f is elsewhere)\n",
           "= attributed", acc, g/n, g/n - acc
  }
}' "$CSV"
