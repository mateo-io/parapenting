#!/bin/zsh
# Build a runnable staged Mac Shipping app for visual QA.
#
# Do not add -archive here. In this UE 5.8 environment the archive path copies
# a thin app without required runtime libraries; the staged app is the valid
# artifact for deterministic VisualQACapture evidence.
set -euo pipefail

project_root="$(cd "$(dirname "$0")/../.." && pwd)"
engine_root="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
uat="$engine_root/Engine/Build/BatchFiles/RunUAT.sh"
stage_root="$project_root/Saved/StagedBuilds/VisualQA"

if [[ ! -x "$uat" ]]; then
    print -u2 "Unreal Automation Tool not found: $uat"
    exit 66
fi

"$uat" BuildCookRun \
    -project="$project_root/Parapenting.uproject" \
    -noP4 -platform=Mac -clientconfig=Shipping \
    -build -cook -stage -pak \
    -stagingdirectory="$stage_root" \
    -utf8output

print "Staged visual-QA app: $stage_root/Mac/Parapenting-Mac-Shipping.app"
