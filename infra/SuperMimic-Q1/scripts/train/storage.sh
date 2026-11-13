#!/bin/bash
# Q1 deep-fold lifecycle: train UNFOLD and FOLD in one shared policy.
set -e
cd "$(dirname "$0")/../.."
export SUPERMIMIC_ROBOT=q1 PYTHONUNBUFFERED=1 Q1_PD="${Q1_PD:-vendor_getup}"
run="$1"; shift
mkdir -p logs
exec .venv/bin/sm-train --run-name "$run" \
  --data.npz-dir ../../q1/storage/motions --data.clips unfold_facedown fold_facedown \
  --env.num-envs 4096 --env.anchor-obs --env.anchor-ori-only --env.future-frames 10 \
  --schedule.max-hours 4 --schedule.eval-interval-min 15 "$@"
