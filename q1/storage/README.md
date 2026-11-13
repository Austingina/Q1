# Q1 low-level storage skills

This directory prepares two distinct 50 Hz skills for the unified low-level controller:

- `UNFOLD`: deep storage fold to the existing flat face-down get-up start pose.
- `FOLD`: the existing face-down lie-down end pose to the deep storage fold.

The source motion is the Q1 firmware's crawl-to-fold reference plus the final joint pose
from its classic storage trajectory. `build_storage_refs.py` adds smooth endpoint bridges
and recomputes every body pose with the repository's Q1 MuJoCo model. It also writes
source hashes to `motions/manifest.json`.

Build references in the SuperMimic environment:

```bash
cd infra/SuperMimic-Q1
SUPERMIMIC_ROBOT=q1 uv run python ../../q1/storage/build_storage_refs.py
```

Train both clips in one policy (a dedicated launcher is provided):

```bash
scripts/train/storage.sh q1_storage_v1
```

Export the same checkpoint twice, once with each reference, so the runtime gets explicit
packages and cannot swap their direction accidentally:

```bash
SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/export_package.py \
  --ckpt logs/q1_storage_v1/ckpt_latest.pt \
  --motion-npz ../../q1/storage/motions/unfold_facedown.npz \
  --out ../../weights/q1_unfold

SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/export_package.py \
  --ckpt logs/q1_storage_v1/ckpt_latest.pt \
  --motion-npz ../../q1/storage/motions/fold_facedown.npz \
  --out ../../weights/q1_fold
```

These references are training inputs, not motor trajectories. Do not pass either NPZ or
the firmware CSV directly to the real-robot backend. A trained export is still blocked
from hardware until simulation evaluation, fake-topic dry-run, suspended low-level hold,
and independent emergency-stop checks pass.
