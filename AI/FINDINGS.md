# PyroPatrol Fire-Direction Model Findings

## Goal

We tested several model families to predict where fire is relative to the rover from one full sensor scan:

- `front`
- `left`
- `right`
- `none`

The main goal was to choose the model that is most reliable at scan level, because the robot acts on a whole scan, not on a single sensor frame.

## Data Used

- Total real scans evaluated: `116`
- Class balance:
  - `front`: 35
  - `left`: 25
  - `none`: 30
  - `right`: 26
- Evaluation method: leave-one-scan-out cross-validation

This means each scan was held out once, tested once, and never seen during training for that fold.

## Models Compared

### 1. XGBoost on aggregated scan features

- Accuracy: `0.8621`
- Macro F1: `0.8549`

This was the strongest classical ML baseline.

### 2. Random Forest on aggregated scan features

- Accuracy: `0.8276`
- Macro F1: `0.8264`

This performed reasonably well, but clearly behind XGBoost.

### 3. Naive 1D CNN with sequence voting

- Accuracy: `0.8103`
- Macro F1: `0.8053`

This model made one prediction per sector sequence and then used voting. It worked, but it was weaker because it did not compare sectors directly in one forward pass.

### 4. Multi-Sector CNN (MSCNN)

- Accuracy: `0.8966`
- Macro F1: `0.8973`

This was the best overall model.

## Why MSCNN Won

The task is naturally comparative. The rover is not only asking "does this sector look hot?" It is really asking:

- which sector looks stronger than the others?
- is the strongest signal in `left`, `front`, or `right`?
- is there no meaningful fire signal at all?

The MSCNN handles this better because it sees all three sectors together in one sample and uses cross-sector attention to compare them directly.

In plain language:

- XGBoost was a strong baseline because it also saw combined scan features.
- The older CNN was weaker because it split the problem into independent sector predictions first.
- MSCNN kept the raw temporal signal and still compared sectors jointly, which matched the real problem better.

## Final Recommendation

Use the MSCNN as the runtime model for ROS.

Reasons:

- Best scan-level accuracy of all tested models
- Best macro F1, so performance is more balanced across classes
- Works directly on the three-sector structure of the scan
- Also predicts a severity score, which can be used for action gating such as pump activation

## Runtime Decision

The ROS project now keeps the MSCNN as the active model path.

Used runtime artifacts:

- `mscnn_direction.pt`
- `mscnn_scaler.pkl`
- `mscnn_config.pkl`
- `mscnn_classes.npy`

## Notes On Severity

The dual-head MSCNN also predicts a fire severity value.

Reported validation metrics:

- Severity MSE: `0.1656`
- Severity MAE: `0.2266`

This severity output is useful as a secondary signal, but direction classification is still the main decision output.

## Practical Takeaway

The current best path is:

1. ESP32 publishes scan samples.
2. `fire_perception_node` buffers `LEFT`, `FRONT`, and `RIGHT` sectors.
3. MSCNN predicts direction and severity for the full scan.
4. Mission controller uses that output for turning and pump decisions.

That gives the rover one consistent scan-level decision instead of piecing together several weaker local predictions.
