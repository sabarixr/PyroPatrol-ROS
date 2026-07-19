# AI Architecture And Findings

## Problem Framing

The rover does not just need to know whether a sensor is active. It needs to compare three sectors together and decide:

- `left`
- `front`
- `right`
- `none`

It also benefits from a secondary estimate of how strong or urgent the fire signal is.

## Why The Final Approach Was Chosen

Earlier approaches worked, but the chosen MSCNN matched the task better because fire direction is comparative by nature.

The model needs to reason across sectors, not just within a single sector independently.

## Chosen Model

Multi-Sector CNN with cross-sector attention and dual heads:

- shared 1D CNN encoder per sector
- cross-sector attention across `LEFT`, `FRONT`, `RIGHT`
- direction classification head
- severity regression head

## Input Shape

- `3` sectors
- `10` sensor channels
- `15` timesteps per sector

## Inference Flow

```mermaid
flowchart TD
    A[scan_sample telemetry] --> B[sector buffers]
    B --> C[15-step tensor build]
    C --> D[feature scaling]
    D --> E[MSCNN]
    E --> F[direction]
    E --> G[severity]
    F --> H[/mission/fire_perception]
    G --> H
```

## Findings Summary

The final comparison used leave-one-scan-out cross-validation over `116` real scans.

| Model | Accuracy | Macro F1 |
|---|---:|---:|
| XGBoost (aggregated) | `0.8621` | `0.8549` |
| Random Forest (aggregated) | `0.8276` | `0.8264` |
| Naive 1D CNN (sequence voting) | `0.8103` | `0.8053` |
| MSCNN (chosen) | `0.8966` | `0.8973` |

Severity metrics for the dual-head MSCNN:

- MSE: `0.1656`
- MAE: `0.2266`

## Why MSCNN Beat The Others

- It sees all three sectors together.
- It keeps short temporal behavior instead of only summary statistics.
- It uses attention to compare sectors directly.
- It predicts both direction and severity in one pass.

## Runtime Model Assets

The deployed inference assets are packaged in:

- `src/frr_sensors/models/mscnn_direction.pt`
- `src/frr_sensors/models/mscnn_scaler.pkl`
- `src/frr_sensors/models/mscnn_config.pkl`
- `src/frr_sensors/models/mscnn_classes.npy`

## More Detail

See also:

- [`AI/FINDINGS.md`](../AI/FINDINGS.md)
- `AI/notebooks/train_mscnn.py`
- `AI/data/build_sequence_dataset.py`
