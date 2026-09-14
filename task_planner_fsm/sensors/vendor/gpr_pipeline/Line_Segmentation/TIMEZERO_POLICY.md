# Time-zero coordinate policy

Robot-facing line coordinates are computed only after Berta's detector has run on the complete gain-only B-scan.

For the GPRTools image convention, sample/row 0 is at the top and radar time increases downward.

```text
original_delta_samples = detected_sample - time_zero_sample
corrected_sample = max(detected_sample, time_zero_sample)
robot_offset_samples = corrected_sample - time_zero_sample
```

Therefore:

- detected sample 35, time-zero 40 -> corrected sample 40, robot position 0, depth 0 cm.
- detected sample 40, time-zero 35 -> corrected sample 40, positive robot position/depth.

The original Berta coordinate is kept only for diagnostics.
