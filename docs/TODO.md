# TODO

Purpose: track deferred fork-specific work that is intentionally not part of the current implementation pass.

## Passive SLAM Confidence Weighting

- Add confidence weighting to boundary-based GPS/LIDAR alignment.
- Candidate inputs: RTK/GPS accuracy, scan health, timestamp match, motion stability, geometry richness, scan-to-map fit, and SLAM pose jump detection.
- Use confidence only to weight alignment samples and report map quality; passive SLAM must remain visualization-only.
