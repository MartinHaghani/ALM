# TODO

Purpose: track deferred fork-specific work that is intentionally not part of the current implementation pass.

## Passive SLAM Confidence Weighting

- Use the published read-only confidence diagnostics to weight boundary-based GPS/LIDAR alignment samples.
- Candidate inputs: GPS confidence, LIDAR local confidence, timestamp match, geometry richness, scan-to-map fit, and SLAM pose jump detection.
- Use confidence only to weight alignment samples and report map quality; passive SLAM must remain visualization-only.
