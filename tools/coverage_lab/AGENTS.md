# Coverage planner lab guide

Purpose: guardrails for the laptop-only Fields2Cover evaluation lab.

- This directory is for offline route-planning experiments only. Do not wire it into `open_mower.launch`, `mower_logic`, runtime Docker images, or mower startup scripts.
- Do not add automatic SSH or mower-control behavior here. Current mower maps are copied manually into `data/maps/`.
- Treat files under `data/maps/` and `runs/` as private local artifacts. They are ignored on purpose and should not be committed unless a user explicitly asks to publish a scrubbed sample.
- Keep the first migration target compatible with the existing planner concept: map-frame points and `PlanPath`-like `paths[]` output. Do not change the mower-side service until the offline behavior is accepted.
- Prefer adding small tracked example maps under `examples/` for tests and documentation.
- If planner defaults drift from Mowrator params, update `configs/default.yaml`, `README.md`, and `docs/COVERAGE_PLANNER_LAB.md` together.
