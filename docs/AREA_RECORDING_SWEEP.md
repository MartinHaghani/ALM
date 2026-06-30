# Swept Area Recording

Purpose: document the mower-runtime behavior that records mowing and obstacle polygons from the mower's full swept footprint instead of from a single reference corner.

## Current Status

This behavior is in the `mower_logic` runtime path, not the laptop-only coverage planner lab.

The implementation adds `mower_logic::area_recording::SweptAreaRecorder`, backed by vendored Clipper2 C++ geometry. The recorder samples the mower footprint at selected recording poses, interpolates between poses by distance and yaw, unions all footprint polygons, then selects one boundary from that union:

- mowing area: largest exterior boundary of the swept union
- obstacle: largest interior hole of the swept union
- navigation area: unchanged, still records the `base_link` polygon

This is intended to make a recorded mowing area represent "where the physical mower footprint actually swept" instead of only the front-right corner breadcrumb. It should better match what the mower can physically cover and what the future coverage planner expects from a map boundary.

## Files

- `src/mower_logic/src/mower_logic/behaviors/SweptAreaRecorder.h`
- `src/mower_logic/src/mower_logic/behaviors/SweptAreaRecorder.cpp`
- `src/mower_logic/src/mower_logic/behaviors/AreaRecordingBehavior.cpp`
- `src/mower_logic/third_party/clipper2/`
- `src/mower_logic/test/test_swept_area_recorder.cpp`

`src/mower_logic/third_party/clipper2/README.md` records the vendored Clipper2 version and license.

## Recording Flow

During polygon recording, `AreaRecordingBehavior` still records the existing breadcrumb polygons and publishes boundary samples for GPS/LIDAR alignment. In parallel, it records pose segments for swept-area reconstruction. The default geometry source is `/localization_fusion/pose`; `/next/` can switch runtime recording back to the legacy `/xbot_positioning/xb_pose` source while no polygon is actively recording. A new swept segment starts after selected-pose quality drops so the union does not bridge across an untrusted localization gap.

Boundary samples on `/area_recorder/boundary_samples` remain GPS-truth samples. They are published only when raw/legacy GPS is RTK fixed and within `/xbot_positioning/max_gps_accuracy`, even while fused-pose recording is enabled. LIDAR-derived fused positions are not fed back into GPS/LIDAR alignment calibration.

With the multi-lawn map selector, recording writes into the currently selected active map. Create or select the intended saved map from `/next/` while the mower is idle before entering area-recording mode.

When the operator finishes recording:

- mowing outlines are rebuilt from the largest exterior swept boundary
- obstacle outlines are rebuilt from the largest interior swept hole
- navigation outlines keep the existing center/base polygon

If swept reconstruction fails, the area is not saved. This is deliberate: saving a corrupted geometry would be much harder to detect later than rejecting the recording immediately.

The live preview remains preview-only, but it now displays each accepted mower footprint on the shared map overlay instead of relying only on the old reference-point breadcrumb. Recording-pose quality gaps start a new swept segment, so the preview shows visible gaps where the selected localization source was unavailable or untrusted. The final saved mowing and obstacle polygons are still produced only by the swept-union pass when the recording is saved.

The `/next/` Map tab can queue paint-style draft edits while area recording is paused. Those queued strokes are applied to the swept boundary immediately before the area is saved. They do not create or modify `area_recorder/boundary_samples`; those samples remain the recorded RTK GPS-truth path used by GPS/LIDAR alignment.

## Diagnostics

When a swept mowing outline or obstacle is saved, `mower_logic` logs:

- input pose count and interpolated footprint count
- number of swept segments, which is greater than one after recording-pose quality gaps
- number of exterior components and interior holes in the union
- union area, selected boundary area, largest exterior area, and largest hole area
- selected boundary perimeter, bounding-box size, fill ratio, and saved vertex count

Warnings are emitted when pose-quality gaps split a recording, when the union has disconnected exterior components, or when a saved mowing exterior has a low bounding-box fill ratio. The fill-ratio warning usually means the swept loop is open or has a gap, so the exterior boundary is tracing both outside and inside edges of the swept paint.

## Parameters

The implementation reads these ROS params under `/mower_logic/`. They are also exposed in the schema and legacy shell example as `OM_AREA_RECORDING_*` environment variables:

- `area_recording_pose_step_m`, default `0.03`
- `area_recording_yaw_step_rad`, default `0.05`
- `area_recording_simplify_epsilon_m`, default `0.03`
- `area_recording_min_polygon_area_m2`, default `0.25`
- `area_recording_use_localization_fusion`, default `true`
- `area_recording_fused_pose_topic`, default `/localization_fusion/pose`
- `area_recording_legacy_pose_topic`, default `/xbot_positioning/xb_pose`
- `area_recording_max_pose_age_sec`, default `1.0`
- `area_recording_max_fused_position_accuracy_m`, default `0.2`
- `area_recording_max_fused_yaw_accuracy_rad`, default `0.15`

The code clamps these values to conservative ranges before use.

The runtime source switch is exposed through `/mower_service/set_area_recording_use_fused_pose` (`std_srvs/SetBool`) and the latched `/area_recorder/use_fused_pose` (`std_msgs/Bool`) topic for `/next/`.

## Safety Notes

This is safety-sensitive because it changes what geometry is persisted into mower maps. The intended invariant is that the saved mowing polygon must never claim more area than the trusted selected pose stream and mower footprint actually swept.

Manual map edits are a deliberate exception for operator repair after imperfect recording. They run through gated mower services, update only selected-map geometry, and leave boundary samples untouched so calibration diagnostics continue to reflect real recorded motion.

Before deploying this path on the mower:

- build `mower_logic` with tests enabled
- run `test_swept_area_recorder`
- run `test_area_recording_pose_gate`
- record a simple rectangle and one obstacle in a controlled area
- inspect the stored map in both WebUIs
- confirm boundary samples still accumulate only during RTK-fixed GPS-truth recording intervals
- clear existing mower maps before using the new semantics; old front-corner maps should be rerecorded

## Known Follow-Ups

- Add integration-level tests around `AreaRecordingBehavior` so mowing, navigation, and obstacle recording semantics are checked together.
- Confirm with a live recording that the selected obstacle hole is the intended interior boundary when the mower circles an obstacle with the front-left side facing it.
- Validate the fused-pose recording path live with RTK fixed, LIDAR-only/fused, and fusion-stale segments before trusting it for production maps.
