# Swept Area Recording

Purpose: document the mower-runtime behavior that records mowing and obstacle polygons from the mower's full swept footprint instead of from a single reference corner.

## Current Status

This behavior is in the `mower_logic` runtime path, not the laptop-only coverage planner lab.

The implementation adds `mower_logic::area_recording::SweptAreaRecorder`, backed by vendored Clipper2 C++ geometry. The recorder samples the mower footprint at recorded `base_link` poses, interpolates between poses by distance and yaw, unions all footprint polygons, then selects one boundary from that union:

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

During polygon recording, `AreaRecordingBehavior` still records the existing breadcrumb polygons and publishes boundary samples for GPS/LIDAR alignment. In parallel, it records pose segments for swept-area reconstruction. A new swept segment starts after GPS quality drops so the union does not bridge across an untrusted localization gap.

When the operator finishes recording:

- mowing outlines are rebuilt from the largest exterior swept boundary
- obstacle outlines are rebuilt from the largest interior swept hole
- navigation outlines keep the existing center/base polygon

If swept reconstruction fails, the area is not saved. This is deliberate: saving a corrupted geometry would be much harder to detect later than rejecting the recording immediately.

The live preview remains lightweight and preview-only. It still shows the old reference-point breadcrumb path while recording: mowing outlines use the front-right footprint corner, obstacle outlines use the front-left footprint corner, and navigation outlines use `base_link`. The final saved mowing and obstacle polygons are produced only by the swept-union pass when the recording is saved.

## Diagnostics

When a swept mowing outline or obstacle is saved, `mower_logic` logs:

- input pose count and interpolated footprint count
- number of swept segments, which is greater than one after GPS quality dropouts
- number of exterior components and interior holes in the union
- union area, selected boundary area, largest exterior area, and largest hole area
- selected boundary perimeter, bounding-box size, fill ratio, and saved vertex count

Warnings are emitted when GPS dropouts split a recording, when the union has disconnected exterior components, or when a saved mowing exterior has a low bounding-box fill ratio. The fill-ratio warning usually means the swept loop is open or has a gap, so the exterior boundary is tracing both outside and inside edges of the swept paint.

## Parameters

The implementation reads these ROS params under `/mower_logic/`. They are also exposed in the schema and legacy shell example as `OM_AREA_RECORDING_*` environment variables:

- `area_recording_pose_step_m`, default `0.03`
- `area_recording_yaw_step_rad`, default `0.05`
- `area_recording_simplify_epsilon_m`, default `0.03`
- `area_recording_min_polygon_area_m2`, default `0.25`

The code clamps these values to conservative ranges before use.

## Safety Notes

This is safety-sensitive because it changes what geometry is persisted into mower maps. The intended invariant is that the saved mowing polygon must never claim more area than the trusted RTK/fused pose stream and mower footprint actually swept.

Before deploying this path on the mower:

- build `mower_logic` with tests enabled
- run `test_swept_area_recorder`
- record a simple rectangle and one obstacle in a controlled area
- inspect the stored map in both WebUIs
- confirm boundary samples still accumulate for GPS/LIDAR alignment
- clear existing mower maps before using the new semantics; old front-corner maps should be rerecorded

## Known Follow-Ups

- Add integration-level tests around `AreaRecordingBehavior` so mowing, navigation, and obstacle recording semantics are checked together.
- Confirm with a live recording that the selected obstacle hole is the intended interior boundary when the mower circles an obstacle with the front-left side facing it.
- Decide whether the front-right/front-left boundary samples should remain corner-based for alignment while saved map geometry becomes swept-footprint based. That split is intentional today, but it should be validated against the GPS/LIDAR calibration workflow.
