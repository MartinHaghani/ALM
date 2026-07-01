# Open Mower Next WebUI

This is the source for the new Vite + React + TypeScript WebUI served under `/next/`.
It defaults to an OpenLayers combined map that can overlay optional satellite imagery, stored mower map JSON from
`/mower_map_service/json_map`, passive SLAM occupied cells from `/slam_toolbox/map`, optional live scan points from
`/slam_toolbox/scan`, separate GPS/fused and LIDAR/SLAM robot markers, the shadow unified localization marker from
`/localization_fusion/pose` when that node is running, live area-recording overlays from
`/xbot_monitoring/map_overlay`, coverage route snapshots from `/mower_logic/route_plan_json`, MBF planner/controller
paths, and the sampled actual mowing track. The Sensors tab shows C1 LIDAR/IMU diagnostics, thermal health cards
for the Flipsky ESCs, Pi CPU, IMU, and GNSS receiver through `xbot_monitoring/sensors/*/data`, and the left drive,
right drive, and blade ESC battery-input voltages from `/hw/power`.
Passive SLAM controls are visualization-only and do not change mower localization, planning,
costmaps, or control.

The Map tab also exposes the existing mower area-recording workflow through rosbridge: it subscribes to the live
`/xbot_monitoring/actions_json` action set, publishes selected action IDs to `/xbot/action`, and publishes manual
drive commands to `/web_joy_vel` while the mower is in `AREA_RECORDING` and the manual input source is `web_gamepad`.
The header Bluetooth panel subscribes `/bluetooth_gamepad/status` and `/mower_input/status`, calls the matching
Bluetooth/source-selection services, and disables browser gamepad publishing when `direct_bluetooth` is active.
This mirrors the root WebUI's recording controls
except record docking, which is intentionally not exposed in `/next/`.

The Map tab also includes the multi-lawn map selector. It subscribes to `/mower_map_service/map_catalog` for the saved
map list and calls the guarded `/mower_service/create_map`, `/mower_service/select_map`, `/mower_service/rename_map`,
and `/mower_service/delete_map` services so map mutations are accepted only while mower logic is idle. The rendered
map still comes from `/mower_map_service/json_map`, which is the currently selected active map.

The same Map tab includes v1 map editing tools. Paint mode sends one pen or eraser stroke at a time to the guarded
`/mower_service/apply_map_edit` service while idle; during paused area recording it queues draft strokes through
`/mower_service/apply_recording_edit` before the area is saved. Polygon mode edits saved area vertices by dragging,
adding, or removing points. Editing changes selected-map geometry only; area-recording boundary samples remain the
recorded path used by GPS/LIDAR alignment.

Runtime map settings can be overridden in `/next/config.js` via `window.OPEN_MOWER_NEXT_CONFIG`.
The default basemap is York Region's 2023 10 cm orthophoto ArcGIS REST service. Supported keys include
`satelliteSourceType`, `satelliteArcGisRestUrl`, `satelliteArcGisLayers`, `satelliteArcGisFormat`,
`satelliteTileUrl`, `satelliteAttribution`, `satelliteMaxZoom`, `gpsFixTopic`, `gpsStatusTopic`, `gpsRawPoseTopic`,
`gpsFusedPoseTopic`, `mowerMapTopic`, `scanTopic`, `slamScanTopic`, `slamMapTopic`, `slamMapFrame`, `slamOdomFrame`,
`slamOriginFrame`, `slamBaseFrame`, `slamAlignmentStatusTopic`, `slamManagerStatusTopic`, `slamSetMappingService`,
`slamClearMapService`, `passiveSlamOdomStatusTopic`, `passiveSlamGyroCalibrateService`, `localizationConfidenceTopic`,
`localizationFusionPoseTopic`, `localizationFusionStatusTopic`, `localizationFusionBaseFrame`, `actionTopic`,
`actionsTopic`, `webJoyTopic`, `joyTopic`, `bluetoothStatusTopic`, `manualInputStatusTopic`, `hwPowerTopic`,
`batteryFullVoltage`, `batteryEmptyVoltage`, `batteryCriticalVoltage`, `driveVoltageMismatchWarnV`,
`mapOverlayTopic`, `mapCatalogTopic`, `mapCreateService`, `mapSelectService`,
`mapRenameService`, `mapDeleteService`, `mapEditService`, `recordingEditService`, `mbfGlobalPlanTopic`,
`mbfControllerPlanTopic`, `routePlanPreviewService`, `routePlanTopic`, `robotStateTopic`, `tfTopic`,
`tfStaticTopic`, `mowerFootprint`,
and the `defaultLayer*` layer toggles. Satellite reference imagery and
boundary-calibration traces are optional layers so alignment can be judged against the mower-local GPS outline and SLAM map instead of imagery tiles.

Build it from the repo root with:

```bash
utils/scripts/web/build_next_webui.sh
```

The build output is generated under `web/next/` and is intentionally not tracked.
