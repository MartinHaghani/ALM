# Open Mower Next WebUI

This is the source for the new Vite + React + TypeScript WebUI served under `/next/`.
It defaults to an OpenLayers combined map that can overlay optional satellite imagery, stored mower map JSON from
`/mower_map_service/json_map`, passive SLAM occupied cells from `/slam_toolbox/map`, optional live scan points from
`/slam_toolbox/scan`, separate GPS/fused and LIDAR/SLAM robot markers, and the shadow unified localization marker from
`/localization_fusion/pose` when that node is running. The C1 LIDAR/IMU sensor viewer remains
behind the Sensors tab. Passive SLAM controls are visualization-only and do not change mower localization, planning,
costmaps, or control.

Runtime map settings can be overridden in `/next/config.js` via `window.OPEN_MOWER_NEXT_CONFIG`.
The default basemap is York Region's 2023 10 cm orthophoto ArcGIS REST service. Supported keys include
`satelliteSourceType`, `satelliteArcGisRestUrl`, `satelliteArcGisLayers`, `satelliteArcGisFormat`,
`satelliteTileUrl`, `satelliteAttribution`, `satelliteMaxZoom`, `gpsFixTopic`, `gpsStatusTopic`, `gpsRawPoseTopic`,
`gpsFusedPoseTopic`, `mowerMapTopic`, `scanTopic`, `slamScanTopic`, `slamMapTopic`, `slamMapFrame`, `slamOdomFrame`,
`slamOriginFrame`, `slamBaseFrame`, `slamAlignmentStatusTopic`, `slamManagerStatusTopic`, `slamSetMappingService`,
`slamClearMapService`, `passiveSlamOdomStatusTopic`, `passiveSlamGyroCalibrateService`, `localizationConfidenceTopic`,
`localizationFusionPoseTopic`, `localizationFusionStatusTopic`, `localizationFusionBaseFrame`, `tfTopic`, `tfStaticTopic`,
`mowerFootprint`, and the `defaultLayer*` layer toggles. Satellite reference imagery and
boundary-calibration traces are optional layers so alignment can be judged against the mower-local GPS outline and SLAM map instead of imagery tiles.

Build it from the repo root with:

```bash
utils/scripts/web/build_next_webui.sh
```

The build output is generated under `web/next/` and is intentionally not tracked.
