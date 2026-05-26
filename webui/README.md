# Open Mower Next WebUI

This is the source for the new Vite + React + TypeScript WebUI served under `/next/`.
It defaults to an OpenLayers GPS map that displays the raw GNSS antenna fix from `/hw/position/gps/fix`
with RTK status from `/hw/position/gps`, keeps the C1 LIDAR/IMU sensor viewer behind the Sensors tab,
and adds a passive SLAM view for `/slam_toolbox/scan` plus `/slam_toolbox/map` with start/stop and clear controls
for the shadow mapper only.

Runtime map settings can be overridden in `/next/config.js` via `window.OPEN_MOWER_NEXT_CONFIG`.
The default basemap is York Region's 2023 10 cm orthophoto ArcGIS REST service. Supported keys include
`satelliteSourceType`, `satelliteArcGisRestUrl`, `satelliteArcGisLayers`, `satelliteArcGisFormat`,
`satelliteTileUrl`, `satelliteAttribution`, `satelliteMaxZoom`, `gpsFixTopic`, `gpsStatusTopic`,
`scanTopic`, `slamScanTopic`, `slamMapTopic`, `slamMapFrame`, `slamOdomFrame`, `slamOriginFrame`, `slamBaseFrame`,
`slamManagerStatusTopic`, `slamSetMappingService`, `slamClearMapService`, `tfTopic`, and `tfStaticTopic`.

Build it from the repo root with:

```bash
utils/scripts/web/build_next_webui.sh
```

The build output is generated under `web/next/` and is intentionally not tracked.
