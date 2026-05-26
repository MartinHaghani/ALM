# Next WebUI guide

Purpose: guidance for editing the Vite + React + TypeScript source for the `/next/` mower UI.

## Source and output

- Source lives in `webui/`.
- Generated static output lands in `web/next/`.
- Build with `utils/scripts/web/build_next_webui.sh`.
- Do not hand-edit `web/next/`; change this source app and rebuild.

## Current scope

- The Map and Sensors views are read-only through rosbridge. The SLAM view may call only the passive SLAM manager's start/stop and clear services.
- It defaults to an OpenLayers GPS map using `sensor_msgs/NavSatFix` from `/hw/position/gps/fix` plus RTK/status flags from `/hw/position/gps`.
- It also visualizes Slamtec C1 `sensor_msgs/LaserScan` data and raw `sensor_msgs/Imu` output behind the Sensors view.
- It has a SLAM view for passive `slam_toolbox` output from `/slam_toolbox/map` with `/slam_toolbox/scan` scan overlay and shadow-mapper start/stop/clear controls.
- The default rosbridge URL is `ws://<current-host>:9090`.
- The default GPS fix topic is `/hw/position/gps/fix`.
- The default GPS status topic is `/hw/position/gps`.
- The default scan topic is `/hw/lidar`.
- The default IMU topic is `/hw/imu/data_raw`.

## Working rules

- Keep this UI side-by-side with the existing Flutter UI at `/`; Vite must keep `base: "/next/"`.
- Do not add mower controls, mapping controls, SLAM controls, localization authority, or navigation behavior from this slice.
- Keep `/next/config.js` runtime-overridable for ArcGIS REST or XYZ imagery, satellite max zoom, GPS topic changes, scan topic changes, and passive SLAM topic/frame changes.
- Keep dependencies compatible with the Docker build image `node:22-bookworm-slim`.
- If the nginx path or build output changes, update `docker/assets/nginx.conf`, `docs/DOCKER.md`, and `docs/RASPBERRY_PI.md`.
