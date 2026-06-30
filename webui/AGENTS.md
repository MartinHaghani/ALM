# Next WebUI guide

Purpose: guidance for editing the Vite + React + TypeScript source for the `/next/` mower UI.

## Source and output

- Source lives in `webui/`.
- Generated static output lands in `web/next/`.
- Build with `utils/scripts/web/build_next_webui.sh`.
- Do not hand-edit `web/next/`; change this source app and rebuild.

## Current scope

- The Map view is a combined GPS/LIDAR/mower-map view. It may call the passive SLAM manager's start/stop and clear services, and it may expose the area-recording controls that mirror the existing root WebUI except record docking.
- Area-recording controls publish the same existing `xbot/action` strings through rosbridge, subscribe to the live `xbot_monitoring/actions_json` action set, publish browser manual drive to `/web_joy_vel`, and render `xbot_monitoring/map_overlay` previews. The mower-side router is responsible for forwarding the selected source to `/joy_vel`.
- The Map view can also show mowing plan progress from `xbot_monitoring/map_overlay` and a browser-sampled actual track from the live map-frame robot pose.
- Do not expose `mower_logic:area_recording/record_dock` in `/next/`; docking remains intentionally omitted during the transition.
- The Sensors view is diagnostic through rosbridge, with operational positioning and passive SLAM gyro recalibration available through configured trigger services.
- It defaults to an OpenLayers GPS map using `sensor_msgs/NavSatFix` from `/hw/position/gps/fix` plus RTK/status flags from `/hw/position/gps`.
- It also visualizes Slamtec C1 `sensor_msgs/LaserScan` data and raw `sensor_msgs/Imu` output behind the Sensors view.
- It has a SLAM view for passive `slam_toolbox` output from `/slam_toolbox/map` with `/slam_toolbox/scan` scan overlay and shadow-mapper start/stop/clear controls.
- The default rosbridge URL is `ws://<current-host>:9090`.
- The default GPS fix topic is `/hw/position/gps/fix`.
- The default GPS status topic is `/hw/position/gps`.
- The default scan topic is `/hw/lidar`.
- The default IMU topic is `/hw/imu/data_raw`.
- The default action topic is `/xbot/action`.
- The default action availability topic is `/xbot_monitoring/actions_json`.
- The default robot-state topic is `/xbot_monitoring/robot_state`.
- The default recording overlay topic is `/xbot_monitoring/map_overlay`.
- The default browser manual-drive topic is `/web_joy_vel`; `/joy_vel` is the router output.

## Working rules

- Keep this UI side-by-side with the existing Flutter UI at `/`; Vite must keep `base: "/next/"`.
- Do not add additional mower controls, localization authority, or navigation behavior beyond the scoped area-recording transition described above.
- Keep `/next/config.js` runtime-overridable for ArcGIS REST or XYZ imagery, satellite max zoom, GPS topic changes, scan topic changes, and passive SLAM topic/frame changes.
- Keep dependencies compatible with the Docker build image `node:22-bookworm-slim`.
- If the nginx path or build output changes, update `docker/assets/nginx.conf`, `docs/DOCKER.md`, and `docs/RASPBERRY_PI.md`.
