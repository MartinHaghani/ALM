export type SatelliteSourceType = "arcgis-rest" | "xyz";
export type MowerFootprintPoint = [number, number];

export interface NextWebUiConfig {
  actionTopic: string;
  actionsTopic: string;
  bluetoothConnectService: string;
  bluetoothDisconnectService: string;
  bluetoothForgetService: string;
  bluetoothPairService: string;
  bluetoothPowerService: string;
  bluetoothScanService: string;
  bluetoothStatusTopic: string;
  batteryCriticalVoltage: number;
  batteryEmptyVoltage: number;
  batteryFullVoltage: number;
  driveVoltageMismatchWarnV: number;
  hwPowerTopic: string;
  satelliteArcGisFormat: string;
  satelliteArcGisLayers: string;
  satelliteArcGisRestUrl: string;
  defaultLayerActualTrack: boolean;
  defaultLayerGpsAccuracy: boolean;
  defaultLayerCalibration: boolean;
  defaultLayerLabels: boolean;
  defaultLayerLiveScan: boolean;
  defaultLayerMbfPath: boolean;
  defaultLayerMowerMap: boolean;
  defaultLayerPlanProgress: boolean;
  defaultLayerSatellite: boolean;
  defaultLayerSlamObstacles: boolean;
  gpsFusedPoseTopic: string;
  gpsFixTopic: string;
  gpsRawPoseTopic: string;
  gpsStatusTopic: string;
  localizationConfidenceTopic: string;
  localizationFusionBaseFrame: string;
  localizationFusionPoseTopic: string;
  localizationFusionStatusTopic: string;
  joyTopic: string;
  manualPathRecorderExportService: string;
  manualPathRecorderMarkEventService: string;
  manualPathRecorderStartService: string;
  manualPathRecorderStatusTopic: string;
  manualPathRecorderStopService: string;
  manualInputSetSourceService: string;
  manualInputStatusTopic: string;
  mapOverlayTopic: string;
  mbfControllerPlanTopic: string;
  mbfGlobalPlanTopic: string;
  mapCatalogTopic: string;
  mapCreateService: string;
  mapDeleteService: string;
  mapEditService: string;
  mapRenameService: string;
  mapSelectService: string;
  recordingEditService: string;
  areaRecordingUseFusedPoseService: string;
  areaRecordingUseFusedPoseTopic: string;
  mowerFootprint: MowerFootprintPoint[];
  mowerMapTopic: string;
  routePlanPreviewService: string;
  routePlanTopic: string;
  passiveSlamGyroCalibrateService: string;
  passiveSlamOdomStatusTopic: string;
  positioningGyroCalibrateService: string;
  satelliteAttribution: string;
  satelliteMaxZoom: number;
  satelliteSourceType: SatelliteSourceType;
  satelliteTileUrl: string;
  scanTopic: string;
  robotStateTopic: string;
  slamBaseFrame: string;
  slamAlignmentStatusTopic: string;
  slamClearMapService: string;
  slamManagerStatusTopic: string;
  slamMapFrame: string;
  slamMapTopic: string;
  slamOdomFrame: string;
  slamOriginFrame: string;
  slamScanTopic: string;
  slamSetMappingService: string;
  tfStaticTopic: string;
  tfTopic: string;
  webJoyTopic: string;
}

const defaultConfig: NextWebUiConfig = {
  actionTopic: "/xbot/action",
  actionsTopic: "/xbot_monitoring/actions_json",
  bluetoothConnectService: "/bluetooth_gamepad/connect",
  bluetoothDisconnectService: "/bluetooth_gamepad/disconnect",
  bluetoothForgetService: "/bluetooth_gamepad/forget",
  bluetoothPairService: "/bluetooth_gamepad/pair",
  bluetoothPowerService: "/bluetooth_gamepad/set_powered",
  bluetoothScanService: "/bluetooth_gamepad/set_scan_enabled",
  bluetoothStatusTopic: "/bluetooth_gamepad/status",
  batteryCriticalVoltage: 43.0,
  batteryEmptyVoltage: 45.0,
  batteryFullVoltage: 58.4,
  driveVoltageMismatchWarnV: 1.0,
  hwPowerTopic: "/hw/power",
  satelliteArcGisFormat: "jpg",
  satelliteArcGisLayers: "show:4",
  satelliteArcGisRestUrl:
    "https://ww3.yorkmaps.ca/arcgis/rest/services/WMS/YorkRegion_OrthosImages_2023_WMS/MapServer",
  defaultLayerActualTrack: true,
  defaultLayerGpsAccuracy: true,
  defaultLayerCalibration: false,
  defaultLayerLabels: false,
  defaultLayerLiveScan: false,
  defaultLayerMbfPath: true,
  defaultLayerMowerMap: true,
  defaultLayerPlanProgress: true,
  defaultLayerSatellite: false,
  defaultLayerSlamObstacles: true,
  gpsFusedPoseTopic: "/xbot_positioning/xb_pose",
  gpsFixTopic: "/hw/position/gps/fix",
  gpsRawPoseTopic: "/hw/position/gps",
  gpsStatusTopic: "/hw/position/gps",
  localizationConfidenceTopic: "/localization_confidence/status",
  localizationFusionBaseFrame: "fused_base_link",
  localizationFusionPoseTopic: "/localization_fusion/pose",
  localizationFusionStatusTopic: "/localization_fusion/status",
  joyTopic: "/joy_vel",
  manualPathRecorderExportService: "/manual_path_recorder/export",
  manualPathRecorderMarkEventService: "/manual_path_recorder/mark_event",
  manualPathRecorderStartService: "/manual_path_recorder/start",
  manualPathRecorderStatusTopic: "/manual_path_recorder/status",
  manualPathRecorderStopService: "/manual_path_recorder/stop",
  manualInputSetSourceService: "/mower_input/set_source",
  manualInputStatusTopic: "/mower_input/status",
  mapOverlayTopic: "/xbot_monitoring/map_overlay",
  mbfControllerPlanTopic: "/move_base_flex/FTCPlanner/global_plan",
  mbfGlobalPlanTopic: "/move_base_flex/GlobalPlanner/plan",
  mapCatalogTopic: "/mower_map_service/map_catalog",
  mapCreateService: "/mower_service/create_map",
  mapDeleteService: "/mower_service/delete_map",
  mapEditService: "/mower_service/apply_map_edit",
  mapRenameService: "/mower_service/rename_map",
  mapSelectService: "/mower_service/select_map",
  recordingEditService: "/mower_service/apply_recording_edit",
  areaRecordingUseFusedPoseService: "/mower_service/set_area_recording_use_fused_pose",
  areaRecordingUseFusedPoseTopic: "/area_recorder/use_fused_pose",
  mowerFootprint: [
    [0.0, 0.34],
    [0.82, 0.34],
    [0.82, -0.34],
    [0.0, -0.34],
  ],
  mowerMapTopic: "/mower_map_service/json_map",
  routePlanPreviewService: "/mower_service/preview_mowing_plan",
  routePlanTopic: "/mower_logic/route_plan_json",
  passiveSlamGyroCalibrateService: "/passive_slam_odom/calibrate_gyro",
  passiveSlamOdomStatusTopic: "/passive_slam_odom/status",
  positioningGyroCalibrateService: "/xbot_positioning/recalibrate_gyro",
  satelliteAttribution: "Imagery &copy; The Regional Municipality of York",
  satelliteMaxZoom: 22,
  satelliteSourceType: "arcgis-rest",
  satelliteTileUrl:
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  scanTopic: "/hw/lidar",
  robotStateTopic: "/xbot_monitoring/robot_state",
  slamAlignmentStatusTopic: "/slam_toolbox_alignment/status",
  slamBaseFrame: "slam_base_link",
  slamClearMapService: "/slam_toolbox_manager/clear_map",
  slamManagerStatusTopic: "/slam_toolbox_manager/status",
  slamMapFrame: "slam_map",
  slamMapTopic: "/slam_toolbox/map",
  slamOdomFrame: "slam_odom",
  slamScanTopic: "/slam_toolbox/scan",
  slamOriginFrame: "map",
  slamSetMappingService: "/slam_toolbox_manager/set_mapping_enabled",
  tfStaticTopic: "/tf_static",
  tfTopic: "/tf",
  webJoyTopic: "/web_joy_vel",
};

declare global {
  interface Window {
    OPEN_MOWER_NEXT_CONFIG?: Partial<NextWebUiConfig>;
  }
}

function normalizeFootprint(value: unknown): MowerFootprintPoint[] {
  if (!Array.isArray(value)) {
    return defaultConfig.mowerFootprint;
  }

  const points = value
    .map((point): MowerFootprintPoint | null => {
      if (!Array.isArray(point) || point.length < 2) {
        return null;
      }
      const x = Number(point[0]);
      const y = Number(point[1]);
      return Number.isFinite(x) && Number.isFinite(y) ? [x, y] : null;
    })
    .filter((point): point is MowerFootprintPoint => point !== null);

  return points.length >= 3 ? points : defaultConfig.mowerFootprint;
}

export function getNextWebUiConfig(): NextWebUiConfig {
  const overrides = window.OPEN_MOWER_NEXT_CONFIG ?? {};
  const config = {
    ...defaultConfig,
    ...overrides,
  };

  return {
    ...config,
    mowerFootprint: normalizeFootprint(config.mowerFootprint),
    satelliteMaxZoom: Number.isFinite(config.satelliteMaxZoom)
      ? Math.max(1, config.satelliteMaxZoom)
      : defaultConfig.satelliteMaxZoom,
    satelliteSourceType: config.satelliteSourceType === "xyz" ? "xyz" : "arcgis-rest",
    webJoyTopic: overrides.webJoyTopic ?? overrides.joyTopic ?? defaultConfig.webJoyTopic,
  };
}
