export type SatelliteSourceType = "arcgis-rest" | "xyz";
export type MowerFootprintPoint = [number, number];

export interface NextWebUiConfig {
  satelliteArcGisFormat: string;
  satelliteArcGisLayers: string;
  satelliteArcGisRestUrl: string;
  defaultLayerGpsAccuracy: boolean;
  defaultLayerCalibration: boolean;
  defaultLayerLabels: boolean;
  defaultLayerLiveScan: boolean;
  defaultLayerMowerMap: boolean;
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
  mowerFootprint: MowerFootprintPoint[];
  mowerMapTopic: string;
  passiveSlamGyroCalibrateService: string;
  passiveSlamOdomStatusTopic: string;
  satelliteAttribution: string;
  satelliteMaxZoom: number;
  satelliteSourceType: SatelliteSourceType;
  satelliteTileUrl: string;
  scanTopic: string;
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
}

const defaultConfig: NextWebUiConfig = {
  satelliteArcGisFormat: "jpg",
  satelliteArcGisLayers: "show:4",
  satelliteArcGisRestUrl:
    "https://ww3.yorkmaps.ca/arcgis/rest/services/WMS/YorkRegion_OrthosImages_2023_WMS/MapServer",
  defaultLayerGpsAccuracy: true,
  defaultLayerCalibration: false,
  defaultLayerLabels: false,
  defaultLayerLiveScan: false,
  defaultLayerMowerMap: true,
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
  mowerFootprint: [
    [0.0, 0.34],
    [0.82, 0.34],
    [0.82, -0.34],
    [0.0, -0.34],
  ],
  mowerMapTopic: "/mower_map_service/json_map",
  passiveSlamGyroCalibrateService: "/passive_slam_odom/calibrate_gyro",
  passiveSlamOdomStatusTopic: "/passive_slam_odom/status",
  satelliteAttribution: "Imagery &copy; The Regional Municipality of York",
  satelliteMaxZoom: 22,
  satelliteSourceType: "arcgis-rest",
  satelliteTileUrl:
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  scanTopic: "/hw/lidar",
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
  const config = {
    ...defaultConfig,
    ...(window.OPEN_MOWER_NEXT_CONFIG ?? {}),
  };

  return {
    ...config,
    mowerFootprint: normalizeFootprint(config.mowerFootprint),
    satelliteMaxZoom: Number.isFinite(config.satelliteMaxZoom)
      ? Math.max(1, config.satelliteMaxZoom)
      : defaultConfig.satelliteMaxZoom,
    satelliteSourceType: config.satelliteSourceType === "xyz" ? "xyz" : "arcgis-rest",
  };
}
