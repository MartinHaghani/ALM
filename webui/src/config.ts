export type SatelliteSourceType = "arcgis-rest" | "xyz";

export interface NextWebUiConfig {
  satelliteArcGisFormat: string;
  satelliteArcGisLayers: string;
  satelliteArcGisRestUrl: string;
  gpsFixTopic: string;
  gpsStatusTopic: string;
  satelliteAttribution: string;
  satelliteMaxZoom: number;
  satelliteSourceType: SatelliteSourceType;
  satelliteTileUrl: string;
  scanTopic: string;
  slamBaseFrame: string;
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
  gpsFixTopic: "/hw/position/gps/fix",
  gpsStatusTopic: "/hw/position/gps",
  satelliteAttribution: "Imagery &copy; The Regional Municipality of York",
  satelliteMaxZoom: 22,
  satelliteSourceType: "arcgis-rest",
  satelliteTileUrl:
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  scanTopic: "/hw/lidar",
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

export function getNextWebUiConfig(): NextWebUiConfig {
  const config = {
    ...defaultConfig,
    ...(window.OPEN_MOWER_NEXT_CONFIG ?? {}),
  };

  return {
    ...config,
    satelliteMaxZoom: Number.isFinite(config.satelliteMaxZoom)
      ? Math.max(1, Math.min(config.satelliteMaxZoom, 24))
      : defaultConfig.satelliteMaxZoom,
    satelliteSourceType: config.satelliteSourceType === "xyz" ? "xyz" : "arcgis-rest",
  };
}
