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
