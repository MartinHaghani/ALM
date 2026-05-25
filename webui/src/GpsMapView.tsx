import "ol/ol.css";

import Feature from "ol/Feature.js";
import type { FeatureLike } from "ol/Feature.js";
import OlMap from "ol/Map.js";
import View from "ol/View.js";
import Point from "ol/geom/Point.js";
import { circular } from "ol/geom/Polygon.js";
import ImageLayer from "ol/layer/Image.js";
import TileLayer from "ol/layer/Tile.js";
import VectorLayer from "ol/layer/Vector.js";
import { fromLonLat } from "ol/proj.js";
import ImageArcGISRest from "ol/source/ImageArcGISRest.js";
import VectorSource from "ol/source/Vector.js";
import XYZ from "ol/source/XYZ.js";
import CircleStyle from "ol/style/Circle.js";
import Fill from "ol/style/Fill.js";
import Stroke from "ol/style/Stroke.js";
import Style from "ol/style/Style.js";
import { Crosshair } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef } from "react";
import type { Ros } from "roslib";

import type { NextWebUiConfig } from "./config";
import type { AbsolutePose, NavSatFix } from "./types";
import { useGpsFix } from "./useGpsFix";
import { useGpsStatus } from "./useGpsStatus";

const GPS_STALE_MS = 3000;
const DEFAULT_FOLLOW_ZOOM = 20;
const FLAG_GPS_RTK_FIXED = 2;
const FLAG_GPS_RTK_FLOAT = 4;
const FLAG_GPS_DEAD_RECKONING = 8;

type GpsStateKind = "dead" | "fixed" | "float" | "gps" | "no-fix" | "offline" | "stale" | "waiting";

interface GpsMapViewProps {
  config: NextWebUiConfig;
  connected: boolean;
  now: number;
  ros: Ros | null;
}

const antennaStyle = new Style({
  image: new CircleStyle({
    fill: new Fill({ color: "#eef9f7" }),
    radius: 7,
    stroke: new Stroke({ color: "#0f7a7a", width: 3 }),
  }),
});

const accuracyStyle = new Style({
  fill: new Fill({ color: "rgba(15, 122, 122, 0.18)" }),
  stroke: new Stroke({ color: "rgba(15, 122, 122, 0.88)", width: 2 }),
});

function gpsFeatureStyle(feature: FeatureLike): Style {
  return feature.get("kind") === "accuracy" ? accuracyStyle : antennaStyle;
}

function formatNumber(value: number, digits = 1): string {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return value.toFixed(digits);
}

function formatAge(lastMessageAt: number | null, now: number): string {
  if (!lastMessageAt) {
    return "--";
  }
  return `${formatNumber(Math.max(0, (now - lastMessageAt) / 1000), 1)} s`;
}

function validLonLat(fix: NavSatFix | null): [number, number] | null {
  if (!fix || !Number.isFinite(fix.longitude) || !Number.isFinite(fix.latitude)) {
    return null;
  }
  if (Math.abs(fix.longitude) > 180 || Math.abs(fix.latitude) > 90) {
    return null;
  }
  return [fix.longitude, fix.latitude];
}

function accuracyFromFix(fix: NavSatFix | null): number {
  if (!fix || fix.position_covariance_type === 0) {
    return Number.NaN;
  }

  const eastVariance = fix.position_covariance[0];
  const northVariance = fix.position_covariance[4];
  if (!Number.isFinite(eastVariance) || !Number.isFinite(northVariance)) {
    return Number.NaN;
  }

  return Math.sqrt(Math.max(eastVariance, northVariance, 0));
}

function accuracyMeters(status: AbsolutePose | null, fix: NavSatFix | null): number {
  if (status && Number.isFinite(status.position_accuracy) && status.position_accuracy >= 0) {
    return status.position_accuracy;
  }
  return accuracyFromFix(fix);
}

function gpsStateLabel(kind: GpsStateKind): string {
  switch (kind) {
    case "dead":
      return "Dead reckoning";
    case "fixed":
      return "RTK fixed";
    case "float":
      return "RTK float";
    case "gps":
      return "GPS fix";
    case "no-fix":
      return "No fix";
    case "offline":
      return "ROS offline";
    case "stale":
      return "Stale";
    case "waiting":
      return "Waiting";
  }
}

function gpsStateKind(
  connected: boolean,
  fix: NavSatFix | null,
  status: AbsolutePose | null,
  lastFixAt: number | null,
  now: number,
): GpsStateKind {
  if (!connected) {
    return "offline";
  }
  if (!fix || !validLonLat(fix)) {
    return "waiting";
  }
  if (!lastFixAt || now - lastFixAt > GPS_STALE_MS) {
    return "stale";
  }
  if (fix.status.status < 0) {
    return "no-fix";
  }

  const flags = status?.flags ?? 0;
  if ((flags & FLAG_GPS_RTK_FIXED) !== 0) {
    return "fixed";
  }
  if ((flags & FLAG_GPS_RTK_FLOAT) !== 0) {
    return "float";
  }
  if ((flags & FLAG_GPS_DEAD_RECKONING) !== 0) {
    return "dead";
  }
  return "gps";
}

function createSatelliteLayer(config: NextWebUiConfig) {
  if (config.satelliteSourceType === "arcgis-rest") {
    return new ImageLayer({
      source: new ImageArcGISRest({
        attributions: config.satelliteAttribution,
        params: {
          FORMAT: config.satelliteArcGisFormat,
          LAYERS: config.satelliteArcGisLayers,
        },
        ratio: 1.2,
        url: config.satelliteArcGisRestUrl,
      }),
    });
  }

  return new TileLayer({
    source: new XYZ({
      attributions: config.satelliteAttribution,
      maxZoom: config.satelliteMaxZoom,
      url: config.satelliteTileUrl,
    }),
  });
}

export function GpsMapView({ config, connected, now, ros }: GpsMapViewProps) {
  const mapElementRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<OlMap | null>(null);
  const hasCenteredOnFirstFixRef = useRef(false);
  const vectorSourceRef = useRef<VectorSource | null>(null);

  const { fix, stats: fixStats } = useGpsFix({
    ros,
    topicName: config.gpsFixTopic,
  });
  const { stats: statusStats, status } = useGpsStatus({
    ros,
    topicName: config.gpsStatusTopic,
  });

  const statusIsFresh = statusStats.lastMessageAt !== null && now - statusStats.lastMessageAt <= GPS_STALE_MS;
  const freshStatus = statusIsFresh ? status : null;
  const lonLat = validLonLat(fix);
  const accuracy = accuracyMeters(freshStatus, fix);
  const gpsState = gpsStateKind(connected, fix, freshStatus, fixStats.lastMessageAt, now);
  const followZoom = Math.min(DEFAULT_FOLLOW_ZOOM, config.satelliteMaxZoom);

  const centerMapOnLatestFix = useCallback(() => {
    const map = mapRef.current;
    if (!map || !lonLat) {
      return;
    }

    const view = map.getView();
    view.setCenter(fromLonLat(lonLat));
    if ((view.getZoom() ?? 0) < followZoom) {
      view.setZoom(followZoom);
    }
  }, [followZoom, lonLat]);

  useEffect(() => {
    if (!mapElementRef.current) {
      return undefined;
    }

    const vectorSource = new VectorSource();
    hasCenteredOnFirstFixRef.current = false;
    vectorSourceRef.current = vectorSource;

    const map = new OlMap({
      layers: [
        createSatelliteLayer(config),
        new VectorLayer({
          source: vectorSource,
          style: gpsFeatureStyle,
        }),
      ],
      target: mapElementRef.current,
      view: new View({
        center: fromLonLat([0, 0]),
        maxZoom: config.satelliteMaxZoom,
        zoom: 2,
      }),
    });

    mapRef.current = map;
    window.requestAnimationFrame(() => map.updateSize());

    return () => {
      map.setTarget(undefined);
      mapRef.current = null;
      vectorSourceRef.current = null;
    };
  }, [
    config.satelliteArcGisFormat,
    config.satelliteArcGisLayers,
    config.satelliteArcGisRestUrl,
    config.satelliteAttribution,
    config.satelliteMaxZoom,
    config.satelliteSourceType,
    config.satelliteTileUrl,
  ]);

  useEffect(() => {
    const vectorSource = vectorSourceRef.current;
    const map = mapRef.current;
    if (!vectorSource || !map) {
      return;
    }

    vectorSource.clear(true);

    if (!lonLat) {
      return;
    }

    const features: Feature[] = [];
    if (Number.isFinite(accuracy) && accuracy > 0) {
      const accuracyGeometry = circular(lonLat, accuracy, 64);
      accuracyGeometry.transform("EPSG:4326", "EPSG:3857");
      features.push(new Feature({ geometry: accuracyGeometry, kind: "accuracy" }));
    }
    features.push(new Feature({ geometry: new Point(fromLonLat(lonLat)), kind: "antenna" }));
    vectorSource.addFeatures(features);

    if (!hasCenteredOnFirstFixRef.current) {
      centerMapOnLatestFix();
      hasCenteredOnFirstFixRef.current = true;
    }
  }, [accuracy, centerMapOnLatestFix, lonLat]);

  const metricValues = useMemo(
    () => ({
      accuracy: Number.isFinite(accuracy) ? `${formatNumber(accuracy, accuracy < 1 ? 2 : 1)} m` : "--",
      age: formatAge(fixStats.lastMessageAt, now),
      fixHz: formatNumber(fixStats.hz, 1),
      latitude: fix ? formatNumber(fix.latitude, 8) : "--",
      longitude: fix ? formatNumber(fix.longitude, 8) : "--",
    }),
    [accuracy, fix, fixStats.hz, fixStats.lastMessageAt, now],
  );

  return (
    <main className="map-workspace">
      <section className="gps-map-panel">
        <div className="gps-map-header">
          <div>
            <span className="eyebrow">Live GPS antenna</span>
            <h1>Map</h1>
          </div>
          <div className={`gps-state is-${gpsState}`}>
            <Crosshair size={18} aria-hidden="true" />
            <span>{gpsStateLabel(gpsState)}</span>
          </div>
        </div>

        <div className="gps-map-shell">
          <div className="gps-map" ref={mapElementRef} />
          {!lonLat && (
            <div className="gps-empty-state">
              <strong>No GPS fix received</strong>
              <span>{config.gpsFixTopic}</span>
            </div>
          )}
          <button
            aria-label="Center map on latest GPS fix"
            className="gps-map-control-button"
            disabled={!lonLat}
            onClick={centerMapOnLatestFix}
            title="Center map on latest GPS fix"
            type="button"
          >
            <Crosshair size={20} aria-hidden="true" />
          </button>
          <div className="gps-status-strip" aria-label="GPS metrics">
            <div>
              <span>Accuracy</span>
              <strong>{metricValues.accuracy}</strong>
            </div>
            <div>
              <span>Age</span>
              <strong>{metricValues.age}</strong>
            </div>
            <div>
              <span>Lat</span>
              <strong>{metricValues.latitude}</strong>
            </div>
            <div>
              <span>Lon</span>
              <strong>{metricValues.longitude}</strong>
            </div>
            <div>
              <span>Hz</span>
              <strong>{metricValues.fixHz}</strong>
            </div>
          </div>
        </div>

        <div className="gps-topic-row">
          <span>{config.gpsFixTopic}</span>
          <span>{config.gpsStatusTopic}</span>
        </div>
      </section>
    </main>
  );
}
