import { useEffect, useMemo, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { MapCatalog, MapSummary, SensorStats, StringMessage } from "./types";

interface UseMapCatalogOptions {
  ros: Ros | null;
  throttleMs?: number;
  topicName: string;
}

interface UseMapCatalogResult {
  catalog: MapCatalog | null;
  selectedMap: MapSummary | null;
  stats: SensorStats;
}

function finiteNumber(value: unknown, fallback = 0): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function parseMapSummary(value: unknown): MapSummary | null {
  if (!value || typeof value !== "object") {
    return null;
  }
  const partial = value as Partial<MapSummary>;
  if (typeof partial.id !== "string" || !partial.id) {
    return null;
  }

  return {
    area_count: finiteNumber(partial.area_count),
    bounds_valid: Boolean(partial.bounds_valid),
    center_x: finiteNumber(partial.center_x),
    center_y: finiteNumber(partial.center_y),
    created_at: typeof partial.created_at === "string" ? partial.created_at : "",
    datum_lat: finiteNumber(partial.datum_lat),
    datum_lon: finiteNumber(partial.datum_lon),
    datum_source: typeof partial.datum_source === "string" ? partial.datum_source : "",
    has_docking_station: Boolean(partial.has_docking_station),
    id: partial.id,
    map_hash: typeof partial.map_hash === "string" ? partial.map_hash : "",
    max_x: finiteNumber(partial.max_x),
    max_y: finiteNumber(partial.max_y),
    min_x: finiteNumber(partial.min_x),
    min_y: finiteNumber(partial.min_y),
    mowing_area_count: finiteNumber(partial.mowing_area_count),
    name: typeof partial.name === "string" && partial.name.trim() ? partial.name : partial.id,
    navigation_area_count: finiteNumber(partial.navigation_area_count),
    obstacle_count: finiteNumber(partial.obstacle_count),
    selected: Boolean(partial.selected),
    updated_at: typeof partial.updated_at === "string" ? partial.updated_at : "",
  };
}

function parseMapCatalog(message: StringMessage): MapCatalog | null {
  try {
    const parsed = JSON.parse(message.data) as Partial<MapCatalog>;
    if (!Array.isArray(parsed.maps)) {
      return null;
    }
    const maps = parsed.maps.map(parseMapSummary).filter((summary): summary is MapSummary => summary !== null);
    return {
      maps,
      selected_map_id: typeof parsed.selected_map_id === "string" ? parsed.selected_map_id : "",
    };
  } catch {
    return null;
  }
}

export function useMapCatalog({ ros, throttleMs = 1000, topicName }: UseMapCatalogOptions): UseMapCatalogResult {
  const [catalog, setCatalog] = useState<MapCatalog | null>(null);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setCatalog(null);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<StringMessage>({
      messageType: "std_msgs/String",
      name: topicName,
      queue_length: 1,
      ros,
      throttle_rate: throttleMs,
    });

    topic.subscribe((message: StringMessage) => {
      const now = Date.now();
      arrivalsRef.current = [...arrivalsRef.current, now].filter((stamp) => now - stamp <= 5000);
      const hz = estimateHz(arrivalsRef.current);
      setStats((current) => ({
        hz,
        lastMessageAt: now,
        messageCount: current.messageCount + 1,
      }));
      setCatalog(parseMapCatalog(message));
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, throttleMs, topicName]);

  const selectedMap = useMemo(() => {
    if (!catalog) {
      return null;
    }
    return catalog.maps.find((summary) => summary.id === catalog.selected_map_id || summary.selected) ?? null;
  }, [catalog]);

  return { catalog, selectedMap, stats };
}
