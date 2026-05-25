import { useEffect, useMemo, useState } from "react";
import { Ros } from "roslib";

interface UseRosBridgeResult {
  connected: boolean;
  error: string | null;
  ros: Ros | null;
  url: string;
}

function defaultRosbridgeUrl(): string {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname}:9090`;
}

export function useRosBridge(): UseRosBridgeResult {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [ros, setRos] = useState<Ros | null>(null);
  const url = useMemo(defaultRosbridgeUrl, []);

  useEffect(() => {
    let active = true;
    const nextRos = new Ros({ url });
    setRos(nextRos);

    nextRos.on("connection", () => {
      if (!active) {
        return;
      }
      setConnected(true);
      setError(null);
    });

    nextRos.on("close", () => {
      if (!active) {
        return;
      }
      setConnected(false);
    });

    nextRos.on("error", (event: unknown) => {
      if (!active) {
        return;
      }
      setConnected(false);
      setError(event instanceof Error ? event.message : "rosbridge connection failed");
    });

    return () => {
      active = false;
      setConnected(false);
      setRos(null);
      nextRos.close();
    };
  }, [url]);

  return {
    connected,
    error,
    ros,
    url,
  };
}
