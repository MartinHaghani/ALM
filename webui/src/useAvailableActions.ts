import { useEffect, useMemo, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import { emptyStats, estimateHz } from "./rosStats";
import type { MowerActionInfo, SensorStats, StringMessage } from "./types";

interface UseAvailableActionsOptions {
  ros: Ros | null;
  topicName: string;
}

interface UseAvailableActionsResult {
  actions: MowerActionInfo[];
  enabledActionIds: Set<string>;
  stats: SensorStats;
}

function actionEnabled(action: MowerActionInfo): boolean {
  return action.enabled === true || action.enabled === 1;
}

function parseActions(message: StringMessage): MowerActionInfo[] {
  try {
    const parsed = JSON.parse(message.data) as unknown;
    const maybeActions =
      Array.isArray(parsed)
        ? parsed
        : parsed && typeof parsed === "object" && Array.isArray((parsed as { d?: unknown }).d)
          ? (parsed as { d: unknown[] }).d
          : [];

    return maybeActions
      .map((action): MowerActionInfo | null => {
        if (!action || typeof action !== "object") {
          return null;
        }
        const partial = action as Partial<MowerActionInfo>;
        if (typeof partial.action_id !== "string" || typeof partial.action_name !== "string") {
          return null;
        }
        return {
          action_id: partial.action_id,
          action_name: partial.action_name,
          enabled: partial.enabled === true || partial.enabled === 1,
        };
      })
      .filter((action): action is MowerActionInfo => action !== null);
  } catch {
    return [];
  }
}

export function useAvailableActions({ ros, topicName }: UseAvailableActionsOptions): UseAvailableActionsResult {
  const [actions, setActions] = useState<MowerActionInfo[]>([]);
  const [stats, setStats] = useState<SensorStats>(emptyStats);
  const arrivalsRef = useRef<number[]>([]);

  useEffect(() => {
    arrivalsRef.current = [];
    setActions([]);
    setStats(emptyStats());

    if (!ros) {
      return undefined;
    }

    const topic = new Topic<StringMessage>({
      messageType: "std_msgs/String",
      name: topicName,
      queue_length: 1,
      ros,
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
      setActions(parseActions(message));
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, topicName]);

  const enabledActionIds = useMemo(
    () => new Set(actions.filter(actionEnabled).map((action) => action.action_id)),
    [actions],
  );

  return { actions, enabledActionIds, stats };
}
