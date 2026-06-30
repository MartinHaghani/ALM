import type { RoutePlan, RoutePlanPath, RoutePlanPose, RoutePlanSource } from "./types";

const ROUTE_SCHEMA = "open_mower.route_plan.v0" as const;

function finiteNumber(value: unknown): number | null {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function textValue(value: unknown, fallback = ""): string {
  return typeof value === "string" && value.trim() ? value : fallback;
}

function boolValue(value: unknown): boolean {
  return value === true || value === 1 || value === "1" || value === "true";
}

function poseFromUnknown(value: unknown, index: number): RoutePlanPose | null {
  if (!value || typeof value !== "object") {
    return null;
  }
  const record = value as Record<string, unknown>;
  const x = finiteNumber(record.x);
  const y = finiteNumber(record.y);
  const yaw = finiteNumber(record.yaw) ?? 0;
  if (x === null || y === null) {
    return null;
  }
  return {
    pose_index: finiteNumber(record.pose_index) ?? finiteNumber(record.index) ?? index,
    x,
    y,
    yaw,
  };
}

function poseWithTool(base: unknown, tool: unknown, index: number): RoutePlanPose | null {
  const pose = poseFromUnknown(base, index);
  if (!pose) {
    return null;
  }
  if (tool && typeof tool === "object") {
    const record = tool as Record<string, unknown>;
    const toolX = finiteNumber(record.x);
    const toolY = finiteNumber(record.y);
    const toolYaw = finiteNumber(record.yaw);
    if (toolX !== null && toolY !== null) {
      pose.tool_x = toolX;
      pose.tool_y = toolY;
      pose.tool_yaw = toolYaw ?? pose.yaw;
    }
  }
  return pose;
}

function routeFromPaths(options: {
  active?: boolean;
  currentPathIndex?: number;
  currentPoseIndex?: number;
  frameId?: string;
  paths: RoutePlanPath[];
  planId?: string;
  source: RoutePlanSource;
}): RoutePlan {
  return {
    active: options.active ?? false,
    current_path_index: options.currentPathIndex ?? 0,
    current_pose_index: options.currentPoseIndex ?? 0,
    frame_id: options.frameId || "map",
    paths: options.paths,
    plan_id: options.planId || `${options.source}-${Date.now()}`,
    schema: ROUTE_SCHEMA,
    source: options.source,
  };
}

export function parseLiveRoutePlanPayload(payload: string): RoutePlan | null {
  try {
    const parsed = JSON.parse(payload) as Record<string, unknown>;
    return parseLiveRoutePlan(parsed);
  } catch {
    return null;
  }
}

function parseLiveRoutePlan(parsed: Record<string, unknown>): RoutePlan | null {
  if (parsed.schema !== ROUTE_SCHEMA || !Array.isArray(parsed.paths)) {
    return null;
  }
  const frameId = textValue(parsed.frame_id, "map");
  const paths: RoutePlanPath[] = parsed.paths
    .map((pathValue, pathIndex): RoutePlanPath | null => {
      if (!pathValue || typeof pathValue !== "object") {
        return null;
      }
      const path = pathValue as Record<string, unknown>;
      const poses = Array.isArray(path.poses)
        ? path.poses.map((pose, poseIndex) => poseFromUnknown(pose, poseIndex)).filter((pose): pose is RoutePlanPose => pose !== null)
        : [];
      if (poses.length === 0) {
        return null;
      }
      return {
        frame_id: textValue(path.frame_id, frameId),
        is_outline: boolValue(path.is_outline),
        label: textValue(path.label, `Path ${pathIndex + 1}`),
        path_index: finiteNumber(path.path_index) ?? pathIndex,
        poses,
      };
    })
    .filter((path): path is RoutePlanPath => path !== null);

  return routeFromPaths({
    active: boolValue(parsed.active),
    currentPathIndex: finiteNumber(parsed.current_path_index) ?? 0,
    currentPoseIndex: finiteNumber(parsed.current_pose_index) ?? 0,
    frameId,
    paths,
    planId: textValue(parsed.plan_id, "live-route"),
    source: "mower_logic",
  });
}

function parsePlanpathCompat(parsed: Record<string, unknown>): RoutePlan | null {
  if (parsed.schema !== "open_mower.planpath_compat.v0" || !Array.isArray(parsed.paths)) {
    return null;
  }
  const frameId = textValue(parsed.frame_id, "map");
  const paths: RoutePlanPath[] = parsed.paths
    .map((pathValue, pathIndex): RoutePlanPath | null => {
      if (!pathValue || typeof pathValue !== "object") {
        return null;
      }
      const path = pathValue as Record<string, unknown>;
      const pathRecord = path.path && typeof path.path === "object" ? (path.path as Record<string, unknown>) : {};
      const poses = Array.isArray(pathRecord.poses)
        ? pathRecord.poses.map((pose, poseIndex) => poseFromUnknown(pose, poseIndex)).filter((pose): pose is RoutePlanPose => pose !== null)
        : [];
      if (poses.length === 0) {
        return null;
      }
      return {
        frame_id: textValue(pathRecord.frame_id, frameId),
        is_outline: boolValue(path.is_outline),
        label: textValue(path.label, `Path ${pathIndex + 1}`),
        path_index: pathIndex,
        poses,
      };
    })
    .filter((path): path is RoutePlanPath => path !== null);

  return routeFromPaths({
    frameId,
    paths,
    planId: textValue(parsed.profile, "planpath-import"),
    source: "planpath_compat",
  });
}

function segmentPath(
  segment: Record<string, unknown>,
  pathIndex: number,
  label: string,
  isOutline: boolean,
  frameId: string,
): RoutePlanPath | null {
  const basePoses = Array.isArray(segment.base_poses) ? segment.base_poses : [];
  const toolPoints = Array.isArray(segment.tool_points) ? segment.tool_points : [];
  const sourcePoses = basePoses.length > 0 ? basePoses : toolPoints;
  const poses = sourcePoses
    .map((basePose, poseIndex) => poseWithTool(basePose, toolPoints[poseIndex], poseIndex))
    .filter((pose): pose is RoutePlanPose => pose !== null);
  if (poses.length === 0) {
    return null;
  }
  return {
    frame_id: frameId,
    is_outline: isOutline,
    label,
    path_index: pathIndex,
    poses,
  };
}

function parseV2TaskPaths(parsed: Record<string, unknown>): RoutePlan | null {
  if (parsed.schema !== "open_mower.coverage_lab.v2_task_paths.v0" || !Array.isArray(parsed.areas)) {
    return null;
  }
  const frameId = textValue(parsed.frame_id, "map");
  const paths: RoutePlanPath[] = [];

  parsed.areas.forEach((areaValue, areaIndex) => {
    if (!areaValue || typeof areaValue !== "object") {
      return;
    }
    const area = areaValue as Record<string, unknown>;
    const outlines = Array.isArray(area.outline_paths) ? area.outline_paths : [];
    outlines.forEach((outlineValue, outlineIndex) => {
      if (!outlineValue || typeof outlineValue !== "object") {
        return;
      }
      const outline = outlineValue as Record<string, unknown>;
      const outlineId = textValue(outline.outline_path_id, `area ${areaIndex + 1} outline ${outlineIndex + 1}`);
      const segments = Array.isArray(outline.segments) ? outline.segments : [];
      segments.forEach((segmentValue, segmentIndex) => {
        if (!segmentValue || typeof segmentValue !== "object") {
          return;
        }
        const segment = segmentValue as Record<string, unknown>;
        const phase = textValue(segment.phase, `segment ${segmentIndex + 1}`);
        const path = segmentPath(segment, paths.length, `${outlineId} ${phase}`, true, frameId);
        if (path) {
          paths.push(path);
        }
      });
    });

    const taskPaths = Array.isArray(area.task_paths) ? area.task_paths : [];
    taskPaths.forEach((taskValue, taskIndex) => {
      if (!taskValue || typeof taskValue !== "object") {
        return;
      }
      const task = taskValue as Record<string, unknown>;
      const taskId = textValue(task.task_path_id, textValue(task.task_id, `area ${areaIndex + 1} task ${taskIndex + 1}`));
      const segments = Array.isArray(task.segments) ? task.segments : [];
      segments.forEach((segmentValue, segmentIndex) => {
        if (!segmentValue || typeof segmentValue !== "object") {
          return;
        }
        const segment = segmentValue as Record<string, unknown>;
        const phase = textValue(segment.phase, `segment ${segmentIndex + 1}`);
        const lane = finiteNumber(segment.lane_index);
        const laneLabel = lane === null ? "" : ` lane ${lane}`;
        const path = segmentPath(segment, paths.length, `${taskId}${laneLabel} ${phase}`, false, frameId);
        if (path) {
          paths.push(path);
        }
      });
    });
  });

  return routeFromPaths({
    frameId,
    paths,
    planId: textValue(parsed.source_map, "v2-task-paths-import"),
    source: "v2_task_paths",
  });
}

export function parseImportedRoutePlanPayload(payload: string): RoutePlan | null {
  try {
    const parsed = JSON.parse(payload) as Record<string, unknown>;
    return parsePlanpathCompat(parsed) ?? parseV2TaskPaths(parsed) ?? parseLiveRoutePlan(parsed);
  } catch {
    return null;
  }
}
