import type { TransformStamped, Vector3 } from "./types";

export interface Transform2D {
  x: number;
  y: number;
  yaw: number;
}

export interface StampedTransform2D extends Transform2D {
  childFrame: string;
  isStatic: boolean;
  parentFrame: string;
  receivedAt: number;
  stampMs: number;
}

export type TransformMap = Map<string, StampedTransform2D>;

function normalizeFrame(frame: string): string {
  return frame.replace(/^\/+/, "");
}

export function transformKey(parentFrame: string, childFrame: string): string {
  return `${normalizeFrame(parentFrame)}->${normalizeFrame(childFrame)}`;
}

export function yawFromQuaternion({ w, x, y, z }: { w: number; x: number; y: number; z: number }): number {
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function stampToMs(transform: TransformStamped): number {
  return transform.header.stamp.secs * 1000 + transform.header.stamp.nsecs / 1_000_000;
}

export function toTransform2D(transform: TransformStamped, receivedAt: number, isStatic = false): StampedTransform2D {
  return {
    childFrame: normalizeFrame(transform.child_frame_id),
    isStatic,
    parentFrame: normalizeFrame(transform.header.frame_id),
    receivedAt,
    stampMs: stampToMs(transform),
    x: transform.transform.translation.x,
    y: transform.transform.translation.y,
    yaw: yawFromQuaternion(transform.transform.rotation),
  };
}

function transformIsFresh(transform: StampedTransform2D, nowMs?: number, maxAgeMs?: number): boolean {
  if (transform.isStatic || nowMs === undefined || maxAgeMs === undefined) {
    return true;
  }
  return nowMs - transform.receivedAt <= maxAgeMs;
}

export function composeTransform(a: Transform2D, b: Transform2D): Transform2D {
  const cos = Math.cos(a.yaw);
  const sin = Math.sin(a.yaw);

  return {
    x: a.x + cos * b.x - sin * b.y,
    y: a.y + sin * b.x + cos * b.y,
    yaw: a.yaw + b.yaw,
  };
}

export function invertTransform(transform: Transform2D): Transform2D {
  const cos = Math.cos(transform.yaw);
  const sin = Math.sin(transform.yaw);

  return {
    x: -cos * transform.x - sin * transform.y,
    y: sin * transform.x - cos * transform.y,
    yaw: -transform.yaw,
  };
}

export function applyTransform(transform: Transform2D, point: Pick<Vector3, "x" | "y">): Vector3 {
  const cos = Math.cos(transform.yaw);
  const sin = Math.sin(transform.yaw);

  return {
    x: transform.x + cos * point.x - sin * point.y,
    y: transform.y + sin * point.x + cos * point.y,
    z: 0,
  };
}

export function lookupTransform2D(
  transforms: TransformMap,
  parentFrame: string,
  childFrame: string,
  options: { maxAgeMs?: number; nowMs?: number } = {},
): Transform2D | null {
  const parent = normalizeFrame(parentFrame);
  const child = normalizeFrame(childFrame);

  if (parent === child) {
    return { x: 0, y: 0, yaw: 0 };
  }

  const direct = transforms.get(transformKey(parent, child));
  if (direct && transformIsFresh(direct, options.nowMs, options.maxAgeMs)) {
    return direct;
  }

  const queue: Array<{ frame: string; transform: Transform2D }> = [{ frame: parent, transform: { x: 0, y: 0, yaw: 0 } }];
  const visited = new Set<string>([parent]);

  while (queue.length > 0) {
    const current = queue.shift();
    if (!current) {
      break;
    }

    for (const transform of transforms.values()) {
      if (!transformIsFresh(transform, options.nowMs, options.maxAgeMs)) {
        continue;
      }

      const edges = [
        {
          frame: transform.childFrame,
          transform,
        },
        {
          frame: transform.parentFrame,
          transform: invertTransform(transform),
        },
      ];

      for (const edge of edges) {
        const isForward = edge.frame === transform.childFrame;
        const sourceFrame = isForward ? transform.parentFrame : transform.childFrame;
        if (sourceFrame !== current.frame || visited.has(edge.frame)) {
          continue;
        }

        const nextTransform = composeTransform(current.transform, edge.transform);
        if (edge.frame === child) {
          return nextTransform;
        }

        visited.add(edge.frame);
        queue.push({ frame: edge.frame, transform: nextTransform });
      }
    }
  }

  return null;
}
