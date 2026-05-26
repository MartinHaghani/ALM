import { useEffect, useMemo, useRef } from "react";

import { applyTransform, type Transform2D } from "./tfMath";
import type { LaserScan, OccupancyGrid } from "./types";

interface SlamMapCanvasProps {
  clampMeters: number;
  grid: OccupancyGrid | null;
  mapOpacity: number;
  scan: LaserScan | null;
  slamToBase: Transform2D | null;
  slamToLidar: Transform2D | null;
  zoom: number;
}

interface Bounds {
  maxX: number;
  maxY: number;
  minX: number;
  minY: number;
}

const FREE = [248, 250, 252] as const;
const UNKNOWN = [214, 222, 232] as const;
const OCCUPIED = [31, 41, 55] as const;

function createGridCanvas(grid: OccupancyGrid | null): HTMLCanvasElement | null {
  if (!grid || grid.info.width <= 0 || grid.info.height <= 0 || grid.data.length === 0) {
    return null;
  }

  const canvas = document.createElement("canvas");
  canvas.width = grid.info.width;
  canvas.height = grid.info.height;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return null;
  }

  const image = ctx.createImageData(grid.info.width, grid.info.height);
  for (let y = 0; y < grid.info.height; y += 1) {
    for (let x = 0; x < grid.info.width; x += 1) {
      const sourceIndex = y * grid.info.width + x;
      const targetIndex = ((grid.info.height - 1 - y) * grid.info.width + x) * 4;
      const value = grid.data[sourceIndex] ?? -1;
      const color = value < 0 ? UNKNOWN : value >= 50 ? OCCUPIED : FREE;

      image.data[targetIndex] = color[0];
      image.data[targetIndex + 1] = color[1];
      image.data[targetIndex + 2] = color[2];
      image.data[targetIndex + 3] = value < 0 ? 150 : 245;
    }
  }

  ctx.putImageData(image, 0, 0);
  return canvas;
}

function gridBounds(grid: OccupancyGrid | null): Bounds {
  if (!grid || grid.info.width <= 0 || grid.info.height <= 0) {
    return {
      maxX: 8,
      maxY: 8,
      minX: -8,
      minY: -8,
    };
  }

  const origin = grid.info.origin.position;
  return {
    maxX: origin.x + grid.info.width * grid.info.resolution,
    maxY: origin.y + grid.info.height * grid.info.resolution,
    minX: origin.x,
    minY: origin.y,
  };
}

function drawWaiting(ctx: CanvasRenderingContext2D, width: number, height: number, scan: LaserScan | null): void {
  ctx.fillStyle = "#f7f9fb";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d7dee8";
  ctx.lineWidth = 1;

  const step = 48;
  for (let x = 0; x <= width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  ctx.fillStyle = "#4f5f70";
  ctx.font = "16px Inter, system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(scan ? "Waiting for SLAM map" : "Waiting for SLAM map and scan", width / 2, height / 2);
}

function drawRobot(ctx: CanvasRenderingContext2D, x: number, y: number, yaw: number): void {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#0f7a7a";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(15, 0);
  ctx.lineTo(-10, -9);
  ctx.lineTo(-7, 0);
  ctx.lineTo(-10, 9);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

export function SlamMapCanvas({
  clampMeters,
  grid,
  mapOpacity,
  scan,
  slamToBase,
  slamToLidar,
  zoom,
}: SlamMapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const gridCanvas = useMemo(() => createGridCanvas(grid), [grid]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    let animationFrame: number | null = null;

    const draw = () => {
      animationFrame = null;
      const rect = canvas.getBoundingClientRect();
      const pixelRatio = window.devicePixelRatio || 1;
      const width = Math.max(320, Math.floor(rect.width));
      const height = Math.max(320, Math.floor(rect.height));
      const pixelWidth = Math.floor(width * pixelRatio);
      const pixelHeight = Math.floor(height * pixelRatio);

      if (canvas.width !== pixelWidth) {
        canvas.width = pixelWidth;
      }
      if (canvas.height !== pixelHeight) {
        canvas.height = pixelHeight;
      }

      ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
      ctx.clearRect(0, 0, width, height);

      const bounds = gridBounds(grid);
      const boundsWidth = Math.max(1, bounds.maxX - bounds.minX);
      const boundsHeight = Math.max(1, bounds.maxY - bounds.minY);
      const scale = Math.min(width / (boundsWidth * 1.12), height / (boundsHeight * 1.12)) * zoom;
      const centerX = (bounds.minX + bounds.maxX) / 2;
      const centerY = (bounds.minY + bounds.maxY) / 2;

      const worldToCanvas = (x: number, y: number) => ({
        x: width / 2 + (x - centerX) * scale,
        y: height / 2 - (y - centerY) * scale,
      });

      if (!grid || !gridCanvas) {
        drawWaiting(ctx, width, height, scan);
      } else {
        ctx.fillStyle = "#edf3f7";
        ctx.fillRect(0, 0, width, height);

        const origin = grid.info.origin.position;
        const topLeft = worldToCanvas(origin.x, origin.y + grid.info.height * grid.info.resolution);
        ctx.globalAlpha = mapOpacity;
        ctx.drawImage(
          gridCanvas,
          topLeft.x,
          topLeft.y,
          grid.info.width * grid.info.resolution * scale,
          grid.info.height * grid.info.resolution * scale,
        );
        ctx.globalAlpha = 1;

        ctx.strokeStyle = "rgba(79, 95, 112, 0.16)";
        ctx.lineWidth = 1;
        const meterStep = scale > 80 ? 0.5 : scale > 28 ? 1 : 2;
        const startX = Math.floor(bounds.minX / meterStep) * meterStep;
        const startY = Math.floor(bounds.minY / meterStep) * meterStep;
        for (let x = startX; x <= bounds.maxX; x += meterStep) {
          const a = worldToCanvas(x, bounds.minY);
          const b = worldToCanvas(x, bounds.maxY);
          ctx.beginPath();
          ctx.moveTo(a.x, a.y);
          ctx.lineTo(b.x, b.y);
          ctx.stroke();
        }
        for (let y = startY; y <= bounds.maxY; y += meterStep) {
          const a = worldToCanvas(bounds.minX, y);
          const b = worldToCanvas(bounds.maxX, y);
          ctx.beginPath();
          ctx.moveTo(a.x, a.y);
          ctx.lineTo(b.x, b.y);
          ctx.stroke();
        }
      }

      if (scan && slamToLidar) {
        ctx.fillStyle = "#2459a6";
        ctx.strokeStyle = "rgba(36, 89, 166, 0.18)";
        ctx.lineWidth = 1;
        const maxRange = Math.min(clampMeters, scan.range_max || clampMeters);
        const stride = Math.max(1, Math.ceil(scan.ranges.length / 720));

        for (let index = 0; index < scan.ranges.length; index += stride) {
          const range = scan.ranges[index];
          if (!Number.isFinite(range) || range < scan.range_min || range > maxRange) {
            continue;
          }

          const angle = scan.angle_min + index * scan.angle_increment;
          const point = applyTransform(slamToLidar, {
            x: Math.cos(angle) * range,
            y: Math.sin(angle) * range,
          });
          const canvasPoint = worldToCanvas(point.x, point.y);

          if (index % (stride * 48) === 0 && slamToBase) {
            const robotPoint = worldToCanvas(slamToBase.x, slamToBase.y);
            ctx.beginPath();
            ctx.moveTo(robotPoint.x, robotPoint.y);
            ctx.lineTo(canvasPoint.x, canvasPoint.y);
            ctx.stroke();
          }

          ctx.beginPath();
          ctx.arc(canvasPoint.x, canvasPoint.y, 2, 0, Math.PI * 2);
          ctx.fill();
        }
      }

      if (slamToBase) {
        const robot = worldToCanvas(slamToBase.x, slamToBase.y);
        drawRobot(ctx, robot.x, robot.y, slamToBase.yaw);
      }
    };

    const scheduleDraw = () => {
      if (animationFrame === null) {
        animationFrame = window.requestAnimationFrame(draw);
      }
    };

    scheduleDraw();
    const resizeObserver = new ResizeObserver(scheduleDraw);
    resizeObserver.observe(canvas);

    return () => {
      resizeObserver.disconnect();
      if (animationFrame !== null) {
        window.cancelAnimationFrame(animationFrame);
      }
    };
  }, [clampMeters, grid, gridCanvas, mapOpacity, scan, slamToBase, slamToLidar, zoom]);

  return <canvas ref={canvasRef} className="slam-map-canvas" aria-label="Passive SLAM map canvas" />;
}
