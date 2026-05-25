import { useEffect, useRef } from "react";

import type { LaserScan } from "./types";

interface ScanCanvasProps {
  clampMeters: number;
  scan: LaserScan | null;
}

const GRID_COLOR = "#d7dee8";
const AXIS_COLOR = "#9ca9ba";
const POINT_COLOR = "#0f7a7a";
const RAY_COLOR = "rgba(15, 122, 122, 0.16)";

function drawEmptyState(ctx: CanvasRenderingContext2D, width: number, height: number): void {
  ctx.fillStyle = "#f7f9fb";
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = "#4f5f70";
  ctx.font = "16px Inter, system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("Waiting for LaserScan", width / 2, height / 2);
}

function drawGrid(ctx: CanvasRenderingContext2D, width: number, height: number, maxRange: number): void {
  const cx = width / 2;
  const cy = height / 2;
  const radius = Math.min(width, height) * 0.44;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f7f9fb";
  ctx.fillRect(0, 0, width, height);

  ctx.strokeStyle = GRID_COLOR;
  ctx.lineWidth = 1;
  for (let ring = 1; ring <= 4; ring += 1) {
    ctx.beginPath();
    ctx.arc(cx, cy, (radius * ring) / 4, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.strokeStyle = AXIS_COLOR;
  ctx.beginPath();
  ctx.moveTo(cx - radius, cy);
  ctx.lineTo(cx + radius, cy);
  ctx.moveTo(cx, cy - radius);
  ctx.lineTo(cx, cy + radius);
  ctx.stroke();

  ctx.fillStyle = "#4f5f70";
  ctx.font = "12px Inter, system-ui, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  ctx.fillText(`${maxRange.toFixed(1)} m`, cx - 8, cy - radius);
}

function drawScan(ctx: CanvasRenderingContext2D, scan: LaserScan, width: number, height: number, clampMeters: number): void {
  const cx = width / 2;
  const cy = height / 2;
  const radius = Math.min(width, height) * 0.44;
  const maxRange = Math.min(clampMeters, scan.range_max || clampMeters);
  const scale = radius / maxRange;

  drawGrid(ctx, width, height, maxRange);

  ctx.strokeStyle = RAY_COLOR;
  ctx.lineWidth = 1;
  ctx.fillStyle = POINT_COLOR;

  for (let index = 0; index < scan.ranges.length; index += 1) {
    const rawRange = scan.ranges[index];
    if (!Number.isFinite(rawRange) || rawRange < scan.range_min || rawRange > maxRange) {
      continue;
    }

    const angle = scan.angle_min + index * scan.angle_increment;
    const x = cx + Math.cos(angle) * rawRange * scale;
    const y = cy - Math.sin(angle) * rawRange * scale;

    if (index % 24 === 0) {
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(x, y);
      ctx.stroke();
    }

    ctx.beginPath();
    ctx.arc(x, y, 2, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.fillStyle = "#17202a";
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();
}

export function ScanCanvas({ clampMeters, scan }: ScanCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

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

      if (!scan) {
        drawEmptyState(ctx, width, height);
        return;
      }

      drawScan(ctx, scan, width, height, clampMeters);
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
  }, [clampMeters, scan]);

  return <canvas ref={canvasRef} className="scan-canvas" aria-label="LIDAR scan canvas" />;
}
