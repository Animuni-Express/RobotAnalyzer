import { useMemo, useRef, useState } from 'react';
import type { PointerEvent } from 'react';

export type EngineeringSeries = {
  name: string;
  points: readonly (readonly [number, number])[];
};

type Bounds = { xMin: number; xMax: number; yMin: number; yMax: number };
const colors = ['#5eead4', '#60a5fa', '#fbbf24', '#f472b6', '#a78bfa', '#fb7185'];
const width = 800;
const height = 440;
const margin = { top: 20, right: 24, bottom: 58, left: 76 };
const plotWidth = width - margin.left - margin.right;
const plotHeight = height - margin.top - margin.bottom;

const padded = (min: number, max: number) => {
  const span = max - min || Math.max(Math.abs(min) * 0.1, 1);
  return { min: min - span * 0.05, max: max + span * 0.05 };
};
const nice = (value: number) => Number(value.toPrecision(4)).toString();

export function EngineeringPlot({ series, xLabel, yLabel, onExport }: {
  series: readonly EngineeringSeries[];
  xLabel: string;
  yLabel: string;
  onExport?: (svg: SVGSVGElement) => void;
}) {
  const svgRef = useRef<SVGSVGElement>(null);
  const dragRef = useRef<{ x: number; y: number; bounds: Bounds } | null>(null);
  const bounds = useMemo<Bounds>(() => {
    const points = series.flatMap(item => item.points).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
    if (!points.length) return { xMin: 0, xMax: 1, yMin: 0, yMax: 1 };
    const x = padded(Math.min(...points.map(([v]) => v)), Math.max(...points.map(([v]) => v)));
    const y = padded(Math.min(...points.map(([, v]) => v)), Math.max(...points.map(([, v]) => v)));
    return { xMin: x.min, xMax: x.max, yMin: y.min, yMax: y.max };
  }, [series]);
  const [view, setView] = useState<Bounds | null>(null);
  const [dragging, setDragging] = useState(false);
  const [cursor, setCursor] = useState<readonly [number, number] | null>(null);
  const range = view ?? bounds;
  const x = (value: number) => margin.left + (value - range.xMin) / (range.xMax - range.xMin) * plotWidth;
  const y = (value: number) => margin.top + (range.yMax - value) / (range.yMax - range.yMin) * plotHeight;
  const ticks = Array.from({ length: 6 }, (_, index) => index / 5);
  const pointerPosition = (event: PointerEvent<SVGSVGElement>) => {
    const rect = event.currentTarget.getBoundingClientRect();
    return { x: (event.clientX - rect.left) / rect.width * width, y: (event.clientY - rect.top) / rect.height * height };
  };
  const zoom = (factor: number) => setView(current => {
    const b = current ?? bounds;
    const cx = (b.xMin + b.xMax) / 2; const cy = (b.yMin + b.yMax) / 2;
    return { xMin: cx + (b.xMin - cx) * factor, xMax: cx + (b.xMax - cx) * factor,
      yMin: cy + (b.yMin - cy) * factor, yMax: cy + (b.yMax - cy) * factor };
  });

  return <section aria-label="Engineering plot" style={{ color: '#cbd5e1', background: '#111827', border: '1px solid #334155', borderRadius: 8, padding: 12, font: '13px system-ui, sans-serif' }}>
    <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 8 }}>
      <button type="button" aria-label="Zoom in" onClick={() => zoom(0.8)}>＋</button>
      <button type="button" aria-label="Zoom out" onClick={() => zoom(1.25)}>−</button>
      <button type="button" onClick={() => setView(null)}>Reset view</button>
      {onExport && <button type="button" onClick={() => { if (svgRef.current) onExport(svgRef.current); }}>Export</button>}
      {cursor && <output aria-live="polite">{xLabel}: {nice(cursor[0])}, {yLabel}: {nice(cursor[1])}</output>}
      <span aria-live="polite" style={{ marginLeft: cursor ? 0 : 'auto' }}>
        {series.map((item, index) => <span key={item.name} style={{ marginLeft: 12, whiteSpace: 'nowrap' }}><i style={{ display: 'inline-block', width: 10, height: 3, background: colors[index % colors.length], marginRight: 5, verticalAlign: 'middle' }} />{item.name}</span>)}
      </span>
    </div>
    <svg ref={svgRef} viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${yLabel} versus ${xLabel}`} style={{ display: 'block', width: '100%', height: 'auto', touchAction: 'none', cursor: dragging ? 'grabbing' : 'crosshair' }}
      onPointerDown={event => { if (event.button !== 0) return; setDragging(true); event.currentTarget.setPointerCapture(event.pointerId); dragRef.current = { ...pointerPosition(event), bounds: range }; }}
      onPointerMove={event => {
        const pos = pointerPosition(event);
        setCursor([range.xMin + (pos.x - margin.left) / plotWidth * (range.xMax - range.xMin), range.yMax - (pos.y - margin.top) / plotHeight * (range.yMax - range.yMin)]);
        const drag = dragRef.current;
        if (drag && event.buttons) {
          const dx = (pos.x - drag.x) / plotWidth * (drag.bounds.xMax - drag.bounds.xMin);
          const dy = (pos.y - drag.y) / plotHeight * (drag.bounds.yMax - drag.bounds.yMin);
          setView({ xMin: drag.bounds.xMin - dx, xMax: drag.bounds.xMax - dx, yMin: drag.bounds.yMin + dy, yMax: drag.bounds.yMax + dy });
        }
      }}
      onPointerUp={() => { dragRef.current = null; setDragging(false); }} onPointerCancel={() => { dragRef.current = null; setDragging(false); }} onPointerLeave={() => setCursor(null)}
      onWheel={event => { event.preventDefault(); zoom(event.deltaY < 0 ? 0.9 : 1.1); }}>
      {ticks.map((t, index) => {
        const gx = margin.left + t * plotWidth; const gy = margin.top + t * plotHeight;
        const xv = range.xMin + t * (range.xMax - range.xMin); const yv = range.yMax - t * (range.yMax - range.yMin);
        return <g key={index} fill="#94a3b8" fontSize="11">
          <path d={`M${gx} ${margin.top}V${margin.top + plotHeight} M${margin.left} ${gy}H${margin.left + plotWidth}`} stroke="#334155" strokeWidth="1" />
          <text x={gx} y={margin.top + plotHeight + 20} textAnchor="middle">{nice(xv)}</text>
          <text x={margin.left - 10} y={gy + 4} textAnchor="end">{nice(yv)}</text>
        </g>;
      })}
      <path d={`M${margin.left} ${margin.top}V${margin.top + plotHeight}H${margin.left + plotWidth}`} fill="none" stroke="#94a3b8" />
      <defs><clipPath id="plot-area"><rect x={margin.left} y={margin.top} width={plotWidth} height={plotHeight} /></clipPath></defs>
      {series.map((item, index) => <polyline clipPath="url(#plot-area)" key={item.name} points={item.points.filter(([px, py]) => Number.isFinite(px) && Number.isFinite(py)).map(([px, py]) => `${x(px)},${y(py)}`).join(' ')} fill="none" stroke={colors[index % colors.length]} strokeWidth="2" strokeLinejoin="round" />)}
      <text x={margin.left + plotWidth / 2} y={height - 10} textAnchor="middle" fill="#cbd5e1">{xLabel}</text>
      <text transform={`translate(18 ${margin.top + plotHeight / 2}) rotate(-90)`} textAnchor="middle" fill="#cbd5e1">{yLabel}</text>
    </svg>
  </section>;
}
