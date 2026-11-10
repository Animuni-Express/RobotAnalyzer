import type { Mat4 } from '../robotics/math';
import type { Vec3 } from '../robotics/models';
import type { CameraView } from './RobotViewport';

/** CPU projection keeps joint visualization usable when WebGL is unavailable. */
export function ProjectionViewport({ frames, view, selected, onSelect, trace, target }: {
  frames: readonly Mat4[]; view: CameraView; selected?: number; onSelect?: (index: number) => void; trace?: readonly Vec3[]; target?: Vec3;
}) {
  const project = ([x, y, z]: Vec3): [number, number] => {
    switch (view) {
      case 'front': return [x, -z]; case 'back': return [-x, -z];
      case 'left': return [-y, -z]; case 'right': return [y, -z];
      case 'top': return [x, -y]; case 'bottom': return [x, y];
      default: return [(x - y) * .7071, (x + y) * .4082 - z * .8165];
    }
  };
  const world = frames.map(m => [m[3], m[7], m[11]] as Vec3);
  const points = world.map(project);
  const trail = trace?.map(project);
  const goal = target && project(target);
  const all = [...points, ...(trail ?? []), ...(goal ? [goal] : [])];
  const minX = Math.min(...all.map(p => p[0])), maxX = Math.max(...all.map(p => p[0]));
  const minY = Math.min(...all.map(p => p[1])), maxY = Math.max(...all.map(p => p[1]));
  const scale = 360 / Math.max(maxX - minX, maxY - minY, .2);
  const screen = ([x,y]: [number, number]): [number, number] => [300 + (x - (minX + maxX) / 2) * scale, 240 + (y - (minY + maxY) / 2) * scale];
  const coords = points.map(screen);
  return <div className="projection-viewport"><svg viewBox="0 0 600 480" role="img" aria-label="Robot joint projection">
    <defs><pattern id="projection-grid" width="30" height="30" patternUnits="userSpaceOnUse"><path d="M30 0H0V30" fill="none" stroke="#263849" strokeWidth=".6" /></pattern></defs>
    <rect width="600" height="480" fill="url(#projection-grid)" />
    {trail && <polyline points={trail.map(p => screen(p).join(',')).join(' ')} fill="none" stroke="#f0b866" strokeWidth="2" />}
    {coords.slice(1).map((p,i) => <g key={i}><line x1={coords[i]![0]} y1={coords[i]![1]} x2={p[0]} y2={p[1]} stroke="#2f5269" strokeWidth="17" strokeLinecap="round" /><line x1={coords[i]![0]} y1={coords[i]![1]} x2={p[0]} y2={p[1]} stroke="#78bad9" strokeWidth="8" strokeLinecap="round" /></g>)}
    {coords.map((p, i) => <g key={i}><circle cx={p[0]} cy={p[1]} r={i === selected ? 11 : 8} fill={i === selected ? '#f1c378' : '#c5e5f5'} stroke="#183348" strokeWidth="3" />
      <text x={p[0] + 14} y={p[1] - 12} fill="#c8dfed" fontSize="12">{i === coords.length - 1 ? 'EE' : `J${i + 1}`}</text></g>)}
    {goal && <circle cx={screen(goal)[0]} cy={screen(goal)[1]} r="12" fill="none" stroke="#ffc77e" strokeWidth="2" />}
  </svg><p className="projection-note">Compatibility projection · WebGL is unavailable. Choose a camera view above.</p>
    <div className="projection-joints" aria-label="Select a joint">{frames.slice(1).map((_, i) => <button key={i} aria-pressed={i === selected} onClick={() => onSelect?.(i)}>J{i + 1}</button>)}</div>
  </div>;
}
