import { forwardRef, useEffect, useImperativeHandle, useRef, useState } from 'react';
import type { Mat4 } from '../robotics/math';
import type { Joint, Vec3 } from '../robotics/models';
import { createRobotScene } from './scene';
import './viewport.css';

export type CameraView = 'perspective' | 'front' | 'back' | 'left' | 'right' | 'top' | 'bottom' | 'isometric';

export interface ViewportSettings {
  profile: 'low' | 'balanced' | 'high';
  grid: boolean;
  frames: boolean;
  labels: boolean;
  arrows: boolean;
  jointAxes: boolean;
  antialias: boolean;
  shadows: boolean;
  pixelRatio?: number;
  diagnostics?: boolean;
}

export const defaultViewportSettings: ViewportSettings = {
  profile: 'balanced', grid: true, frames: false, labels: true, arrows: true,
  jointAxes: false, antialias: true, shadows: false,
};

export interface ViewportHandle {
  fit(): void;
  reset(): void;
  setView(view: CameraView): void;
  setFrames(frames: readonly Mat4[]): void;
}

export interface ViewportProps {
  frames: readonly Mat4[];
  trace?: readonly Vec3[];
  settings: ViewportSettings;
  selectedJoint?: number;
  jointTypes?: readonly Joint['type'][];
  onSelectJoint?: (joint: number) => void;
  dh?: { joint: number; angle: number; offset: number; length: number; twist: number; progress: number; allFrames: boolean };
  target?: Vec3;
  onTargetChange?: (target: Vec3) => void;
}

export const RobotViewport = forwardRef<ViewportHandle, ViewportProps>(function RobotViewport(props, ref) {
  const host = useRef<HTMLDivElement>(null);
  const labels = useRef<HTMLDivElement>(null);
  const diagnostics = useRef<HTMLDivElement>(null);
  const scene = useRef<ReturnType<typeof createRobotScene>>(null);
  const [error, setError] = useState('');
  const antialias = props.settings.profile !== 'low' && props.settings.antialias;

  useImperativeHandle(ref, () => ({
    fit: () => scene.current?.fit(),
    reset: () => scene.current?.setView('perspective'),
    setView: (view) => scene.current?.setView(view),
    setFrames: (frames) => scene.current?.setFrames(frames),
  }), []);

  useEffect(() => {
    if (!host.current || !labels.current || !diagnostics.current) return;
    try {
      scene.current = createRobotScene(host.current, labels.current, diagnostics.current, antialias);
      setError('');
    } catch (cause) {
      const detail = cause instanceof Error ? cause.message : String(cause);
      setError(`3D viewport unavailable. ${detail}`);
    }
    return () => {
      scene.current?.dispose();
      scene.current = null;
    };
  }, [antialias]);

  useEffect(() => { scene.current?.update(props); }, [props, antialias]);

  return (
    <div ref={host} className="robot-viewport">
      <div ref={labels} className="robot-viewport-labels" aria-hidden="true" />
      <div ref={diagnostics} className="robot-viewport-diagnostics" hidden />
      {error && <p className="robot-viewport-error" role="alert">{error}</p>}
    </div>
  );
});
