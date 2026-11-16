import { isTauri } from '@tauri-apps/api/core';
import { open, save } from '@tauri-apps/plugin-dialog';
import { readTextFile, stat, writeTextFile } from '@tauri-apps/plugin-fs';
import { validateRobot } from '../robotics/models';
import type { Robot } from '../robotics/models';
import type { TrajectoryKind } from '../robotics/trajectories/trajectory';
import { defaultViewportSettings } from '../renderer/RobotViewport';
import type { ViewportSettings } from '../renderer/RobotViewport';

export const trajectoryKinds: readonly TrajectoryKind[] = ['linear', 'cubic', 'quintic', 'cycloidal'];
export type MotionSettings = {
  kind: TrajectoryKind; startTime: number; endTime: number;
  initialVelocity: number[]; finalVelocity: number[]; initialAcceleration: number[]; finalAcceleration: number[];
};

export const createMotion = (count: number): MotionSettings => ({
  kind: 'quintic', startTime: 0, endTime: 3,
  initialVelocity: Array<number>(count).fill(0), finalVelocity: Array<number>(count).fill(0),
  initialAcceleration: Array<number>(count).fill(0), finalAcceleration: Array<number>(count).fill(0),
});

export type Project = { version: 1; robot: Robot; q: number[]; motion: MotionSettings; viewport: ViewportSettings };
const maxProjectSize = 1024 * 1024;
const record = (value: unknown): value is Record<string, unknown> => typeof value === 'object' && value !== null && !Array.isArray(value);
const number = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const vector = (value: unknown, count: number): value is number[] => Array.isArray(value) && value.length === count && value.every(number);

export function parseProject(text: string): Project {
  if (text.length > maxProjectSize) throw new Error('Project exceeds the 1 MiB size limit.');
  let raw: unknown;
  try { raw = JSON.parse(text); } catch { throw new Error('Project file contains invalid JSON.'); }
  if (!record(raw) || raw.version !== 1) throw new Error('Expected a Robot Analyzer project with format version 1.');
  const result = validateRobot(raw.robot);
  if (!result.success) throw new Error(result.errors.join(' '));
  const count = result.data.joints.length;
  const q = raw.q;
  if (!vector(q, count) || result.data.joints.some((joint, i) => q[i]! < joint.min || q[i]! > joint.max)) {
    throw new Error('The saved joint configuration is outside the robot’s limits.');
  }
  const motion = raw.motion;
  if (!record(motion) || !trajectoryKinds.includes(motion.kind as TrajectoryKind) || !number(motion.startTime) ||
    !number(motion.endTime) || motion.endTime <= motion.startTime || !vector(motion.initialVelocity, count) ||
    !vector(motion.finalVelocity, count) || !vector(motion.initialAcceleration, count) || !vector(motion.finalAcceleration, count)) {
    throw new Error('Project trajectory settings are invalid.');
  }
  const view = raw.viewport;
  if (!record(view) || (view.profile !== 'low' && view.profile !== 'balanced' && view.profile !== 'high')) {
    throw new Error('Project graphics profile is invalid.');
  }
  const viewport = { ...defaultViewportSettings, profile: view.profile } as ViewportSettings;
  for (const key of ['grid', 'frames', 'labels', 'arrows', 'jointAxes', 'antialias', 'shadows'] as const) {
    if (typeof view[key] !== 'boolean') throw new Error(`Invalid viewport setting ${key}.`);
    viewport[key] = view[key];
  }
  if (view.pixelRatio !== undefined) {
    if (!number(view.pixelRatio) || view.pixelRatio < 0.5 || view.pixelRatio > 2) throw new Error('Pixel ratio must be between 0.5 and 2.');
    viewport.pixelRatio = view.pixelRatio;
  }
  return { version: 1, robot: result.data, q, motion: { ...motion, kind: motion.kind as TrajectoryKind } as MotionSettings, viewport };
}

export async function readProject(pickBrowserFile: () => void): Promise<Project | undefined> {
  if (window.tiny) {
    const text = await window.tiny.api.call('openProject');
    return text === null ? undefined : parseProject(text);
  }
  if (!isTauri()) { pickBrowserFile(); return; }
  const path = await open({ multiple: false, directory: false, filters: [{ name: 'Robot project', extensions: ['json'] }] });
  if (!path) return;
  const info = await stat(path);
  if (!info.isFile || info.size > maxProjectSize) throw new Error('Select a project file no larger than 1 MiB.');
  return parseProject(await readTextFile(path));
}

export async function readBrowserProject(file: File): Promise<Project> {
  if (file.size > maxProjectSize) throw new Error('Project exceeds the 1 MiB size limit.');
  return parseProject(await file.text());
}

export async function writeFile(name: string, extension: 'csv' | 'json', content: string): Promise<boolean> {
  if (window.tiny) return window.tiny.api.call('saveFile', { name, extension, content });
  if (isTauri()) {
    const path = await save({ defaultPath: name, filters: [{ name: extension.toUpperCase(), extensions: [extension] }] });
    if (!path) return false;
    await writeTextFile(path, content);
    return true;
  }
  const url = URL.createObjectURL(new Blob([content], { type: extension === 'csv' ? 'text/csv' : 'application/json' }));
  const anchor = document.createElement('a');
  anchor.download = name;
  anchor.href = url;
  anchor.click();
  window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  return true;
}

export function csv(columns: readonly string[], rows: readonly (readonly (string | number)[])[]) {
  return [columns, ...rows].map(row => row.map(value => typeof value === 'string' ? `"${value.replaceAll('"', '""')}"` : value).join(',')).join('\r\n');
}
