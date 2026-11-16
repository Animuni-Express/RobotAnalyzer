import { afterAll, expect, test } from 'bun:test';
import { mkdtemp, open, readFile, readdir, rename, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { api } from '../../backend/main.js';
import { builtInRobots } from '../../src/robotics/models';
import { defaultViewportSettings } from '../../src/renderer/RobotViewport';
import { createMotion, readProject, writeFile as exportFile } from '../../src/state/projects';
import type { Project } from '../../src/state/projects';

const directory = await mkdtemp(join(tmpdir(), 'robot-tinyjs-test-'));
const path = join(directory, 'project.json');
const originalTjs = Object.getOwnPropertyDescriptor(globalThis, 'tjs');
const originalWindow = Object.getOwnPropertyDescriptor(globalThis, 'window');
const runtime = {
  async open(filePath: string) {
    const file = await open(filePath, 'r');
    return {
      async stat() { const info = await file.stat(); return { size: info.size, isFile: info.isFile() }; },
      async read(buffer: Uint8Array) { return (await file.read(buffer, 0, Math.min(buffer.length, 31), null)).bytesRead; },
      close: () => file.close(),
    };
  },
  async makeTempFile(template: string) {
    const filePath = template.replace('XXXXXX', crypto.randomUUID());
    const file = await open(filePath, 'wx');
    return { path: filePath, close: () => file.close() };
  },
  writeFile, rename, remove: rm,
};
Object.defineProperty(globalThis, 'tjs', { value: runtime, configurable: true });
const app = { dialog: { openFile: async (): Promise<string | null> => path, saveFile: async (): Promise<string | null> => path } };
Object.defineProperty(globalThis, 'window', { configurable: true, value: {
  tiny: { api: { call: (method: 'openProject' | 'saveFile', params: unknown) => api[method](params, app) } },
} });

afterAll(async () => {
  if (originalTjs) Object.defineProperty(globalThis, 'tjs', originalTjs);
  else Reflect.deleteProperty(globalThis, 'tjs');
  if (originalWindow) Object.defineProperty(globalThis, 'window', originalWindow);
  else Reflect.deleteProperty(globalThis, 'window');
  await rm(directory, { recursive: true, force: true });
});

test('TinyJS saves and opens the existing project format through the frontend bridge', async () => {
  const robot = builtInRobots[0]!;
  const project: Project = { version: 1, robot, q: robot.joints.map(joint => joint.initial),
    motion: createMotion(robot.joints.length), viewport: defaultViewportSettings };
  expect(await exportFile('project.robot.json', 'json', JSON.stringify(project))).toBe(true);
  expect(await readProject(() => { throw new Error('Unexpected browser picker'); })).toEqual(project);
  expect(await exportFile('dh.csv', 'csv', 'joint,a_m\r\n1,0.5')).toBe(true);
  expect(await readFile(path, 'utf8')).toBe('joint,a_m\r\n1,0.5');
  expect(await readdir(directory)).toEqual(['project.json']);
});

test('cancelling either native picker leaves the file untouched', async () => {
  await writeFile(path, 'keep on cancel');
  const cancelled = { dialog: { openFile: async () => null, saveFile: async () => null } };
  const before = await readFile(path, 'utf8');
  expect(await api.openProject(undefined, cancelled)).toBeNull();
  expect(await api.saveFile({ name: 'project.json', extension: 'json', content: '{}' }, cancelled)).toBe(false);
  expect(await readFile(path, 'utf8')).toBe(before);
});

test('rejects oversized projects, malformed exports and invalid project JSON', async () => {
  await writeFile(path, 'x'.repeat(1024 * 1024 + 1));
  await expect(api.openProject(undefined, app)).rejects.toThrow('1 MiB');
  await expect(api.saveFile({ name: '../bad.json', extension: 'json', content: '{}' }, app)).rejects.toThrow('Invalid file export');
  await expect(api.saveFile({ name: 'bad.exe', extension: 'exe', content: '{}' }, app)).rejects.toThrow('Invalid file export');
  await writeFile(path, '{broken');
  await expect(readProject(() => {})).rejects.toThrow('invalid JSON');
});

test('a failed replacement preserves existing data and removes its temporary file', async () => {
  await writeFile(path, 'keep me');
  const originalRename = runtime.rename;
  runtime.rename = async () => { throw new Error('Write denied'); };
  try {
    await expect(api.saveFile({ name: 'project.json', extension: 'json', content: '{}' }, app)).rejects.toThrow('Write denied');
    expect(await readFile(path, 'utf8')).toBe('keep me');
    expect(await readdir(directory)).toEqual(['project.json']);
  } finally { runtime.rename = originalRename; }
});
