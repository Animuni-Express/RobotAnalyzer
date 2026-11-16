import { expect, test } from 'bun:test';
import { createMotion, parseProject, csv } from '../../src/state/projects';
import { builtInRobots } from '../../src/robotics/models';
import { defaultViewportSettings } from '../../src/renderer/RobotViewport';
const robot = builtInRobots[1]!;
const project = { version: 1 as const, robot, q: robot.joints.map(j => j.initial), motion: createMotion(2), viewport: defaultViewportSettings };
test('saved project round trip preserves model, state and settings', () => expect(parseProject(JSON.stringify(project))).toEqual(project));
test('invalid project data fails with actionable errors', () => {
  expect(() => parseProject('{')).toThrow('invalid JSON');
  expect(() => parseProject(JSON.stringify({ ...project, q: [100, 0] }))).toThrow('limits');
  expect(() => parseProject(JSON.stringify({ ...project, motion: { ...project.motion, endTime: 0 } }))).toThrow('trajectory');
  expect(() => parseProject(JSON.stringify({ ...project, robot: { ...robot, joints: [] } }))).toThrow();
});
test('CSV escapes quotes and preserves line endings', () => expect(csv(['name', 'value'], [['a"b', 2]])).toBe('"name","value"\r\n"a""b",2'));
