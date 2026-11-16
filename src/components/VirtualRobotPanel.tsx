import { useState } from 'react';
import { NumberField } from './Fields';
import { computeFrames, forwardKinematics, solveIK } from '../robotics/kinematics';
import type { Robot, Vec3 } from '../robotics/models';
import { createCartesianTrajectory } from '../robotics/trajectories/cartesian';
import type { Trajectory } from '../robotics/trajectories/trajectory';
import { positionOf, orientationOf, rpyFromRotation, rotationFromRPY } from '../robotics/transforms';

export interface VirtualRobotPanelProps {
  robot: Robot;
  configuration: readonly number[];
  onConfigurationChange: (configuration: number[]) => void;
  onTrajectory: (trajectory: Trajectory) => void;
}

const axes = ['X', 'Y', 'Z'] as const;

export function VirtualRobotPanel({ robot, configuration, onConfigurationChange, onTrajectory }: VirtualRobotPanelProps) {
  const currentPosition = positionOf(computeFrames(robot, configuration).endEffector);
  const [target, setTarget] = useState<Vec3>(currentPosition);
  const [previousPosition, setPreviousPosition] = useState(currentPosition);
  if (currentPosition.some((v, i) => v !== previousPosition[i])) { setPreviousPosition(currentPosition); setTarget(currentPosition); }
  const [step, setStep] = useState(0.05);
  const [angleStep, setAngleStep] = useState(0.05);
  const [keepOrientation, setKeepOrientation] = useState(false);
  const [duration, setDuration] = useState(2);
  const [message, setMessage] = useState('');


  const jogCartesian = (axis: number, direction: number) => {
    const current = positionOf(computeFrames(robot, configuration).endEffector);
    const next: [number, number, number] = [...current];
    next[axis] = next[axis]! + step * direction;
    const result = solveIK(robot, { position: next, orientation: keepOrientation ? orientationOf(computeFrames(robot, configuration).endEffector) : undefined }, configuration);
    if (!result.success) { setMessage(result.errors.join(' ')); return; }
    onConfigurationChange(result.data.solutions[0]!);
    setMessage('');
  };

  const generate = () => {
    const fk = forwardKinematics(robot, configuration);
    if (!fk.success) { setMessage(fk.errors.join(' ')); return; }
    const result = createCartesianTrajectory(robot, {
      seed: configuration,
      start: { position: positionOf(fk.data.endEffector), orientation: keepOrientation ? orientationOf(fk.data.endEffector) : undefined },
      target: { position: target }, duration,
    });
    if (!result.success) { setMessage(result.errors.join(' ')); return; }
    setMessage('Cartesian trajectory generated.');
    onTrajectory(result.data);
  };

  return <section aria-label="Virtual robot jogging">
    <h2>Virtual robot</h2>
    <NumberField label="Jog step (rad for joints, m for XYZ)" value={step} min={0.000001} onChange={setStep} />
    {robot.joints.map((joint, index) => <div className="actions" key={index}>
      <span>J{index + 1}</span>
      <button aria-label={`Jog joint ${index + 1} negative`} disabled={configuration[index]! <= joint.min} onClick={() => {
        const next = [...configuration];
        next[index] = Math.max(joint.min, next[index]! - step);
        onConfigurationChange(next);
      }}>−</button>
      <output>{configuration[index]!.toFixed(3)} {joint.type === 'revolute' ? 'rad' : 'm'}</output>
      <button aria-label={`Jog joint ${index + 1} positive`} disabled={configuration[index]! >= joint.max} onClick={() => {
        const next = [...configuration];
        next[index] = Math.min(joint.max, next[index]! + step);
        onConfigurationChange(next);
      }}>+</button>
    </div>)}
    <h3>Cartesian jog</h3>
    {axes.map((axis, index) => <div className="actions" key={axis}>
      <span>{axis}</span>
      <button aria-label={`Jog ${axis} negative`} onClick={() => jogCartesian(index, -1)}>−</button>
      <button aria-label={`Jog ${axis} positive`} onClick={() => jogCartesian(index, 1)}>+</button>
    </div>)}
    <p><label><input type="checkbox" checked={keepOrientation} onChange={event => setKeepOrientation(event.target.checked)} /> Hold end-effector orientation</label></p>
    <h3>Orientation jog</h3><NumberField label="Rotation step (rad)" value={angleStep} min={0.000001} max={Math.PI} onChange={setAngleStep} />
    {['Roll', 'Pitch', 'Yaw'].map((axis, index) => <div className="actions" key={axis}><span>{axis}</span>{[-1, 1].map(direction => <button key={direction} aria-label={`Jog ${axis} ${direction < 0 ? 'negative' : 'positive'}`} onClick={() => {
      const frame = computeFrames(robot, configuration).endEffector;
      const rpy: [number, number, number] = [...rpyFromRotation(orientationOf(frame))]; rpy[index] = rpy[index]! + direction * angleStep;
      const result = solveIK(robot, { position: positionOf(frame), orientation: rotationFromRPY(...rpy) }, configuration);
      if (!result.success) { setMessage(result.errors.join(' ')); return; }
      onConfigurationChange(result.data.solutions[0]!); setMessage('');
    }}>{direction < 0 ? '−' : '+'}</button>)}</div>)}
    <h3>Straight-line trajectory</h3>
    <div className="fields">{axes.map((axis, index) => <NumberField key={axis} label={`Target ${axis} (m)`} value={target[index]!} onChange={value => {
      const next: [number, number, number] = [...target];
      next[index] = value;
      setTarget(next);
    }} />)}</div>
    <NumberField label="Duration (s)" value={duration} min={0.000001} onChange={setDuration} />
    <button onClick={generate}>Generate trajectory</button>
    {message && <p role="status">{message}</p>}
  </section>;
}
