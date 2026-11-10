import { useMemo, useState } from 'react';
import { NumberField } from './Fields';
import { sampledTrajectory } from '../robotics/trajectories/trajectory';
import type { Trajectory } from '../robotics/trajectories/trajectory';
import type { Robot, Vec3 } from '../robotics/models';
import { forwardDynamics, inverseDynamics, simulateDynamics } from '../robotics/dynamics/dynamics';

export interface DynamicsPanelProps {
  robot: Robot;
  q: readonly number[];
  onTrajectory: (trajectory: Trajectory) => void;
  onRobotChange: (robot: Robot) => void;
  onExportCsv: (name: string, content: string) => void;
}

export function DynamicsPanel({ robot, q, onRobotChange, onExportCsv, onTrajectory }: DynamicsPanelProps) {
  const [mode, setMode] = useState<'inverse' | 'forward'>('inverse');
  const [position, setPosition] = useState(() => [...q]);
  const [velocity, setVelocity] = useState(() => robot.joints.map(() => 0));
  const [acceleration, setAcceleration] = useState(() => robot.joints.map(() => 0));
  const [effort, setEffort] = useState(() => robot.joints.map(() => 0));
  const [gravity, setGravity] = useState<Vec3>([0, 0, -9.80665]);
  const [previous, setPrevious] = useState({ q, robot });
  const [duration, setDuration] = useState(1);
  const [simulationError, setSimulationError] = useState('');
  if (previous.q !== q || previous.robot !== robot) {
    setPrevious({ q, robot }); setPosition([...q]);
    if (velocity.length !== robot.joints.length) {
      setVelocity(robot.joints.map(() => 0)); setAcceleration(robot.joints.map(() => 0)); setEffort(robot.joints.map(() => 0));
    }
  }
  const result = useMemo(() => mode === 'inverse'
    ? inverseDynamics(robot, { q: position, velocity, acceleration, gravity })
    : forwardDynamics(robot, { q: position, velocity, effort, gravity }),
  [mode, robot, position, velocity, acceleration, effort, gravity]);
  const output = result.success ? result.data : undefined;
  const setVector = (setter: (values: number[]) => void, values: readonly number[], index: number, value: number) => {
    setter(values.map((item, i) => i === index ? value : item));
  };
  const changeJoint = (index: number, changes: Record<string, unknown>) => onRobotChange({
    ...robot, joints: robot.joints.map((joint, i) => i === index ? { ...joint, ...changes } : joint),
  });
  const csv = (headers: string[], rows: (string | number)[][]) => [headers, ...rows]
    .map(row => row.map(value => `"${String(value).replaceAll('"', '""')}"`).join(',')).join('\r\n');

  return <section aria-label="Dynamics analysis">
    <label className="field">Analysis<select value={mode} onChange={event => setMode(event.target.value === 'forward' ? 'forward' : 'inverse')}>
      <option value="inverse">Inverse dynamics · effort from acceleration</option>
      <option value="forward">Forward dynamics · acceleration from effort</option>
    </select></label>
    <h3>State</h3>
    <div className="fields">{robot.joints.map((joint, i) => <NumberField key={`q${i}`} label={`J${i + 1} position (${joint.type === 'revolute' ? 'rad' : 'm'})`}
      value={position[i]!} min={joint.min} max={joint.max} onChange={value => setVector(setPosition, position, i, value)} />)}</div>
    <div className="fields">{velocity.map((value, i) => <NumberField key={i} label={`J${i + 1} velocity (${robot.joints[i]!.type === 'revolute' ? 'rad/s' : 'm/s'})`}
      value={value} onChange={next => setVector(setVelocity, velocity, i, next)} />)}</div>
    {mode === 'inverse' ? <div className="fields">{acceleration.map((value, i) => <NumberField key={i} label={`J${i + 1} acceleration`}
      value={value} onChange={next => setVector(setAcceleration, acceleration, i, next)} />)}</div>
      : <div className="fields">{effort.map((value, i) => <NumberField key={i} label={`J${i + 1} effort (${robot.joints[i]!.type === 'revolute' ? 'N·m' : 'N'})`}
        value={value} onChange={next => setVector(setEffort, effort, i, next)} />)}</div>}
    <h3>Gravity (m/s²)</h3><div className="fields">{gravity.map((value, i) => <NumberField key={i} label={['X', 'Y', 'Z'][i]!} value={value}
      onChange={next => {
        const updated: [number, number, number] = [...gravity];
        updated[i] = next;
        setGravity(updated);
      }} />)}</div>
    <h3>Link physical properties</h3>
    {robot.joints.map((joint, i) => <details key={i}><summary>Joint {i + 1}</summary>
      <div className="fields"><NumberField label="Mass (kg)" min={0.000001} value={joint.mass} onChange={mass => changeJoint(i, { mass })} />
        {joint.centerOfMass.map((value, axis) => <NumberField key={axis} label={`COM ${['X', 'Y', 'Z'][axis]} (m)`} value={value}
          onChange={next => changeJoint(i, { centerOfMass: joint.centerOfMass.map((item, a) => a === axis ? next : item) })} />)}
      </div><div className="fields">{joint.inertia.map((value, entry) => <NumberField key={entry} label={`I${Math.floor(entry / 3) + 1}${entry % 3 + 1} (kg·m²)`} value={value}
        onChange={next => changeJoint(i, { inertia: joint.inertia.map((item, a) => a === entry || a === (entry % 3) * 3 + Math.floor(entry / 3) ? next : item) })} />)}</div>
    </details>)}
    {mode === 'forward' && <><h3>Forward simulation</h3>
      <NumberField label="Simulation duration (s)" value={duration} min={0.01} max={10} onChange={setDuration} />
      <button onClick={() => {
        const simulation = simulateDynamics(robot, { q: position, velocity, effort, gravity, duration, timeStep: 0.01 });
        if (!simulation.success) { setSimulationError(simulation.errors.join(' ')); return; }
        setSimulationError(''); onTrajectory(sampledTrajectory(simulation.data));
      }}>Simulate and play</button><p className="muted">Constant joint effort, RK4 integration at 0.01 s. Simulation stops with an error if joint limits are exceeded. Open Graphs to inspect the motion.</p>
      {simulationError && <p className="error" role="alert">{simulationError}</p>}</>}
    <h3>Results</h3>
    {output ? <><p>{mode === 'inverse' ? 'Required joint effort' : 'Joint acceleration'}</p>
      {output.map((value, i) => <p key={i}>J{i + 1}: {value.toPrecision(6)} {mode === 'inverse' ? robot.joints[i]!.type === 'revolute' ? 'N·m' : 'N' : robot.joints[i]!.type === 'revolute' ? 'rad/s²' : 'm/s²'}</p>)}
      <button onClick={() => onExportCsv('dynamics.csv', csv(['joint', 'result'], output.map((value, i) => [i + 1, value])))}>Export dynamics CSV</button>
    </> : <p className="error" role="alert">{result.errors.join(' ')}</p>}
  </section>;
}
