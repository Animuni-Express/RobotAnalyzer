import { lazy, Suspense, useEffect, useMemo, useRef, useState } from 'react';
import { Matrix, NumberField, format } from '../components/Fields';
import { RobotViewport, defaultViewportSettings } from '../renderer/RobotViewport';
import type { CameraView, ViewportHandle } from '../renderer/RobotViewport';
import { computeFrames, geometricJacobian, solveIK } from '../robotics/kinematics';
import { builtInRobots, createJoint, maxJoints, validateRobot } from '../robotics/models';
import type { Joint, Robot, Vec3 } from '../robotics/models';
import { createTrajectory } from '../robotics/trajectories/trajectory';
import { positionOf, orientationOf, rpyFromRotation } from '../robotics/transforms';
import { createMotion, csv, readBrowserProject, readProject, trajectoryKinds, writeFile } from '../state/projects';
import type { Project } from '../state/projects';

const EngineeringPlot = lazy(() => import('../graphs/EngineeringPlot').then(module => ({ default: module.EngineeringPlot })));
const tabs = ['Robot', 'DH', 'FKin', 'IKin', 'Trajectory', 'Graphs', 'Settings'] as const;
const views: CameraView[] = ['perspective', 'front', 'back', 'left', 'right', 'top', 'bottom', 'isometric'];
const firstRobot = builtInRobots.find(robot => robot.joints.length === 2) ?? builtInRobots[0]!;
const axes = ['X', 'Y', 'Z'] as const;

export function Workspace() {
  const [project, setProject] = useState<Project>({
    version: 1, robot: firstRobot, q: firstRobot.joints.map(joint => joint.initial),
    motion: createMotion(firstRobot.joints.length), viewport: defaultViewportSettings,
  });
  const [tab, setTab] = useState<typeof tabs[number]>('Robot');
  const [selected, setSelected] = useState(0);
  const [precision, setPrecision] = useState(4);
  const [message, setMessage] = useState('');
  const [target, setTarget] = useState<Vec3>([0.5, 0.3, 0]);
  const [solutions, setSolutions] = useState<number[][]>([]);
  const [dhProgress, setDhProgress] = useState(1);
  const [allFrames, setAllFrames] = useState(true);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [loop, setLoop] = useState(false);
  const [showTrace, setShowTrace] = useState(false);
  const [tracePoints, setTracePoints] = useState(121);
  const [plotValue, setPlotValue] = useState<'position' | 'velocity' | 'acceleration'>('position');
  const [pose, setPose] = useState<number[]>();
  const clock = useRef(0);
  const file = useRef<HTMLInputElement>(null);
  const view = useRef<ViewportHandle>(null);
  const { robot, q, motion, viewport } = project;
  const joint = robot.joints[selected]!;
  const configuration = pose ?? q;
  const fk = useMemo(() => computeFrames(robot, configuration), [robot, configuration]);
  const jointTypes = useMemo(() => robot.joints.map(item => item.type), [robot]);
  const trajectory = useMemo(() => createTrajectory({
    kind: motion.kind, startTime: motion.startTime, endTime: motion.endTime,
    initial: robot.joints.map(item => item.initial), final: robot.joints.map(item => item.final),
    initialVelocity: motion.kind === 'cubic' || motion.kind === 'quintic' ? motion.initialVelocity : undefined,
    finalVelocity: motion.kind === 'cubic' || motion.kind === 'quintic' ? motion.finalVelocity : undefined,
    initialAcceleration: motion.kind === 'quintic' ? motion.initialAcceleration : undefined,
    finalAcceleration: motion.kind === 'quintic' ? motion.finalAcceleration : undefined,
  }), [robot, motion]);
  const plotSamples = useMemo(() => tab === 'Graphs' && trajectory.success
    ? Array.from({ length: 161 }, (_, i) => trajectory.data.sample(motion.startTime + (motion.endTime - motion.startTime) * i / 160))
    : [], [tab, trajectory, motion.startTime, motion.endTime]);
  const trace = useMemo(() => showTrace && trajectory.success ? Array.from({ length: tracePoints }, (_, i) => {
    const t = motion.startTime + (motion.endTime - motion.startTime) * i / (tracePoints - 1);
    return positionOf(computeFrames(robot, trajectory.data.sample(t).position).endEffector);
  }) : undefined, [showTrace, trajectory, tracePoints, motion.startTime, motion.endTime, robot]);

  useEffect(() => {
    if (!playing || !trajectory.success) return;
    let last = performance.now();
    let displayed = last;
    let frame = 0;
    const step = (now: number) => {
      clock.current += Math.min(now - last, 100) * speed / 1000;
      last = now;
      const finished = clock.current >= motion.endTime;
      if (finished) clock.current = loop ? motion.startTime : motion.endTime;
      const next = trajectory.data.sample(clock.current).position;
      view.current?.setFrames(computeFrames(robot, next).frames);
      if (finished || now - displayed >= 100) {
        displayed = now;
        setTime(clock.current);
        setPose(next);
      }
      if (finished && !loop) setPlaying(false);
      else frame = requestAnimationFrame(step);
    };
    frame = requestAnimationFrame(step);
    return () => cancelAnimationFrame(frame);
  }, [playing, trajectory, robot, speed, loop, motion.startTime, motion.endTime]);

  function stop() {
    setPlaying(false);
    setPose(undefined);
    clock.current = motion.startTime;
    setTime(motion.startTime);
    view.current?.setFrames(computeFrames(robot, q).frames);
  }

  function load(next: Project) {
    stop();
    setSelected(0);
    setSolutions([]);
    setProject(next);
    clock.current = next.motion.startTime;
    setTime(next.motion.startTime);
  }

  function applyRobot(next: Robot) {
    load({ ...project, robot: next, q: next.joints.map(item => item.initial), motion: createMotion(next.joints.length) });
  }

  function updateJoint(changes: Partial<Joint>) {
    const joints = robot.joints.map((item, i) => i === selected ? { ...item, ...changes } : item);
    const result = validateRobot({ ...robot, joints });
    if (!result.success) { setMessage(result.errors.join(' ')); return; }
    stop();
    setSolutions([]);
    setMessage('');
    setProject({ ...project, robot: result.data, q: q.map((value, i) => Math.max(joints[i]!.min, Math.min(joints[i]!.max, value))) });
  }

  function setConfiguration(next: number[]) {
    stop();
    setProject({ ...project, q: next });
  }

  async function openFile() {
    try {
      const next = await readProject(() => file.current?.click());
      if (next) { load(next); setMessage('Project opened.'); }
    } catch (error) { setMessage(String(error)); }
  }

  async function exportFile(name: string, extension: 'json' | 'csv', content: string) {
    try {
      if (await writeFile(name, extension, content)) setMessage('File saved.');
    } catch (error) { setMessage(String(error)); }
  }

  const saveFile = () => exportFile('project.robot.json', 'json', JSON.stringify(project, null, 2));
  function togglePlay() {
    if (!trajectory.success) { setMessage(trajectory.errors.join(' ')); return; }
    if (clock.current >= motion.endTime || clock.current < motion.startTime) clock.current = motion.startTime;
    setPlaying(!playing);
  }

  function solve(targetPosition: Vec3) {
    const result = solveIK(robot, { position: targetPosition }, configuration);
    setSolutions(result.success ? result.data.solutions : []);
    setMessage(result.success ? `Residual ${result.data.residual.toExponential(2)} m · ${result.data.iterations} iterations` : result.errors.join(' '));
  }

  const jacobian = tab === 'FKin' ? geometricJacobian(robot, configuration) : undefined;
  const matrix = (label: string, values: readonly number[], columns = 4) =>
    <Matrix label={label} values={values} columns={columns} precision={precision} onError={setMessage} />;
  const jog = (value: number) => setConfiguration(q.map((previous, i) => i === selected ? value : previous));
  return <main className="workspace" onKeyDown={event => {
    if (event.ctrlKey && ['o', 's'].includes(event.key.toLowerCase())) {
      event.preventDefault();
      void (event.key.toLowerCase() === 'o' ? openFile() : saveFile());
    } else if (event.target instanceof HTMLElement && !event.target.closest('input, select, button, textarea')) {
      if (event.key === 'Escape') stop();
      if (event.key.toLowerCase() === 'f') view.current?.fit();
      if (event.key.toLowerCase() === 'r') view.current?.reset();
      if (event.code === 'Space') { event.preventDefault(); togglePlay(); }
    }
  }} tabIndex={0}>
    <header className="top"><strong>Robot Analyzer</strong><nav aria-label="Analysis tools">
      {tabs.map(item => <button key={item} aria-pressed={item === tab} onClick={() => { stop(); setTab(item); }}>{item}</button>)}
    </nav><div className="actions"><button onClick={() => void openFile()}>Open</button><button onClick={() => void saveFile()}>Save</button></div></header>
    <input hidden ref={file} type="file" accept=".json" onChange={async event => {
      const picked = event.currentTarget.files?.[0];
      event.currentTarget.value = '';
      if (picked) try { load(await readBrowserProject(picked)); setMessage('Project opened.'); } catch (error) { setMessage(String(error)); }
    }} />
    <div className="panels">
      <aside className="panel" aria-label="Robot configuration">
        <h2>Robot configuration</h2>
        <label className="field">Example<select value="" onChange={event => { const next = builtInRobots[Number(event.target.value)]; if (next) applyRobot(next); }}>
          <option value="" disabled>Select an example</option>{builtInRobots.map((item, i) => <option key={item.name} value={i}>{item.name}</option>)}
        </select></label>
        <p>{robot.name} · {robot.joints.length} DOF</p>
        <div className="actions"><button disabled={robot.joints.length >= maxJoints} onClick={() => applyRobot({ ...robot, joints: [...robot.joints, createJoint()] })}>Add joint</button>
          <button disabled={robot.joints.length < 2} onClick={() => applyRobot({ ...robot, joints: robot.joints.filter((_, i) => i !== selected) })}>Remove joint</button></div>
        <div className="table-scroll"><table aria-label="DH parameters"><thead><tr><th>Joint</th><th>a</th><th>α</th><th>d</th><th>θ</th></tr></thead><tbody>
          {robot.joints.map((item, i) => <tr key={i}><td><button aria-pressed={selected === i} onClick={() => setSelected(i)}>J{i + 1}</button></td>
            {(['a', 'alpha', 'd', 'theta'] as const).map(key => <td key={key}>{format(item[key], precision)}</td>)}</tr>)}
        </tbody></table></div>
        <button onClick={() => void exportFile('dh.csv', 'csv', csv(['joint', 'type', 'a_m', 'alpha_rad', 'd_m', 'theta_rad'],
          robot.joints.map((item, i) => [i + 1, item.type, item.a, item.alpha, item.d, item.theta])))}>Export DH CSV</button>
        <h3>Joint {selected + 1}</h3>
        <label className="field">Type<select value={joint.type} onChange={event => updateJoint({ type: event.target.value === 'prismatic' ? 'prismatic' : 'revolute' })}>
          <option value="revolute">Revolute</option><option value="prismatic">Prismatic</option></select></label>
        <div className="fields">{(['a', 'alpha', 'd', 'theta', 'min', 'max', 'initial', 'final'] as const).map(key =>
          <NumberField key={`${selected}-${key}`} label={key} value={joint[key]} onChange={value => updateJoint({ [key]: value })} />)}
          <NumberField label={joint.type === 'revolute' ? 'q (rad)' : 'q (m)'} min={joint.min} max={joint.max} value={q[selected]!} onChange={jog} />
        </div>
      </aside>
      <section className="stage" aria-label="Robot visualization">
        <div className="actions camera"><select aria-label="Camera view" defaultValue="perspective" onChange={event => view.current?.setView(event.target.value as CameraView)}>
          {views.map(item => <option key={item}>{item}</option>)}</select>
          <button onClick={() => view.current?.fit()}>Fit</button><button onClick={() => view.current?.reset()}>Reset</button></div>
        <RobotViewport ref={view} frames={fk.frames} trace={trace} settings={viewport} selectedJoint={selected} onSelectJoint={setSelected}
          jointTypes={jointTypes} dh={tab === 'DH' ? { joint: selected, angle: joint.theta + (joint.type === 'revolute' ? q[selected]! : 0),
            offset: joint.d + (joint.type === 'prismatic' ? q[selected]! : 0), length: joint.a, twist: joint.alpha, progress: dhProgress, allFrames } : undefined} />
        <div className="actions playback"><button onClick={togglePlay}>{playing ? 'Pause' : 'Play'}</button><button onClick={stop}>Stop</button>
          <input aria-label="Simulation time" type="range" min={motion.startTime} max={motion.endTime} step="any" value={time} onChange={event => {
            if (!trajectory.success) return;
            setPlaying(false);
            clock.current = Number(event.target.value);
            setTime(clock.current);
            setPose(trajectory.data.sample(clock.current).position);
          }} /><output>{format(time, 2)} s</output></div>
      </section>
      <aside className="panel results" aria-label="Analysis results">
        <h2>{tab === 'Robot' ? 'End effector' : tab}</h2>
        {(tab === 'Robot' || tab === 'FKin' || tab === 'DH') && <>
          <p>XYZ (m) · {positionOf(fk.endEffector).map(value => format(value, precision)).join(' / ')}</p>
          <p>Roll, pitch, yaw (rad) · {rpyFromRotation(orientationOf(fk.endEffector)).map(value => format(value, precision)).join(' / ')}</p>
          {matrix('Base to end effector', fk.endEffector)}
        </>}
        {tab === 'DH' && <>
          <div className="actions"><button disabled={selected === 0} onClick={() => setSelected(selected - 1)}>Previous</button>
            <button disabled={selected + 1 === robot.joints.length} onClick={() => setSelected(selected + 1)}>Next</button></div>
          <p>Rotate about Z by θ, translate along Z by d, translate along X by a, then rotate about X by α.</p>
          <label className="field">Transformation progress<input type="range" min={0} max={1} step={0.01} value={dhProgress} onChange={event => setDhProgress(Number(event.target.value))} /></label>
          <label><input type="checkbox" checked={allFrames} onChange={event => setAllFrames(event.target.checked)} /> All frames</label>
          {matrix(`Frame ${selected} to ${selected + 1}`, fk.relative[selected]!)}
        </>}
        {jacobian?.success && matrix('Geometric Jacobian', jacobian.data.flat(), robot.joints.length)}
        {tab === 'IKin' && <>
          <div className="fields">{axes.map((axis, i) => <NumberField key={axis} label={`${axis} (m)`} value={target[i]!} onChange={value => {
            const next: [number, number, number] = [...target];
            next[i] = value;
            setTarget(next);
            setSolutions([]);
          }} />)}</div><button onClick={() => solve(target)}>Solve position</button>
          {solutions.map((solution, i) => <p key={i}><button onClick={() => setConfiguration(solution)}>Apply solution {i + 1}</button> {solution.map(value => format(value, precision)).join(', ')}</p>)}
        </>}
        {tab === 'Trajectory' && <>
          <label className="field">Profile<select value={motion.kind} onChange={event => {
            const kind = trajectoryKinds.find(item => item === event.target.value);
            if (kind) { stop(); setProject({ ...project, motion: { ...motion, kind } }); }
          }}>{trajectoryKinds.map(kind => <option key={kind}>{kind}</option>)}</select></label>
          <div className="fields">{(['startTime', 'endTime'] as const).map(key => <NumberField key={key} label={key} value={motion[key]}
            min={key === 'endTime' ? motion.startTime + 0.000001 : -Infinity} max={key === 'startTime' ? motion.endTime - 0.000001 : Infinity}
            onChange={value => { stop(); setProject({ ...project, motion: { ...motion, [key]: value } }); }} />)}
          <NumberField label="Playback speed" value={speed} min={0.1} max={4} onChange={setSpeed} />
          <NumberField label="Trace points" value={tracePoints} min={2} max={1001} integer onChange={setTracePoints} /></div>
          <p><label><input type="checkbox" checked={loop} onChange={event => setLoop(event.target.checked)} /> Loop</label></p>
          <p><label><input type="checkbox" checked={showTrace} onChange={event => setShowTrace(event.target.checked)} /> Show end-effector trace</label></p>
        </>}
        {tab === 'Settings' && <>
          <NumberField label="Matrix decimals" value={precision} min={0} max={10} integer onChange={setPrecision} />
          <label className="field">Graphics profile<select value={viewport.profile} onChange={event => {
            const profile = event.target.value;
            if (profile === 'low' || profile === 'balanced' || profile === 'high') setProject({ ...project, viewport: { ...viewport, profile } });
          }}><option>low</option><option>balanced</option><option>high</option></select></label>
          {(['grid', 'frames', 'labels', 'arrows', 'jointAxes', 'antialias', 'shadows'] as const).map(key =>
            <p key={key}><label><input type="checkbox" checked={viewport[key]} onChange={event =>
              setProject({ ...project, viewport: { ...viewport, [key]: event.target.checked } })} /> {key}</label></p>)}
        </>}
        {tab === 'Graphs' && (trajectory.success ? <>
          <label className="field">Joint quantity<select value={plotValue} onChange={event => {
            const value = event.target.value;
            if (value === 'position' || value === 'velocity' || value === 'acceleration') setPlotValue(value);
          }}><option>position</option><option>velocity</option><option>acceleration</option></select></label>
          <Suspense fallback={<p>Loading plot…</p>}><EngineeringPlot xLabel="Time (s)" yLabel={plotValue}
            series={robot.joints.map((_, i) => ({ name: `J${i + 1}`, points: plotSamples.map(sample => [sample.time, sample[plotValue][i]!] as const) }))} /></Suspense>
          <button onClick={() => void exportFile('trajectory.csv', 'csv', csv(
            ['time_s', ...robot.joints.flatMap((_, i) => [`J${i + 1}_position`, `J${i + 1}_velocity`, `J${i + 1}_acceleration`])],
            plotSamples.map(sample => [sample.time, ...robot.joints.flatMap((_, i) => [sample.position[i]!, sample.velocity[i]!, sample.acceleration[i]!])]),
          ))}>Export trajectory CSV</button>
        </> : <p>{trajectory.errors.join(' ')}</p>)}
      </aside>
    </div>
    <footer role="status">{message || 'SI units · lengths in meters · angles in radians'}</footer>
  </main>;
}
