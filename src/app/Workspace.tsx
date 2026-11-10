import { lazy, Suspense, useEffect, useMemo, useRef, useState } from 'react';
import { Matrix, NumberField, format } from '../components/Fields';
import { RobotViewport, defaultViewportSettings } from '../renderer/RobotViewport';
import type { CameraView, ViewportHandle } from '../renderer/RobotViewport';
import { computeFrames, geometricJacobian, solveIK } from '../robotics/kinematics';
import { builtInRobots, createJoint, maxJoints, validateRobot } from '../robotics/models';
import type { Joint, Robot, Vec3 } from '../robotics/models';
import { createTrajectory } from '../robotics/trajectories/trajectory';
import type { Trajectory } from '../robotics/trajectories/trajectory';
import { positionOf, orientationOf, rpyFromRotation, rotationFromRPY, multiply, inverseTransform } from '../robotics/transforms';
import { createMotion, csv, readBrowserProject, readProject, trajectoryKinds, writeFile } from '../state/projects';
import type { Project } from '../state/projects';

const EngineeringPlot = lazy(() => import('../graphs/EngineeringPlot').then(module => ({ default: module.EngineeringPlot })));
const DynamicsPanel = lazy(() => import('../components/DynamicsPanel').then(module => ({ default: module.DynamicsPanel })));
const VirtualRobotPanel = lazy(() => import('../components/VirtualRobotPanel').then(module => ({ default: module.VirtualRobotPanel })));
const tabs = ['Robot', 'DH', 'FKin', 'IKin', 'Dynamics', 'Trajectory', 'Graphs', 'Virtual Robot', 'Settings'] as const;
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
  const [targetRPY, setTargetRPY] = useState<Vec3>([0, 0, 0]);
  const [solveOrientation, setSolveOrientation] = useState(false);
  const [help, setHelp] = useState(false);
  const [solutions, setSolutions] = useState<number[][]>([]);
  const [fromFrame, setFromFrame] = useState(0);
  const [toFrame, setToFrame] = useState(firstRobot.joints.length);
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
  const [cartesian, setCartesian] = useState<Trajectory>();
  const clock = useRef(0);
  const file = useRef<HTMLInputElement>(null);
  const view = useRef<ViewportHandle>(null);
  const { robot, q, motion, viewport } = project;
  const joint = robot.joints[selected]!;
  const configuration = pose ?? q;
  const fk = useMemo(() => computeFrames(robot, configuration), [robot, configuration]);
  const jointTypes = useMemo(() => robot.joints.map(item => item.type), [robot]);
  const trajectory = useMemo(() => createTrajectory({
    kind: motion.kind, startTime: motion.startTime, endTime: motion.endTime, limits: robot.joints,
    initial: robot.joints.map(item => item.initial), final: robot.joints.map(item => item.final),
    initialVelocity: motion.kind === 'cubic' || motion.kind === 'quintic' ? motion.initialVelocity : undefined,
    finalVelocity: motion.kind === 'cubic' || motion.kind === 'quintic' ? motion.finalVelocity : undefined,
    initialAcceleration: motion.kind === 'quintic' ? motion.initialAcceleration : undefined,
    finalAcceleration: motion.kind === 'quintic' ? motion.finalAcceleration : undefined,
  }), [robot, motion]);
  const playback = cartesian ?? (trajectory.success ? trajectory.data : undefined);
  const playbackStart = playback?.startTime ?? motion.startTime;
  const playbackEnd = playback?.endTime ?? motion.endTime;
  const plotSamples = useMemo(() => tab === 'Graphs' && playback
    ? Array.from({ length: 161 }, (_, i) => playback.sample(playback.startTime + (playback.endTime - playback.startTime) * i / 160))
    : [], [tab, playback]);
  const trace = useMemo(() => showTrace && playback ? Array.from({ length: tracePoints }, (_, i) => {
    const t = playback.startTime + (playback.endTime - playback.startTime) * i / (tracePoints - 1);
    return positionOf(computeFrames(robot, playback.sample(t).position).endEffector);
  }) : undefined, [showTrace, playback, tracePoints, robot]);

  useEffect(() => {
    if (!playing || !playback) return;
    let last = performance.now();
    let displayed = last;
    let frame = 0;
    const step = (now: number) => {
      clock.current += Math.min(now - last, 100) * speed / 1000;
      last = now;
      const finished = clock.current >= playbackEnd;
      if (finished) clock.current = loop ? playbackStart : playbackEnd;
      const next = playback.sample(clock.current).position;
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
  }, [playing, playback, robot, speed, loop, playbackStart, playbackEnd]);

  function stop(clearPath = true) {
    setPlaying(false);
    if (clearPath) setCartesian(undefined);
    const start = clearPath ? motion.startTime : playbackStart;
    const next = !clearPath && playback ? playback.sample(start).position : undefined;
    setPose(next);
    clock.current = start;
    setTime(start);
    view.current?.setFrames(computeFrames(robot, next ?? q).frames);
  }

  function load(next: Project) {
    stop();
    setSelected(0);
    setFromFrame(0); setToFrame(next.robot.joints.length);
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

  const saveFile = () => exportFile('project.robot.json', 'json', JSON.stringify({ ...project, q: configuration }, null, 2));
  function togglePlay() {
    if (!playback) { setMessage(trajectory.errors.join(' ')); return; }
    if (clock.current >= playbackEnd || clock.current < playbackStart) clock.current = playbackStart;
    setPlaying(!playing);
  }

  function solve(targetPosition: Vec3) {
    const result = solveIK(robot, { position: targetPosition, orientation: solveOrientation ? rotationFromRPY(...targetRPY) : undefined }, configuration);
    setSolutions(result.success ? result.data.solutions : []);
    setMessage(result.success ? `Residual ${result.data.residual.toExponential(2)} m · ${result.data.iterations} iterations` : result.errors.join(' '));
  }

  const jacobian = tab === 'FKin' ? geometricJacobian(robot, configuration) : undefined;
  const matrix = (label: string, values: readonly number[], columns = 4) =>
    <Matrix label={label} values={values} columns={columns} precision={precision} onError={setMessage} />;
  const jog = (value: number) => setConfiguration(configuration.map((previous, i) => i === selected ? value : previous));
  return <main className="workspace" onKeyDown={event => {
    if ((event.ctrlKey || event.metaKey) && ['o', 's'].includes(event.key.toLowerCase())) {
      event.preventDefault();
      void (event.key.toLowerCase() === 'o' ? openFile() : saveFile());
    } else if (event.target instanceof HTMLElement && !event.target.closest('input, select, button, textarea')) {
      if (event.key === 'Escape') stop(false);
      if (event.key.toLowerCase() === 'f') view.current?.fit();
      if (event.key.toLowerCase() === 'r') view.current?.reset();
      if (event.code === 'Space') { event.preventDefault(); togglePlay(); }
    }
  }} tabIndex={0}>
    <header className="top"><div className="brand"><span className="brand-mark" aria-hidden="true">R<span>↗</span></span><div><strong>Robot Analyzer</strong><small>ROBOTICS WORKSTATION</small></div></div><nav aria-label="Analysis tools">
      {tabs.map(item => <button key={item} aria-pressed={item === tab} onClick={() => { setPlaying(false); setTab(item); }}>{item}</button>)}
    </nav><div className="actions"><button onClick={() => void openFile()}>Open</button><button onClick={() => void saveFile()}>Save</button><button aria-expanded={help} onClick={() => setHelp(!help)}>Help</button></div></header>
    <input hidden ref={file} type="file" accept=".json" onChange={async event => {
      const picked = event.currentTarget.files?.[0];
      event.currentTarget.value = '';
      if (picked) try { load(await readBrowserProject(picked)); setMessage('Project opened.'); } catch (error) { setMessage(String(error)); }
    }} />
    {help && <section className="help" aria-label="Workspace help"><strong>Start with a robot, then explore its motion.</strong><p>Choose a model and edit its standard DH parameters. Select an analysis tool above. Angles use radians; lengths use meters. Commit number edits with Enter or by leaving the field; Escape cancels an edit.</p><p>Drag to orbit · wheel to zoom · right-drag to pan · F to fit · R to reset · Space to play/pause · Escape to stop · Ctrl/⌘ O/S to open/save. Select the viewport before using shortcuts.</p><button onClick={() => setHelp(false)}>Close help</button></section>}
    <div className={`panels ${tab === 'Graphs' ? 'graph-mode' : ''}`}>
      <aside className="panel" aria-label="Robot configuration">
        <h2>Robot configuration</h2>
        <label className="field">Example<select value="" onChange={event => { const next = builtInRobots[Number(event.target.value)]; if (next) applyRobot(next); }}>
          <option value="" disabled>Select an example</option>{builtInRobots.map((item, i) => <option key={item.name} value={i}>{item.name}</option>)}
        </select></label>
        <div className="model-summary"><strong>{robot.name}</strong><span>{robot.joints.length} DOF · Standard DH</span></div>
        <label className="field">Robot name<input type="text" maxLength={120} value={robot.name} onChange={event => {
          if (event.target.value.trim()) setProject({ ...project, robot: { ...robot, name: event.target.value } });
        }} /></label>
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
          <NumberField key={`${selected}-${key}`} label={{ a: 'a (m)', alpha: 'α (rad)', d: 'd (m)', theta: 'θ offset (rad)', min: `Minimum (${joint.type === 'revolute' ? 'rad' : 'm'})`, max: `Maximum (${joint.type === 'revolute' ? 'rad' : 'm'})`, initial: 'Initial position', final: 'Final position' }[key]} value={joint[key]} onChange={value => updateJoint({ [key]: value })} />)}
          <NumberField label={joint.type === 'revolute' ? 'q (rad)' : 'q (m)'} min={joint.min} max={joint.max} value={configuration[selected]!} onChange={jog} />
        </div>
      </aside>
      <section className="stage" tabIndex={0} aria-label="Robot visualization">
        <div className="actions camera"><select aria-label="Camera view" defaultValue="perspective" onChange={event => view.current?.setView(event.target.value as CameraView)}>
          {views.map(item => <option key={item}>{item}</option>)}</select>
          <button onClick={() => view.current?.fit()} title="Fit robot (F)">Fit</button><button onClick={() => view.current?.reset()} title="Reset camera (R)">Reset</button><span className="stage-hint">Orbit · Pan · Zoom</span><span className="stage-badge">{robot.joints.length} AXES</span></div>
        <div className="viewport-container"><RobotViewport ref={view} target={tab === 'IKin' ? target : undefined} onTargetChange={next => { setTarget(next); setSolutions([]); }} frames={fk.frames} trace={trace} settings={viewport} selectedJoint={selected} onSelectJoint={setSelected}
          jointTypes={jointTypes} dh={tab === 'DH' ? { joint: selected, angle: joint.theta + (joint.type === 'revolute' ? q[selected]! : 0),
            offset: joint.d + (joint.type === 'prismatic' ? q[selected]! : 0), length: joint.a, twist: joint.alpha, progress: dhProgress, allFrames } : undefined} /></div>
        <div className="actions playback"><button onClick={togglePlay}>{playing ? 'Pause' : 'Play'}</button><button onClick={() => stop(false)}>Stop</button>
          <input aria-label="Simulation time" type="range" min={playbackStart} max={playbackEnd} step="any" value={time} onChange={event => {
            if (!playback) return;
            setPlaying(false);
            clock.current = Number(event.target.value);
            setTime(clock.current);
            setPose(playback.sample(clock.current).position);
          }} /><output>{format(time, 2)} / {format(playbackEnd, 2)} s</output></div>
      </section>
      <aside className="panel results" aria-label="Analysis results">
        <div className="section-eyebrow">ANALYSIS</div><h2>{tab === 'Robot' ? 'End effector' : tab}</h2>
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
        {(tab === 'DH' || tab === 'FKin') && <>
          <h3>Transform between frames</h3><div className="fields">{(['From', 'To'] as const).map((label, k) => <label className="field" key={label}>{label}<select value={k === 0 ? fromFrame : toFrame} onChange={event => (k === 0 ? setFromFrame : setToFrame)(Number(event.target.value))}>
            {fk.frames.map((_, i) => <option key={i} value={i}>{i === 0 ? 'Base (0)' : i === robot.joints.length ? `End effector (${i})` : `Frame ${i}`}</option>)}
          </select></label>)}</div>
          {matrix(`Frame ${fromFrame} to ${toFrame}`, multiply(inverseTransform(fk.frames[fromFrame]!), fk.frames[toFrame]!))}
        </>}
        {jacobian?.success && matrix('Geometric Jacobian', jacobian.data.flat(), robot.joints.length)}
        {tab === 'IKin' && <>
          <div className="fields">{axes.map((axis, i) => <NumberField key={axis} label={`${axis} (m)`} value={target[i]!} onChange={value => {
            const next: [number, number, number] = [...target];
            next[i] = value;
            setTarget(next);
            setSolutions([]);
          }} />)}</div><p><label><input type="checkbox" checked={solveOrientation} onChange={event => { setSolveOrientation(event.target.checked); setSolutions([]); }} /> Include orientation</label></p>
          {solveOrientation && <div className="fields">{['Roll', 'Pitch', 'Yaw'].map((axis, i) => <NumberField key={axis} label={`${axis} (rad)`} value={targetRPY[i]!} onChange={value => {
            const next: [number, number, number] = [...targetRPY]; next[i] = value; setTargetRPY(next); setSolutions([]);
          }} />)}</div>}
          <div className="actions"><button onClick={() => { setTarget(positionOf(fk.endEffector)); setTargetRPY(rpyFromRotation(orientationOf(fk.endEffector))); setSolutions([]); }}>Use current pose</button><button className="primary" onClick={() => solve(target)}>Solve {solveOrientation ? 'pose' : 'position'}</button></div>
          <p className="muted">Enter target coordinates. The WebGL viewport also supports dragging the target. Numerical IK returns solutions near the current pose; it does not enumerate every solution for a general robot.</p>
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
          <h3>Joint boundary conditions</h3>
          {robot.joints.map((item, i) => <details key={i}><summary>J{i + 1} · {item.type === 'revolute' ? 'rad' : 'm'}</summary><div className="fields">
            {(['initial', 'final'] as const).map(key => <NumberField key={key} label={key === 'initial' ? 'Start position' : 'End position'} value={item[key]} min={item.min} max={item.max} onChange={value => {
              stop(); setProject({ ...project, robot: { ...robot, joints: robot.joints.map((j, k) => k === i ? { ...j, [key]: value } : j) } });
            }} />)}
            {(motion.kind === 'cubic' || motion.kind === 'quintic') && (['initialVelocity', 'finalVelocity', ...(motion.kind === 'quintic' ? ['initialAcceleration', 'finalAcceleration'] as const : [])] as const).map(key =>
              <NumberField key={key} label={key.replace(/([A-Z])/g, ' $1')} value={motion[key][i]!} onChange={value => { stop(); setProject({ ...project, motion: { ...motion, [key]: motion[key].map((v, k) => k === i ? value : v) } }); }} />)}
          </div></details>)}
          <button onClick={() => { stop(); setMessage('Joint trajectory selected.'); }}>Use joint trajectory</button>
          <p><label><input type="checkbox" checked={loop} onChange={event => setLoop(event.target.checked)} /> Loop</label></p>
          <p><label><input type="checkbox" checked={showTrace} onChange={event => setShowTrace(event.target.checked)} /> Show end-effector trace</label></p>
        </>}
        {tab === 'Dynamics' && <Suspense fallback={<p>Loading dynamics…</p>}><DynamicsPanel robot={robot} q={configuration}
          onRobotChange={next => { const valid = validateRobot(next); if (!valid.success) { setMessage(valid.errors.join(' ')); return; } stop(); setProject({ ...project, robot: valid.data }); }}
          onTrajectory={next => { setCartesian(next); clock.current = next.startTime; setTime(next.startTime); setPose(next.sample(next.startTime).position); setPlaying(true); }}
          onExportCsv={(name, content) => { void exportFile(name, 'csv', content); }} /></Suspense>}
        {tab === 'Virtual Robot' && <Suspense fallback={<p>Loading virtual robot…</p>}><VirtualRobotPanel robot={robot}
          configuration={configuration} onConfigurationChange={setConfiguration} onTrajectory={next => {
            setCartesian(next);
            clock.current = next.startTime;
            setTime(next.startTime);
            setPose(next.sample(next.startTime).position);
            setPlaying(true);
          }} /></Suspense>}
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
        {tab === 'Graphs' && (playback ? <>
          <label className="field">Joint quantity<select value={plotValue} onChange={event => {
            const value = event.target.value;
            if (value === 'position' || value === 'velocity' || value === 'acceleration') setPlotValue(value);
          }}><option>position</option><option>velocity</option><option>acceleration</option></select></label>
          <Suspense fallback={<p>Loading plot…</p>}><EngineeringPlot xLabel="Time (s)" key={`${robot.name}-${plotValue}-${Boolean(cartesian)}`} yLabel={`${plotValue} (${plotValue === 'position' ? 'rad / m' : plotValue === 'velocity' ? 'rad/s / m/s' : 'rad/s² / m/s²'})`}
            series={robot.joints.map((_, i) => ({ name: `J${i + 1}`, points: plotSamples.map(sample => [sample.time, sample[plotValue][i]!] as const) }))} /></Suspense>
          <button onClick={() => void exportFile('trajectory.csv', 'csv', csv(
            ['time_s', ...robot.joints.flatMap((_, i) => [`J${i + 1}_position`, `J${i + 1}_velocity`, `J${i + 1}_acceleration`])],
            plotSamples.map(sample => [sample.time, ...robot.joints.flatMap((_, i) => [sample.position[i]!, sample.velocity[i]!, sample.acceleration[i]!])]),
          ))}>Export trajectory CSV</button>
        </> : <p>{trajectory.errors.join(' ')}</p>)}
      </aside>
    </div>
    <footer><span className="status-dot" aria-hidden="true" /><span role="status">{message || 'Ready · SI units · meters · radians'}</span><span className="footer-model">{cartesian ? 'Generated motion' : `${motion.kind} trajectory`} · {viewport.profile} graphics</span></footer>
  </main>;
}
