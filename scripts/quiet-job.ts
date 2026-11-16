import { dlopen, ptr } from 'bun:ffi';

export function applyQuietJob(memoryLimit = 1_000_000_000n) {
  if (process.env.ROBOT_ANALYZER_IN_QUIET_JOB === '1') return;
  if (process.arch !== 'x64' && process.arch !== 'arm64') throw new Error('Quiet builds require 64-bit Windows.');
  const { symbols: win } = dlopen('kernel32.dll', {
    CreateJobObjectW: { args: ['ptr', 'ptr'], returns: 'ptr' },
    SetInformationJobObject: { args: ['ptr', 'i32', 'ptr', 'u32'], returns: 'i32' },
    AssignProcessToJobObject: { args: ['ptr', 'ptr'], returns: 'i32' },
    GetCurrentProcess: { args: [], returns: 'ptr' },
    SetPriorityClass: { args: ['ptr', 'u32'], returns: 'i32' },
    GetLastError: { args: [], returns: 'u32' },
  });
  const job = win.CreateJobObjectW(null, null);
  if (!job) throw new Error(`Could not create the quiet build job (Windows error ${win.GetLastError()}).`);
  const cpu = new Uint32Array([0x5, 1000]);
  // Windows x64/ARM64 JOBOBJECT_EXTENDED_LIMIT_INFORMATION layout.
  const limits = Buffer.alloc(144);
  limits.writeUInt32LE(0x2200, 16);
  limits.writeBigUInt64LE(memoryLimit, 120);
  const current = win.GetCurrentProcess();
  if (!win.SetPriorityClass(current, 0x4000) ||
      !win.SetInformationJobObject(job, 15, ptr(cpu), cpu.byteLength) ||
      !win.SetInformationJobObject(job, 9, ptr(limits), limits.byteLength) ||
      !win.AssignProcessToJobObject(job, current)) {
    throw new Error(`Could not apply the quiet build limits (Windows error ${win.GetLastError()}).`);
  }
  // Windows closes the non-inheritable job handle when this launcher exits.
  process.env.ROBOT_ANALYZER_IN_QUIET_JOB = '1';
}
