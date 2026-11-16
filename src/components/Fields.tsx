import { useState } from 'react';
import type { Mat4 } from '../robotics/math';

export const format = (value: number, decimals = 4) => Number(value.toFixed(decimals)).toString();

export function NumberField({ label, value, onChange, min = -Infinity, max = Infinity, integer = false, decimals = 8 }: {
  label: string; value: number; onChange: (value: number) => void;
  min?: number; max?: number; integer?: boolean; decimals?: number;
}) {
  const [draft, setDraft] = useState(() => format(value, decimals));
  const [error, setError] = useState('');
  const [previous, setPrevious] = useState({ value, decimals });
  if (previous.value !== value || previous.decimals !== decimals) {
    setPrevious({ value, decimals }); setDraft(format(value, decimals)); setError('');
  }
  return <label className="field"><span>{label}</span><input type="text" inputMode="decimal" value={draft}
    aria-invalid={Boolean(error)} title={error || label}
    onChange={event => { setDraft(event.target.value); setError(''); }}
    onBlur={() => {
      const raw = draft.trim();
      if (raw === format(value, decimals)) return;
      const next = Number(raw);
      if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(raw) || !Number.isFinite(next) ||
        next < min || next > max || (integer && !Number.isInteger(next))) {
        setError(`Enter ${integer ? 'an integer' : 'a number'} in the range ${min} to ${max}.`);
        return;
      }
      onChange(next);
      setDraft(format(value, decimals));
    }}
    onKeyDown={event => {
      if (event.key === 'Enter') event.currentTarget.blur();
      if (event.key === 'Escape') { event.stopPropagation(); setDraft(format(value, decimals)); setError(''); }
    }} />{error && <small className="error" role="alert">{error}</small>}</label>;
}

export function Matrix({ label, values, columns = 4, precision, onError }: {
  label: string; values: Mat4 | readonly number[]; columns?: number; precision: number; onError: (message: string) => void;
}) {
  const rows = Array.from({ length: Math.ceil(values.length / columns) }, (_, row) =>
    values.slice(row * columns, (row + 1) * columns).map(value => value.toFixed(precision)));
  return <section className="matrix"><header><h3>{label}</h3><button type="button" onClick={() => {
    void navigator.clipboard.writeText(rows.map(row => row.join('\t')).join('\n')).catch(() => onError('Clipboard access is unavailable.'));
  }}>Copy</button></header><div className="table-scroll"><table aria-label={label}><tbody>{rows.map((row, index) =>
    <tr key={index}>{row.map((value, column) => <td key={column}>{value}</td>)}</tr>)}</tbody></table></div></section>;
}
