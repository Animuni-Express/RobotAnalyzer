const maxProjectSize = 1024 * 1024;

export const api = {
  async openProject(_params, app) {
    const path = await app.dialog.openFile({ types: ['json'] });
    if (!path) return null;
    const file = await tjs.open(path, 'r');
    try {
      const info = await file.stat();
      if (!info.isFile || info.size > maxProjectSize) throw new Error('Select a project file no larger than 1 MiB.');
      const bytes = new Uint8Array(maxProjectSize + 1);
      let length = 0;
      while (length < bytes.length) {
        const count = await file.read(bytes.subarray(length));
        if (!count) break;
        length += count;
      }
      if (length > maxProjectSize) throw new Error('Project exceeds the 1 MiB size limit.');
      return new TextDecoder('utf-8', { fatal: true }).decode(bytes.subarray(0, length));
    } finally {
      await file.close();
    }
  },

  async saveFile(params, app) {
    if (!params || typeof params.name !== 'string' || !params.name || /[\\/\0]/.test(params.name) ||
        !['csv', 'json'].includes(params.extension) || typeof params.content !== 'string') {
      throw new Error('Invalid file export.');
    }
    const path = await app.dialog.saveFile({ name: params.name, types: [params.extension] });
    if (!path) return false;
    const temporary = await tjs.makeTempFile(path + '.XXXXXX');
    await temporary.close();
    try {
      await tjs.writeFile(temporary.path, params.content);
      await tjs.rename(temporary.path, path);
    } catch (error) {
      try { await tjs.remove(temporary.path); }
      catch (cleanupError) { throw new AggregateError([error, cleanupError], 'Export failed and its temporary file could not be removed.'); }
      throw error;
    }
    return true;
  },
};

export function init(app) {
  app.window('main').setMinSize(840, 560);
}
