interface Window {
  tiny?: {
    api: {
      call(method: 'openProject'): Promise<string | null>;
      call(method: 'saveFile', params: { name: string; extension: 'csv' | 'json'; content: string }): Promise<boolean>;
    };
  };
}
