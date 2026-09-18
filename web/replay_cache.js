// Cache Storage keeps the complete replay on disk; only eight decoded frames stay in RAM.
// Bump the namespace when the map snapshot calculation/format changes.
const CACHE_NAME = 'mooncar-replay-v3';
const LIMIT = 512 * 1024 * 1024;

export function bufferedRanges(frames, total) {
  if (total <= 0) return [];
  const sorted = [...new Set(frames)].filter(f=>Number.isInteger(f)&&f>=0&&f<=total).sort((a,b)=>a-b);
  const ranges = [];
  for (const frame of sorted) {
    const last = ranges.at(-1);
    if (last && frame === last[1] + 1) last[1] = frame;
    else ranges.push([frame, frame]);
  }
  return ranges.map(([start,end])=>[Math.max(0,(start-.5)/total)*100,Math.min(1,(end+.5)/total)*100]);
}

export function replayDelay(rate, elapsed) {
  const speed = Number(rate);
  return Math.max(0,350/(speed>0?speed:1)-elapsed);
}

export class ReplayCache {
  constructor(session, version, storage=globalThis.caches, origin=globalThis.location?.origin) {
    this.base = `${origin}/api/map?session=${encodeURIComponent(session)}&archive_version=${version}&frame=`;
    this.version = version;
    this.storage = storage;
    this.frames = new Set();
    this.memory = new Map();
    this.pending = new Map();
    this.bytes = 0;
  }

  async open() {
    if (!this.storage) throw new Error('浏览器不支持本地回放缓存');
    this.disk = await this.storage.open(CACHE_NAME);
    // ponytail: retain one archive version per browser; add a library quota only for multi-archive use.
    for (const request of await this.disk.keys()) {
      if (!request.url.startsWith(this.base)) { await this.disk.delete(request); continue; }
      const response = await this.disk.match(request);
      if (!response) continue;
      this.frames.add(Number(request.url.slice(this.base.length)));
      this.bytes += Number(response.headers.get('Content-Length') || 0);
    }
  }

  has(frame) { return this.frames.has(Number(frame)); }

  async read(frame, signal) {
    frame = Number(frame);
    if (this.memory.has(frame)) {
      const data = this.memory.get(frame);
      this.memory.delete(frame); this.memory.set(frame, data);
      return data;
    }
    if (this.pending.has(frame)) return this.pending.get(frame);
    const task = this.load(frame, signal);
    this.pending.set(frame, task);
    try { return await task; } finally { this.pending.delete(frame); }
  }

  async load(frame, signal) {
    const url = this.base + frame;
    const saved = await this.disk.match(url);
    const response = saved || await fetch(url, {signal});
    const data = await response.clone().json();
    if (!response.ok) throw new Error(data.error || '读取回放帧失败');
    if (data.archive_updated_ns !== this.version || data.frames !== frame)
      throw new Error('采集档案已更新，请重新选择记录以准备新版本缓存');
    if (!saved) {
      const size = Number(response.headers.get('Content-Length'));
      if (!Number.isFinite(size) || size <= 0 || this.bytes + size > LIMIT)
        throw new Error('回放缓存达到 512 MB 上限，请拆分更小的采集档案');
      await this.disk.put(url, response);
      this.bytes += size;
      this.frames.add(frame);
      this.onChange?.();
    }
    this.memory.set(frame, data);
    if (this.memory.size > 8) this.memory.delete(this.memory.keys().next().value);
    return data;
  }
}
