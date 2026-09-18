import test from 'node:test';
import assert from 'node:assert/strict';
import {ReplayCache,bufferedRanges,replayDelay} from './replay_cache.js';

test('buffered track preserves gaps and replay speed accounts for work already spent',()=>{
  assert.deepEqual(bufferedRanges([0,1,4,5,10,4,-1,20],10).map(range=>range.map(value=>Math.round(value*1e6)/1e6)),[[0,15],[35,55],[95,100]]);
  assert.deepEqual(bufferedRanges([0,1,2],2),[[0,100]]);
  assert.deepEqual(bufferedRanges([0],0),[]);
  assert.equal(replayDelay(.5,20),680);
  assert.equal(replayDelay(1,20),330);
  assert.equal(replayDelay(2,20),155);
  assert.equal(replayDelay(4,20),67.5);
  assert.equal(replayDelay(4,100),0);
});

test('complete replay persists, stays bounded, and never reuses a different archive version', async()=>{
  const entries=new Map();
  const key=request=>typeof request==='string'?request:request.url;
  const disk={
    keys:async()=>Array.from(entries.keys(),url=>new Request(url)),
    match:async request=>entries.get(key(request))?.clone(),
    put:async(request,response)=>{entries.set(key(request),response.clone());},
    delete:async request=>entries.delete(key(request))
  };
  const storage={open:async()=>disk};
  const originalFetch=globalThis.fetch;
  let requests=0,version=101;
  globalThis.fetch=async url=>{
    requests++;
    const frame=Number(new URL(url).searchParams.get('frame'));
    const body=JSON.stringify({frames:frame,archive_updated_ns:version,points:[[frame,0,0]]});
    return new Response(body,{headers:{'Content-Length':String(body.length)}});
  };
  try{
    const first=new ReplayCache('test.npz',101,storage,'http://localhost');await first.open();
    let updates=0;first.onChange=()=>updates++;
    for(let i=0;i<12;i++) assert.equal((await first.read(i)).frames,i);
    assert.equal(first.memory.size,8);
    assert.equal(first.frames.size,12);
    assert.equal(updates,12,'background and foreground cache writes notify the progress display');
    const reopened=new ReplayCache('test.npz',101,storage,'http://localhost');await reopened.open();
    assert.equal(reopened.frames.size,12);
    for(const i of [11,0,8,3,1]) assert.deepEqual((await reopened.read(i)).points,[[i,0,0]]);
    assert.equal(requests,12,'reopening and arbitrary seeking must not fetch cached frames');
    const changed=new ReplayCache('test.npz',102,storage,'http://localhost');await changed.open();
    assert.equal(changed.frames.size,0);
    assert.equal(entries.size,0);
    await assert.rejects(changed.read(1),/档案已更新/);
    assert.equal(changed.has(1),false);
    version=102;
    await Promise.all([changed.read(1),changed.read(1)]);
    assert.equal(requests,14,'concurrent requests for one frame share the same fetch');
    changed.bytes=512*1024*1024;
    await assert.rejects(changed.read(2),/512 MB/);
    assert.equal(changed.has(2),false);
  }finally{globalThis.fetch=originalFetch;}
});
