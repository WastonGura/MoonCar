import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const source=readFileSync(new URL('./app.js',import.meta.url),'utf8');
const compare=vm.runInNewContext(source.slice(source.indexOf('function compareSessions('),source.indexOf('function sessionLabel('))+';compareSessions');
const names=['experiment-a/survey-20260913T080000000001Z.npz','fixture.npz',
  'experiment-z/survey-20260913T080000000002Z.npz','experiment-0/survey-20260912T235959999999Z.npz'];
assert.deepEqual(names.map(name=>({name})).sort(compare).map(s=>s.name),[names[2],names[0],names[3],names[1]]);
assert.equal(compare({name:names[0]},{name:names[0]}),0);
for(const [previous,saved,preferred,expected] of [['','','',names[2]],[names[0],'','',names[0]],['',names[3],'',names[3]],[names[0],'',names[3],names[3]]]){
  const select={value:previous,replaceChildren(){this.value='';},add(){},onchange(){}};
  const run=vm.runInNewContext(source.slice(source.indexOf('async function sessions('),source.indexOf("$('session').onchange=async"))+';sessions',{
    api:async()=>({sessions:names.map(name=>({name}))}),$:id=>id==='session'?select:{},
    Option:function(){},sessionLabel:name=>name,compareSessions:compare,
    localStorage:{getItem:()=>saved},message:msg=>{throw Error(msg);}});
  await run(preferred||null);assert.equal(select.value,expected);
}
console.log('Session order: newest first, full microseconds, UUID-independent, undated last.');
