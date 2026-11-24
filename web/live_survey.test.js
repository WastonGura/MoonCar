import test from 'node:test';
import assert from 'node:assert/strict';
import {connectionLabel,liveSurveyPanel} from './live_survey.js';

test('receipt age and source timestamp remain distinct, restored data stays historical',()=>{
  const restored=connectionLabel({connection_state:'historical',accepted_frames:12,seconds_since_received:null,latest_stamp_s:18});
  assert.match(restored,/历史/);assert.match(restored,/18\.000/);assert.doesNotMatch(restored,/正在接收/);
  assert.match(connectionLabel({connection_state:'stale',accepted_frames:12,seconds_since_received:6,latest_stamp_s:18}),/停止更新/);
});

test('live updates do not replace archive view until selected; leaving blocks later live redraws',async()=>{
  const elements=new Map(), intervals=[],drawn=[];
  const previousDocument=globalThis.document,previousInterval=globalThis.setInterval;
  const get=id=>{
    if(!elements.has(id))elements.set(id,{textContent:'',hidden:false,disabled:false,dataset:{},
      setAttribute(){},removeAttribute(){},getContext(){return {fillRect(){},fillStyle:''};}});
    return elements.get(id);
  };
  globalThis.document={getElementById:get};globalThis.setInterval=fn=>intervals.push(fn);
  const original={enabled:true,session:'one',revision:1,connection_state:'receiving',accepted_frames:1,
    seconds_since_received:0,latest_stamp_s:1,map:{frames:1},
    task:{id:'<example>',source:{kind:'synthetic'},coordinates:{frame:'map'},region:{bounds_m:[0,0,1,1]},
      time:{start_s:0,end_s:10,clock:'simulation'},grid:{cell_m:1},
      targets:{height_tolerance_m:.05,slope_tolerance_deg:5,minimum_precision:.9,status:'provisional'}},
    coverage:{task_cells:1,observed_fraction:1,fitted_fraction:1,state_counts:{unobserved:0,invalid_fit:0,fitted_accuracy_unknown:1,qualified:0,error_exceeds_threshold:0}},
    quality_cells:[{ix:0,iy:0,state:'fitted_accuracy_unknown'}],accuracy:{evaluable:false},configuration:{checks:[]},
    frame_counts:{fused:1,outside_time_window:0}};
  let next=original,entered=0,left=0;
  try{
    const panel=liveSurveyPanel({api:async()=>next,onEnter:()=>entered++,onMap:data=>drawn.push(data.frames),onLeave:()=>left++});
    await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(drawn,[]);assert.match(get('live-survey-accuracy').textContent,/准确度未知/);
    assert.match(get('live-survey-task').textContent,/<example>/);
    assert.match(get('live-survey-targets').textContent,/覆盖率仅作统计，不作为通过条件/);
    assert.doesNotMatch(get('live-survey-targets').textContent,/NaN|80%/);
    get('live-survey-open').onclick();assert.equal(entered,1);assert.deepEqual(drawn,[1]);
    next={...original,revision:2,map:{frames:2},accepted_frames:2};await intervals[0]();
    assert.deepEqual(drawn,[1,2]);
    original.task.targets.minimum_coverage=.8;
    panel.leave();assert.equal(left,1);assert.equal(panel.isActive(),false);
    next={...original,revision:3,map:{frames:3},accepted_frames:3};await intervals[0]();
    assert.doesNotMatch(get('live-survey-targets').textContent,/80%/);
    assert.deepEqual(drawn,[1,2]);
    next={enabled:true,session:'one',revision:3,connection_state:'stale',accepted_frames:3,seconds_since_received:8};
    await intervals[0]();assert.match(get('live-survey-status').textContent,/停止更新/);assert.deepEqual(drawn,[1,2]);
  }finally{globalThis.document=previousDocument;globalThis.setInterval=previousInterval;}
});
