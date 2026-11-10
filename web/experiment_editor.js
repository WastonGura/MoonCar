import * as THREE from 'three';

export function experimentEditor({scene,camera,controls,renderer,api,message,stopPlayback,onSession}) {
  const $=id=>document.getElementById(id), fields=['region-x0','region-y0','region-x1','region-y1','spawn-x','spawn-y','spawn-yaw'];
  const sceneNames={survey:'标准场景',survey_v2_native:'Terrain V2（原始地形）',survey_v2:'Terrain V2（定位标柱）',visible_ramp_v1:'坡面测试场景',visible_curve_v1:'曲面测试场景'};
  const sceneLimit=scene=>['survey_v2','survey_v2_native'].includes(scene)?15:25;
  function updateSceneBounds(){
    const selected=$('experiment-scene').value,limit=sceneLimit(selected);
    for(const id of fields.slice(0,6)){$(id).min=-limit;$(id).max=limit;}
    $('full-region').textContent=`选择 ${limit*2}×${limit*2} 米全场`;
    $('experiment-scene-note').textContent=selected==='survey_v2_native'?'原始 V2 月面地形，无额外定位标柱；定位方案以本次采集档案为准，请在受支承的缓坡或平地放置车辆。':selected==='survey_v2'?'V2 月面地形加入四个真实定位标柱，保留旧激光方案的对照场景。':'标准场景及测试场景使用原地形配置。';
  }
  const overlay=new THREE.Group();scene.add(overlay);
  const navigationOverlay=new THREE.Group();scene.add(navigationOverlay);let navigationKey='',lastMotionPhase='',lastSurveyPhase='';
  function drawNavigation(){
    const m=state.motion||{},visible=$('live').checked&&$('session').value===state.session;
    navigationOverlay.visible=visible;
    $('actual-navigation').textContent=!visible?'实际导航仅在跟随当前采集时显示。':`黑色实线：${m.active?'当前':'最后一次'} Nav2 路径；灰色虚线：本次已行驶轨迹；圆环：目标点。导航底图：浅灰可通行、黄褐有代价、橙色安全余量、红色禁行、透明未知。点击地图查阅具体格子的判定。`+(m.path_age>5&&m.active?' 路径尚未更新。':'');
    if(visible&&m.goal&&m.navigation_grid){const g=m.navigation_grid,x=Math.floor((m.goal[0]-g.origin[0])/g.resolution),y=Math.floor((m.goal[1]-g.origin[1])/g.resolution);if(x>=0&&y>=0&&x<g.width&&y<g.height){const cost=atob(g.data).charCodeAt(y*g.width+x);if(cost>=99&&cost!==255)$('actual-navigation').textContent+=' 当前观测已将目标判为障碍或安全余量区，请在其外侧重新选点。';}}
    const a=state.autonomy||{};
    const key=JSON.stringify([m.path,m.track,m.goal,m.map_bounds,m.navigation_grid?.version,a.active,a.origin,a.radius,a.decision]);if(key===navigationKey)return;navigationKey=key;
    while(navigationOverlay.children.length){const o=navigationOverlay.children[0];navigationOverlay.remove(o);o.traverse(c=>{c.geometry?.dispose();c.material?.map?.dispose();c.material?.dispose();});}
    if(m.navigation_grid){const g=m.navigation_grid,raw=atob(g.data),pixels=new Uint8Array(g.width*g.height*4);for(let i=0;i<raw.length;i++){const cost=raw.charCodeAt(i);pixels.set(cost===255?[0,0,0,0]:cost>=100?[185,53,48,185]:cost===99?[223,113,36,155]:cost>0?[183,145,80,110]:[220,220,220,110],i*4);}const texture=new THREE.DataTexture(pixels,g.width,g.height);texture.needsUpdate=true;const mesh=new THREE.Mesh(new THREE.PlaneGeometry(g.width*g.resolution,g.height*g.resolution),new THREE.MeshBasicMaterial({map:texture,transparent:true,side:THREE.DoubleSide,depthWrite:false}));mesh.position.set(g.origin[0]+g.width*g.resolution/2,g.origin[1]+g.height*g.resolution/2,-.08);navigationOverlay.add(mesh);}
    function route(points,dashed,color,z){if(!points?.length)return;const mat=dashed?new THREE.LineDashedMaterial({color,dashSize:.12,gapSize:.08,depthTest:false}):new THREE.LineBasicMaterial({color,depthTest:false});const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(p[0],p[1],(p[2]||0)+z))),mat);line.computeLineDistances();line.renderOrder=30;navigationOverlay.add(line);}
    route(m.path,false,0x111111,.18);route(m.track,true,0x777777,.21);
    if(m.map_bounds){const [x0,y0,x1,y1]=m.map_bounds;route([[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]],true,0xaaaaaa,.02);}
    if(m.goal){const ring=new THREE.Mesh(new THREE.RingGeometry(.13,.19,32),new THREE.MeshBasicMaterial({color:0x111111,side:THREE.DoubleSide,depthTest:false}));ring.position.set(...m.goal,.25);ring.renderOrder=31;navigationOverlay.add(ring);}
    if(a.origin&&a.radius)route(Array.from({length:65},(_,i)=>[a.origin[0]+a.radius*Math.cos(i*Math.PI/32),a.origin[1]+a.radius*Math.sin(i*Math.PI/32)]),true,0x777777,.04);
    if(a.active&&a.decision){const d=a.decision;const arrow=new THREE.ArrowHelper(new THREE.Vector3(Math.cos(d.yaw),Math.sin(d.yaw),0),new THREE.Vector3(d.x,d.y,.3),.7,0x111111,.2,.12);navigationOverlay.add(arrow);}
  }
  const ray=new THREE.Raycaster(), plane=new THREE.Plane(new THREE.Vector3(0,0,1),0);
  let mode=null,busy=false,state={enabled:false},followed=null,drag=null,polling=false,motionToken='',heartbeatBusy=false,sceneBusy=false;
  const leftAction=controls.mouseButtons.LEFT;
  $('fit-navigation').onclick=()=>{
    const m=state.motion||{},points=[...(m.path||[]),...(m.track||[]),...(m.goal?[m.goal]:[])];
    if(m.pose)points.push([m.pose.x,m.pose.y]);
    if(points.length<2&&m.map_bounds){const [x0,y0,x1,y1]=m.map_bounds;points.push([x0,y0],[x1,y1]);}
    if(!points.length)return;
    const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),x0=Math.min(...xs)-1,x1=Math.max(...xs)+1,y0=Math.min(...ys)-1,y1=Math.max(...ys)+1;
    controls.target.set((x0+x1)/2,(y0+y1)/2,0);
    const height=Math.max(y1-y0,(x1-x0)/camera.aspect)/(2*Math.tan(camera.fov*Math.PI/360));
    camera.position.copy(controls.target).add(new THREE.Vector3(0,-.001,height*1.15));controls.update();
  };
  function cancel(){if(drag){fields.slice(0,4).forEach((id,i)=>$(id).value=drag.previous[i]);drag=null;render();}mode=null;controls.mouseButtons.LEFT=leftAction;renderer.domElement.style.cursor='';}
  function task(){
    if(!$('experiment-form').reportValidity())throw new Error('请填写有效的区域和起点。');
    const [x0,y0,x1,y1,x,y,degrees]=fields.map(id=>Number($(id).value));
    if(x1-x0<.5||y1-y0<.5)throw new Error('区域右上角应大于左下角，每边至少 0.5 米。');
    return {version:1,region:[x0,y0,x1,y1],start:[x,y,degrees*Math.PI/180],scene:$('experiment-scene').value,simulation_profile:$('simulation-profile').value};
  }
  function render(){
    overlay.visible=$('show-experiment').checked;
    while(overlay.children.length){const child=overlay.children[0];overlay.remove(child);child.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});}
    const [x0,y0,x1,y1,x,y,degrees]=fields.map(id=>Number($(id).value));
    if(![x0,y0,x1,y1,x,y,degrees].every(Number.isFinite))return;
    const points=[[x0,y0,.04],[x1,y0,.04],[x1,y1,.04],[x0,y1,.04],[x0,y0,.04]].map(p=>new THREE.Vector3(...p));
    const border=new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineDashedMaterial({color:0x555555,dashSize:.18,gapSize:.12,depthTest:false}));border.computeLineDistances();overlay.add(border);
    const yaw=degrees*Math.PI/180;
    overlay.add(new THREE.ArrowHelper(new THREE.Vector3(Math.cos(yaw),Math.sin(yaw),0),new THREE.Vector3(x,y,.12),.8,0x111111,.22,.16));
    overlay.traverse(o=>{o.renderOrder=20;if(o.material)o.material.depthTest=false;});
    try{localStorage.setItem('mooncar-experiment-draft-r5-window-v1',JSON.stringify({version:1,region:[x0,y0,x1,y1],start:[x,y,yaw],scene:$('experiment-scene').value,simulation_profile:$('simulation-profile').value}));}catch{}
  }
  function fill(value){
    if(!value||value.version!==1||!Array.isArray(value.region)||value.region.length!==4||!Array.isArray(value.start)||value.start.length!==3||![...value.region,...value.start].every(v=>typeof v==='number'&&Number.isFinite(v)))throw new Error('任务文件格式无效。');
    const values=[...value.region,...value.start.slice(0,2),value.start[2]*180/Math.PI];
    const selected=value.scene||'survey';
    if(!Object.hasOwn(sceneNames,selected))throw new Error('不支持的地形场景。');
    if(values.slice(0,6).some(v=>Math.abs(v)>sceneLimit(selected))||Math.abs(values[6])>180||values[2]-values[0]<.5||values[3]-values[1]<.5)throw new Error('任务坐标或区域范围无效。');
    if(!['legacy','r5_support_v1'].includes(value.simulation_profile||'legacy'))throw new Error('不支持的仿真车辆配置。');
    $('simulation-profile').value=value.simulation_profile||'legacy';
    $('experiment-scene').value=selected;updateSceneBounds();
    fields.forEach((id,i)=>$(id).value=Number(values[i].toFixed(4)));render();
  }
  function fit(){
    overlay.visible=$('show-experiment').checked=true;
    const [x0,y0,x1,y1,x,y]=fields.map(id=>Number($(id).value));
    const loX=Math.min(x0,x1,x)-1,hiX=Math.max(x0,x1,x)+1,loY=Math.min(y0,y1,y)-1,hiY=Math.max(y0,y1,y)+1;
    controls.target.set((loX+hiX)/2,(loY+hiY)/2,0);
    const height=Math.max(hiY-loY,(hiX-loX)/camera.aspect)/(2*Math.tan(camera.fov*Math.PI/360));
    camera.position.copy(controls.target).add(new THREE.Vector3(0,-.001,height*1.15));controls.update();
  }
  function begin(next){cancel();stopPlayback();mode=next;overlay.visible=true;$('show-experiment').checked=true;controls.mouseButtons.LEFT=null;renderer.domElement.style.cursor='crosshair';message(next==='region'?'按住左键拖出矩形，松开完成；滚轮缩放，右键平移，Esc 取消。':'点击车辆实际起点，再点击前进方向；滚轮缩放，右键平移，Esc 取消。');}
  function ground(event){
    const rect=renderer.domElement.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),camera);
    return ray.ray.intersectPlane(plane,new THREE.Vector3());
  }
  function preview(event){
    const point=ground(event);if(!point)return;
    const limit=sceneLimit($('experiment-scene').value),x=Math.max(-limit,Math.min(limit,point.x)),y=Math.max(-limit,Math.min(limit,point.y));
    [Math.min(x,drag.x),Math.min(y,drag.y),Math.max(x,drag.x),Math.max(y,drag.y)].forEach((v,i)=>$(fields[i]).value=v.toFixed(2));render();
  }
  renderer.domElement.addEventListener('pointerdown',event=>{
    if(mode!=='region'||event.button!==0)return;
    event.stopImmediatePropagation();event.preventDefault();const point=ground(event);
    const limit=sceneLimit($('experiment-scene').value);
    if(!point||Math.abs(point.x)>limit||Math.abs(point.y)>limit){message(`绘制范围为 -${limit} 至 ${limit} 米；可滚轮缩放、右键平移。`,true);return;}
    drag={id:event.pointerId,x:point.x,y:point.y,previous:fields.slice(0,4).map(id=>$(id).value)};
    renderer.domElement.setPointerCapture(event.pointerId);preview(event);
  },true);
  renderer.domElement.addEventListener('pointermove',event=>{if(drag?.id===event.pointerId){event.stopImmediatePropagation();preview(event);}},true);
  renderer.domElement.addEventListener('pointerup',event=>{
    if(!drag||drag.id!==event.pointerId)return;
    event.stopImmediatePropagation();preview(event);
    const [x0,y0,x1,y1]=fields.slice(0,4).map(id=>Number($(id).value));
    if(x1-x0<.5||y1-y0<.5){cancel();message('区域每边至少 0.5 米；已恢复原区域。',true);}
    else{drag=null;cancel();message('区域已更新。可以继续调整坐标，或应用新的实验设置。');}
    renderer.domElement.releasePointerCapture(event.pointerId);
  },true);
  renderer.domElement.addEventListener('pointercancel',()=>{if(drag)cancel();},true);
  window.addEventListener('blur',cancel);
  fields.forEach(id=>$(id).oninput=()=>{cancel();render();});
  $('simulation-profile').onchange=()=>render();
  $('experiment-scene').onchange=()=>{
    updateSceneBounds();const limit=sceneLimit($('experiment-scene').value);
    for(const id of fields.slice(0,6))$(id).value=Math.max(-limit,Math.min(limit,Number($(id).value)));
    render();message('地形草稿已更新，应用后创建新实验。');
  };
  $('draw-region').onclick=()=>begin('region');$('place-spawn').onclick=()=>begin('spawn');$('fit-experiment').onclick=fit;
  $('full-region').onclick=()=>{cancel();const limit=sceneLimit($('experiment-scene').value);[-limit,-limit,limit,limit].forEach((v,i)=>$(fields[i]).value=v);render();fit();message(`已选择 ${limit*2}×${limit*2} 米任务区域；未知区域仍需逐步观测。`);};
  $('show-experiment').onchange=()=>{overlay.visible=$('show-experiment').checked;if(!overlay.visible)cancel();};
  document.addEventListener('keydown',e=>{if(e.key==='Escape'){cancel();message('已取消地图选点。');}});
  $('save-experiment').onclick=async()=>{try{const url='/api/task?task='+encodeURIComponent(JSON.stringify(task()));await api(url);const a=document.createElement('a');a.href=url;a.download='mooncar-experiment.json';document.body.append(a);a.click();a.remove();message('已请求下载任务文件。');}catch(e){message(e.message,true);}};
  $('load-experiment').onclick=()=>$('experiment-file').click();
  $('experiment-file').onchange=async()=>{try{const file=$('experiment-file').files[0];if(!file)return;if(file.size>8192)throw new Error('任务文件不能超过 8 KB。');fill(JSON.parse(await file.text()));cancel();fit();message('已载入任务草稿；尚未应用到仿真。');}catch(e){message(e.message,true);}finally{$('experiment-file').value='';}};
  function buttons(){const changing=['restarting','stopping'].includes(state.phase);$('apply-experiment').disabled=!state.enabled||busy||changing;$('stop-experiment').disabled=!state.enabled||busy||changing||!['starting','recording','failed'].includes(state.phase);$('drive-start').disabled=!state.motion?.ready||!state.session||state.motion?.active||state.autonomy?.active||busy||changing;$('drive-stop').disabled=!state.motion?.active;$('survey-start').disabled=!state.motion?.ready||!state.session||state.motion?.active||state.autonomy?.active||busy||changing;$('survey-stop').disabled=!state.autonomy?.active;for(const id of ['survey-budget','survey-radius'])$(id).disabled=!!state.autonomy?.active;}
  $('survey-start').onclick=async()=>{
    const inputs=[$('survey-budget'),$('survey-radius')];if(!inputs.every(i=>i.reportValidity()))return;
    busy=true;buttons();stopPlayback();motionToken='';
    try{await api('/api/simulation/explore',{experiment:state.task?.id,budget_seconds:Number(inputs[0].value),radius:Number(inputs[1].value),strategy:$('survey-strategy').value});$('live').checked=true;await onSession(state.session);message('自主探测已启动，车辆将自行选择下一观察位置。');}
    catch(e){message(e.message,true);}finally{busy=false;buttons();poll();}
  };
  $('survey-stop').onclick=async()=>{try{await api('/api/simulation/explore_stop',{});message('已请求停止自主探测，保留采集记录。');}catch(e){message(e.message,true);}};
  $('pick-drive-goal').onclick=()=>{begin('goal');message('点击地面设置导航目标；滚轮缩放，右键平移。');};
  $('drive-start').onclick=async()=>{
    const goal=[$('drive-x'),$('drive-y')];if(!goal.every(input=>input.value!==''&&input.reportValidity()))return;
    busy=true;buttons();stopPlayback();
    try{const result=await api('/api/simulation/move',{experiment:state.task?.id,goal:goal.map(input=>Number(input.value))});motionToken=result.token;$('live').checked=true;await onSession(state.session);message('已提交 Nav2 导航任务，正在沿途采集。');}
    catch(e){message(e.message,true);}finally{busy=false;buttons();poll();}
  };
  $('drive-stop').onclick=async()=>{try{await api('/api/simulation/cancel',{});motionToken='';message('已请求停止移动，采集档案保留。');}catch(e){message(e.message,true);}};
  setInterval(async()=>{if(!motionToken||heartbeatBusy)return;heartbeatBusy=true;try{const result=await api('/api/simulation/heartbeat',{token:motionToken});if(!result.active)motionToken='';}catch{motionToken='';}finally{heartbeatBusy=false;}},250);
  window.addEventListener('pagehide',()=>{if(motionToken)fetch('/api/simulation/cancel',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{});});
  function sceneFrame(){
    const visible=state.enabled&&['starting','recording'].includes(state.phase);$('gazebo-scene').hidden=!state.enabled;
    if(!visible||state.task?.gui===false){
      $('gazebo-image').hidden=true;
      $('scene-status').textContent=state.task?.gui===false?'本轮关闭图形画面采集':'仿真未运行';
      return;
    }
    if(sceneBusy)return;sceneBusy=true;const picture=new Image();picture.alt='实际 Gazebo 运行画面';
    picture.onload=()=>{picture.id='gazebo-image';$('gazebo-image').replaceWith(picture);$('scene-status').textContent='实时画面 · '+new Date().toLocaleTimeString();sceneBusy=false;};
    picture.onerror=()=>{$('scene-status').textContent='等待 Gazebo 画面';sceneBusy=false;};
    picture.src='/api/simulation/scene.jpg?t='+Date.now();
  }
  setInterval(sceneFrame,600);
  async function poll(){
    if(polling||busy)return;polling=true;
    try{
      state=await api('/api/simulation');
      if(state.autonomy?.active){overlay.visible=false;$('show-experiment').checked=false;}
      const a=state.autonomy||{},names={idle:'尚未开始',planning:'比较观察位置',moving:'自动行驶',turning:'转向观察',observing:'采集新视角',localization_paused:'已停车，等待定位恢复',budget_reached:'预算已用尽',no_candidates:'本轮探测结束',stopped:'已停止',fault:'已保护停车',distance_reached:'达到活动距离限制'};
      $('survey-strategy').disabled=!!a.active||busy;
      if(a.active){$('survey-budget').value=a.budget_seconds;$('survey-radius').value=a.radius;if(a.strategy)$('survey-strategy').value=a.strategy;}
      $('survey-state').textContent=(names[a.phase]||'等待自主探测服务')+(a.budget_seconds?` · ${Math.round(a.elapsed_seconds||0)} / ${a.budget_seconds} 仿真秒`:'');
      $('survey-reason').textContent=a.reason||'启动后显示下一观察位置及选择原因。';
      const last=a.history?.filter(h=>Number.isFinite(h.actual_quality_gain_m2)).at(-1);$('survey-result').textContent=`已记录 ${a.history?.length||0} 次观察动作`+(last?`；上次${last.success?'完成':'未完成'}，实际采样质量增量 ${last.actual_quality_gain_m2.toFixed(2)} m²（活动范围内质量加权面积）。`:'');
      $('live-motion').hidden=!state.enabled;
      const motionNames={idle:'等待导航目标',planning:'正在规划',executing:'车辆正在行驶并采集',cancelling:'正在停车',cancelled:'已停车',succeeded:'已到达目标',failed:'导航未完成'};
      const p=state.motion?.pose;
      $('drive-state').textContent=(motionNames[state.motion?.phase]||'等待仿真')+(p?` · (${p.x.toFixed(2)}, ${p.y.toFixed(2)}) m`:'')+(state.motion?.error?'。'+state.motion.error:'');
      const remaining=state.motion?.feedback?.distance_remaining;
      if(state.motion?.active&&Number.isFinite(remaining))$('drive-state').textContent+=`；剩余约 ${remaining.toFixed(2)} 米`;
      $('navigation-bounds').textContent=state.motion?.map_bounds?`导航地图当前范围：X ${state.motion.map_bounds[0].toFixed(1)}～${state.motion.map_bounds[2].toFixed(1)}，Y ${state.motion.map_bounds[1].toFixed(1)}～${state.motion.map_bounds[3].toFixed(1)} 米。边界内仍有未知区域。`:'等待导航地图。';
      drawNavigation();
      const phaseKey=state.task?.id+':'+state.motion?.phase;
      if(phaseKey!==lastMotionPhase){lastMotionPhase=phaseKey;if(['succeeded','cancelled','failed'].includes(state.motion?.phase))message($('drive-state').textContent,state.motion.phase!=='succeeded');}
      const surveyKey=a.archive+':'+a.phase+':'+a.reason;
      if(a.phase&&a.phase!=='idle'&&(a.active||surveyKey!==lastSurveyPhase))message(`${names[a.phase]||a.phase}：${a.reason}`,a.phase==='fault');
      lastSurveyPhase=surveyKey;
      const labels={idle:'尚未启动',restarting:'正在结束旧实验并初始化',starting:'仿真启动中，等待传感器档案',recording:'仿真运行，采集档案已建立',stopping:'正在结束实验',stopped:'已结束，档案已保留',failed:'启动或运行失败'};
      const profile=state.task?.simulation_profile==='r5_support_v1'?'R5 独立悬架':'旧刚性配置';
      const active=state.task?`实际任务：${sceneNames[state.task.scene||'survey']} · ${profile} · (${state.task.start[0].toFixed(2)}, ${state.task.start[1].toFixed(2)}) m · ${(state.task.start[2]*180/Math.PI).toFixed(1)}°。`:'';
      const version=state.software?.verified?`代码版本 ${state.software.git_commit.slice(0,8)}。`:'代码版本未核验。';
      $('experiment-state').textContent=state.enabled?`${labels[state.phase]||state.phase}。${version}${active}${state.error||''}`:'当前服务只支持编辑和保存；启动服务时加 --simulation 才能应用到 Gazebo。';
      if(state.session&&state.session!==followed&&state.phase==='recording'){await onSession(state.session);followed=state.session;}
    }catch(e){state={enabled:false};navigationOverlay.visible=false;$('experiment-state').textContent=`仿真服务未连接：${e.message}`;}
    finally{polling=false;buttons();}
  }
  $('experiment-form').onsubmit=async e=>{e.preventDefault();if(busy)return;try{const value=task();cancel();stopPlayback();busy=true;buttons();await api('/api/simulation/apply',value);state.phase='restarting';message('正在新建仿真与采集会话。区域仅保存为任务边界，尚未自动遍历。');}catch(error){message(error.message,true);}finally{busy=false;buttons();poll();}};
  $('stop-experiment').onclick=async()=>{if(busy)return;busy=true;buttons();try{await api('/api/simulation/stop',{});state.phase='stopping';}catch(e){message(e.message,true);}finally{busy=false;buttons();poll();}};
  updateSceneBounds();
  try{const saved=localStorage.getItem('mooncar-experiment-draft-r5-window-v1');if(saved)fill(JSON.parse(saved));}catch{}
  render();poll();setInterval(poll,600);
  return {cancel,telemetry:()=>state.motion,inspectionSession:()=>state.session,pick(event){
    if(!mode||event.button!==0)return false;
    if(mode==='region')return true;
    const point=ground(event);
    const limit=sceneLimit(mode==='goal'?(state.task?.scene||'survey'):$('experiment-scene').value);
    if(!point||Math.abs(point.x)>limit||Math.abs(point.y)>limit){message(`请选择 -${limit} 至 ${limit} 米范围内的地面。`,true);return true;}
    const x=Number(point.x.toFixed(2)),y=Number(point.y.toFixed(2));
    if(mode==='goal'){$('drive-x').value=x;$('drive-y').value=y;cancel();message(`导航目标：(${x}, ${y}) m。点击“开始导航并采集”执行。`);return true;}
    if(mode==='spawn'){$('spawn-x').value=x;$('spawn-y').value=y;mode='heading';message('再点击车辆前进方向。');}
    else{const dx=x-Number($('spawn-x').value),dy=y-Number($('spawn-y').value);if(Math.hypot(dx,dy)<.1)return true;$('spawn-yaw').value=(Math.atan2(dy,dx)*180/Math.PI).toFixed(1);cancel();}
    render();if(!mode)message('草稿已更新；点击“应用起点并采集”才会改变仿真。');return true;
  }};
}
