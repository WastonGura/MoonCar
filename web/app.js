import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';
import {ReplayCache,bufferedRanges,replayDelay} from './replay_cache.js';
import {experimentEditor} from './experiment_editor.js';

const $ = id => document.getElementById(id);
const scene = new THREE.Scene(); scene.background = new THREE.Color('#f5f5f3');
const camera = new THREE.PerspectiveCamera(45, 1, .01, 1000); camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias: true, preserveDrawingBuffer: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); $('viewport').appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
const ambient = new THREE.HemisphereLight(0xffffff, 0x888888, 2); scene.add(ambient);
const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(5, -5, 12); scene.add(light);
const groups = Object.fromEntries(['surface','mesh','cloud','hazards','track','vehicle','routes'].map(k => [k,new THREE.Group()]));
Object.values(groups).forEach(g => scene.add(g));
const markers = new THREE.Group(); scene.add(markers);
const objLoader = new OBJLoader();
let map = null, pickMode = null, start = null, goal = null, surfaceMesh = null, meshObj = null, inspectedCell = null;
let generation = 0, frameCount = 0, playing = false, playTimer = null, loading = false;
let mapTask=null, pendingMap=null, seekTimer=null, playbackGeneration=0;
let seekRAF=null, replayCache=null, cacheController=null;
let liveBusy=false, lastArchiveVersion=null;
let navigation={enabled:false}, navigationToken='', navigationBusy=false, heartbeatBusy=false;
const ray = new THREE.Raycaster();
const modes = {fixed:'固定地形代价',direction:'加入行驶方向',uncertainty:'加入观测支持度',full:'方向 + 观测支持度'};
const statuses = {ok:'找到路线',invalid_start:'起点或车体覆盖区域不可通行',invalid_goal:'终点或车体覆盖区域不可通行',no_path:'没有可行路线',budget_exceeded:'超出计算预算'};

function message(text, error=false) {$('message').textContent=text; $('message').classList.toggle('error',error);}
async function api(url, body) {
  const response = await fetch(url, body === undefined ? {} : {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const result = await response.json(); if(!response.ok) throw new Error(result.error || '请求失败'); return result;
}
function clear(group) {while(group.children.length) {const child=group.children[0]; group.remove(child); child.traverse(o=>{o.geometry?.dispose();if(Array.isArray(o.material))o.material.forEach(m=>m.dispose());else o.material?.dispose();});}}
function geometry(vertices) {const g=new THREE.BufferGeometry(); g.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3)); return g;}
function line(points, material, group) {if(points.length<2)return; const object=new THREE.Line(geometry(points.flat()),material); object.computeLineDistances(); group.add(object);}
function vehicleMarker(direction,origin){const arrow=new THREE.ArrowHelper(direction,origin,.45,0x111111,.14,.10);arrow.traverse(o=>{if(o.material){o.material.depthTest=false;o.material.depthWrite=false;}o.renderOrder=10;});groups.vehicle.add(arrow);}
function fit(top=false) {
  let box=new THREE.Box3().setFromObject(groups.cloud);
  if(box.isEmpty())box=new THREE.Box3().setFromObject(groups.mesh);
  if(box.isEmpty())box=new THREE.Box3().setFromObject(groups.surface);
  if(box.isEmpty()){camera.position.set(4,-6,5); controls.target.set(0,0,0); return;}
  const center=box.getCenter(new THREE.Vector3());
  const size=Math.max(box.getSize(new THREE.Vector3()).length(),2);
  controls.target.copy(center); camera.position.copy(center).add(top?new THREE.Vector3(0,-.001,size*1.25):new THREE.Vector3(size*.65,-size*.85,size*.8)); controls.update();
}
function draw(data, refit=false, preserveSelection=false) {
  const previousStart=start,previousGoal=goal;
  map=data; ['surface','cloud','hazards','track','vehicle','routes'].forEach(k => clear(groups[k])); surfaceMesh=null;
  const cloud=new THREE.Points(geometry(data.points.flat()),new THREE.PointsMaterial({color:0x505050,size:.018})); groups.cloud.add(cloud);
  const vertices=[],colors=[],pickCells=[],hazardEdges=[];
  const heights=data.cells.filter(c=>c.valid).map(c=>c.z);
  let min=Infinity,max=-Infinity; for(const h of heights){min=Math.min(min,h);max=Math.max(max,h);}
  for(const cell of data.cells) {
    if(!cell.valid) continue;
    const half=data.resolution/2;
    const corners=[[-half,-half],[half,-half],[half,half],[-half,half]].map(([dx,dy])=>[cell.x+dx,cell.y+dy,cell.z+cell.gx*dx+cell.gy*dy]);
    const shade=$('quality-mode').checked ? .92-.65*(cell.quality||0) : .78-.20*(cell.z-min)/(max-min || 1), color=new THREE.Color(shade,shade,shade);
    for(const i of [0,1,2,0,2,3]) {vertices.push(...corners[i]);colors.push(color.r,color.g,color.b);}
    pickCells.push(cell);
    if(cell.slope>12 || cell.roughness>.04 || cell.support<.34) for(let i=0;i<4;i++) for(const p of [corners[i],corners[(i+1)%4]]) hazardEdges.push(p[0],p[1],p[2]+.008);
  }
  groups.hazards.add(new THREE.LineSegments(geometry(hazardEdges),new THREE.LineBasicMaterial({color:0x222222})));
  const g=geometry(vertices); g.setAttribute('color',new THREE.Float32BufferAttribute(colors,3)); g.computeVertexNormals();
  const SurfaceMaterial=$('quality-mode').checked?THREE.MeshBasicMaterial:THREE.MeshLambertMaterial;
  surfaceMesh=new THREE.Mesh(g,new SurfaceMaterial({vertexColors:true,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:1}));
  surfaceMesh.userData.cells=pickCells; groups.surface.add(surfaceMesh);
  const track=data.robot_trajectory?.length?data.robot_trajectory:data.trajectory;
  line(track.map(p=>p.position),new THREE.LineDashedMaterial({color:0x888888,dashSize:.07,gapSize:.04}),groups.track);
  if(data.robot) {
    const orientation=new THREE.Quaternion().fromArray(data.robot.quaternion);
    const direction=new THREE.Vector3(1,0,0).applyQuaternion(orientation);
    const origin=new THREE.Vector3().fromArray(data.robot.position);origin.z+=.1;
    vehicleMarker(direction,origin);
  } else if(data.trajectory.length) {
    const cursor=new THREE.Mesh(new THREE.SphereGeometry(.055,12,8),new THREE.MeshBasicMaterial({color:0x222222}));
    cursor.position.fromArray(data.trajectory.at(-1).position); groups.track.add(cursor);
  }
  $('map-info').textContent=`${data.points.length.toLocaleString()} 个体素点 · ${data.cells.filter(c=>c.valid).length} 个地表格 · ${data.resolution} m/格`;
  $('empty').hidden=!!data.points.length;
  $('source').textContent=`来源：${data.source}。坐标系：${data.frame}。${data.robot?'显示车体位置、方向与轨迹。':'缺少车体位姿，仅显示相机轨迹。'}`;
  $('results').textContent='选择起终点，比较路线。';
  const inspected=data.cells.find(c=>c.valid&&c.ix===inspectedCell?.ix&&c.iy===inspectedCell?.iy);
  if(inspected)showCell(inspected);else $('cell-detail').textContent='点击地表查看观测结果。';
  const retain=old=>old&&data.cells.find(c=>c.valid&&c.ix===old.ix&&c.iy===old.iy)||null;
  start=preserveSelection?retain(previousStart):null;goal=preserveSelection?retain(previousGoal):null;
  renderMarkers(); if(refit)fit();
}
function renderMarkers() {
  clear(markers);
  for(const [point,dark] of [[start,false],[goal,true]]) if(point) {
    const mesh=new THREE.Mesh(new THREE.ConeGeometry(.08,.2,16),new THREE.MeshBasicMaterial({color:dark?0x111111:0x888888}));
    mesh.rotation.x=Math.PI/2;mesh.position.set(point.x,point.y,point.z+.15);markers.add(mesh);
  }
  $('endpoints').textContent=`起点：${start?`${start.x.toFixed(2)}, ${start.y.toFixed(2)}`:'未选择'}；终点：${goal?`${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}`:'未选择'}`;
  $('plan').disabled=!(start&&goal) || loading;
  navigationControls();
}
async function loadMesh(sessionName) {
  clear(groups.mesh);
  meshObj = null;
  if (!sessionName) return;
  try {
    const res = await fetch(`/api/mesh?session=${encodeURIComponent(sessionName)}`);
    if (!res.ok) return;
    const text = await res.text();
    if (!text || text.startsWith('# Empty mesh')) return;
    const object = objLoader.parse(text);

    let minZ = Infinity, maxZ = -Infinity;
    object.traverse(child => {
      if (child.isMesh && child.geometry) {
        const pos = child.geometry.attributes.position;
        if (pos) {
          for (let i = 0; i < pos.count; i++) {
            const z = pos.getZ(i);
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
          }
        }
      }
    });

    const range = (maxZ - minZ) || 1;
    const isWireframe = $('wireframe')?.checked || false;

    object.traverse(child => {
      if (child.isMesh && child.geometry) {
        child.geometry.computeVertexNormals();
        const pos = child.geometry.attributes.position;
        if (pos) {
          const colors = [];
          for (let i = 0; i < pos.count; i++) {
            const z = pos.getZ(i);
            const ratio = (z - minZ) / range;
            const r = 0.50 + 0.38 * ratio;
            const g = 0.52 + 0.38 * ratio;
            const b = 0.55 + 0.38 * ratio;
            colors.push(r, g, b);
          }
          child.geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        }
        child.material = new THREE.MeshLambertMaterial({
          vertexColors: true,
          wireframe: isWireframe,
          side: THREE.DoubleSide
        });
      }
    });

    meshObj = object;
    groups.mesh.add(object);
  } catch (err) {
    console.warn('Failed to load 3D mesh:', err);
  }
}
function stopPlayback(){playing=false;playbackGeneration++;clearTimeout(playTimer);$('play').textContent='播放';}
function cancelSeek(){clearTimeout(seekTimer);cancelAnimationFrame(seekRAF);seekTimer=seekRAF=null;}
function showBuffered(){
  const frames=replayCache&&!$('live').checked?replayCache.frames:new Set();
  $('buffered-track').replaceChildren(...bufferedRanges(frames,frameCount).map(([start,end])=>{
    const span=document.createElement('span');span.style.left=`${start}%`;span.style.width=`${end-start}%`;return span;
  }));
  const count=Array.from(frames).filter(f=>f>0&&f<=frameCount).length;
  $('buffered-label').textContent=$('live').checked?'跟随采集 · 读取最新帧':`灰色：已缓存 ${frameCount?Math.round(count/frameCount*100):0}%`;
  $('buffered-label').title=`已缓存 ${count} / ${frameCount} 帧；浅色区段尚未缓存。`;
}
async function prepareReplay(info,initial=false){
  cacheController?.abort();replayCache=null;
  showBuffered();
  const controller=new AbortController();cacheController=controller;
  if($('live').checked){$('cache-status').textContent='跟随采集时读取最新档案；关闭后准备离线缓存。';if(initial)await loadMap(true);return;}
  $('cache-status').textContent='正在检查本地回放缓存…';
  try{
    const cache=new ReplayCache($('session').value,info.archive_updated_ns);
    await cache.open();if(controller.signal.aborted)return;replayCache=cache;
    const progress=()=>{
      if(controller.signal.aborted)return;
      const count=Array.from(cache.frames).filter(f=>f>0&&f<=info.frames).length;
      const text=count===info.frames&&cache.has(0)?`回放缓存已就绪 · ${info.frames} 帧`:`准备回放缓存 · ${count} / ${info.frames} 帧`;
      if($('cache-status').textContent!==text)$('cache-status').textContent=text;
      $('cache-status').dataset.bytes=cache.bytes;
      showBuffered();
    };
    cache.onChange=progress;
    progress();
    if(initial)await loadMap(true);
    if(controller.signal.aborted)return;
    for(let frame=0;frame<=info.frames;frame++){
      if(controller.signal.aborted)return;
      if(cache.has(frame))continue;
      while(loading){
        if(controller.signal.aborted)return;
        await new Promise(resolve=>setTimeout(resolve,50));
      }
      if(controller.signal.aborted)return;
      await cache.read(frame,controller.signal);
      if(controller.signal.aborted)return;
    }
  }catch(e){if(!controller.signal.aborted){replayCache=null;showBuffered();$('cache-status').textContent=`缓存准备未完成：${e.message}。仍可按需回放。`;if(initial&&!map)await loadMap(true);}}
}
async function displayMap(request){
  try{
    const began=performance.now();
    const cache=!$('live').checked?replayCache:null;
    const data=cache?await cache.read(request.frame):await api(`/api/map?session=${encodeURIComponent(request.name)}&frame=${request.frame}`);
    if(request.token!==generation)return;
    const received=performance.now();
    draw(data,request.refit,$('live').checked&&!request.refit);lastArchiveVersion=data.archive_updated_ns;
    $('viewport').dataset.replayReadMs=(received-began).toFixed(1);$('viewport').dataset.replayDrawMs=(performance.now()-received).toFixed(1);
    $('frame-label').textContent=`${data.frames} / ${frameCount} 帧`;
    $('connection').textContent=$('live').checked?'采集档案 · 跟随更新':'本地档案 · 离线回放';
  }catch(e){if(request.token===generation){stopPlayback();map=null;start=null;goal=null;surfaceMesh=null;Object.values(groups).forEach(clear);clear(markers);message(e.message,true);$('connection').textContent='读取失败';$('frame-label').textContent='读取失败';}}
  finally{if(request.token===generation){loading=!!seekTimer||!!seekRAF;renderMarkers();}}
}
function loadMap(refit=false) {
  if(!$('session').value)return;
  const request={token:++generation,name:$('session').value,frame:$('timeline').value,refit};
  loading=true;renderMarkers();
  pendingMap=null;
  // Cached seeks bypass an obsolete network request and render at the next animation frame.
  if(!$('live').checked&&replayCache?.has(request.frame))return displayMap(request);
  pendingMap=request;
  // Only one map computation in flight; replace queued seeks with the latest target.
  if(mapTask)return mapTask;
  mapTask=(async()=>{
    try{while(pendingMap){
      const request=pendingMap;pendingMap=null;
      await displayMap(request);
    }}finally{mapTask=null;}
  })();
  return mapTask;
}
function compareSessions(a,b) {
  const stamp=name=>name.match(/(?:^|\/)survey-(\d{8}T\d{12}Z)\.npz$/)?.[1]||'';
  return stamp(b.name).localeCompare(stamp(a.name))||a.name.localeCompare(b.name);
}
function sessionLabel(name) {
  const match=name.match(/(?:^|\/)survey-(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})(\d{6})Z\.npz$/);
  if(!match)return name;
  const [,year,month,day,hour,minute,second,microsecond]=match;
  const iso=`${year}-${month}-${day}T${hour}:${minute}:${second}.${microsecond.slice(0,3)}Z`,date=new Date(iso);
  if(!Number.isFinite(date.getTime())||date.toISOString()!==iso)return name;
  const pad=n=>String(n).padStart(2,'0');
  const time=`${date.getFullYear()}-${pad(date.getMonth()+1)}-${pad(date.getDate())} ${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
  const experiment=name.match(/(?:^|\/)experiment-([a-f0-9]+)\//)?.[1];
  return `${time} · 地形采集${experiment?` · ${experiment.slice(0,6)}`:''}`;
}

async function sessions(preferred=null){
  try{const data=await api('/api/sessions'); const previous=$('session').value;$('session').replaceChildren(new Option('请选择记录',''));
    data.sessions.sort(compareSessions);
    for(const item of data.sessions){const option=new Option(sessionLabel(item.name),item.name);option.title=`电脑本地时间 · ${item.name}`;$('session').add(option);}
    if(typeof preferred==='string'&&data.sessions.some(s=>s.name===preferred)){$('session').value=preferred;await $('session').onchange();}
    else if(data.sessions.some(s=>s.name===previous))$('session').value=previous;
    else if(data.sessions.length){
      let saved='';try{saved=localStorage.getItem('mooncar-last-session')||'';}catch{}
      $('session').value=data.sessions.some(s=>s.name===saved)?saved:data.sessions[0].name;
      $('session').onchange();
    }
    else if(previous)$('session').onchange();
    $('connection').textContent=`本地档案服务 · ${data.sessions.length} 份记录`;
    if(!data.sessions.length)message('暂无采集档案。请启动采集节点，或载入明确标注的算法测试档案。');
  }catch(e){message(e.message,true);$('connection').textContent='服务未连接';}
}
$('session').onchange=async()=>{
  inspectedCell=null;
  stopPlayback();cancelSeek();pendingMap=null;cacheController?.abort();replayCache=null;const token=++generation;
  $('cache-status').textContent=$('session').value?'正在检查档案…':'选择档案后自动准备回放缓存。';
  frameCount=0;showBuffered();$('frame-label').textContent='0 / 0 帧';
  if($('session').value)try{localStorage.setItem('mooncar-last-session',$('session').value);}catch{}
  map=null;start=null;goal=null;surfaceMesh=null;meshObj=null;loading=true;renderMarkers();
  $('timeline').disabled=true;$('play').disabled=true;
  if(!$('session').value){map=null;start=null;goal=null;surfaceMesh=null;meshObj=null;Object.values(groups).forEach(clear);clear(markers);$('empty').hidden=false;$('plan').disabled=true;$('play').disabled=true;$('timeline').disabled=true;$('download').removeAttribute('href');$('download').setAttribute('aria-disabled','true');$('download-mesh')?.removeAttribute('href');$('download-mesh')?.setAttribute('aria-disabled','true');$('download-dem')?.removeAttribute('href');$('download-dem')?.setAttribute('aria-disabled','true');return;}
  try{const info=await api(`/api/info?session=${encodeURIComponent($('session').value)}`); if(token!==generation)return;frameCount=info.frames;
    $('timeline').max=frameCount;$('timeline').value=frameCount;$('timeline').disabled=false;$('play').disabled=!frameCount;
    $('download').href=`/api/archive?session=${encodeURIComponent($('session').value)}`;$('download').setAttribute('aria-disabled','false');
    $('download-mesh').href=`/api/mesh?session=${encodeURIComponent($('session').value)}&download=1`;$('download-mesh').setAttribute('aria-disabled','false');
    $('download-dem').href=`/api/dem?session=${encodeURIComponent($('session').value)}&download=1`;$('download-dem').setAttribute('aria-disabled','false');
    loadMesh($('session').value);
    await prepareReplay(info,true);
  }catch(e){if(token===generation)message(e.message,true);}
};
$('refresh').onclick=sessions;$('fit').onclick=()=>fit();$('top').onclick=()=>fit(true);
$('quality-mode').onchange=()=>{$('quality-legend').hidden=!$('quality-mode').checked;if(map)draw(map,false,true);};
for(const key of Object.keys(groups)) if($(key)) { groups[key].visible = $(key).checked; $(key).onchange=()=>groups[key].visible=$(key).checked; }
if($('wireframe')) $('wireframe').onchange=()=>{if(meshObj)meshObj.traverse(c=>{if(c.isMesh&&c.material)c.material.wireframe=$('wireframe').checked;});};
for(const name of ['start','goal'])$(`pick-${name}`).onclick=()=>{experiment.cancel();stopPlayback();pickMode=name;message(`请点击已观测地表选择${name==='start'?'起点':'终点'}。`);};
$('heading').onchange=()=>{clear(groups.routes);$('results').textContent='朝向已改变，请重新规划。';};
function showCell(cell){
  const dl=document.createElement('dl');
  for(const [label,value] of [['坐标',`${cell.x.toFixed(2)}, ${cell.y.toFixed(2)} m`],['高度',`${cell.z.toFixed(3)} m`],['坡度',`${cell.slope.toFixed(1)}°`],['粗糙度',`${(cell.roughness*1000).toFixed(1)} mm`],['采样充分度',`${Math.round((cell.quality||0)*100)}%`],['格内覆盖',`${Math.round((cell.coverage||0)*100)}%`],['观察方向数',cell.view_directions||0],['观测支持度',`${Math.round(cell.support*100)} / 100`],['观测帧数',cell.observations]]) {const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;dd.textContent=value;dl.append(dt,dd);}
  $('cell-detail').replaceChildren(dl);
}
let pointerDown=null;
renderer.domElement.addEventListener('pointerdown',e=>pointerDown=[e.clientX,e.clientY]);
renderer.domElement.addEventListener('pointerup',e=>{
  if(e.button!==0||!pointerDown||Math.hypot(e.clientX-pointerDown[0],e.clientY-pointerDown[1])>4)return;
  if(experiment.pick(e)){pickMode=null;return;}
  const rect=renderer.domElement.getBoundingClientRect(); ray.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),camera);
  let cell=null;
  if(surfaceMesh&&groups.surface.visible){
    const hit=ray.intersectObject(surfaceMesh)[0];
    if(hit)cell=surfaceMesh.userData.cells[Math.floor(hit.faceIndex/2)];
  }
  if(!cell&&meshObj&&groups.mesh.visible){
    const hit=ray.intersectObject(meshObj,true)[0];
    if(hit&&map?.cells){
      const half=map.resolution/2;
      cell=map.cells.find(c=>c.valid&&Math.abs(c.x-hit.point.x)<=half&&Math.abs(c.y-hit.point.y)<=half);
    }
  }
  if(!cell)return;
  inspectedCell=cell;showCell(cell);
  if(pickMode){if(pickMode==='start')start=cell;else goal=cell;pickMode=null;clear(groups.routes);$('results').textContent='目标已改变，请重新规划。';renderMarkers();message('已选择位置。规划会检查整个车体覆盖范围。');}
});
$('timeline').oninput=()=>{
  $('live').checked=false;stopPlayback();generation++;pendingMap=null;loading=true;renderMarkers();
  cancelSeek();
  if(replayCache?.has($('timeline').value))seekRAF=requestAnimationFrame(()=>{seekRAF=null;loadMap();});
  else seekTimer=setTimeout(()=>{seekTimer=null;loadMap();},100);
};
$('play').onclick=()=>{
  if(playing){stopPlayback();return;}$('live').checked=false;playing=true;$('play').textContent='暂停';
  cancelSeek();const token=++playbackGeneration;
  if(Number($('timeline').value)>=frameCount)$('timeline').value=0;
  const step=async()=>{if(!playing||token!==playbackGeneration)return;if(Number($('timeline').value)>=frameCount){stopPlayback();return;}const tick=performance.now();$('timeline').value=Number($('timeline').value)+1;await loadMap();if(playing&&token===playbackGeneration)playTimer=setTimeout(step,replayDelay($('playback-speed').value,performance.now()-tick));};step();
};
$('plan').onclick=async()=>{
  stopPlayback();const token=generation;const startPoint=start,goalPoint=goal;const yaw=Number($('heading').value);
  $('plan').disabled=true;clear(groups.routes);$('results').textContent='正在比较四组代价模型…';
  try{const results=await api('/api/plan',{session:$('session').value,frame:Number($('timeline').value),start:[startPoint.x,startPoint.y,yaw],goal:[goalPoint.x,goalPoint.y]});
    if(token!==generation||start!==startPoint||goal!==goalPoint||yaw!==Number($('heading').value))return;
    $('results').replaceChildren();
    for(const result of results.results){
      const article=document.createElement('article'),title=document.createElement('b'),detail=document.createElement('div');title.textContent=modes[result.mode];
      detail.textContent=`${statuses[result.status]||result.status} · ${result.length.toFixed(2)} m · ${result.compute_ms.toFixed(0)} ms`;
      article.append(title,detail);$('results').append(article);
      if(['fixed','full'].includes(result.mode))line(result.path.map(p=>[p.x,p.y,p.z+.05]),result.mode==='full'?new THREE.LineBasicMaterial({color:0x000000}):new THREE.LineDashedMaterial({color:0x606060,dashSize:.12,gapSize:.09}),groups.routes);
    }message('对照完成。路线仅为离线规划结果，未发送到车辆。');
  }catch(e){message(e.message,true);$('results').textContent='规划请求失败。';}finally{renderMarkers();}
};
$('capture').onclick=()=>{renderer.render(scene,camera);const a=document.createElement('a');a.href=renderer.domElement.toDataURL('image/png');a.download='mooncar-map-view.png';a.click();};
new ResizeObserver(()=>{const r=$('viewport').getBoundingClientRect();renderer.setSize(r.width,r.height);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();}).observe($('viewport'));
const experiment=experimentEditor({scene,camera,controls,renderer,api,message,stopPlayback:()=>{stopPlayback();pickMode=null;},onSession:async name=>{$('live').checked=true;await sessions(name);}});
fit();renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});sessions();
$('live').onchange=()=>{stopPlayback();cancelSeek();generation++;pendingMap=null;loading=false;renderMarkers();$('connection').textContent=$('live').checked?'采集档案 · 跟随更新':'本地档案 · 离线回放';$('live-status').textContent=$('live').checked?'每 2 秒检查采集档案；地形变化后需重新规划。':'离线回放。';if($('session').value&&lastArchiveVersion!==null)prepareReplay({frames:frameCount,archive_updated_ns:lastArchiveVersion});};
setInterval(async()=>{
  if(!$('live').checked||!$('session').value||liveBusy||loading)return;
  liveBusy=true;const name=$('session').value;
  try{const info=await api(`/api/info?session=${encodeURIComponent(name)}`);
    if(name!==$('session').value||!$('live').checked)return;
    const age=(Date.now()-info.archive_updated_ns/1e6)/1000;
    const telemetry=experiment.telemetry();
    $('live-status').textContent=telemetry?.depth_active&&age>10?(telemetry.active?'传感器正在采集，等待下一次档案更新。':'传感器正常；静止时重复视角不增加关键帧。'):(age>10?`采集未更新已 ${Math.floor(age)} 秒；显示历史观测。`:`采集档案 ${Math.max(0,Math.floor(age))} 秒前更新。`);
    if(info.archive_updated_ns!==lastArchiveVersion){frameCount=info.frames;$('timeline').max=frameCount;$('timeline').value=frameCount;await loadMap();await loadMesh(name);}
  }catch(e){$('live-status').textContent=`采集连接失败：${e.message}`;}finally{liveBusy=false;}
},2000);

function navigationControls(){
  $('navigation-controls').hidden=!navigation.enabled;
  $('navigate').disabled=!navigation.enabled||!navigation.servers_ready||navigation.active||navigation.fault||!goal||loading||$('session').value!==navigation.session||Number($('timeline').value)!==frameCount;
  $('cancel-navigation').disabled=!navigation.active||!navigationToken;
}
async function navigationState(){
  if(navigationBusy)return;navigationBusy=true;
  try{
    navigation=await api('/api/navigation');navigationControls();
    if(!navigation.enabled)return;
    const phases={idle:'空闲',planning:'正在规划',executing:'正在执行',checking:'检查下一路段',succeeded:'到达目标',cancelled:'已取消',failed:'执行失败',fault:'取消未确认，保持禁止运动'};
    $('navigation-state').textContent=`${phases[navigation.phase]||navigation.phase}${navigation.error?'：'+navigation.error:''}`;
    if(!navigation.active)navigationToken='';
    if(navigation.pose&&map?.frame==='map'&&$('session').value===navigation.session){
      clear(groups.vehicle);const p=navigation.pose;
      vehicleMarker(new THREE.Vector3(Math.cos(p.yaw),Math.sin(p.yaw),0),new THREE.Vector3(p.x,p.y,p.z+.15));
      if(navigation.track.length){clear(groups.track);line(navigation.track.map(p=>[p.x,p.y,p.z+.04]),new THREE.LineDashedMaterial({color:0x777777,dashSize:.06,gapSize:.04}),groups.track);}
      if(navigation.active){clear(groups.routes);line(navigation.path.map(p=>[p.x,p.y,p.z+.08]),new THREE.LineBasicMaterial({color:0x111111}),groups.routes);}
    }
  }catch(e){if(navigation.enabled){$('navigation-state').textContent=`车辆连接中断：${e.message}`;$('navigate').disabled=true;}}
  finally{navigationBusy=false;}
}
$('navigate').onclick=async()=>{
  stopPlayback();$('navigate').disabled=true;
  try{const result=await api('/api/navigation/start',{session:$('session').value,frame:Number($('timeline').value),goal:[goal.x,goal.y],yaw:0,mode:$('navigation-mode').value});navigationToken=result.token;message('已提交导航目标。');await navigationState();}
  catch(e){message(e.message,true);navigationControls();}
};
$('cancel-navigation').onclick=async()=>{try{await api('/api/navigation/cancel',{token:navigationToken});await navigationState();}catch(e){message(e.message,true);}};
setInterval(async()=>{if(!navigationToken||heartbeatBusy)return;heartbeatBusy=true;try{const result=await api('/api/navigation/heartbeat',{token:navigationToken});if(result.active===false)navigationToken='';}catch(e){message(`导航连接失效：${e.message}`,true);navigationToken='';}finally{heartbeatBusy=false;}},750);
setInterval(navigationState,1500);navigationState();
