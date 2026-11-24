// Task/quality UI for the ordered observation receiver. No vehicle commands.
export const qualityColors = {unobserved:'#eeeeee',invalid_fit:'#b77945',fitted_accuracy_unknown:'#568ab0',
  qualified:'#4c956c',error_exceeds_threshold:'#c34b52',outside_representation:'#9388a2',not_evaluable:'#dbba65'};
export function connectionLabel(data) {
  const labels={waiting:'等待首帧',receiving:'正在接收',stale:'数据已停止更新',historical:'恢复的历史数据，等待新帧',fault:'接收异常'};
  const age=data.seconds_since_received;
  return `${labels[data.connection_state]||'连接状态未知'} · 已接收 ${data.accepted_frames} 帧${age==null?'':` · ${Math.floor(age)} 秒前收到新帧`}${data.latest_stamp_s==null?'':` · 最新数据时间 ${data.latest_stamp_s.toFixed(3)} 秒（任务时钟）`}`;
}
export function qualityDescription(data) {
  const c=data.coverage, states=c.state_counts;
  return `任务 ${c.task_cells} 格 · 已观测 ${(100*c.observed_fraction).toFixed(1)}% · 有效地表 ${(100*c.fitted_fraction).toFixed(1)}%\n未观测 ${states.unobserved} 格 · 拟合失败 ${states.invalid_fit} 格 · 精度未知 ${states.fitted_accuracy_unknown} 格\n相对参考达标 ${states.qualified} 格 · 超限 ${states.error_exceeds_threshold} 格\n表示范围外 ${states.outside_representation||0} 格 · 证据不足 ${states.not_evaluable||0} 格`;
}

export function liveSurveyPanel({api,onEnter,onMap,onLeave}) {
  const $=id=>document.getElementById(id);
  let active=false,busy=false,revision=null,session=null,current=null;
  function paint(data) {
    $('live-survey-task').textContent=`${data.task.id} · ${data.task.source.kind}（来源声明）\n${data.task.coordinates.frame} / 米 · ${data.task.region.bounds_m.join(', ')}\n时间 ${data.task.time.start_s}–${data.task.time.end_s} 秒（${data.task.time.clock}）`;
    const t=data.task.targets;
    $('live-survey-targets').textContent=`目标：高度 ${t.height_tolerance_m} m，坡度 ${t.slope_tolerance_deg}°；几何达标率 ${100*t.minimum_precision}%（${t.status==='provisional'?'暂定':'本任务设定'}）。覆盖率仅作统计，不作为通过条件；未测区域精度未知。`;
    $('live-survey-quality').textContent=qualityDescription(data);
    const a=data.accuracy;
    $('live-survey-accuracy').textContent=a.evaluable?`相对所给参考：高度 RMSE ${a.height_rmse_m==null?'未知':a.height_rmse_m.toFixed(4)+' m'}，坡度 MAE ${a.slope_mae_deg==null?'未知':a.slope_mae_deg.toFixed(3)+'°'}；不等于实物认证。`:'无合格独立参考，准确度未知。';
    const audit=data.configuration;
    const scope={source_declarations:'源码声明，不代表实测',runtime_snapshot:'所给运行快照',synthetic_fixture:'合成测试配置'}[audit.scope]||'配置来源未知';
    $('live-survey-configuration').textContent=audit.checks?.length?scope+'\n'+audit.checks.map(c=>`${c.label}：${c.state==='difference'?'有差异':c.state==='unknown'?'未知':c.state==='consistent'?'一致':c.state}`).join('\n'):'未提供配置快照，配置尚未核对。';
    $('live-survey-counts').textContent=`用于任务内建图 ${data.frame_counts.fused} 帧；任务时间外 ${data.frame_counts.outside_time_window} 帧未采用。`;
    const download=$('live-survey-download');
    if(data.accepted_frames){download.href=`/api/survey/export?session=${encodeURIComponent(data.session)}`;download.setAttribute('aria-disabled','false');}
    else {download.removeAttribute('href');download.setAttribute('aria-disabled','true');}
    const canvas=$('live-survey-grid'),ctx=canvas.getContext('2d'),r=data.task.grid.cell_m;
    const xs=data.quality_cells.map(c=>c.ix),ys=data.quality_cells.map(c=>c.iy);
    let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;
    for(let i=0;i<xs.length;i++){x0=Math.min(x0,xs[i]);x1=Math.max(x1,xs[i]);y0=Math.min(y0,ys[i]);y1=Math.max(y1,ys[i]);}
    canvas.width=Math.min(600,Math.max(120,x1-x0+1));canvas.height=Math.min(600,Math.max(100,Math.round(canvas.width*(y1-y0+1)/(x1-x0+1))));
    const w=canvas.width/(x1-x0+1),h=canvas.height/(y1-y0+1);
    for(const c of data.quality_cells){ctx.fillStyle=qualityColors[c.state];ctx.fillRect((c.ix-x0)*w,(y1-c.iy)*h,w+.1,h+.1);}
    canvas.setAttribute('aria-label',`任务质量图，X向右、Y向上，每格 ${r} 米。${qualityDescription(data)}`);
  }
  async function poll(force=false) {
    if(busy)return;busy=true;
    try {
      const data=await api(`/api/survey${revision===null||force?'':`?after=${revision}`}`);
      if(!data.enabled){$('live-survey-status').textContent='未启用实时接收；原有档案回放仍可使用。';$('live-survey-open').disabled=true;return;}
      $('live-survey-open').disabled=false;
      $('live-survey-results').hidden=false;
      $('live-survey-status').textContent=connectionLabel(data);
      $('live-survey-status').dataset.state=data.connection_state;
      if(data.session!==session){session=data.session;revision=null;if(!data.map)return;}
      if(data.map){current=data;revision=data.revision;paint(data);if(active)onMap(data.map);}
    }catch(error){$('live-survey-status').textContent=`实时服务未连接，保留最后收到的地图：${error.message}`;$('live-survey-status').dataset.state='stale';}
    finally{busy=false;}
  }
  function leave(restore=true) {
    if(!active)return;
    active=false;$('live-survey-open').textContent='查看实时勘测';
    $('live-survey-close').hidden=true;
    if(restore)onLeave();
  }
  $('live-survey-open').onclick=()=>{
    if(!active){active=true;onEnter();$('live-survey-open').textContent='正在查看实时勘测';$('live-survey-close').hidden=false;}
    if(current)onMap(current.map,true);else poll(true);
  };
  $('live-survey-close').onclick=()=>leave();
  poll();setInterval(()=>poll(),1000);
  return {isActive:()=>active,leave};
}
