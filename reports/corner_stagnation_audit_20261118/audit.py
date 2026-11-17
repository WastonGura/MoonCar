"""Read-only audit of corner attraction; no new world or controller changes."""
import hashlib
import json
import math
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/'reports/v3_eight_minute_comparison_20261117'
OUTPUT=Path(__file__).resolve().parent
RAW=ROOT/'tmp/ros_acceptance/v3-eight-minute-comparison-20261117/candidate/case/observer/truth.jsonl'


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):value.update(chunk)
    return value.hexdigest()


def main():
    result=read(REPORT/'candidate-result.json');mission=result['autonomy']
    quality=read(REPORT/'candidate-map.json')['payload'];resolution=quality['resolution']
    bounds=read(REPORT/'reference-metadata.json')['bounds']
    run=next(r for r in read(REPORT/'long-run-summary.json') if r['strategy']=='candidate')
    capacity=run['first_capacity_warning']['elapsed_seconds']
    source=ROOT/'scripts/autonomous_survey.py';code=source.read_text(encoding='utf-8')
    visibility_code=code.split('    def visibility(',1)[1].split('    candidates=[]',1)[0]
    matches=re.findall(r'abs\(px\)>([0-9.]+) or abs\(py\)>([0-9.]+)',visibility_code)
    assert matches and all(value==matches[0] for value in matches)
    xlimit,ylimit=map(float,matches[0])
    def outside(key):
        x,y=(key[0]+.5)*resolution,(key[1]+.5)*resolution
        return not(bounds[0]<=x<bounds[2] and bounds[1]<=y<bounds[3])
    observed={(c['ix'],c['iy']) for c in quality['cells']}
    outside_observed={key for key in observed if outside(key)}
    assert not outside_observed
    actions=[]
    for index,action in enumerate(mission['history'],1):
        visible={tuple(key) for key in action['visible_cells']}
        out={key for key in visible if outside(key)}
        outside_new=len(out-observed)*resolution**2
        expected=action['expected_new_m2']
        # Original predictions are rounded to three decimals. Outside cells
        # were never present in the cumulative map (it does not evict cells).
        assert outside_new<=expected+.001
        actions.append(dict(action=index,kind=action['kind'],started_seconds=action['started_seconds'],
            ended_seconds=action.get('elapsed_seconds'),target=[action['x'],action['y']],
            yaw_degrees=math.degrees(action['yaw']),predicted_new_m2=expected,
            outside_scene_predicted_new_m2=outside_new,
            all_predicted_new_outside=expected>0 and abs(expected-outside_new)<=.001,
            actual_new_total_m2=action['roi_gain']['new_observed_m2'],
            actual_new_in_predicted_view_m2=action['predicted_view_gain']['new_observed_m2'],
            motion_success=action['success'],motion_stop=action.get('motion_stop'),
            decision_before_capacity=action['started_seconds']<capacity,
            selected_score=action['score'],feasible_moves=action['candidate_diagnostics'].get('feasible_moves')))
    proof=[a for a in actions if a['all_predicted_new_outside'] and a['decision_before_capacity']]
    assert proof and proof[0]['action']==24
    late=[a for a in actions if a['started_seconds']>=315.]
    assert all(a['motion_success'] and a['motion_stop'] is None for a in late)
    track=[];previous=-math.inf
    with RAW.open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line);elapsed=row['stamp']-mission['started']
            if 315<=elapsed<=mission['elapsed_seconds'] and row['stamp']-previous>=.1:
                track.append(row['xyz'][:2]);previous=row['stamp']
    trajectory=dict(window_elapsed_seconds=[315.,mission['elapsed_seconds']],sample_interval_s=.1,
        min_xy=[min(p[k] for p in track) for k in (0,1)],max_xy=[max(p[k] for p in track) for k in (0,1)],
        path_length_m=sum(math.dist(a,b) for a,b in zip(track,track[1:])))
    inputs=[REPORT/'candidate-result.json',REPORT/'candidate-map.json',REPORT/'reference-metadata.json',
            REPORT/'long-run-summary.json',source,RAW]
    findings=dict(tested_commit=result['software']['git_commit'],experiment=result['experiment'],
        actual_scene_bounds_m=bounds,visibility_assumed_bounds_m=[-xlimit,-ylimit,xlimit,ylimit],
        scoring_boundary_occurrences=len(matches),never_observed_outside_scene_cells=len(outside_observed),
        first_capacity_warning_elapsed_seconds=capacity,pre_capacity_all_outside_example=proof[0],
        late_motion=trajectory,late_actions=late,
        confirmed_mechanisms=[
            'Visibility rewards include cells outside physical scene; motion execution still checks actual task region',
            'Same surface_view_v2 planner is used with both terrain classification strategies',
            'Ordinary action success records motion success even when measured map gain is zero',
            'Measured gain/residual are recorded but not used to down-rank failed view predictions',
            'History excludes nearby matching orientations, allowing different headings and nearby moves',
            'Wider boundary fallback is only attempted when ordinary candidate selection has no positive candidate',
            'Capacity rejection freezes the measured map later; it cannot explain the earlier out-of-scene rewards alone'],
        limits=[
            'No new simulation and no controller or planner mutation',
            'Outside predictions are per-action values, not unique cumulative area',
            'Final map is used only to verify outside cells were never observed; recorded expected_new is used for each historical decision',
            'No counterfactual claim that removing capacity alone or clipping rewards guarantees full-field coverage',
            'Existing resource-isolation limitation remains; geometric boundary mismatch is directly inspectable in saved decisions'],
        input_sha256={str(p.relative_to(ROOT)):sha(p) for p in inputs})
    (OUTPUT/'findings.json').write_text(json.dumps(findings,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    table=['| 动作 | 开始 / 秒 | 类型 | 预计新增 / m² | 其中场地外 / m² | 实际新增 / m² |\n',
           '|---:|---:|---|---:|---:|---:|\n']
    for a in late:
        table.append(f"| {a['action']} | {a['started_seconds']:.1f} | {a['kind']} | {a['predicted_new_m2']:.3f} | {a['outside_scene_predicted_new_m2']:.4f} | {a['actual_new_total_m2']:.4f} |\n")
    text=f'''# 右下角徘徊原因复核

2026-11-18。只复核已有八分钟预算实验，不重跑、不修改控制或选点代码。

结论：车辆并未机械卡死。自主选点将实际场地外的未知格计入预期测绘收益，使右下角长期保持虚假的吸引力；无收益动作未使规划器切换选区。约6分钟后的容量限制使地图不再接纳新帧，但边界评分错误在容量告警之前已经存在。

## 已确认的边界错误

实际V3场地为30×30米，范围`{bounds}`。`choose_view`的两个可见性分支仍以`[-25,-25,25,25]`作为外侧截断范围，而且调用时没有传入任务真实边界。车辆执行检查使用实际任务区域，因此车可以留在场地内，却被朝向场外的虚假观测收益吸引。

第24个动作在任务第{proof[0]['started_seconds']:.2f}秒选择，早于第{capacity:.2f}秒的容量告警。它记录的预计新增为{proof[0]['predicted_new_m2']:.3f} m²；其中{proof[0]['outside_scene_predicted_new_m2']:.4f} m²全部在实际场地外（三位小数取整差异已计入）。这证明不能只用容量满解释角落行为。

## 为什么不断重复

收益分数采用“预计新增和补采收益÷预计时间”，候选主要是附近1.5米、3米处与若干朝向。只要附近仍存在被高估的正收益候选，就不会启用更大范围的边界候选搜索。

历史排重只拦接近的位置与相似朝向，换方向或挪一点仍可入选。执行记录把到达/转向完成计为成功；普通动作即使实际新增为0，也只是记录预测残差，不据此降低这类视角的后续评分。容量到限后，消息还能继续发布旧图，这一循环也未被中止。

'''+''.join(table)+f'''
表中是每个动作的预测和实际新增，存在跨动作重叠，不能求和当作唯一面积。实际新增是整图新增，并不等于只在预测视野中的新增。

## 不是车轮无法转动

第315秒至结束的这些动作全部返回运动成功，没有`motion_stop`。独立Gazebo真值轨迹在此阶段移动约{trajectory['path_length_m']:.2f}米，范围X={trajectory['min_xy'][0]:.2f}～{trajectory['max_xy'][0]:.2f}、Y={trajectory['min_xy'][1]:.2f}～{trajectory['max_xy'][1]:.2f}米。是反复在角落选位置和朝向，而非物理上无法移动。

该选点层由原、新地形策略共用；新地形策略更容易向外扩展，本次走到边界后暴露了这个缺陷。原策略本轮在起点附近提前停车，未检验到这个边界行为。

## 修复应针对的地方

统一实际任务边界与候选/观测收益边界；普通动作应检查真实采集进度，并对持续低收益、失败预测触发重新选区；建图容量到限应有明确的任务状态和记录处理。这里没有实施这些修复，也不推断单独解除容量就一定能摆脱角落。

完整逐动作结果、范围及输入摘要见`findings.json`。已有八分钟测试的资源隔离限制保持不变；本复核依据几何边界、已保存目标和收益字段，不作为新的运行性能对照。
'''
    (OUTPUT/'README.md').write_text(text,encoding='utf-8',newline='\n')
    print(json.dumps(dict(actual_bounds=bounds,predictor_bounds=findings['visibility_assumed_bounds_m'],
        pre_capacity_example=proof[0],late_motion=trajectory),ensure_ascii=False))


if __name__=='__main__':main()
