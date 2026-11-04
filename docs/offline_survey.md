# 离线地形勘测流程

已有[实时接收与工作台入口](live_survey.md)：可边接收边绘图，并把当前收到的数据前缀导出为本流程的成果包。离线入口继续用于独立复算与回归。

这套入口完成四件事：定义任务、自动处理观测、说明地图质量、核对观察与移动配置。只需Python 3.10及以上与NumPy；不需要硬件、运行中的ROS或Gazebo。它复用现有`TerrainMap`、深度投影和DEM导出，车辆和重建基线没有切换。评分规则为`mapping-acceptance-v1.2`：覆盖率仅作统计，不设通过门槛；仍检查已测地表精度、参考证据与运行故障，详见[评价规则](mapping_acceptance.md)。

## 运行一个完整示例

在项目根目录执行，输出目录必须是新目录：

```powershell
python scripts/offline_survey.py create-example tmp/survey-example-input
python scripts/offline_survey.py validate tmp/survey-example-input/task.json
python scripts/offline_survey.py run tmp/survey-example-input/task.json --output tmp/survey-example-result
```

打开`tmp/survey-example-result/report.html`即可查看报告，无需启动网页服务。示例是明确标注的合成平面，故意包含缺测、12 cm高度偏差和多层混合单元；还有一帧位于任务截止之后。图上的合格格只代表相对该数学参考达标。

结果包自带复算任务，保留原始适配输入及哈希：

```powershell
python scripts/offline_survey.py run tmp/survey-example-result/task-replay.json --output tmp/survey-example-replayed
```

## 任务文件

`create-example`生成可编辑的完整`task.json`。任务使用独立格式`mooncar-survey-task-v1`，不会替换旧仿真任务。

| 字段 | 含义 |
| --- | --- |
| `id` | 本轮任务名称 |
| `coordinates` | 会话坐标名、米制单位、ENU轴向和局部高程基准；不猜测海拔或测地坐标 |
| `region` | 首版支持矩形`bounds_m=[xmin,ymin,xmax,ymax]`；按半开区域内的格心纳入，格网原点固定为会话坐标零点 |
| `time` | 明确`simulation`、`monotonic`或`utc`时钟，以及同一时钟下的绝对起止秒；仅使用闭区间内观测 |
| `grid` | 地表格长`cell_m`和体素`voxel_m`，均为米；沿用基线的体素不大于半格限制 |
| `targets` | 高度、坡度、几何达标率目标及依据；不设覆盖目标。旧`minimum_coverage`字段兼容读取但不参与判断；精度目标仍为暂定，不代表硬件能力 |
| `source` | 明确`synthetic`、`gazebo`、`physical`或`unknown`，并记录标定与位姿版本。属于来源声明，不是工具颁发的认证 |
| `input` | 选择`folder-v1`或`observations-v2`及路径 |
| `reference` | 可选的独立参考路径；无参考时准确度保持未知 |
| `configuration` | 可选配置快照路径；缺少时明确标记未核对 |

任务范围、活动范围和评价范围不再通过隐含默认值互相替代。本离线入口按任务矩形计数，不控制车辆或更改旧策略的活动半径。多边形、圆形及跨会话自动配准不在v1支持范围内，会明确拒绝。

## 输入现有NPZ档案

支持已有`mooncar-observations-v2`档案，原档案不改写。任务输入写为：

```json
{"format":"observations-v2","path":"session.npz","contract":"input-contract.json"}
```

旁路契约需要声明档案缺少的上下文，且必须与任务完全对应：

```json
{
  "format":"mooncar-survey-input-v1",
  "frame":"map",
  "unit":"m",
  "axes":"ENU",
  "vertical_datum":"会话局部原点；无物理高程基准",
  "clock":"simulation",
  "source_kind":"gazebo",
  "calibration_version":"历史档案；设备标定未核验",
  "pose_version":"档案内逐帧估计位姿"
}
```

`frame`必须与NPZ内部元数据一致；其他契约字段与任务对应。旧档案缺少相机内参或独立参考时，不凭自由文字推断它已完成标定。本轮使用181帧既有Gazebo档案验证兼容性，结果中的几何准确度保持未知。

## 输入规范化观测文件夹

目录内包含`manifest.json`和数组文件。manifest的`contract`结构同上，`frames`内每帧需要递增的`stamp_s`以及传感器到会话坐标的`pose`：

```json
{
  "stamp_s": 1.0,
  "points_file": "points-001.npy",
  "pose": {"translation_m":[0,0,1],"quaternion_xyzw":[0,0,0,1]}
}
```

点文件为不含pickle的N×3数值数组，单位为传感器坐标米。可选`robot_pose`与传感器pose格式相同。相机与车体位姿必须区分，不默认两者重合。文件夹内引用只允许相对路径，拒绝目录逃逸。

深度帧改用`depth_file`。manifest还需提供相机：

```json
{
  "width":40,"height":40,"encoding":"16UC1",
  "intrinsics":[40,40,20,20],"pixel_offset":0,"rectified":true
}
```

`intrinsics`为`[fx,fy,cx,cy]`，示例数值仅演示格式。只接受已校正针孔深度：`16UC1`为uint16毫米，`32FC1`为float32米，数组尺寸／类型必须匹配声明。沿用当前基线投影的stride=4和0.1～6 m有效光轴深度，不补齐无效像素，不支持直接输入未去畸变双目图。`pixel_offset`必须显式给定。

零有效点或区域内不足三点的帧保留原因；少量点出现过的格子标为观测不足，而非伪造平地。全部输入时间戳都核验，截止之后的损坏或重复时间戳也不会被静默忽略。

当前沿用单会话上限：5000帧、500万原始点，每帧最多10万投影点；旧NPZ仍受既有读取容量限制。任务最多10万个格心，输出不覆盖已有目录或输入材料。

## 质量与参考

报告区分未观测、无法有效拟合、已拟合但准确度未知、相对参考达标、误差超限、参考声明的表示范围外、参考证据不足。所有任务格子写入CSV；缺测不从分母删除。DEM会补足声明任务格网的范围，以NODATA保留缺测及拟合失败。

无参考时，可报告原始观测覆盖和有效拟合覆盖，P、S及误差为空；这些覆盖比例不代表准确勘测完成率。采样充分度和拟合残差也不作为测量误差界限。

参考采用`mooncar-survey-reference-v1`：要求相同坐标／高程基准、格长和完整任务格心集合，每格沿用现有评分器的`ground`、`terrain`、`edge_or_complex`、`no_reference`类别及高度／坡度定义。不能只提交已测部分来缩小评价分母。

参考的`source`声明`synthetic`、`gazebo_world`或`independent_measurement`及说明。`alignment`记录对应方法与状态：

- `unverified`：不能输出几何通过或误差达标颜色。
- `synthetic_exact`：只允许观测与参考都为明确的合成数据。
- `provided_verified`：要求输入者提供对应说明及`evidence_file`、SHA256。工具核对文件完整性与坐标格网，报告仍说明这是所给参考及资格声明，不冒充独立物理认证。

参考不会进入观测点云或填补缺测。相对参考的评分继续使用已有公式和分地形门槛；安全资格始终单列，运行流程完成不代表几何或安全验收通过。

## 配置核对

先生成当前项目源码声明的快照：

```powershell
python scripts/offline_survey.py project-config --output tmp/project-survey-config.json
```

它只读提取规划器视场、相机XY代理、移动／转动耗时代理、R5相机视场及导航声明。无法识别的表达式和缺少的实际TF／碰撞包络保持未知，不猜数值。快照带来源文件哈希；源码已变时会标记。

也可以提供`mooncar-survey-configuration-v1`文件，含`expected`、`observed`和`scope`。scope明确为`source_declarations`、`runtime_snapshot`或`synthetic_fixture`。支持比较：

- `horizontal_fov_deg`，或observed中的`camera_info={width,height,k}`；只按已校正零斜切针孔模型估算连续图像边缘视场。
- `camera_offset_xy_m`，双方均相对车体基准，以米表示。
- `linear_speed_m_s`与`angular_speed_rad_s`，明确比较预测代理与相应控制配置，不当作已测地速。
- `footprint_size_m`，长度／宽度以米表示；缺少独立包络时不把导航自己的声明当作验证。

结果分为consistent、difference、unknown；比较容差分别为1°、1 cm、0.01 m/s、0.01 rad/s和1 cm，仅用于差异筛查。它们不修改运行参数、项目目标或安全余量。一致不等于物理正确；差异也不自动构成缺陷。

当前源码对照会指出60°／73°视场及0.20／0.16 m/s速度代理差异；真实运行中的相机位置、转动速度和碰撞包络仍需对应快照，本轮没有拿源码值代替这些观测。

## 结果包

`report.html`和`quality-map.svg`可离线查看；`cells.csv`保留逐格状态；`observations.npz`、`cloud.ply`、`dem.asc`和`surface.obj`保留地图成果。`dem.json`包含任务范围、坐标、单位和标定／位姿版本。

`frame-log.json`列出每帧使用、超出时间窗口或任务内点不足的原因。`input-index.json`与`inputs/`保存来源和复制件，`task-replay.json`用于重算；`run-metadata.json`记录代码指纹和运行版本，`checksums.json`记录结果文件哈希。源文件处理前后及复制时均核对，不覆盖原数据。

本入口只做离线任务处理和报告，不发布控制指令，不加载或修改物理库。真实设备精度、完整勘测任务可靠性及实车安全仍需各自验证。
