# Terrain v2 论文存档（2026-09-30）

本目录位于时间戳归档 `reports/evidence/terrain-v2/20260930T170028+0900/`。PDF 只供本地阅读，不纳入 Git；Git 保存本索引、来源链接与校验值。

以下 PDF 与 `MoonCar-mapping/ros2_ws/src/mooncar_gazebo/README.md` 中的场景依据对应。已核对 PDF 首页标题和页数。论文中的实测或统计量只为场景参数提供依据；合成地形不等于真实月面重建。

| 论文 | 本地文件 | 来源 | 页数 | 用途 |
| --- | --- | --- | ---: | --- |
| Ding et al. (2024), *Lunar rock investigation and tri-aspect characterization of lunar farside regolith by a digital twin*, DOI: 10.1038/s41467-024-46233-8 | [Ding_2024_NatureCommunications.pdf](Ding_2024_NatureCommunications.pdf) | [Nature Communications PDF](https://www.nature.com/articles/s41467-024-46233-8.pdf) | 15 | 坡度上限和相邻陨石坑直径的参考 |
| Wu et al. (2021), *Centimeter-resolution topographic modeling and fine-scale analysis of craters and rocks at the Chang'E-4 landing site*, DOI: 10.1016/j.epsl.2020.116666 | [Wu_2021_EPSL.pdf](Wu_2021_EPSL.pdf) | [香港理工大学机构库 PDF](https://ira.lib.polyu.edu.hk/bitstream/10397/90959/1/1-s2.0-S0012821X20306105-main.pdf) | 14 | 坑深径比、着陆区石块面积占比参考 |
| Ding et al. (2020), *Fragments Delivered by Secondary Craters at the Chang'E-4 Landing Site*, DOI: 10.1029/2020GL087361 | [Ding_2020_GRL.pdf](Ding_2020_GRL.pdf) | [香港理工大学机构库 PDF](https://ira.lib.polyu.edu.hk/bitstream/10397/113266/1/Ding_Fragments_Delivered_Secondary.pdf) | 9 | 实测小坑尺度及 DTM 说明；原始 DTM 另存于 mapping 仓库 |
| Guo et al. (2021), *Millimeter- to Decimeter-Scale Surface Roughness of the Moon at the Chang'e-4 Exploration Region*, DOI: 10.1029/2021GL094931 | [Guo_ChangE4_SurfaceRoughness.pdf](Guo_ChangE4_SurfaceRoughness.pdf) | [香港理工大学机构库 PDF](https://ira.lib.polyu.edu.hk/bitstream/10397/113260/1/Guo_Millimeter_Decimeter_Scale.pdf) | 14 | 对照小尺度高程粗糙度；公开的更宽范围 DTM 已存于 mapping 仓库，但因中央缺测尚未嵌入场景 |
| Watkins et al. (2019), *Boulder Distributions Around Young, Small Lunar Impact Craters and Implications for Regolith Production Rates and Landing Site Safety*, DOI: 10.1029/2019JE005963 | **期刊正文尚未取得** | [AGU/Wiley 正文](https://agupubs.onlinelibrary.wiley.com/doi/10.1029/2019JE005963) | — | 石块在坑缘附近分布的定性参考；该论文研究的坑远大于当前场景 |
| Watkins et al. (2018), *Boulder Distributions Around Young, Small Lunar Impact Craters* | [Watkins_2018_LPSC_abstract.pdf](Watkins_2018_LPSC_abstract.pdf) | [LPSC 会议摘要 PDF](https://www.hou.usra.edu/meetings/lpsc2018/pdf/1201.pdf) | 2 | 与 2019 年期刊文相关的公开摘要，**不能替代期刊正文** |

2019 年 Watkins 论文的期刊 PDF 接口及 ResearchGate 下载接口在本机返回访问限制；浏览器页面也显示安全验证，因此未保存伪造的“全文”。可通过上面的期刊正文链接在线阅读或在有机构访问权限的浏览器中下载。

## SHA-256

```text
Ding_2020_GRL.pdf                 a7c83aa46967cc0307abec148e25247a92a4753ed73230de6655e7e1a60fc8ad
Ding_2024_NatureCommunications.pdf 06a272d6392817e0869f80ec58f5e83d32850a6cdf990e8a522c7365227a5aca
Guo_ChangE4_SurfaceRoughness.pdf   e49df4ff9c4e8b2d95845104355266a28ebc8254bdcc6260f8af810a59bff693
Watkins_2018_LPSC_abstract.pdf    7f0731cf6b9451b3eea807cb866f4e1fa4230ea4038ca2fcfc904b1a10
Wu_2021_EPSL.pdf                  d89355cb9c9329f33ac668f81f86c036bc238866fdd72a146277156826101021
```
