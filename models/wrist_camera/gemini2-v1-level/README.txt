Gemini 2 V1 平视版支架（2026-11-13）

适用来源：reBot-DevArm 的 B601 腕部相机支架 V1。
支架供 Gemini 2、Gemini 336 和 D435i 共用；机械臂侧 RS / DM 共用接口保持原样。
当前视场几何校核使用 Gemini 2 与 B601-RS；各相机的实际安装和视野需实装确认。

文件
Gemini2_V1_Level.step：可编辑 CAD 实体，单位 mm。
Gemini2_V1_Level.stl：用于切片的封闭三角网格，单位 mm。
Gemini2_V1_Level_preview.png：修改前后及 RS 夹爪视场对比图。
原 V1 模型归档在上级 source/D435_Gemini2_Mount_V1.step。

修改内容
原 V1 相机固定面下倾约 14.99982°，现调整为相对夹持接口基准的名义 0°。
保持机械臂侧夹持接口不变；相机侧承载结构做刚性旋转，保留孔位、孔径、安装槽等几何。
只重建夹持部与相机承载部之间的连接段。未增加额外抬高量。
平视方向以末端安装基准为参考；整个相机的空间朝向仍随机械臂腕部姿态变化。

验证
STEP：有效且只有一个实体；导出后重新导入通过。
STL：6434 个三角形，边均由两个三角面共享，封闭且无退化三角形。
机械臂侧接口与原模型的体积差为零。
相机接口区域逆旋转后，顶点、体积、表面积及面数量与原模型一致。

RS 视场校核采用仓库的相机安装参考系与夹爪网格。
相机按 V1 的背面、底面支承平面对齐，名义装配间隙为 0.15 mm。
深度垂直视场按 66° + 3° 公差，即上下各 34.5° 检查；RGB 视场更窄。
闭合、半开和全开时，夹爪均在该视场下沿之外。
最接近视场的夹爪仰角约 -41.72°，与下沿约有 7.22° 的几何余量。
仓库装配参考系含约 0.68° 的微小俯角；相机固定面的 CAD 名义角度为 0°。
上述为模型校核，尚未实机试装、测量或拍摄验证。

官方视场来源
https://www.orbbec.com/products/structured-light-camera/gemini-2/

原模型来源
https://github.com/Seeed-Projects/reBot-DevArm/blob/008f5c173fe58ae6d6f5ec82f8b3dae1d937d652/hardware/camera-mounts/b601-camera-mounts/V1/D435_Gemini2_Mount.step
Git blob: df04352816f5ae9979c09327cc40758182c3df90

CAD 派生模型沿用 CERN-OHL-W-2.0，许可见上级 licenses/CERN-OHL-W-2.0.txt。
对比图使用 Orbbec 的相机模型，Apache-2.0 许可见上级 licenses/Apache-2.0.txt。
