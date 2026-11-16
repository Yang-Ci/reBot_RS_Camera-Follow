# Gemini 336 腕部模型

- 相机本体 `gemini336_camera_link.STL` 来自 Orbbec 官方 [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/ffba347cadc96d0457a91af59fc06cdf86052728/orbbec_description/meshes/gemini335_336)，对应 Gemini 335/336 共用模型，提交 `ffba347cadc96d0457a91af59fc06cdf86052728`；SHA-256 `d82a0f210c174f7de902bf79690d9565756cbe39114499fc675e3d36973108ee`。主体可视原点取自该提交的 `orbbec_description/urdf/gemini_335_336.urdf.xacro`；Apache-2.0 许可证见 `licenses/`。
- 当前支架来自 [reBot-DevArm 的 D435/Gemini 2 V1 STEP](https://github.com/Seeed-Projects/reBot-DevArm/blob/008f5c173fe58ae6d6f5ec82f8b3dae1d937d652/hardware/camera-mounts/b601-camera-mounts/V1/D435_Gemini2_Mount.step)，Git blob `df04352816f5ae9979c09327cc40758182c3df90`，SHA-256 `38aa815b210ee4fe73275a46bb305ebb92315b5b0ae1167a030ad5bccbae1c35`。原始 STEP 保存在 `source/`；`meshes/Gemini2_D435i_Mount_Original.stl` 使用 CadQuery 2.7.0 转换，线性与角度网格容差均为 0.1。STL 单位为毫米，URDF 中使用 0.001 缩放为米。按底座两孔重合、同侧安装面贴合进行名义对齐：相对于旧支架坐标系，V1 网格平移约 `[0.20, -44.72, 34.52]` mm；底座下表面从 V1 的 z=-22.00 mm 对齐到旧模型的 z=12.52 mm。`meshes/mount.stl` 是此前的旧支架网格，保留作对照，当前 URDF 不使用。CERN-OHL-W-2.0 许可证见 `licenses/`。
- `gemini336.urdf` 的相机位姿使用底座对齐结果，并依据 V1 与旧支架的相机安装面及相机安装孔进行近似装配。V1 安装面相对旧支架旋转约 15°；光学坐标因相机随安装面移动而变化。CAD 测量、实物公差、相机孔位及固定面选择均可能引入偏差，真实相机位姿尚未经实测验证。此模型不作为碰撞模型。

- [Gemini 2 / Gemini 336 / D435i 共用支架](gemini2-gemini336-d435i/)可供 Gemini 2、Gemini 336 和 D435i 共用。`gemini2_level.urdf`、`d435i_level.urdf`、`gemini336_level.urdf` 同时用于网页显示和跟随计算，采用相同的支架模型和各相机的光学坐标；D435i 模型的相机外壳以包络盒表示。
- 平视版的承载部相对于原 V1 绕 CAD X 轴旋转 -14.999820°，旋转中心为 `[0, 40, -11]` mm；URDF 根据 V1 网格的装配平移转换旋转中心。Gemini 2 与 D435i 从数字孪生旧支架位姿转换到 Gemini 336 的 V1 名义装配，再施加该平视变换。此转换基于安装面的名义对齐，并非实测手眼标定；特别是 D435i 实际安装孔位与光心需要实机核对。
- `gemini336.urdf` 保留作为旧 V1 装配参考，不再是网页或默认跟随使用的模型。平视模型和相机实际安装仍需低速验证，不能保证碰撞安全。 可使用 `tools/build_level_camera_models.py` 和项目内 `assets/wrist-cameras/urdf/` 的源模型重建这三份 URDF。

## Gemini 305 居中平视支架

`gemini305_centered.urdf` 为居中平视支架的名义装配。相机外壳使用近似方盒；控制用光学中心取两枚 CAD 镜片中心的中点，并非实测 RGB 光心，机械臂侧装配使用 V1 夹持接口的 CAD 到旧安装坐标的平移 `[0.20, -44.72, 34.52]` mm，光学位姿同步应用相同变换。该位姿和夹爪／支架的实际间隙均未经实机验证；不得将其视为标定结果。

`meshes/Gemini305_D405_Mount.stl` 对应的源文件和重建程序见 `gemini305-d405/source/`，许可为 CERN-OHL-W-2.0，见 `gemini305-d405/LICENSE.txt`。原支架来自 Seeed-Projects/reBot-DevArm，Git blob `812120494a7e6145c5971e24d4d27c8e04603090`；相机光学轴位置依据 Orbbec Gemini 305 官方 CAD V1.1 的装配近似。
