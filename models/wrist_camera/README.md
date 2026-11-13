# Gemini 336 腕部模型

- 相机本体 `gemini336_camera_link.STL` 来自 Orbbec 官方 [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/ffba347cadc96d0457a91af59fc06cdf86052728/orbbec_description/meshes/gemini335_336)，对应 Gemini 335/336 共用模型，提交 `ffba347cadc96d0457a91af59fc06cdf86052728`；SHA-256 `d82a0f210c174f7de902bf79690d9565756cbe39114499fc675e3d36973108ee`。主体可视原点取自该提交的 `orbbec_description/urdf/gemini_335_336.urdf.xacro`；Apache-2.0 许可证见 `licenses/`。
- 当前支架来自 [reBot-DevArm 的 D435/Gemini 2 V1 STEP](https://github.com/Seeed-Projects/reBot-DevArm/blob/008f5c173fe58ae6d6f5ec82f8b3dae1d937d652/hardware/camera-mounts/b601-camera-mounts/V1/D435_Gemini2_Mount.step)，Git blob `df04352816f5ae9979c09327cc40758182c3df90`，SHA-256 `38aa815b210ee4fe73275a46bb305ebb92315b5b0ae1167a030ad5bccbae1c35`。原始 STEP 保存在 `source/`；`meshes/D435_Gemini2_Mount_V1.stl` 使用 CadQuery 2.7.0 转换，线性与角度网格容差均为 0.1。STL 单位为毫米，URDF 中使用 0.001 缩放为米。按底座两孔重合、同侧安装面贴合进行名义对齐：相对于旧支架坐标系，V1 网格平移约 `[0.20, -44.72, 34.52]` mm；底座下表面从 V1 的 z=-22.00 mm 对齐到旧模型的 z=12.52 mm。`meshes/mount.stl` 是此前的旧支架网格，保留作对照，当前 URDF 不使用。CERN-OHL-W-2.0 许可证见 `licenses/`。
- `gemini336.urdf` 的相机位姿使用底座对齐结果，并依据 V1 与旧支架的相机安装面及相机安装孔进行近似装配。V1 安装面相对旧支架旋转约 15°；光学坐标因相机随安装面移动而变化。CAD 测量、实物公差、相机孔位及固定面选择均可能引入偏差，真实相机位姿尚未经实测验证。此模型不作为碰撞模型。

- [本项目作者制作的 Gemini 2 V1 平视版支架](gemini2-v1-level/)单独提供 STEP、STL 和预览图；其几何视场校核尚未实机试装。当前 `gemini336.urdf` 仍使用原 V1 网格及名义装配，未自动切换到平视版。
