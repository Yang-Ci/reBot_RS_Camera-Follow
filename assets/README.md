# 集成资源及来源

本目录和 `vendor/` 包含运行所需的项目内资源，不要求额外克隆源码仓库。

- `robot/description/urdf/ReBot_Arm_RS.urdf`：控制与重力计算模型，来自 Seeed-Projects/reBotArm_control_py，提交 `3524ebc5316985d6d74fe30269ae771fea715507`。仅将网格引用改为指向本目录共用的 `meshes_rs/`；关节、动力学和惯量未改动。
- `robot/description/urdf/ReBot_Arm_RS_visual.urdf` 和 `robot/description/meshes_rs/`：网页模型，来自 Yang-Ci/ReBot_Arm_DigitalTwin_RS，提交 `68ff6e45701ebee5d580066260700f907457423f`。
- `wrist-cameras/`：Gemini 2 模型、参考 URDF 及其来源信息；详细来源见 `source.json`，相关许可证位于 `licenses/`。
- `../vendor/rebot_control/`：LAN-GER/rebot_control 控制器和配置，提交 `a901ec109ad99d7140c9b2110c618f54a9107ae9`。
- `../web/lib/mediapipe/`：Google `@mediapipe/tasks-vision` 1.0.1 浏览器运行资源，Apache-2.0，许可证在该目录中。

控制器和 RS 模型的上述源仓库中未找到明确的许可证文件；发布或再分发前应确认权利和授权。网页资源仅用于可视化，实际运动仍依赖真机配置、安装和安全检查。
