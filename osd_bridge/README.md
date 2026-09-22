# osd_bridge — RK3576 HDMI OSD 桥接节点

订阅 apm_bridge (`custom_link_bridge`) 的遥测话题，渲染 Betaflight 风格 FPV OSD
（+ AI 检测 BBoxRect 框）到 RK3576 的 HDMI 输出（720x480p60 NTSC）。

```
apm_bridge 话题                       osd_bridge
  /mavros/imu/data      (200Hz BE) ──► 航向 / 水平仪
  /mavros/state         (10Hz)    ──► 模式横幅 / ARMED / 解锁计时 / home 记录
  /mavros/battery       (10Hz)    ──► 电压/电流/mAh/用量条/电量告警
  /mavros/gpsstatus/gpsraw    ──►   卫星数/对地速度
  /mavros/global_position/global ──► 经纬度 → home 方向箭头+距离
  /fpv/static_pressure  (100Hz BE) ─► 气压高度 (地面基准 EMA)
  /fpv/temperature_baro (100Hz BE) ─► 温度
  /fpv/low_level_feedback ──► 电池分级 (GOOD/LOW/CRITICAL 配色)
  /fpv/tracker_state / radar_state ─► tracker 横幅 + 检测框配色
  /fpv/tracker_trigger ──► 触发横幅 (TRIG ON/OFF/MISSION)
  /fpv/failsafe        ──► 失效保护告警 + 消息行
                                        │
                        Pipeline (渲染线程, 60fps) ──► RGA 三段合成 ──► DRM/KMS
                                        │                                   │
                     osd::layout (元素布局)        osd::hw (V4L2 相机/HDMI/RGA)
                     osd::core  (CPU 光栅 + BBoxRect)
```

双层合成：HUD 层（α=0/255，RGA 混入视频）+ 检测层（独立 VOP overlay plane，
半透明扫描合成，锁定目标逐像素 α 脉冲）——检测框永不破坏 HUD 文字。

## 依赖与构建（仅板上）

本包直接驱动 RK3576 硬件，**只能在板上编译运行**（x86 上构建 osd 栈的可移植
子集：`cmake -B build-x86 -DOSD_WITH_HW=OFF`，跑 `ctest`）。

本包随 ar0234_osd_hdmi 仓库一起部署（同仓库同置布局，`OSD_STACK_ROOT`
默认即父目录）。板上一次性准备：

```bash
# osd 栈 + 本包（hw/osd/layout 三库 + 桥接包同仓库）
cd ~/workspace && git clone <ar0234_osd_hdmi 仓库>   # 或 tar 同步
# librga SDK 位于 ~/workspace/librga (v1.10.6; root 下自动探测 /home/*/workspace/librga)
# ROS 工作空间里符号链接本包（colcon 会跟随）:
ln -s ~/workspace/ar0234_osd_hdmi/osd_bridge ~/workspace/fpv_ws/src/osd_bridge

cd ~/workspace/fpv_ws
colcon build --packages-select osd_bridge   # 一次 colcon 出全部四个目标
```

## 运行

```bash
sudo systemctl stop lightdm      # 释放 DRM master（必须）
source ~/fpv_ws/install/setup.bash

# 1) apm_bridge (SITL 在板上: transport tcp; 真机: 串口)
ros2 launch apm_bridge custom_link.launch.py transport:=tcp tcp_port:=5763 &

# 2) OSD (另一个终端; 需要 root 拿 DRM master + V4L2 + dma_heap)
sudo -E env PATH=$PATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
  ros2 launch osd_bridge osd_bridge.launch.py boxes.sim:=true

# 板上无相机时 (灰帧台架模式):
ros2 launch osd_bridge osd_bridge.launch.py camera:=false boxes.sim:=true

# 抓快照 (三件套: back*.xrgb + hud*.rgba + ovl*.rgba 到 /tmp):
ros2 service call /osd_bridge/snapshot std_srvs/srv/Trigger {}

# 轮换 tracker_state 看框变色 (白→青→绿+锁定→红):
test/sim_boxes.sh

sudo systemctl start lightdm    # 收尾恢复桌面
```

快照回开发机查看（沿用 osd_demo 的 PIL 方法）：

```bash
scp -O firefly@<board>:/tmp/bridge_back_000180.xrgb .
python3 - <<'EOF'
from PIL import Image
Image.frombytes("RGB",(720,480),open('bridge_back_000180.xrgb','rb').read()[:720*480*4],'raw',"BGRX").save('snap.png')
EOF
```

## BBoxRect 注入接口（检测器对接预留）

`osd_bridge::Pipeline::set_boxes(std::vector<osd::BBoxRect>)` —— 坐标默认为
**相机像素坐标**（内部经 `osd::layout::CanvasMap` 做 letterbox 映射；注入画布
坐标则置 `boxes.canvas_space: true`）。框颜色逐帧可变；未指定颜色时按
tracker_state 策略配色（0 白 / 1 青 / 2 绿+锁定 / 3 红）。后续检测节点按此
seam 接入（ros 话题映射另议）。

## 参数

见 [parameters/osd_bridge_parameters.yaml](parameters/osd_bridge_parameters.yaml)。
要点：`camera.sensor` 三选一、`battery.capacity_mah` 与 apm_bridge 保持一致、
`boxes.sim` 验证模式、`baro.ground_ref` 高度基准。

## 已知事项

- **DRM master 互斥**：lightdm 必须停；osd_demo 与本节点不可同时跑。
- **权限**：DRM master + V4L2 + /dev/dma_heap 都要 root（`sudo -E` 保环境）。
- 水平仪 pitch/roll 符号沿用桥接节点四元数，**台架实测时若方向反了**在
  `bridge_node.cpp onImu` 处取反。
- apm_bridge 的 `mah` 计数单位 (int16) 若与固件不符，改 `handleSlow` 中
  `kMahPerCount` 一处常量。
