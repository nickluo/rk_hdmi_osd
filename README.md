# AR0234 / SC233HGS → RGA 硬件 OSD 叠加 → HDMI NTSC 输出 (RK3576)

在 RK3576 (Firefly, Ubuntu 22.04, kernel 6.1.141-rt52) 上实现并验证：

```text
VEYE RAW-MIPI-AR0234M (MIPI CSI, GREY 1920x1200@60fps, mvcam → rkcif)
   │  V4L2 MMAP 采集 + DMABUF 导出（零拷贝）
   ▼
RGA op1  Y400 → NV12 720x450 缩放（Y 通道逐字节直通）
   │  CPU: UV 平面填 128（162KB，灰度无色度）
   ▼
RGA op2  NV12 → BGRX8888（BT.601 FULL range，灰度恒等映射），letterbox 定位
   ▼
RGA op3  HUD 层 (720x480 RGBA8888, 只含 α=0/255) 硬件 alpha 混合
   ▼
DRM/KMS 双缓冲页翻转 ──────────────► 主 plane → HDMI-A-1 @ 720x480p60
                                        ▲
检测层 (CPU 直画 opaque 图形 +          │ VOP2 扫描时硬件半透明合成
        per-pixel α 脉冲) ──► overlay plane (Esmart2, plane α=235,
                                      Coverage, zpos=1)
```

## SC233HGS 支持（HZ-RK3576 BSP）

同一 `hw::MipiCamera` 现在支持两颗传感器，`--sensor` 选择：

| | AR0234M（默认 `--sensor ar0234` 恢复旧行为） | SC233HGS（默认） |
| --- | --- | --- |
| 驱动 | VEYE mvcam（vendor subdev） | 内核 in-tree 驱动（BSP 单目定制） |
| 模式 | Y8_1X8 1920x1200@60 | Y10_1X10 1920x1200@60（双目：`--cam 0` / `--cam 1`） |
| 采集格式 | `V4L2_PIX_FMT_GREY`（RGA Y400 直入） | `V4L2_PIX_FMT_Y10`（LE 位流打包，见下） |
| Y10 → Y8 | — | CPU 解包到 dma-heap 灰度 scratch 再走 RGA |
| 管线定位 | 固定 `/dev/video0` + `media-ctl` 实体名 | 扫描 `/dev/media0..7`：含 `m0X_b_sc233hgs` 实体的 graph → 同 graph 的 `stream_cif_mipi_id0` 节点 |

**rkcif 的 `Y10 ` 内存布局（真机实测）**：既不是 V4L2 规范的 16bit 容器，也不是
MIPI 式"2bit 尾字节"分组，而是**连续小端位流**（字节内 LSB 在前，每样本 10bit）。
每 5 字节 4 像素：`p0=b0|(b1&3)<<8; p1=b1>>2|(b2&0xf)<<6; p2=b2>>4|(b3&0x3f)<<4;
p3=b3>>6|b4<<2`。行有效字节 = w×10/8（1920→2400），stride 对齐到 256（2560），
行尾 padding 为 0。验证方法：`test_cam` 打印布局探测，或抓帧后按上述公式解码
（MIPI 式解出的是噪声，此式解出的是清晰场景）。注意传感器模组安装方向可能使
图像旋转 90°（演示画面所见即传感器原生方向，如需转正由显示侧处理）。

### 帧时间戳（SOF → Epoch）

rkcif 驱动在 **SOF 硬中断**里用 `ktime_get_raw_ts64()` 给每帧打
CLOCK_MONOTONIC_RAW 时间戳（BSP 已定制），随 `v4l2_buffer.timestamp`
（µs）交付。`hw::Sof2Epoch`（`hw/sof2epoch.h`）把它映射到 Epoch：

```cpp
hw::CameraBase::Frame f = cam.capture();
f.sof_raw_ns;   // 驱动 SOF 打点，CLOCK_MONOTONIC_RAW 域
f.epoch_ns;     // sof2epoch 线性模型映射的 CLOCK_REALTIME（ns）
f.sequence;     // 驱动帧序号
```

模型 `epoch = rt_anchor + rate*(raw - raw_anchor)`：RAW/REALTIME/RAW 三明治
采样定锚，后台线程周期重校准，>500ppm 速率离群判定为系统时间 step 只换锚；
`adjtimex` 的 NTP 频率修正作 rate 先验。x86 实测转换误差 < ±2µs（受 v4l2
µs 截断与 ISR 抖动兜底）。OSD 时钟槽显示的即当前帧 SOF 的 Epoch 时间。

检测框是持续移动的特殊元素：独立 overlay plane 使其与 HUD 层**永不互相破坏**，
压过 HUD 文字/分划时 VOP 硬件做真半透明叠加（文字透出），锁定目标时 per-pixel
alpha 脉冲（约 3.3Hz，无任何 per-frame ioctl）。

## 实测结果

| 指标 | 数值 |
| --- | --- |
| 帧率 | **59.76 fps 持续 60 秒**（3600 帧，vblank 同步上限 59.94） |
| RGA 硬件耗时 | 0.93 ms/op × 3 op ≈ 2.8 ms/帧 |
| RGA 硬件占用证据 | /proc/interrupts 中 rga2 中断 ~107 次/秒（运行时实测增量） |
| 灰度色彩 | 像素级验证 R=G=B（异常像素 0.43%，全部为红色分划线本身） |
| NTSC 模式 | 内核日志 `Update mode to 720x480p60 ... dclk: 27000000`，PHY 锁定 |
| 稳定性 | 无内存泄漏迹象，Ctrl-C 干净退出，governor 自动恢复 |
| CPU 占用（单核） | 旧版单文件 OSD 8.6%；**libosd 全功能版 5.5%**（元素更多反而更省） |

## libosd —— OSD 库（`osd/osd.h`）

参考 Betaflight osd.h 的元素体系设计的 FPV/AI 拦截 OSD 库，输出 RGBA8888
dma-heap 层供 RGA `imblend` 硬件混合：

```cpp
osd::Layer osd;
osd.init(720, 480);                       // cached dma-heap + RGA import

osd.begin_frame();
osd.draw_bbox({.rect={x,y,w,h}, .label="UAV", .tag="T01",
               .conf=0.97, .color=osd::kRed, .locked=true});
osd.draw_text(24, 10, "ALT 102M", {.color=osd::kWhite});
osd.draw_home_arrow(360, 58, angle_deg, osd::kAmber);
osd.draw_bar(20, 350, 150, 12, link_frac, kGreen, kBlack);
osd.draw_warning(600, 430, "LOW LINK", osd_blink);
osd.draw_crosshair(360, 240, osd::kRed);
osd.end_frame();                          // 有绘制才 DMA_BUF_SYNC(END)

IM_STATUS st = imblend(osd.rga_buf(), fb, IM_ALPHA_BLEND_SRC_OVER, 1);
```

**元素清单**（Betaflight 对应 + AI 拦截新增）：

| libosd | Betaflight osd.h 对应 | 说明 |
| --- | --- | --- |
| `draw_crosshair` | OSD_CROSSHAIRS | 十字+圆环+中心点 |
| `draw_home_arrow` | OSD_HOME_DIR | 归航方向箭头（任意角度） |
| `draw_warning` | OSD_WARNINGS | 闪烁告警（~3.3Hz 内置相位） |
| `draw_bar` | OSD_LINK_QUALITY / OSD_MAIN_BATT_USAGE | 链路/电量条 |
| `draw_text` + TextSlot 模式 | OSD_FLYMODE / OSD_ALTITUDE / OSD_RTC_DATETIME / OSD_TIMER 等 | 变更驱动重绘 |
| `draw_bbox` | （新增） | **AI 检测框（BBoxRect）**：角标/全框、标签+置信度+track id、威胁配色、锁定脉冲菱形+外圈；颜色逐帧可变 |
| `draw_horizon` | （新增） | 俯仰梯水平仪（配 `draw_crosshair`） |
| 模式横幅 | OSD_FLYMODE | SEARCH/TRACK/INTERCEPT |

**低 CPU 设计**：字形缓存（字符×颜色预渲染含描边 → 绘制退化为行 memcpy）、
脏标记（end_frame 无绘制则跳过 cache clean）、OSD 只被 CPU 写故只 clean 不
invalidate、检测框每帧仅清旧足迹区域。

**对接真实检测器**：相机坐标 → OSD 画布映射
`x_osd = x_cam*720/1920`，`y_osd = y_cam*450/1200 + 14`（letterbox 区域）。

## 关键坑与解法（本板实测）

1. **`wrapbuffer_handle` 是宏，第 4 参是 format，stride 是可变参**
   `wrapbuffer_handle(h, w, h, fmt, wstride, hstride)`。按 `wrapbuffer_handle_t`
   的顺序传参会把 stride 填进 format 字段，报 `dst invaild format [0x2d0]`
   （0x2d0=720=被误当 format 的 wstride）。**直接调用非宏的 `wrapbuffer_handle_t` 最安全。**

2. **RGA2-E (RK3576, hw 3.e.19357) 的 Y400→RGB 直转是坏的**：G 通道恒 ~130，
   画面绿/品红偏色（与 CSC 标志无关）。解法：三段链
   `Y400→NV12`（Y 直通）→ `UV=128 CPU 填充` → `NV12→RGB + IM_YUV_BT601_FULL_RANGE`
   （实测灰度恒等：Y32→RGB(32,32,32) 逐值相等）。

3. **`drmHandleEvent` 内部自己做 `read()`**：之前手动 read 会把事件数据偷走，
   它随后在 `drm_read` 里永久阻塞。正确用法：`poll()` → `drmHandleEvent()`。

4. **DRM XRGB8888 ↔ RGA BGRX_8888**：RGA 格式命名是内存字节序（大端命名），
   DRM XRGB8888 内存布局 B,G,R,X 对应 `RK_FORMAT_BGRX_8888`；X 作高字节的
   `RK_FORMAT_XRGB_8888` 被 RGA 拒绝（`dst invaild format`）。

5. **interactive governor + RT 内核**：60fps 周期性突发负载下 CPU 停在 408MHz，
   长跑掉到 ~40fps。程序启动时自动把两个 policy 设为 performance 并绑定
   cpu4-7（2.2GHz 大核簇），退出时恢复。

6. GREY 采集 stride 是 256 对齐的 **2048**（宽 1920），运行时从 `G_FMT` 读取。

7. **cached dma-heap 必须自己做 cache 维护**：RGA 驱动不会（可靠地）为 cached
   dmabuf 做 cache 同步。CPU 写入（UV 填充、OSD 文字）滞留在 L2 cache 时，
   RGA 读到的是 RAM 里的旧数据——表现为画面边缘持续绿/品红偏色、中部偶发
   偏色线条（损坏粒度正好是 NV12 的 UV 行 = 2 个像素行成对出现）、OSD 文字
   刷新残留。修复：CPU 写完后对 dmabuf 调用 `DMA_BUF_IOCTL_SYNC`
   （START → 写 → END）。A/B 实测：无 sync 版本 14 张快照中 1 张出现 10 行
   成对偏色行，加 sync 后 60 张快照（覆盖 60 秒）零偏色。

   编译开关 `-DNO_DMABUF_SYNC` 可复现未修复的行为（验证用）。

8. **RGA2-E 的 alpha blend（src α<255）数学不可靠，勿用于半透明**：探针实测
   RGBA→BGRX 混合时 G/B 通道符合直通数学、**R 通道异常钳位**；RGBA→RGBA 目标
   混合完全不生效；相同参数多次运行结果不稳定。OSD 画布只允许 α=0/255。
   **半透明需求交给 VOP2 overlay plane**：检测层独占一个空闲 Esmart plane
   （ABGR8888，zpos=1，`pixel blend mode=Coverage`，plane alpha 固定一次），
   显示硬件在扫描输出时做精确的 像素α×平面α 半透明合成——动态透明叠加
   （检测框压过 HUD 文字时文字透出）零 CPU/RGA 代价，且锁定脉冲用
   **per-pixel alpha** 实现而非每次改 plane 属性（属性 ioctl 会阻塞到
   vblank，实测把帧率从 59.8 拉到 58.2）。

## 文件

| 文件 | 说明 |
| ---- | ---- |
| `osd/osd.h` `osd/osd.cpp` | **libosd**：OSD 库，纯 CPU 光栅化 + dma-heap，不依赖任何厂商库（`libosd.a`） |
| `hw/image.h` | `hw::ImageDesc`：与厂商无关的 dmabuf 图像描述（各层之间的通用语言） |
| `hw/camera.h` `hw/camera.cpp` | `hw::CameraBase` 抽象接口 + `hw::MipiCamera`：V4L2 采集 + dmabuf 导出，AR0234M/SC233HGS 双传感器 + SOF 时间戳 |
| `hw/sof2epoch.h` `hw/sof2epoch.cpp` | `hw::Sof2Epoch`：CLOCK_MONOTONIC_RAW(SOF) → CLOCK_REALTIME(Epoch) 线性模型转换器（后台校准线程） |
| `hw/display.h` `hw/display.cpp` | `hw::HdmiDisplay`：DRM/KMS 双缓冲 + overlay plane |
| `hw/compositor.h` `hw/compositor.cpp` | `hw::Compositor`：**全工程唯一使用 RGA 的地方**，封装 NV12 中转与三步硅后端 |
| `hw/dmabuf.h` `hw/dmabuf.cpp` | `hw::DmaBuf`：dma-heap 分配小工具（bridge 台架模式灰帧用） |
| `layout/telemetry.h` | `osd::layout::Telemetry`：ROS 无关的遥测快照 POD |
| `layout/layout.h` `layout/layout.cpp` | **osd::layout**：Betaflight 元素布局引擎（TextSlot 变更驱动、水平仪、消息行、框足迹清除），只依赖 libosd |
| `apps/osd_demo.cpp` | 全功能演示：管线编排 + libosd + 模拟双目标 AI 检测 + 遥测 |
| `tests/test_layout_snapshot.cpp` | layout 快照回归（x86 可跑，输出 PPM + 断言） |
| `gen_font.py` | 用 PIL 生成 16x32 + 8x16 双字号抗锯齿 ASCII 字库（本地跑） |
| `osd/osd_font.h` | 生成的字库点阵 |
| `probe*.cpp` | 探针：`probe8` libosd 回归、`probe9` RGA 能力/优化、`probe10` 光栅微基准 |
| `CMakeLists.txt` | 构建（standalone 或被 osd_bridge 包 `add_subdirectory`） |

## 编译（板上）

CMake（osd_bridge 包经 `add_subdirectory` 复用同一套目标）：

```bash
cd ~/workspace/ar0234_osd_hdmi
cmake -B build -DLIBRGA_ROOT=~/workspace/librga -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
(cd build && ctest)        # probe8 回归 + layout 快照测试
```

板上回归实测（SC233HGS-ISP 自由跑 + HDMI，600 帧）：CMake 版 59.45 fps /
旧 Makefile 版 59.35 fps，rga/det/sof_dt 一致 —— CMake 化无性能回退，旧
Makefile 已移除。

x86 主机（可移植子集，无 libdrm/librga 依赖）：

```bash
cmake -B build-x86 -DOSD_WITH_HW=OFF && cmake --build build-x86 -j && (cd build-x86 && ctest)
```

库目标：`osd::core`（纯 CPU 光栅 + BBoxRect，零厂商依赖）、`osd::layout`
（Betaflight 元素布局，只依赖 core）、`osd::hw`（V4L2/DRM/RGA）。
依赖：`g++`、`pkg-config libdrm`、`~/workspace/librga`（librga 1.10.6）。

ROS2 桥接见 `fpv_ws/src/osd_bridge`（订阅 apm_bridge 话题 → Pipeline 渲染线程
→ HDMI；检测框经 `Pipeline::set_boxes(BBoxRect)` 注入，ROS 话题映射后续再接）。

## 运行（板上）

```bash
sudo systemctl stop lightdm          # 释放 DRM master 给本程序（必须：lightdm 与本程序
                                     # 争抢 DRM 会造成偶发 PageFlip EBUSY）
sudo ./osd_demo                      # 检测框+遥测+锁定（模拟数据）
# NV12 ISP 路径（video22，需保持 rkaiq_3A 运行提供 3A）：
sudo ./osd_demo --sensor sc233hgs-isp
# 定长运行 + 快照：
sudo ./osd_demo --frames 360 --snapframe 180
sudo systemctl start lightdm         # 恢复桌面
```

程序启动时自动执行（把传感器配成 Y8 1920x1200@60fps）：

```text
media-ctl -d /dev/media0 --set-v4l2 '"m00_b_mvcam 4-003b":0[fmt:Y8_1X8/1920x1200@1/60 field:none]'
```

## 快照查看（本地）

```bash
scp -O firefly@192.168.77.152:/tmp/osd_snap_003000.xrgb .   # 板上 scp 需 -O
python3 - <<'EOF'
from PIL import Image
Image.frombytes("RGB",(720,480),open('osd_snap_003000.xrgb','rb').read()[:720*480*4],'raw',"BGRX").save('snap.png')
EOF
```

## 参数

```text
-S 传感器: sc233hgs (默认) | ar0234
-c SC233HGS 模块号 0/1 (默认 0, 对应 m00_b_/m01_b_)
-n 运行 N 帧后退出               -s 第 N 帧转储快照 (默认 120, -1 禁用)
-o 快照输出目录 (默认 /tmp)
```

## 设计要点

- **零拷贝**：相机 buffer 经 `VIDIOC_EXPBUF` 导出 dmabuf，RGA `importbuffer_fd`
  直接读取；DRM dumb buffer 经 `drmPrimeHandleToFD` 导出后由 RGA 直接写入。
- **CPU 每帧仅**：OSD 动态文字光栅化（30Hz）+ UV 平面 162KB 填充；其余全部硬件。
- **NTSC 输出**：EDID 模式表选 720x480p（59.94/60Hz）；16:10 源 letterbox 到
  4:3（720x450 + 上下 15px 黑边）。
- **无撕裂**：双缓冲 + `DRM_MODE_PAGE_FLIP_EVENT`，写后台缓冲，vblank 翻转。
