# AR0234 → RGA 硬件 OSD 叠加 → HDMI NTSC 输出 (RK3576)

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
osd.draw_detect_box({.rect={x,y,w,h}, .label="UAV", .tag="T01",
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
| `draw_detect_box` | （新增） | **AI 检测框**：角标/全框、标签+置信度+track id、威胁配色、锁定脉冲菱形+外圈 |
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
| `hw/camera.h` `hw/camera.cpp` | `hw::CameraBase` 抽象接口 + `hw::MipiCamera`：V4L2 采集 + dmabuf 导出 |
| `hw/display.h` `hw/display.cpp` | `hw::HdmiDisplay`：DRM/KMS 双缓冲 + overlay plane |
| `hw/compositor.h` `hw/compositor.cpp` | `hw::Compositor`：**全工程唯一使用 RGA 的地方**，封装 NV12 中转与三步硅后端 |
| `osd_demo.cpp` | 全功能演示：管线编排 + libosd + 模拟双目标 AI 检测 + 遥测 |
| `gen_font.py` | 用 PIL 生成 16x32 抗锯齿 ASCII 字库（本地跑） |
| `osd_font.h` | 生成的字库点阵 |
| `probe*.cpp` | 探针：`probe8` libosd 回归、`probe9` RGA 能力/优化、`probe10` 光栅微基准 |
| `Makefile` | 板上编译 |

## 编译（板上）

```bash
cd ~/workspace/ar0234_osd_hdmi
make LIBRGA=/home/firefly/workspace/librga            # libosd.a + libhw.a + osd_demo
make LIBRGA=/home/firefly/workspace/librga probes     # probe8/9/10（不在默认目标里）
```

依赖：`g++`、`pkg-config libdrm`、`~/workspace/librga`（librga 1.10.6）。

## 运行（板上）

```bash
sudo systemctl stop lightdm          # 释放 DRM master 给本程序
sudo ./osd_demo                      # 检测框+遥测+锁定（模拟数据）
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
-d 视频节点 (默认 /dev/video0)   -m media 节点 (默认 /dev/media0)
-e 传感器实体名                  -W/-H/-F 相机宽高/帧率 (1920/1200/60)
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
