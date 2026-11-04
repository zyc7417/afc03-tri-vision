# RK3576 三路分析和显示

这些模块取自三路演示工程的 SoC 提交源码。三路相机画面在板端组成一幅图像，模型执行一次检测后，分析器按相机分别关联目标、统计人车并写入本地事件记录。

|源码（位于 `project/board_npu_stage1/`）|用途|
|---|---|
|`tri_aerial_lite_20261103/analyzer.py`|在 A0、A1、B0 各自的画面内，用交并比关联同类检测框。生成短时目标编号和当前可见人数、车辆数。|
|`tri_aerial_lite_20261103/event_log.py`|通过有界队列把观测事件异步写入 JSONL，分别记录入队、写出和落盘状态。|
|`tri_native_xshm_20261103/display_panel_direct.c`、`display_panel.h`|通过 X11 共享内存在板载屏幕显示像素，等待对应的 XShm 完成事件。|
|`tri_native_xshm_20261103/web/`、`tri_touch_console_20261103/web/app.css`|触摸工作台的 HTML、JavaScript 和样式。页面通过板端服务切换三路总览、单路放大及采集状态。|

## 运行离线测试

从本目录使用 Python 3.10 或更新版本执行：

```sh
python -B -m unittest discover -v -s project/board_npu_stage1/tri_aerial_lite_20261103 -p tests.py
```

12 项测试使用合成检测框，覆盖不同相机的目标互不串号、短时遮挡、重复帧、计数及事件写入失败等情况。分析器与事件写入器只依赖 Python 标准库。

## 在板上使用

整机采集需要对应的 FPGA 位流、Rockchip BSP、PCIe 驱动、RKNN 模型与板端控制服务。网页文件展示触摸界面的实现，控制请求由该服务处理。原生显示源码使用 Linux 的 X11、Xext 和 System V 共享内存接口，在目标 Linux 环境可采用以下命令编译：

```sh
cd project/board_npu_stage1/tri_native_xshm_20261103
cc -O3 -std=c11 -Wall -Wextra -fPIC -shared display_panel_direct.c -o display_panel_direct.so $(pkg-config --cflags --libs x11 xext)
```
