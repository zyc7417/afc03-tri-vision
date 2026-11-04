报告第3.6节两项数据的复算方法

pcie_sample.csv 来自三路 aerial21 实测的两个批段末槽位记录，每帧为1920×1080 GRAY8。四帧含封装传输字节数之和除以四帧原始字节数之和为37.05%，以1减去该比值得到62.95%。
rknn_calls.csv 来自同一模型的独立固定输入板测。两个工作线程各4次，预热0次。按全部8次记录合并排序，P50、P95按(n−1)×p位置线性插值。run_ms计rknn_run主机调用，c_total_ms计设置输入至输出释放。
测量原文件SHA256：
segment_0.json: 749f4ed4214c1d71d32b047ff4f2365fc2a5c6067f3f74a99c8b68936f68b2e6
segment_1.json: 22c8fe992f88626ff6d61986b5e8f40bb2c08dbe54eeeb9876ea8d152e414878
measurement.json: a6d2a57e96b8e5d7fed446a3bce27e69147ea6a67938e76d2dd07599c24be9f2
