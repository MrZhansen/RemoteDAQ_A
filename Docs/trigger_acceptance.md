# CMD03/04/05 实机验收记录

日期：2026-10-31

本记录只区分已经具备实机证据的项目与仍需验证的项目。主机测试和 IAR
构建结果不能替代真实 A/B 双板验收。

## 已通过

| 项目 | 实机证据 | 结论 |
|---|---|---|
| 固件版本 | 当前 A 板运行触发构建，HEX SHA-256 为 `367D4E2894E2B5C9184AA1FDFF7BB7090E3E1FEF823F608A559AC32B0CB65EC2` | 通过 |
| CMD03/04/05 下发 | A Watch 读到 `daq_trigger_source=2`、`daq_trigger_mode=0`，电平先后为 96 和 64 | 通过 |
| CH1 模拟触发 | 上升沿、下降沿、任意边沿均产生完整触发帧 | 通过 |
| CH2 模拟触发 | 上升沿、下降沿、任意边沿均产生完整触发帧，记录帧 `CRC=0` | 通过 |
| CH2 麦克风触发链路 | 约 400～600Hz 输入下，边沿计数和帧计数持续增长，B 波形持续刷新 | 通过触发链路 |
| 采集健康状态 | `adc_running=1`，ADC/HAL/DMA 错误为 0，`digital_sync.error=0`，UDP 传输为活动状态 | 通过 |

电平 96 时的一次连续观测为：

```text
trigger_edge_count       8807 -> 9083
trigger_frame_count      8806 -> 9081
trigger_group_gap_count  1
digital_sync.error       0
```

边沿计数和帧计数基本同步增长，说明边沿被识别后会继续收集窗口并重新布防。
帧计数比边沿计数少 1～2，对应截图时仍有窗口处于采集或待发送状态。

## 待实机验证

| 项目 | 通过标准 |
|---|---|
| 500Hz 音频稳定显示 | 手机单频音源下，B 波形周期稳定、刷新连续，频率测量接近 500Hz |
| 1kHz 音频稳定显示 | B 波形周期稳定、刷新连续，频率测量接近 1kHz |
| 2kHz 音频稳定显示 | B 波形周期稳定、刷新连续，频率测量接近 2kHz |
| CH2 麦克风下降沿 | `trigger_edge_count` 和 `trigger_frame_count` 持续增长，触发点位于窗口中部 |
| CH2 麦克风任意沿 | 上升和下降均能触发，计数持续增长 |
| D1 三种边沿 | PB4 接 PG6/Arduino D2，上升、下降、任意沿均产生完整帧 |
| D2 三种边沿 | PB4 接 PG7/Arduino D4，上升、下降、任意沿均产生完整帧 |
| D3 三种边沿 | PB4 接 PI3/Arduino D7，上升、下降、任意沿均产生完整帧 |
| 五通道时间对齐 | M1F 同窗采集时，CH1、CH2、D1、D2、D3 的边沿位于同一时间索引附近 |
| 最终联调 | UDP 持续运行，记录 `RX/ST/CRC/DROP`，ADC/DMA/数字同步错误保持为 0 |
| 恢复与收尾 | CMD03=0，输出恢复 1kHz，GEAR0；更新文档、提交并正常快进推送 |

## 下一轮最短顺序

1. 保持麦克风接线，测试准确的 500Hz、1kHz、2kHz 上升沿。
2. 在同一接线下补测下降沿和任意沿，避免重接模拟线。
3. 将 CH2 触发关闭后，按 D1、D2、D3 的顺序每次只接一路 PB4 信号，并逐一测试三种边沿。
4. D1/D2/D3 全部通过后，同时开启 M1F，检查五通道触发点和时间对齐。
5. 保存最终 B 屏与 IAR Watch 截图，恢复 CMD03=0、1kHz、GEAR0。

## 每项至少保留

- 当前 CMD03/04/05 参数和发送帧。
- B 屏中的波形、触发标签、频率、`RX/ST/CRC/DROP/GEAR`。
- A 屏中的 `trigger_edge_count`、`trigger_frame_count`、
  `trigger_rearm_count`、`trigger_group_gap_count`。
- A 屏中的 `adc_running`、`adc_error`、`adc_hal_error`、`dma_error`、
  `digital_sync.error` 和 `daq_udp_data_error_count`。

具体命令帧和通道引脚统一定义见
`tmp/trigger_live_20261031/test_matrix.md`。
