# Trigger software/build checkpoint — 2026-10-31

Implemented CMD03/04/05 runtime configuration and a main-context trigger
engine while preserving the protocol frame sizes, CRC, ADC DMA layout and
clock configuration. Trigger sources are 0=off, 1=CH1, 2=CH2, 3=D1, 4=D2 and
5=D3; modes are 0=rising, 1=falling, 2=any edge. A completed window contains
24 pre-trigger points, the edge point and 25 post-trigger points. CH1, CH2 and
D1-D3 are copied from the same `Full_Snapshot` sample indices. Source 0 keeps
the original free-run path, and a published trigger window clears and re-arms.

The full host regression passed on 2026-10-31, including 145 trigger-engine
checks, 3480 protocol checks, 438334 control/status checks and all existing
snapshot, UART, ADC/DMA, digital-sync, GEAR, CMD01/02 and transport suites.
The CMD07 follow-up suites also passed independently: 109 TIM3/CMD07 checks and
204 UART/UDP/status/G0-G2 checks. These are host simulations, not hardware
evidence. The trigger-engine suite explicitly cross-checks every CH1/CH2/D1/D2/D3
source against rising, falling and any-edge modes.

IAR Rebuild All completed and generated the HEX successfully; the operator
reported no build errors. The current HEX was regenerated at 2026-10-31
16:23:58 +08:00. Linker memory: code 43418 bytes, readonly data 1768,
readwrite data 54383 plus 400 absolute. HEX SHA-256 is
`367D4E2894E2B5C9184AA1FDFF7BB7090E3E1FEF823F608A559AC32B0CB65EC2`.
The MAP still places `adc_dma_buffer` at `0x20010000` with size `0x190` and
`ethernet_dma` at `0x20018000` with size `0x4a00`; the 4096-byte CSTACK
configuration is unchanged.
OUT, HEX, MAP, SHA-256 manifest and the verification log are retained under
`D:/QRS/Project/RemoteDAQ_A/RemoteDAQ_A/tmp/trigger_build_20261031/`.
The prior verified CMD07 OUT/HEX/MAP is separately backed up under
`D:/QRS/Project/RemoteDAQ_A/RemoteDAQ_A/backups/pre_trigger_20261031_cmd07_stable/`;
its HEX SHA-256 is
`712DC9CFB2F68E4735154294F5AC742853A2A9CB570AAC651B30B307F8EB4C3D`.

The A board was subsequently programmed with this trigger build without a full
chip erase. CMD07 regression on the new image passed for 100 Hz, 1 kHz, 5 kHz
and 10 kHz, with B CRC=0 and acquisition/DMA faults clear. At that initial
checkpoint, CMD03/04/05 and the five actual trigger sources were still pending;
the hardware progress obtained afterwards is recorded below.

Real-board trigger progress on 2026-10-31:

- CMD03/04/05 reached A over UDP. Live SWD reads confirmed CH2 source
  (`daq_trigger_source=2`), rising-edge mode (`daq_trigger_mode=0`) and
  protocol levels 96 and 64.
- CH1 and CH2 rising, falling and any-edge trigger runs produced complete
  trigger frames; the captured CH2 evidence under
  `tmp/trigger_live_20261031/` has `CRC=0`.
- A microphone signal in the approximately 400-600 Hz range made
  `trigger_edge_count` and `trigger_frame_count` increase together. At level
  96 the observed edge count advanced from 8807 to 9083 while the frame count
  advanced from 8806 to 9081, with one group gap and
  `digital_sync.error=0`. After lowering the level to 64, the operator
  confirmed continued counter growth and a continuously refreshed B waveform.
- Exact 500 Hz, 1 kHz and 2 kHz microphone acceptance, microphone falling/any
  edge records, D1/D2/D3 physical trigger runs and the five-channel alignment
  run remain pending.

Hardware signal mapping used by this build:

- CH1 analog: PF10 / ADC3_IN8 / Arduino A1.
- CH2 analog: PA0 / ADC3_IN0 / Arduino A0.
- D1 digital: PG6 / Arduino D2, protocol bit 0.
- D2 digital: PG7 / Arduino D4, protocol bit 1.
- D3 digital: PI3 / Arduino D7, protocol bit 2.
- All external trigger inputs must remain within 0..3.3 V and share ground.

---

# CMD07 hardware acceptance checkpoint — 2026-10-30

CMD07 rectangular-wave frequency control passed real-hardware acceptance on the
A board for 100 Hz, 1 kHz, 5 kHz and 10 kHz. The control path was Ethernet UDP:
the test host temporarily used 192.168.1.20:5005 and sent control frames to A at
192.168.1.10:5006. The only enumerated COM port was COM5, the ST-LINK Virtual
COM Port; a real CMD06 transmission there produced no response, so it was not
treated as USART6. The temporary host address was removed after testing and the
host's original network configuration was retained.

All four valid CMD07 requests produced CRC-correct 12-byte status frames whose
frequency fields matched the request. Real reads of the current map variables
confirmed `square_wave_running=1`, `square_wave_last_set_status=0`, success count
incrementing 1..4, zero command errors and a 108 MHz TIM3 counter clock. TIM3
PSC/ARR/CCR1 measured 107/9999/5000 for 100 Hz, 107/999/500 for 1 kHz,
107/199/100 for 5 kHz and 107/99/50 for 10 kHz.

PB4 output was measured physically through the existing PB4-to-CH1/PA0 loopback.
A real 50-point ADC capture at 200 us/point showed a 10 ms period at 100 Hz and a
1 ms period at 1 kHz. With acquisition temporarily switched to GEAR2 for a
20 us/point capture, 5 kHz repeated every ten samples (200 us) and 10 kHz
repeated every five samples (100 us). The captured logic sequences, raw UDP
status frames and SWD reads are retained under
D:/QRS/Project/RemoteDAQ_A/tmp/cmd07_live/.

Invalid 99 Hz and 10001 Hz requests were sent over the real UDP path and were
rejected without a success status response. The 10 kHz output and running
acquisition remained unchanged. ADC remained running with zero ADC/HAL/DMA
errors, and `adc_group_count` continued increasing after the test.

The board was restored to 1000 Hz, GEAR0, default UART transport mode and
DBGMCU_CR.DBG_SLEEP=0. No firmware download or Flash erase/write occurred.
No independent oscilloscope or logic analyzer was available, so this checkpoint
accepts functional frequency and period behavior through the physical loopback;
it does not claim instrument-grade absolute frequency accuracy or jitter.

The real B-board loop was then verified without resetting B. B sent all four
CMD07 values through its own interface and displayed W=100, 1000, 5000 and
10000 with the corresponding GEAR. A accepted every control frame, executed the
requested frequency and returned a status frame; B reported CRC=0 throughout.
At 1000 Hz/GEAR0 B showed RX about 102 and DROP stable over a 10-second window.
At 10000 Hz/GEAR2 B showed RX about 1020, W=10000, GEAR=2, CRC=0 and DROP stable
over a 10-second window. The cumulative DROP increased only while the operator
repeatedly switched waveform frequencies and GEAR settings; it did not increase
in either steady-state window. After restoration B showed W=1000, GEAR=0,
RX about 102, CRC=0 and DROP stable at 429 for 10 seconds.

---

# Current checkpoint — formal UDP CMD01 GEAR2 (2026-10-30)

User explicitly approved extending CMD01 to GEAR2 only on qualified, healthy UDP. Implemented using the existing stopped acquisition manager: G2 uses two ADC ranks at 144 cycles and TIM2 PSC107/ARR19; leaving G2 restores both ranks to 480 and G0/G1 timing. UART still rejects G2. Default UART/G0/480 is unchanged. REV_ID policy, 4-us early-read guards, NDTR/TIM5 checks, CRC, mask/DIG layout and success-only SEQ remain unchanged. Network loss/transport exit requests G0/480; acquisition faults stop and latch without automatic retry. Existing offline/finite-UDP experiment ownership remains separate. New Watch permissions are acquisition_udp_gear2_available and acquisition_udp_gear2_active; neither certifies analog accuracy or absolute timing.

Executed host tests: all 15 legacy suites passed (596944 checks), network/identity/adapter/isolated-trial regression passed, new real-manager UDP-only policy test 2281 checks, remote command/status/mask/SEQ integration 103 checks, and layout/init/handshake lifecycle tests passed. These are host simulations, not physical timing measurements. IAR ordinary incremental Build 2026-10-30 16:40:41–16:40:46 +08:00 passed: 0 errors, 0 warnings, exit0. Flash OUT/HEX identical (43833 bytes, 0x08000000..0x0800AB38); ADC DMA 0x20010000/400, ETH arena 0x20018000/18944, CSTACK4096; code42066, ROdata1767, RW53579+400 absolute. HEX SHA256 43e26cdd1eabf116aeed901e1df1da151b5dd77df1b32cfaa5112b6044d697d7. This new remote-control build was explicitly approved and programmed/verified in sectors0/1 at 2026-10-30 16:43–16:45 +08:00, then hardware-reset into the application. Real readback confirms default UART/G0/two480, followed by UDP link/peer ready, no acquisition/network errors and G2 permission1. B remote CMD01 G2 acceptance passed: B gear button entered G2/M1F, RX displayed1020, CRC0 and DROP0 with no increase, CH1 square-wave display normal, FPS22; B requested G3/G4/G0 to exit, unsupported3/4 stayed inactive, G0 restored at RX102. A readback confirms G0, both480 (SMPR2=0x07000007), rank order unchanged, TIM2 PSC107/ARR199, ADC/DMA/digital/network errors0. At readback, 24 real gear requests had completed successfully, 19 unsupported requests were rejected, and no submitted gear request failed. This is functional remote-control acceptance, not independent timing/analog qualification.

Earlier e36838c3… firmware DID complete real board-to-board UDP functional trials: G0 about100 frames/s, G1 about200, finite G2 about1000. B reported stable G2 DROP881→881 and CRC0→0 over its approximately10-second window, RX1021, G2/M1F, then G0 recovery. A second finite G2 window published/consumed/submitted16793 complete groups in16794ms, with no skipped groups or acquisition errors, and restored both ranks480. This evidence applies to the existing finite entry, not yet the new CMD01 entry. B RX is its displayed approximate rate. Cumulative DROP881 is historical; stable-window delta was0.

During the G2 throughput window CH1 was fed by A's PB4 square-wave output; CH2 and the three digital inputs had no test stimulus. Subsequently, individual PB4 loopback tests at G0 showed square waves on CH1, CH2(A0/PA0), D1(D2/PG6), D2(D4/PG7) and D3(D7/PI3), as reported by the user, with B RX about102. These sequential functional checks do not establish simultaneous edge alignment or0.1ms edge resolution. No independent oscilloscope/logic analyzer/function generator was available. Full IRQ worst case, absolute20-us timing, analog settling/crosstalk/microphone quality and GEAR2 dynamic digital synchronization and ADC144 analog quality remain unaccepted. DMA C/publish paths exceeded their retained20-us diagnostic budget in the finite trial (sticky flag0x20); they are preemptible by TIM2, and the observed functional success does not waive that flag. Do not claim complete timing qualification. A temporary DBGMCU_CR.DBG_SLEEP=1 is used to keep SWD access while WFI executes; original0 restoration remains pending.

During input rewiring, the user reported no UDP reception. The initial HOTPLUG read had an invalid IDCODE and was discarded. After test leads were removed, controlled reset showed correct ID and unchanged Flash vectors, DBG_SLEEP read0 and was restored to1 for debug access, and the same firmware was run with its existing UDP request enabled. Valid reads confirmed PB4 AF2/TIM3 PWM1 enabled, PSC107/ARR999/CCR500, mask1F and no sampling/network faults. B reported CH1 waveform restored and then all remaining individual inputs working. The pre-reset cause was not established; do not describe stale/invalid debug reads as a proven firmware failure or claim hot-link recovery. No additional code or Flash programming was performed for this recovery.

Detailed backup, tests, build, resource comparison and final hashes: D:/QRS/Project/RemoteDAQ_A/tmp/formal_udp_gear2_2026-10-30T08-17-48-455Z_4d5306/. Prior live evidence: D:/QRS/Project/RemoteDAQ_A/tmp/hw_udp_2026-10-30_11-36-53_c82b53/rx0_followup_2026-10-30_14-31-56_8eaa15/fresh_arp_14-58-40/. Historical checkpoints below are superseded by this paragraph.

---

# Hardware bring-up checkpoint — 2026-10-30

Real A-board IDCODE was read twice as 0x10016449 (DEV 0x449, REV 0x1001/Z) with CubeProgrammer 2.23.0 via reset-under-connect. The identity gate remains unchanged. The old 0x1000 value was not a verified board reading.

Live bring-up found and fixed two faults: missing IAR LwIP structure packing (official bpstruct/epstruct port headers), and polling W1C acknowledgement of TBUS/RBUS that stranded HAL DMA transmission. Compile-time target Ethernet/IPv4/UDP layouts and bounded init-assert failure handling were added. First assertion text is retained for diagnostics. Sampling, clocks, ADC DMA layout and protocol were not changed.

The second corrected HEX e36838c3a1ef15e6ae13c13d2cf785cc622d239960081178307b7b6a41459e5f was explicitly approved, programmed into sectors 0/1 and verified. A board was reset into the application afterward. IAR ordinary incremental build passed with 0 errors/0 warnings/exit0. Target ADC DMA remains 0x20010000/400 bytes; ETH arena remains 0x20018000/18944; stack 4096.

Latest live A observation: default GEAR0/480 checks passed; PHY LAN8742 ID 0x0007C131, 100Mbps full-duplex link and B ARP resolution succeeded. A locally submitted 222 data datagrams and 1 status datagram; 224 Ethernet transmissions completed including ARP; network/ADC/DMA/digital errors were zero. This is NOT evidence of B UDP delivery. User reports B UDP firmware ready but no waveform, RX=0, CRC=0; CMD06 exchange remains unverified. Do not advance to GEAR1/GEAR2 or claim end-to-end throughput.

Evidence and firmware hashes: D:/QRS/Project/RemoteDAQ_A/tmp/hw_udp_2026-10-30_11-36-53_c82b53/LIVE_PROGRESS.md. Further probe commands were blocked before execution by automatic approval service usage quota; no bypass was attempted. Next step is actual transmitted IPv4/UDP bytes and B UDP-callback counters.

---

The following is the earlier software-only checkpoint, retained as historical evidence.

# Software verification checkpoint — 2026-10-29

## Implemented and protected

A IPv4 192.168.1.10/24, UDP bind 5006, B destination 192.168.1.20:5005. A MAC 02:46:DA:00:00:0A is passed in byte-array order to HAL and differs from confirmed B MAC 00:80:E1:00:00:20. ARP resolves B; its MAC is not hardcoded. One UDP datagram contains exactly one application frame: data 260, status 12, control 9 bytes. V1.1 CMD 0x06 means request handshake/status, not a new protocol. CRC-16/CCITT-FALSE and independent SEQ behavior are unchanged.

This checkpoint hardens RX allocation/chain ownership, rejects malformed or duplicate/cyclic buffers with bounded validation, prevents polling/transmission after a DMA callback fault, and frees the UDP PCB/netif on initialization failure. RX faults pin a finite pool rather than releasing potentially corrupt memory. Link/DMA faults latch; hot reconnection is NOT an automatic recovery guarantee.

Default UART/GEAR0/480 cycles and CMD01/02/06 remain unchanged. The latest production-source edits are daq_ethernet.c and daq_network.c; no new ADC/TIM/DMA/MPU/stack configuration changes were made in this hardening checkpoint.

## Silicon qualification gate

The real IDCODE comes from DBGMCU->IDCODE, not a fallback/test constant. DEV_ID uses bits 11:0, REV_ID bits 31:16. RM0385 rev9 section 40.6.1 identifies DEV_ID 0x449, revision A=0x1000, revisions Z/1=0x1001. ES0290 rev7 is the relevant F746/F75 errata sheet; ES0334 concerns F76/F77 and cannot establish an F746 RMII defect. ES0290 revision history says revision A was changed into Z; this does not prove equivalence or qualification of A.

Firmware accepts 0x449/0x1001 for identity qualification only; known A/0x1000 remains unqualified (network error 9), as do unknown revisions and wrong devices. This is a conservative qualification policy, not evidence of a demonstrated F746-A RMII defect. It does not replace other errata, cache or PHY checks. No ID spoof or gate bypass is present. Raw identity can later be read from adc_debug_start.device_idcode when adc_debug_start.valid is set, without enabling Ethernet. A real A-revision device requires applicable ST evidence/qualification before changing this policy.

Official references: [ES0290](https://www.st.com/resource/en/errata_sheet/es0290-stm32f74xxx-and-stm32f75xxx-device-limitations-stmicroelectronics.pdf), [RM0385](https://www.st.com/resource/en/reference_manual/dm00124865.pdf), [ES0334](https://www.st.com/resource/en/errata_sheet/es0334-stm32f76xxx-and-stm32f77xxx-device-errata-stmicroelectronics.pdf).

## Executed software evidence

IAR incremental build 2026-10-29 21:48:18–21:48:21 +08:00: compile/link/HEX conversion succeeded, 0 errors, 0 warnings, exit 0. Final ELF and HEX contain the same 43,483 Flash bytes. MAP: code 41,586 bytes; readonly data 1,897; readwrite data 53,535 plus 400 absolute. ADC DMA remains 0x20010000/400; ETH arena 0x20018000/18,944; CSTACK 4096. Static free DTCM is not a measured stack watermark.

Tests executed during this checkpoint include 12,377 adapter/gate/interop checks; 326 nominal sampling-envelope checks; 116 completion/fault checks; 44 padding checks; 66 PCB/init-failure checks; 45 actual-service CMD06 scheduling checks; 661 UART transport regression checks; latest-B-interface RX/interop checks (10,094 + 2,186) and 1,406 independent packet checks; 17 PC probe tests. Counts overlap across reruns and are intentionally not added as unique coverage. Initial red tests reproduced the fixed faults; fixture compilation errors were corrected before passing runs. Earlier complete legacy regression results are retained as historical evidence, not claimed as rerun here.

Final SHA-256:
- OUT: 39b6688a8d23ab316a59ea457b524b86003674b2fc80f0c991676fcbd3343c54
- HEX: 220f717a4be52cce768490b4d44d1ba0008b92c08594c40702905c6e85f18eb7
- MAP: f906a8ef246c4ab3df214659e1e83c6a22ef0a12efe0687eb943d37b1e88bfdc

## Remaining acceptance

No board, ST-LINK or live network access occurred. Real IDCODE/marking, PHY clock/link, 20-us response/IRQ worst case, analog 144-cycle settling/crosstalk/microphone behavior, sustained 1000 complete groups/s and B reception/display remain unaccepted. 480-cycle dual-rank conversion cannot fit 20 us; 144 cycles nominally fits at 27 MHz ADC, but theory does not establish signal quality. Local IRQ C timestamps exclude exception entry/return and compiler boundary overhead.

B display currently may skip network processing for 30–40 ms: that permits 30–40 incoming frames at 1000/s. B buffering/service must be validated; A does not decimate, reduce point count, or alter the 260-byte frame to hide this risk.
