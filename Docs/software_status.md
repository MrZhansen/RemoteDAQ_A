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
