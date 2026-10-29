# RemoteDAQ network component provenance

Existing STM32CubeF7 HAL/package: V1.17.4 project, HAL core V1.3.3. HAL source files were not replaced.

- LwIP: STMicroelectronics/stm32-mw-lwip, tag v2.1.3_20241213, commit 32a06157b2f8fb66e53d5a4d7a1dd27c2f71802d.
  Source: https://github.com/STMicroelectronics/stm32-mw-lwip/tree/v2.1.3_20241213
  ZIP SHA-256: 3c8f0493f0d3cc9e3366463da91e69836825c8c442b595cc1b3f110318ba9ae8
  License: LwIP/COPYING (BSD-3-Clause), per-file notices, and LwIP/st_readme.txt ST integration terms, including the ST-device restriction. Preserved verbatim.
  CubeF7 V1.17.4 itself bundles 2.1.2; this uses ST's later 2.1.3 maintenance release with the compatible raw IPv4/UDP API. Only 17 core/Ethernet C units are built.
- LAN8742: official BSP component v1.0.4, commit 9f365d062b3cfae33859acd50e5df932e815b76a, as referenced by CubeF7 v1.17.4.
  Source: https://github.com/STMicroelectronics/stm32-lan8742/tree/v1.0.4
  ZIP SHA-256: 4ba4d685b28ffad2e7e50928f4a5281513ba31a9fd8e080be58f28a70210242a
  License: Drivers/BSP/Components/lan8742/LICENSE.md (BSD-3-Clause). Driver C/header unchanged.
- Low-level design reference: ST CubeF7 v1.17.4 STM32746G-Discovery LwIP_HTTP_Server_Netconn_RTOS/Src/ethernetif.c. This project implements a local NO_SYS polling adapter, without FreeRTOS.

No CubeMX regeneration or HAL replacement was performed. Network integration is manually maintained in Core, HAL config, EWP and ICF; the .ioc remains the verified acquisition baseline and does not describe this manual network port. Review these additions before any future regeneration.
