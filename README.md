# RemoteDAQ_A

STM32F746G-DISCO A-board acquisition firmware; IAR EWARM 8.32.1.169 / STM32CubeF7 HAL V1.17.4.

Default boot remains UART6 (PC6/PC7, 921600 8N1), GEAR0 and 480-cycle sampling on both ADC3 ranks. ADC3 ranks are PA0/CH2 then PF10/CH1. TIM2 drives acquisition; DMA2 Stream0 Channel2 uses 400 bytes at 0x20010000.

The software includes a guarded, locally armed GEAR2/20-us UDP candidate. Ordinary CMD01 still supports only GEAR0/GEAR1. Hardware timing, 144-cycle analog quality and two-board UDP have NOT been accepted. Do not infer production readiness from a successful build. See [software status](Docs/software_status.md).

## Build

Open EWARM/Project.eww and select RemoteDAQ_A. Use ordinary Build, not Clean/Rebuild. Command-line equivalent: `IarBuild.exe EWARM/RemoteDAQ_A.ewp -make RemoteDAQ_A -log all`. Install the stated IAR release; its runtime libraries are not vendored. No automatic CubeMX regeneration: the manually maintained network port is not represented by the acquisition-baseline .ioc.

The repository contains the actual build dependency subset of the original HAL/CMSIS package, LAN8742A driver, required LwIP sources/headers, and their license notices. Unused SDK examples, DSP binary libraries, local debug sessions, backups and firmware outputs are excluded; local originals are retained. Component provenance is in Middlewares/Third_Party/REMOTE_DAQ_SOURCES.md.

## Host tests

Run PowerShell scripts with a native GCC (the Compiler argument overrides the local default). Put generated output outside this checkout. First run Tests/run_host_tests.ps1 -OutputDirectory <work>/host_network_baseline; then Tests/run_network_tests.ps1 -OutputDirectory <work>/host_network. These generate the extraction fixtures from current main.c. Run Tests/run_network_lifecycle_tests.ps1 -OutputDirectory <work>/lifecycle -FixtureRoot <work> and Tests/run_network_targeted_tests.ps1 -OutputDirectory <work>/targeted. Python tools check candidate clock constraints and UDP vectors without MCU access. Tests/udp_peer_probe.py is offline by default; --live is an explicit, separate hardware/network operation.

Synthetic packets and HAL simulations verify software logic only. No firmware download, target reset or debugger attachment is performed by these host tests.
