param(
    [string]$Compiler = 'E:\mingw64\bin\gcc.exe',
    [Parameter(Mandatory = $true)][string]$OutputDirectory
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testOutput = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null

function Get-FunctionText([string]$Source, [string]$Signature) {
    $start = $Source.IndexOf($Signature, [StringComparison]::Ordinal)
    if ($start -lt 0) { throw "Missing function: $Signature" }
    $brace = $Source.IndexOf('{', $start)
    $depth = 1
    $cursor = $brace + 1
    while (($depth -gt 0) -and ($cursor -lt $Source.Length)) {
        if ($Source[$cursor] -eq '{') { $depth++ }
        if ($Source[$cursor] -eq '}') { $depth-- }
        $cursor++
    }
    if ($depth -ne 0) { throw "Unclosed function: $Signature" }
    return $Source.Substring($start, $cursor - $start)
}

$mainSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Src/main.c'))
$mainHeader = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Inc/main.h'))
$sampleDefine = [regex]::Match($mainHeader, '(?m)^#define CH1_SNAPSHOT_SAMPLES[^\r\n]+')
$snapshotType = [regex]::Match($mainHeader,
    '(?s)typedef struct\s*\{\s*uint16_t samples\[CH1_SNAPSHOT_SAMPLES\];\s*uint32_t group_count;\s*\} CH1_Snapshot;')
$analogType = [regex]::Match($mainHeader,
    '(?s)typedef struct\s*\{\s*uint16_t ch1\[CH1_SNAPSHOT_SAMPLES\];\s*uint16_t ch2\[CH1_SNAPSHOT_SAMPLES\];\s*uint32_t group_count;\s*\} Analog_Snapshot;')
$fullType = [regex]::Match($mainHeader,
    '(?s)typedef struct\s*\{\s*uint16_t ch1\[CH1_SNAPSHOT_SAMPLES\];\s*uint16_t ch2\[CH1_SNAPSHOT_SAMPLES\];\s*uint8_t dig\[CH1_SNAPSHOT_SAMPLES\];\s*uint32_t group_count;\s*uint32_t gear;[^}]*\} Full_Snapshot;')
if (-not $sampleDefine.Success -or -not $snapshotType.Success -or -not $analogType.Success -or -not $fullType.Success) {
    throw 'Snapshot header layout changed; update the host fixture explicitly.'
}
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_snapshot_type.inc'),
    ($sampleDefine.Value + "`r`n" + $snapshotType.Value + "`r`n" + $analogType.Value + "`r`n" + $fullType.Value))
$publish = Get-FunctionText $mainSource "static void CH1_PublishGroup(uint32_t offset)`r`n{"
$reader = Get-FunctionText $mainSource '__root HAL_StatusTypeDef CH1_GetSnapshot(CH1_Snapshot *snapshot)'
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_snapshot_under_test.inc'),
    ($publish + "`r`n" + $reader))

$protocolExe = Join-Path $testOutput 'test_daq_protocol.exe'
$snapshotExe = Join-Path $testOutput 'test_ch1_snapshot.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $PSScriptRoot 'test_daq_protocol.c') '-o' $protocolExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Protocol host compile failed: $LASTEXITCODE" }
& $protocolExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Protocol tests failed: $LASTEXITCODE" }
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' $testOutput (Join-Path $PSScriptRoot 'test_ch1_snapshot.c') '-o' $snapshotExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Snapshot host compile failed: $LASTEXITCODE" }
& $snapshotExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Snapshot tests failed: $LASTEXITCODE" }
$previewState = [regex]::Match($mainSource,
    '(?s)/\* CH1 frame preview state: main-context only\. \*/(.*?)/\* End CH1 frame preview state\. \*/')
if (-not $previewState.Success) { throw 'Missing frame preview state for host fixture.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_preview_state.inc'),
    $previewState.Groups[1].Value)
$preview = Get-FunctionText $mainSource "static void CH1_FramePreviewPoll(void)`r`n{"
$previewBuilder = Get-FunctionText $mainSource "static uint32_t CH1_BuildFramePreview(uint16_t sequence)`r`n{"
$analogBuilder = Get-FunctionText $mainSource "static uint32_t Analog_BuildFramePreview(uint16_t sequence, uint32_t include_ch2)`r`n{"
[IO.File]::WriteAllText((Join-Path $testOutput 'analog_frame_builder.inc'), $analogBuilder)
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_frame_builder.inc'),
    ($analogBuilder + "`r`n" + $previewBuilder))
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_preview_under_test.inc'),
    ($analogBuilder + "`r`n" + $previewBuilder + "`r`n" + $preview))
$previewExe = Join-Path $testOutput 'test_ch1_preview.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $PSScriptRoot 'test_ch1_preview.c') '-o' $previewExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Preview host compile failed: $LASTEXITCODE" }
& $previewExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Preview integration tests failed: $LASTEXITCODE" }
$uartState = [regex]::Match($mainSource,
    '(?s)/\* CH1 UART transmit state: main-context only\. \*/(.*?)/\* End CH1 UART transmit state\. \*/')
if (-not $uartState.Success) { throw 'Missing CH1 UART transmit state for host fixture.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_uart_state.inc'), $uartState.Groups[1].Value)
$uartService = Get-FunctionText $mainSource "static uint32_t CH1_FrameTransmitPoll(void)`r`n{"
$background = Get-FunctionText $mainSource "static void CH1_BackgroundPoll(void)`r`n{"
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_uart_under_test.inc'),
    ((Get-FunctionText $mainSource "static HAL_StatusTypeDef UART6_TransmitFrame(const uint8_t *data,`r`n                                           uint16_t length, uint32_t timeout)`r`n{") + "`r`n" + $uartService + "`r`n" + $background))
$serviceDefines = [regex]::Matches($mainSource,
    '(?m)^#define (UART6_FRAME_TIMEOUT_MS|GPIO_INPUT_PERIOD_MS|UART6_RATE_WINDOW_MS)[^\r\n]+')
if ($serviceDefines.Count -ne 3) { throw 'Missing service timeout/period definitions.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'ch1_service_defines.inc'),
    (($serviceDefines | ForEach-Object { $_.Value }) -join "`r`n"))
$startup = Get-FunctionText $mainSource "static void CH1_SamplingStart(void)`r`n{"
if ($startup -notmatch 'Acquisition_ApplyGear\(0U\)') { throw 'Power-on gear is no longer zero.' }
$mainFunction = Get-FunctionText $mainSource "int main(void)`r`n{"
if ($mainFunction -match 'HAL_Delay\s*\(|UART6_TestTransmitPoll\s*\(') {
    throw 'Main still schedules a fixed delay or the ASCII UART sender.'
}
if ($mainFunction -notmatch 'UART6_ServicePoll\(\) == 0U' -or
    $mainFunction -notmatch '__WFI\(\)' -or
    $mainFunction -notmatch 'CH1_BackgroundPoll\(\)') {
    throw 'Main lacks binary service, periodic background tasks, or idle wait.'
}
$uartExe = Join-Path $testOutput 'test_ch1_uart.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $PSScriptRoot 'test_ch1_uart.c') '-o' $uartExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "UART host compile failed: $LASTEXITCODE" }
& $uartExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "UART host tests failed: $LASTEXITCODE" }
$analogReader = Get-FunctionText $mainSource '__root HAL_StatusTypeDef Analog_GetSnapshot(Analog_Snapshot *snapshot)'
# Host-only preemption hook between the two channel reads, in addition to
# the existing barrier hooks. The firmware source receives no test hooks.
$copyAnchor = 'candidate.ch1[i] = adc_samples[i];'
if ([regex]::Matches($analogReader, [regex]::Escape($copyAnchor)).Count -ne 1) {
    throw 'Dual copy anchor changed; review the mid-copy interrupt test.'
}
$instrumentedReader = $analogReader.Replace($copyAnchor,
    ($copyAnchor + "`r`n    ANALOG_COPY_TEST_HOOK(i);"))
[IO.File]::WriteAllText((Join-Path $testOutput 'analog_snapshot_under_test.inc'),
    ($publish + "`r`n" + $instrumentedReader))
$analogExe = Join-Path $testOutput 'test_dual_analog.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $PSScriptRoot 'test_dual_analog.c') '-o' $analogExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Dual analog host compile failed: $LASTEXITCODE" }
& $analogExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Dual analog tests failed: $LASTEXITCODE" }
$syncState = [regex]::Match($mainSource,
    '(?s)/\* Synchronous digital state\. \*/(.*?)/\* End synchronous digital state\. \*/')
if (-not $syncState.Success) { throw 'Missing synchronous digital state.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'full_sync_state.inc'), $syncState.Groups[1].Value)
$fullPublish = Get-FunctionText $mainSource "static void Full_PublishGroup(uint32_t offset)`r`n{"
$fullReader = Get-FunctionText $mainSource '__root HAL_StatusTypeDef Full_GetSnapshot(Full_Snapshot *snapshot)'
$capture = Get-FunctionText $mainSource "void Acquisition_TIM2Update(uint32_t irq_entry_phase)`r`n{"
$faultCapture = Get-FunctionText $mainSource "static void DigitalSync_CaptureFault(uint32_t source, uint32_t ndtr, uint32_t trigger)`r`n{"
$halfCallback = Get-FunctionText $mainSource "void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)`r`n{"
$fullCallback = Get-FunctionText $mainSource "void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)`r`n{"
$irqSource = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Src/stm32f7xx_it.c'))
$timerIrq = Get-FunctionText $irqSource "void TIM2_IRQHandler(void)`r`n{"
$dmaIrq = Get-FunctionText $irqSource "void DMA2_Stream0_IRQHandler(void)`r`n{"
# Host-only scripted register timestamps. No hooks enter the production ISR.
$capture = $capture.Replace("DWT->CYCCNT", "timing_read_cycles()").Replace("TIM2->CNT", "timing_read_phase()")
$timerIrq = $timerIrq.Replace("DWT->CYCCNT", "timing_read_cycles()").Replace("TIM2->CNT", "timing_read_phase()")
$dmaIrq = $dmaIrq.Replace("DWT->CYCCNT", "timing_read_cycles()")
$halfCallback = $halfCallback.Replace("DWT->CYCCNT", "timing_read_cycles()")
$fullCallback = $fullCallback.Replace("DWT->CYCCNT", "timing_read_cycles()")
$fullPublish = $fullPublish.Replace(
    'full_published_snapshot.dig[i] = digital_sync.points[half * DIGITAL_SYNC_HALF + i];',
    'full_published_snapshot.dig[i] = digital_sync.points[half * DIGITAL_SYNC_HALF + i]; FULL_COPY_TEST_HOOK(i);')
$fullReader = $fullReader.Replace('candidate.dig[i] = full_published_snapshot.dig[i];',
    'candidate.dig[i] = full_published_snapshot.dig[i]; FULL_READ_TEST_HOOK(i);')
[IO.File]::WriteAllText((Join-Path $testOutput 'full_sync_under_test.inc'),
    ($faultCapture + "`r`n" + $publish + "`r`n" + $fullPublish + "`r`n" + $fullReader + "`r`n" + $capture + "`r`n" + $timerIrq + "`r`n" + $halfCallback + "`r`n" + $fullCallback + "`r`n" + $dmaIrq))
$gearState = [regex]::Match($mainSource,
    '(?s)/\* Acquisition gear state\. \*/(.*?)/\* End acquisition gear state\. \*/')
[IO.File]::WriteAllText((Join-Path $testOutput 'gear_state.inc'), $gearState.Groups[1].Value)
$gearDefines = [regex]::Matches($mainSource, '(?m)^#define ACQ_[^\r\n]+')
[IO.File]::WriteAllText((Join-Path $testOutput 'gear_defines.inc'),
    (($gearDefines | ForEach-Object { $_.Value }) -join "
"))
$counterFunction = Get-FunctionText $mainSource "static HAL_StatusTypeDef DigitalCounter_Init(void)`r`n{"
[IO.File]::WriteAllText((Join-Path $testOutput 'gear_counter.inc'), $counterFunction)
$gearFunctions = @(
    (Get-FunctionText $mainSource "static void Acquisition_Invalidate(void)
{"),
    (Get-FunctionText $mainSource "static uint32_t Acquisition_Stop(void)
{"),
    (Get-FunctionText $mainSource "static HAL_StatusTypeDef Acquisition_Fail(uint32_t reason)
{"),
    (Get-FunctionText $mainSource "static HAL_StatusTypeDef Acquisition_ConfigureADC(uint32_t sample_cycles)
{"),
    (Get-FunctionText $mainSource "static HAL_StatusTypeDef Acquisition_ApplyConfiguration(uint32_t gear, uint32_t sample_cycles)
{"),
    (Get-FunctionText $mainSource "static HAL_StatusTypeDef Acquisition_ApplyGear(uint32_t gear)
{"),
    (Get-FunctionText $mainSource '__root HAL_StatusTypeDef Acquisition_RequestGear(uint32_t gear)'),
    (Get-FunctionText $mainSource '__root HAL_StatusTypeDef Acquisition_RequestADCSampling(uint32_t sample_cycles)'),
    (Get-FunctionText $mainSource "static void Acquisition_ADCSamplingPoll(void)
{"),
    (Get-FunctionText $mainSource "static void Acquisition_GearPoll(void)
{")
)
[IO.File]::WriteAllText((Join-Path $testOutput 'gear_under_test.inc'), ($gearFunctions -join "
"))
$fullExe = Join-Path $testOutput 'test_full_sync.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $projectRoot 'Core/Src/digital_sync.c') `
    (Join-Path $PSScriptRoot 'test_full_sync.c') '-o' $fullExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Full sync host compile failed: $LASTEXITCODE" }
& $fullExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Full sync tests failed: $LASTEXITCODE" }
$gearExe = Join-Path $testOutput 'test_gear.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $projectRoot 'Core/Src/digital_sync.c') `
    (Join-Path $PSScriptRoot 'test_gear.c') '-o' $gearExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Gear host compile failed: $LASTEXITCODE" }
& $gearExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Gear host tests failed: $LASTEXITCODE" }
$adcSamplingExe = Join-Path $testOutput 'test_adc_sampling.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $projectRoot 'Core/Src/digital_sync.c') `
    (Join-Path $PSScriptRoot 'test_adc_sampling.c') '-o' $adcSamplingExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "ADC sampling host compile failed: $LASTEXITCODE" }
& $adcSamplingExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "ADC sampling tests failed: $LASTEXITCODE" }
$controlStatusExe = Join-Path $testOutput 'test_control_status.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $PSScriptRoot 'test_control_status.c') '-o' $controlStatusExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Control/status host compile failed: $LASTEXITCODE" }
& $controlStatusExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Control/status tests failed: $LASTEXITCODE" }
# Extract the same executor for DMA-service tests and real hardware-manager tests.
$gearCommandState = [regex]::Match($mainSource,
    '(?s)/\* UART6 gear command state: main-context only\. \*/(.*?)/\* End UART6 gear command state\. \*/')
if (-not $gearCommandState.Success) { throw 'Missing UART gear command state.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'uart6_gear_state.inc'), $gearCommandState.Groups[1].Value)
$gearCommandFunctions = @(
    (Get-FunctionText $mainSource ('static void UART6_DispatchControl(const DAQ_ControlCommand *command)' + [Environment]::NewLine + '{')),
    (Get-FunctionText $mainSource ('static uint32_t UART6_GearControlPoll(void)' + [Environment]::NewLine + '{'))
)
[IO.File]::WriteAllText((Join-Path $testOutput 'uart6_gear_under_test.inc'), ($gearCommandFunctions -join [Environment]::NewLine))
[IO.File]::WriteAllText((Join-Path $testOutput 'uart6_status_source.inc'),
    (Get-FunctionText $mainSource ('static uint32_t UART6_GetStatus(DAQ_StatusState *state)' + [Environment]::NewLine + '{')))
$controlState = [regex]::Match($mainSource,
    '(?s)/\* USART6 control receive/status state\. \*/(.*?)/\* End USART6 control receive/status state\. \*/')
$controlFunctions = [regex]::Match($mainSource,
    '(?s)(static void UART6_QueueStatus\(void\).*?)(?=/\* TIM2 update IRQ only:)')
if (-not $controlState.Success -or -not $controlFunctions.Success) { throw 'Missing UART control service.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'uart6_control_state.inc'),
    ([regex]::Replace($controlState.Groups[1].Value, '(?m)^#pragma[^\r\n]*', '')))
[IO.File]::WriteAllText((Join-Path $testOutput 'uart6_control_under_test.inc'), $controlFunctions.Groups[1].Value)
$controlDefines = [regex]::Matches($mainSource,
    '(?m)^#define (UART6_STATUS_TIMEOUT_MS|UART6_RX_BUFFER_SIZE|UART6_RX_COPY_SIZE|UART6_STATUS_PENDING_LIMIT)[^\r\n]+')
if ($controlDefines.Count -ne 4) { throw 'Missing UART receive/status limits.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'uart6_control_defines.inc'),
    (($controlDefines | ForEach-Object { $_.Value }) -join "`r`n"))
if ([regex]::Matches($mainSource, 'HAL_UART_Transmit\(').Count -ne 1) { throw 'More than one HAL TX entry.' }
if ($mainFunction -notmatch 'UART6_ControlStart\(\)') { throw 'RX/boot service not started.' }
if ($irqSource -notmatch 'HAL_DMA_IRQHandler\(&hdma_usart6_rx\)' -or
    $irqSource -notmatch 'UART6_ReceiveIRQ\(\)') { throw 'Missing RX IRQ dispatch.' }
$uartControlExe = Join-Path $testOutput 'test_uart6_control.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Core/Inc') '-I' $testOutput `
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c') `
    (Join-Path $projectRoot 'Core/Src/daq_control_stream.c') `
    (Join-Path $PSScriptRoot 'test_uart6_control.c') '-o' $uartControlExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "UART control host compile failed: $LASTEXITCODE" }
& $uartControlExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "UART control integration tests failed: $LASTEXITCODE" }
$cmd01Exe = Join-Path $testOutput 'test_cmd01.exe'
$cmd01Arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-I', (Join-Path $projectRoot 'Core/Inc'), '-I', $testOutput,
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c'),
    (Join-Path $projectRoot 'Core/Src/daq_control_stream.c'),
    (Join-Path $projectRoot 'Core/Src/digital_sync.c'),
    (Join-Path $PSScriptRoot 'test_cmd01.c'), '-o', $cmd01Exe)
& $Compiler @cmd01Arguments 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "CMD01 host compile failed: $LASTEXITCODE" }
& $cmd01Exe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "CMD01 integration tests failed: $LASTEXITCODE" }
$cmd02Exe = Join-Path $testOutput 'test_cmd02.exe'
$cmd02Arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-I', (Join-Path $projectRoot 'Core/Inc'), '-I', $testOutput,
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c'),
    (Join-Path $projectRoot 'Core/Src/daq_control_stream.c'),
    (Join-Path $PSScriptRoot 'test_cmd02.c'), '-o', $cmd02Exe)
& $Compiler @cmd02Arguments 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "CMD02 host compile failed: $LASTEXITCODE" }
& $cmd02Exe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "CMD02 integration tests failed: $LASTEXITCODE" }
$timingExe = Join-Path $testOutput 'test_timing_phase1.exe'
$timingArguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-DDIGITAL_TIMING_TEST', '-I', (Join-Path $projectRoot 'Core/Inc'), '-I', $testOutput,
    (Join-Path $projectRoot 'Core/Src/daq_protocol.c'),
    (Join-Path $projectRoot 'Core/Src/digital_sync.c'),
    (Join-Path $PSScriptRoot 'test_timing_phase1.c'), '-o', $timingExe)
& $Compiler @timingArguments 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Timing host compile failed: $LASTEXITCODE" }
& $timingExe 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "Timing phase1 tests failed: $LASTEXITCODE" }

# Stage18: actual offline manager and RX isolation bridge (no hardware).
$offlinePrivate = [regex]::Match($mainSource,
    '(?s)/\* Offline private state\. \*/(.*?)/\* End offline private state\. \*/')
if (-not $offlinePrivate.Success) { throw 'Missing offline private state.' }
[IO.File]::WriteAllText((Join-Path $testOutput 'offline_private_state.inc'), $offlinePrivate.Groups[1].Value)
$offlineFunctions = @(
    '__root HAL_StatusTypeDef Acquisition_RequestOfflineGEAR2(uint32_t groups)',
    'static uint32_t Offline_ClockReady(void)',
    'static void Offline_CaptureEnd(void)',
    'static void Offline_LatchFailure(uint32_t result)',
    'static void Offline_Finish(uint32_t result)',
    'static void Offline_Begin(uint32_t groups)',
    'static uint32_t Acquisition_OfflinePoll(void)'
) | ForEach-Object { Get-FunctionText $mainSource ($_ + [Environment]::NewLine + '{') }
[IO.File]::WriteAllText((Join-Path $testOutput 'offline_under_test.inc'), ($offlineFunctions -join [Environment]::NewLine))
$offlineUart = @('static HAL_StatusTypeDef Offline_UARTPause(void)',
    'static HAL_StatusTypeDef Offline_UARTResume(void)') |
    ForEach-Object { Get-FunctionText $mainSource ($_ + [Environment]::NewLine + '{') }
[IO.File]::WriteAllText((Join-Path $testOutput 'offline_uart_under_test.inc'), ($offlineUart -join [Environment]::NewLine))
if ($mainFunction -notmatch '(?s)Acquisition_OfflinePoll\(\).*?__WFI\(\);\s*continue;.*?Acquisition_GearPoll\(\)') {
    throw 'Offline loop does not exclude production gear/UART service.'
}
foreach ($suite in @('offline_gear2','offline_uart')) {
    $exe = Join-Path $testOutput ('test_' + $suite + '.exe')
    $argsList = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
        '-I', (Join-Path $projectRoot 'Core/Inc'), '-I', $testOutput,
        (Join-Path $projectRoot 'Core/Src/daq_protocol.c'),
        (Join-Path $projectRoot 'Core/Src/daq_control_stream.c'),
        (Join-Path $projectRoot 'Core/Src/digital_sync.c'),
        (Join-Path $PSScriptRoot ('test_' + $suite + '.c')), '-o', $exe)
    & $Compiler @argsList 2>&1 | ForEach-Object { $_ }
    if ($LASTEXITCODE -ne 0) { throw ("Offline host compile failed: " + $suite + ' ' + $LASTEXITCODE) }
    & $exe 2>&1 | ForEach-Object { $_ }
    if ($LASTEXITCODE -ne 0) { throw ("Offline host tests failed: " + $suite + ' ' + $LASTEXITCODE) }
}
Write-Output 'PASS all host software tests; UART is simulated; no board access.'
