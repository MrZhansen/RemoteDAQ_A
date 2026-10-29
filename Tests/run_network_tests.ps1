param([string]$Compiler='E:\mingw64\bin\gcc.exe', [Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
[xml]$project=[IO.File]::ReadAllText((Join-Path $root 'EWARM/RemoteDAQ_A.ewp'))
$sources=@($project.SelectNodes('//file/name') | ForEach-Object { $_.InnerText } | Where-Object { $_ -like '*LwIP/src/*.c' } | ForEach-Object { Join-Path $root ($_.Replace('$PROJ_DIR$/../','')) })
$argsList=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',(Join-Path $root 'Core/Inc'),'-I',(Join-Path $root 'Middlewares/Third_Party/LwIP/src/include'))
$argsList+=$sources
$argsList+=@((Join-Path $root 'Core/Src/daq_protocol.c'),(Join-Path $root 'Core/Src/daq_network.c'),(Join-Path $PSScriptRoot 'test_network.c'),'-o',(Join-Path $OutputDirectory 'test_network.exe'))
& $Compiler @argsList 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "Network test compile failed: $LASTEXITCODE" }
& (Join-Path $OutputDirectory 'test_network.exe') 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "Network test failed: $LASTEXITCODE" }

# Main integration uses the existing extracted HAL fixtures from full regression.
$main=[IO.File]::ReadAllText((Join-Path $root 'Core/Src/main.c'))
$state=[regex]::Match($main,'(?s)/\* Transport selection is local/preselected.*?(?=/\* USER CODE END PV \*/)')
if(-not $state.Success) { throw 'Missing transport state' }
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'transport_state.inc'),$state.Value)
$start=$main.IndexOf('static uint32_t DAQ_TransportGetStatus(DAQ_StatusState *state)')
$end=$main.IndexOf('/* USER CODE END 0 */',$start)
if($start -lt 0 -or $end -lt 0) { throw 'Missing transport service' }
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'transport_under_test.inc'),$main.Substring($start,$end-$start))
$legacy=Join-Path (Split-Path $OutputDirectory -Parent) 'host_network_baseline'
$exe=Join-Path $OutputDirectory 'test_transport.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' '-I' (Join-Path $root 'Core/Inc') '-I' $OutputDirectory '-I' $legacy (Join-Path $root 'Core/Src/daq_protocol.c') (Join-Path $root 'Core/Src/daq_control_stream.c') (Join-Path $PSScriptRoot 'test_transport.c') '-o' $exe 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "Transport test compile failed: $LASTEXITCODE" }
& $exe 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "Transport tests failed: $LASTEXITCODE" }

$ethExe=Join-Path $OutputDirectory 'test_ethernet.exe'
$ethArgs=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-Wno-unknown-pragmas','-I',(Join-Path $PSScriptRoot 'NetworkHAL'),'-I',(Join-Path $root 'Core/Inc'),'-I',(Join-Path $root 'Drivers/BSP/Components/lan8742'),'-I',(Join-Path $root 'Middlewares/Third_Party/LwIP/src/include'))
$ethArgs+=$sources
# Vendor v1.0.4 ClearIT has an unused Interrupt parameter. Keep its source
# unchanged; the one GCC exception applies only to that official object.
$phyObject=Join-Path $OutputDirectory 'lan8742_host.o'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' '-Wno-unused-parameter' '-I' (Join-Path $root 'Drivers/BSP/Components/lan8742') '-c' (Join-Path $root 'Drivers/BSP/Components/lan8742/lan8742.c') '-o' $phyObject 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "Official PHY host compile failed: $LASTEXITCODE" }
$ethArgs+=@($phyObject,(Join-Path $PSScriptRoot 'test_ethernet.c'),'-o',$ethExe)
& $Compiler @ethArgs 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "Ethernet adapter test compile failed: $LASTEXITCODE" }
foreach($scenario in 0..19) {
  & $ethExe $scenario 2>&1 | ForEach-Object { $_ }
  if($LASTEXITCODE -ne 0) { throw "Ethernet adapter scenario $scenario failed: $LASTEXITCODE" }
  Write-Output "PASS Ethernet adapter scenario $scenario"
}

function Get-TrialFunction([string]$source,[string]$signature) {
  $s=$source.IndexOf($signature,[StringComparison]::Ordinal)
  if($s -lt 0) { throw "Missing trial function $signature" }
  $b=$source.IndexOf('{',$s); $depth=1; $cursor=$b+1
  while($depth -gt 0 -and $cursor -lt $source.Length) {
    if($source[$cursor] -eq '{') { $depth++ }; if($source[$cursor] -eq '}') { $depth-- }; $cursor++
  }
  if($depth -ne 0) { throw 'Unclosed trial function' }
  $source.Substring($s,$cursor-$s)
}
$trialFunctions=@('static uint32_t UDPTrial_Delta(','static void UDPTrial_Latch(void)','static void UDPTrial_Restore(uint32_t result)','static void DAQ_UDPTrialPoll(void)') | ForEach-Object { Get-TrialFunction $main $_ }
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'udp_trial_under_test.inc'),($trialFunctions -join [Environment]::NewLine))
$trialExe=Join-Path $OutputDirectory 'test_udp_trial.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' '-I' (Join-Path $root 'Core/Inc') '-I' $OutputDirectory '-I' $legacy (Join-Path $root 'Core/Src/daq_protocol.c') (Join-Path $root 'Core/Src/daq_control_stream.c') (Join-Path $root 'Core/Src/digital_sync.c') (Join-Path $PSScriptRoot 'test_udp_trial.c') '-o' $trialExe 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "UDP GEAR2 trial compile failed: $LASTEXITCODE" }
& $trialExe 2>&1 | ForEach-Object { $_ }
if($LASTEXITCODE -ne 0) { throw "UDP GEAR2 trial tests failed: $LASTEXITCODE" }
