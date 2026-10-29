param([string]$Compiler='E:\mingw64\bin\gcc.exe', [Parameter(Mandatory=$true)][string]$OutputDirectory, [switch]$ConfirmedInterfaceOnly, [string[]]$AdapterOnly=@())
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
[xml]$project=[IO.File]::ReadAllText((Join-Path $root 'EWARM/RemoteDAQ_A.ewp'))
$sources=@($project.SelectNodes('//file/name') | ForEach-Object {$_.InnerText} | Where-Object {$_ -like '*LwIP/src/*.c'} | ForEach-Object {Join-Path $root ($_.Replace('$PROJ_DIR$/../',''))})
$base=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',(Join-Path $root 'Core/Inc'),'-I',(Join-Path $root 'Middlewares/Third_Party/LwIP/src/include'))
$phy=Join-Path $OutputDirectory 'lan8742_host.o'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' '-Wno-unused-parameter' '-I' (Join-Path $root 'Drivers/BSP/Components/lan8742') '-c' (Join-Path $root 'Drivers/BSP/Components/lan8742/lan8742.c') '-o' $phy 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) { throw "PHY host compile failed: $LASTEXITCODE" }
$adapter=Join-Path $OutputDirectory 'test_network_adapter.exe'
$adapterArgs=@('-I',(Join-Path $PSScriptRoot 'NetworkHAL'))+$base+@('-Wno-unknown-pragmas','-I',(Join-Path $root 'Drivers/BSP/Components/lan8742'))+$sources+@($phy,(Join-Path $root 'Core/Src/daq_protocol.c'),(Join-Path $root 'Core/Src/daq_network.c'),(Join-Path $PSScriptRoot 'test_network_adapter.c'),'-o',$adapter)
& $Compiler @adapterArgs 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) { throw "Service/adapter host compile failed: $LASTEXITCODE" }
if(-not $ConfirmedInterfaceOnly -and $AdapterOnly.Count -eq 0) {
foreach($id in @('0x10010449','0x10000449','0x10020449','0x10010451','0x00000000')) {
  & $adapter 'gate' $id 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0) { throw "Identity integration failed $id : $LASTEXITCODE" }
}
}
$scenarios=if($AdapterOnly.Count -gt 0) {$AdapterOnly} elseif($ConfirmedInterfaceOnly) {@('rx')} else {@('unowned','duplicate','cycle','rx','rx_fault','tx_release_fault','poll_release_fault','rx_input_fault','send_frames','padding')}
foreach($scenario in $scenarios) {
  & $adapter $scenario 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0) { throw "RX integration failed $scenario : $LASTEXITCODE" }
}
if($AdapterOnly.Count -gt 0) { return }
$contract=Join-Path $OutputDirectory 'test_udp_contract.exe'
$contractArgs=$base+$sources+@((Join-Path $root 'Core/Src/daq_protocol.c'),(Join-Path $root 'Core/Src/daq_network.c'),(Join-Path $PSScriptRoot 'test_udp_contract.c'),'-o',$contract)
& $Compiler @contractArgs 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) { throw "UDP contract host compile failed: $LASTEXITCODE" }
$vectors=Join-Path $OutputDirectory 'interop_vectors'
New-Item -ItemType Directory -Path $vectors -Force | Out-Null
& $contract $vectors 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) { throw "UDP contract host test failed: $LASTEXITCODE" }
Write-Output 'PASS all new targeted tests; host simulation only; no hardware or physical network.'
