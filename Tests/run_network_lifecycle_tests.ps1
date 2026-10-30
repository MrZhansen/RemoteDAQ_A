param([string]$Compiler='E:\mingw64\bin\gcc.exe',[Parameter(Mandatory=$true)][string]$OutputDirectory,[Parameter(Mandatory=$true)][string]$FixtureRoot,[switch]$HandshakeOnly)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
[xml]$project=[IO.File]::ReadAllText((Join-Path $root 'EWARM/RemoteDAQ_A.ewp'))
$sources=@($project.SelectNodes('//file/name') | ForEach-Object {$_.InnerText} | Where-Object {$_ -like '*LwIP/src/*.c'} | ForEach-Object {Join-Path $root ($_.Replace('$PROJ_DIR$/../',''))})
if(-not $HandshakeOnly) {
foreach($packingMode in @('gcc','iar_macro_branch')) {
$layout=Join-Path $OutputDirectory ('test_lwip_layout_'+$packingMode+'.exe')
$layoutArgs=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',(Join-Path $root 'Core/Inc'),'-I',(Join-Path $root 'Middlewares/Third_Party/LwIP/src/include'))
if($packingMode -eq 'iar_macro_branch') { $layoutArgs+=@('-D__ICCARM__','-D__IAR_SYSTEMS_ICC__') }
& $Compiler @layoutArgs (Join-Path $PSScriptRoot 'test_lwip_layout.c') '-o' $layout 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) {throw "Layout compile $packingMode failed: $LASTEXITCODE"}
& $layout 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) {throw "Layout test $packingMode failed: $LASTEXITCODE"}
Write-Output "PASS host layout mode $packingMode; actual IAR verification is a separate target build."
}
$exe=Join-Path $OutputDirectory 'test_network_failures.exe'
$compile=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',(Join-Path $root 'Core/Inc'),'-I',(Join-Path $root 'Middlewares/Third_Party/LwIP/src/include'))+$sources+@((Join-Path $root 'Core/Src/daq_protocol.c'),(Join-Path $root 'Core/Src/daq_network.c'),(Join-Path $PSScriptRoot 'test_network_failures.c'),'-Wl,--wrap=udp_new,--wrap=udp_bind,--wrap=lwip_init','-o',$exe)
& $Compiler @compile 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) {throw "Network failure test compile failed: $LASTEXITCODE"}
foreach($scenario in @('new','bind','init','guard','lwip_assert','fault','healthy')) {
  & $exe $scenario 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0) {throw "Network lifecycle $scenario failed: $LASTEXITCODE"}
}
}
$handshake=Join-Path $OutputDirectory 'test_udp_handshake.exe'
& $Compiler '-std=c11' '-O2' '-Wall' '-Wextra' '-Werror' '-pedantic' '-I' (Join-Path $root 'Core/Inc') '-I' (Join-Path $FixtureRoot 'host_network') '-I' (Join-Path $FixtureRoot 'host_network_baseline') (Join-Path $root 'Core/Src/daq_protocol.c') (Join-Path $root 'Core/Src/daq_control_stream.c') (Join-Path $PSScriptRoot 'test_udp_handshake.c') '-o' $handshake 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) {throw "Handshake scheduling compile failed: $LASTEXITCODE"}
& $handshake 2>&1 | ForEach-Object {$_}
if($LASTEXITCODE -ne 0) {throw "Handshake scheduling test failed: $LASTEXITCODE"}
