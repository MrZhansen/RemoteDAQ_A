param([string]$Compiler='E:\mingw64\bin\gcc.exe',
      [Parameter(Mandatory=$true)][string]$OutputDirectory,
      [Parameter(Mandatory=$true)][string]$FixtureRoot)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$main=[IO.File]::ReadAllText((Join-Path $root 'Core/Src/main.c'))
function Function-Text([string]$Signature) {
  $start=$main.IndexOf($Signature+[Environment]::NewLine+'{',[StringComparison]::Ordinal)
  if($start -lt 0) { throw "Missing function $Signature" }
  $brace=$main.IndexOf('{',$start); $depth=1; $end=$brace+1
  while($depth -gt 0 -and $end -lt $main.Length) {
    if($main[$end] -eq '{') {$depth++}; if($main[$end] -eq '}') {$depth--}; $end++
  }
  if($depth -ne 0) {throw 'Function not balanced'}
  $main.Substring($start,$end-$start)
}
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'udp_production_policy.inc'),(Function-Text 'static void Acquisition_UDPPolicyPoll(void)'))
$trial=@('static uint32_t UDPTrial_Delta(','static void UDPTrial_Latch(void)','static void UDPTrial_Restore(uint32_t result)','static void DAQ_UDPTrialPoll(void)') | ForEach-Object {
  if($_.EndsWith('(')) { Function-Text 'static uint32_t UDPTrial_Delta(volatile uint32_t *current, volatile uint32_t *base)' }
  else {Function-Text $_}
}
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'udp_trial_under_test.inc'),($trial -join [Environment]::NewLine))
$baseArgs=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',(Join-Path $root 'Core/Inc'),'-I',$OutputDirectory,'-I',(Join-Path $FixtureRoot 'host_network'),'-I',(Join-Path $FixtureRoot 'host_network_baseline'))
foreach($suite in @('remote_gear2','remote_gear2_control','udp_trial')) {
  $exe=Join-Path $OutputDirectory ('test_'+$suite+'.exe')
  & $Compiler @baseArgs (Join-Path $root 'Core/Src/daq_protocol.c') (Join-Path $root 'Core/Src/daq_control_stream.c') (Join-Path $root 'Core/Src/digital_sync.c') (Join-Path $PSScriptRoot ('test_'+$suite+'.c')) '-o' $exe 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0) {throw "Compile $suite failed: $LASTEXITCODE"}
  & $exe 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0) {throw "Test $suite failed: $LASTEXITCODE"}
}
