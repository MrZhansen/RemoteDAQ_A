param([string]$Compiler='E:\mingw64\bin\gcc.exe',
      [Parameter(Mandatory=$true)][string]$OutputDirectory,
      [Parameter(Mandatory=$true)][string]$FixtureRoot)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$main=[IO.File]::ReadAllText((Join-Path $root 'Core/Src/main.c'))
$header=[IO.File]::ReadAllText((Join-Path $root 'Core/Inc/main.h'))
function Function-Text([string]$Signature) {
  $start=$main.IndexOf($Signature+[Environment]::NewLine+'{',[StringComparison]::Ordinal)
  if($start -lt 0){throw "Missing function $Signature"}
  $brace=$main.IndexOf('{',$start);$depth=1;$end=$brace+1
  while($depth -gt 0 -and $end -lt $main.Length) {
    if($main[$end] -eq '{'){$depth++};if($main[$end] -eq '}'){$depth--};$end++
  }
  if($depth -ne 0){throw 'Function not balanced'}
  $main.Substring($start,$end-$start)
}
$state=[regex]::Match($main,'(?s)volatile uint32_t square_wave_running.*?(?=/\* USART6 control receive/status state\. \*/)')
if(-not $state.Success){throw 'Missing square-wave state'}
$defines=[regex]::Matches($header+$main,'(?m)^#define (?:SQUARE_WAVE|DAQ_TRIGGER)_[A-Z0-9_]+[^\r\n]+') | ForEach-Object {$_.Value}
$functions=@('HAL_StatusTypeDef SquareWave_Init(uint32_t frequency_hz)',
  'HAL_StatusTypeDef SquareWave_Start(void)',
  '__root HAL_StatusTypeDef SquareWave_Stop(void)',
  'static HAL_StatusTypeDef SquareWave_ChangeFrequency(uint32_t frequency_hz)') |
  ForEach-Object {Function-Text $_}
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'square_wave_api_under_test.inc'),
  (($defines -join [Environment]::NewLine)+[Environment]::NewLine+$state.Value+($functions -join [Environment]::NewLine)))
$transportState=[regex]::Match($main,'(?s)/\* Transport selection is local/preselected.*?(?=/\* USER CODE END PV \*/)')
if(-not $transportState.Success){throw 'Missing transport state'}
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'transport_state.inc'),$transportState.Value)
$transportStart=$main.IndexOf('static uint32_t DAQ_TransportGetStatus(DAQ_StatusState *state)')
$transportEnd=$main.IndexOf('/* USER CODE END 0 */',$transportStart)
if($transportStart -lt 0 -or $transportEnd -lt 0){throw 'Missing transport service'}
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'transport_under_test.inc'),
  $main.Substring($transportStart,$transportEnd-$transportStart))
$base=@('-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',(Join-Path $root 'Core/Inc'),
  '-I',$OutputDirectory,'-I',$FixtureRoot,'-I',(Join-Path $FixtureRoot 'host_network'),'-I',(Join-Path $FixtureRoot 'host_network_baseline'))
foreach($suite in @('square_wave_control','cmd07')) {
  $exe=Join-Path $OutputDirectory ('test_'+$suite+'.exe')
  & $Compiler @base (Join-Path $root 'Core/Src/daq_protocol.c') (Join-Path $root 'Core/Src/daq_control_stream.c') (Join-Path $root 'Core/Src/digital_sync.c') (Join-Path $PSScriptRoot ('test_'+$suite+'.c')) '-o' $exe 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0){throw "Compile $suite failed: $LASTEXITCODE"}
  & $exe 2>&1 | ForEach-Object {$_}
  if($LASTEXITCODE -ne 0){throw "Test $suite failed: $LASTEXITCODE"}
}
