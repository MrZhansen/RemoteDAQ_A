# Host-side static gate for the GEAR3/GEAR4 fast-ISR instruction-fetch fix.
# Verifies that the accelerators the cycle budgets assume are actually enabled,
# that D-Cache stays off (DMA coherence), and that no cycle budget was relaxed.
$projectRoot = Split-Path -Parent $PSScriptRoot
$conf = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Inc/stm32f7xx_hal_conf.h'))
$main = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Src/main.c'))
$sync = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Src/digital_sync.c'))
$checks = 0
function Check([bool]$condition, [string]$message)
{
    if (-not $condition) { throw "FAIL: $message" }
    $script:checks++
}

Check ($conf -match '(?m)^#define\s+PREFETCH_ENABLE\s+1U') 'PREFETCH_ENABLE must be 1U'
Check ($conf -match '(?m)^#define\s+ART_ACCELERATOR_ENABLE\s+1U') 'ART_ACCELERATOR_ENABLE must be 1U'

$mainStart = $main.IndexOf('int main(void)')
Check ($mainStart -ge 0) 'main() not found'
$mainBody = $main.Substring($mainStart)
$icache = $mainBody.IndexOf('SCB_EnableICache();')
$halInit = $mainBody.IndexOf('HAL_Init();')
Check ($icache -ge 0) 'main() must call SCB_EnableICache()'
Check ($halInit -gt $icache) 'I-Cache must be enabled before HAL_Init()'
Check ($mainBody -match 'adc_startup_icache\s*=\s*\(\(SCB->CCR & SCB_CCR_IC_Msk\) != 0U\)') 'missing I-Cache evidence capture'
Check ($mainBody -match 'adc_startup_flash_acr\s*=\s*FLASH->ACR') 'missing FLASH->ACR evidence capture'
Check ($main -notmatch 'SCB_EnableDCache\s*\(') 'D-Cache must stay disabled for DMA coherence'

# Budgets must remain exactly as qualified; the fix is speed, not loosening.
Check ($sync -match '\{\s*10U,\s*2U,\s*4U,\s*28U,\s*540U,\s*972U,\s*1U,\s*1U\s*\}') 'GEAR3 profile must stay 28 cycles / phase 540 / body 972'
Check ($sync -match '\{\s*2U,\s*1U,\s*1U,\s*3U,\s*216U,\s*390U,\s*1U,\s*1U\s*\}') 'GEAR4 profile must stay 3 cycles / phase 216 / body 390'
Check ($main -match '\(elapsed >= acquisition_fast_irq_budget_cycles\)') 'body budget check must stay fail-closed (>=)'
Check ($main -match 'DIGITAL_SYNC_DETAIL_BODY_BUDGET') 'body budget fault detail must stay latched'
Check ($main -match 'if \(digital_sync\.error != 0U\) \{ DigitalSync_CaptureFault\(1U, ndtr, trigger\); \}') 'fast path must keep the first-fault capture'

# GEAR3/4 fast path must execute from ITCM, not from flash.
$header = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Inc/digital_sync.h'))
$icf = [IO.File]::ReadAllText((Join-Path $projectRoot 'EWARM/stm32f746xx_flash.icf'))
$it = [IO.File]::ReadAllText((Join-Path $projectRoot 'Core/Src/stm32f7xx_it.c'))
Check ($header -match '#if defined\(__ICCARM__\)' -and $header -match '#define DIGITAL_SYNC_FAST_CODE __ramfunc') 'DIGITAL_SYNC_FAST_CODE must map to __ramfunc on IAR'
Check ($header -match '(?s)#else\s*#define DIGITAL_SYNC_FAST_CODE\s*#endif') 'DIGITAL_SYNC_FAST_CODE must be empty off-target'
Check ($main -match '(?m)^DIGITAL_SYNC_FAST_CODE\r?\nvoid Acquisition_TIM2FastUpdate\(') 'Acquisition_TIM2FastUpdate must be __ramfunc'
Check ($main -match '(?m)^DIGITAL_SYNC_FAST_CODE\r?\nstatic void DigitalSync_CaptureFault\(') 'DigitalSync_CaptureFault must be __ramfunc'
Check ($sync -match '(?m)^DIGITAL_SYNC_FAST_CODE\r?\nuint32_t DigitalSync_RecordFast\(') 'DigitalSync_RecordFast must be __ramfunc'
Check ($sync -match '(?m)^DIGITAL_SYNC_FAST_CODE\r?\nvoid DigitalSync_FaultWithDetail\(') 'DigitalSync_FaultWithDetail must be __ramfunc'
Check ($icf -match 'place in ITCMRAM_region \{ section \.textrw \};') 'ICF must place .textrw in ITCM'
Check ($icf -match 'initialize by copy \{ readwrite \}') 'ICF must keep initialization by copy for .textrw'

# GEAR4 alignment: the DMA write pointer is captured at ISR entry.
Check ($it -match 'Acquisition_TIM2FastUpdate\(start, hdma_adc3\.Instance->NDTR\);') 'TIM2 fast ISR must capture NDTR at entry'
$fastBody = [regex]::Match($main, '(?s)void Acquisition_TIM2FastUpdate\(uint32_t irq_entry_cycles, uint32_t entry_ndtr\)\r?\n\{(.*?)\r?\n\}').Groups[1].Value
Check ($fastBody.Length -gt 0) 'fast update body not found'
Check ($fastBody -match 'uint32_t ndtr = entry_ndtr;') 'fast update must use the entry-time NDTR'
Check ($fastBody -notmatch 'hdma_adc3\.Instance->NDTR') 'fast update must not read NDTR directly'

Write-Output "PASS fast-ISR fetch configuration: $checks checks"
