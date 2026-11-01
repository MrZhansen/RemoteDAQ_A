"""Check nominal timing from actual source; does not measure MCU time or ADC quality."""
import fractions, json, math, pathlib, re, sys
root = pathlib.Path(__file__).resolve().parents[1]
main = (root/"Core/Src/main.c").read_text(encoding="utf-8")
config = (root/"Core/Inc/stm32f7xx_hal_conf.h").read_text(encoding="utf-8")
sync = (root/"Core/Src/digital_sync.c").read_text(encoding="utf-8")
irq = (root/"Core/Src/stm32f7xx_it.c").read_text(encoding="utf-8")
checks = 0
def check(value, label):
    global checks
    checks += 1
    if not value:
        raise AssertionError(label)
def number(pattern, text):
    match = re.search(pattern, text)
    check(match is not None, pattern)
    return int(match.group(1))
clock = main[main.index("void SystemClock_Config(void)\n{"):] if "void SystemClock_Config(void)\n{" in main else main[main.index("void SystemClock_Config(void)\r\n{"):]
check("RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;" in clock, "HSI source, not HSE")
hsi = number(r"#define HSI_VALUE\s+\(\(uint32_t\)(\d+)U\)", config)
m = number(r"PLL\.PLLM = (\d+);", clock)
n = number(r"PLL\.PLLN = (\d+);", clock)
p = number(r"PLL\.PLLP = RCC_PLLP_DIV(\d+);", clock)
sysclk = fractions.Fraction(hsi*n, m*p)
ahb = number(r"AHBCLKDivider = RCC_SYSCLK_DIV(\d+);", clock)
apb1 = number(r"APB1CLKDivider = RCC_HCLK_DIV(\d+);", clock)
apb2 = number(r"APB2CLKDivider = RCC_HCLK_DIV(\d+);", clock)
hclk, pclk1, pclk2 = sysclk/ahb, sysclk/ahb/apb1, sysclk/ahb/apb2
adcdiv = number(r"hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV(\d+);", main)
adcclk = pclk2/adcdiv
timerclk = pclk1*2
check((hclk,pclk1,pclk2,adcclk,timerclk) == (216000000,54000000,108000000,27000000,108000000), "nominal clocks")
check("htim2.Init.Prescaler = timer_clock / 1000000U - 1U;" in main and "htim2.Init.Period = timing->sample_interval_us - 1U;" in main, "timer arithmetic")
check("RCC_DCKCFGR1_TIMPRE" in main, "TIMPRE verified before candidate entry")
check("static const DigitalSyncTimingConfig timing_profiles[]" in sync, "single gear table")
check("{ 20U,  4U,  4U, 144U,  864U,  864U, 0U, 1U}" in sync, "GEAR2 row")
check("{ 10U,  2U,  4U,  28U,  540U,  972U, 1U, 1U}" in sync, "GEAR3 row")
check("{  2U,  1U,  1U,   3U,  216U,  390U, 1U, 1U}" in sync, "GEAR4 row")
check("if (gear >= ACQ_GEAR_COUNT)" in main, "unknown gears rejected")
check("(gear >= 2U) && (acquisition_udp_gear2_available == 0U)" in main, "UDP-only gate")
check("((200U - ndtr) / 2U) != index" in sync, "NDTR criterion unchanged")
for index in range(100):
    check((200-(200-2*index))//2 == index, "before rank1")
    check((200-(199-2*index))//2 == index, "after rank1")
    check((200-(198-2*index))//2 != index, "after rank2 rejected")
check("compiler prologue/epilogue" in irq and "accounting record's own update is outside" in irq, "ISR measurement boundary is explicit")
adc_latency = fractions.Fraction(2,adcclk) + fractions.Fraction(1,pclk2)
rows = []
gear_rows=((0,200,480),(1,100,480),(2,20,144),(3,10,28),(4,2,3))
for gear, period, cycles in gear_rows:
    arr = period-1
    check(fractions.Fraction((107+1)*(arr+1)*1000000,timerclk) == period, "TIM2 period")
    pair = fractions.Fraction(2*(cycles+12)*1000000,adcclk)
    including_latency = pair + adc_latency*1000000
    check(including_latency < period, "nominal pair fits")
    rows.append({"gear":gear,"period_us":period,"psc":107,"arr":arr,"sample_cycles":cycles,"pair_us":float(pair),"including_regular_trigger_latency_us":float(including_latency),"groups_per_second":1000000/period/50})
check(fractions.Fraction(2*(480+12)*1000000,adcclk) > 20, "480 cannot fit candidate")
uart_ms = 260*10/921600*1000
check(uart_ms > 1 and uart_ms < 5, "UART bandwidth bound")
response_upper = 4+1
pair144 = fractions.Fraction(2*(144+12)*1000000,adcclk)
check(response_upper < pair144, "CNT=4 upper bound precedes second conversion completion")
result = {
 "checks": checks, "nominal_only": True, "hardware_qualified": False,
 "source": "HSI nominal 16MHz; HAL_GetFreq reports configured frequency, not measured oscillator accuracy",
 "clocks_hz": {"hsi":hsi,"hclk":int(hclk),"pclk1":int(pclk1),"pclk2":int(pclk2),"adc":int(adcclk),"tim2":int(timerclk)},
 "profiles":rows,"regular_external_trigger_latency_us":float(adc_latency*1000000),
  "gear_budget_cycles":{"2_phase":864,"2_body":864,"3_phase":540,"3_body_exclusive":972,"4_phase":216,"4_body_exclusive":390},
 "CNT_4_upper_us_exclusive":response_upper,
 "uart_frame_ms":uart_ms,"wire_100M_us":326*8/100,"wire_10M_us":326*8/10,
 "assumptions":["12-bit ADC 12-cycle conversion; RM0385/table62 external-trigger latency model",
 "HSI absolute tolerance and real clock must be measured; relative ADC/timer clocks share same source",
 "No assumption about actual source impedance, VDDA, sensor bandwidth or MCU worst-case IRQ occupancy"]
}
pathlib.Path(sys.argv[1]).write_text(json.dumps(result,indent=2),encoding="utf-8")
print("PASS source-derived nominal sampling envelope: %d checks; not MCU measurement" % checks)
