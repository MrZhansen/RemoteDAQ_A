/* Host simulation of actual TIM2 ISR, publisher and reader. Timer/ADC register
 * progress is modeled explicitly; this does NOT measure interrupt timing. */
#include "daq_protocol.h"
#include "digital_sync.h"
#include "acquisition_offline.h"
#include "ch1_snapshot_type.inc"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { uint32_t CNT, PSC, ARR, SR, DIER, CR1, CR2, EGR; } FakeTimer;
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload; } FakeTimInit;
typedef struct { FakeTimer *Instance; FakeTimInit Init; } TIM_HandleTypeDef;
typedef struct { uint32_t NDTR, CR; } FakeDma;
typedef struct { FakeDma *Instance; } FakeDmaHandle;
typedef struct { uint32_t IDR; } FakeGpio;
typedef struct { uint32_t CR2, SR, SMPR2, SQR3; } FakeAdc;
typedef struct { FakeAdc *Instance; } ADC_HandleTypeDef;
static FakeAdc adc3;
static ADC_HandleTypeDef hadc3 = { &adc3 };
#define ADC3 (&adc3)
static FakeTimer timer2, timer5;
static FakeDma dma;
static struct { uint32_t LISR; } fake_dma2;
#define DMA2 (&fake_dma2)
#define DMA_LISR_FEIF0 1U
#define DMA_LISR_DMEIF0 4U
#define DMA_LISR_TEIF0 8U
#define DMA_LISR_HTIF0 16U
#define DMA_LISR_TCIF0 32U
static uint32_t dma2_stream0_irq_count, dma2_stream0_last_lisr;
static uint32_t timing_publish_cost;
static FakeDmaHandle hdma_adc3 = { &dma };
static TIM_HandleTypeDef htim2 = { .Instance = &timer2 };
static FakeGpio gpio_g, gpio_i;
#define TIM2 (&timer2)
#define TIM5 (&timer5)
#define GPIOG (&gpio_g)
#define GPIOI (&gpio_i)
#define TIM_SR_UIF 1U
#define TIM_DIER_UIE 1U
#define TIM_IT_UPDATE 1U
#define __HAL_TIM_DISABLE_IT(handle, mask) ((handle)->Instance->DIER &= ~(mask))
#define __root
#define CH1_GROUP_SAMPLES CH1_SNAPSHOT_SAMPLES
#define ADC_SCAN_CHANNELS 2U
#define ADC_DMA_HALF_SAMPLES 100U
static uint32_t SystemCoreClock = 216000000U;
static volatile uint16_t adc_samples[50], adc_ch2_samples[50], adc_dma_buffer[200];
static volatile uint32_t adc_snapshot_seq, adc_snapshot_valid, adc_group_count;
static volatile uint32_t adc_group_ready, adc_running, adc_value, adc_ch2_value;
static volatile uint32_t adc_group_period_ms, adc_group_cycles, adc_last_group_tick;
static uint32_t adc_previous_group_cycle;
static volatile uint32_t adc_half_callback_count, adc_full_callback_count;
static struct { uint32_t CYCCNT, CTRL; } fake_dwt;
#define DWT (&fake_dwt)
static uint32_t timing_phases[8], timing_phase_count, timing_phase_pos;
static uint32_t timing_cycles[8], timing_cycle_count, timing_cycle_pos;
static uint32_t timing_read_phase(void)
{
  return (timing_phase_pos < timing_phase_count) ?
      timing_phases[timing_phase_pos++] : timer2.CNT;
}
static uint32_t timing_read_cycles(void)
{
  return (timing_cycle_pos < timing_cycle_count) ?
      timing_cycles[timing_cycle_pos++] : fake_dwt.CYCCNT;
}
static uint32_t model_tick = 100U;
static uint32_t HAL_GetTick(void) { return model_tick; }
static unsigned int checks;
#define CHECK(c) do { checks++; if (!(c)) { \
  fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while(0)

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
/* Scripted HAL dispatch cost, not an MCU timing claim. */
static void HAL_DMA_IRQHandler(FakeDmaHandle *handle)
{
  uint32_t flags=fake_dma2.LISR;
  CHECK(handle==&hdma_adc3);
  fake_dwt.CYCCNT+=17U;
  if((flags & (DMA_LISR_FEIF0|DMA_LISR_DMEIF0|DMA_LISR_TEIF0))==0U)
  {
    if((flags&DMA_LISR_HTIF0)!=0U) { HAL_ADC_ConvHalfCpltCallback(&hadc3); }
    if((flags&DMA_LISR_TCIF0)!=0U) { HAL_ADC_ConvCpltCallback(&hadc3); }
  }
  fake_dma2.LISR=0U;
  fake_dwt.CYCCNT+=23U;
}
static uint32_t pair_index, serial, pattern_mode;
static uint32_t inject_copy, inject_read;
static void copy_hook(uint32_t i);
static void read_hook(uint32_t i);
#define FULL_COPY_TEST_HOOK(i) copy_hook(i)
#define FULL_READ_TEST_HOOK(i) read_hook(i)
#define __DMB() ((void)0)
#include "gear_defines.inc"
#ifdef GEAR_TEST
#include "gear_state.inc"
#else
static volatile uint32_t acquisition_active_gear = 0U;
static volatile uint32_t full_group_publish_count;
#endif
#include "full_sync_state.inc"
#include "full_sync_under_test.inc"

static uint8_t pattern(uint32_t point)
{
  if (pattern_mode == 0U) { return 0U; }
  if (pattern_mode == 1U) { return 7U; }
  if (pattern_mode == 2U) { return (uint8_t)(point & 7U); }
  return (uint8_t)(((point / (pattern_mode - 2U)) & 1U) << (pattern_mode - 3U));
}

static void reset_model(void)
{
  acquisition_active_gear = 0U;
  full_group_publish_count = 0U;
  DigitalSync_Init(&digital_sync);
  memset((void *)&digital_timing, 0, sizeof(digital_timing));
  timing_phase_count=0U; timing_phase_pos=0U; timing_cycle_count=0U; timing_cycle_pos=0U;
  fake_dwt.CYCCNT=0U; digital_irq_max_cycles=0U; digital_read_phase_max_us=0U;
  fake_dma2.LISR=0U; dma2_stream0_irq_count=0U; dma2_stream0_last_lisr=0U; timing_publish_cost=0U;
  adc_half_callback_count=0U; adc_full_callback_count=0U;
  full_snapshot_seq = 0U; full_snapshot_valid = 0U;
  adc_group_count = 0U; adc_snapshot_seq = 0U; adc_snapshot_valid = 0U;
  adc_running = 1U; pair_index = 0U; serial = 0U;
  inject_copy = 0U; inject_read = 0U;
  timer2.CNT = 5U; timer2.DIER = 1U; timer2.SR = 0U; timer2.CR1 = 1U; timer2.ARR=199U;
  timer5.CNT = 0U; dma.NDTR = 200U;
  htim5.Instance = TIM5; /* Counter initialization itself is checked by IAR. */
}

static void generate(uint32_t count, uint32_t publish)
{
  uint32_t n;
  for (n = 0U; n < count; n++)
  {
    uint8_t bits = pattern(serial);
    timer5.CNT++; /* Hardware counts every TRGO, regardless of NVIC delivery. */
    gpio_g.IDR = (uint32_t)(bits & 3U) << 6U;
    gpio_i.IDR = (uint32_t)(bits >> 2U) << 3U;
    dma.NDTR = 200U - pair_index * 2U; /* Before current scan has completed. */
    timer2.SR = 1U;
    TIM2_IRQHandler();
    adc_dma_buffer[2U * pair_index] = (uint16_t)((1001U + serial) & 0xFFFU);
    adc_dma_buffer[2U * pair_index + 1U] = (uint16_t)((1U + serial) & 0xFFFU);
    serial++; pair_index++;
    dma.NDTR = (pair_index == 100U) ? 200U : 200U - pair_index * 2U;
    if (publish != 0U && (pair_index == 50U || pair_index == 100U))
    {
      if (pair_index == 50U) { HAL_ADC_ConvHalfCpltCallback(&hadc3); }
      else { HAL_ADC_ConvCpltCallback(&hadc3); }
    }
    if (pair_index == 100U) { pair_index = 0U; }
  }
}

static void copy_hook(uint32_t i)
{
  if(i==49U) { fake_dwt.CYCCNT+=timing_publish_cost; }
  if (inject_copy != 0U && i == 25U)
  {
    inject_copy = 0U;
    generate(51U, 0U); /* Reuse the source half in the middle of copying. */
  }
}

static void read_hook(uint32_t i)
{
  if (inject_read != 0U && i == 25U)
  {
    inject_read = 0U;
    generate(50U, 1U); /* Publish a whole new group during reader copying. */
  }
}

static uint16_t reference_crc(const uint8_t *p, uint32_t n)
{
  uint32_t r=0xFFFFU, i;
  int bit;
  for(i=0U;i<n;i++)
  {
    r=(r<<8U)^((uint32_t)p[i]<<16U);
    for(bit=23;bit>=16;bit--) { if(r&((uint32_t)1U<<bit)) { r^=(uint32_t)0x11021U<<(bit-16); } }
  }
  return (uint16_t)r;
}

int main(void)
{
  Full_Snapshot out, saved;
  uint8_t frame[260], previous[260];
  uint32_t mode, group, i, old_count;
  uint16_t crc;
  for (mode=0U;mode<6U;mode++)
  {
    reset_model(); pattern_mode=mode;
    memset(&out,0x5A,sizeof(out)); memcpy(&saved,&out,sizeof(saved));
    CHECK(Full_GetSnapshot(NULL)==HAL_ERROR);
    CHECK(Full_GetSnapshot(&out)==HAL_BUSY);
    generate(49U,0U);
    CHECK(digital_sync.half_ready[0]==0U && digital_sync.sample_count==49U);
    CHECK(Full_GetSnapshot(&out)==HAL_BUSY);
    CHECK(memcmp(&out,&saved,sizeof(out))==0);
    generate(1U,1U);
    for(group=1U;group<=4U;group++)
    {
      CHECK(Full_GetSnapshot(&out)==HAL_OK && out.group_count==group);
      CHECK(digital_sync.completed_groups==group && adc_group_count==group);
      CHECK(digital_sync.error==0U && digital_sync.sample_count==group*50U);
      for(i=0U;i<50U;i++)
      {
        uint32_t point=(group-1U)*50U+i;
        CHECK(out.ch1[i]==point+1U && out.ch2[i]==point+1001U);
        CHECK(out.dig[i]==pattern(point) && out.dig[i]<8U);
      }
      CHECK(DAQ_BuildFullFrame(out.ch1,out.ch2,out.dig,(uint16_t)group,frame,sizeof(frame))==DAQ_FRAME_OK);
      CHECK(frame[0]==0xAAU && frame[1]==0x55U && frame[2]==1U);
      CHECK(frame[3]==group && frame[4]==0U && frame[5]==0U && frame[6]==0x1FU && frame[7]==50U);
      for(i=0U;i<50U;i++)
      {
        CHECK(frame[12U+5U*i]==out.dig[i]);
        CHECK((uint16_t)(frame[8U+5U*i]|((uint16_t)frame[9U+5U*i]<<8U))==(uint16_t)(out.ch1[i]*16U));
        CHECK((uint16_t)(frame[10U+5U*i]|((uint16_t)frame[11U+5U*i]<<8U))==(uint16_t)(out.ch2[i]*16U));
      }
      crc=reference_crc(frame,258U);
      CHECK(crc==(uint16_t)(frame[258]|((uint16_t)frame[259]<<8U)));
      if(group<4U) { generate(50U,1U); }
    }
  }
  /* Invalid DIG values anywhere must leave the complete previous frame intact. */
  memcpy(previous,frame,sizeof(frame));
  for(i=0U;i<50U;i++)
  {
    uint8_t old=out.dig[i]; out.dig[i]=8U;
    CHECK(DAQ_BuildFullFrame(out.ch1,out.ch2,out.dig,0U,frame,sizeof(frame))==DAQ_FRAME_DIG_OUT_OF_RANGE);
    CHECK(memcmp(previous,frame,sizeof(frame))==0); out.dig[i]=old;
  }
  reset_model(); pattern_mode=2U;
  generate(49U,0U); Full_PublishGroup(0U); /* Incomplete initial digital half. */
  CHECK(full_snapshot_valid==0U && digital_sync.error!=0U);
  CHECK(adc_running==1U && timer2.CR1==1U && timer2.DIER==0U);
  reset_model(); generate(50U,1U);
  CHECK(Full_GetSnapshot(&out)==HAL_OK); memcpy(&saved,&out,sizeof(saved));
  inject_read=1U;
  CHECK(Full_GetSnapshot(&out)==HAL_BUSY && memcmp(&saved,&out,sizeof(out))==0);
  CHECK(Full_GetSnapshot(&out)==HAL_OK && out.group_count==2U);
  reset_model(); generate(50U,0U); inject_copy=1U; Full_PublishGroup(0U);
  CHECK(full_snapshot_valid==0U && digital_sync.error!=0U); /* Reused half rejected. */
  reset_model(); generate(2U,0U); timer5.CNT+=100U;
  generate(1U,0U); /* NDTR aliases its old index, hardware event count does not. */
  CHECK(digital_sync.error==DIGITAL_SYNC_ERROR_ALIGN && digital_sync.sample_count==2U);
  CHECK(timer2.DIER==0U && timer2.CR1==1U && adc_running==1U);
  reset_model(); timer2.CNT=21U; generate(1U,0U);
  CHECK(digital_sync.error==DIGITAL_SYNC_ERROR_LATE && digital_sync.sample_count==0U);
  reset_model(); timer2.SR=0U; TIM2_IRQHandler();
  CHECK(digital_sync.sample_count==0U); /* Stale/spurious vector cannot add a point. */
  reset_model(); timer5.CNT=UINT32_MAX; digital_sync.last_trigger=UINT32_MAX;
  generate(1U,0U); CHECK(digital_sync.error==0U && digital_sync.last_trigger==0U);
  reset_model(); digital_sync.completed_groups=UINT32_MAX; adc_group_count=UINT32_MAX;
  full_snapshot_seq=UINT32_MAX-1U; generate(50U,1U);
  CHECK(Full_GetSnapshot(&out)==HAL_OK && out.group_count==0U && full_snapshot_seq==0U);
  old_count=digital_sync.sample_count; digital_sync.error=DIGITAL_SYNC_ERROR_CONFIG;
  generate(1U,0U); CHECK(digital_sync.sample_count==old_count);
  /* Profile defaults and the original strict phase/NDTR/trigger semantics. */
  {
    const DigitalSyncTimingConfig *p0=DigitalSync_GetTimingConfig(0U);
    const DigitalSyncTimingConfig *p1=DigitalSync_GetTimingConfig(1U);
    DigitalSync state;
    CHECK(p0!=NULL && p0->sample_interval_us==200U);
    CHECK(p1!=NULL && p1->sample_interval_us==100U);
    CHECK(p0->max_phase_us==20U && p1->max_phase_us==20U);
    CHECK(p0->max_body_us==20U && p1->max_body_us==20U);
    CHECK(DigitalSync_GetTimingConfig(2U)==NULL && DigitalSync_GetTimingConfig(UINT32_MAX)==NULL);
    for(i=0U;i<2U;i++)
    {
      DigitalSync_Init(&state); state.timing=*DigitalSync_GetTimingConfig(i);
      CHECK(DigitalSync_Record(&state,1U,200U,20U,7U)==1U);
      CHECK(DigitalSync_Record(&state,2U,197U,20U,0U)==1U); /* Rank1 already moved. */
      CHECK(state.next_index==2U && state.error==0U);
      CHECK(DigitalSync_Record(&state,3U,196U,21U,0U)==0U);
      CHECK(state.error==DIGITAL_SYNC_ERROR_LATE && state.fault_detail==DIGITAL_SYNC_DETAIL_PHASE);
    }
    DigitalSync_Init(&state);
    CHECK(state.timing.sample_interval_us==200U);
    CHECK(DigitalSync_Record(&state,1U,198U,1U,0U)==0U);
    CHECK(state.fault_detail==DIGITAL_SYNC_DETAIL_NDTR_INDEX && state.sample_count==0U);
    DigitalSync_Init(&state); CHECK(DigitalSync_Record(&state,1U,0U,1U,0U)==0U);
    CHECK(state.fault_detail==DIGITAL_SYNC_DETAIL_NDTR_RANGE);
    DigitalSync_Init(&state); CHECK(DigitalSync_Record(&state,2U,200U,1U,0U)==0U);
    CHECK(state.fault_detail==DIGITAL_SYNC_DETAIL_TRIGGER);
    DigitalSync_Init(&state); CHECK(DigitalSync_Record(&state,1U,200U,1U,8U)==0U);
    CHECK(state.fault_detail==DIGITAL_SYNC_DETAIL_BITS);
  }
  /* Scripted timestamps validate accounting only, not measured MCU timings. */
  reset_model();
  timing_phases[0]=1U; timing_phases[1]=2U; timing_phases[2]=3U; timing_phases[3]=4U;
  timing_phase_count=4U;
  timing_cycles[0]=100U; timing_cycles[1]=110U; timing_cycles[2]=410U; timing_cycles[3]=440U;
  timing_cycle_count=4U; generate(1U,0U);
  CHECK(digital_timing.irq_entry_phase_us==1U && digital_timing.gpio_phase_us==3U);
  CHECK(digital_timing.checks_phase_us==4U && digital_sync.last_phase_us==4U);
  CHECK(digital_timing.irq_entry_phase_max_us==1U && digital_timing.gpio_phase_max_us==3U);
  CHECK(digital_timing.checks_phase_max_us==4U && digital_irq_max_cycles==300U);
  CHECK(digital_timing.tim2_handler_max_cycles==340U && digital_timing.fault.valid==0U);
  reset_model();
  timing_phases[0]=198U; timing_phases[1]=199U; timing_phases[2]=0U; timing_phases[3]=1U;
  timing_phase_count=4U; generate(1U,0U);
  CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_CNT_WRAP && digital_sync.sample_count==0U);
  CHECK(digital_timing.fault.valid==1U && digital_timing.fault.source==1U);
  CHECK(digital_timing.fault.gpio_phase_us==0U && digital_timing.fault.record_ndtr==200U);
  CHECK(digital_timing.fault.record_trigger==1U && digital_timing.fault.sample_interval_us==200U);
  CHECK(digital_timing.fault.tim2_dier==0U && digital_timing.fault.tim2_arr==timer2.ARR);
  CHECK(timer2.CR1==1U && adc_running==1U);
  timer5.CNT=99U; DigitalSync_CaptureFault(3U,99U,99U);
  CHECK(digital_timing.fault.source==1U && digital_timing.fault.record_trigger==1U);
  reset_model(); digital_sync.timing=*DigitalSync_GetTimingConfig(1U);
  timer2.ARR=99U;
  timing_phases[0]=98U; timing_phases[1]=99U; timing_phases[2]=0U; timing_phases[3]=1U;
  timing_phase_count=4U; generate(1U,0U);
  CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_CNT_WRAP && digital_sync.sample_count==0U);
  CHECK(digital_timing.fault.sample_interval_us==100U && digital_timing.fault.tim2_arr==99U);
  reset_model();
  timing_phases[0]=1U; timing_phases[1]=2U; timing_phases[2]=3U; timing_phases[3]=21U;
  timing_phase_count=4U; generate(1U,0U);
  CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_PHASE && digital_sync.sample_count==0U);
  CHECK(digital_timing.fault.checks_phase_us==21U && digital_timing.fault.valid==1U);
  /* Unsigned DWT wrap and strict >20 us body budget, including boundary. */
  reset_model();
  timing_cycles[0]=UINT32_MAX-20U; timing_cycles[1]=UINT32_MAX-10U;
  timing_cycles[2]=9U; timing_cycles[3]=19U; timing_cycle_count=4U; generate(1U,0U);
  CHECK(digital_irq_max_cycles==20U && digital_timing.tim2_handler_max_cycles==40U);
  CHECK(digital_sync.error==0U);
  for(i=0U;i<2U;i++)
  {
    reset_model(); timing_cycles[0]=0U; timing_cycles[1]=10U;
    timing_cycles[2]=4330U+i; timing_cycles[3]=4400U; timing_cycle_count=4U; generate(1U,0U);
    CHECK(digital_irq_max_cycles==4320U+i);
    CHECK((digital_sync.error!=0U)==(i!=0U));
    if(i!=0U) { CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_BODY_BUDGET); }
  }
  reset_model(); generate(50U,0U);
  timing_cycles[0]=UINT32_MAX-15U; timing_cycles[1]=32U; timing_cycle_count=2U;
  HAL_ADC_ConvHalfCpltCallback(&hadc3);
  CHECK(digital_timing.dma_half_callback_max_cycles==48U && adc_half_callback_count==1U);
  timing_cycle_count=0U; generate(50U,0U);
  timing_cycles[0]=100U; timing_cycles[1]=260U; timing_cycle_pos=0U; timing_cycle_count=2U;
  HAL_ADC_ConvCpltCallback(&hadc3);
  CHECK(digital_timing.dma_full_callback_max_cycles==160U && adc_full_callback_count==1U);
  CHECK(full_group_publish_count==2U && digital_sync.error==0U);
  printf("PASS full sync: %u checks; 8 digital states, independent edges, "
         "first-point/50-point/half ordering, coherent frames, concurrent "
         "read/reuse rejection, phase/fault/wrap guards, timing profiles/diagnostics.\n",checks);
  return 0;
}
