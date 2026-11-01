/* Logical accounting tests using scripted registers/cycles. No MCU, trace
 * probe or oscilloscope is used; no real timing limit is certified here. */
#define main timing_legacy_main
#include "test_full_sync.c"
#undef main

static volatile DigitalTimingStat *hook_stat;
static uint32_t hook_barrier;
void DigitalTiming_TestHook(void)
{
  if(hook_barrier!=0U && --hook_barrier==0U)
  { DigitalTiming_Record(hook_stat,77U,100U,0U,0x1234U); }
}

int main(void)
{
  DigitalTimingStat stat={0}, out, saved;
  uint32_t i, start=0xFFFFFF00U;
  volatile DigitalTimingGroup *g;
  memset(&out,0xA5,sizeof(out)); saved=out;
  CHECK(DigitalTiming_Read(&stat,&out)==0U && memcmp(&out,&saved,sizeof(out))==0);
  CHECK(DigitalTiming_Read(NULL,&out)==0U && DigitalTiming_Read(&stat,NULL)==0U);
  DigitalTiming_Record(NULL,0U,0U,0U,0U);
  DigitalTiming_RecordSpan(&stat,100U,120U,20U,0U,9U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U);
  CHECK(out.last==20U && out.max==20U && out.count==1U && out.limit==20U);
  CHECK(out.flags==0U && out.over_count==0U && out.context==9U);
  DigitalTiming_RecordSpan(&stat,120U,141U,20U,DIGITAL_TIMING_ERROR,11U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U);
  CHECK(out.last==21U && out.max==21U && out.count==2U && out.over_count==1U);
  CHECK(out.flags==(DIGITAL_TIMING_ERROR|DIGITAL_TIMING_OVER_BUDGET));
  CHECK(out.anomaly_count==1U && out.context==11U);
  DigitalTiming_RecordSpan(&stat,0xFFFFFFF0U,10U,100U,0U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==26U);
  CHECK(out.flags==DIGITAL_TIMING_CYCLE_WRAP && out.anomaly_count==1U);
  CHECK((out.sticky_flags&DIGITAL_TIMING_ERROR)!=0U);
  stat.seq=UINT32_MAX-1U; stat.count=UINT32_MAX;
  DigitalTiming_Record(&stat,1U,100U,0U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.seq==0U && out.count==0U);
  CHECK(out.valid==1U && out.count_wraps==1U && out.max==26U);
  for(i=0U;i<1000U;i++)
  {
    uint32_t duration=(i*997U)%50000U;
    DigitalTiming_RecordSpan(&stat,start,start+duration,50000U,0U,i);
    CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==duration);
    CHECK(out.context==i && out.max>=out.last && (out.seq&1U)==0U);
    start+=0x01FFFFFFU;
  }
  saved=out; stat.seq|=1U;
  CHECK(DigitalTiming_Read(&stat,&out)==0U && memcmp(&out,&saved,sizeof(out))==0);
  stat.seq++;
  for(i=1U;i<=2U;i++)
  {
    hook_stat=&stat; hook_barrier=i;
    CHECK(DigitalTiming_Read(&stat,&out)==0U && memcmp(&out,&saved,sizeof(out))==0);
    CHECK(hook_barrier==0U);
    CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==77U && out.context==0x1234U);
    saved=out;
  }

  memset(&stat,0,sizeof(stat));
  DigitalTiming_Response(&stat,1U,2U,3U,4U,200U,20U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==4U && out.limit==21U);
  CHECK(out.context==(2U|(3U<<16U)) && out.flags==0U);
  DigitalTiming_Response(&stat,19U,20U,20U,20U,100U,20U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==21U && out.flags==0U);
  DigitalTiming_Response(&stat,20U,20U,21U,21U,100U,20U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==22U);
  CHECK((out.flags&(DIGITAL_TIMING_LATE|DIGITAL_TIMING_OVER_BUDGET))!=0U);
  DigitalTiming_Response(&stat,199U,199U,0U,1U,200U,20U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==0U && out.max==22U);
  CHECK((out.flags&(DIGITAL_TIMING_INVALID|DIGITAL_TIMING_PHASE_WRAP))!=0U);
  DigitalTiming_Response(&stat,0U,0U,1U,1U,200U,20U,DIGITAL_SYNC_ERROR_ALIGN);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==0U);
  CHECK((out.flags&(DIGITAL_TIMING_INVALID|DIGITAL_TIMING_SYNC_FAULT))!=0U);
  DigitalTiming_Response(&stat,0U,0U,0U,0U,0U,20U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && (out.flags&DIGITAL_TIMING_INVALID)!=0U);
  /* Existing candidate threshold logic only: no ADC configuration is changed. */
  DigitalTiming_Response(&stat,1U,3U,4U,4U,200U,4U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && out.last==5U && out.flags==0U);
  DigitalTiming_Response(&stat,1U,3U,5U,5U,200U,4U,0U);
  CHECK(DigitalTiming_Read(&stat,&out)==1U && (out.flags&DIGITAL_TIMING_LATE)!=0U);
  CHECK(DigitalSync_GetTimingConfig(0U)->sample_interval_us==200U);
  CHECK(DigitalSync_GetTimingConfig(1U)->sample_interval_us==100U);
  CHECK(DigitalSync_GetTimingConfig(0U)->max_phase_us==20U);
  CHECK(DigitalSync_GetTimingConfigForADC(0U,144U)->max_phase_us==4U);
  CHECK(DigitalSync_GetTimingConfig(2U)->sample_interval_us==20U);
  CHECK(DigitalSync_GetTimingConfig(3U)->sample_interval_us==10U);
  CHECK(DigitalSync_GetTimingConfig(4U)->sample_interval_us==2U);
  CHECK(DigitalSync_GetTimingConfig(UINT32_MAX)==NULL);

  reset_model();
  timing_phases[0]=1U; timing_phases[1]=2U; timing_phases[2]=3U; timing_phases[3]=4U;
  timing_phase_count=4U;
  timing_cycles[0]=100U; timing_cycles[1]=110U; timing_cycles[2]=410U;
  timing_cycles[3]=440U; timing_cycles[4]=480U; timing_cycle_count=5U;
  generate(1U,0U); g=&digital_timing.gear[0];
  CHECK(DigitalTiming_Read(&g->tim2_c,&out)==1U && out.last==340U && out.count==1U);
  CHECK(DigitalTiming_Read(&g->tim2_accounting,&out)==1U && out.last==40U);
  CHECK(DigitalTiming_Read(&g->gpio_upper_us,&out)==1U && out.last==4U);
  CHECK(digital_timing.gear[1].tim2_c.valid==0U);
  timing_cycle_count=0U; timer2.SR=0U; TIM2_IRQHandler();
  CHECK(DigitalTiming_Read(&g->tim2_c,&out)==1U && (out.flags&DIGITAL_TIMING_EMPTY)!=0U);
  acquisition_active_gear=1U; digital_sync.timing=*DigitalSync_GetTimingConfig(1U);
  timer2.ARR=99U; timer2.CNT=1U; generate(1U,0U);
  CHECK(digital_timing.gear[1].tim2_c.count==1U && digital_sync.error==0U);
  CHECK(DigitalTiming_Group(&digital_timing,2U)==&digital_timing.offline);
  CHECK(DigitalTiming_Group(&digital_timing,UINT32_MAX)==&digital_timing.inactive);
  CHECK(DigitalSync_GetTimingConfig(2U)->adc_sample_cycles==144U);

  reset_model(); timer2.CNT=1U; generate(50U,0U);
  fake_dwt.CYCCNT=100U; timing_publish_cost=100U;
  fake_dma2.LISR=DMA_LISR_HTIF0; DMA2_Stream0_IRQHandler();
  g=&digital_timing.gear[0];
  CHECK(dma2_stream0_irq_count==1U && dma2_stream0_last_lisr==DMA_LISR_HTIF0);
  CHECK(DigitalTiming_Read(&g->dma_c,&out)==1U && out.last==140U && out.context==16U);
  CHECK(DigitalTiming_Read(&g->dma_half_c,&out)==1U && out.last==140U && out.count==1U);
  CHECK(DigitalTiming_Read(&g->publish_half,&out)==1U && out.last==100U && out.context==0U);
  CHECK(full_snapshot_valid==1U && full_group_publish_count==1U);
  CHECK(g->dma_full_c.valid==0U && digital_sync.error==0U);
  generate(50U,0U); timing_publish_cost=150U; fake_dwt.CYCCNT=300U;
  fake_dma2.LISR=DMA_LISR_TCIF0; DMA2_Stream0_IRQHandler();
  CHECK(DigitalTiming_Read(&g->dma_full_c,&out)==1U && out.last==190U && out.count==1U);
  CHECK(DigitalTiming_Read(&g->publish_full,&out)==1U && out.last==150U && out.context==100U);
  CHECK(g->dma_c.count==2U && full_group_publish_count==2U && digital_sync.error==0U);

  reset_model(); timer2.CNT=1U; generate(50U,0U);
  fake_dwt.CYCCNT=UINT32_MAX-50U; timing_publish_cost=100U;
  fake_dma2.LISR=DMA_LISR_HTIF0; DMA2_Stream0_IRQHandler();
  g=&digital_timing.gear[0];
  CHECK(DigitalTiming_Read(&g->dma_c,&out)==1U && out.last==140U);
  CHECK((out.flags&DIGITAL_TIMING_CYCLE_WRAP)!=0U);
  CHECK(DigitalTiming_Read(&g->publish_half,&out)==1U && out.last==100U);
  CHECK((out.flags&DIGITAL_TIMING_CYCLE_WRAP)!=0U);
  for(i=0U;i<3U;i++)
  {
    static const uint32_t error_flags[]={DMA_LISR_TEIF0,DMA_LISR_DMEIF0,DMA_LISR_FEIF0};
    reset_model(); fake_dma2.LISR=error_flags[i]; DMA2_Stream0_IRQHandler();
    g=&digital_timing.gear[0];
    CHECK(DigitalTiming_Read(&g->dma_c,&out)==1U && out.last==40U);
    CHECK((out.flags&(DIGITAL_TIMING_ERROR|DIGITAL_TIMING_NO_CALLBACK))==
          (DIGITAL_TIMING_ERROR|DIGITAL_TIMING_NO_CALLBACK));
    CHECK(g->publish_half.valid==0U && g->publish_full.valid==0U);
  }
  reset_model(); DMA2_Stream0_IRQHandler();
  CHECK(DigitalTiming_Read(&digital_timing.gear[0].dma_c,&out)==1U);
  CHECK(out.flags==DIGITAL_TIMING_NO_CALLBACK && out.count==1U);
  reset_model(); timer2.CNT=1U; generate(100U,0U);
  fake_dma2.LISR=DMA_LISR_HTIF0|DMA_LISR_TCIF0; DMA2_Stream0_IRQHandler();
  g=&digital_timing.gear[0];
  CHECK(DigitalTiming_Read(&g->dma_c,&out)==1U && (out.flags&DIGITAL_TIMING_COALESCED)!=0U);
  CHECK(g->dma_half_c.count==1U && g->dma_full_c.count==1U);
  CHECK(g->dma_half_c.last==g->dma_full_c.last && out.context==48U);
  reset_model(); timer2.CNT=1U; generate(50U,0U); timing_publish_cost=50000U;
  fake_dma2.LISR=DMA_LISR_HTIF0; DMA2_Stream0_IRQHandler();
  CHECK(DigitalTiming_Read(&digital_timing.gear[0].dma_c,&out)==1U);
  CHECK((out.flags&DIGITAL_TIMING_OVER_BUDGET)!=0U && out.over_count==1U);
  CHECK(adc_running==1U); /* Diagnostics observe; never change protection. */
  printf("PASS timing phase1: %u checks (scripted cycles/IRQs, not hardware timing)\n",checks);
  return 0;
}
