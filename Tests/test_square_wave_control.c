/* Actual TIM3 APIs and CMD07 executor; register/HAL models, no board. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do { checks++; if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
#define __root
#define HAL_TIM_STATE_RESET 0U
#define HAL_TIM_STATE_READY 1U
#define TIM_CHANNEL_1 0U
#define TIM_EVENTSOURCE_UPDATE 1U
#define TIM_FLAG_UPDATE 1U
#define RCC_DCKCFGR1_TIMPRE 1U
typedef struct { uint32_t PSC,ARR,CCR1,CNT,SR; } TIM_TypeDef;
static TIM_TypeDef timer3;
#define TIM3 (&timer3)
static struct { uint32_t DCKCFGR1; } rcc;
#define RCC (&rcc)
typedef struct { TIM_TypeDef *Instance; struct {uint32_t Prescaler,Period;} Init; uint32_t State; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim3;
static uint32_t mock_hclk=216000000U,mock_pclk=54000000U;
static uint32_t start_calls,stop_calls,event_calls,fail_event,fail_start;
static HAL_StatusTypeDef stop_status=HAL_OK;
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return mock_pclk; }
static uint32_t HAL_RCC_GetHCLKFreq(void) { return mock_hclk; }
#define __HAL_TIM_SET_PRESCALER(h,v) ((h)->Instance->PSC=(v))
#define __HAL_TIM_SET_AUTORELOAD(h,v) do { (h)->Init.Period=(v); (h)->Instance->ARR=(v); } while(0)
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->Instance->CCR1=(v))
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->Instance->CNT=(v))
#define __HAL_TIM_CLEAR_FLAG(h,v) ((h)->Instance->SR&=~(v))
static HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *h,uint32_t e)
{
  CHECK(h->Instance==TIM3 && e==TIM_EVENTSOURCE_UPDATE); event_calls++;
  if(fail_event) { fail_event--; return HAL_ERROR; } return HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t c)
{
  CHECK(h->Instance==TIM3 && c==TIM_CHANNEL_1); start_calls++;
  if(fail_start) { fail_start--; return HAL_BUSY; } return HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h,uint32_t c)
{ CHECK(h->Instance==TIM3 && c==TIM_CHANNEL_1); stop_calls++; return stop_status; }
#include "square_wave_api_under_test.inc"
static void setup(void)
{
  memset(&timer3,0,sizeof(timer3)); memset(&htim3,0,sizeof(htim3));
  htim3.Instance=TIM3; htim3.State=HAL_TIM_STATE_READY;
  mock_hclk=216000000U; mock_pclk=54000000U; rcc.DCKCFGR1=0U;
  start_calls=0U; stop_calls=0U; event_calls=0U; fail_event=0U; fail_start=0U;
  stop_status=HAL_OK; square_wave_running=0U; square_wave_initialized=0U;
  square_wave_error=0U; square_wave_frequency_hz=0U; square_wave_actual_hz=0U;
  square_wave_set_ok_count=0U; square_wave_set_error_count=0U;
  CHECK(SquareWave_Init(SQUARE_WAVE_DEFAULT_HZ)==HAL_OK);
  CHECK(SquareWave_Start()==HAL_OK);
  CHECK(square_wave_actual_hz==1000U && timer3.PSC==107U && timer3.ARR==999U);
}
int main(void)
{
  uint32_t i,calls;
  const uint32_t hz[]={100U,1000U,5000U,10000U};
  const uint32_t invalid[]={0U,99U,10001U,65535U,UINT32_MAX};
  TIM_TypeDef saved;
  setup();
  for(i=0U;i<4U;i++)
  {
    CHECK(SquareWave_ChangeFrequency(hz[i])==HAL_OK);
    CHECK(timer3.PSC==107U && timer3.ARR==1000000U/hz[i]-1U);
    CHECK(2U*timer3.CCR1==timer3.ARR+1U && square_wave_running==1U);
    CHECK(square_wave_actual_hz==hz[i] && square_wave_frequency_hz==hz[i]);
    CHECK(square_wave_last_set_status==HAL_OK && square_wave_last_request_hz==hz[i]);
    calls=stop_calls; CHECK(SquareWave_ChangeFrequency(hz[i])==HAL_OK);
    CHECK(stop_calls==calls); /* Identical request does not reset the waveform. */
  }
  saved=timer3; calls=stop_calls;
  for(i=0U;i<5U;i++)
  {
    CHECK(SquareWave_ChangeFrequency(invalid[i])==HAL_ERROR);
    CHECK(memcmp(&saved,&timer3,sizeof(saved))==0 && stop_calls==calls);
    CHECK(square_wave_actual_hz==10000U && square_wave_running==1U);
  }
  setup(); stop_status=HAL_BUSY;
  CHECK(SquareWave_ChangeFrequency(5000U)==HAL_BUSY);
  CHECK(square_wave_running==1U && square_wave_actual_hz==1000U && event_calls==1U);
  CHECK(square_wave_restore_status==UINT32_MAX && square_wave_set_error_count==1U);
  setup(); fail_event=1U;
  CHECK(SquareWave_ChangeFrequency(5000U)==HAL_ERROR);
  CHECK(square_wave_restore_status==HAL_OK && square_wave_running==1U);
  CHECK(square_wave_frequency_hz==1000U && timer3.ARR==999U);
  CHECK(square_wave_last_set_status==HAL_ERROR && square_wave_set_ok_count==0U);
  setup(); fail_start=1U;
  CHECK(SquareWave_ChangeFrequency(5000U)==HAL_BUSY);
  CHECK(square_wave_restore_status==HAL_OK && square_wave_actual_hz==1000U);
  CHECK(square_wave_last_set_status==HAL_BUSY && square_wave_running==1U);
  setup(); fail_event=2U;
  CHECK(SquareWave_ChangeFrequency(5000U)==HAL_ERROR);
  CHECK(square_wave_restore_status==HAL_ERROR && square_wave_running==0U);
  CHECK(event_calls==3U && square_wave_set_error_count==1U); /* No retry loop. */
  CHECK(SquareWave_ChangeFrequency(1000U)==HAL_OK && square_wave_running==1U);
  printf("PASS actual TIM3 APIs/CMD07 executor: %u checks; four frequencies, range, duplicate, HAL failure and bounded restore (host only)\n",checks);
  return 0;
}
