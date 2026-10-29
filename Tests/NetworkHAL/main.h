#ifndef NETWORK_HAL_TEST_MAIN_H
#define NETWORK_HAL_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2 } HAL_StatusTypeDef;
typedef struct { uint32_t CCR; } TestSCB;
typedef struct { uint32_t CYCCNT; } TestDWT;
typedef struct { uint32_t DMAIER,DMAOMR,DMASR; } TestETH;
extern TestSCB test_scb; extern TestDWT test_dwt; extern TestETH test_eth;
typedef struct { volatile uint32_t IDCODE; } TestDBGMCU;
extern TestDBGMCU test_dbgmcu;
#define DBGMCU (&test_dbgmcu)
#define DBGMCU_IDCODE_DEV_ID_Pos 0U
#define DBGMCU_IDCODE_DEV_ID_Msk 0x00000FFFUL
#define DBGMCU_IDCODE_REV_ID_Pos 16U
#define DBGMCU_IDCODE_REV_ID_Msk 0xFFFF0000UL
#define SCB (&test_scb)
#define DWT (&test_dwt)
#define ETH (&test_eth)
#define SCB_CCR_DC_Msk 0x10000U
#define ETH_RX_DESC_CNT 4U
#define ETH_TX_DESC_CNT 4U
#define ETH_DMATXDESC_ES 1U
#define ETH_DMASR_FBES 2U
#define ETH_DMASR_NIS 4U
#define ETH_DMASR_AIS 8U
#define ETH_DMASR_RS 16U
#define ETH_DMASR_TS 32U
#define ETH_DMASR_RBUS 64U
#define ETH_DMASR_TBUS 128U
#define ETH_DMASR_TPS 0x700000U
#define ETH_DMASR_RPS 0xE0000U
#define ETH_DMAOMR_ST 0x2000U
#define ETH_DMAOMR_SR 2U
#define HAL_ETH_STATE_READY 1U
#define HAL_ETH_STATE_STARTED 2U
#define HAL_ETH_RMII_MODE 1U
#define ETH_TX_PACKETS_FEATURES_CRCPAD 2U
#define ETH_TX_PACKETS_FEATURES_CSUM 1U
#define ETH_CHECKSUM_DISABLE 0U
#define ETH_DMATXDESC_CIC 0x00C00000U
#define ETH_DMATXDESC_OWN 0x80000000U
#define ETH_CRC_PAD_INSERT 1U
#define ETH_FULLDUPLEX_MODE 1U
#define ETH_HALFDUPLEX_MODE 0U
#define ETH_SPEED_100M 100U
#define ETH_SPEED_10M 10U
#define DISABLE 0U
#define GPIOA 1U
#define GPIOC 2U
#define GPIOG 3U
#define GPIO_PIN_1 (1U<<1)
#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define GPIO_PIN_7 (1U<<7)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_14 (1U<<14)
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF11_ETH 11U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ETH_CLK_ENABLE() ((void)0)
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)
typedef struct { uint32_t RBAR,RASR; } TestMPU;
extern TestMPU test_mpu;
#define MPU (&test_mpu)
#define MPU_REGION_ENABLE 1U
#define MPU_REGION_NUMBER1 1U
#define MPU_REGION_SIZE_512B 8U
#define MPU_REGION_FULL_ACCESS 3U
#define MPU_INSTRUCTION_ACCESS_DISABLE 1U
#define MPU_TEX_LEVEL0 0U
#define MPU_ACCESS_SHAREABLE 1U
#define MPU_ACCESS_NOT_CACHEABLE 0U
#define MPU_ACCESS_BUFFERABLE 1U
typedef struct { uint32_t Enable,Number,BaseAddress,Size,AccessPermission,DisableExec,TypeExtField,IsShareable,IsCacheable,IsBufferable,SubRegionDisable; } MPU_Region_InitTypeDef;
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *region);
#define __no_init
#define __root
/* Suppress only IAR placement pragmas in this host fixture, via compiler flag. */
typedef struct { uint32_t DESC0,DESC1,DESC2,DESC3,DESC4,DESC5,DESC6,DESC7,BackupAddr0,BackupAddr1; } ETH_DMADescTypeDef;
typedef struct ETH_BufferTypeDef { uint8_t *buffer; uint32_t len; struct ETH_BufferTypeDef *next; } ETH_BufferTypeDef;
typedef struct { uint32_t Attributes,Length,CRCPadCtrl,ChecksumCtrl; ETH_BufferTypeDef *TxBuffer; void *pData; } ETH_TxPacketConfig;
typedef struct { uint8_t *MACAddr; uint32_t MediaInterface; ETH_DMADescTypeDef *TxDesc,*RxDesc; uint32_t RxBuffLen; } TestETHInit;
typedef struct { TestETH *Instance; TestETHInit Init; uint32_t ErrorCode,gState; struct {uint32_t releaseIndex;} TxDescList; } ETH_HandleTypeDef;
typedef struct { uint32_t DuplexMode,Speed,ChecksumOffload; } ETH_MACConfigTypeDef;
typedef struct { uint32_t Mode,Pull,Speed,Alternate,Pin; } GPIO_InitTypeDef;
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(uint32_t port,GPIO_InitTypeDef *gpio);
void HAL_ETH_MspInit(ETH_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *h);
void HAL_ETH_SetMDIOClockRange(ETH_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *h,uint32_t address,uint32_t reg,uint32_t *value);
HAL_StatusTypeDef HAL_ETH_WritePHYRegister(const ETH_HandleTypeDef *h,uint32_t address,uint32_t reg,uint32_t value);
HAL_StatusTypeDef HAL_ETH_GetMACConfig(ETH_HandleTypeDef *h,ETH_MACConfigTypeDef *config);
HAL_StatusTypeDef HAL_ETH_SetMACConfig(ETH_HandleTypeDef *h,ETH_MACConfigTypeDef *config);
HAL_StatusTypeDef HAL_ETH_Start(ETH_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ETH_ReleaseTxPacket(ETH_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ETH_Transmit_IT(ETH_HandleTypeDef *h,ETH_TxPacketConfig *config);
HAL_StatusTypeDef HAL_ETH_ReadData(ETH_HandleTypeDef *h,void **packet);
HAL_StatusTypeDef HAL_ETH_GetRxDataErrorCode(ETH_HandleTypeDef *h,uint32_t *error);
#endif
