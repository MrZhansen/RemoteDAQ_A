#ifndef DAQ_NETWORK_H
#define DAQ_NETWORK_H
#include <stdint.h>
#include "daq_protocol.h"

/* Identity qualification only, not evidence of physical RMII validation. */
typedef enum {
  DAQ_NET_ID_UNREAD = 0,
  DAQ_NET_ID_ES0290_LISTED = 1, /* DEV 0x449, REV 0x1001 (Z/1). */
  DAQ_NET_ID_WRONG_DEVICE = 2,
  DAQ_NET_ID_REV_A_UNQUALIFIED = 3, /* RM0385 names A; ES0290 Rev7 omits it. */
  DAQ_NET_ID_UNKNOWN_REVISION = 4
} DAQ_NetIdentityStatus;

typedef struct {
  uint32_t initialized, enabled, link_up, peer_ready, fault;
  uint32_t init_count, init_error_count, last_hal_status, hal_error;
  int32_t phy_state;
  uint32_t phy_address, phy_id, revision, cache_ccr;
  uint32_t device_idcode, device_id, identity_status;
  uint32_t descriptor_mpu_base, descriptor_mpu_attributes;
  uint32_t rx_packets, rx_dropped, rx_wrong_peer, rx_bad_length;
  uint32_t rx_control_count, rx_crc_error, rx_format_error, rx_queue_overflow;
  uint32_t tx_submitted, tx_completed, tx_error, tx_busy, tx_dma_error, tx_stall_count;
  uint32_t data_submitted, status_submitted, last_error;
  uint32_t poll_max_cycles, poll_count, assertion_count;
} DAQ_NetDiagnostics;
extern volatile DAQ_NetDiagnostics daq_net;

/* Main context only. SetEnabled(0) gates application traffic; it does not
 * reset acquisition or claim PHY link down. Init failure is latched, no loop. */
/* Result: 1=complete, 2=disable waiting for TX completion, 0=failure.
 * Caller must stop submissions while disable is pending. No spin/wait here. */
uint32_t DAQ_NetSetEnabled(uint32_t enabled);
uint32_t DAQ_NetPoll(void);
uint32_t DAQ_NetReady(void);
uint32_t DAQ_NetTakeControl(DAQ_ControlCommand *command);
/* Complete 260/12-byte datagram only; 1 means submitted to local ETH DMA,
 * not DMA completion, peer acknowledgement or B-board acceptance. */
uint32_t DAQ_NetSendFrame(const uint8_t *frame, uint16_t length);

#endif
