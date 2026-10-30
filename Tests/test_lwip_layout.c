/* Wire layouts must also pass the IAR compile-time checks in daq_network.c. */
#include "lwip/arch.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/ip4.h"
#include "lwip/prot/udp.h"
#include <stddef.h>
#include <stdio.h>

#ifdef PACK_STRUCT_USE_INCLUDES
#include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct packing_probe {
  PACK_STRUCT_FLD_8(u8_t byte);
  PACK_STRUCT_FIELD(u32_t word);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#include "arch/epstruct.h"
#endif

int main(void)
{
  if ((sizeof(struct packing_probe) != 5U) ||
      (sizeof(struct eth_hdr) != 14U) ||
      (offsetof(struct eth_hdr, type) != 12U) ||
      (sizeof(struct etharp_hdr) != 28U) ||
      (sizeof(struct ip_hdr) != 20U) ||
      (offsetof(struct ip_hdr, src) != 12U) ||
      (offsetof(struct ip_hdr, dest) != 16U) ||
      (sizeof(struct udp_hdr) != 8U))
  { fputs("FAIL LwIP wire layouts\n", stderr); return 1; }
  puts("PASS LwIP wire layouts: 8 checks (host compiler only)");
  return 0;
}
