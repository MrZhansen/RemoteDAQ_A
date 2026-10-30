#ifndef REMOTEDAQ_LWIP_CC_H
#define REMOTEDAQ_LWIP_CC_H
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>
#define BYTE_ORDER LITTLE_ENDIAN
#define LWIP_PLATFORM_DIAG(x) do { } while (0)
/* Official stm32-mw-lwip IAR port: bpstruct/epstruct bracket each wire
 * structure with pragma pack(1)/pack(). GCC uses lwip/arch.h defaults. */
#if defined(__ICCARM__)
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_USE_INCLUDES
#endif
void DAQ_NetAssert(const char *message);
#define LWIP_PLATFORM_ASSERT(x) DAQ_NetAssert(x)
#endif
