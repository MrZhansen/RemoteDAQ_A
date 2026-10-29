#ifndef REMOTEDAQ_LWIP_CC_H
#define REMOTEDAQ_LWIP_CC_H
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>
#define BYTE_ORDER LITTLE_ENDIAN
#define LWIP_PLATFORM_DIAG(x) do { } while (0)
void DAQ_NetAssert(const char *message);
#define LWIP_PLATFORM_ASSERT(x) DAQ_NetAssert(x)
#endif
