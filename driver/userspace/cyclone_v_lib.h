#ifndef __CYCLONE_V_LIB_H__
#define __CYCLONE_V_LIB_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __KERNEL__ /* The initial part of the file is driver-internal stuff */
 #include <stdint.h>
#endif
#include "cyclone_v.h"

int cyclone_V_regread(int fd, uint32_t offset, uint32_t *value);
int cyclone_V_regwrite(int fd, uint32_t offset, uint32_t value);
int cyclone_V_blockread(int fd, uint8_t npage, uint8_t pages, uint32_t *data);
int cyclone_V_blockwrite(int fd, uint8_t npage, uint8_t pages, uint32_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __CYCLONE_V_LIB_H__ */
