#ifndef NODE_SWO_TRACE_H_
#define NODE_SWO_TRACE_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void NodeSwo_Init(uint32_t core_clock_hz, uint32_t swo_clock_hz);
uint8_t NodeSwo_TryWrite(uint8_t value);
size_t NodeSwo_Write(const uint8_t *data, size_t size);
void NodeSwo_Process(void);
void NodeSwo_Logf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* NODE_SWO_TRACE_H_ */
