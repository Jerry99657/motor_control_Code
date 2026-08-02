#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#include <stdint.h>

void lv_port_disp_init(void);
void lv_port_disp_process(void);
uint8_t lv_port_disp_wait_idle(uint32_t timeout_ms);

#endif /* LV_PORT_DISP_H */
