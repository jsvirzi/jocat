#ifndef STM_INTERFACE_H
#define STM_INTERFACE_H

#include "vdm.h"

int initialize_stm_interface(void *stack, int udp_port);
int close_stm_interface(void *stack);
int consume_vehicle_data(void *stack, vehicle_data_t *vdm);

#endif

