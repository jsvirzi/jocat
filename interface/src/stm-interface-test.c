#include <stdio.h>
#include <string.h>

#include "stm-interface.h"

int main(int argc, char **argv) {
    int port = 0;

    for (int i = 1; i < argc; ++i) { if ((strcmp(argv[i], "-p") == 0) || (strcmp(argv[i], "--port") == 0)) { port = atoi(argv[++i]); } }

    void *stm_stack = NULL;
    initialize_stm_interface(stm_stack, port);
    while (1) {
        vehicle_data_t vehicle_data;
        int status = consume_vehicle_data(stm_stack, &vehicle_data);
        if (status > 0) {
            vehicle_data_t *vdm = &vehicle_data;
            // printf("$VEHICLE,%8.8x,%8.8x,%4.4x,%8.8x,%4.4x,%4.4x,%u,%u*\n", 0, vdm->gtime, vdm->ltime, vdm->ticks, vdm->speed, vdm->timer, vdm->n_can_msgs, vdm->index);
            printf("$VEHICLE GTIME=%8.8x LTIME=%4.4x TICKS=%8.8x SPEED=%4.4x TIMER=%4.4x CAN_MSGS=%u INDEX=%u*\n",
                vdm->gtime, vdm->ltime, vdm->ticks, vdm->speed, vdm->timer, vdm->n_can_msgs, vdm->index);
        }
    }
    close_stm_interface(stm_stack);

    return 0;
}
