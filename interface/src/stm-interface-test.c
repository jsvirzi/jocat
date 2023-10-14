#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm-interface.h"

int main(int argc, char **argv) {

    int port = 0; /* default value of udp port will be used, if port specifies as 0 */

    for (int i = 1; i < argc; ++i) { if ((strcmp(argv[i], "-p") == 0) || (strcmp(argv[i], "--port") == 0)) { port = atoi(argv[++i]); } }

    void *stm_stack = NULL; /* use internal stack */
    initialize_stm_interface(stm_stack, port);
    static uint16_t expected_index = 0;
    while (1) {
        vehicle_data_t vdm[1];
        int status = consume_vehicle_data(stm_stack, vdm);
        if (status > 0) { /* example usage of data */
            if (expected_index != vdm->index) {
                printf("*************************************************************************************************** packet out of order\n");
            }
            printf("$VEHICLE GTIME=%8.8x LTIME=%4.4x TICKS=%8.8x SPEED=%4.4x TIMER=%4.4x CAN_MSGS=%u INDEX=%u*\n",
                vdm->gtime, vdm->ltime, vdm->ticks, vdm->speed, vdm->timer, vdm->n_can_msgs, vdm->index);
            expected_index = vdm->index + 1;
        }
    }
    close_stm_interface(stm_stack);

    return 0;
}
