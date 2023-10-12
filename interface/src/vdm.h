#ifndef VDM_H
#define VDM_H

#include <stdint.h>

typedef struct {
    uint32_t gtime; /* gnss time */
    uint32_t ticks; /* total ticks thus far */
    uint16_t ltime; /* time from tim3 */
    uint16_t speed; /* wheel-based vehicle speed */
    uint16_t index; /* serial number */
    uint16_t timer; /* in case of wt generation, this is period between ticks */
    uint16_t stats; /* gnss convergence, sbsa stats, can stat/analog stat */
    uint16_t n_can_msgs; /* number of can messages received */
} vehicle_data_t;

#endif
