#include "sw_clock.h"

/* Base: tick snapshot + real time at sync moment */
static int32_t base_tick = -1;   /* -1 = not synced */
static uint8_t base_hour;
static uint8_t base_min;
static uint8_t base_sec;

/* ==================== API ==================== */

void SWClock_Sync(uint8_t hour, uint8_t min, uint8_t sec) {
    base_tick = (int32_t)HAL_GetTick();
    base_hour = hour;
    base_min  = min;
    base_sec  = sec;
}

void SWClock_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec) {
    if (base_tick < 0) {
        *hour = *min = *sec = 0;
        return;
    }
    uint32_t elapsed = ((uint32_t)((int32_t)HAL_GetTick() - base_tick)) / 1000u;
    uint32_t total   = (uint32_t)base_hour * 3600u
                     + (uint32_t)base_min  * 60u
                     + (uint32_t)base_sec
                     + elapsed;
    total %= 86400u;
    *hour = (uint8_t)(total / 3600u);
    *min  = (uint8_t)((total % 3600u) / 60u);
    *sec  = (uint8_t)(total % 60u);
}

uint8_t SWClock_IsSynced(void) {
    return (base_tick >= 0) ? 1 : 0;
}
