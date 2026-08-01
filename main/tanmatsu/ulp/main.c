#include <stdbool.h>
#include <stdint.h>
#include "hardware.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "ulp_lp_core_utils.h"

int main(void) {
    while (1) {
        ulp_lp_core_wait_for_intr();
    }
    return 0;
}
