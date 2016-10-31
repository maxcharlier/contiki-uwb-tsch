#include "contiki.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "dev/dw1000/dw1000-util.h"


PROCESS(theoretical_transmission_time_test_process, "Theoretical transmission time test");

AUTOSTART_PROCESSES(&theoretical_transmission_time_test_process);

PROCESS_THREAD(theoretical_transmission_time_test_process, ev, data)
{
  
  PROCESS_BEGIN();
  rtimer_clock_t t0;

  printf("Theoretical transmission time test:\n");
  printf("Data rate of 110 kbps\n");
  printf("Data length 5 bytes %d, excepted 1975.39\n", (int) theorical_transmission_approx(1024, 110, 16, 5));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(1024, 110, 16, 5);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));

  printf("Data length 127 bytes %d, excepted 11165.13\n", (int) theorical_transmission_approx(1024, 110, 16, 127));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(1024, 110, 16, 127);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));

  printf("Data length 265 bytes %d, excepted 21405.13\n", (int) theorical_transmission_approx(1024, 110, 16, 265));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(1024, 110, 16, 265);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));

  printf("Data rate of 850 kbps\n");
  printf("Data length 5 bytes %d, excepted 374.10\n", (int) theorical_transmission_approx(256, 850, 16, 5));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(256, 850, 16, 5);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));

  printf("Data length 127 bytes %d, excepted 1522.82\n", (int) theorical_transmission_approx(256, 850, 16, 127));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(256, 850, 16, 127);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));

  printf("Data length 265 bytes %d, excepted 2802.82\n", (int) theorical_transmission_approx(256, 850, 16, 265));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(256, 850, 16, 265);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));

  printf("Data rate of 6800 kbps\n");
  printf("Data length 5 bytes %d, excepted 167.95\n", (int) theorical_transmission_approx(128, 6800, 16, 5));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(128, 6800, 16, 5);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));
  printf("Data length 127 bytes %d, excepted 311.54\n", (int) theorical_transmission_approx(128, 6800, 16, 127));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(128, 6800, 16, 127);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));
  printf("Data length 265 bytes %d, excepted 471.55\n", (int) theorical_transmission_approx(128, 6800, 16, 265));
  t0 = RTIMER_NOW();
  theorical_transmission_approx(128, 6800, 16, 265);
  printf("Duration of the computation (µs): %d\n", clock_ticks_to_microsecond(RTIMER_NOW() - t0));
  
  PROCESS_END();
}
