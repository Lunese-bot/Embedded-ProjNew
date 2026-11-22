/* Host stub for the IF-S4-3 speed hook.  task_motion.c declares
   s4_set_speed_mm_s weak and skips the call when it is absent.  Linux leaves
   an undefined weak symbol NULL, but the macOS linker refuses it, so the host
   test links this empty one.  Kept in its own file so the compiler still sees
   the weak declaration in task_motion.c. */
#include <stdint.h>

void s4_set_speed_mm_s(int32_t speed_mm_s);

void s4_set_speed_mm_s(int32_t speed_mm_s)
{
    (void)speed_mm_s;
}
