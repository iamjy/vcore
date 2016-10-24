
#ifndef _LGE_MPNG_H_
#define _LGE_MPNG_H_


extern void (*_odin_mpng_init(unsigned int reg_base,
							struct device *_dev))(void);
extern void odin_mpng_runtime_resume(void);
extern void odin_mpng_runtime_suspend(void);
extern void odin_mpng_reserve_running_weight(unsigned int width,
										unsigned int height,
										unsigned int frame_rate_residual,
										unsigned int frame_rate_divider);
extern void odin_mpng_unreserve_running_weight(unsigned int width,
										unsigned int height,
										unsigned int frame_rate_residual,
										unsigned int frame_rate_divider);

extern int odin_mpng_ch_lock(void *ch, void(*isr_func)(void *, unsigned long));
extern int odin_mpng_ch_unlock(void *ch);
extern int odin_mpng_ch_is_locked(void *ch);
extern void odin_mpng_update_run_time(int run);

void  odin_mpng_clock_on(void);
void  odin_mpng_clock_off(void);

#endif
