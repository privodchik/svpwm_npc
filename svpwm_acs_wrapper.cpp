
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include "CPP/CSVPWM_NPC.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
CSVPWM_NPC pwm;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void svpwm_acs_Outputs_wrapper(const real_T *mag,
			const real_T *th,
			const real_T *selCap,
			real_T *ccr,
			real_T *sctr,
			real_T *subsctr,
			const real_T *_Ts, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
const float* _pccr = pwm.ccr_est( std::pair<float, float>(mag[0], th[0]), 
        static_cast<CSVPWM_NPC::eSupplyCapacitor_t>(selCap[0]) );

ccr[0] = _pccr[0];
ccr[1] = _pccr[1];
ccr[2] = _pccr[2];
ccr[3] = _pccr[3];
ccr[4] = _pccr[4];
ccr[5] = _pccr[5];

sctr[0] = pwm.sector_get();
subsctr[0] = pwm.subsector_get();
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


