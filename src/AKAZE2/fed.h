#ifndef __OPENCV_FEATURES_2D_FED_H__
#define __OPENCV_FEATURES_2D_FED_H__

//******************************************************************************
//******************************************************************************

// Includes
#include <vector>

//*************************************************************************************
//*************************************************************************************

// Declaration of functions
int fed_tau_by_process_timeV2(float T, int M, float tau_max,
                            bool reordering, std::vector<float>& tau);
int fed_tau_by_cycle_timeV2(float t, float tau_max,
                          bool reordering, std::vector<float> &tau) ;
int fed_tau_internalV2(int n, float scale, float tau_max,
                     bool reordering, std::vector<float> &tau);
bool fed_is_prime_internalV2(int number);

//*************************************************************************************
//*************************************************************************************

#endif // __OPENCV_FEATURES_2D_FED_H__
