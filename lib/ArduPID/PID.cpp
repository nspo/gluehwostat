/*-----------------------------------------------------------------------------------//
  Arduino Standard PID Library
  by Andrea Vivani <andrea.vivani@gmail.com> 
  This Library is licensed under GPLv3 License
//-----------------------------------------------------------------------------------*/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID.h>
 
/*--------------------------------Calculate Output-------------------------------//
This function should be called inside an If statement that checks if the time 
elapsed since the last step is greater or equal than the step time
 //---------------------------------------------------------------------------------*/
void PID::Compute(double e)
{
	double Du_p=_kp*e;
	_Du_i+=_ki*(e+_e_old);
	_Du_d=_kf*_Du_d+_kd*(e-_e_old);
    _e_old=e;
    *_Output=constrain2((Du_p+_Du_i+_Du_d),_outMin,_outMax);
}

/*-------------------------------------Reset---------------------------------------//
This function can be called everywhere in order to re-initialize the PID
 //---------------------------------------------------------------------------------*/
void PID::Reset()
{
PID_def:Reset_def();
}